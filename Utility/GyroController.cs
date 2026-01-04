using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Manages orientation control via gyroscopes.
    /// Provides a simple LookAt interface for callers - they specify what to look at,
    /// this class handles all the gyro math, PID control, and intelligent turn modes.
    /// </summary>
    public class GyroController
    {
        // === Dependencies ===
        private IMyShipController _reference;
        private PIDController3D _orientationPID;
        private Action<string> _echo;

        // === Gyro hardware ===
        private List<IMyGyro> _gyros = new List<IMyGyro>();

        // === Predictive braking state (per-axis tracking) ===
        private double _lastPitchError;
        private double _lastYawError;
        private double _lastRollError;
        private double _lastPitchVelocity;
        private double _lastYawVelocity;
        private double _lastRollVelocity;

        // === Control tuning ===
        private const double MAX_ANGULAR_VELOCITY = 4.0;  // rad/s - max rotation speed for dampened control
        private const double ALIGNMENT_DEADBAND = 0.001;  // ~0.3 degrees - very tight deadband
        private const double DAMPEN_THRESHOLD = 0.1;      // ~6 degrees - switch to fine control below this
        private const double PID_ANGLE_THRESHOLD = 0.05;  // ~3 degrees - angle threshold for PID
        private const double PID_VELOCITY_THRESHOLD = 0.25; // rad/s - velocity threshold for PID
        private const double PID_MAX_OUTPUT = 2.0;        // rad/s - cap PID output
        private const double DEFAULT_MAX_TILT = 0.35;     // ~20 degrees default max tilt

        // === Tilt limiting ===
        private double _maxTiltAngle = DEFAULT_MAX_TILT;  // radians
        private OrientationMode _orientationMode = OrientationMode.HorizonLock;

        // === PID state tracking ===
        private bool _pidActive = false;

        // === Current state ===
        private double _deltaTime = 0.016;  // Default to ~60fps

        /// <summary>
        /// The current total angular error in radians.
        /// </summary>
        public double TotalError { get; private set; }

        /// <summary>
        /// The current angular velocity magnitude in radians per second.
        /// </summary>
        public double AngularVelocity => Math.Sqrt(
            _lastPitchVelocity * _lastPitchVelocity +
            _lastYawVelocity * _lastYawVelocity +
            _lastRollVelocity * _lastRollVelocity);

        /// <summary>
        /// Whether the controller is currently aligned (within deadband).
        /// </summary>
        public bool IsAligned => TotalError < ALIGNMENT_DEADBAND;

        /// <summary>
        /// Creates a new GyroController.
        /// </summary>
        /// <param name="reference">The ship controller used as orientation reference</param>
        /// <param name="gyros">List of gyros to control</param>
        /// <param name="pidGains">PID gains for fine orientation control</param>
        /// <param name="echo">Optional echo callback for debug output</param>
        public GyroController(IMyShipController reference, List<IMyGyro> gyros, PIDGains pidGains, Action<string> echo = null)
        {
            _reference = reference;
            _gyros = gyros;
            _echo = echo;
            _orientationPID = new PIDController3D(pidGains);
        }

        /// <summary>
        /// Updates the reference controller (e.g., if player switches cockpits).
        /// </summary>
        public void SetReference(IMyShipController reference)
        {
            _reference = reference;
        }

        /// <summary>
        /// Sets the time delta for this update tick.
        /// Call this each tick before LookAt().
        /// </summary>
        public void SetDeltaTime(double deltaTime)
        {
            _deltaTime = deltaTime > 0.001 ? deltaTime : 0.016;
        }

        /// <summary>
        /// Sets the maximum tilt angle (pitch from level) allowed.
        /// </summary>
        /// <param name="degrees">Maximum tilt in degrees (default 20)</param>
        public void SetMaxTilt(double degrees)
        {
            _maxTiltAngle = degrees * Math.PI / 180.0;
        }

        /// <summary>
        /// Sets how the controller locks roll and tilt relative to gravity.
        /// </summary>
        public void SetOrientationMode(OrientationMode mode)
        {
            _orientationMode = mode;
        }

        /// <summary>
        /// Orients the grid to match a leader's compass heading while staying level to gravity.
        /// Projects the leader's forward onto the horizontal plane and orients toward it.
        /// Use this for formation flying where the drone should face the same direction as leader
        /// but remain level regardless of leader's pitch/roll.
        /// </summary>
        /// <param name="leaderForward">The leader's forward direction vector</param>
        /// <param name="leaderUp">The leader's up direction vector (used as fallback)</param>
        /// <returns>True if orientation was applied</returns>
        public bool MatchCompassHeading(Vector3D leaderForward, Vector3D leaderUp)
        {
            if (_reference == null || _gyros.Count == 0)
                return false;

            Vector3D gravity = _reference.GetNaturalGravity();

            // Fallback right vector in case leader is pointing straight up/down
            Vector3D leaderRight = Vector3D.Cross(leaderUp, leaderForward);

            Vector3D horizontalForward = VectorMath.ProjectOntoHorizontalPlane(leaderForward, gravity, leaderRight);

            if (horizontalForward.LengthSquared() > 0.001)
            {
                return OrientToward(horizontalForward, null, true);
            }

            // Degenerate case - maintain current orientation
            return false;
        }

        /// <summary>
        /// Orients the grid to be level with gravity (zero pitch, zero roll).
        /// Maintains current heading.
        /// </summary>
        /// <returns>True if orientation was applied</returns>
        public bool OrientLevel()
        {
            if (_reference == null || _gyros.Count == 0)
                return false;

            Vector3D gravity = _reference.GetNaturalGravity();
            if (gravity.LengthSquared() < 0.1)
                return false;

            Vector3D worldUp = -Vector3D.Normalize(gravity);
            MatrixD refMatrix = _reference.WorldMatrix;
            Vector3D currentForward = refMatrix.Forward;

            // Project current forward onto horizontal plane to maintain heading
            Vector3D horizontalForward = currentForward - worldUp * Vector3D.Dot(currentForward, worldUp);
            if (horizontalForward.LengthSquared() < 0.001)
            {
                horizontalForward = refMatrix.Backward;
            }
            horizontalForward = Vector3D.Normalize(horizontalForward);

            return OrientToward(horizontalForward, null, true);
        }

        /// <summary>
        /// Orients the grid to face a target position using level turning (yaw only).
        /// Locks pitch and roll to horizon during the turn to prevent rolling.
        /// This is now a wrapper around OrientToward with forceLevelTurn = true.
        /// </summary>
        /// <param name="worldTarget">The world position to turn toward</param>
        /// <returns>True if orientation was applied</returns>
        public bool LevelTurnToward(Vector3D worldTarget)
        {
            if (_reference == null || _gyros.Count == 0)
                return false;

            Vector3D myPosition = _reference.GetPosition();
            Vector3D toTarget = worldTarget - myPosition;

            if (toTarget.LengthSquared() < 1)
                return false;

            Vector3D desiredForward = Vector3D.Normalize(toTarget);
            return OrientToward(desiredForward, null, true);
        }

        /// <summary>
        /// Checks if the grid is approximately level (within tolerance).
        /// </summary>
        /// <param name="toleranceDegrees">Tolerance in degrees (default 5)</param>
        /// <returns>True if pitch and roll are within tolerance</returns>
        public bool IsLevel(double toleranceDegrees = 5.0)
        {
            if (_reference == null)
                return false;

            Vector3D gravity = _reference.GetNaturalGravity();
            if (gravity.LengthSquared() < 0.1)
                return true;  // No gravity = always "level"

            Vector3D worldUp = -Vector3D.Normalize(gravity);
            MatrixD refMatrix = _reference.WorldMatrix;

            // Check pitch: angle of forward from horizontal
            double pitch = Math.Abs(Math.Asin(MathHelper.Clamp(Vector3D.Dot(refMatrix.Forward, worldUp), -1, 1)));

            // Check roll: angle of up from worldUp (projected onto plane perpendicular to forward)
            Vector3D worldUpProjected = worldUp - refMatrix.Forward * Vector3D.Dot(worldUp, refMatrix.Forward);
            double roll = 0;
            if (worldUpProjected.LengthSquared() > 0.001)
            {
                worldUpProjected = Vector3D.Normalize(worldUpProjected);
                roll = Math.Acos(MathHelper.Clamp(Vector3D.Dot(refMatrix.Up, worldUpProjected), -1, 1));
            }

            double toleranceRad = toleranceDegrees * Math.PI / 180.0;
            return pitch < toleranceRad && roll < toleranceRad;
        }

        /// <summary>
        /// Checks if the grid is facing approximately toward a target (yaw only, ignoring pitch).
        /// </summary>
        /// <param name="worldTarget">Target position to check</param>
        /// <param name="toleranceDegrees">Tolerance in degrees (default 10)</param>
        /// <returns>True if facing within tolerance</returns>
        public bool IsFacingTarget(Vector3D worldTarget, double toleranceDegrees = 10.0)
        {
            if (_reference == null)
                return false;

            Vector3D gravity = _reference.GetNaturalGravity();
            Vector3D worldUp = gravity.LengthSquared() > 0.1
                ? -Vector3D.Normalize(gravity)
                : _reference.WorldMatrix.Up;

            Vector3D myPosition = _reference.GetPosition();
            Vector3D toTarget = worldTarget - myPosition;

            // Project both onto horizontal plane
            Vector3D horizontalToTarget = toTarget - worldUp * Vector3D.Dot(toTarget, worldUp);
            Vector3D horizontalForward = _reference.WorldMatrix.Forward - worldUp * Vector3D.Dot(_reference.WorldMatrix.Forward, worldUp);

            if (horizontalToTarget.LengthSquared() < 0.001 || horizontalForward.LengthSquared() < 0.001)
                return true;  // Target directly above/below or we're pointing straight up

            horizontalToTarget = Vector3D.Normalize(horizontalToTarget);
            horizontalForward = Vector3D.Normalize(horizontalForward);

            double angle = Math.Acos(MathHelper.Clamp(Vector3D.Dot(horizontalForward, horizontalToTarget), -1, 1));
            double toleranceRad = toleranceDegrees * Math.PI / 180.0;

            return angle < toleranceRad;
        }

        /// <summary>
        /// Orients the grid to face a world-space target position.
        /// Automatically uses level turning for large yaw errors to prevent rolling.
        /// Call this every tick to maintain orientation.
        /// </summary>
        /// <param name="worldTarget">The world position to look at</param>
        /// <returns>True if orientation was applied, false if unable (no gyro, invalid target)</returns>
        public bool LookAt(Vector3D worldTarget)
        {
            if (_reference == null || _gyros.Count == 0)
                return false;

            Vector3D myPosition = _reference.GetPosition();
            Vector3D toTarget = worldTarget - myPosition;

            if (toTarget.LengthSquared() < 1)  // Too close, avoid division by zero
                return false;

            Vector3D desiredForward = Vector3D.Normalize(toTarget);
            return OrientToward(desiredForward);
        }

        /// <summary>
        /// Orients the grid so its forward vector aligns with the specified world direction.
        /// Automatically detects large yaw errors and uses level turning to prevent rolling.
        /// </summary>
        /// <param name="worldDirection">The world-space direction to face (will be normalized)</param>
        /// <param name="customUp">Optional custom up direction (for docking). If null, uses gravity.</param>
        /// <param name="forceLevelTurn">Force level turn mode regardless of yaw error</param>
        /// <returns>True if orientation was applied</returns>
        public bool OrientToward(Vector3D worldDirection, Vector3D? customUp = null, bool forceLevelTurn = false)
        {
            if (_reference == null || _gyros.Count == 0)
                return false;

            if (worldDirection.LengthSquared() < 0.001)
                return false;

            Vector3D desiredForward = Vector3D.Normalize(worldDirection);
            MatrixD refMatrix = _reference.WorldMatrix;
            Vector3D currentForward = refMatrix.Forward;
            Vector3D currentUp = refMatrix.Up;

            // Get gravity/up reference
            Vector3D gravity = _reference.GetNaturalGravity();
            bool hasGravity = gravity.LengthSquared() > 0.1;
            Vector3D worldUp = hasGravity ? -Vector3D.Normalize(gravity) : refMatrix.Up;

            // If custom up is provided (docking), use it instead
            if (customUp.HasValue)
            {
                worldUp = Vector3D.Normalize(customUp.Value);
            }

            // === Execute appropriate control mode ===
            // Only use level turn if explicitly forced (via LevelTurnToward or OrientLevel)
            // Normal orientation should handle roll correctly now with fixed gravity reference
            if (forceLevelTurn)
            {
                return ExecuteLevelTurn(desiredForward, worldUp, currentForward, currentUp);
            }
            else
            {
                OrientationMode mode = (!hasGravity && !customUp.HasValue) ? OrientationMode.Free : _orientationMode;
                bool lockRoll = customUp.HasValue || mode == OrientationMode.HorizonLock || mode == OrientationMode.Limited;
                bool limitTilt = !customUp.HasValue && mode == OrientationMode.Limited;
                Vector3D adjustedForward = desiredForward;
                if (lockRoll && limitTilt)
                {
                    adjustedForward = ClampForwardToMaxTilt(desiredForward, worldUp, _maxTiltAngle, refMatrix.Forward);
                }
                return ExecuteFullOrientation(adjustedForward, worldUp, lockRoll, limitTilt, currentForward, currentUp, refMatrix);
            }
        }

        /// <summary>
        /// Level turn: Only yaw around gravity axis, keep pitch and roll locked to horizon.
        /// This prevents rolling during large turns.
        /// </summary>
        private bool ExecuteLevelTurn(Vector3D desiredForward, Vector3D worldUp, Vector3D currentForward, Vector3D currentUp)
        {
            // === SkunkBot's Simple Approach ===
            // Transform desired direction into local space, then use simple trig
            MatrixD refMatrix = _reference.WorldMatrix;
            MatrixD lookAtMatrix = MatrixD.CreateLookAt(Vector3D.Zero, refMatrix.Forward, refMatrix.Up);
            Vector3D localTarget = Vector3D.TransformNormal(desiredForward, lookAtMatrix);
            // In local space: +Z is forward, +Y is up, +X is right

            // === YAW: Only consider horizontal plane (X-Z) ===
            Vector3D yawPlane = new Vector3D(localTarget.X, 0, localTarget.Z);
            double yawError = 0;
            if (yawPlane.LengthSquared() > 0.0001)
            {
                yawPlane = Vector3D.Normalize(yawPlane);
                // Angle from forward in horizontal plane
                double yawAngle = Math.Acos(MathHelper.Clamp(Vector3D.Dot(yawPlane, Vector3D.Forward), -1, 1));
                // Sign based on which side (left is negative X, so flip sign)
                yawError = yawAngle * (localTarget.X < 0 ? -1 : 1);
            }

            // === PITCH: Drive to zero (lock nose to horizon) ===
            double currentPitch = Math.Asin(MathHelper.Clamp(Vector3D.Dot(currentForward, worldUp), -1, 1));
            double pitchError = -currentPitch;

            // === ROLL: Lock to gravity (lock wings to horizon) ===
            Vector3D worldUpProjected = worldUp - currentForward * Vector3D.Dot(worldUp, currentForward);
            double rollError = 0;
            if (worldUpProjected.LengthSquared() > 0.001)
            {
                worldUpProjected = Vector3D.Normalize(worldUpProjected);
                double rollDot = Vector3D.Dot(currentUp, worldUpProjected);
                double rollCross = Vector3D.Dot(currentForward, Vector3D.Cross(currentUp, worldUpProjected));
                rollError = Math.Atan2(rollCross, rollDot);
            }

            ApplyOrientationControl(pitchError, yawError, rollError);
            return true;
        }

        /// <summary>
        /// Full 3-axis orientation: Allows pitch, yaw, and roll to reach target orientation.
        /// Used for smaller corrections or when custom up vector is specified (docking).
        /// </summary>
        private bool ExecuteFullOrientation(Vector3D desiredForward, Vector3D worldUp, bool lockRollToUp, bool limitTilt,
            Vector3D currentForward, Vector3D currentUp, MatrixD refMatrix)
        {
            // === Transform to local space using LookAt matrix ===
            // This creates a stable local reference frame that doesn't couple axes
            MatrixD lookAtMatrix = MatrixD.CreateLookAt(Vector3D.Zero, refMatrix.Forward, refMatrix.Up);
            Vector3D localTarget = Vector3D.TransformNormal(desiredForward, lookAtMatrix);

            // In this local space: +Z is forward, +Y is up, +X is right
            // Decompose into yaw plane (X-Z) and pitch plane (Y-Z)
            Vector3D yawVector = new Vector3D(localTarget.X, 0, localTarget.Z);
            Vector3D pitchVector = new Vector3D(0, localTarget.Y, localTarget.Z);

            double yawError = 0;
            double pitchError = 0;
            double rollError = 0;

            // Yaw: angle in the horizontal plane
            if (yawVector.LengthSquared() > 0.0001)
            {
                yawVector = Vector3D.Normalize(yawVector);
                yawError = Math.Acos(MathHelper.Clamp(Vector3D.Dot(yawVector, Vector3D.Forward), -1, 1));
                if (localTarget.X < 0) yawError = -yawError;
            }

            // Pitch: angle in the vertical plane
            if (pitchVector.LengthSquared() > 0.0001)
            {
                pitchVector = Vector3D.Normalize(pitchVector);
                pitchError = Math.Acos(MathHelper.Clamp(Vector3D.Dot(pitchVector, Vector3D.Forward), -1, 1));
                if (localTarget.Y < 0) pitchError = -pitchError;
            }

            // === TILT LIMITING ===
            // Clamp pitch error to maximum tilt angle to prevent flipping
            if (lockRollToUp && limitTilt)
            {
                pitchError = MathHelper.Clamp(pitchError, -_maxTiltAngle, _maxTiltAngle);
            }

            // === ROLL: Align with up vector (gravity or custom) ===
            // For gravity-based roll: use the fixed world up, not "perpendicular to current forward"
            // This prevents roll drift during yaw/pitch rotations
            if (!lockRollToUp)
            {
                rollError = 0;
            }
            else if (lockRollToUp)
            {
                // Normal flight: keep wings level relative to horizon (fixed world reference)
                // Calculate roll by checking if our right axis is horizontal
                Vector3D currentRight = refMatrix.Right;

                // Project our right axis onto the horizontal plane
                Vector3D horizontalRight = currentRight - worldUp * Vector3D.Dot(currentRight, worldUp);

                if (horizontalRight.LengthSquared() > 0.001)
                {
                    horizontalRight = Vector3D.Normalize(horizontalRight);

                    // Roll error is how far our right axis deviates from horizontal
                    double rollDot = Vector3D.Dot(currentRight, horizontalRight);
                    Vector3D rollCrossVec = Vector3D.Cross(currentRight, horizontalRight);
                    double rollCross = Vector3D.Dot(rollCrossVec, currentForward);
                    rollError = Math.Atan2(rollCross, rollDot);
                }
            }

            // Apply full orientation control
            ApplyOrientationControl(pitchError, yawError, rollError);
            return true;
        }

        private Vector3D ClampForwardToMaxTilt(Vector3D forward, Vector3D worldUp, double maxTiltAngle, Vector3D fallbackForward)
        {
            if (maxTiltAngle <= 0)
                return forward;

            double maxSin = Math.Sin(maxTiltAngle);
            double vertical = Vector3D.Dot(forward, worldUp);
            double clampedVertical = MathHelper.Clamp(vertical, -maxSin, maxSin);

            Vector3D horizontal = forward - worldUp * vertical;
            if (horizontal.LengthSquared() < 0.001)
            {
                horizontal = fallbackForward - worldUp * Vector3D.Dot(fallbackForward, worldUp);
            }
            if (horizontal.LengthSquared() < 0.001)
                return forward;

            Vector3D horizontalDir = Vector3D.Normalize(horizontal);
            double horizontalMag = Math.Sqrt(Math.Max(0, 1.0 - clampedVertical * clampedVertical));
            return horizontalDir * horizontalMag + worldUp * clampedVertical;
        }

        /// <summary>
        /// Orients the grid so a specific connector faces the target direction.
        /// Used for docking where a specific block (not the ship's nose) must align.
        /// </summary>
        /// <param name="block">The block to align</param>
        /// <param name="targetDirection">World direction the block should face</param>
        /// <param name="desiredUp">Desired up direction (world space)</param>
        /// <returns>True if orientation was applied</returns>
        public bool AlignBlockToDirection(
            IMyCubeBlock block,
            Vector3D targetDirection,
            Vector3D desiredUp)
        {
            if (_reference == null || _gyros.Count == 0 || block == null)
                return false;

            if (targetDirection.LengthSquared() < 0.001)
                return false;

            targetDirection = Vector3D.Normalize(targetDirection);
            desiredUp = Vector3D.Normalize(desiredUp);

            // Get current orientations
            MatrixD shipMatrix = _reference.WorldMatrix;
            MatrixD connectorMatrix = block.WorldMatrix;

            // Get rotation from ship to connector
            MatrixD shipToConnector = connectorMatrix * MatrixD.Invert(shipMatrix);

            // Build desired orientation where connector would face target
            Vector3D desiredConnectorForward = targetDirection;

            // Orthogonalize desiredUp to be perpendicular to connector forward
            Vector3D desiredConnectorUp = desiredUp - desiredConnectorForward * Vector3D.Dot(desiredUp, desiredConnectorForward);
            if (desiredConnectorUp.LengthSquared() < 0.001)
            {
                // desiredUp is parallel to forward, pick an arbitrary perpendicular
                desiredConnectorUp = Vector3D.CalculatePerpendicularVector(desiredConnectorForward);
            }
            desiredConnectorUp = Vector3D.Normalize(desiredConnectorUp);

            // Build desired connector world matrix
            Vector3D desiredConnectorRight = Vector3D.Cross(desiredConnectorForward, desiredConnectorUp);

            MatrixD desiredConnectorMatrix = new MatrixD(
                desiredConnectorRight.X, desiredConnectorRight.Y, desiredConnectorRight.Z, 0,
                desiredConnectorUp.X, desiredConnectorUp.Y, desiredConnectorUp.Z, 0,
                -desiredConnectorForward.X, -desiredConnectorForward.Y, -desiredConnectorForward.Z, 0,
                0, 0, 0, 1
            );

            // Calculate desired ship orientation
            MatrixD connectorToShip = MatrixD.Invert(shipToConnector);
            MatrixD desiredShipMatrix = connectorToShip * desiredConnectorMatrix;

            // Use orientation control with the computed desired forward and up (never use level turn for docking)
            return OrientToward(desiredShipMatrix.Forward, desiredShipMatrix.Up, false);
        }

        /// <summary>
        /// Applies orientation control given the current errors.
        /// Uses multi-stage control: coarse dampened, gap, and fine PID.
        /// </summary>
        private void ApplyOrientationControl(double pitchError, double yawError, double rollError)
        {
            // === Predictive braking ===
            bool pitchBraking = ShouldBrake(pitchError, ref _lastPitchError, ref _lastPitchVelocity);
            bool yawBraking = ShouldBrake(yawError, ref _lastYawError, ref _lastYawVelocity);
            bool rollBraking = ShouldBrake(rollError, ref _lastRollError, ref _lastRollVelocity);

            // Total angular error for status display and deadband
            TotalError = Math.Sqrt(pitchError * pitchError + yawError * yawError + rollError * rollError);

            // === Deadband without PID reset ===
            if (TotalError < ALIGNMENT_DEADBAND)
            {
                SetGyroOverride(Vector3D.Zero);
                // Don't reset PID - let integral persist for stability
                return;
            }

            // Calculate angular velocity (how fast we're rotating)
            double angularVelocity = Math.Sqrt(
                _lastPitchVelocity * _lastPitchVelocity +
                _lastYawVelocity * _lastYawVelocity
            );

            // === Multi-stage control with proper PID gating ===
            Vector3D correction;

            if (TotalError > DAMPEN_THRESHOLD)
            {
                // Large error: use dampened sinusoidal control
                correction = new Vector3D(
                    pitchBraking ? 0 : DampenAngle(pitchError),
                    yawBraking ? 0 : DampenAngle(yawError),
                    rollBraking ? 0 : DampenAngle(rollError) * 0.5
                );

                // Reset PID when in coarse mode to prevent integral windup
                if (_pidActive)
                {
                    _orientationPID.Reset();
                    _pidActive = false;
                }
            }
            else if (TotalError < PID_ANGLE_THRESHOLD || angularVelocity < PID_VELOCITY_THRESHOLD)
            {
                // Fine control: PID only when close AND/OR moving slowly
                _pidActive = true;

                Vector3D localError = new Vector3D(pitchError, yawError, rollError);
                correction = _orientationPID.Compute(localError, _deltaTime);

                // Cap output like SkunkBot does
                correction = new Vector3D(
                    MathHelper.Clamp(correction.X, -PID_MAX_OUTPUT, PID_MAX_OUTPUT),
                    MathHelper.Clamp(correction.Y, -PID_MAX_OUTPUT, PID_MAX_OUTPUT),
                    MathHelper.Clamp(correction.Z, -PID_MAX_OUTPUT, PID_MAX_OUTPUT)
                );
            }
            else
            {
                // In the gap: still use dampened but gentler
                correction = new Vector3D(
                    pitchBraking ? 0 : DampenAngle(pitchError) * 0.5,
                    yawBraking ? 0 : DampenAngle(yawError) * 0.5,
                    rollBraking ? 0 : DampenAngle(rollError) * 0.25
                );
            }

            // Apply to gyro
            SetGyroOverride(correction);
        }

        /// <summary>
        /// Sinusoidal dampening function for smooth acceleration/deceleration.
        /// Returns a value between -MAX_ANGULAR_VELOCITY and +MAX_ANGULAR_VELOCITY.
        /// </summary>
        private double DampenAngle(double angle)
        {
            if (Math.Abs(angle) < 0.001) return 0;

            double sign = Math.Sign(angle);
            double absAngle = Math.Abs(angle);

            // Above threshold, return max speed
            if (absAngle > DAMPEN_THRESHOLD * 5)  // ~30 degrees
                return sign * MAX_ANGULAR_VELOCITY;

            // Sinusoidal ramp: smooth acceleration and deceleration
            double t = absAngle / (DAMPEN_THRESHOLD * 5);
            return sign * Math.Sin(t * Math.PI / 2) * MAX_ANGULAR_VELOCITY;
        }

        /// <summary>
        /// Predictive braking: estimates if we'll overshoot and should stop applying torque.
        /// </summary>
        private bool ShouldBrake(double currentError, ref double lastError, ref double lastVelocity)
        {
            double dt = _deltaTime;
            if (dt < 0.001) dt = 0.016;  // Default to ~60fps if dt is invalid

            // Calculate current angular velocity (change in error per second)
            double velocity = Math.Abs(lastError - currentError) / dt;

            // Calculate deceleration rate
            double decel = Math.Abs(lastVelocity - velocity) / dt;

            // Estimate time to reach target vs time to stop
            double timeToTarget = velocity > 0.001 ? Math.Abs(currentError) / velocity : double.MaxValue;
            double timeToStop = decel > 0.001 ? velocity / decel : double.MaxValue;

            // Are we getting closer?
            bool closing = Math.Abs(currentError) < Math.Abs(lastError);

            // Update state for next tick
            lastError = currentError;
            lastVelocity = velocity;

            // Brake if: closing AND would overshoot AND not already very close
            return closing && timeToStop > timeToTarget * 1.2 && Math.Abs(currentError) > ALIGNMENT_DEADBAND * 2;
        }

        /// <summary>
        /// Applies rotation commands to a single gyro.
        /// A single gyro override inherits total grid torque, so we don't need to drive all gyros.
        /// </summary>
        private void SetGyroOverride(Vector3D localRotation)
        {
            if (_reference == null || _gyros.Count == 0)
                return;

            MatrixD refMatrix = _reference.WorldMatrix;

            // Negate pitch at input (SE gyro convention)
            Vector3D rotationVec = new Vector3D(-localRotation.X, localRotation.Y, localRotation.Z);
            Vector3D worldRotation = Vector3D.TransformNormal(rotationVec, refMatrix);

            for (int i = 0; i < _gyros.Count; i++)
            {
                var gyro = _gyros[i];
                if (gyro == null || gyro.Closed || !gyro.IsFunctional || !gyro.Enabled)
                    continue;

                // Transform to this gyro's local space
                Vector3D gyroLocal = Vector3D.TransformNormal(worldRotation, MatrixD.Transpose(gyro.WorldMatrix));

                gyro.GyroOverride = true;
                gyro.Pitch = (float)gyroLocal.X;
                gyro.Yaw = (float)gyroLocal.Y;
                gyro.Roll = (float)gyroLocal.Z;
                break;
            }
        }

        /// <summary>
        /// Releases gyro control, setting all overrides to zero.
        /// </summary>
        public void Release()
        {
            foreach (var gyro in _gyros)
            {
                if (gyro != null && !gyro.Closed)
                {
                    gyro.GyroOverride = false;
                    gyro.Pitch = 0;
                    gyro.Yaw = 0;
                    gyro.Roll = 0;
                }
            }
        }

        /// <summary>
        /// Gets the count of available gyros.
        /// </summary>
        public int GyroCount => _gyros.Count;
    }
}
