using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Manages orientation control via gyroscopes.
    /// Provides a simple LookAt interface for callers - they specify what to look at,
    /// this class handles all the gyro math, PID control, and predictive braking.
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
        
        // === 180° turn hysteresis ===
        private bool _inBackwardsTurn = false;  // Are we in the middle of a 180° turn?
        private double _lockedTurnSign = 1.0;   // Which direction we committed to (+1 = right, -1 = left)
        
        // === Control tuning ===
        private const double MAX_ANGULAR_VELOCITY = 2.0;  // rad/s - max rotation speed for dampened control
        private const double ALIGNMENT_DEADBAND = 0.005;  // ~0.3 degrees - very tight deadband
        private const double DAMPEN_THRESHOLD = 0.1;      // ~6 degrees - switch to fine control below this
        private const double PID_ANGLE_THRESHOLD = 0.05;  // ~3 degrees - angle threshold for PID
        private const double PID_VELOCITY_THRESHOLD = 0.25; // rad/s - velocity threshold for PID
        private const double PID_MAX_OUTPUT = 2.0;        // rad/s - cap PID output
        private const double DEFAULT_MAX_TILT = 0.35;     // ~20 degrees default max tilt
        
        // === Tilt limiting ===
        private double _maxTiltAngle = DEFAULT_MAX_TILT;  // radians
        
        // === PID state tracking ===
        private bool _pidActive = false;
        
        // === Current state ===
        private double _deltaTime = 0.016;  // Default to ~60fps
        
        /// <summary>
        /// The current total angular error in radians.
        /// </summary>
        public double TotalError { get; private set; }
        
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
        /// Resets the backwards turn lock state.
        /// Call this when switching orientation modes to prevent stale lock state.
        /// </summary>
        public void ResetTurnLock()
        {
            _inBackwardsTurn = false;
            _lockedTurnSign = 1.0;
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
            Vector3D leaderRight = Vector3D.Cross(leaderForward, leaderUp);
            
            Vector3D horizontalForward = VectorMath.ProjectOntoHorizontalPlane(leaderForward, gravity, leaderRight);
            
            if (horizontalForward.LengthSquared() > 0.001)
            {
                return OrientToward(horizontalForward);
            }
            
            // Degenerate case - maintain current orientation
            return false;
        }

        /// <summary>
        /// Orients the grid to face a target position using yaw only (rotation around gravity axis).
        /// Keeps the grid level by not applying pitch, only yaw and roll correction.
        /// Use this for conservative turns where you want to stay level while changing heading.
        /// </summary>
        /// <param name="worldTarget">The world position to turn toward</param>
        /// <returns>True if orientation was applied</returns>
        public bool LevelTurnToward(Vector3D worldTarget)
        {
            if (_reference == null || _gyros.Count == 0)
                return false;

            Vector3D gravity = _reference.GetNaturalGravity();
            if (gravity.LengthSquared() < 0.1)
            {
                // No gravity - fall back to regular LookAt
                return LookAt(worldTarget);
            }

            Vector3D myPosition = _reference.GetPosition();
            Vector3D toTarget = worldTarget - myPosition;
            
            if (toTarget.LengthSquared() < 1)
                return false;

            // Project target direction onto horizontal plane
            Vector3D worldUp = -Vector3D.Normalize(gravity);
            Vector3D horizontalToTarget = toTarget - worldUp * Vector3D.Dot(toTarget, worldUp);
            
            if (horizontalToTarget.LengthSquared() < 0.001)
            {
                // Target is directly above/below - just maintain level, no yaw needed
                return OrientLevel();
            }

            Vector3D desiredForward = Vector3D.Normalize(horizontalToTarget);
            return OrientTowardLevel(desiredForward, worldUp);
        }

        /// <summary>
        /// Orients the grid to face a horizontal direction while staying level.
        /// Only applies yaw (rotation around gravity axis) and roll correction.
        /// Pitch is actively driven to zero (level flight).
        /// Uses gentler control than ApplyOrientationControl to avoid oscillation.
        /// </summary>
        /// <param name="horizontalDirection">The horizontal direction to face (should already be in horizontal plane)</param>
        /// <param name="worldUp">The "up" direction (opposite of gravity)</param>
        /// <returns>True if orientation was applied</returns>
        private bool OrientTowardLevel(Vector3D horizontalDirection, Vector3D worldUp)
        {
            if (_reference == null || _gyros.Count == 0)
                return false;

            MatrixD refMatrix = _reference.WorldMatrix;
            Vector3D currentForward = refMatrix.Forward;
            Vector3D currentUp = refMatrix.Up;

            // Project current forward onto horizontal plane for yaw calculation
            Vector3D currentHorizontalForward = currentForward - worldUp * Vector3D.Dot(currentForward, worldUp);
            if (currentHorizontalForward.LengthSquared() < 0.001)
            {
                currentHorizontalForward = refMatrix.Backward; // Pointing straight up/down, use any horizontal reference
            }
            currentHorizontalForward = Vector3D.Normalize(currentHorizontalForward);

            // === YAW: Use atan2 for proper signed angle, with hysteresis for large turns ===
            double yawCos = Vector3D.Dot(currentHorizontalForward, horizontalDirection);
            Vector3D yawCross = Vector3D.Cross(currentHorizontalForward, horizontalDirection);
            double yawSin = Vector3D.Dot(yawCross, worldUp);
            
            // Calculate facing angle for threshold checks
            double facingAngleRad = Math.Acos(MathHelper.Clamp(yawCos, -1, 1));
            double facingAngleDeg = facingAngleRad * 180.0 / Math.PI;
            
            // Handle the 180° instability with hysteresis
            // Enter backwards mode when more than ~120° off (cos < -0.5)
            // Stay locked until actually facing target within 30° (same as LEVEL_TURN_COMPLETE_THRESHOLD)
            double yawError;
            
            if (_inBackwardsTurn)
            {
                // We're in the middle of a large turn - keep turning the locked direction
                // Only exit when we're actually facing the target (within 30°)
                if (facingAngleDeg < 30.0)
                {
                    // We're now facing the target - exit backwards turn mode
                    _inBackwardsTurn = false;
                    yawError = Math.Atan2(yawSin, yawCos);  // Fine control for final alignment
                }
                else
                {
                    // Still turning - keep the locked direction
                    yawError = _lockedTurnSign * facingAngleRad;
                }
            }
            else
            {
                // Normal mode - check if we should enter backwards turn mode
                if (yawCos < -0.5)  // More than ~120° off
                {
                    // Enter backwards turn mode
                    _inBackwardsTurn = true;
                    // Lock to whichever direction has less turn, but if ambiguous (sin near 0), pick positive
                    _lockedTurnSign = (yawSin >= 0) ? 1.0 : -1.0;
                    yawError = _lockedTurnSign * facingAngleRad;
                }
                else
                {
                    // Normal case - use atan2
                    yawError = Math.Atan2(yawSin, yawCos);
                }
            }

            // === DEBUG OUTPUT ===
            double yawErrorDeg = yawError * 180.0 / Math.PI;
            _echo?.Invoke($"[GYRO] Angle:{facingAngleDeg:F0}° Cmd:{yawErrorDeg:F0}° Lock:{(_inBackwardsTurn ? "Y" : "N")}");

            // === PITCH: Drive to zero (level flight) ===
            // Current pitch is the angle of our forward from horizontal
            double currentPitch = Math.Asin(MathHelper.Clamp(Vector3D.Dot(currentForward, worldUp), -1, 1));
            double pitchError = -currentPitch;  // We want zero pitch, so error is negative of current

            // === ROLL: Align up with gravity ===
            double rollError = 0;
            Vector3D worldUpProjected = worldUp - currentForward * Vector3D.Dot(worldUp, currentForward);
            if (worldUpProjected.LengthSquared() > 0.001)
            {
                worldUpProjected = Vector3D.Normalize(worldUpProjected);
                double rollDot = Vector3D.Dot(currentUp, worldUpProjected);
                double rollCross = Vector3D.Dot(currentForward, Vector3D.Cross(currentUp, worldUpProjected));
                rollError = Math.Atan2(rollCross, rollDot);
            }

            // Use gentler proportional control for level turns to avoid oscillation
            // Cap yaw rate to prevent overshoot
            ApplyLevelTurnControl(pitchError, yawError, rollError);
            return true;
        }

        /// <summary>
        /// Simpler control specifically for level turns.
        /// Uses proportional control only, no predictive braking, to avoid oscillation.
        /// </summary>
        private void ApplyLevelTurnControl(double pitchError, double yawError, double rollError)
        {
            // Simple proportional gains - gentle to avoid oscillation
            const double PITCH_GAIN = 1.5;  // Level quickly but not aggressively
            const double YAW_GAIN = 0.8;    // Gentle yaw to prevent overshoot
            const double ROLL_GAIN = 1.0;   // Moderate roll correction
            const double MAX_YAW_RATE = 0.5;  // rad/s - cap yaw to prevent overshoot
            const double MAX_PITCH_RATE = 1.0;
            const double MAX_ROLL_RATE = 0.5;

            // Simple proportional control with rate limiting
            double pitchCommand = MathHelper.Clamp(pitchError * PITCH_GAIN, -MAX_PITCH_RATE, MAX_PITCH_RATE);
            double yawCommand = MathHelper.Clamp(yawError * YAW_GAIN, -MAX_YAW_RATE, MAX_YAW_RATE);
            double rollCommand = MathHelper.Clamp(rollError * ROLL_GAIN, -MAX_ROLL_RATE, MAX_ROLL_RATE);

            // Deadband
            const double DEADBAND = 0.02;  // ~1 degree
            if (Math.Abs(pitchError) < DEADBAND) pitchCommand = 0;
            if (Math.Abs(yawError) < DEADBAND) yawCommand = 0;
            if (Math.Abs(rollError) < DEADBAND) rollCommand = 0;

            // DEBUG: Show what we're commanding
            _echo?.Invoke($"[GYRO CMD] Yaw:{yawCommand:F2} Pitch:{pitchCommand:F2} Roll:{rollCommand:F2}");

            Vector3D correction = new Vector3D(pitchCommand, yawCommand, rollCommand);
            SetGyroOverride(correction);
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
            Vector3D currentUp = refMatrix.Up;

            // Project current forward onto horizontal plane to maintain heading
            Vector3D horizontalForward = currentForward - worldUp * Vector3D.Dot(currentForward, worldUp);
            if (horizontalForward.LengthSquared() < 0.001)
            {
                horizontalForward = refMatrix.Backward;
            }
            horizontalForward = Vector3D.Normalize(horizontalForward);

            return OrientTowardLevel(horizontalForward, worldUp);
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
            double angleDeg = angle * 180.0 / Math.PI;
            double toleranceRad = toleranceDegrees * Math.PI / 180.0;
            
            bool facing = angle < toleranceRad;
            _echo?.Invoke($"[FACING] Angle:{angleDeg:F1}° Tol:{toleranceDegrees}° => {(facing ? "YES" : "NO")}");
            
            return facing;
        }

        /// <summary>
        /// Orients the grid to face a world-space target position.
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
        /// </summary>
        /// <param name="worldDirection">The world-space direction to face (will be normalized)</param>
        /// <returns>True if orientation was applied</returns>
        public bool OrientToward(Vector3D worldDirection)
        {
            if (_reference == null || _gyros.Count == 0)
                return false;
                
            if (worldDirection.LengthSquared() < 0.001)
                return false;

            Vector3D desiredForward = Vector3D.Normalize(worldDirection);
            MatrixD refMatrix = _reference.WorldMatrix;
            Vector3D currentForward = refMatrix.Forward;
            Vector3D currentUp = refMatrix.Up;

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
            pitchError = MathHelper.Clamp(pitchError, -_maxTiltAngle, _maxTiltAngle);

            // === ROLL: Align with gravity if available ===
            Vector3D gravityVec = _reference.GetNaturalGravity();
            if (gravityVec.LengthSquared() > 0.1)
            {
                Vector3D worldUp = -Vector3D.Normalize(gravityVec);
                
                // Project world up onto the plane perpendicular to forward
                Vector3D worldUpProjected = worldUp - currentForward * Vector3D.Dot(worldUp, currentForward);
                if (worldUpProjected.LengthSquared() > 0.001)
                {
                    worldUpProjected = Vector3D.Normalize(worldUpProjected);
                    double rollDot = Vector3D.Dot(currentUp, worldUpProjected);
                    double rollCross = Vector3D.Dot(currentForward, Vector3D.Cross(currentUp, worldUpProjected));
                    rollError = Math.Atan2(rollCross, rollDot);
                }
            }

            // Apply control
            ApplyOrientationControl(pitchError, yawError, rollError);
            return true;
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
        /// Applies rotation commands to gyros.
        /// Uses single-gyro mode if a pilot is in control (allows pilot override).
        /// Uses all gyros when autonomous for maximum turning authority.
        /// </summary>
        private void SetGyroOverride(Vector3D localRotation)
        {
            if (_reference == null || _gyros.Count == 0)
                return;

            MatrixD refMatrix = _reference.WorldMatrix;
            
            // Negate pitch at input (SE gyro convention)
            Vector3D rotationVec = new Vector3D(-localRotation.X, localRotation.Y, localRotation.Z);
            Vector3D worldRotation = Vector3D.TransformNormal(rotationVec, refMatrix);

            // If pilot is in control, use only one gyro (allows pilot to override other gyros)
            // If autonomous, use all gyros for maximum turning power
            bool singleGyroMode = _reference.IsUnderControl;

            foreach (var gyro in _gyros)
            {
                if (gyro == null || gyro.Closed || !gyro.IsFunctional || !gyro.Enabled)
                    continue;

                // Transform to this gyro's local space
                Vector3D gyroLocal = Vector3D.TransformNormal(worldRotation, MatrixD.Transpose(gyro.WorldMatrix));

                gyro.GyroOverride = true;
                gyro.Pitch = (float)gyroLocal.X;
                gyro.Yaw = (float)gyroLocal.Y;
                gyro.Roll = (float)gyroLocal.Z;

                // In single-gyro mode, stop after the first functional gyro
                if (singleGyroMode)
                    break;
            }
        }

        /// <summary>
        /// Returns the number of functional, enabled gyros available.
        /// </summary>
        private int GetActiveGyroCount()
        {
            int count = 0;
            foreach (var gyro in _gyros)
            {
                if (gyro != null && !gyro.Closed && gyro.IsFunctional && gyro.Enabled)
                    count++;
            }
            return count;
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
