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
        private IMyGyro _activeGyro;  // Single gyro for consistent control authority
        
        // === Predictive braking state (per-axis tracking) ===
        private double _lastPitchError;
        private double _lastYawError;
        private double _lastRollError;
        private double _lastPitchVelocity;
        private double _lastYawVelocity;
        private double _lastRollVelocity;
        
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
            SelectActiveGyro();
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
            if (_reference == null || _activeGyro == null)
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
        /// Orients the grid to face a world-space target position.
        /// Call this every tick to maintain orientation.
        /// </summary>
        /// <param name="worldTarget">The world position to look at</param>
        /// <returns>True if orientation was applied, false if unable (no gyro, invalid target)</returns>
        public bool LookAt(Vector3D worldTarget)
        {
            if (_reference == null || _activeGyro == null)
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
            if (_reference == null || _activeGyro == null)
                return false;
                
            if (worldDirection.LengthSquared() < 0.001)
                return false;

            Vector3D desiredForward = Vector3D.Normalize(worldDirection);
            MatrixD refMatrix = _reference.WorldMatrix;

            // === Transform to local space using LookAt matrix ===
            // This creates a stable local reference frame that doesn't couple axes
            MatrixD lookAtMatrix = MatrixD.CreateLookAt(Vector3D.Zero, refMatrix.Forward, refMatrix.Up);
            Vector3D localTarget = Vector3D.TransformNormal(desiredForward, lookAtMatrix);
            
            // In this local space: +Z is forward, +Y is up, +X is right
            // We need to find yaw (rotation around Y) and pitch (rotation around X)
            
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
                // Use dot product with Forward (0,0,-1), not raw Z component
                yawError = Math.Acos(MathHelper.Clamp(Vector3D.Dot(yawVector, Vector3D.Forward), -1, 1));
                if (localTarget.X < 0) yawError = -yawError;  // Sign based on which side target is
            }
            
            // Pitch: angle in the vertical plane
            if (pitchVector.LengthSquared() > 0.0001)
            {
                pitchVector = Vector3D.Normalize(pitchVector);
                // Use dot product with Forward (0,0,-1), not raw Z component
                pitchError = Math.Acos(MathHelper.Clamp(Vector3D.Dot(pitchVector, Vector3D.Forward), -1, 1));
                if (localTarget.Y < 0) pitchError = -pitchError;  // Negative Y = target below = pitch down
            }
            
            // === TILT LIMITING ===
            // Clamp pitch to maximum tilt angle to prevent flipping
            pitchError = MathHelper.Clamp(pitchError, -_maxTiltAngle, _maxTiltAngle);

            // Roll: align with gravity if available (try to stay level)
            Vector3D gravityVec = _reference.GetNaturalGravity();
            if (gravityVec.LengthSquared() > 0.1)
            {
                Vector3D worldUp = -Vector3D.Normalize(gravityVec);
                Vector3D currentUp = refMatrix.Up;
                Vector3D currentForward = refMatrix.Forward;
                
                // Project world up onto the plane perpendicular to forward
                Vector3D worldUpProjected = worldUp - currentForward * Vector3D.Dot(worldUp, currentForward);
                if (worldUpProjected.LengthSquared() > 0.001)
                {
                    worldUpProjected = Vector3D.Normalize(worldUpProjected);
                    double rollDot = Vector3D.Dot(currentUp, worldUpProjected);
                    double rollCross = Vector3D.Dot(currentForward, Vector3D.Cross(currentUp, worldUpProjected));
                    rollError = Math.Atan2(rollCross, rollDot);
                    
                    // Clamp roll to maximum tilt angle
                    rollError = MathHelper.Clamp(rollError, -_maxTiltAngle, _maxTiltAngle);
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

        private void SetGyroOverride(Vector3D localRotation)
        {
            if (_activeGyro == null || !_activeGyro.IsFunctional)
            {
                SelectActiveGyro();
                if (_activeGyro == null) return;
            }

            MatrixD refMatrix = _reference.WorldMatrix;
            
            // Match SkunkBot exactly: negate pitch at input, not yaw/roll at output
            Vector3D rotationVec = new Vector3D(-localRotation.X, localRotation.Y, localRotation.Z);
            Vector3D worldRotation = Vector3D.TransformNormal(rotationVec, refMatrix);
            Vector3D gyroLocal = Vector3D.TransformNormal(worldRotation, MatrixD.Transpose(_activeGyro.WorldMatrix));

            _activeGyro.GyroOverride = true;
            _activeGyro.Pitch = (float)gyroLocal.X;
            _activeGyro.Yaw = (float)gyroLocal.Y;
            _activeGyro.Roll = (float)gyroLocal.Z;
        }

        /// <summary>
        /// Selects a single active gyro for consistent control authority.
        /// </summary>
        private void SelectActiveGyro()
        {
            _activeGyro = null;
            foreach (var gyro in _gyros)
            {
                if (gyro != null && gyro.IsFunctional && !gyro.Closed && gyro.Enabled)
                {
                    _activeGyro = gyro;
                    break;  // Use only one gyro for consistent behavior
                }
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
