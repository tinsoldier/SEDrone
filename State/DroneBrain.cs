using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Brain for follower drones.
    /// Responsibilities:
    /// - Listen for leader broadcasts over IGC
    /// - Track leader position
    /// - Orient to face the leader (Milestone 1)
    /// - (Future) Maintain formation position
    /// - (Future) Execute formation commands
    /// </summary>
    public class DroneBrain : IBrain
    {
        public string Name => "Drone";
        public string Status => _status;

        private string _status = "Initializing";
        private BrainContext _context;
        private IMyBroadcastListener _listener;
        
        // Leader tracking
        private LeaderStateMessage _lastLeaderState;
        private bool _hasLeaderContact;
        private double _lastContactTime;
        private const double CONTACT_TIMEOUT = 2.0;  // Seconds before leader considered lost

        // Gyro control
        private List<IMyGyro> _gyros = new List<IMyGyro>();
        private IMyGyro _activeGyro;  // Single gyro for consistent control authority
        private PIDController3D _orientationPID;
        
        // Predictive braking state (per-axis tracking)
        private double _lastPitchError;
        private double _lastYawError;
        private double _lastRollError;
        private double _lastPitchVelocity;
        private double _lastYawVelocity;
        private double _lastRollVelocity;
        
        // Control tuning
        private const double MAX_ANGULAR_VELOCITY = 2.0;  // rad/s - max rotation speed for dampened control
        private const double ALIGNMENT_DEADBAND = 0.005;  // ~0.3 degrees - very tight deadband
        private const double DAMPEN_THRESHOLD = 0.1;      // ~6 degrees - switch to fine control below this
        private const double PID_ANGLE_THRESHOLD = 0.05;  // ~3 degrees - angle threshold for PID
        private const double PID_VELOCITY_THRESHOLD = 0.25; // rad/s - velocity threshold for PID
        private const double PID_MAX_OUTPUT = 2.0;        // rad/s - cap PID output
        
        // PID state tracking
        private bool _pidActive = false;

        public void Initialize(BrainContext context)
        {
            _context = context;
            
            // Register for IGC broadcasts
            // Note: We poll for messages in ProcessMessages() rather than using callbacks
            // This ensures we process messages every tick regardless of callback timing
            _listener = _context.IGC.RegisterBroadcastListener(_context.Config.IGCChannel);

            // Find gyros on this grid only
            _context.GridTerminalSystem.GetBlocksOfType(_gyros, g => g.CubeGrid == _context.Me.CubeGrid);
            
            // Select a single active gyro for consistent control authority
            SelectActiveGyro();
            
            // Initialize orientation PID (used for fine control only)
            _orientationPID = new PIDController3D(_context.Config.OrientationPID);

            _status = $"Listening on: {_context.Config.IGCChannel} (Gyros: {_gyros.Count})";
        }
        
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

        public IEnumerator<bool> Run()
        {
            // === PHASE 1: Wait for leader contact ===
            _status = "Searching for leader...";
            _context.Echo?.Invoke($"[{Name}] Waiting for leader broadcasts on: {_context.Config.IGCChannel}");

            while (!_hasLeaderContact)
            {
                ProcessMessages();
                yield return true;
            }

            // === PHASE 2: Track and face leader ===
            _context.Echo?.Invoke($"[{Name}] Leader acquired: {_lastLeaderState.GridName}");

            while (true)
            {
                ProcessMessages();

                // Check for leader timeout
                if (_context.GameTime - _lastContactTime > CONTACT_TIMEOUT)
                {
                    _hasLeaderContact = false;
                    _status = "Leader lost! Searching...";
                    ReleaseGyros();
                    
                    // Wait for reacquisition
                    while (!_hasLeaderContact)
                    {
                        ProcessMessages();
                        yield return true;
                    }
                    _context.Echo?.Invoke($"[{Name}] Leader reacquired: {_lastLeaderState.GridName}");
                }

                // Face the leader
                FaceLeader();

                // Calculate distance and angular offset for status
                Vector3D myPosition = _context.Reference.GetPosition();
                Vector3D toLeader = _lastLeaderState.Position - myPosition;
                double distance = toLeader.Length();
                
                // Calculate angle between current forward and direction to leader
                double angleOff = 0;
                if (distance > 1)
                {
                    Vector3D dirToLeader = toLeader / distance;  // Normalize
                    Vector3D currentForward = _context.Reference.WorldMatrix.Forward;
                    double dot = Vector3D.Dot(currentForward, dirToLeader);
                    angleOff = Math.Acos(MathHelper.Clamp(dot, -1, 1)) * 180.0 / Math.PI;
                }
                
                _status = $"Tracking: {_lastLeaderState.GridName} ({distance:F0}m, {angleOff:F1}°)";

                yield return true;
            }
        }

        private void ProcessMessages()
        {
            while (_listener.HasPendingMessage)
            {
                var msg = _listener.AcceptMessage();
                var data = msg.Data as string;
                if (data != null)
                {
                    LeaderStateMessage leaderState;
                    if (LeaderStateMessage.TryParse(data, out leaderState))
                    {
                        _lastLeaderState = leaderState;
                        _hasLeaderContact = true;
                        _lastContactTime = _context.GameTime;
                    }
                }
            }
        }

        private void FaceLeader()
        {
            if (_context.Reference == null || !_hasLeaderContact || _activeGyro == null)
                return;

            // Calculate direction to leader
            Vector3D myPosition = _context.Reference.GetPosition();
            Vector3D toLeader = _lastLeaderState.Position - myPosition;
            
            if (toLeader.LengthSquared() < 1)  // Too close, avoid division by zero
                return;

            Vector3D desiredForward = Vector3D.Normalize(toLeader);
            MatrixD refMatrix = _context.Reference.WorldMatrix;

            // === FIX #1: Transform to local space using LookAt matrix ===
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
            
            // // Yaw: angle in the horizontal plane
            if (yawVector.LengthSquared() > 0.0001)
            {
                yawVector = Vector3D.Normalize(yawVector);
                // FIX: Use dot product with Forward (0,0,-1), not raw Z component
                yawError = Math.Acos(MathHelper.Clamp(Vector3D.Dot(yawVector, Vector3D.Forward), -1, 1));
                if (localTarget.X < 0) yawError = -yawError;  // Sign based on which side target is
            }
            
            // Pitch: angle in the vertical plane
            if (pitchVector.LengthSquared() > 0.0001)
            {
                pitchVector = Vector3D.Normalize(pitchVector);
                // FIX: Same fix - use dot product with Forward
                pitchError = Math.Acos(MathHelper.Clamp(Vector3D.Dot(pitchVector, Vector3D.Forward), -1, 1));
                if (localTarget.Y < 0) pitchError = -pitchError;  // Negative Y = target below = pitch down
            }     

            _context.Echo?.Invoke($"localTarget: {localTarget.X:F2}, {localTarget.Y:F2}, {localTarget.Z:F2}");
            _context.Echo?.Invoke($"errors: P={pitchError:F3} Y={yawError:F3}");

            // Roll: align with gravity if available
            Vector3D gravityVec = _context.Reference.GetNaturalGravity();
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
                }
            }

            // === FIX #2: Predictive braking ===
            bool pitchBraking = ShouldBrake(pitchError, ref _lastPitchError, ref _lastPitchVelocity);
            bool yawBraking = ShouldBrake(yawError, ref _lastYawError, ref _lastYawVelocity);
            bool rollBraking = ShouldBrake(rollError, ref _lastRollError, ref _lastRollVelocity);

            // Total angular error for status display and deadband
            double totalError = Math.Sqrt(pitchError * pitchError + yawError * yawError + rollError * rollError);
            
            // === FIX #3: Deadband without PID reset ===
            if (totalError < ALIGNMENT_DEADBAND)
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
            
            if (totalError > DAMPEN_THRESHOLD)
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
            else if (totalError < PID_ANGLE_THRESHOLD || angularVelocity < PID_VELOCITY_THRESHOLD)
            {
                // Fine control: PID only when close AND/OR moving slowly
                _pidActive = true;
                
                Vector3D localError = new Vector3D(pitchError, yawError, rollError);
                correction = _orientationPID.Compute(localError, _context.DeltaTime);
                
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
            double dt = _context.DeltaTime;
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

            MatrixD refMatrix = _context.Reference.WorldMatrix;
            
            // Match SkunkBot exactly: negate pitch at input, not yaw/roll at output
            Vector3D rotationVec = new Vector3D(-localRotation.X, localRotation.Y, localRotation.Z);
            Vector3D worldRotation = Vector3D.TransformNormal(rotationVec, refMatrix);
            Vector3D gyroLocal = Vector3D.TransformNormal(worldRotation, MatrixD.Transpose(_activeGyro.WorldMatrix));

            _activeGyro.GyroOverride = true;
            _activeGyro.Pitch = (float)gyroLocal.X;
            _activeGyro.Yaw = (float)gyroLocal.Y;
            _activeGyro.Roll = (float)gyroLocal.Z;
        }

        // private void SetGyroOverride(Vector3D localRotation)
        // {
        //     if (_activeGyro == null || !_activeGyro.IsFunctional)
        //     {
        //         SelectActiveGyro();  // Try to find a new gyro
        //         if (_activeGyro == null) return;
        //     }

        //     // localRotation is: (pitch, yaw, roll) in reference block local space
        //     // We need to transform this to the gyro's local space
            
        //     MatrixD refMatrix = _context.Reference.WorldMatrix;
        //     MatrixD gyroMatrix = _activeGyro.WorldMatrix;
            
        //     // Build rotation vector in world space from reference local
        //     // Pitch rotates around Right (X), Yaw around Up (Y), Roll around Forward (Z)
        //     Vector3D worldRotation = 
        //         refMatrix.Right * localRotation.X +    // Pitch
        //         refMatrix.Up * localRotation.Y +       // Yaw
        //         refMatrix.Forward * localRotation.Z;   // Roll
            
        //     // Transform to gyro local space using dot products
        //     double gyroPitch = Vector3D.Dot(worldRotation, gyroMatrix.Right);
        //     double gyroYaw = Vector3D.Dot(worldRotation, gyroMatrix.Up);
        //     double gyroRoll = Vector3D.Dot(worldRotation, gyroMatrix.Forward);

        //     _activeGyro.GyroOverride = true;
        //     _activeGyro.Pitch = (float)gyroPitch;
        //     _activeGyro.Yaw = (float)-gyroYaw;    // SE has inverted yaw
        //     _activeGyro.Roll = (float)-gyroRoll;  // SE has inverted roll
        // }

        private void ReleaseGyros()
        {
            foreach (var gyro in _gyros)
            {
                gyro.GyroOverride = false;
                gyro.Pitch = 0;
                gyro.Yaw = 0;
                gyro.Roll = 0;
            }
        }

        public void Shutdown()
        {
            ReleaseGyros();
            _status = "Shutdown";
        }
    }
}
