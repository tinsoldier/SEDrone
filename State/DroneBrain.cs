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
    /// - Track leader position and orientation
    /// - Orient to face destination (when out of formation) or match leader (when in formation)
    /// - Maintain formation position relative to leader
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

        // Formation state
        private bool _inFormation;  // Hysteresis state for orientation mode
        private const double FORMATION_ENTER_THRESHOLD = 1.0;  // Multiplier of StationRadius to enter formation mode
        private const double FORMATION_EXIT_THRESHOLD = 2.5;   // Multiplier of StationRadius to exit formation mode

        // Hardware controllers
        private GyroController _gyroController;
        private ThrusterController _thrusterController;

        public void Initialize(BrainContext context)
        {
            _context = context;
            
            // Register for IGC broadcasts
            _listener = _context.IGC.RegisterBroadcastListener(_context.Config.IGCChannel);

            // Initialize gyro controller
            var gyros = new List<IMyGyro>();
            _context.GridTerminalSystem.GetBlocksOfType(gyros, g => g.CubeGrid == _context.Me.CubeGrid);
            _gyroController = new GyroController(
                _context.Reference, 
                gyros, 
                _context.Config.OrientationPID,
                _context.Echo
            );

            // Initialize thruster controller
            var thrusters = new List<IMyThrust>();
            _context.GridTerminalSystem.GetBlocksOfType(thrusters, t => t.CubeGrid == _context.Me.CubeGrid);
            _thrusterController = new ThrusterController(
                _context.Reference,
                thrusters,
                _context.Config.ThrustConfig,
                _context.Echo
            );

            _status = $"Listening: {_context.Config.IGCChannel} (G:{_gyroController.GyroCount} T:{_thrusterController.ThrusterCount})";
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

            // === PHASE 2: Track and follow leader ===
            _context.Echo?.Invoke($"[{Name}] Leader acquired: {_lastLeaderState.GridName}");

            while (true)
            {
                ProcessMessages();

                // Check for leader timeout
                if (_context.GameTime - _lastContactTime > CONTACT_TIMEOUT)
                {
                    _hasLeaderContact = false;
                    _status = "Leader lost! Searching...";
                    _gyroController.Release();
                    _thrusterController.Release();
                    
                    // Wait for reacquisition
                    while (!_hasLeaderContact)
                    {
                        ProcessMessages();
                        yield return true;
                    }
                    _context.Echo?.Invoke($"[{Name}] Leader reacquired: {_lastLeaderState.GridName}");
                }

                // Update controllers
                UpdateControllers();

                // Calculate formation position (where we want to be)
                Vector3D formationPosition = CalculateFormationPosition();
                
                // Calculate distance to formation for behavior decisions
                Vector3D myPosition = _context.Reference.GetPosition();
                double distanceToFormation = Vector3D.Distance(myPosition, formationPosition);
                
                // Update formation state with hysteresis
                UpdateFormationState(distanceToFormation);
                
                // Calculate desired velocity (match leader + correction toward formation)
                Vector3D desiredVelocity = CalculateDesiredVelocity(formationPosition);

                // Orientation behavior:
                // - In formation: match leader's compass heading only (stay level to gravity)
                // - Out of formation: face toward formation waypoint
                if (_inFormation)
                {
                    // Match leader's yaw only - project forward onto horizontal plane
                    // This keeps the drone level regardless of leader's pitch/roll
                    Vector3D gravity = _context.Reference.GetNaturalGravity();
                    Vector3D leaderForward = _lastLeaderState.Forward;
                    
                    // Fallback right vector in case leader is pointing straight up/down
                    Vector3D leaderRight = Vector3D.Cross(_lastLeaderState.Forward, _lastLeaderState.Up);
                    
                    Vector3D horizontalForward = VectorMath.ProjectOntoHorizontalPlane(leaderForward, gravity, leaderRight);
                    
                    if (horizontalForward.LengthSquared() > 0.001)
                    {
                        _gyroController.OrientToward(horizontalForward);
                    }
                    // If degenerate, hold current orientation (gyro continues last command)
                }
                else
                {
                    // Face destination waypoint
                    _gyroController.LookAt(formationPosition);
                }

                // Move toward formation position
                _thrusterController.MoveToward(
                    formationPosition,
                    desiredVelocity,
                    _context.Config.MaxSpeed,
                    _context.Config.PrecisionRadius
                );

                // Update status
                UpdateStatus(formationPosition, distanceToFormation);

                yield return true;
            }
        }

        /// <summary>
        /// Updates the in-formation state with hysteresis to prevent flickering.
        /// </summary>
        private void UpdateFormationState(double distanceToFormation)
        {
            double enterThreshold = _context.Config.StationRadius * FORMATION_ENTER_THRESHOLD;
            double exitThreshold = _context.Config.StationRadius * FORMATION_EXIT_THRESHOLD;
            
            if (_inFormation)
            {
                // Currently in formation - check if we've drifted too far
                if (distanceToFormation > exitThreshold)
                {
                    _inFormation = false;
                }
            }
            else
            {
                // Currently out of formation - check if we've arrived
                if (distanceToFormation < enterThreshold)
                {
                    _inFormation = true;
                }
            }
        }

        /// <summary>
        /// Updates controller references and timing.
        /// </summary>
        private void UpdateControllers()
        {
            // Update timing
            _gyroController.SetDeltaTime(_context.DeltaTime);
            _thrusterController.SetDeltaTime(_context.DeltaTime);
            
            // Update references
            _gyroController.SetReference(_context.Reference);
            _thrusterController.SetReference(_context.Reference);
            
            // Update tilt limit from config
            _gyroController.SetMaxTilt(_context.Config.MaxTiltAngle);
            
            // Update ship mass
            var massData = _context.Reference.CalculateShipMass();
            _thrusterController.SetShipMass(massData.PhysicalMass);
        }

        /// <summary>
        /// Calculates the world-space position where this drone should be.
        /// Transforms the local offset from DroneConfig into world space based on leader orientation.
        /// Also adjusts for terrain - if formation point would be underground, lifts it to maintain clearance.
        /// </summary>
        private Vector3D CalculateFormationPosition()
        {
            // Get leader orientation matrix
            // The offset is in leader-local coordinates:
            // X = right (+) / left (-)
            // Y = up (+) / down (-)
            // Z = forward (+) / backward (-)
            
            Vector3D leaderRight = Vector3D.Cross(_lastLeaderState.Forward, _lastLeaderState.Up);
            
            // Transform offset from leader-local to world space
            Vector3D offset = _context.Config.StationOffset;
            Vector3D worldOffset = 
                leaderRight * offset.X +
                _lastLeaderState.Up * offset.Y +
                _lastLeaderState.Forward * offset.Z;
            
            Vector3D rawFormationPosition = _lastLeaderState.Position + worldOffset;
            
            // Terrain clearance adjustment
            // Check if the formation point is too close to or below the terrain
            return AdjustForTerrainClearance(rawFormationPosition);
        }
        
        /// <summary>
        /// Adjusts a position upward if it would be below minimum terrain clearance.
        /// Uses the drone's current position to sample terrain elevation.
        /// </summary>
        private Vector3D AdjustForTerrainClearance(Vector3D targetPosition)
        {
            // Get gravity direction - we need this to know which way is "up"
            Vector3D gravity = _context.Reference.GetNaturalGravity();
            if (gravity.LengthSquared() < 0.1)
            {
                // No gravity = no terrain to worry about
                return targetPosition;
            }
            
            Vector3D up = -Vector3D.Normalize(gravity);
            
            // Get our current elevation above terrain
            double currentElevation;
            if (!_context.Reference.TryGetPlanetElevation(MyPlanetElevation.Surface, out currentElevation))
            {
                // Can't get elevation - return unadjusted
                return targetPosition;
            }
            
            // Calculate how much higher/lower the target is compared to us
            Vector3D toTarget = targetPosition - _context.Reference.GetPosition();
            double targetHeightDelta = Vector3D.Dot(toTarget, up);
            
            // Estimate target's elevation: our elevation + height difference
            double estimatedTargetElevation = currentElevation + targetHeightDelta;
            
            // Check if target is below minimum clearance
            double minClearance = _context.Config.MinTerrainClearance;
            if (estimatedTargetElevation < minClearance)
            {
                // Lift the target position to maintain minimum clearance
                double liftAmount = minClearance - estimatedTargetElevation;
                return targetPosition + up * liftAmount;
            }
            
            return targetPosition;
        }

        /// <summary>
        /// Calculates the desired velocity for smooth formation flying.
        /// Base velocity matches leader, with correction toward formation position.
        /// Uses velocity dampening to prevent oscillation/overshoot.
        /// </summary>
        private Vector3D CalculateDesiredVelocity(Vector3D formationPosition)
        {
            Vector3D myPosition = _context.Reference.GetPosition();
            Vector3D myVelocity = _context.Reference.GetShipVelocities().LinearVelocity;
            Vector3D toFormation = formationPosition - myPosition;
            double distance = toFormation.Length();
            
            // Start with leader's velocity as base
            Vector3D desired = _lastLeaderState.Velocity;
            
            // If we're very close, just match leader velocity (no correction needed)
            if (distance <= _context.Config.StationRadius)
            {
                return desired;
            }
            
            // Calculate direction to formation
            Vector3D directionToFormation = toFormation / distance;
            
            // Calculate our current velocity relative to the leader
            Vector3D relativeVelocity = myVelocity - _lastLeaderState.Velocity;
            
            // Decompose relative velocity into components:
            // - Toward/away from formation (closing velocity)
            // - Perpendicular to formation direction (lateral drift)
            double closingSpeed = Vector3D.Dot(relativeVelocity, directionToFormation);
            Vector3D lateralVelocity = relativeVelocity - directionToFormation * closingSpeed;
            
            // Calculate target correction speed based on distance
            // This is how fast we WANT to be closing on the formation
            double targetClosingSpeed = CalculateCorrectionSpeed(distance);
            
            // Calculate closing speed error
            // Positive = we need to speed up toward formation
            // Negative = we're going too fast, need to brake
            double closingError = targetClosingSpeed - closingSpeed;
            
            // Apply correction for closing speed error
            // Scale by a gain factor to control responsiveness (lower = smoother, slower)
            double closingGain = 0.5;  // Only correct half the error per tick
            Vector3D closingCorrection = directionToFormation * closingError * closingGain;
            
            // Dampen lateral drift - we want zero lateral velocity relative to formation
            // Apply full dampening for lateral (perpendicular) motion to prevent oscillation
            double lateralGain = 1.0;  // Dampen 100% of lateral velocity
            Vector3D lateralDampening = -lateralVelocity * lateralGain;
            
            // Combine corrections
            desired += closingCorrection + lateralDampening;
            
            // Clamp to max speed
            double speed = desired.Length();
            if (speed > _context.Config.MaxSpeed)
            {
                desired = desired / speed * _context.Config.MaxSpeed;
            }
            
            return desired;
        }

        /// <summary>
        /// Calculates how fast to correct toward formation position based on distance.
        /// </summary>
        private double CalculateCorrectionSpeed(double distance)
        {
            // Within precision radius: scale down correction
            if (distance < _context.Config.PrecisionRadius)
            {
                return _context.Config.ApproachSpeed * (distance / _context.Config.PrecisionRadius);
            }
            
            // Beyond precision radius: full approach speed
            return _context.Config.ApproachSpeed;
        }

        /// <summary>
        /// Updates the status display.
        /// </summary>
        private void UpdateStatus(Vector3D formationPosition, double distToFormation)
        {
            Vector3D myPosition = _context.Reference.GetPosition();
            double distToLeader = Vector3D.Distance(myPosition, _lastLeaderState.Position);
            double speed = _context.Reference.GetShipSpeed();
            double angleOff = _gyroController.TotalError * 180.0 / Math.PI;
            string mode = _inFormation ? "FORM" : "MOVE";
            
            _status = $"{mode} | F:{distToFormation:F0}m L:{distToLeader:F0}m | {speed:F0}m/s {angleOff:F1}°";
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

        public void Shutdown()
        {
            _gyroController?.Release();
            _thrusterController?.Release();
            _status = "Shutdown";
        }
    }
}
