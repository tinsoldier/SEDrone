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
    /// - Orient to face the leader
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
                
                // Calculate desired velocity (match leader + correction toward formation)
                Vector3D desiredVelocity = CalculateDesiredVelocity(formationPosition);

                // Face the leader (or formation position based on preference)
                _gyroController.LookAt(_lastLeaderState.Position);

                // Move toward formation position
                _thrusterController.MoveToward(
                    formationPosition,
                    desiredVelocity,
                    _context.Config.MaxSpeed,
                    _context.Config.PrecisionRadius
                );

                // Update status
                UpdateStatus(formationPosition);

                yield return true;
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
            
            // Update ship mass
            var massData = _context.Reference.CalculateShipMass();
            _thrusterController.SetShipMass(massData.PhysicalMass);
        }

        /// <summary>
        /// Calculates the world-space position where this drone should be.
        /// Transforms the local offset from DroneConfig into world space based on leader orientation.
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
            
            return _lastLeaderState.Position + worldOffset;
        }

        /// <summary>
        /// Calculates the desired velocity for smooth formation flying.
        /// Base velocity matches leader, with correction toward formation position.
        /// </summary>
        private Vector3D CalculateDesiredVelocity(Vector3D formationPosition)
        {
            Vector3D myPosition = _context.Reference.GetPosition();
            Vector3D toFormation = formationPosition - myPosition;
            double distance = toFormation.Length();
            
            // Start with leader's velocity as base
            Vector3D desired = _lastLeaderState.Velocity;
            
            // Add correction toward formation position
            if (distance > _context.Config.StationRadius)
            {
                // Calculate correction speed based on distance
                double correctionSpeed = CalculateCorrectionSpeed(distance);
                Vector3D correction = Vector3D.Normalize(toFormation) * correctionSpeed;
                desired += correction;
            }
            
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
        private void UpdateStatus(Vector3D formationPosition)
        {
            Vector3D myPosition = _context.Reference.GetPosition();
            double distToFormation = Vector3D.Distance(myPosition, formationPosition);
            double distToLeader = Vector3D.Distance(myPosition, _lastLeaderState.Position);
            double speed = _context.Reference.GetShipSpeed();
            double angleOff = _gyroController.TotalError * 180.0 / Math.PI;
            
            _status = $"{_lastLeaderState.GridName} | F:{distToFormation:F0}m L:{distToLeader:F0}m | {speed:F0}m/s {angleOff:F1}°";
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
