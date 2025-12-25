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

        // Hardware controllers
        private GyroController _gyroController;

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

            _status = $"Listening on: {_context.Config.IGCChannel} (Gyros: {_gyroController.GyroCount})";
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
                    _gyroController.Release();
                    
                    // Wait for reacquisition
                    while (!_hasLeaderContact)
                    {
                        ProcessMessages();
                        yield return true;
                    }
                    _context.Echo?.Invoke($"[{Name}] Leader reacquired: {_lastLeaderState.GridName}");
                }

                // Update gyro controller timing and reference
                _gyroController.SetDeltaTime(_context.DeltaTime);
                _gyroController.SetReference(_context.Reference);

                // Face the leader
                _gyroController.LookAt(_lastLeaderState.Position);

                // Calculate distance for status
                Vector3D myPosition = _context.Reference.GetPosition();
                Vector3D toLeader = _lastLeaderState.Position - myPosition;
                double distance = toLeader.Length();
                
                // Get angle from gyro controller
                double angleOff = _gyroController.TotalError * 180.0 / Math.PI;
                
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

        public void Shutdown()
        {
            _gyroController?.Release();
            _status = "Shutdown";
        }
    }
}
