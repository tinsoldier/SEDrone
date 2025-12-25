using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;

namespace IngameScript
{
    /// <summary>
    /// Brain for the leader/commander grid.
    /// Responsibilities:
    /// - Broadcast position over IGC for drones to follow
    /// - (Future) Issue formation commands
    /// - (Future) Coordinate drone behavior
    /// </summary>
    public class LeaderBrain : IBrain
    {
        public string Name => "Leader";
        public string Status => _status;

        private string _status = "Initializing";
        private BrainContext _context;
        private IMyBroadcastListener _listener;
        private double _lastBroadcastTime;
        private const double BROADCAST_INTERVAL = 0.1;  // 10 Hz broadcast rate

        public void Initialize(BrainContext context)
        {
            _context = context;
            _status = "Initialized";
        }

        public IEnumerator<bool> Run()
        {
            // === PHASE 1: Setup ===
            _status = "Starting broadcast loop";
            _context.Echo?.Invoke($"[{Name}] Broadcasting on channel: {_context.Config.IGCChannel}");

            // Main loop - runs indefinitely
            while (true)
            {
                // Broadcast position at fixed interval
                if (_context.GameTime - _lastBroadcastTime >= BROADCAST_INTERVAL)
                {
                    BroadcastState();
                    _lastBroadcastTime = _context.GameTime;
                }

                _status = $"Broadcasting @ {_context.GameTime:F1}s";
                yield return true;  // Continue next tick
            }
        }

        private void BroadcastState()
        {
            if (_context.Reference == null)
            {
                _status = "ERROR: No reference block";
                return;
            }

            var matrix = _context.Reference.WorldMatrix;
            var velocity = _context.Reference.GetShipVelocities().LinearVelocity;

            var message = new LeaderStateMessage
            {
                EntityId = _context.Me.CubeGrid.EntityId,
                GridName = _context.Me.CubeGrid.CustomName,
                Position = matrix.Translation,
                Velocity = velocity,
                Forward = matrix.Forward,
                Up = matrix.Up,
                Timestamp = _context.GameTime
            };

            // Broadcast to all listeners on the channel
            _context.IGC.SendBroadcastMessage(_context.Config.IGCChannel, message.Serialize());
        }

        public void Shutdown()
        {
            _status = "Shutdown";
            // Nothing to clean up for leader
        }
    }
}
