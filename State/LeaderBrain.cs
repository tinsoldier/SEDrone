using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;

namespace IngameScript
{
    /// <summary>
    /// Brain for the leader/commander grid.
    /// Responsibilities:
    /// - Broadcast position over IGC for drones to follow
    /// - Manage docking pad assignments for requesting drones
    /// - (Future) Issue formation commands
    /// - (Future) Coordinate drone behavior
    /// </summary>
    public class LeaderBrain : IBrain
    {
        public string Name { get { return "Leader"; } }
        public string Status { get { return _status; } }

        private string _status = "Initializing";
        private BrainContext _context;
        private IMyBroadcastListener _dockingRequestListener;
        private IMyBroadcastListener _commandListener; // For future use
        private double _lastBroadcastTime;
        private double _lastCleanupTime;
        private const double BROADCAST_INTERVAL = 0.1;  // 10 Hz broadcast rate
        private const double CLEANUP_INTERVAL = 5.0;    // Cleanup every 5 seconds

        // Docking pad management
        private DockingPadManager _dockingPadManager;

        public void Initialize(BrainContext context)
        {
            _context = context;

            // Register listener for docking requests
            string dockingChannel = context.Config.IGCChannel + "_DOCK_REQUEST";
            _dockingRequestListener = context.IGC.RegisterBroadcastListener(dockingChannel);

            // Initialize docking pad manager
            _dockingPadManager = new DockingPadManager(
                context.GridTerminalSystem,
                context.Me,
                context.Reference,
                context.Echo
            );

            _status = "Initialized";
            context.Echo?.Invoke($"[{Name}] Listening for docking requests on: {dockingChannel}");
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

                // Process docking requests
                ProcessDockingRequests();

                // Cleanup stale assignments periodically
                if (_context.GameTime - _lastCleanupTime >= CLEANUP_INTERVAL)
                {
                    _dockingPadManager.CleanupStaleAssignments(_context.GameTime);
                    _lastCleanupTime = _context.GameTime;
                }

                _status = string.Format("Broadcasting | Pads: {0}/{1}",
                    _dockingPadManager.AssignedPadCount,
                    _dockingPadManager.AssignedPadCount + _dockingPadManager.AvailablePadCount);

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

        private void ProcessDockingRequests()
        {
            // Process all pending docking requests
            while (_dockingRequestListener.HasPendingMessage)
            {
                var msg = _dockingRequestListener.AcceptMessage();
                var data = msg.Data as string;
                if (data != null)
                {
                    DockingPadRequest request;
                    if (DockingPadRequest.TryParse(data, out request))
                    {
                        // Process the request
                        DockingPadResponse response = _dockingPadManager.ProcessRequest(request, _context.GameTime);

                        // Send response back on response channel
                        string responseChannel = _context.Config.IGCChannel + "_DOCK_RESPONSE";
                        _context.IGC.SendBroadcastMessage(responseChannel, response.Serialize());
                    }
                }
            }
        }

        public void Shutdown()
        {
            _status = "Shutdown";
            // Nothing to clean up for leader
        }
    }
}
