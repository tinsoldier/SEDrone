using System.Collections.Generic;
using System.Linq;
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
        public IMyProgrammableBlock PB { get; set; }

        private string _status = "Initializing";
        private BrainContext _context;
        private IMyBroadcastListener _dockingRequestListener;
        private double _lastCleanupTime;
        private const double BROADCAST_INTERVAL = 0.1;  // 10 Hz broadcast rate
        private const double CLEANUP_INTERVAL = 5.0;    // Cleanup every 5 seconds

        private Dictionary<MyDetectedEntityInfo, float> _detectedEnemies = new Dictionary<MyDetectedEntityInfo, float>();

        // Docking pad management
        private DockingPadManager _dockingPadManager;
        private Program.WcPbApi _wcApi;
        private bool _hasWeaponCore;

        public void Initialize(BrainContext context)
        {
            _context = context;
            if (_context.GridId == 0)
            {
                _context.GridId = _context.Reference != null
                    ? _context.Reference.CubeGrid.EntityId
                    : _context.Me.CubeGrid.EntityId;
            }

            // Register listener for docking requests
            string dockingChannel = context.Config.IGCChannel + "_DOCK_REQUEST";
            _dockingRequestListener = context.IGC.RegisterBroadcastListener(dockingChannel);

            if (_context.CommandBus == null)
            {
                _context.CommandBus = new IgcCommandBus(context.IGC, context.Config.IGCChannel + "_COMMAND");
            }
            if (_context.LeaderStateBus == null)
            {
                _context.LeaderStateBus = new IgcStateBus(context.IGC, context.Config.IGCChannel);
            }

            // Initialize docking pad manager
            _dockingPadManager = new DockingPadManager(
                context.GridTerminalSystem,
                context.Me,
                context.Reference,
                context.Echo
            );

            // Initialize WeaponCore API (optional)
            _wcApi = new Program.WcPbApi();
            try
            {
                _hasWeaponCore = _wcApi.Activate(context.Me);
            }
            catch
            {
                _hasWeaponCore = false;
                _wcApi = null;
            }

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
                // if (_context.GameTime - _lastBroadcastTime >= BROADCAST_INTERVAL)
                // {
                //     BroadcastState();
                //     _lastBroadcastTime = _context.GameTime;
                // }
                BroadcastState();

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
                EntityId = _context.GridId,
                GridName = _context.Me.CubeGrid.CustomName,
                Position = matrix.Translation,
                Velocity = velocity,
                Forward = matrix.Forward,
                Up = matrix.Up,
                Timestamp = _context.GameTime,
                TargetEntityId = GetFocusedTargetId()
            };

            if (_context.LeaderStateBus != null)
            {
                _context.LeaderStateBus.Publish(message);
                return;
            }

            // Broadcast to all listeners on the channel
            _context.IGC.SendBroadcastMessage(_context.Config.IGCChannel, message.Serialize());
        }

        private long GetFocusedTargetId()
        {
            if (!_hasWeaponCore || _wcApi == null)
                return 0;

            try
            {
                var focus = _wcApi.GetAiFocus(_context.GridId, 0);
                
                //if focus has a valid value, return that, otherwise call GetSortedThreats and return the top threat, set that as the focus
                if (focus.HasValue && focus.Value.EntityId != 0)
                {
                    _context.Echo?.Invoke($"(Leader) Current focus targetId={focus.Value.EntityId}");
                    return focus.Value.EntityId;
                }
                else
                {
                    _context.Echo?.Invoke($"(Leader) Scanning for threats");
                    _wcApi.GetSortedThreats(PB, _detectedEnemies);
                    if (_detectedEnemies.Count > 0)
                    {
                        var entityId = _detectedEnemies.Keys.First().EntityId;
                        _wcApi.SetAiFocus(_context.Me, entityId, 0);
                        return entityId;
                    }
                    else
                        return 0;
                }
            }
            catch
            {
                return 0;
            }
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
