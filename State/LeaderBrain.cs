using System.Collections.Generic;
using System.Text;
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
        private IMyBroadcastListener _formationRegisterListener;
        private double _lastCleanupTime;
        private const double BROADCAST_INTERVAL = 0.1;  // 10 Hz broadcast rate
        private const double CLEANUP_INTERVAL = 5.0;    // Cleanup every 5 seconds
        private const double FORMATION_BROADCAST_INTERVAL = 1.0;
        private const double FORMATION_TIMEOUT_SECONDS = 10.0;

        private Dictionary<MyDetectedEntityInfo, float> _detectedEnemies = new Dictionary<MyDetectedEntityInfo, float>();

        // Docking pad management
        private DockingPadManager _dockingPadManager;
        private Program.WcPbApi _wcApi;
        private bool _hasWeaponCore;
        private DroneHardware _hardware;
        private TacticalSnapshot _tacticalSnapshot;
        private double _lastFlareTime;
        private const double FLARE_COOLDOWN_SECONDS = 1.0;
        private double _lastFormationBroadcastTime;
        private readonly Dictionary<long, FormationSlot> _formationRoster = new Dictionary<long, FormationSlot>();
        private readonly List<long> _formationOrder = new List<long>();
        public DockingPadManager DockingPads { get { return _dockingPadManager; } }
        private readonly List<MyDetectedEntityInfo> _obstructionCache = new List<MyDetectedEntityInfo>();

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
            if (context.IGC != null)
            {
                _dockingRequestListener = context.IGC.RegisterBroadcastListener(dockingChannel);
            }
            if (context.IGC != null)
            {
                string formationChannel = context.Config.IGCChannel + "_FORM";
                _formationRegisterListener = context.IGC.RegisterBroadcastListener(formationChannel);
            }

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

            _tacticalSnapshot = new TacticalSnapshot();
            _hardware = DroneHardware.Capture(context.GridTerminalSystem, context.GridId, context.Reference, _wcApi);

            _status = "Initialized";
            if (context.IGC != null)
            {
                context.Echo?.Invoke($"[{Name}] Listening for docking requests on: {dockingChannel}");
            }
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

                // TEMP: Debug LCD obstructions list from WC API
                UpdateObstructionDebug();

                UpdateLeaderFlares();
                ProcessFormationRegistrations();
                CleanupFormationRoster();
                BroadcastFormationAssignments();

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

            // Use grid center for formation anchor (more predictable than cockpit position)
            var gridCenter = _context.Reference.CubeGrid.WorldVolume.Center;

            var message = new LeaderStateMessage
            {
                EntityId = _context.GridId,
                GridName = _context.Me.CubeGrid.CustomName,
                Position = gridCenter,
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
                
                // Only return the player-selected focus target
                // Don't auto-select - let the player control targeting
                if (focus.HasValue && focus.Value.EntityId != 0)
                {
                    return focus.Value.EntityId;
                }
                
                return 0;
            }
            catch
            {
                return 0;
            }
        }

        private void ProcessDockingRequests()
        {
            if (_dockingRequestListener == null || _context.IGC == null)
                return;

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

        private void UpdateObstructionDebug()
        {
            if (_wcApi == null || _context == null || _context.GridTerminalSystem == null)
                return;

            var debugBlock = _context.GridTerminalSystem.GetBlockWithName("Debug") as IMyTextPanel;
            if (debugBlock == null)
                return;

            _obstructionCache.Clear();
            _wcApi.GetObstructions(_context.Me, _obstructionCache);

            var output = new StringBuilder();
            output.Append("Obstructions: ");
            output.Append(_obstructionCache.Count);

            for (int i = 0; i < _obstructionCache.Count; i++)
            {
                output.Append("\n- ");
                output.Append(_obstructionCache[i].Name);
                output.Append(" - ");
                output.Append(_obstructionCache[i].EntityId);
            }

            debugBlock.WriteText(output.ToString(), false);
        }

        private void UpdateLeaderFlares()
        {
            if (_context == null || _context.TacticalCoordinator == null || _tacticalSnapshot == null)
                return;

            if (_hardware == null || _hardware.FlareBlocks == null || _hardware.FlareBlocks.Count == 0)
                return;

            if (_wcApi == null)
                return;

            _context.TacticalCoordinator.UpdateSnapshot(_tacticalSnapshot, _context.GridId, 0, _context.GameTime);
            if (_tacticalSnapshot.ProjectileCount <= 0)
                return;

            if (_context.GameTime - _lastFlareTime < FLARE_COOLDOWN_SECONDS)
                return;

            FireFlares();
            _lastFlareTime = _context.GameTime;
        }

        private void FireFlares()
        {
            for (int i = 0; i < _hardware.FlareBlocks.Count; i++)
            {
                var block = _hardware.FlareBlocks[i];
                if (block != null && block.IsFunctional)
                {
                    _wcApi.FireWeaponOnce(block, true, 0);
                }
            }
        }

        private void ProcessFormationRegistrations()
        {
            if (_formationRegisterListener == null)
                return;

            while (_formationRegisterListener.HasPendingMessage)
            {
                var msg = _formationRegisterListener.AcceptMessage();
                var data = msg.Data as string;
                if (data == null)
                    continue;

                FormationRegisterMessage register;
                if (!FormationRegisterMessage.TryParse(data, out register))
                    continue;

                if (register.DroneEntityId == 0 || register.DroneEntityId == _context.GridId)
                    continue;

                FormationSlot slot;
                if (!_formationRoster.TryGetValue(register.DroneEntityId, out slot))
                {
                    slot = new FormationSlot
                    {
                        DroneEntityId = register.DroneEntityId,
                        DroneGridName = register.DroneGridName
                    };
                    _formationRoster[register.DroneEntityId] = slot;
                    _formationOrder.Add(register.DroneEntityId);
                }

                slot.LastSeen = _context.GameTime;
            }
        }

        private void CleanupFormationRoster()
        {
            if (_formationRoster.Count == 0)
                return;

            for (int i = _formationOrder.Count - 1; i >= 0; i--)
            {
                long droneId = _formationOrder[i];
                FormationSlot slot;
                if (!_formationRoster.TryGetValue(droneId, out slot))
                {
                    _formationOrder.RemoveAt(i);
                    continue;
                }

                if (_context.GameTime - slot.LastSeen > FORMATION_TIMEOUT_SECONDS)
                {
                    _formationRoster.Remove(droneId);
                    _formationOrder.RemoveAt(i);
                }
            }
        }

        private void BroadcastFormationAssignments()
        {
            if (_context.IGC == null)
                return;

            if (_formationOrder.Count == 0)
                return;

            if (_context.GameTime - _lastFormationBroadcastTime < FORMATION_BROADCAST_INTERVAL)
                return;

            _lastFormationBroadcastTime = _context.GameTime;
            int count = _formationOrder.Count;
            string channel = _context.Config.IGCChannel + "_FORM";

            for (int i = 0; i < count; i++)
            {
                var message = new FormationAssignmentMessage
                {
                    DroneEntityId = _formationOrder[i],
                    FormationIndex = i,
                    FormationCount = count,
                    Timestamp = _context.GameTime
                };
                _context.IGC.SendBroadcastMessage(channel, message.Serialize());
            }
        }

        public void Shutdown()
        {
            _status = "Shutdown";
            // Nothing to clean up for leader
        }

        private class FormationSlot
        {
            public long DroneEntityId;
            public string DroneGridName;
            public double LastSeen;
        }
    }
}
