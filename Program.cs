using Sandbox.ModAPI.Ingame;
using System;
using System.Collections.Generic;
using System.Linq;
using VRageMath;

namespace IngameScript
{
    public partial class Program : MyGridProgram
    {
        // === Core State ===
        private DroneConfig _config;
        private IBrain _activeBrain;
        private IEnumerator<bool> _brainStateMachine;
        private BrainContext _context;
        private bool _refHackMode;
        private List<BrainEntry> _refHackBrains = new List<BrainEntry>();
        private LocalStateBus _localStateBus;
        private List<ICommandBus> _refHackCommandBuses = new List<ICommandBus>();
        private DockingPadManager _localDockingManager;

        // === Hardware ===
        private IMyShipController _reference;

        // === Tactical ===
        private TacticalCoordinator _tacticalCoordinator;
        private TacticalSnapshot _tacticalSnapshot;
        private const string REFHACK_CONNECTOR_KEYWORD = "DronePad";

        // === Timing ===
        private double _lastRunTime;
        private const double TICKS_PER_SECOND_1 = 1.0;
        private const double TICKS_PER_SECOND_10 = 1.0 / 6.0;   // ~0.167s
        private const double TICKS_PER_SECOND_100 = 1.0 / 60.0; // ~0.0167s

        public Program()
        {
            // Parse configuration
            _config = DroneConfig.Parse(Me.CustomData);

            // If no config exists, generate template
            if (string.IsNullOrWhiteSpace(Me.CustomData))
            {
                Me.CustomData = DroneConfig.GenerateTemplate();
                Echo("Generated default configuration in Custom Data.");
                Echo("Please configure and recompile.");
                return;
            }

            // Find reference block (cockpit or remote control)
            _reference = FindReferenceBlock();
            if (_reference == null)
            {
                Echo("ERROR: No cockpit or remote control found!");
                return;
            }

            // Build shared context
            _context = new BrainContext
            {
                GridTerminalSystem = GridTerminalSystem,
                Me = Me,
                IGC = IGC,
                Config = _config,
                Reference = _reference,
                GridId = _reference.CubeGrid.EntityId,
                CommandBus = new IgcCommandBus(IGC, _config.IGCChannel + "_COMMAND"),
                LeaderStateBus = new IgcStateBus(IGC, _config.IGCChannel),
                Echo = Echo,
                GameTime = 0,
                DeltaTime = GetDeltaTime()
            };

            _tacticalCoordinator = new TacticalCoordinator(Me, Echo);
            _tacticalSnapshot = new TacticalSnapshot();
            _context.TacticalCoordinator = _tacticalCoordinator;
            _context.SharedTacticalSnapshot = _tacticalSnapshot;

            if (_config.RefHackMode)
            {
                if (_config.Role != GridRole.Leader)
                {
                    Echo("RefHack mode requires Role=Leader, falling back to normal mode.");
                    _refHackMode = false;
                    SelectBrain();
                }
                else
                {
                    _refHackMode = true;
                    InitializeRefHackBrains();
                }
            }
            else
            {
                // Select and initialize brain based on role
                SelectBrain();
            }

            // Set update frequency
            switch (_config.UpdateFrequency)
            {
                case 1:
                    Runtime.UpdateFrequency = UpdateFrequency.Update1;
                    break;
                case 100:
                    Runtime.UpdateFrequency = UpdateFrequency.Update100;
                    break;
                default:
                    Runtime.UpdateFrequency = UpdateFrequency.Update10;
                    break;
            }

            Echo($"Initialized as {_config.Role}");
            Echo($"Channel: {_config.IGCChannel}");
        }

        public void Save()
        {
            // Future: Save state for persistence across reloads
        }

        public void Main(string argument, UpdateType updateSource)
        {
            // Calculate timing
            double currentTime = Runtime.TimeSinceLastRun.TotalSeconds + _lastRunTime;
            _context.DeltaTime = Runtime.TimeSinceLastRun.TotalSeconds;
            _context.GameTime = currentTime;
            _lastRunTime = currentTime;

            // Handle commands
            if (!string.IsNullOrEmpty(argument))
            {
                HandleCommand(argument);
            }

            if (_refHackMode)
            {
                RunRefHackBrains();
            }
            else
            {
                // Run brain state machine
                if (_brainStateMachine != null)
                {
                    try
                    {
                        if (!_brainStateMachine.MoveNext() || !_brainStateMachine.Current)
                        {
                            Echo($"Brain '{_activeBrain?.Name}' completed or failed.");
                            _brainStateMachine.Dispose();
                            _brainStateMachine = null;
                        }
                    }
                    catch (Exception ex)
                    {
                        Echo($"Brain error: {ex.Message}");
                        _brainStateMachine?.Dispose();
                        _brainStateMachine = null;
                    }
                }

                var droneBrain = _activeBrain as DroneBrain;
                droneBrain?.Debug?.UpdatePerfStats(Runtime.LastRunTimeMs, _context.GameTime);
            }

            // Status display
            //DisplayStatus();
        }

        private void SelectBrain()
        {
            // Shutdown existing brain
            if (_activeBrain != null)
            {
                _activeBrain.Shutdown();
                _brainStateMachine?.Dispose();
            }

            // Create appropriate brain based on role
            switch (_config.Role)
            {
                case GridRole.Leader:
                    _activeBrain = new LeaderBrain();
                    break;
                case GridRole.Drone:
                default:
                    _activeBrain = new DroneBrain()
                    {
                        PB = Me
                    };
                    break;
            }

            // Initialize and start
            _activeBrain.Initialize(_context);
            _brainStateMachine = _activeBrain.Run();
        }

        private void InitializeRefHackBrains()
        {
            _refHackBrains.Clear();
            _refHackCommandBuses.Clear();

            _localStateBus = new LocalStateBus();

            _context.IGC = null;
            _context.CommandBus = null;
            _context.LeaderStateBus = _localStateBus;
            _context.LocalDockingManager = null;

            var leaderBrain = new LeaderBrain { PB = Me };
            AddRefHackBrain(leaderBrain, _context);
            _activeBrain = leaderBrain;
            _localDockingManager = leaderBrain.DockingPads;

            var droneContexts = DiscoverRefHackDrones();
            if (droneContexts.Count == 0)
            {
                Echo("[RefHack] No docked drones detected on DronePad connectors.");
            }

            for (int i = 0; i < droneContexts.Count; i++)
            {
                var droneBrain = new DroneBrain { PB = Me };
                AddRefHackBrain(droneBrain, droneContexts[i]);
            }
        }

        private void AddRefHackBrain(IBrain brain, BrainContext context)
        {
            brain.Initialize(context);
            _refHackBrains.Add(new BrainEntry
            {
                Brain = brain,
                Context = context,
                StateMachine = brain.Run()
            });
        }

        private List<BrainContext> DiscoverRefHackDrones()
        {
            var contexts = new List<BrainContext>();
            var connectors = new List<IMyShipConnector>();
            GridTerminalSystem.GetBlocksOfType(connectors, c =>
                c.CubeGrid.EntityId == _context.GridId
                && c.CustomName.IndexOf(REFHACK_CONNECTOR_KEYWORD, StringComparison.OrdinalIgnoreCase) >= 0);

            var seen = new HashSet<long>();

            for (int i = 0; i < connectors.Count; i++)
            {
                var connector = connectors[i];
                if (connector == null || connector.Status != MyShipConnectorStatus.Connected)
                    continue;
                if (connector.OtherConnector == null || connector.OtherConnector.CubeGrid == null)
                    continue;

                long gridId = connector.OtherConnector.CubeGrid.EntityId;
                if (gridId == 0 || gridId == _context.GridId || seen.Contains(gridId))
                    continue;

                seen.Add(gridId);

                var droneReference = FindReferenceBlockForGrid(gridId);
                if (droneReference == null)
                {
                    Echo($"[RefHack] No ship controller found for grid {gridId}");
                    continue;
                }

                var hardware = DroneHardware.Capture(GridTerminalSystem, gridId, droneReference);
                if (hardware == null || !hardware.IsValid())
                {
                    Echo($"[RefHack] Hardware capture failed for grid {gridId}");
                    continue;
                }

                Vector3D? stationOffsetOverride = BuildStationOffsetFromPad(connector);

                var commandBus = new LocalCommandBus();
                _refHackCommandBuses.Add(commandBus);

                var droneContext = new BrainContext
                {
                    GridTerminalSystem = GridTerminalSystem,
                    Me = Me,
                    IGC = null,
                    Config = _config,
                    Reference = droneReference,
                    GridId = gridId,
                    Hardware = hardware,
                    CommandBus = commandBus,
                    LeaderStateBus = _localStateBus,
                    LocalDockingManager = _localDockingManager,
                    StationOffsetOverride = stationOffsetOverride,
                    TacticalCoordinator = _tacticalCoordinator,
                    SharedTacticalSnapshot = _tacticalSnapshot,
                    Echo = Echo,
                    GameTime = _context.GameTime,
                    DeltaTime = _context.DeltaTime
                };

                contexts.Add(droneContext);
            }

            return contexts;
        }

        private void RunRefHackBrains()
        {
            for (int i = 0; i < _refHackBrains.Count; i++)
            {
                _refHackBrains[i].Context.GameTime = _context.GameTime;
                _refHackBrains[i].Context.DeltaTime = _context.DeltaTime;
            }

            for (int i = _refHackBrains.Count - 1; i >= 0; i--)
            {
                var entry = _refHackBrains[i];
                try
                {
                    if (!entry.StateMachine.MoveNext() || !entry.StateMachine.Current)
                    {
                        Echo($"Brain '{entry.Brain?.Name}' completed or failed.");
                        entry.StateMachine.Dispose();
                        _refHackBrains.RemoveAt(i);
                    }
                }
                catch (Exception ex)
                {
                    Echo($"Brain error: {ex.Message}");
                    entry.StateMachine.Dispose();
                    _refHackBrains.RemoveAt(i);
                }
            }

            for (int i = 0; i < _refHackBrains.Count; i++)
            {
                var droneBrain = _refHackBrains[i].Brain as DroneBrain;
                if (droneBrain != null)
                {
                    droneBrain.Debug?.UpdatePerfStats(Runtime.LastRunTimeMs, _refHackBrains[i].Context.GameTime);
                }
            }
        }

        private void HandleCommand(string argument)
        {
            string cmd = argument.ToUpperInvariant().Trim();

            switch (cmd)
            {
                // case "RELOAD":
                //     _config = DroneConfig.Parse(Me.CustomData);
                //     _context.Config = _config;
                //     SelectBrain();
                //     Echo("Configuration reloaded.");
                //     break;

                // case "STATUS":
                //     Echo($"Role: {_config.Role}");
                //     Echo($"Brain: {_activeBrain?.Name ?? "None"}");
                //     Echo($"Status: {_activeBrain?.Status ?? "N/A"}");
                //     break;

                // case "STOP":
                //     _activeBrain?.Shutdown();
                //     _brainStateMachine?.Dispose();
                //     _brainStateMachine = null;
                //     Runtime.UpdateFrequency = UpdateFrequency.None;
                //     Echo("Stopped.");
                //     break;

                // case "START":
                //     SelectBrain();
                //     Runtime.UpdateFrequency = UpdateFrequency.Update10;
                //     Echo("Started.");
                //     break;

                case "DOCK":
                    HandleDockCommand();
                    break;

                case "ESCORT":
                    HandleEscortCommand();
                    break;
                case "REFRESH_DOCKS":
                    RefreshDockingData();
                    break;

                default:
                    // Check for leader commands (e.g., "DOCK_ALL", "ESCORT_ALL")
                    if (HandleLeaderCommand(cmd))
                    {
                        // Command was handled
                    }
                    else
                    {
                        Echo($"Unknown command: {argument}");
                        Echo("Available commands: RELOAD, STATUS, STOP, START, DOCK, FASTDOCK, ESCORT");
                        if (_config.Role == GridRole.Leader)
                        {
                            Echo("Leader commands: DOCK_ALL, ESCORT_ALL, FORMUP_ALL, REFRESH_DOCKS");
                        }
                    }
                    break;
            }
        }

        private void HandleDockCommand()
        {
            if (_config.Role != GridRole.Drone)
            {
                Echo("ERROR: DOCK command only works for drones.");
                return;
            }

            var droneBrain = _activeBrain as DroneBrain;
            if (droneBrain == null)
            {
                Echo("ERROR: Drone brain not active.");
                return;
            }

            // Switch to DockDirective
            droneBrain.SetDirective(new DockDirective());
            Echo("Switching to DOCK directive.");
            Echo("Requesting docking pad from leader...");
        }

        private void HandleEscortCommand()
        {
            if (_config.Role != GridRole.Drone)
            {
                Echo("ERROR: ESCORT command only works for drones.");
                return;
            }

            var droneBrain = _activeBrain as DroneBrain;
            if (droneBrain == null)
            {
                Echo("ERROR: Drone brain not active.");
                return;
            }

            // Switch back to EscortDirective
            droneBrain.SetDirective(new EscortDirective());
            Echo("Switching to ESCORT directive.");
            Echo("Resuming formation flying...");
        }

        private void RefreshDockingData()
        {
            if (_refHackMode)
            {
                for (int i = 0; i < _refHackBrains.Count; i++)
                {
                    var leaderBrain = _refHackBrains[i].Brain as LeaderBrain;
                    if (leaderBrain != null && leaderBrain.DockingPads != null)
                    {
                        leaderBrain.DockingPads.RefreshConnectors();
                    }

                    var droneBrain = _refHackBrains[i].Brain as DroneBrain;
                    if (droneBrain != null && droneBrain.Context != null && droneBrain.Context.Hardware != null)
                    {
                        droneBrain.Context.Hardware.RefreshConnectors(GridTerminalSystem);
                    }
                }

                Echo("RefHack docking data refreshed.");
                return;
            }

            var activeLeader = _activeBrain as LeaderBrain;
            if (activeLeader != null && activeLeader.DockingPads != null)
            {
                activeLeader.DockingPads.RefreshConnectors();
                Echo("Leader docking pads refreshed.");
                return;
            }

            var activeDrone = _activeBrain as DroneBrain;
            if (activeDrone != null && activeDrone.Context != null && activeDrone.Context.Hardware != null)
            {
                activeDrone.Context.Hardware.RefreshConnectors(GridTerminalSystem);
                Echo("Drone connectors refreshed.");
            }
        }

        /// <summary>
        /// Handles leader-specific commands that broadcast to drones.
        /// </summary>
        /// <returns>True if the command was recognized and handled</returns>
        private bool HandleLeaderCommand(string cmd)
        {
            if (_config.Role != GridRole.Leader)
            {
                return false;
            }

            DroneCommand droneCommand = DroneCommand.None;
            bool broadcast = false;

            if (cmd == "DOCK_ALL")
            {
                droneCommand = DroneCommand.Dock;
                broadcast = true;
            }
            else if (cmd == "FASTDOCK")
            {
                droneCommand = DroneCommand.FastDock;
                broadcast = true;
            }
            else if (cmd == "ESCORT_ALL")
            {
                droneCommand = DroneCommand.Escort;
                broadcast = true;
            }
            else if (cmd == "FORMUP_ALL")
            {
                droneCommand = DroneCommand.FormUp;
                broadcast = true;
            }
            else if (cmd == "FOLLOWME_ALL")
            {
                droneCommand = DroneCommand.FollowMe;
                broadcast = true;
            }
            else if (cmd == "SCATTER_ALL")
            {
                droneCommand = DroneCommand.Scatter;
                broadcast = true;
            }
            else if (cmd == "ATTACK_ALL")
            {
                droneCommand = DroneCommand.Attack;
                broadcast = true;
            }
            else
            {
                return false; // Command not recognized
            }

            if (broadcast)
            {
                BroadcastDroneCommand(droneCommand, 0); // 0 = broadcast to all
                Echo($"Broadcasting {droneCommand} command to all drones...");
                return true;
            }

            return false;
        }

        /// <summary>
        /// Broadcasts a command message to drones via IGC.
        /// </summary>
        /// <param name="command">The command to send</param>
        /// <param name="targetDroneId">Target drone entity ID (0 for broadcast)</param>
        private void BroadcastDroneCommand(DroneCommand command, long targetDroneId)
        {
            var message = new DroneCommandMessage
            {
                TargetDroneId = targetDroneId,
                Command = command,
                Timestamp = _context.GameTime
            };

            if (_refHackMode && _refHackCommandBuses.Count > 0)
            {
                for (int i = 0; i < _refHackCommandBuses.Count; i++)
                {
                    _refHackCommandBuses[i].Publish(message);
                }
                return;
            }

            if (_context.CommandBus != null)
            {
                _context.CommandBus.Publish(message);
                return;
            }

            string channel = _config.IGCChannel + "_COMMAND";
            IGC.SendBroadcastMessage(channel, message.Serialize());
        }

        private void DisplayStatus()
        {
            Echo($"=== {_config.Role} ===");
            Echo($"Brain: {_activeBrain?.Name ?? "None"}");
            Echo($"Status: {_activeBrain?.Status ?? "N/A"}");
            Echo($"Time: {_context.GameTime:F1}s");
        }

        private IMyShipController FindReferenceBlock()
        {
            return FindReferenceBlockForGrid(Me.CubeGrid.EntityId);
        }

        private IMyShipController FindReferenceBlockForGrid(long gridId)
        {
            var controllers = new List<IMyShipController>();
            GridTerminalSystem.GetBlocksOfType(controllers, c => c.CubeGrid.EntityId == gridId);

            // Prefer cockpits that are set as main
            for (int i = 0; i < controllers.Count; i++)
            {
                var cockpit = controllers[i] as IMyCockpit;
                if (cockpit != null && cockpit.IsMainCockpit)
                    return cockpit;
            }

            // Then any cockpit
            for (int i = 0; i < controllers.Count; i++)
            {
                if (controllers[i] is IMyCockpit)
                    return controllers[i];
            }

            // Then remote control
            for (int i = 0; i < controllers.Count; i++)
            {
                if (controllers[i] is IMyRemoteControl)
                    return controllers[i];
            }

            // Any controller
            return controllers.FirstOrDefault();
        }

        private Vector3D? BuildStationOffsetFromPad(IMyShipConnector padConnector)
        {
            if (padConnector == null || _context == null || _context.Reference == null)
                return null;

            Vector3D padRightWorld = padConnector.WorldMatrix.Right;
            Vector3D padUpWorld = padConnector.WorldMatrix.Up;
            Vector3D padForwardWorld = padConnector.WorldMatrix.Forward;

            if (padRightWorld.LengthSquared() < 0.001 || padUpWorld.LengthSquared() < 0.001 || padForwardWorld.LengthSquared() < 0.001)
                return null;

            Vector3D configOffset = _config.StationOffset;
            if (configOffset.LengthSquared() < 0.01)
                return null;

            MatrixD leaderMatrix = _context.Reference.WorldMatrix;
            // Use connector Up as pseudo-forward for offsets.
            Vector3D worldOffset =
                padRightWorld * configOffset.X +
                padForwardWorld * configOffset.Y +
                padUpWorld * configOffset.Z;

            Vector3D leaderPos = _context.Reference.GetPosition();
            Vector3D formationWorldPos = padConnector.GetPosition() + worldOffset;
            Vector3D worldOffsetFromLeader = formationWorldPos - leaderPos;
            Vector3D localOffset = Vector3D.TransformNormal(worldOffsetFromLeader, MatrixD.Transpose(leaderMatrix));
            Echo($"[RefHack] Station offset from pad '{padConnector.CustomName}': {localOffset.X:F1},{localOffset.Y:F1},{localOffset.Z:F1}");
            return localOffset;
        }

        private class BrainEntry
        {
            public IBrain Brain;
            public BrainContext Context;
            public IEnumerator<bool> StateMachine;
        }

        private double GetDeltaTime()
        {
            switch (_config.UpdateFrequency)
            {
                case 1: return TICKS_PER_SECOND_1;
                case 100: return TICKS_PER_SECOND_100;
                default: return TICKS_PER_SECOND_10;
            }
        }
    }
}
