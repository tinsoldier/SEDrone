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

        // === Hardware ===
        private IMyShipController _reference;

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
                Echo = Echo,
                GameTime = 0,
                DeltaTime = GetDeltaTime()
            };

            // Select and initialize brain based on role
            SelectBrain();

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

            // Status display
            DisplayStatus();
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
                    _activeBrain = new DroneBrain();
                    break;
            }

            // Initialize and start
            _activeBrain.Initialize(_context);
            _brainStateMachine = _activeBrain.Run();
        }

        private void HandleCommand(string argument)
        {
            string cmd = argument.ToUpperInvariant().Trim();

            switch (cmd)
            {
                case "RELOAD":
                    _config = DroneConfig.Parse(Me.CustomData);
                    _context.Config = _config;
                    SelectBrain();
                    Echo("Configuration reloaded.");
                    break;

                case "STATUS":
                    Echo($"Role: {_config.Role}");
                    Echo($"Brain: {_activeBrain?.Name ?? "None"}");
                    Echo($"Status: {_activeBrain?.Status ?? "N/A"}");
                    break;

                case "STOP":
                    _activeBrain?.Shutdown();
                    _brainStateMachine?.Dispose();
                    _brainStateMachine = null;
                    Runtime.UpdateFrequency = UpdateFrequency.None;
                    Echo("Stopped.");
                    break;

                case "START":
                    SelectBrain();
                    Runtime.UpdateFrequency = UpdateFrequency.Update10;
                    Echo("Started.");
                    break;

                case "DOCK":
                    HandleDockCommand();
                    break;

                case "ESCORT":
                    HandleEscortCommand();
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
                        Echo("Available commands: RELOAD, STATUS, STOP, START, DOCK, ESCORT");
                        if (_config.Role == GridRole.Leader)
                        {
                            Echo("Leader commands: DOCK_ALL, ESCORT_ALL, FORMUP_ALL");
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
            var controllers = new List<IMyShipController>();
            GridTerminalSystem.GetBlocksOfType(controllers, c => c.CubeGrid == Me.CubeGrid);

            // Prefer cockpits that are set as main
            foreach (var controller in controllers)
            {
                var cockpit = controller as IMyCockpit;
                if (cockpit != null && cockpit.IsMainCockpit)
                    return cockpit;
            }

            // Then any cockpit
            foreach (var controller in controllers)
            {
                if (controller is IMyCockpit)
                    return controller;
            }

            // Then remote control
            foreach (var controller in controllers)
            {
                if (controller is IMyRemoteControl)
                    return controller;
            }

            // Any controller
            return controllers.FirstOrDefault();
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
