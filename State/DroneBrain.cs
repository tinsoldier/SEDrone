using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Brain for follower drones.
    /// 
    /// This class acts as a context provider and state machine runner.
    /// Actual behavioral logic lives in IDroneMode implementations.
    /// 
    /// Responsibilities:
    /// - Initialize hardware controllers
    /// - Process IGC messages for leader tracking
    /// - Run the state machine (delegate to current mode)
    /// - Expose components and state to modes
    /// </summary>
    public class DroneBrain : IBrain
    {
        // === IBrain implementation ===
        public string Name => "Drone";
        public string Status { get; private set; } = "Initializing";

        // === Context exposed to modes ===
        public BrainContext Context { get; private set; }
        public GyroController Gyros { get; private set; }
        public ThrusterController Thrusters { get; private set; }
        public FormationNavigator Navigator { get; private set; }
        public Action<string> Echo => Context?.Echo;
        public DroneConfig Config => Context.Config;

        // === Leader tracking (updated by brain, read by modes) ===
        public LeaderStateMessage LastLeaderState { get; private set; }
        public bool HasLeaderContact { get; private set; }
        
        private IMyBroadcastListener _listener;
        private double _lastContactTime;
        private const double CONTACT_TIMEOUT = 2.0;  // Seconds before leader considered lost

        // === Convenience accessors for modes ===
        public Vector3D Position => Context.Reference.GetPosition();
        public Vector3D Velocity => Context.Reference.GetShipVelocities().LinearVelocity;

        // === State machine ===
        private IDroneMode _currentMode;

        // === Cached formation data for status display ===
        private Vector3D _lastFormationPosition;
        private double _lastDistanceToFormation;

        public void Initialize(BrainContext context)
        {
            Context = context;

            // Register for IGC broadcasts
            _listener = context.IGC.RegisterBroadcastListener(context.Config.IGCChannel);

            // Initialize gyro controller
            var gyros = new List<IMyGyro>();
            context.GridTerminalSystem.GetBlocksOfType(gyros, g => g.CubeGrid == context.Me.CubeGrid);
            Gyros = new GyroController(
                context.Reference,
                gyros,
                context.Config.OrientationPID,
                context.Echo
            );

            // Initialize thruster controller
            var thrusters = new List<IMyThrust>();
            context.GridTerminalSystem.GetBlocksOfType(thrusters, t => t.CubeGrid == context.Me.CubeGrid);
            Thrusters = new ThrusterController(
                context.Reference,
                thrusters,
                context.Config.ThrustConfig,
                context.Echo
            );

            // Initialize formation navigator
            Navigator = new FormationNavigator(context.Config);

            // Set initial mode
            _currentMode = new SearchingMode();

            Status = $"G:{Gyros.GyroCount} T:{Thrusters.ThrusterCount} | {_currentMode.Name}";
        }

        public IEnumerator<bool> Run()
        {
            // Enter initial mode
            _currentMode.Enter(this);
            UpdateStatus();

            while (true)
            {
                // === Brain responsibilities (every tick) ===
                ProcessMessages();
                CheckLeaderTimeout();
                UpdateControllers();

                // === Run state machine ===
                IDroneMode nextMode = _currentMode.Update(this);

                if (nextMode != _currentMode)
                {
                    TransitionTo(nextMode);
                }

                UpdateStatus();
                yield return true;
            }
        }

        /// <summary>
        /// Transitions to a new mode, calling Exit on current and Enter on new.
        /// </summary>
        private void TransitionTo(IDroneMode newMode)
        {
            Echo?.Invoke($"[Drone] {_currentMode.Name} → {newMode.Name}");
            _currentMode.Exit(this);
            _currentMode = newMode;
            _currentMode.Enter(this);
        }

        /// <summary>
        /// Updates controller references and timing.
        /// Called every tick before mode update.
        /// </summary>
        private void UpdateControllers()
        {
            // Update timing
            Gyros.SetDeltaTime(Context.DeltaTime);
            Thrusters.SetDeltaTime(Context.DeltaTime);

            // Update references
            Gyros.SetReference(Context.Reference);
            Thrusters.SetReference(Context.Reference);

            // Update tilt limit from config
            Gyros.SetMaxTilt(Config.MaxTiltAngle);

            // Update ship mass
            var massData = Context.Reference.CalculateShipMass();
            Thrusters.SetShipMass(massData.PhysicalMass);
        }

        /// <summary>
        /// Processes pending IGC messages and updates leader state.
        /// </summary>
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
                        LastLeaderState = leaderState;
                        HasLeaderContact = true;
                        _lastContactTime = Context.GameTime;
                    }
                }
            }
        }

        /// <summary>
        /// Checks if leader contact has timed out.
        /// </summary>
        private void CheckLeaderTimeout()
        {
            if (HasLeaderContact && Context.GameTime - _lastContactTime > CONTACT_TIMEOUT)
            {
                HasLeaderContact = false;
                Echo?.Invoke("[Drone] Leader contact lost!");
            }
        }

        /// <summary>
        /// Called by modes to update cached formation data for status display.
        /// </summary>
        public void UpdateFormationData(Vector3D formationPosition, double distanceToFormation)
        {
            _lastFormationPosition = formationPosition;
            _lastDistanceToFormation = distanceToFormation;
        }

        /// <summary>
        /// Updates the status string.
        /// </summary>
        private void UpdateStatus()
        {
            string modeName = _currentMode?.Name ?? "None";
            
            if (!HasLeaderContact)
            {
                Status = $"{modeName} | No leader";
                return;
            }

            double distToLeader = Vector3D.Distance(Position, LastLeaderState.Position);
            double speed = Context.Reference.GetShipSpeed();
            double angleOff = Gyros.TotalError * 180.0 / Math.PI;

            Status = $"{modeName} | F:{_lastDistanceToFormation:F0}m L:{distToLeader:F0}m | {speed:F0}m/s {angleOff:F1}°";
        }

        public void Shutdown()
        {
            _currentMode?.Exit(this);
            Gyros?.Release();
            Thrusters?.Release();
            Status = "Shutdown";
        }
    }
}
