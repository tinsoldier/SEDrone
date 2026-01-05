using System;
using System.Collections.Generic;
using System.Runtime.InteropServices.ComTypes;
using Sandbox.ModAPI.Ingame;
using VRage;
using VRage.Game;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Brain for follower drones.
    /// 
    /// This class acts as a context provider and directive orchestrator.
    /// Behavioral logic is expressed through IDirective implementations
    /// which yield BehaviorIntents executed by Executors.
    /// 
    /// Responsibilities:
    /// - Initialize hardware controllers
    /// - Process IGC messages for leader tracking
    /// - Run the directive iterator and execute behaviors
    /// - Update tactical context for threat awareness
    /// </summary>
    public class DroneBrain : IBrain
    {
        // === IBrain implementation ===
        public string Name { get { return "Drone"; } }
        public string Status { get; private set; }
        public IMyProgrammableBlock PB { get; set; }

        // === Context exposed to directives and executors ===
        public BrainContext Context { get; private set; }
        public GyroController Gyros { get; private set; }
        public ThrusterController Thrusters { get; private set; }
        public FormationNavigator Navigator { get; private set; }
        public DockingNavigator DockingNav { get; private set; }
        public DebugLogger Debug => _droneContext?.Debug;
        public FixedWeaponRigProvider WeaponRigs { get; private set; }
        public Program.WcPbApi WcApi { get { return _wcApi; } }
        public IGCRequestManager IGCRequests { get; private set; }
        public Action<string> Echo { get { return Context != null ? Context.Echo : null; } }
        public DroneConfig Config { get { return Context.Config; } }

        // === Leader tracking ===
        public LeaderStateMessage LastLeaderState { get; private set; }
        public bool HasLeaderContact { get; private set; }

        private double _lastContactTime;
        private const double CONTACT_TIMEOUT = 2.0;

        // === Convenience accessors ===
        public Vector3D Position => Context.Reference.GetPosition();
        public Vector3D Velocity => Context.Reference.GetShipVelocities().LinearVelocity;

        // === Directive system ===
        private IDirective _currentDirective;
        private IEnumerator<BehaviorIntent> _directiveEnumerator;
        private BehaviorIntent _currentIntent;
        private DroneContext _droneContext;
        private TacticalContext _tacticalContext;

        // === State tracking for behaviors ===
        private IPositionBehavior _currentPositionBehavior;
        private IOrientationBehavior _currentOrientationBehavior;

        // === Cached data for status display ===
        private double _lastDistanceToFormation;

        private Program.WcPbApi _wcApi;

        private Dictionary<MyDetectedEntityInfo, float> _tempEnemyTargets = new Dictionary<MyDetectedEntityInfo, float>();
        private Dictionary<MyDetectedEntityInfo, float> _enemyTargets = new Dictionary<MyDetectedEntityInfo, float>();

        public int ProjectileCount { get; private set; }

        public DroneBrain()
        {
            Status = "Initializing";
        }

        public void Initialize(BrainContext context)
        {
            Context = context;
            if (Context.GridId == 0)
            {
                Context.GridId = Context.Reference != null
                    ? Context.Reference.CubeGrid.EntityId
                    : Context.Me.CubeGrid.EntityId;
            }

            if (context.CommandBus == null)
            {
                context.CommandBus = new IgcCommandBus(context.IGC, context.Config.IGCChannel + "_COMMAND");
            }
            if (context.LeaderStateBus == null)
            {
                context.LeaderStateBus = new IgcStateBus(context.IGC, context.Config.IGCChannel);
            }

            if (context.Hardware == null)
            {
                context.Hardware = DroneHardware.Capture(context.GridTerminalSystem, context.GridId, context.Reference);
            }

            if (context.Hardware == null || !context.Hardware.IsValid())
            {
                Status = "Error: Missing drone hardware";
                Echo?.Invoke("[Drone] Missing drone hardware (capture failed)");
                return;
            }

            // Initialize gyro controller
            var gyros = context.Hardware.Gyros;
            Gyros = new GyroController(
                context.Reference,
                gyros,
                context.Config.OrientationPID,
                context.Echo
            );

            // Initialize thruster controller
            var thrusters = context.Hardware.Thrusters;
            Thrusters = new ThrusterController(
                context.Reference,
                thrusters,
                context.Config.ThrustConfig,
                context.Echo
            );

            // Initialize formation navigator
            Navigator = new FormationNavigator(context.Config);

            // Initialize docking navigator
            DockingNav = new DockingNavigator(Navigator, context.Config);

            // Initialize IGC request manager
            IGCRequests = new IGCRequestManager(
                context.IGC,
                context.Me,
                context.Config.IGCChannel,
                context.Echo
            );
            
            // Initialize WeaponCore APIs
            InitializeWeaponCore(context);

            // Initialize tactical context
            _tacticalContext = new TacticalContext(_wcApi, context.Echo);

            // Initialize debug logger (writes to PB Echo)
            var debugLogger = new DebugLogger(context.Echo, () => context.GameTime);

            // Initialize drone context (passed to directives)
            _droneContext = new DroneContext(this, _tacticalContext, debugLogger);

            WeaponRigs = new FixedWeaponRigProvider(context.GridTerminalSystem, context.Me, context.Reference, _wcApi, _droneContext.Debug.Log, context.Hardware);

            // Set initial directive
            SetDirective(new EscortDirective());

            Status = $"G:{Gyros.GyroCount} T:{Thrusters.ThrusterCount} | {_currentDirective.Name}";
        }

        private void InitializeWeaponCore(BrainContext context)
        {
            _wcApi = new Program.WcPbApi();
            try
            {
                if (_wcApi.Activate(context.Me))
                {
                    context.Echo?.Invoke("[Drone] WcPbApi connected (count/threats enabled)");
                }
                else
                {
                    _wcApi = null;
                    context.Echo?.Invoke("[Drone] No WeaponCore API available");
                }
            }
            catch
            {
                _wcApi = null;
                context.Echo?.Invoke("[Drone] WeaponCore not detected");
            }
        }

        /// <summary>
        /// Sets a new directive, disposing the old enumerator.
        /// </summary>
        public void SetDirective(IDirective directive)
        {
            // Dispose old enumerator for cleanup
            if (_directiveEnumerator != null)
            {
                _directiveEnumerator.Dispose();
            }

            _currentDirective = directive;
            _directiveEnumerator = directive.Execute(_droneContext).GetEnumerator();
            _currentIntent = null;  // Force advancement on next update

            Echo?.Invoke($"[Drone] Directive: {directive.Name}");
        }

        public IEnumerator<bool> Run()
        {
            Exception caughtException = null;
            while (true)
            {
                try
                {
                    // === Brain responsibilities (every tick) ===
                    ProcessMessages();
                    CheckLeaderTimeout();
                    UpdateControllers();
                    UpdateTacticalContext();

                    // === Run directive ===
                    RunDirective();

                    UpdateStatus();

                    // Flush debug logs to Echo
                    _droneContext.Debug?.Flush();
                }
                catch (Exception ex)
                {
                    Status = "Error: " + ex.Message;
                    Echo?.Invoke("[Drone] Exception in Run(): " + ex.ToString());
                    caughtException = ex;
                    break;
                }
                yield return true;
            }
            // If an exception was caught, exit the enumerator
            yield break;
        }

        /// <summary>
        /// Runs one tick of the directive system.
        /// </summary>
        private void RunDirective()
        {
            // Check if we need to advance to next intent
            bool shouldAdvance = _currentIntent == null
                || _currentIntent.IsComplete
                || _currentIntent.IsAborted
                || (_currentIntent.ExitWhen != null && _currentIntent.ExitWhen());

            if (shouldAdvance)
            {
                if (!_directiveEnumerator.MoveNext())
                {
                    // Directive exhausted - restart with escort
                    Echo?.Invoke("[Drone] Directive exhausted, restarting escort");
                    SetDirective(new EscortDirective());
                    _directiveEnumerator.MoveNext();
                }

                var newIntent = _directiveEnumerator.Current;

                // Handle terminal states
                if (newIntent != null && (newIntent.IsComplete || newIntent.IsAborted))
                {
                    string reason = newIntent.IsAborted && newIntent.Abort != null
                        ? newIntent.Abort.Reason.ToString()
                        : "Complete";
                    Echo?.Invoke($"[Drone] Directive ended: {reason}");
                    SetDirective(new EscortDirective());
                    _directiveEnumerator.MoveNext();
                    newIntent = _directiveEnumerator.Current;
                }

                // Track behavior changes (reset state if behavior type changed)
                if (newIntent != null)
                {
                    bool positionTypeChanged = !IsSameBehaviorType(_currentPositionBehavior, newIntent.Position);
                    bool orientationTypeChanged = !IsSameBehaviorType(_currentOrientationBehavior, newIntent.Orientation);

                    _currentPositionBehavior = newIntent.Position;
                    _currentOrientationBehavior = newIntent.Orientation;
                }

                _currentIntent = newIntent;
            }

            // Execute current behaviors
            if (_currentIntent != null)
            {
                // Execute position behavior
                if (_currentIntent.Position != null)
                {
                    _currentIntent.Position.Execute(_droneContext);
                }
                else
                {
                    Thrusters.Release();
                }

                // Execute orientation behavior (unless position behavior is coupled and handles both)
                bool isCoupledBehavior = _currentIntent.Position is IOrientationBehavior;
                if (!isCoupledBehavior)
                {
                    if (_currentIntent.Orientation != null)
                    {
                        _currentIntent.Orientation.Execute(_droneContext);
                    }
                    else
                    {
                        // Default: match leader or stay level
                        if (HasLeaderContact)
                        {
                            Gyros.MatchCompassHeading(LastLeaderState.Forward, LastLeaderState.Up);
                        }
                        else
                        {
                            Gyros.OrientLevel();
                        }
                    }
                }

                // Update cached formation data for status
                if (HasLeaderContact)
                {
                    _lastDistanceToFormation = _droneContext.DistanceToFormation();
                }
            }
        }

        private bool IsSameBehaviorType(object a, object b)
        {
            if (a == null || b == null) return a == b;
            return a.GetType() == b.GetType();
        }

        /// <summary>
        /// Updates tactical context with current threat data.
        /// </summary>
        private void UpdateTacticalContext()
        {
            if (Context.TacticalCoordinator != null && Context.SharedTacticalSnapshot != null)
            {
                long droneEntityId = Context.GridId;
                long leaderEntityId = HasLeaderContact ? LastLeaderState.EntityId : 0;
                Context.TacticalCoordinator.UpdateSnapshot(Context.SharedTacticalSnapshot, droneEntityId, leaderEntityId, Context.GameTime);
                _tacticalContext.ApplySnapshot(Context.SharedTacticalSnapshot);
                ProjectileCount = _tacticalContext.ProjectileCount;
                return;
            }

            _tacticalContext.ClearProjectileThreats();
            if (_wcApi != null)
            {
                CheckEnemyTargets();
                _tacticalContext.UpdateEnemyTargets(_enemyTargets);
            }
            ProjectileCount = _tacticalContext.ProjectileCount;
        }

        private void CheckEnemyTargets()
        {
            if(_wcApi == null)
            {
                return; 
            }

            // Example: Get list of enemy targets
            _enemyTargets.Clear();
            _tempEnemyTargets.Clear();
            // _wcApi.GetSortedThreats(PB, _tempEnemyTargets);

            // Copy to main enemy targets dictionary if threat value is greater than zero
            foreach(var target in _tempEnemyTargets)
            {
                // if(target.Key.Type == MyDetectedEntityType.CharacterHuman || target.Key.Type == MyDetectedEntityType.CharacterOther)
                // {
                //     Context.Echo?.Invoke($"[Drone] character target: {target.Key.Name}");
                // }

                if(target.Value > 0)
                {
                    _enemyTargets[target.Key] = target.Value;
                }
            }
        }

        private void UpdateControllers()
        {
            Gyros.SetDeltaTime(Context.DeltaTime);
            Thrusters.SetDeltaTime(Context.DeltaTime);

            Gyros.SetReference(Context.Reference);
            Thrusters.SetReference(Context.Reference);

            Gyros.SetMaxTilt(Config.MaxTiltAngle);
            Gyros.SetOrientationMode(Config.OrientationMode);

            var massData = Context.Reference.CalculateShipMass();
            Thrusters.SetShipMass(massData.PhysicalMass);
        }

        private void ProcessMessages()
        {
            // Process leader state messages
            LeaderStateMessage leaderState;
            if (Context.LeaderStateBus != null && Context.LeaderStateBus.TryGetLatest(out leaderState))
            {
                LastLeaderState = leaderState;
                HasLeaderContact = true;
                _lastContactTime = Context.GameTime;
            }

            // Process command messages from leader
            ProcessCommandMessages();

            // Process IGC request/response messages
            if (IGCRequests != null)
            {
                IGCRequests.ProcessMessages(Context.GameTime);
            }
        }

        /// <summary>
        /// Processes command messages from the leader.
        /// </summary>
        private void ProcessCommandMessages()
        {
            if (Context.CommandBus == null)
                return;

            DroneCommandMessage command;
            while (Context.CommandBus.TryDequeue(out command))
            {
                // Check if command is for us (or broadcast to all)
                long myEntityId = Context.GridId;
                if (command.TargetDroneId == 0 || command.TargetDroneId == myEntityId)
                {
                    ExecuteCommand(command.Command);
                }
            }
        }

        /// <summary>
        /// Executes a received command by switching directives.
        /// </summary>
        private void ExecuteCommand(DroneCommand command)
        {
            switch (command)
            {
                case DroneCommand.Dock:
                    SetDirective(new DockDirective());
                    Echo?.Invoke("[Drone] Received DOCK command from leader");
                    break;

                case DroneCommand.FastDock:
                    SetDirective(new FastDockDirective());
                    Echo?.Invoke("[Drone] Received FASTDOCK command from leader");
                    break;

                case DroneCommand.Escort:
                    SetDirective(new EscortDirective());
                    Echo?.Invoke("[Drone] Received ESCORT command from leader");
                    break;

                case DroneCommand.FormUp:
                    SetDirective(new EscortDirective());
                    Echo?.Invoke("[Drone] Received FORMUP command from leader");
                    break;

                case DroneCommand.HoldPosition:
                    // TODO: Implement HoldPositionDirective
                    Echo?.Invoke("[Drone] HoldPosition not yet implemented");
                    break;

                case DroneCommand.ReturnToBase:
                    // TODO: Implement RTB directive
                    Echo?.Invoke("[Drone] ReturnToBase not yet implemented");
                    break;

                case DroneCommand.FollowMe:
                    SetDirective(new EscortDirective());
                    Echo?.Invoke("[Drone] Received FOLLOWME command from leader");
                    break;

                case DroneCommand.Scatter:
                    // TODO: Implement scatter behavior
                    Echo?.Invoke("[Drone] Scatter not yet implemented");
                    break;

                case DroneCommand.Attack:
                    // TODO: Implement attack directive (if needed, WC handles targeting)
                    Echo?.Invoke("[Drone] Attack command received (WeaponCore auto-targeting active)");
                    break;

                default:
                    Echo?.Invoke($"[Drone] Unknown command: {command}");
                    break;
            }
        }

        private void CheckLeaderTimeout()
        {
            if (HasLeaderContact && Context.GameTime - _lastContactTime > CONTACT_TIMEOUT)
            {
                HasLeaderContact = false;
                Echo?.Invoke("[Drone] Leader contact lost!");
            }
        }

        private void UpdateStatus()
        {
            string directiveName = _currentDirective?.Name ?? "None";

            if (!HasLeaderContact)
            {
                Status = $"{directiveName} | No leader";
                return;
            }

            double distToLeader = Vector3D.Distance(Position, LastLeaderState.Position);
            double speed = Context.Reference.GetShipSpeed();
            double angleOff = Gyros.TotalError * 180.0 / Math.PI;

            string threatInfo = ProjectileCount > 0 ? $" T:{ProjectileCount}" : "";

            Status = $"{directiveName} | F:{_lastDistanceToFormation:F0}m L:{distToLeader:F0}m | {speed:F0}m/s {angleOff:F1}°{threatInfo}";
        }

        public void Shutdown()
        {
            _directiveEnumerator?.Dispose();
            Gyros?.Release();
            Thrusters?.Release();
            Status = "Shutdown";
        }
    }
}
