using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRage;
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
        public string Name => "Drone";
        public string Status { get; private set; } = "Initializing";

        // === Context exposed to directives and executors ===
        public BrainContext Context { get; private set; }
        public GyroController Gyros { get; private set; }
        public ThrusterController Thrusters { get; private set; }
        public FormationNavigator Navigator { get; private set; }
        public Action<string> Echo => Context?.Echo;
        public DroneConfig Config => Context.Config;

        // === Leader tracking ===
        public LeaderStateMessage LastLeaderState { get; private set; }
        public bool HasLeaderContact { get; private set; }
        
        private IMyBroadcastListener _listener;
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

        // === Executors ===
        private PositionExecutor _positionExecutor;
        private OrientationExecutor _orientationExecutor;

        // === Cached data for status display ===
        private double _lastDistanceToFormation;

        // === WcPbApiBridge integration for missile interception ===
        private WcPbApiBridgeClient _wcBridge;
        private Program.WcPbApi _wcApi;
        private bool _hasPositionTracking;
        private List<Vector3D> _projectilePositions = new List<Vector3D>();
        private List<Vector3D> _tempPositions = new List<Vector3D>();
        
        public int ProjectileCount { get; private set; }
        public bool HasPositionTracking => _hasPositionTracking;

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

            // Initialize tactical context
            _tacticalContext = new TacticalContext();

            // Initialize drone context (passed to directives)
            _droneContext = new DroneContext(this, _tacticalContext);

            // Initialize executors
            _positionExecutor = new PositionExecutor(Navigator, context.Echo);
            _orientationExecutor = new OrientationExecutor(context.Echo);

            // Initialize WeaponCore APIs
            InitializeWeaponCore(context);

            // Set initial directive
            SetDirective(new EscortDirective());

            Status = $"G:{Gyros.GyroCount} T:{Thrusters.ThrusterCount} | {_currentDirective.Name}";
        }

        private void InitializeWeaponCore(BrainContext context)
        {
            _hasPositionTracking = false;
            _wcBridge = new WcPbApiBridgeClient();
            
            try
            {
                if (_wcBridge.Activate(context.Me))
                {
                    _hasPositionTracking = true;
                    context.Echo?.Invoke("[Drone] WcPbApiBridge connected (position tracking enabled)");
                }
            }
            catch
            {
                // Bridge activation failed
            }
            
            if (!_hasPositionTracking)
            {
                _wcApi = new Program.WcPbApi();
                try
                {
                    if (_wcApi.Activate(context.Me))
                    {
                        context.Echo?.Invoke("[Drone] WcPbApi connected (count only)");
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
            while (true)
            {
                // === Brain responsibilities (every tick) ===
                ProcessMessages();
                CheckLeaderTimeout();
                UpdateControllers();
                UpdateTacticalContext();

                // === Run directive ===
                RunDirective();

                UpdateStatus();
                yield return true;
            }
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

                // Notify executors of behavior changes
                if (newIntent != null)
                {
                    _positionExecutor.OnBehaviorChanged(newIntent.Position);
                    _orientationExecutor.OnBehaviorChanged(newIntent.Orientation);
                }

                _currentIntent = newIntent;
            }

            // Execute current behaviors
            if (_currentIntent != null)
            {
                _positionExecutor.Execute(_currentIntent.Position, _droneContext);
                _orientationExecutor.Execute(_currentIntent.Orientation, _droneContext);

                // Update cached formation data for status
                if (HasLeaderContact)
                {
                    _lastDistanceToFormation = _droneContext.DistanceToFormation();
                }
            }
        }

        /// <summary>
        /// Updates tactical context with current threat data.
        /// </summary>
        private void UpdateTacticalContext()
        {
            ProjectileCount = 0;
            _projectilePositions.Clear();
            
            long droneEntityId = Context.Me.CubeGrid.EntityId;
            long leaderEntityId = HasLeaderContact ? LastLeaderState.EntityId : 0;
            
            if (_hasPositionTracking && _wcBridge != null && _wcBridge.IsReady && _wcBridge.IsWcApiReady)
            {
                CheckThreatsWithPositions(droneEntityId, leaderEntityId);
                _tacticalContext.UpdateThreats(_projectilePositions);
            }
            else if (_wcApi != null)
            {
                CheckThreatsCountOnly(droneEntityId, leaderEntityId);
                _tacticalContext.UpdateThreatCount(ProjectileCount);
            }
            else
            {
                _tacticalContext.ClearThreats();
            }
        }

        private void CheckThreatsWithPositions(long droneEntityId, long leaderEntityId)
        {
            _tempPositions.Clear();
            int droneCount = _wcBridge.GetProjectilesLockedOnPos(droneEntityId, _tempPositions);
            if (droneCount > 0)
            {
                _projectilePositions.AddRange(_tempPositions);
            }
            
            if (leaderEntityId != 0)
            {
                _tempPositions.Clear();
                int leaderCount = _wcBridge.GetProjectilesLockedOnPos(leaderEntityId, _tempPositions);
                if (leaderCount > 0)
                {
                    foreach (var pos in _tempPositions)
                    {
                        bool isDuplicate = false;
                        foreach (var existing in _projectilePositions)
                        {
                            if (Vector3D.DistanceSquared(pos, existing) < 1.0)
                            {
                                isDuplicate = true;
                                break;
                            }
                        }
                        if (!isDuplicate)
                        {
                            _projectilePositions.Add(pos);
                        }
                    }
                }
            }
            
            ProjectileCount = _projectilePositions.Count;
        }

        private void CheckThreatsCountOnly(long droneEntityId, long leaderEntityId)
        {
            int totalCount = 0;
            
            var droneResult = _wcApi.GetProjectilesLockedOn(droneEntityId);
            if (droneResult.Item1)
            {
                totalCount += droneResult.Item2;
            }
            
            if (leaderEntityId != 0)
            {
                var leaderResult = _wcApi.GetProjectilesLockedOn(leaderEntityId);
                if (leaderResult.Item1)
                {
                    totalCount += leaderResult.Item2;
                }
            }
            
            ProjectileCount = totalCount;
        }

        private void UpdateControllers()
        {
            Gyros.SetDeltaTime(Context.DeltaTime);
            Thrusters.SetDeltaTime(Context.DeltaTime);

            Gyros.SetReference(Context.Reference);
            Thrusters.SetReference(Context.Reference);

            Gyros.SetMaxTilt(Config.MaxTiltAngle);

            var massData = Context.Reference.CalculateShipMass();
            Thrusters.SetShipMass(massData.PhysicalMass);
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
                        LastLeaderState = leaderState;
                        HasLeaderContact = true;
                        _lastContactTime = Context.GameTime;
                    }
                }
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
            string phase = "";
            
            // Add approach phase info if relevant
            if (_currentIntent?.Position is Approach)
            {
                phase = $" ({_positionExecutor.CurrentPhase})";
            }
            else if (_currentIntent?.Position is FormationFollow)
            {
                phase = " (Formation)";
            }
            
            if (!HasLeaderContact)
            {
                Status = $"{directiveName}{phase} | No leader";
                return;
            }

            double distToLeader = Vector3D.Distance(Position, LastLeaderState.Position);
            double speed = Context.Reference.GetShipSpeed();
            double angleOff = Gyros.TotalError * 180.0 / Math.PI;

            string threatInfo = ProjectileCount > 0 ? $" T:{ProjectileCount}" : "";

            Status = $"{directiveName}{phase} | F:{_lastDistanceToFormation:F0}m L:{distToLeader:F0}m | {speed:F0}m/s {angleOff:F1}°{threatInfo}";
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
