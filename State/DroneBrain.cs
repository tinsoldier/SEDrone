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

        // === WcPbApiBridge integration for missile interception ===
        private WcPbApiBridgeClient _wcBridge;
        private Program.WcPbApi _wcApi;  // Fallback when bridge mod not available
        private bool _hasPositionTracking;  // True if we can get missile positions
        private List<Vector3D> _projectilePositions = new List<Vector3D>();
        private List<Vector3D> _tempPositions = new List<Vector3D>();  // Temp buffer for API calls
        
        /// <summary>
        /// Number of smart projectiles currently detected.
        /// </summary>
        public int ProjectileCount { get; private set; }
        
        /// <summary>
        /// Whether we have position tracking (bridge mod) or just count (standard WcPbApi).
        /// </summary>
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

            // Initialize WeaponCore APIs for missile tracking
            // Try WcPbApiBridge first (provides positions), fall back to standard WcPbApi (count only)
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
                // Bridge activation failed - continue to fallback
            }
            
            if (!_hasPositionTracking)
            {
                // Try standard WcPbApi as fallback
                _wcApi = new Program.WcPbApi();
                try
                {
                    if (_wcApi.Activate(context.Me))
                    {
                        context.Echo?.Invoke("[Drone] WcPbApi connected (count only, will nose-up on threat)");
                    }
                    else
                    {
                        _wcApi = null;
                        context.Echo?.Invoke("[Drone] No WeaponCore API available - intercept mode disabled");
                    }
                }
                catch
                {
                    _wcApi = null;
                    context.Echo?.Invoke("[Drone] WeaponCore not detected - intercept mode disabled");
                }
            }

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
                
                // === Threat detection (interrupt pattern) ===
                CheckForThreats();

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
        /// Checks for incoming missile threats and handles mode interruption.
        /// Uses WcPbApiBridge (with positions) or falls back to WcPbApi (count only).
        /// </summary>
        private void CheckForThreats()
        {
            ProjectileCount = 0;
            _projectilePositions.Clear();
            
            long droneEntityId = Context.Me.CubeGrid.EntityId;
            long leaderEntityId = HasLeaderContact ? LastLeaderState.EntityId : 0;
            
            if (_hasPositionTracking && _wcBridge != null && _wcBridge.IsReady && _wcBridge.IsWcApiReady)
            {
                // === Full position tracking via WcPbApiBridge ===
                CheckThreatsWithPositions(droneEntityId, leaderEntityId);
            }
            else if (_wcApi != null)
            {
                // === Fallback: count-only via standard WcPbApi ===
                CheckThreatsCountOnly(droneEntityId, leaderEntityId);
            }
            else
            {
                // No WC API available - can't detect missiles
                return;
            }
            
            // Handle mode transitions based on threat state
            if (ProjectileCount > 0)
            {
                var interceptMode = _currentMode as InterceptMode;
                if (interceptMode != null)
                {
                    // Already in intercept mode - update
                    if (_hasPositionTracking && _projectilePositions.Count > 0)
                    {
                        interceptMode.UpdateProjectilePositions(_projectilePositions);
                    }
                    else
                    {
                        interceptMode.UpdateProjectileCount(ProjectileCount);
                    }
                }
                else
                {
                    // Not in intercept mode - transition immediately
                    // Default to ClosestMissileAimStrategy for aggressive point defense
                    var newMode = new InterceptMode(new ClosestMissileAimStrategy(), ProjectileCount);
                    newMode.SetHasPositionTracking(_hasPositionTracking);
                    TransitionTo(newMode);
                    
                    if (_hasPositionTracking && _projectilePositions.Count > 0)
                    {
                        newMode.UpdateProjectilePositions(_projectilePositions);
                    }
                }
            }
            else
            {
                // No threats
                if (_currentMode is InterceptMode)
                {
                    TransitionTo(new SearchingMode());
                }
            }
        }

        /// <summary>
        /// Checks for threats using WcPbApiBridge (provides missile positions).
        /// </summary>
        private void CheckThreatsWithPositions(long droneEntityId, long leaderEntityId)
        {
            // Get projectiles targeting drone
            _tempPositions.Clear();
            int droneCount = _wcBridge.GetProjectilesLockedOnPos(droneEntityId, _tempPositions);
            if (droneCount > 0)
            {
                _projectilePositions.AddRange(_tempPositions);
            }
            
            // Get projectiles targeting leader (if we have contact)
            if (leaderEntityId != 0)
            {
                _tempPositions.Clear();
                int leaderCount = _wcBridge.GetProjectilesLockedOnPos(leaderEntityId, _tempPositions);
                if (leaderCount > 0)
                {
                    // Add leader-targeted missiles, avoiding duplicates
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

        /// <summary>
        /// Checks for threats using standard WcPbApi (count only, no positions).
        /// </summary>
        private void CheckThreatsCountOnly(long droneEntityId, long leaderEntityId)
        {
            int totalCount = 0;
            
            // Check drone
            var droneResult = _wcApi.GetProjectilesLockedOn(droneEntityId);
            if (droneResult.Item1)  // Item1 = isLockedOn
            {
                totalCount += droneResult.Item2;  // Item2 = count
            }
            
            // Check leader
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
