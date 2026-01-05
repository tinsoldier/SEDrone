using System;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Read-only context passed to directives.
    /// Provides access to drone state, leader tracking, and tactical information.
    ///
    /// Directives should not modify hardware directly - they yield BehaviorIntents
    /// which are executed by the Brain through Executors.
    /// </summary>
    public class DroneContext
    {
        // === Core references (set by Brain) ===
        private DroneBrain _brain;

        /// <summary>
        /// Drone configuration.
        /// </summary>
        public DroneConfig Config => _brain.Config;

        /// <summary>
        /// The ship controller used as reference.
        /// </summary>
        public IMyShipController Reference => _brain.Context.Reference;

        /// <summary>
        /// Grid terminal system for block access.
        /// </summary>
        public IMyGridTerminalSystem GridTerminalSystem => _brain.Context.GridTerminalSystem;

        /// <summary>
        /// The programmable block running this code.
        /// </summary>
        public IMyProgrammableBlock Me => _brain.Context.Me;

        /// <summary>
        /// Grid entity ID associated with this drone.
        /// </summary>
        public long GridId => _brain.Context.GridId != 0 ? _brain.Context.GridId : Me.CubeGrid.EntityId;

        /// <summary>
        /// Current game time in seconds.
        /// </summary>
        public double GameTime => _brain.Context.GameTime;

        /// <summary>
        /// Time since last update in seconds.
        /// </summary>
        public double DeltaTime => _brain.Context.DeltaTime;

        /// <summary>
        /// Current world position.
        /// </summary>
        public Vector3D Position => Reference.GetPosition();

        /// <summary>
        /// Current velocity.
        /// </summary>
        public Vector3D Velocity => Reference.GetShipVelocities().LinearVelocity;

        /// <summary>
        /// Current gravity vector (world space, points down).
        /// </summary>
        public Vector3D Gravity => Reference.GetNaturalGravity();

        /// <summary>
        /// Current world matrix.
        /// </summary>
        public MatrixD WorldMatrix => Reference.WorldMatrix;

        // === Leader tracking ===

        /// <summary>
        /// Whether leader contact is currently active.
        /// </summary>
        public bool HasLeaderContact => _brain.HasLeaderContact;

        /// <summary>
        /// Last received leader state. Check HasLeaderContact before using.
        /// </summary>
        public LeaderStateMessage LastLeaderState => _brain.LastLeaderState;

        // === Tactical awareness ===

        /// <summary>
        /// Tactical context with threat information.
        /// </summary>
        public TacticalContext Tactical { get; }

        /// <summary>
        /// Debug logger for development and testing.
        /// Writes to a text panel named "Debug" if present.
        /// </summary>
        public DebugLogger Debug { get; }

        /// <summary>
        /// Pre-collected hardware references (ref-hack mode).
        /// </summary>
        public DroneHardware Hardware => _brain.Context.Hardware;

        // === Hardware access (for executors) ===

        /// <summary>
        /// Gyro controller. Used by OrientationExecutor.
        /// </summary>
        public GyroController Gyros => _brain.Gyros;

        /// <summary>
        /// Thruster controller. Used by PositionExecutor.
        /// </summary>
        public ThrusterController Thrusters => _brain.Thrusters;

        /// <summary>
        /// Formation navigator. Used by PositionExecutor.
        /// </summary>
        public FormationNavigator Navigator => _brain.Navigator;

        /// <summary>
        /// Docking navigator. Used by DockDirective.
        /// </summary>
        public DockingNavigator DockingNav => _brain.DockingNav;

        /// <summary>
        /// Fixed weapon rig provider.
        /// </summary>
        public FixedWeaponRigProvider WeaponRigs => _brain.WeaponRigs;

        /// <summary>
        /// WeaponCore PB API (may be null if unavailable).
        /// </summary>
        public Program.WcPbApi WcApi => _brain.WcApi;

        /// <summary>
        /// IGC request manager for request-response patterns.
        /// </summary>
        public IGCRequestManager IGCRequests => _brain.IGCRequests;

        /// <summary>
        /// Creates a new DroneContext.
        /// </summary>
        public DroneContext(DroneBrain brain, TacticalContext tactical, DebugLogger debug)
        {
            _brain = brain;
            Tactical = tactical;
            Debug = debug;
        }

        // === Helper methods for directives ===

        /// <summary>
        /// Calculates distance from current position to a target.
        /// </summary>
        public double DistanceTo(Vector3D target)
        {
            return Vector3D.Distance(Position, target);
        }

        /// <summary>
        /// Checks if the drone has arrived at a position within tolerance.
        /// </summary>
        /// <param name="target">Target position</param>
        /// <param name="tolerance">Distance tolerance, or -1 for config StationRadius</param>
        public bool HasArrived(Vector3D target, double tolerance = -1)
        {
            double tol = tolerance > 0 ? tolerance : Config.StationRadius;
            return DistanceTo(target) <= tol;
        }

        /// <summary>
        /// Calculates the current formation position based on leader state.
        /// Returns Zero if no leader contact.
        /// </summary>
        public Vector3D GetFormationPosition()
        {
            if (!HasLeaderContact)
                return Vector3D.Zero;

            return Navigator.CalculateFormationPosition(LastLeaderState, Config.StationOffset);
        }

        /// <summary>
        /// Gets distance to formation position.
        /// Returns double.MaxValue if no leader contact.
        /// </summary>
        public double DistanceToFormation()
        {
            if (!HasLeaderContact)
                return double.MaxValue;

            Vector3D formationPos = GetFormationPosition();
            formationPos = Navigator.AdjustForTerrainClearance(formationPos, Reference, Config.MinTerrainClearance);
            return Vector3D.Distance(Position, formationPos);
        }

        /// <summary>
        /// Checks if the drone is currently in formation (within StationRadius).
        /// </summary>
        public bool IsInFormation()
        {
            if (!HasLeaderContact)
                return false;

            double distance = DistanceToFormation();
            double brakingDistance = Thrusters.GetBrakingDistance(Velocity.Length(), Velocity);
            return Navigator.IsInFormationWithBrakingMargin(distance, brakingDistance);
        }

        /// <summary>
        /// Checks if the drone has drifted far enough from formation to require reapproach.
        /// </summary>
        public bool HasExitedFormation()
        {
            if (!HasLeaderContact)
                return true;

            double distance = DistanceToFormation();
            double brakingDistance = Thrusters.GetBrakingDistance(Velocity.Length(), Velocity);
            return Navigator.HasExitedFormationWithBrakingMargin(distance, brakingDistance, 2.5);
        }

        // === Docking state ===

        private IMyShipConnector _activeConnector;

        /// <summary>
        /// The currently active connector used for docking.
        /// Set by DockDirective when connector is selected.
        /// Auto-detects connected connector if not set (e.g., after game reload).
        /// </summary>
        public IMyShipConnector ActiveConnector
        {
            get
            {
                // If explicitly set and still connected, use it
                if (_activeConnector != null && _activeConnector.Status == MyShipConnectorStatus.Connected)
                    return _activeConnector;

                // Auto-detect: find any connected connector on our grid
                var connectors = new System.Collections.Generic.List<IMyShipConnector>();
                if (Hardware != null && Hardware.Connectors.Count > 0)
                {
                    connectors.AddRange(Hardware.Connectors);
                }
                else
                {
                    long gridId = GridId;
                    GridTerminalSystem.GetBlocksOfType(connectors, c => c.CubeGrid.EntityId == gridId);
                }

                IMyShipConnector firstConnected = null;
                foreach (var connector in connectors)
                {
                    if (connector.Status == MyShipConnectorStatus.Connected)
                    {
                        // Prefer connector connected to leader's grid
                        if (HasLeaderContact && connector.OtherConnector != null)
                        {
                            long leaderGridId = LastLeaderState.EntityId;
                            if (connector.OtherConnector.CubeGrid.EntityId == leaderGridId)
                            {
                                _activeConnector = connector;
                                return connector;
                            }
                        }

                        // Track first connected as fallback
                        if (firstConnected == null)
                            firstConnected = connector;
                    }
                }

                // Return first connected if no leader match found
                if (firstConnected != null)
                {
                    _activeConnector = firstConnected;
                    return firstConnected;
                }

                return null;
            }
            set { _activeConnector = value; }
        }

        /// <summary>
        /// Returns true if the drone is currently docked (connector connected).
        /// </summary>
        public bool IsDocked
        {
            get
            {
                return ActiveConnector != null && ActiveConnector.Status == MyShipConnectorStatus.Connected;
            }
        }

        /// <summary>
        /// Disconnects the active connector if docked.
        /// </summary>
        public void Undock()
        {
            if (ActiveConnector != null && ActiveConnector.Status == MyShipConnectorStatus.Connected)
            {
                ActiveConnector.Disconnect();
            }
        }

        /// <summary>
        /// Enables or disables inertial dampeners.
        /// </summary>
        public void SetDampeners(bool enabled)
        {
            Reference.DampenersOverride = enabled;
        }
    }
}
