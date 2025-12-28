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
        /// Current world position.
        /// </summary>
        public Vector3D Position => Reference.GetPosition();

        /// <summary>
        /// Current velocity.
        /// </summary>
        public Vector3D Velocity => Reference.GetShipVelocities().LinearVelocity;

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
        /// Creates a new DroneContext.
        /// </summary>
        public DroneContext(DroneBrain brain, TacticalContext tactical)
        {
            _brain = brain;
            Tactical = tactical;
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
    }
}
