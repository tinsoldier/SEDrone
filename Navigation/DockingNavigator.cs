using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Result of docking approach calculation.
    /// </summary>
    public struct DockingApproachResult
    {
        public Vector3D Position;
        public Vector3D Heading;

        public DockingApproachResult(Vector3D position, Vector3D heading)
        {
            Position = position;
            Heading = heading;
        }
    }

    /// <summary>
    /// Handles docking-specific navigation calculations.
    /// Encapsulates math and details around docking approach paths,
    /// keeping directives pure and declarative.
    /// </summary>
    public class DockingNavigator
    {
        private readonly FormationNavigator _formationNav;
        private readonly DroneConfig _config;

        public DockingNavigator(FormationNavigator formationNav, DroneConfig config)
        {
            _formationNav = formationNav;
            _config = config;
        }

        /// <summary>
        /// Selects the best connector on the drone grid for docking.
        /// Prefers connectors whose forward axis anti-aligns with the target connector.
        /// </summary>
        /// <param name="gts">Grid terminal system for finding blocks</param>
        /// <param name="droneGrid">The drone's grid</param>
        /// <param name="targetConnectorForward">Target connector's forward direction (world space)</param>
        /// <returns>Best matching connector, or null if none found</returns>
        public IMyShipConnector SelectDroneConnector(
            IMyGridTerminalSystem gts,
            long droneGridEntityId,
            Vector3D targetConnectorForward)
        {
            var connectors = new List<IMyShipConnector>();
            gts.GetBlocksOfType(connectors, c => c.CubeGrid.EntityId == droneGridEntityId && c.IsFunctional);

            return SelectDroneConnector(connectors, targetConnectorForward);
        }

        public IMyShipConnector SelectDroneConnector(
            List<IMyShipConnector> connectors,
            Vector3D targetConnectorForward)
        {
            if (connectors.Count == 0)
                return null;

            IMyShipConnector bestMatch = null;
            double bestAlignment = 1.0; // Want closest to -1 (anti-parallel)

            foreach (var connector in connectors)
            {
                Vector3D droneForward = connector.WorldMatrix.Forward;
                double alignment = Vector3D.Dot(droneForward, targetConnectorForward);

                // Anti-parallel alignment (dot product ≈ -1) is ideal for docking
                if (alignment < bestAlignment)
                {
                    bestAlignment = alignment;
                    bestMatch = connector;
                }
            }

            return bestMatch;
        }

        /// <summary>
        /// Generates scalar distances for approach waypoints.
        /// These are applied dynamically along the connector's approach direction (-forward).
        /// </summary>
        /// <param name="droneConnectorSize">Drone connector radius</param>
        /// <param name="targetConnectorSize">Target connector radius</param>
        /// <returns>List of distances from connector along its approach axis</returns>
        public List<double> GenerateApproachDistances(
            double droneConnectorSize,
            double targetConnectorSize)
        {
            double clearance = droneConnectorSize + targetConnectorSize - 0.25; // Safety margin

            return new List<double>
            {
                clearance + 20,   // Far approach
                clearance + 15,   // Mid approach  
                clearance + 5,    // Near approach
                clearance         // Final docking position
            };
        }

        /// <summary>
        /// Calculates the final docking position and orientation.
        /// Accounts for target velocity prediction (SEAD2-inspired).
        /// </summary>
        /// <param name="targetPosition">Target connector world position</param>
        /// <param name="targetVelocity">Target connector world velocity</param>
        /// <param name="connectorForward">Target connector forward direction</param>
        /// <param name="droneConnectorSize">Drone connector radius</param>
        /// <param name="targetConnectorSize">Target connector radius</param>
        /// <returns>DockingApproachResult with position and heading</returns>
        public DockingApproachResult CalculateDockingApproach(
            Vector3D targetPosition,
            Vector3D targetVelocity,
            Vector3D connectorForward,
            double droneConnectorSize,
            double targetConnectorSize)
        {
            // Predict target position based on velocity
            double predictionTime = _config.PredictionTime; // Default 0.5s
            Vector3D predictedPos = targetPosition + targetVelocity * predictionTime;

            // Calculate clearance-adjusted approach position (approach from -forward).
            double clearance = droneConnectorSize + targetConnectorSize;
            Vector3D dockingPosition = predictedPos - connectorForward * clearance;

            // Drone should face opposite direction to dock
            Vector3D dockingHeading = -connectorForward;

            return new DockingApproachResult(dockingPosition, dockingHeading);
        }

        /// <summary>
        /// Gets the connector radius based on grid size.
        /// SEAD2-inspired calculation.
        /// </summary>
        public static double GetConnectorRadius(IMyShipConnector connector)
        {
            if (connector.CubeGrid.GridSize == 0.5) // Small grid
                return connector.CubeGrid.GridSize; // 0.5m
            return connector.CubeGrid.GridSize * 0.5; // 1.25m for large
        }
    }
}
