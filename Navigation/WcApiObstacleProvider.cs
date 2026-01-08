using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Provides obstacles from WeaponCore's GetObstructions API.
    /// Returns nearby grids that WC considers obstructions to weapon fire.
    /// </summary>
    public class WcApiObstacleProvider : IObstacleProvider
    {
        private readonly Program.WcPbApi _wcApi;
        private readonly IMyProgrammableBlock _me;
        private readonly long _myGridId;
        private readonly Action<string> _log;

        // Reusable collection to avoid allocations
        private readonly List<MyDetectedEntityInfo> _obstructions = new List<MyDetectedEntityInfo>();

        // Throttle detailed logging
        private int _logCounter = 0;
        private const int LOG_INTERVAL = 60; // Log details every ~1 second at 60 TPS

        /// <summary>
        /// Creates a provider using WeaponCore API.
        /// </summary>
        /// <param name="wcApi">WeaponCore PB API instance.</param>
        /// <param name="me">The programmable block (used as reference for GetObstructions).</param>
        /// <param name="myGridId">This drone's grid ID (excluded from results).</param>
        /// <param name="log">Optional logging action for diagnostics.</param>
        public WcApiObstacleProvider(Program.WcPbApi wcApi, IMyProgrammableBlock me, long myGridId, Action<string> log = null)
        {
            _wcApi = wcApi;
            _me = me;
            _myGridId = myGridId;
            _log = log;
        }

        public IEnumerable<Obstacle> GetNearbyObstacles(Vector3D position, double radius)
        {
            if (_wcApi == null || _me == null)
                yield break;

            _obstructions.Clear();
            _wcApi.GetObstructions(_me, _obstructions);

            // Log details periodically
            // bool shouldLog = _log != null && (++_logCounter >= LOG_INTERVAL);
            // if (shouldLog)
            // {
            //     _logCounter = 0;
            //     _log($"WC:GetObstructions raw={_obstructions.Count}");
            //     foreach (var info in _obstructions)
            //     {
            //         double dist = Vector3D.Distance(position, info.Position);
            //         string isSelf = info.EntityId == _myGridId ? " [SELF]" : "";
            //         _log($"  {info.Type}: {info.Name} d={dist:F0}m{isSelf}");
            //     }
            // }

            double radiusSq = radius * radius;

            foreach (var info in _obstructions)
            {
                // Skip our own grid
                if (info.EntityId == _myGridId)
                    continue;

                // Filter by distance
                double distSq = Vector3D.DistanceSquared(position, info.Position);
                if (distSq > radiusSq)
                    continue;

                // Estimate radius from bounding box
                double obstacleRadius = EstimateRadius(info.BoundingBox);

                yield return new Obstacle(
                    info.EntityId,
                    info.Position,
                    info.Velocity,
                    obstacleRadius,
                    ClassifyObstacle(info)
                );
            }
        }

        /// <summary>
        /// Estimates obstacle radius from bounding box.
        /// Uses half the longest dimension as a conservative estimate.
        /// </summary>
        private static double EstimateRadius(BoundingBoxD bbox)
        {
            Vector3D size = bbox.Size;
            double maxDimension = Math.Max(size.X, Math.Max(size.Y, size.Z));
            return maxDimension * 0.5;
        }

        /// <summary>
        /// Classifies the obstacle type based on MyDetectedEntityInfo.
        /// </summary>
        private static ObstacleType ClassifyObstacle(MyDetectedEntityInfo info)
        {
            // Small grids moving fast are likely drones
            // This is a heuristic - adjust thresholds as needed

            // Small grid threshold (~1000 m³ = 10x10x10)
            if (info.BoundingBox.Volume < 1000 && info.Velocity.LengthSquared() > 1)
                return ObstacleType.Drone;

            return ObstacleType.Grid;
        }
    }
}
