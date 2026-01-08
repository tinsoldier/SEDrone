using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Provides obstacles for refhack drones by sharing positions of peer drone grids.
    /// </summary>
    public class RefHackDroneObstacleProvider : IObstacleProvider
    {
        private readonly Func<IEnumerable<BrainContext>> _getContexts;
        private readonly long _myGridId;

        public RefHackDroneObstacleProvider(Func<IEnumerable<BrainContext>> getContexts, long myGridId)
        {
            _getContexts = getContexts;
            _myGridId = myGridId;
        }

        public IEnumerable<Obstacle> GetNearbyObstacles(Vector3D position, double radius)
        {
            if (_getContexts == null)
                yield break;

            double radiusSq = radius * radius;

            foreach (var ctx in _getContexts())
            {
                if (ctx == null || ctx.GridId == 0 || ctx.GridId == _myGridId)
                    continue;

                var reference = ctx.Reference;
                if (reference == null)
                    continue;

                Vector3D obstaclePos = reference.GetPosition();
                if (Vector3D.DistanceSquared(position, obstaclePos) > radiusSq)
                    continue;

                Vector3D obstacleVel = reference.GetShipVelocities().LinearVelocity;
                double obstacleRadius = EstimateGridRadius(reference.CubeGrid);

                yield return new Obstacle(
                    ctx.GridId,
                    obstaclePos,
                    obstacleVel,
                    obstacleRadius,
                    ObstacleType.Drone);
            }
        }

        private static double EstimateGridRadius(IMyCubeGrid grid)
        {
            if (grid == null)
                return 1.0;

            Vector3D size = grid.WorldAABB.Size;
            double maxDim = Math.Max(size.X, Math.Max(size.Y, size.Z));
            return Math.Max(1.0, maxDim * 0.5);
        }
    }
}
