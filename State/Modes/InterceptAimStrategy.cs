using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Interface for intercept aiming strategies.
    /// Strategies determine how the drone orients toward incoming missiles.
    /// </summary>
    public interface IInterceptAimStrategy
    {
        /// <summary>
        /// Strategy name for display/logging.
        /// </summary>
        string Name { get; }

        /// <summary>
        /// Calculates the direction to aim based on missile positions.
        /// </summary>
        /// <param name="dronePos">Current drone position</param>
        /// <param name="droneForward">Current drone forward vector</param>
        /// <param name="projectilePositions">List of missile world positions</param>
        /// <returns>Normalized direction to aim, or Zero if no valid target</returns>
        Vector3D CalculateAimDirection(Vector3D dronePos, Vector3D droneForward, List<Vector3D> projectilePositions);
    }

    /// <summary>
    /// Aims at the weighted centroid (center of mass) of all missiles.
    /// Good for engaging swarms with area weapons or when missiles come from similar directions.
    /// 
    /// Weighting:
    /// - Closer missiles weighted more heavily (1/d²)
    /// - Missiles behind drone have reduced weight
    /// - Two-pass outlier rejection removes scattered missiles
    /// </summary>
    public class CentroidAimStrategy : IInterceptAimStrategy
    {
        // === Configuration ===
        private const double OUTLIER_ANGLE_THRESHOLD = 60.0;  // degrees - missiles beyond this from centroid are outliers
        private const double BEHIND_WEIGHT_FACTOR = 0.1;      // Weight multiplier for missiles behind us
        private const double MIN_DISTANCE = 50.0;             // Ignore missiles closer than this (probably past us)
        private const double MAX_DISTANCE = 2000.0;           // Ignore missiles further than this (not immediate threat)

        public string Name => "Centroid";

        public Vector3D CalculateAimDirection(Vector3D dronePos, Vector3D droneForward, List<Vector3D> projectilePositions)
        {
            if (projectilePositions == null || projectilePositions.Count == 0)
                return Vector3D.Zero;

            // === FIRST PASS: Rough centroid ===
            Vector3D roughDirection = CalculateWeightedCentroid(dronePos, droneForward, projectilePositions, null);

            if (roughDirection.LengthSquared() < 0.001)
                return Vector3D.Zero;

            roughDirection = Vector3D.Normalize(roughDirection);

            // === SECOND PASS: Filter outliers and recalculate ===
            Vector3D finalDirection = CalculateWeightedCentroid(dronePos, droneForward, projectilePositions, roughDirection);

            if (finalDirection.LengthSquared() < 0.001)
                return roughDirection;  // Fall back to rough if filtering removed everything

            return Vector3D.Normalize(finalDirection);
        }

        private Vector3D CalculateWeightedCentroid(Vector3D myPos, Vector3D myForward,
            List<Vector3D> positions, Vector3D? filterDirection)
        {
            Vector3D weightedSum = Vector3D.Zero;
            double totalWeight = 0;
            double outlierThresholdCos = Math.Cos(OUTLIER_ANGLE_THRESHOLD * Math.PI / 180.0);

            foreach (var projPos in positions)
            {
                Vector3D toProj = projPos - myPos;
                double distance = toProj.Length();

                // Distance filtering
                if (distance < MIN_DISTANCE || distance > MAX_DISTANCE)
                    continue;

                Vector3D dirToProj = toProj / distance;  // Normalize

                // Outlier filtering (second pass only)
                if (filterDirection.HasValue)
                {
                    double dotFilter = Vector3D.Dot(dirToProj, filterDirection.Value);
                    if (dotFilter < outlierThresholdCos)
                        continue;  // More than threshold degrees from centroid - outlier
                }

                // Weight by inverse distance squared (closer = much more important)
                double weight = 1.0 / (distance * distance);

                // Reduce weight for missiles behind us
                double dotForward = Vector3D.Dot(dirToProj, myForward);
                if (dotForward < 0)
                {
                    weight *= BEHIND_WEIGHT_FACTOR;
                }

                weightedSum += dirToProj * weight;
                totalWeight += weight;
            }

            if (totalWeight < 0.0001)
                return Vector3D.Zero;

            return weightedSum / totalWeight;
        }
    }

    /// <summary>
    /// Aggressively aims at the closest missile.
    /// Good for point defense with rapid-tracking weapons - prioritize immediate threats.
    /// 
    /// Behavior:
    /// - Always tracks the nearest missile
    /// - Ignores missiles that are too close (past us) or too far
    /// - Slightly prefers missiles in front over behind
    /// </summary>
    public class ClosestMissileAimStrategy : IInterceptAimStrategy
    {
        // === Configuration ===
        private const double MIN_DISTANCE = 50.0;             // Ignore missiles closer than this (probably past us)
        private const double MAX_DISTANCE = 2000.0;           // Ignore missiles further than this (not immediate threat)
        private const double BEHIND_DISTANCE_PENALTY = 1.5;   // Missiles behind us count as 1.5x further away

        public string Name => "Closest";

        public Vector3D CalculateAimDirection(Vector3D dronePos, Vector3D droneForward, List<Vector3D> projectilePositions)
        {
            if (projectilePositions == null || projectilePositions.Count == 0)
                return Vector3D.Zero;

            Vector3D closestDirection = Vector3D.Zero;
            double closestEffectiveDistance = double.MaxValue;

            foreach (var projPos in projectilePositions)
            {
                Vector3D toProj = projPos - dronePos;
                double distance = toProj.Length();

                // Distance filtering
                if (distance < MIN_DISTANCE || distance > MAX_DISTANCE)
                    continue;

                Vector3D dirToProj = toProj / distance;  // Normalize

                // Apply penalty for missiles behind us (prefer engaging what's in front)
                double effectiveDistance = distance;
                double dotForward = Vector3D.Dot(dirToProj, droneForward);
                if (dotForward < 0)
                {
                    effectiveDistance *= BEHIND_DISTANCE_PENALTY;
                }

                // Track closest
                if (effectiveDistance < closestEffectiveDistance)
                {
                    closestEffectiveDistance = effectiveDistance;
                    closestDirection = dirToProj;
                }
            }

            return closestDirection;  // Already normalized, or Zero if none found
        }
    }
}
