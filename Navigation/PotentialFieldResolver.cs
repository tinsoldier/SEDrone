using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Computes repulsive velocity adjustments based on nearby obstacles.
    /// Uses an inverse-square potential field model with per-drone randomization.
    /// Handles terrain avoidance internally via ship controller altitude.
    /// </summary>
    public class PotentialFieldResolver
    {
        private readonly IObstacleProvider _obstacleProvider;
        private readonly PotentialFieldConfig _config;
        private readonly IMyShipController _reference;
        private readonly Action<string> _log;

        // Per-drone personality (derived from EntityId for consistency)
        private readonly double _personalInfluenceMultiplier;
        private readonly double _personalStrengthMultiplier;
        private readonly Vector3D _personalBias;

        // Obstacle cache for performance
        private List<Obstacle> _cachedObstacles = new List<Obstacle>();
        private List<KeyValuePair<double, Obstacle>> _tempObstacleList = new List<KeyValuePair<double, Obstacle>>();
        private int _updateCounter = 0;
        private int _lastRawObstacleCount = 0;

        /// <summary>
        /// Creates a resolver with the given provider and configuration.
        /// </summary>
        /// <param name="obstacleProvider">Source of obstacle data (may be null if only terrain avoidance needed).</param>
        /// <param name="config">Tuning parameters.</param>
        /// <param name="reference">Ship controller for terrain altitude queries.</param>
        /// <param name="droneEntityId">Entity ID for deriving per-drone personality.</param>
        /// <param name="log">Optional debug logging action.</param>
        public PotentialFieldResolver(
            IObstacleProvider obstacleProvider,
            PotentialFieldConfig config,
            IMyShipController reference,
            long droneEntityId,
            Action<string> log = null)
        {
            _obstacleProvider = obstacleProvider;
            _config = config;
            _reference = reference;
            _log = log;

            // Derive consistent "personality" from EntityId
            var random = new Random((int)(droneEntityId % int.MaxValue));

            // +/- variation based on RandomnessFactor
            double variation = config.RandomnessFactor;
            _personalInfluenceMultiplier = 1.0 + (random.NextDouble() - 0.5) * variation * 2;
            _personalStrengthMultiplier = 1.0 + (random.NextDouble() - 0.5) * variation * 2;

            // Small random bias direction (helps break symmetry in head-on situations)
            _personalBias = new Vector3D(
                random.NextDouble() - 0.5,
                random.NextDouble() - 0.5,
                random.NextDouble() - 0.5
            ) * variation;
        }

        /// <summary>
        /// Computes the total repulsion velocity to add to the drone's desired velocity.
        /// </summary>
        /// <param name="currentPosition">Drone's current world position.</param>
        /// <param name="currentVelocity">Drone's current velocity (for future predictive use).</param>
        /// <param name="exclusions">Entity IDs to ignore (e.g., docking target, leader).</param>
        /// <param name="disableTerrainRepulsion">When true, skips gravity/terrain avoidance.</param>
        /// <returns>Velocity adjustment vector to add to desired velocity.</returns>
        public Vector3D ComputeRepulsion(
            Vector3D currentPosition,
            Vector3D currentVelocity,
            HashSet<long> exclusions,
            bool disableTerrainRepulsion = false)
        {
            if (!_config.Enabled)
                return Vector3D.Zero;

            Vector3D totalRepulsion = Vector3D.Zero;

            // Terrain repulsion (handled internally, no provider needed)
            if (!disableTerrainRepulsion)
            {
                Vector3D terrainRepulsion = ComputeTerrainRepulsion();
                totalRepulsion += terrainRepulsion;
            }

            // Obstacle repulsion from providers
            if (_obstacleProvider != null)
            {
                MaybeUpdateCache(currentPosition);

                foreach (var obstacle in _cachedObstacles)
                {
                    if (exclusions != null && exclusions.Contains(obstacle.EntityId))
                        continue;

                    Vector3D repulsion = ComputeObstacleRepulsion(currentPosition, obstacle);
                    totalRepulsion += repulsion;
                }
            }

            // Add personal bias to help break symmetric deadlocks
            if (totalRepulsion.LengthSquared() > 0.01)
            {
                totalRepulsion += _personalBias;
            }

            return ClampMagnitude(totalRepulsion, _config.MaxRepulsionSpeed);
        }

        /// <summary>
        /// Computes terrain repulsion directly from ship controller altitude.
        /// </summary>
        private Vector3D ComputeTerrainRepulsion()
        {
            if (_reference == null)
                return Vector3D.Zero;

            // Check if we're near a planet
            Vector3D gravity = _reference.GetNaturalGravity();
            if (gravity.LengthSquared() < 0.01)
                return Vector3D.Zero; // No gravity = no ground to avoid

            double altitude;
            if (!_reference.TryGetPlanetElevation(MyPlanetElevation.Surface, out altitude))
                return Vector3D.Zero;

            // Only care if within influence radius
            if (altitude > _config.InfluenceRadius)
                return Vector3D.Zero;

            // "Up" is opposite gravity
            Vector3D up = -Vector3D.Normalize(gravity);

            // Inverse-square with terrain multiplier
            double strength = (_config.RepulsionStrength * _config.TerrainMultiplier * _personalStrengthMultiplier)
                            / (altitude * altitude);
            strength = Math.Min(strength, _config.MaxRepulsionSpeed);

            return up * strength;
        }

        /// <summary>
        /// Computes repulsion from a single obstacle.
        /// </summary>
        private Vector3D ComputeObstacleRepulsion(Vector3D myPosition, Obstacle obstacle)
        {
            // Vector from obstacle to me (direction to flee)
            Vector3D toMe = myPosition - obstacle.Position;
            double distance = toMe.Length();

            // Account for obstacle radius (distance to surface, not center)
            double effectiveDistance = Math.Max(0.1, distance - obstacle.Radius);

            if (effectiveDistance < 0.1)
            {
                // Emergency: pick any direction with max force
                if (toMe.LengthSquared() > 0.001)
                    return Vector3D.Normalize(toMe) * _config.MaxRepulsionSpeed;
                return Vector3D.Up * _config.MaxRepulsionSpeed;
            }

            Vector3D direction = toMe / distance;

            // Inverse-square falloff: strength = k / d^2
            double strength = (_config.RepulsionStrength * _personalStrengthMultiplier)
                            / (effectiveDistance * effectiveDistance);

            // Apply type multiplier
            strength *= GetTypeMultiplier(obstacle.Type);

            // Clamp individual obstacle contribution
            strength = Math.Min(strength, _config.MaxRepulsionSpeed);

            return direction * strength;
        }

        /// <summary>
        /// Returns the type-specific repulsion multiplier.
        /// </summary>
        private double GetTypeMultiplier(ObstacleType type)
        {
            switch (type)
            {
                case ObstacleType.Drone:
                    return _config.DroneMultiplier;
                default:
                    return 1.0;
            }
        }

        /// <summary>
        /// Updates the obstacle cache if the update interval has elapsed.
        /// </summary>
        private void MaybeUpdateCache(Vector3D position)
        {
            if (++_updateCounter < _config.UpdateInterval)
                return;

            _updateCounter = 0;
            _cachedObstacles.Clear();

            double effectiveRadius = _config.InfluenceRadius * _personalInfluenceMultiplier;
            var obstacles = _obstacleProvider.GetNearbyObstacles(position, effectiveRadius);

            // Collect and sort by distance, take nearest N (reuse pooled list)
            _tempObstacleList.Clear();
            foreach (var obstacle in obstacles)
            {
                double distSq = Vector3D.DistanceSquared(position, obstacle.Position);
                _tempObstacleList.Add(new KeyValuePair<double, Obstacle>(distSq, obstacle));
            }

            _lastRawObstacleCount = _tempObstacleList.Count;

            // Sort by distance (nearest first)
            _tempObstacleList.Sort((a, b) => a.Key.CompareTo(b.Key));

            // Take up to MaxObstacles
            int count = Math.Min(_tempObstacleList.Count, _config.MaxObstacles);
            for (int i = 0; i < count; i++)
            {
                _cachedObstacles.Add(_tempObstacleList[i].Value);
            }

            // // Log obstacle counts
            // if (_log != null && _lastRawObstacleCount > 0)
            // {
            //     double nearestDist = count > 0 ? Math.Sqrt(_tempObstacleList[0].Key) : 0;
            //     _log($"PF: {_lastRawObstacleCount} raw, {count} used, nearest={nearestDist:F1}m");
            // }
        }

        /// <summary>
        /// Clamps a vector to a maximum magnitude.
        /// </summary>
        private static Vector3D ClampMagnitude(Vector3D vector, double maxMagnitude)
        {
            double lengthSq = vector.LengthSquared();
            if (lengthSq > maxMagnitude * maxMagnitude)
            {
                return vector * (maxMagnitude / Math.Sqrt(lengthSq));
            }
            return vector;
        }
    }
}
