using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Classification of obstacle types for varying avoidance responses.
    /// </summary>
    public enum ObstacleType
    {
        /// <summary>Static or slow-moving structures (stations, large ships).</summary>
        Grid,

        /// <summary>Other drones in the swarm (higher priority avoidance).</summary>
        Drone
    }

    /// <summary>
    /// Represents an obstacle that creates a repulsive potential field.
    /// </summary>
    public struct Obstacle
    {
        /// <summary>
        /// Entity ID for exclusion matching. Use -1 for synthetic obstacles (terrain).
        /// </summary>
        public long EntityId;

        /// <summary>
        /// Center position of the obstacle in world space.
        /// </summary>
        public Vector3D Position;

        /// <summary>
        /// Velocity of the obstacle (for predictive avoidance).
        /// </summary>
        public Vector3D Velocity;

        /// <summary>
        /// Approximate radius of the obstacle for distance calculations.
        /// For grids, this is half the longest bounding box dimension.
        /// </summary>
        public double Radius;

        /// <summary>
        /// Type of obstacle, affecting avoidance behavior.
        /// </summary>
        public ObstacleType Type;

        /// <summary>
        /// Creates an obstacle from position and radius.
        /// </summary>
        public Obstacle(long entityId, Vector3D position, Vector3D velocity, double radius, ObstacleType type)
        {
            EntityId = entityId;
            Position = position;
            Velocity = velocity;
            Radius = radius;
            Type = type;
        }
    }

    /// <summary>
    /// Provides obstacles for potential field calculations.
    /// Implementations gather obstacle data from different sources
    /// (WcApi, shared context, terrain detection, etc.).
    /// </summary>
    public interface IObstacleProvider
    {
        /// <summary>
        /// Returns obstacles near the given position within the specified radius.
        /// Implementations should filter by distance and return only relevant obstacles.
        /// </summary>
        /// <param name="position">Current drone position in world space.</param>
        /// <param name="radius">Maximum distance to consider obstacles.</param>
        /// <returns>Enumerable of nearby obstacles.</returns>
        IEnumerable<Obstacle> GetNearbyObstacles(Vector3D position, double radius);
    }

    /// <summary>
    /// Combines multiple obstacle providers into a single source.
    /// Deduplicates obstacles by EntityId.
    /// </summary>
    public class CompositeObstacleProvider : IObstacleProvider
    {
        private readonly List<IObstacleProvider> _providers = new List<IObstacleProvider>();

        /// <summary>
        /// Creates a composite provider with the given sources.
        /// </summary>
        public CompositeObstacleProvider(params IObstacleProvider[] providers)
        {
            if (providers != null)
            {
                foreach (var provider in providers)
                {
                    if (provider != null)
                        _providers.Add(provider);
                }
            }
        }

        /// <summary>
        /// Adds a provider to this composite.
        /// </summary>
        public void Add(IObstacleProvider provider)
        {
            if (provider != null)
                _providers.Add(provider);
        }

        /// <summary>
        /// Aggregates obstacles from all providers, deduplicating by EntityId.
        /// Terrain obstacles (EntityId = -1) are never deduplicated.
        /// </summary>
        public IEnumerable<Obstacle> GetNearbyObstacles(Vector3D position, double radius)
        {
            var seen = new HashSet<long>();

            foreach (var provider in _providers)
            {
                foreach (var obstacle in provider.GetNearbyObstacles(position, radius))
                {
                    // Terrain obstacles are synthetic and always included
                    if (obstacle.EntityId == -1)
                    {
                        yield return obstacle;
                        continue;
                    }

                    // Deduplicate by EntityId
                    if (seen.Add(obstacle.EntityId))
                        yield return obstacle;
                }
            }
        }
    }
}
