using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Provides tactical awareness to directives and executors.
    /// Updated each tick by the Brain before directive execution.
    /// 
    /// This is a simple wrapper around threat data. More sophisticated
    /// threat analysis can be added later without changing directive code.
    /// </summary>
    public class TacticalContext
    {
        private List<Vector3D> _projectilePositions = new List<Vector3D>();

        /// <summary>
        /// Number of detected threats (missiles/projectiles).
        /// </summary>
        public int ThreatCount { get; private set; }

        /// <summary>
        /// Whether any threats are currently detected.
        /// </summary>
        public bool HasThreats => ThreatCount > 0;

        /// <summary>
        /// Whether position data is available for threats.
        /// False means we only have a count (will aim nose-up).
        /// </summary>
        public bool HasPositionData { get; private set; }

        /// <summary>
        /// Updates threat data with positions.
        /// </summary>
        public void UpdateThreats(List<Vector3D> positions)
        {
            _projectilePositions.Clear();
            if (positions != null)
            {
                _projectilePositions.AddRange(positions);
            }
            ThreatCount = _projectilePositions.Count;
            HasPositionData = true;
        }

        /// <summary>
        /// Updates threat data with count only (no positions).
        /// </summary>
        public void UpdateThreatCount(int count)
        {
            _projectilePositions.Clear();
            ThreatCount = count;
            HasPositionData = false;
        }

        /// <summary>
        /// Clears all threat data.
        /// </summary>
        public void ClearThreats()
        {
            _projectilePositions.Clear();
            ThreatCount = 0;
            HasPositionData = false;
        }

        /// <summary>
        /// Gets the position of the closest threat to the given position.
        /// Returns null if no threats with position data.
        /// </summary>
        public Vector3D? GetClosestThreatPosition(Vector3D fromPosition)
        {
            if (_projectilePositions.Count == 0)
                return null;

            Vector3D closest = _projectilePositions[0];
            double closestDistSq = Vector3D.DistanceSquared(fromPosition, closest);

            for (int i = 1; i < _projectilePositions.Count; i++)
            {
                double distSq = Vector3D.DistanceSquared(fromPosition, _projectilePositions[i]);
                if (distSq < closestDistSq)
                {
                    closestDistSq = distSq;
                    closest = _projectilePositions[i];
                }
            }

            return closest;
        }

        /// <summary>
        /// Gets all threat positions (for advanced strategies).
        /// Returns empty list if no position data.
        /// </summary>
        public IReadOnlyList<Vector3D> GetAllThreatPositions()
        {
            return _projectilePositions;
        }
    }
}
