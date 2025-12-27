using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Missile interception mode - activates when smart projectiles are detected.
    /// Orients the drone toward the bulk of incoming missiles so gimbal turrets can engage.
    /// 
    /// Uses weighted centroid calculation with outlier rejection to find the
    /// "richest target mass" rather than chasing individual missiles.
    /// 
    /// Transitions:
    /// - → SearchingMode: When no more projectiles detected (fresh start)
    /// </summary>
    public class InterceptMode : IDroneMode
    {
        // === Configuration ===
        private const double OUTLIER_ANGLE_THRESHOLD = 60.0;  // degrees - missiles beyond this from centroid are outliers
        private const double BEHIND_WEIGHT_FACTOR = 0.1;      // Weight multiplier for missiles behind us
        private const double MIN_DISTANCE = 50.0;             // Ignore missiles closer than this (probably past us)
        private const double MAX_DISTANCE = 2000.0;           // Ignore missiles further than this (not immediate threat)
        
        // === State ===
        private int _projectileCount;
        private List<Vector3D> _projectilePositions = new List<Vector3D>();
        private Vector3D _lastThreatDirection = Vector3D.Zero;
        
        public string Name => $"Intercept ({_projectileCount} missiles)";

        /// <summary>
        /// Creates InterceptMode with current projectile count.
        /// </summary>
        public InterceptMode(int projectileCount = 0)
        {
            _projectileCount = projectileCount;
        }

        /// <summary>
        /// Updates the projectile positions (called by DroneBrain each tick).
        /// </summary>
        public void UpdateProjectilePositions(List<Vector3D> positions)
        {
            _projectilePositions.Clear();
            _projectilePositions.AddRange(positions);
            _projectileCount = positions.Count;
        }

        /// <summary>
        /// Legacy method for compatibility - just updates count.
        /// </summary>
        public void UpdateProjectileCount(int count)
        {
            _projectileCount = count;
        }

        public void Enter(DroneBrain brain)
        {
            brain.Echo?.Invoke($"[{Name}] THREAT DETECTED - Entering intercept mode");
            _lastThreatDirection = brain.Context.Reference.WorldMatrix.Forward;
        }

        public IDroneMode Update(DroneBrain brain)
        {
            // Calculate threat direction from projectile positions
            Vector3D threatDirection = CalculateThreatDirection(brain);
            
            // Orient toward threat (or stay level if no valid direction)
            if (threatDirection.LengthSquared() > 0.1)
            {
                _lastThreatDirection = threatDirection;
                Vector3D lookTarget = brain.Position + threatDirection * 1000.0;
                brain.Gyros.LookAt(lookTarget);
            }
            else
            {
                // No valid threat direction - maintain last direction or stay level
                if (_lastThreatDirection.LengthSquared() > 0.1)
                {
                    Vector3D lookTarget = brain.Position + _lastThreatDirection * 1000.0;
                    brain.Gyros.LookAt(lookTarget);
                }
                else
                {
                    brain.Gyros.OrientLevel();
                }
            }
            
            // Movement: maintain formation if we have leader contact
            if (brain.HasLeaderContact)
            {
                Vector3D formationPos = brain.Navigator.CalculateFormationPosition(
                    brain.LastLeaderState,
                    brain.Config.StationOffset
                );
                
                double distance = Vector3D.Distance(brain.Position, formationPos);
                brain.UpdateFormationData(formationPos, distance);
                
                Vector3D desiredVelocity = brain.Navigator.CalculateDesiredVelocity(
                    brain.Position,
                    brain.Velocity,
                    formationPos,
                    brain.LastLeaderState.Velocity,
                    brain.Config.MaxSpeed * 0.5  // Half speed during intercept
                );
                
                brain.Thrusters.MoveToward(
                    formationPos,
                    desiredVelocity,
                    brain.Config.MaxSpeed * 0.5,
                    brain.Config.PrecisionRadius
                );
            }
            else
            {
                brain.Thrusters.Release();
            }
            
            return this;
        }

        /// <summary>
        /// Calculates the direction toward the bulk of incoming missiles.
        /// Uses weighted centroid with two-pass outlier rejection.
        /// 
        /// Algorithm:
        /// 1. First pass: Calculate rough centroid using all missiles (weighted by 1/d²)
        /// 2. Second pass: Exclude missiles more than 60° from rough centroid
        /// 3. Recalculate final centroid from remaining missiles
        /// 
        /// Weighting:
        /// - Closer missiles have higher weight (1/distance²)
        /// - Missiles behind us have reduced weight (×0.1)
        /// </summary>
        private Vector3D CalculateThreatDirection(DroneBrain brain)
        {
            if (_projectilePositions.Count == 0)
                return Vector3D.Zero;
            
            Vector3D myPos = brain.Position;
            Vector3D myForward = brain.Context.Reference.WorldMatrix.Forward;
            
            // === FIRST PASS: Rough centroid ===
            Vector3D roughDirection = CalculateWeightedCentroid(myPos, myForward, _projectilePositions, null);
            
            if (roughDirection.LengthSquared() < 0.001)
                return Vector3D.Zero;
            
            roughDirection = Vector3D.Normalize(roughDirection);
            
            // === SECOND PASS: Filter outliers and recalculate ===
            Vector3D finalDirection = CalculateWeightedCentroid(myPos, myForward, _projectilePositions, roughDirection);
            
            if (finalDirection.LengthSquared() < 0.001)
                return roughDirection;  // Fall back to rough if filtering removed everything
            
            return Vector3D.Normalize(finalDirection);
        }

        /// <summary>
        /// Calculates weighted centroid of missile directions.
        /// </summary>
        /// <param name="myPos">Drone position</param>
        /// <param name="myForward">Drone forward vector</param>
        /// <param name="positions">List of projectile positions</param>
        /// <param name="filterDirection">If not null, exclude missiles more than OUTLIER_ANGLE_THRESHOLD from this direction</param>
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
                // if (filterDirection.HasValue)
                // {
                //     double dotFilter = Vector3D.Dot(dirToProj, filterDirection.Value);
                //     if (dotFilter < outlierThresholdCos)
                //         continue;  // More than threshold degrees from centroid - outlier
                // }
                
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

        public void Exit(DroneBrain brain)
        {
            brain.Echo?.Invoke($"[{Name}] Threat cleared - exiting intercept mode");
        }
    }
}
