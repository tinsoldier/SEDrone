using System;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Handles all formation position and velocity calculations.
    /// Extracted from DroneBrain to provide reusable navigation logic.
    /// 
    /// This class is mostly stateless - calculations are based on parameters.
    /// Config reference is kept for accessing tuning values.
    /// </summary>
    public class FormationNavigator
    {
        private readonly DroneConfig _config;

        // === Velocity calculation gains ===
        // These control the responsiveness of formation keeping
        private const double CLOSING_GAIN = 0.5;  // How aggressively to correct closing speed
        private const double LATERAL_GAIN = 1.0;  // How aggressively to dampen lateral drift

        public FormationNavigator(DroneConfig config)
        {
            _config = config;
        }

        /// <summary>
        /// Calculates world-space formation position from leader state and local offset.
        /// </summary>
        /// <param name="leader">Current leader state from IGC</param>
        /// <param name="localOffset">Offset in leader-local coords (X=right, Y=up, Z=forward)</param>
        /// <returns>World position where drone should be</returns>
        public Vector3D CalculateFormationPosition(LeaderStateMessage leader, Vector3D localOffset)
        {
            // Build leader's orientation basis vectors
            Vector3D leaderRight = Vector3D.Cross(leader.Up, leader.Forward);

            // Transform offset from leader-local to world space
            Vector3D worldOffset =
                leaderRight * localOffset.X +
                leader.Up * localOffset.Y +
                leader.Forward * localOffset.Z;

            return leader.Position + worldOffset;
        }

        /// <summary>
        /// Adjusts a formation position upward if it would be below minimum terrain clearance.
        /// Uses the drone's current position to estimate terrain at target location.
        /// </summary>
        /// <param name="targetPosition">The raw formation position</param>
        /// <param name="reference">Ship controller for elevation queries</param>
        /// <param name="minClearance">Minimum allowed terrain clearance</param>
        /// <returns>Adjusted position (lifted if necessary)</returns>
        public Vector3D AdjustForTerrainClearance(Vector3D targetPosition, IMyShipController reference, double minClearance)
        {
            // Get gravity direction - we need this to know which way is "up"
            Vector3D gravity = reference.GetNaturalGravity();
            if (gravity.LengthSquared() < 0.1)
            {
                // No gravity = no terrain to worry about
                return targetPosition;
            }

            Vector3D up = -Vector3D.Normalize(gravity);

            // Get our current elevation above terrain
            double currentElevation;
            if (!reference.TryGetPlanetElevation(MyPlanetElevation.Surface, out currentElevation))
            {
                // Can't get elevation - return unadjusted
                return targetPosition;
            }

            // Calculate how much higher/lower the target is compared to us
            Vector3D toTarget = targetPosition - reference.GetPosition();
            double targetHeightDelta = Vector3D.Dot(toTarget, up);

            // Estimate target's elevation: our elevation + height difference
            double estimatedTargetElevation = currentElevation + targetHeightDelta;

            // Check if target is below minimum clearance
            if (estimatedTargetElevation < minClearance)
            {
                // Lift the target position to maintain minimum clearance
                double liftAmount = minClearance - estimatedTargetElevation;
                return targetPosition + up * liftAmount;
            }

            return targetPosition;
        }

        /// <summary>
        /// Checks if the drone is within formation tolerance.
        /// </summary>
        public bool IsInFormation(double distanceToFormation)
        {
            return distanceToFormation <= _config.StationRadius;
        }

        /// <summary>
        /// Checks if the drone is within velocity-aware formation tolerance.
        /// At higher speeds, allows more trailing distance based on braking capability.
        /// </summary>
        /// <param name="distanceToFormation">Current distance to formation point</param>
        /// <param name="brakingDistance">Current braking distance at this speed/direction</param>
        /// <returns>True if within acceptable tolerance (static + braking margin)</returns>
        public bool IsInFormationWithBrakingMargin(double distanceToFormation, double brakingDistance)
        {
            // Handle infinite braking distance (can't brake in this direction) - use static radius only
            if (double.IsInfinity(brakingDistance) || brakingDistance > 10000)
            {
                return distanceToFormation <= _config.StationRadius;
            }
            
            // Effective tolerance = base radius + (braking distance * safety margin)
            double effectiveTolerance = _config.StationRadius + 
                                        (brakingDistance * _config.BrakingSafetyMargin);
            return distanceToFormation <= effectiveTolerance;
        }

        /// <summary>
        /// Checks if the drone has drifted far enough to require reapproach.
        /// Uses hysteresis to prevent mode flickering.
        /// </summary>
        /// <param name="distanceToFormation">Current distance to formation point</param>
        /// <param name="exitMultiplier">Multiplier of StationRadius to trigger exit (default 2.5)</param>
        public bool HasExitedFormation(double distanceToFormation, double exitMultiplier = 2.5)
        {
            return distanceToFormation > _config.StationRadius * exitMultiplier;
        }

        /// <summary>
        /// Velocity-aware version of HasExitedFormation.
        /// Accounts for braking distance when determining if drone has truly exited formation.
        /// </summary>
        /// <param name="distanceToFormation">Current distance to formation point</param>
        /// <param name="brakingDistance">Current braking distance at this speed/direction</param>
        /// <param name="exitMultiplier">Multiplier applied to effective tolerance</param>
        public bool HasExitedFormationWithBrakingMargin(double distanceToFormation, double brakingDistance, double exitMultiplier = 2.5)
        {
            // Handle infinite braking distance (can't brake in this direction) - use static radius only
            if (double.IsInfinity(brakingDistance) || brakingDistance > 10000)
            {
                return distanceToFormation > _config.StationRadius * exitMultiplier;
            }
            
            double effectiveTolerance = _config.StationRadius + 
                                        (brakingDistance * _config.BrakingSafetyMargin);
            return distanceToFormation > effectiveTolerance * exitMultiplier;
        }
    }
}
