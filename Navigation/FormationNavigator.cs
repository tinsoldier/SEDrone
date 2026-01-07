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

        // === Dynamic Formation Calculation ===

        /// <summary>
        /// Calculates a formation offset for a half-circle arrangement around the leader.
        /// Drones spread across 180 degrees, centered on the rotation angle.
        /// </summary>
        /// <param name="index">This drone's index (0-based)</param>
        /// <param name="count">Total number of drones</param>
        /// <param name="radius">Distance from the arc center</param>
        /// <param name="backOffset">Longitudinal offset from leader (negative Z = behind)</param>
        /// <param name="verticalOffset">Vertical offset from leader (Y)</param>
        /// <param name="rotationDegrees">Rotation of formation center (0=front, 180=behind)</param>
        /// <returns>Local offset in leader space (X=right, Y=up, Z=forward)</returns>
        public static Vector3D GetHalfCircleOffset(int index, int count, double radius, double backOffset, double verticalOffset, double rotationDegrees)
        {
            if (count <= 0 || index < 0 || index >= count)
                return new Vector3D(0, verticalOffset, backOffset);

            // Base rotation (convert degrees to radians)
            double baseAngle = rotationDegrees * Math.PI / 180.0;

            // Spread angle for this drone within the formation
            double spreadAngle;
            if (count == 1)
            {
                spreadAngle = 0; // Single drone: at center of formation
            }
            else
            {
                // Spread across ~162 degrees, leaving small gaps at edges
                double sweepAngle = Math.PI * 0.9;
                double halfSweep = sweepAngle / 2;
                double step = sweepAngle / (count - 1);
                spreadAngle = -halfSweep + step * index;
            }

            // Total angle = base rotation + spread offset
            double angle = baseAngle + spreadAngle;

            // Convert angle to offset (in leader-local space)
            // At angle=0: x=0, z=+radius (front)
            // At angle=180°: x=0, z=-radius (behind)
            double x = radius * Math.Sin(angle);
            double z = radius * Math.Cos(angle) + backOffset;

            return new Vector3D(x, verticalOffset, z);
        }

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
            // Rotate local offset into world space (ignore translation)
            Vector3D worldOffset = Vector3D.TransformNormal(localOffset, leader.WorldMatrix);

            // Anchor at leader position
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
