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
            Vector3D leaderRight = Vector3D.Cross(leader.Forward, leader.Up);

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
        /// Calculates desired velocity for smooth formation flying.
        /// Uses velocity decomposition to prevent oscillation:
        /// - Closing velocity (toward/away from formation) gets proportional correction
        /// - Lateral velocity (perpendicular drift) gets dampened aggressively
        /// 
        /// Optionally caps velocity to a safe braking speed to prevent overshoot.
        /// </summary>
        /// <param name="myPosition">Drone's current world position</param>
        /// <param name="myVelocity">Drone's current world velocity</param>
        /// <param name="formationPosition">Target formation position</param>
        /// <param name="leaderVelocity">Leader's current velocity</param>
        /// <param name="safeSpeed">Optional: Maximum safe speed based on braking capability. Pass 0 to disable.</param>
        /// <returns>Desired world velocity vector</returns>
        public Vector3D CalculateDesiredVelocity(
            Vector3D myPosition,
            Vector3D myVelocity,
            Vector3D formationPosition,
            Vector3D leaderVelocity,
            double safeSpeed = 0)
        {
            Vector3D toFormation = formationPosition - myPosition;
            double distance = toFormation.Length();

            // Start with leader's velocity as base
            Vector3D desired = leaderVelocity;

            // If we're very close, just match leader velocity (no correction needed)
            if (distance <= _config.StationRadius)
            {
                return desired;
            }

            // Calculate direction to formation
            Vector3D directionToFormation = toFormation / distance;

            // Calculate our current velocity relative to the leader
            Vector3D relativeVelocity = myVelocity - leaderVelocity;

            // Decompose relative velocity into components:
            // - Toward/away from formation (closing velocity)
            // - Perpendicular to formation direction (lateral drift)
            double closingSpeed = Vector3D.Dot(relativeVelocity, directionToFormation);
            Vector3D lateralVelocity = relativeVelocity - directionToFormation * closingSpeed;

            // Calculate target correction speed based on distance
            double targetClosingSpeed = CalculateCorrectionSpeed(distance);

            // Calculate closing speed error
            double closingError = targetClosingSpeed - closingSpeed;

            // Apply corrections
            Vector3D closingCorrection = directionToFormation * closingError * CLOSING_GAIN;
            Vector3D lateralDampening = -lateralVelocity * LATERAL_GAIN;

            desired += closingCorrection + lateralDampening;

            // Apply safe-speed cap if provided (braking-aware velocity limiting)
            // This prevents the drone from exceeding a speed it can safely brake from
            if (safeSpeed > 0)
            {
                double desiredSpeed = desired.Length();
                if (desiredSpeed > safeSpeed)
                {
                    desired = desired / desiredSpeed * safeSpeed;
                }
            }

            // Clamp to max speed
            double speed = desired.Length();
            if (speed > _config.MaxSpeed)
            {
                desired = desired / speed * _config.MaxSpeed;
            }

            return desired;
        }

        /// <summary>
        /// Calculates how fast to correct toward formation position based on distance.
        /// Uses precision radius for smooth deceleration near target.
        /// </summary>
        private double CalculateCorrectionSpeed(double distance)
        {
            if (distance < _config.PrecisionRadius)
            {
                // Within precision radius: scale down proportionally
                return _config.ApproachSpeed * (distance / _config.PrecisionRadius);
            }

            // Beyond precision radius: full approach speed
            return _config.ApproachSpeed;
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
