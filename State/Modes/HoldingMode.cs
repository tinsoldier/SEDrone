using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Holding formation position.
    /// Matches leader's compass heading (yaw only, stays level to gravity).
    /// Uses gentle corrections to maintain station.
    /// 
    /// Transitions:
    /// - → SearchingMode: If leader contact is lost
    /// - → ApproachingMode: If drifted too far from formation (hysteresis threshold)
    /// </summary>
    public class HoldingMode : IDroneMode
    {
        public string Name => "Holding";

        // Hysteresis threshold: must exceed this multiplier of StationRadius to exit
        private const double EXIT_THRESHOLD_MULTIPLIER = 2.5;

        public void Enter(DroneBrain brain)
        {
            brain.Echo?.Invoke($"[{Name}] Formation position reached");
        }

        public IDroneMode Update(DroneBrain brain)
        {
            // Check for leader loss
            if (!brain.HasLeaderContact)
            {
                return new SearchingMode();
            }

            // Calculate formation position
            Vector3D formationPos = brain.Navigator.CalculateFormationPosition(
                brain.LastLeaderState,
                brain.Config.StationOffset
            );

            // Apply terrain clearance adjustment
            formationPos = brain.Navigator.AdjustForTerrainClearance(
                formationPos,
                brain.Context.Reference,
                brain.Config.MinTerrainClearance
            );

            // Check distance
            double distance = Vector3D.Distance(brain.Position, formationPos);
            
            // Calculate braking metrics for velocity-aware formation keeping
            Vector3D toFormation = formationPos - brain.Position;
            double currentSpeed = brain.Velocity.Length();
            double brakingDistance = brain.Thrusters.GetBrakingDistance(currentSpeed, brain.Velocity);
            double safeSpeed = brain.Thrusters.GetSafeApproachSpeed(distance, toFormation);

            // Check for excessive drift using velocity-aware tolerance
            // At high speed, trailing by braking distance is acceptable
            if (brain.Navigator.HasExitedFormationWithBrakingMargin(distance, brakingDistance, EXIT_THRESHOLD_MULTIPLIER))
            {
                return new ApproachingMode();
            }

            // Calculate desired velocity with safe-speed cap
            // This prevents exceeding a speed we can't brake from
            Vector3D desiredVelocity = brain.Navigator.CalculateDesiredVelocity(
                brain.Position,
                brain.Velocity,
                formationPos,
                brain.LastLeaderState.Velocity,
                safeSpeed
            );

            // Orientation: match leader's compass heading (yaw only, stay level)
            brain.Gyros.MatchCompassHeading(
                brain.LastLeaderState.Forward,
                brain.LastLeaderState.Up
            );

            // Movement: maintain station
            brain.Thrusters.MoveToward(
                formationPos,
                desiredVelocity,
                brain.Config.MaxSpeed,
                brain.Config.PrecisionRadius
            );

            // Update brain's cached formation data for status display
            brain.UpdateFormationData(formationPos, distance);

            return this;
        }

        public void Exit(DroneBrain brain)
        {
            brain.Echo?.Invoke($"[{Name}] Left formation");
        }
    }
}
