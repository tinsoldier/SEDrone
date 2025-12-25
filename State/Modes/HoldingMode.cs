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

            // Check for excessive drift (hysteresis exit)
            if (brain.Navigator.HasExitedFormation(distance, EXIT_THRESHOLD_MULTIPLIER))
            {
                return new ApproachingMode();
            }

            // Calculate desired velocity (same algorithm, but we're closer so corrections are gentler)
            Vector3D desiredVelocity = brain.Navigator.CalculateDesiredVelocity(
                brain.Position,
                brain.Velocity,
                formationPos,
                brain.LastLeaderState.Velocity
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
