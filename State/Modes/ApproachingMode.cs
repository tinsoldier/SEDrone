using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Moving toward formation position.
    /// Faces the destination waypoint and uses approach velocity calculation.
    /// 
    /// Transitions:
    /// - → SearchingMode: If leader contact is lost
    /// - → HoldingMode: When within StationRadius of formation position
    /// </summary>
    public class ApproachingMode : IDroneMode
    {
        public string Name => "Approaching";

        public void Enter(DroneBrain brain)
        {
            brain.Echo?.Invoke($"[{Name}] Moving to formation position");
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

            // Check distance and calculate braking metrics
            double distance = Vector3D.Distance(brain.Position, formationPos);
            Vector3D toFormation = formationPos - brain.Position;
            double currentSpeed = brain.Velocity.Length();
            double brakingDistance = brain.Thrusters.GetBrakingDistance(currentSpeed, brain.Velocity);
            double safeSpeed = brain.Thrusters.GetSafeApproachSpeed(distance, toFormation);

            // Check for arrival at formation (velocity-aware)
            // At high speed, being within braking distance counts as "arrived"
            if (brain.Navigator.IsInFormationWithBrakingMargin(distance, brakingDistance))
            {
                return new HoldingMode();
            }

            // Calculate desired velocity with safe-speed cap
            Vector3D desiredVelocity = brain.Navigator.CalculateDesiredVelocity(
                brain.Position,
                brain.Velocity,
                formationPos,
                brain.LastLeaderState.Velocity,
                safeSpeed
            );

            // Orientation: face destination waypoint
            brain.Gyros.LookAt(formationPos);

            // Movement: approach formation position
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
            // Nothing special needed
        }
    }
}
