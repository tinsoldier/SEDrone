using VRageMath;

namespace IngameScript
{
    public abstract class NavigationComputer
    {
        // Inputs
        TargetTracker Target;
        DroneConfig Config;

        // Computed outputs
        Vector3D DesiredWorldPosition;
        Vector3D DesiredVelocity;
        Vector3D PositionError;
        double DistanceToStation;

        // Methods
        public abstract void Update();
        public abstract Vector3D CalculateStationPoint();      // Offset → world pos
        public abstract Vector3D CalculateApproachVector();
        public abstract Vector3D CalculateDesiredVelocity();   // Blend correction + matching
    }
}