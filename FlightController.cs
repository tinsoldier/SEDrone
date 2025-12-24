using IngameScript.Utility;
using Sandbox.ModAPI.Ingame;

namespace IngameScript
{
    class FlightController
    {
        // Hardware
        ThrusterManager Thrusters;
        GyroController Gyros;
        IMyShipController Reference;

        // Controllers
        PIDController3D PositionPID;
        PIDController AltitudePID;

        // State
        Vector3D CurrentVelocity;
        Vector3D CurrentPosition;
        double CurrentAltitude;

        // Methods
        void Update(Vector3D desiredPosition, Vector3D desiredVelocity);
        void ApplyThrust(Vector3D worldThrust);
        void SetOrientation(Vector3D forward, Vector3D up);
    }

    class ThrusterManager
    {
        // Thrusters grouped by facing direction (world-relative)
        Dictionary<Base6Directions.Direction, List<IMyThrust>> ThrusterGroups;
        Dictionary<Base6Directions.Direction, double> MaxThrustByDirection;

        // Methods
        void DiscoverThrusters();
        void SetWorldThrust(Vector3D thrust);   // Distributes to appropriate thrusters
        void SetThrustPercent(Base6Directions.Direction dir, double percent);
    }

    class GyroController
    {
        List<IMyGyro> Gyros;
        PIDController3D OrientationPID;

        void SetTargetOrientation(MatrixD target);
        void Update();
        void ReleaseControl();
    }
}