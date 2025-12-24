using VRageMath;

namespace IngameScript
{
    public class DroneConfig
    {
        // Station keeping offset (target-local coordinates)
        // X = right, Y = up, Z = backward (behind target)
        Vector3D StationOffset;

        // Target identification
        string TargetGridName;
        long TargetEntityId;        // Alternative: direct ID

        // Flight parameters
        double MaxSpeed;            // Absolute speed cap
        double ApproachSpeed;       // Speed when far from station
        double PrecisionRadius;     // Distance at which to slow down
        double HoverAltitude;       // Ground-relative altitude

        // Tuning
        PIDGains PositionPID;
        PIDGains AltitudePID;
        PIDGains OrientationPID;
    }

    struct PIDGains
    {
        double Kp, Ki, Kd;
        double IntegralLimit;
    }
}