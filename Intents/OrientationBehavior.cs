using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Base class for orientation behaviors - describes WHERE the drone should look.
    /// Interpreted by OrientationExecutor.
    /// </summary>
    public abstract class OrientationBehavior { }

    /// <summary>
    /// Look at a specific world position.
    /// </summary>
    public class LookAt : OrientationBehavior
    {
        public Vector3D Target { get; }

        public LookAt(Vector3D target)
        {
            Target = target;
        }
    }

    /// <summary>
    /// Look in a specific direction (normalized vector).
    /// </summary>
    public class LookDirection : OrientationBehavior
    {
        public Vector3D Direction { get; }

        public LookDirection(Vector3D direction)
        {
            Direction = direction;
        }
    }

    /// <summary>
    /// Match leader's compass heading (yaw only, stays level to gravity).
    /// </summary>
    public class MatchLeader : OrientationBehavior { }

    /// <summary>
    /// Orient toward the closest detected threat.
    /// Executor handles threat lookup from TacticalContext.
    /// </summary>
    public class FaceClosestThreat : OrientationBehavior { }

    /// <summary>
    /// Maintain level orientation (no pitch/roll relative to gravity).
    /// </summary>
    public class StayLevel : OrientationBehavior { }
}
