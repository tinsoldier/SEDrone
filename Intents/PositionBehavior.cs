using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Base class for position behaviors - describes WHERE the drone should be.
    /// Interpreted by PositionExecutor.
    /// </summary>
    public abstract class PositionBehavior { }

    /// <summary>
    /// Move toward a target position.
    /// Executor handles approach phases, braking curves, strategy selection.
    /// </summary>
    public class Approach : PositionBehavior
    {
        public Vector3D Target { get; }
        public double SpeedLimit { get; }

        /// <param name="target">World position to approach</param>
        /// <param name="speedLimit">Max speed, or -1 for config default</param>
        public Approach(Vector3D target, double speedLimit = -1)
        {
            Target = target;
            SpeedLimit = speedLimit;
        }
    }

    /// <summary>
    /// Maintain formation offset relative to leader.
    /// Executor handles leader velocity matching, station-keeping corrections.
    /// Used when already in/near formation position.
    /// </summary>
    public class FormationFollow : PositionBehavior
    {
        public Vector3D Offset { get; }
        public bool MatchLeaderVelocity { get; }

        /// <param name="offset">Offset from leader in leader's local space</param>
        /// <param name="matchLeaderVelocity">Whether to match leader's velocity</param>
        public FormationFollow(Vector3D offset, bool matchLeaderVelocity = true)
        {
            Offset = offset;
            MatchLeaderVelocity = matchLeaderVelocity;
        }
    }

    /// <summary>
    /// Loiter in an area with tolerance for drift.
    /// Used when waiting without precise position requirement.
    /// </summary>
    public class Loiter : PositionBehavior
    {
        public Vector3D Center { get; }
        public double Radius { get; }

        /// <param name="center">Center of loiter area</param>
        /// <param name="radius">Acceptable drift radius</param>
        public Loiter(Vector3D center, double radius = 50)
        {
            Center = center;
            Radius = radius;
        }
    }
}
