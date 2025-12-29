using System;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Marker interface for position behaviors - describes WHERE the drone should be.
    /// Interpreted by PositionExecutor.
    /// </summary>
    public interface IPositionBehavior { }

    /// <summary>
    /// Marker interface for orientation behaviors - describes WHERE the drone should look.
    /// Interpreted by OrientationExecutor.
    /// </summary>
    public interface IOrientationBehavior { }

    /// <summary>
    /// Move toward a target position. Decoupled - does not control orientation.
    /// Use when you want separate control over where the drone looks.
    /// </summary>
    public class Approach : IPositionBehavior
    {
        private readonly Func<Vector3D> _targetFunc;

        /// <summary>Target position (evaluated each tick if dynamic).</summary>
        public Vector3D Target => _targetFunc();
        public double SpeedLimit { get; }
        public bool MatchLeaderVelocity { get; }
        public bool BypassPrecisionRadius { get; }

        /// <summary>Approach a fixed position.</summary>
        public Approach(Vector3D target, double speedLimit = -1, bool matchLeaderVelocity = false, bool bypassPrecisionRadius = false)
        {
            _targetFunc = () => target;
            SpeedLimit = speedLimit;
            MatchLeaderVelocity = matchLeaderVelocity;
            BypassPrecisionRadius = bypassPrecisionRadius;
        }

        /// <summary>Approach a dynamically-computed position (evaluated each tick).</summary>
        public Approach(Func<Vector3D> targetFunc, double speedLimit = -1, bool matchLeaderVelocity = false, bool bypassPrecisionRadius = false)
        {
            _targetFunc = targetFunc;
            SpeedLimit = speedLimit;
            MatchLeaderVelocity = matchLeaderVelocity;
            BypassPrecisionRadius = bypassPrecisionRadius;
        }
    }

    /// <summary>
    /// Coupled approach maneuver - controls BOTH position AND orientation.
    /// Uses level-first strategy: level grid, yaw to face, approach horizontally, adjust altitude.
    /// Use when approaching from behind or sharp angles where orientation must be sequenced.
    /// </summary>
    public class LevelFirstApproach : IPositionBehavior, IOrientationBehavior
    {
        private readonly Func<Vector3D> _targetFunc;

        /// <summary>Target position (evaluated each tick if dynamic).</summary>
        public Vector3D Target => _targetFunc();
        public double SpeedLimit { get; }

        /// <summary>Approach a fixed position using level-first maneuver.</summary>
        public LevelFirstApproach(Vector3D target, double speedLimit = -1)
        {
            _targetFunc = () => target;
            SpeedLimit = speedLimit;
        }

        /// <summary>Approach a dynamically-computed position using level-first maneuver.</summary>
        public LevelFirstApproach(Func<Vector3D> targetFunc, double speedLimit = -1)
        {
            _targetFunc = targetFunc;
            SpeedLimit = speedLimit;
        }
    }

    /// <summary>
    /// Maintain formation offset relative to leader.
    /// Executor handles leader velocity matching, station-keeping corrections.
    /// </summary>
    public class FormationFollow : IPositionBehavior
    {
        private readonly Func<Vector3D> _offsetFunc;

        /// <summary>Offset from leader in leader's local space (evaluated each tick if dynamic).</summary>
        public Vector3D Offset => _offsetFunc();
        public bool MatchLeaderVelocity { get; }

        /// <summary>Follow at fixed offset.</summary>
        public FormationFollow(Vector3D offset, bool matchLeaderVelocity = true)
        {
            _offsetFunc = () => offset;
            MatchLeaderVelocity = matchLeaderVelocity;
        }

        /// <summary>Follow at dynamically-computed offset (evaluated each tick).</summary>
        public FormationFollow(Func<Vector3D> offsetFunc, bool matchLeaderVelocity = true)
        {
            _offsetFunc = offsetFunc;
            MatchLeaderVelocity = matchLeaderVelocity;
        }
    }

    /// <summary>
    /// Loiter in an area with tolerance for drift.
    /// Used when waiting without precise position requirement.
    /// </summary>
    public class Loiter : IPositionBehavior
    {
        private readonly Func<Vector3D> _centerFunc;

        /// <summary>Center of loiter area (evaluated each tick if dynamic).</summary>
        public Vector3D Center => _centerFunc();
        public double Radius { get; }

        /// <summary>Loiter around fixed position.</summary>
        public Loiter(Vector3D center, double radius = 50)
        {
            _centerFunc = () => center;
            Radius = radius;
        }

        /// <summary>Loiter around dynamically-computed position.</summary>
        public Loiter(Func<Vector3D> centerFunc, double radius = 50)
        {
            _centerFunc = centerFunc;
            Radius = radius;
        }
    }
}
