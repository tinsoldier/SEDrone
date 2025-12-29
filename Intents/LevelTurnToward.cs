using System;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Turn toward a target position using level turning (yaw only, no pitch).
    /// Locks pitch and roll to horizon during the turn to prevent rolling.
    /// </summary>
    public class LevelTurnToward : IOrientationBehavior
    {
        private readonly Func<Vector3D> _targetFunc;

        /// <summary>Target position to turn toward (evaluated each tick if dynamic).</summary>
        public Vector3D Target => _targetFunc();

        /// <summary>Turn toward fixed position.</summary>
        public LevelTurnToward(Vector3D target)
        {
            _targetFunc = () => target;
        }

        /// <summary>Turn toward dynamically-computed position.</summary>
        public LevelTurnToward(Func<Vector3D> targetFunc)
        {
            _targetFunc = targetFunc;
        }

        public void Execute(DroneContext ctx)
        {
            ctx.Gyros.LevelTurnToward(Target);
        }
    }
}
