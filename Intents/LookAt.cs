using System;
using VRageMath;

namespace IngameScript
{
    // Note: IOrientationBehavior interface is defined in PositionBehavior.cs
    // to allow coupled behaviors that implement both interfaces.

    /// <summary>
    /// Look at a specific world position.
    /// </summary>
    public class LookAt : IOrientationBehavior
    {
        private readonly Func<Vector3D> _targetFunc;

        /// <summary>Target position to look at (evaluated each tick if dynamic).</summary>
        public Vector3D Target => _targetFunc();

        /// <summary>Look at fixed position.</summary>
        public LookAt(Vector3D target)
        {
            _targetFunc = () => target;
        }

        /// <summary>Look at dynamically-computed position.</summary>
        public LookAt(Func<Vector3D> targetFunc)
        {
            _targetFunc = targetFunc;
        }

        public void Execute(DroneContext ctx)
        {
            ctx.Gyros.LookAt(Target);
        }
    }
}
