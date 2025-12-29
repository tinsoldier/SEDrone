using System;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Look in a specific direction (normalized vector).
    /// </summary>
    public class LookDirection : IOrientationBehavior
    {
        private readonly Func<Vector3D> _directionFunc;

        /// <summary>Direction to look (evaluated each tick if dynamic).</summary>
        public Vector3D Direction => _directionFunc();

        /// <summary>Look in fixed direction.</summary>
        public LookDirection(Vector3D direction)
        {
            _directionFunc = () => direction;
        }

        /// <summary>Look in dynamically-computed direction.</summary>
        public LookDirection(Func<Vector3D> directionFunc)
        {
            _directionFunc = directionFunc;
        }

        public void Execute(DroneContext ctx)
        {
            if (Direction.LengthSquared() < 0.1)
            {
                ctx.Gyros.OrientLevel();
                return;
            }

            Vector3D target = ctx.Position + Vector3D.Normalize(Direction) * 1000.0;
            ctx.Gyros.LookAt(target);
        }
    }
}
