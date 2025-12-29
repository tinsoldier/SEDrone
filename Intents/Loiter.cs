using System;
using VRageMath;

namespace IngameScript
{
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

        public void Execute(DroneContext ctx)
        {
            double distance = Vector3D.Distance(ctx.Position, Center);

            if (distance > Radius)
            {
                Vector3D toCenter = Center - ctx.Position;
                double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toCenter);
                Vector3D desiredVelocity = ctx.Navigator.CalculateDesiredVelocity(
                    ctx.Position, ctx.Velocity, Center, Vector3D.Zero, safeSpeed * 0.5);
                ctx.Thrusters.MoveToward(Center, desiredVelocity, safeSpeed * 0.5, ctx.Config.PrecisionRadius);
            }
            else
            {
                ctx.Thrusters.MoveToward(ctx.Position, Vector3D.Zero, ctx.Config.MaxSpeed, ctx.Config.PrecisionRadius);
            }
        }
    }
}
