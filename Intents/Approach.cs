using System;
using VRageMath;

namespace IngameScript
{
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

        public void Execute(DroneContext ctx)
        {
            Vector3D target = Target;
            double distance = Vector3D.Distance(ctx.Position, target);
            Vector3D toTarget = target - ctx.Position;

            double speedLimit = SpeedLimit > 0 ? SpeedLimit : ctx.Config.MaxSpeed;
            double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toTarget);
            safeSpeed = Math.Min(safeSpeed, speedLimit);

            // Determine whether to match leader velocity
            Vector3D targetVelocity = Vector3D.Zero;
            if (MatchLeaderVelocity && ctx.HasLeaderContact)
            {
                targetVelocity = ctx.LastLeaderState.Velocity;
            }

            Vector3D desiredVelocity = ctx.Navigator.CalculateDesiredVelocity(
                ctx.Position,
                ctx.Velocity,
                target,
                targetVelocity,
                safeSpeed
            );

            // Determine precision radius
            double precisionRadius = ctx.Config.PrecisionRadius;
            if (BypassPrecisionRadius)
            {
                precisionRadius = 0.1;
            }
            else if (SpeedLimit > 0)
            {
                precisionRadius = 0.5;
            }

            ctx.Thrusters.MoveToward(target, desiredVelocity, safeSpeed, precisionRadius);
        }
    }
}
