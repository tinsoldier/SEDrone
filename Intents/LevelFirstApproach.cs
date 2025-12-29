using System;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Phase within the LevelFirst approach maneuver.
    /// </summary>
    public enum ApproachPhase
    {
        Leveling,
        Turning,
        Approaching,
        AltitudeAdjust
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

        // State maintained across ticks
        private ApproachPhase _approachPhase = ApproachPhase.Leveling;
        private const double LEVEL_TURN_COMPLETE_THRESHOLD = 30.0;

        public ApproachPhase CurrentPhase => _approachPhase;

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

        public void Execute(DroneContext ctx)
        {
            Vector3D target = Target;
            Vector3D toTarget = target - ctx.Position;
            double distance = toTarget.Length();

            double speedLimit = SpeedLimit > 0 ? SpeedLimit : ctx.Config.MaxSpeed;
            double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toTarget);
            safeSpeed = Math.Min(safeSpeed, speedLimit);

            Vector3D gravity = ctx.Reference.GetNaturalGravity();
            Vector3D worldUp = gravity.LengthSquared() > 0.1
                ? -Vector3D.Normalize(gravity)
                : ctx.Reference.WorldMatrix.Up;

            double elevationDelta = Vector3D.Dot(toTarget, worldUp);
            Vector3D horizontalToTarget = toTarget - worldUp * elevationDelta;
            double horizontalDistance = horizontalToTarget.Length();
            double verticalDistance = Math.Abs(elevationDelta);

            bool targetMostlyVertical = (verticalDistance > horizontalDistance * 3.0) && (horizontalDistance < 10.0);
            Vector3D leaderVelocity = ctx.HasLeaderContact ? ctx.LastLeaderState.Velocity : Vector3D.Zero;

            switch (_approachPhase)
            {
                case ApproachPhase.Leveling:
                    ctx.Gyros.OrientLevel();
                    ctx.Thrusters.Release();

                    if (ctx.Gyros.IsLevel(5.0))
                    {
                        _approachPhase = targetMostlyVertical ? ApproachPhase.AltitudeAdjust : ApproachPhase.Turning;
                    }
                    break;

                case ApproachPhase.Turning:
                    if (targetMostlyVertical)
                    {
                        _approachPhase = ApproachPhase.AltitudeAdjust;
                        break;
                    }

                    ctx.Gyros.LevelTurnToward(target);

                    // Hover in place while turning
                    Vector3D hoverVelocity = ctx.Navigator.CalculateDesiredVelocity(
                        ctx.Position, ctx.Velocity, ctx.Position, leaderVelocity, safeSpeed * 0.5);
                    ctx.Thrusters.MoveToward(ctx.Position, hoverVelocity, safeSpeed * 0.5, ctx.Config.PrecisionRadius);

                    if (ctx.Gyros.IsFacingTarget(target, LEVEL_TURN_COMPLETE_THRESHOLD))
                    {
                        _approachPhase = ApproachPhase.Approaching;
                    }
                    break;

                case ApproachPhase.Approaching:
                    if (targetMostlyVertical)
                    {
                        _approachPhase = ApproachPhase.AltitudeAdjust;
                        break;
                    }

                    ctx.Gyros.LevelTurnToward(target);

                    if (horizontalDistance > ctx.Config.StationRadius * 2)
                    {
                        Vector3D horizontalTarget = target - worldUp * elevationDelta;
                        Vector3D desiredVelocity = ctx.Navigator.CalculateDesiredVelocity(
                            ctx.Position, ctx.Velocity, horizontalTarget, leaderVelocity, safeSpeed);
                        ctx.Thrusters.MoveToward(horizontalTarget, desiredVelocity, safeSpeed, ctx.Config.PrecisionRadius);
                    }
                    else
                    {
                        _approachPhase = ApproachPhase.AltitudeAdjust;
                    }
                    break;

                case ApproachPhase.AltitudeAdjust:
                    if (targetMostlyVertical)
                    {
                        ctx.Gyros.OrientLevel();
                    }
                    else
                    {
                        ctx.Gyros.LevelTurnToward(target);
                    }

                    Vector3D finalVelocity = ctx.Navigator.CalculateDesiredVelocity(
                        ctx.Position, ctx.Velocity, target, leaderVelocity, safeSpeed * 0.5);
                    ctx.Thrusters.MoveToward(target, finalVelocity, safeSpeed * 0.5, ctx.Config.PrecisionRadius);
                    break;
            }
        }
    }
}
