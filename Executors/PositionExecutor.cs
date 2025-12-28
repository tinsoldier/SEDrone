using System;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Approach strategy determines how the drone maneuvers toward a target.
    /// </summary>
    public enum ApproachStrategy
    {
        /// <summary>Direct approach: face target and pitch toward it.</summary>
        Direct,

        /// <summary>Conservative: stay level, yaw to face, approach horizontally, then adjust altitude.</summary>
        LevelFirst
    }

    /// <summary>
    /// Phase within the LevelFirst approach strategy.
    /// </summary>
    public enum ApproachPhase
    {
        Leveling,
        Turning,
        Approaching,
        AltitudeAdjust
    }

    /// <summary>
    /// Executes position behaviors by commanding thrusters.
    /// Owns temporal state for approach phases, braking, and station-keeping.
    /// 
    /// Ported from ApproachingMode and HoldingMode.
    /// </summary>
    public class PositionExecutor
    {
        // === Dependencies ===
        private readonly FormationNavigator _navigator;
        private readonly Action<string> _echo;

        // === Persistent state ===
        private PositionBehavior _currentBehavior;
        private ApproachStrategy _strategy = ApproachStrategy.Direct;
        private ApproachPhase _approachPhase = ApproachPhase.Leveling;
        private bool _strategyLocked = false;

        // === Configuration ===
        private const double OVERSHOOT_ANGLE_THRESHOLD = 90.0;
        private const double ELEVATION_DELTA_THRESHOLD = 20.0;
        private const double LEVEL_TURN_COMPLETE_THRESHOLD = 30.0;
        private const double STRATEGY_UNLOCK_ANGLE = 45.0;
        private const double EXIT_THRESHOLD_MULTIPLIER = 2.5;

        public ApproachPhase CurrentPhase => _approachPhase;
        public ApproachStrategy CurrentStrategy => _strategy;

        public PositionExecutor(FormationNavigator navigator, Action<string> echo = null)
        {
            _navigator = navigator;
            _echo = echo;
        }

        /// <summary>
        /// Called when BehaviorIntent changes. Resets internal state if behavior type changed.
        /// </summary>
        public void OnBehaviorChanged(PositionBehavior newBehavior)
        {
            bool typeChanged = !IsSameBehaviorType(_currentBehavior, newBehavior);
            bool targetChanged = HasTargetChanged(_currentBehavior, newBehavior);

            if (typeChanged || targetChanged)
            {
                _approachPhase = ApproachPhase.Leveling;
                _strategyLocked = false;
            }

            _currentBehavior = newBehavior;
        }

        /// <summary>
        /// Executes the current position behavior.
        /// </summary>
        public void Execute(PositionBehavior behavior, DroneContext ctx)
        {
            if (behavior == null)
            {
                ctx.Thrusters.Release();
                return;
            }

            var approach = behavior as Approach;
            var formation = behavior as FormationFollow;
            var loiter = behavior as Loiter;

            if (approach != null)
                ExecuteApproach(approach, ctx);
            else if (formation != null)
                ExecuteFormationFollow(formation, ctx);
            else if (loiter != null)
                ExecuteLoiter(loiter, ctx);
        }

        private void ExecuteApproach(Approach behavior, DroneContext ctx)
        {
            Vector3D target = behavior.Target;
            double distance = Vector3D.Distance(ctx.Position, target);
            Vector3D toTarget = target - ctx.Position;

            double currentSpeed = ctx.Velocity.Length();
            double brakingDistance = ctx.Thrusters.GetBrakingDistance(currentSpeed, ctx.Velocity);
            double speedLimit = behavior.SpeedLimit > 0 ? behavior.SpeedLimit : ctx.Config.MaxSpeed;
            double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toTarget);
            safeSpeed = Math.Min(safeSpeed, speedLimit);

            // Auto-detect strategy
            ApproachStrategy effectiveStrategy = DetermineStrategy(ctx, target, toTarget);

            if (effectiveStrategy == ApproachStrategy.LevelFirst)
            {
                ExecuteLevelFirstApproach(ctx, target, toTarget, distance, safeSpeed);
            }
            else
            {
                ExecuteDirectApproach(ctx, target, distance, safeSpeed);
            }
        }

        private void ExecuteDirectApproach(DroneContext ctx, Vector3D target, double distance, double safeSpeed)
        {
            Vector3D leaderVelocity = ctx.HasLeaderContact ? ctx.LastLeaderState.Velocity : Vector3D.Zero;

            Vector3D desiredVelocity = _navigator.CalculateDesiredVelocity(
                ctx.Position,
                ctx.Velocity,
                target,
                leaderVelocity,
                safeSpeed
            );

            ctx.Gyros.LookAt(target);
            ctx.Thrusters.MoveToward(target, desiredVelocity, ctx.Config.MaxSpeed, ctx.Config.PrecisionRadius);
        }

        private void ExecuteLevelFirstApproach(DroneContext ctx, Vector3D target, Vector3D toTarget, double distance, double safeSpeed)
        {
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
                    Vector3D hoverVelocity = _navigator.CalculateDesiredVelocity(
                        ctx.Position, ctx.Velocity, ctx.Position, leaderVelocity, safeSpeed * 0.5);
                    ctx.Thrusters.MoveToward(ctx.Position, hoverVelocity, ctx.Config.MaxSpeed, ctx.Config.PrecisionRadius);

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
                        Vector3D desiredVelocity = _navigator.CalculateDesiredVelocity(
                            ctx.Position, ctx.Velocity, horizontalTarget, leaderVelocity, safeSpeed);
                        ctx.Thrusters.MoveToward(horizontalTarget, desiredVelocity, ctx.Config.MaxSpeed, ctx.Config.PrecisionRadius);
                    }
                    else
                    {
                        _approachPhase = ApproachPhase.AltitudeAdjust;
                    }
                    break;

                case ApproachPhase.AltitudeAdjust:
                    if (targetMostlyVertical)
                    {
                        ctx.Gyros.ResetTurnLock();
                        ctx.Gyros.OrientLevel();
                    }
                    else
                    {
                        ctx.Gyros.LevelTurnToward(target);
                    }

                    Vector3D finalVelocity = _navigator.CalculateDesiredVelocity(
                        ctx.Position, ctx.Velocity, target, leaderVelocity, safeSpeed * 0.5);
                    ctx.Thrusters.MoveToward(target, finalVelocity, ctx.Config.MaxSpeed, ctx.Config.PrecisionRadius);
                    break;
            }
        }

        private void ExecuteFormationFollow(FormationFollow behavior, DroneContext ctx)
        {
            if (!ctx.HasLeaderContact)
            {
                ctx.Thrusters.Release();
                return;
            }

            Vector3D formationPos = _navigator.CalculateFormationPosition(ctx.LastLeaderState, behavior.Offset);
            formationPos = _navigator.AdjustForTerrainClearance(formationPos, ctx.Reference, ctx.Config.MinTerrainClearance);

            double distance = Vector3D.Distance(ctx.Position, formationPos);
            Vector3D toFormation = formationPos - ctx.Position;
            double brakingDistance = ctx.Thrusters.GetBrakingDistance(ctx.Velocity.Length(), ctx.Velocity);
            double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toFormation);

            Vector3D leaderVelocity = behavior.MatchLeaderVelocity ? ctx.LastLeaderState.Velocity : Vector3D.Zero;
            Vector3D desiredVelocity = _navigator.CalculateDesiredVelocity(
                ctx.Position, ctx.Velocity, formationPos, leaderVelocity, safeSpeed);

            ctx.Thrusters.MoveToward(formationPos, desiredVelocity, ctx.Config.MaxSpeed, ctx.Config.PrecisionRadius);
        }

        private void ExecuteLoiter(Loiter behavior, DroneContext ctx)
        {
            double distance = Vector3D.Distance(ctx.Position, behavior.Center);

            if (distance > behavior.Radius)
            {
                // Drift back toward center
                Vector3D toCenter = behavior.Center - ctx.Position;
                double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toCenter);
                Vector3D desiredVelocity = _navigator.CalculateDesiredVelocity(
                    ctx.Position, ctx.Velocity, behavior.Center, Vector3D.Zero, safeSpeed * 0.5);
                ctx.Thrusters.MoveToward(behavior.Center, desiredVelocity, ctx.Config.MaxSpeed, ctx.Config.PrecisionRadius);
            }
            else
            {
                // Within radius - just dampen velocity
                ctx.Thrusters.MoveToward(ctx.Position, Vector3D.Zero, ctx.Config.MaxSpeed, ctx.Config.PrecisionRadius);
            }
        }

        private ApproachStrategy DetermineStrategy(DroneContext ctx, Vector3D target, Vector3D toTarget)
        {
            Vector3D forward = ctx.Reference.WorldMatrix.Forward;
            Vector3D toTargetNorm = Vector3D.Normalize(toTarget);
            double dotForward = Vector3D.Dot(forward, toTargetNorm);
            double angleOff = Math.Acos(MathHelper.Clamp(dotForward, -1, 1)) * 180.0 / Math.PI;

            // If locked to LevelFirst, check unlock condition
            if (_strategyLocked && _strategy == ApproachStrategy.LevelFirst)
            {
                if (angleOff < STRATEGY_UNLOCK_ANGLE && _approachPhase >= ApproachPhase.Approaching)
                {
                    _strategyLocked = false;
                }
                return ApproachStrategy.LevelFirst;
            }

            // Auto-detect: target behind us
            if (angleOff > OVERSHOOT_ANGLE_THRESHOLD)
            {
                _strategyLocked = true;
                return ApproachStrategy.LevelFirst;
            }

            // Auto-detect: large elevation difference while not facing target
            Vector3D gravity = ctx.Reference.GetNaturalGravity();
            if (gravity.LengthSquared() > 0.1)
            {
                Vector3D worldUp = -Vector3D.Normalize(gravity);
                double elevationDelta = Math.Abs(Vector3D.Dot(toTarget, worldUp));

                if (elevationDelta > ELEVATION_DELTA_THRESHOLD && angleOff > 25.0)
                {
                    _strategyLocked = true;
                    return ApproachStrategy.LevelFirst;
                }
            }

            return ApproachStrategy.Direct;
        }

        private bool IsSameBehaviorType(PositionBehavior a, PositionBehavior b)
        {
            if (a == null || b == null) return a == b;
            return a.GetType() == b.GetType();
        }

        private bool HasTargetChanged(PositionBehavior a, PositionBehavior b)
        {
            // Compare targets for Approach
            var approachA = a as Approach;
            var approachB = b as Approach;
            if (approachA != null && approachB != null)
            {
                return Vector3D.DistanceSquared(approachA.Target, approachB.Target) > 100; // 10m threshold
            }

            // FormationFollow offset changes don't reset
            return false;
        }
    }
}
