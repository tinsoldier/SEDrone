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
    /// Executes position behaviors by commanding thrusters (and gyros for coupled maneuvers).
    /// Owns temporal state for approach phases, braking, and station-keeping.
    /// </summary>
    public class PositionExecutor
    {
        // === Dependencies ===
        private readonly FormationNavigator _navigator;
        private readonly Action<string> _echo;

        // === Persistent state ===
        private IPositionBehavior _currentBehavior;
        private ApproachPhase _approachPhase = ApproachPhase.Leveling;

        // === Configuration ===
        private const double LEVEL_TURN_COMPLETE_THRESHOLD = 30.0;

        public ApproachPhase CurrentPhase => _approachPhase;

        /// <summary>
        /// True if the current behavior is a coupled maneuver that handles orientation.
        /// </summary>
        public bool HandlesCoupledOrientation => _currentBehavior is IOrientationBehavior;

        public PositionExecutor(FormationNavigator navigator, Action<string> echo = null)
        {
            _navigator = navigator;
            _echo = echo;
        }

        /// <summary>
        /// Called when BehaviorIntent changes. Resets internal state if behavior type changed.
        /// </summary>
        public void OnBehaviorChanged(IPositionBehavior newBehavior)
        {
            bool typeChanged = !IsSameBehaviorType(_currentBehavior, newBehavior);

            if (typeChanged)
            {
                _approachPhase = ApproachPhase.Leveling;
            }

            _currentBehavior = newBehavior;
        }

        /// <summary>
        /// Executes the current position behavior.
        /// Returns true if this behavior also handled orientation (coupled maneuver).
        /// </summary>
        public bool Execute(IPositionBehavior behavior, DroneContext ctx)
        {
            if (behavior == null)
            {
                ctx.Thrusters.Release();
                return false;
            }

            // Coupled maneuvers (handle both position and orientation)
            var levelFirstApproach = behavior as LevelFirstApproach;
            if (levelFirstApproach != null)
            {
                ExecuteLevelFirstApproach(levelFirstApproach, ctx);
                return true;  // Orientation handled
            }

            // Decoupled position behaviors
            var approach = behavior as Approach;
            var formation = behavior as FormationFollow;
            var loiter = behavior as Loiter;

            if (approach != null)
                ExecuteApproach(approach, ctx);
            else if (formation != null)
                ExecuteFormationFollow(formation, ctx);
            else if (loiter != null)
                ExecuteLoiter(loiter, ctx);

            return false;  // Orientation NOT handled
        }

        private void ExecuteApproach(Approach behavior, DroneContext ctx)
        {
            Vector3D target = behavior.Target;
            double distance = Vector3D.Distance(ctx.Position, target);
            Vector3D toTarget = target - ctx.Position;

            double currentSpeed = ctx.Velocity.Length();
            double speedLimit = behavior.SpeedLimit > 0 ? behavior.SpeedLimit : ctx.Config.MaxSpeed;
            double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toTarget);
            safeSpeed = Math.Min(safeSpeed, speedLimit);

            // Determine whether to match leader velocity
            Vector3D targetVelocity = Vector3D.Zero;
            if (behavior.MatchLeaderVelocity && ctx.HasLeaderContact)
            {
                targetVelocity = ctx.LastLeaderState.Velocity;
            }

            Vector3D desiredVelocity = _navigator.CalculateDesiredVelocity(
                ctx.Position,
                ctx.Velocity,
                target,
                targetVelocity,
                safeSpeed
            );

            // Determine precision radius
            // If BypassPrecisionRadius is set, use minimal radius (0.1m) to avoid slowdown
            // Otherwise, use minimal radius (0.5m) if speedLimit is set, or config default
            double precisionRadius = ctx.Config.PrecisionRadius;
            if (behavior.BypassPrecisionRadius)
            {
                precisionRadius = 0.1;
            }
            else if (behavior.SpeedLimit > 0)
            {
                precisionRadius = 0.5;
            }

            ctx.Thrusters.MoveToward(target, desiredVelocity, safeSpeed, precisionRadius);
        }

        private void ExecuteLevelFirstApproach(LevelFirstApproach behavior, DroneContext ctx)
        {
            Vector3D target = behavior.Target;
            Vector3D toTarget = target - ctx.Position;
            double distance = toTarget.Length();

            double speedLimit = behavior.SpeedLimit > 0 ? behavior.SpeedLimit : ctx.Config.MaxSpeed;
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
                    Vector3D hoverVelocity = _navigator.CalculateDesiredVelocity(
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
                        Vector3D desiredVelocity = _navigator.CalculateDesiredVelocity(
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

                    Vector3D finalVelocity = _navigator.CalculateDesiredVelocity(
                        ctx.Position, ctx.Velocity, target, leaderVelocity, safeSpeed * 0.5);
                    ctx.Thrusters.MoveToward(target, finalVelocity, safeSpeed * 0.5, ctx.Config.PrecisionRadius);
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
            double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toFormation);

            Vector3D leaderVelocity = behavior.MatchLeaderVelocity ? ctx.LastLeaderState.Velocity : Vector3D.Zero;
            Vector3D desiredVelocity = _navigator.CalculateDesiredVelocity(
                ctx.Position, ctx.Velocity, formationPos, leaderVelocity, safeSpeed);

            ctx.Thrusters.MoveToward(formationPos, desiredVelocity, safeSpeed, ctx.Config.PrecisionRadius);
        }

        private void ExecuteLoiter(Loiter behavior, DroneContext ctx)
        {
            double distance = Vector3D.Distance(ctx.Position, behavior.Center);

            if (distance > behavior.Radius)
            {
                Vector3D toCenter = behavior.Center - ctx.Position;
                double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toCenter);
                Vector3D desiredVelocity = _navigator.CalculateDesiredVelocity(
                    ctx.Position, ctx.Velocity, behavior.Center, Vector3D.Zero, safeSpeed * 0.5);
                ctx.Thrusters.MoveToward(behavior.Center, desiredVelocity, safeSpeed * 0.5, ctx.Config.PrecisionRadius);
            }
            else
            {
                ctx.Thrusters.MoveToward(ctx.Position, Vector3D.Zero, ctx.Config.MaxSpeed, ctx.Config.PrecisionRadius);
            }
        }

        private bool IsSameBehaviorType(IPositionBehavior a, IPositionBehavior b)
        {
            if (a == null || b == null) return a == b;
            return a.GetType() == b.GetType();
        }
    }
}
