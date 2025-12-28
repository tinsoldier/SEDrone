using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Executes orientation behaviors by commanding gyros.
    /// Owns temporal state for threat tracking.
    /// </summary>
    public class OrientationExecutor
    {
        // === Dependencies ===
        private readonly Action<string> _echo;

        // === Persistent state ===
        private IOrientationBehavior _currentBehavior;
        private Vector3D _lastThreatDirection = Vector3D.Zero;

        public OrientationExecutor(Action<string> echo = null)
        {
            _echo = echo;
        }

        /// <summary>
        /// Called when BehaviorIntent changes. Resets internal state if behavior type changed.
        /// </summary>
        public void OnBehaviorChanged(IOrientationBehavior newBehavior)
        {
            bool typeChanged = !IsSameBehaviorType(_currentBehavior, newBehavior);

            if (typeChanged)
            {
                _lastThreatDirection = Vector3D.Zero;
            }

            _currentBehavior = newBehavior;
        }

        /// <summary>
        /// Executes the current orientation behavior.
        /// </summary>
        public void Execute(IOrientationBehavior behavior, DroneContext ctx)
        {
            if (behavior == null)
            {
                // Default: match leader or stay level
                if (ctx.HasLeaderContact)
                {
                    ExecuteMatchLeader(ctx);
                }
                else
                {
                    ctx.Gyros.OrientLevel();
                }
                return;
            }

            var lookAt = behavior as LookAt;
            var lookDir = behavior as LookDirection;

            if (lookAt != null)
                ExecuteLookAt(lookAt, ctx);
            else if (lookDir != null)
                ExecuteLookDirection(lookDir, ctx);
            else if (behavior is MatchLeader)
                ExecuteMatchLeader(ctx);
            else if (behavior is FaceClosestThreat)
                ExecuteFaceClosestThreat(ctx);
            else if (behavior is StayLevel)
                ctx.Gyros.OrientLevel();
        }

        private void ExecuteLookAt(LookAt behavior, DroneContext ctx)
        {
            ctx.Gyros.LookAt(behavior.Target);
        }

        private void ExecuteLookDirection(LookDirection behavior, DroneContext ctx)
        {
            if (behavior.Direction.LengthSquared() < 0.1)
            {
                ctx.Gyros.OrientLevel();
                return;
            }

            Vector3D target = ctx.Position + Vector3D.Normalize(behavior.Direction) * 1000.0;
            ctx.Gyros.LookAt(target);
        }

        private void ExecuteMatchLeader(DroneContext ctx)
        {
            if (!ctx.HasLeaderContact)
            {
                ctx.Gyros.OrientLevel();
                return;
            }

            ctx.Gyros.MatchCompassHeading(ctx.LastLeaderState.Forward, ctx.LastLeaderState.Up);
        }

        private void ExecuteFaceClosestThreat(DroneContext ctx)
        {
            Vector3D? closestThreat = ctx.Tactical.GetClosestThreatPosition(ctx.Position);

            if (closestThreat.HasValue)
            {
                Vector3D threatDir = Vector3D.Normalize(closestThreat.Value - ctx.Position);
                _lastThreatDirection = threatDir;
                ctx.Gyros.LookAt(closestThreat.Value);
            }
            else if (_lastThreatDirection.LengthSquared() > 0.1)
            {
                Vector3D target = ctx.Position + _lastThreatDirection * 1000.0;
                ctx.Gyros.LookAt(target);
            }
            else
            {
                ctx.Gyros.OrientLevel();
            }
        }

        private bool IsSameBehaviorType(IOrientationBehavior a, IOrientationBehavior b)
        {
            if (a == null || b == null) return a == b;
            return a.GetType() == b.GetType();
        }
    }
}
