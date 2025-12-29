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
            var alignConnector = behavior as AlignConnector;

            if (lookAt != null)
                ExecuteLookAt(lookAt, ctx);
            else if (lookDir != null)
                ExecuteLookDirection(lookDir, ctx);
            else if (alignConnector != null)
                ExecuteAlignConnector(alignConnector, ctx);
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

        /// <summary>
        /// Aligns the drone so its connector faces the target direction.
        /// Uses a virtual reference frame based on connector orientation (SEAD2-inspired).
        /// </summary>
        private void ExecuteAlignConnector(AlignConnector behavior, DroneContext ctx)
        {
            var connector = behavior.DroneConnector;
            if (connector == null || !connector.IsFunctional)
            {
                ctx.Gyros.OrientLevel();
                return;
            }

            Vector3D targetDir = behavior.TargetDirection();
            if (targetDir.LengthSquared() < 0.1)
            {
                ctx.Gyros.OrientLevel();
                return;
            }
            targetDir = Vector3D.Normalize(targetDir);

            // Get desired up direction (from behavior or gravity-based)
            Vector3D desiredUp;
            if (behavior.DesiredUp != null)
            {
                desiredUp = Vector3D.Normalize(behavior.DesiredUp());
            }
            else
            {
                // Use gravity-aligned up
                Vector3D gravity = ctx.Gravity;
                if (gravity.LengthSquared() > 0.1)
                {
                    desiredUp = -Vector3D.Normalize(gravity);
                }
                else
                {
                    desiredUp = ctx.WorldMatrix.Up;
                }
            }

            // The connector's forward should face the target direction.
            // We need to calculate what the SHIP's forward should be for that to happen.
            //
            // Connector forward (local to ship) tells us the relationship between
            // ship orientation and connector orientation.
            
            MatrixD connectorMatrix = connector.WorldMatrix;
            MatrixD shipMatrix = ctx.WorldMatrix;

            // Get connector's forward in ship-local coordinates
            Vector3D connectorForwardLocal = Vector3D.TransformNormal(
                connectorMatrix.Forward, 
                MatrixD.Transpose(shipMatrix));

            // If connector forward is (0, -1, 0) in ship coords (belly connector facing down),
            // and we want it to face targetDir (world),
            // then we need to orient the ship so its -Y axis points at targetDir.

            // Create a virtual reference matrix for the gyro controller:
            // This is the orientation we WANT the ship to achieve, expressed as what
            // the ship's forward/up should be when the connector is properly aligned.
            
            // When connector faces targetDir, and connector's "up" is aligned with desiredUp,
            // we can solve for ship orientation.

            // For simplicity, use the gyro's AlignConnectorToDirection method if available,
            // or compute manually:
            ctx.Gyros.AlignConnectorToDirection(connector, targetDir, desiredUp);
        }

        private bool IsSameBehaviorType(IOrientationBehavior a, IOrientationBehavior b)
        {
            if (a == null || b == null) return a == b;
            return a.GetType() == b.GetType();
        }
    }
}
