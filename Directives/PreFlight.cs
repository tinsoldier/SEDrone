using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Shared pre-flight helper methods for directives.
    /// Provides common initialization sequences that directives can use
    /// to ensure the drone is ready for flight operations.
    /// </summary>
    public static class PreFlight
    {
        /// <summary>
        /// Default clearance distance when undocking (meters).
        /// </summary>
        public const double DEFAULT_CLEARANCE = 20.0;

        /// <summary>
        /// Ensures the drone is undocked and clear of the docking area.
        /// If already undocked, yields nothing and returns immediately.
        /// 
        /// Usage in a directive:
        /// <code>
        /// foreach (var intent in PreFlight.EnsureUndocked(ctx))
        ///     yield return intent;
        /// </code>
        /// </summary>
        /// <param name="ctx">Drone context</param>
        /// <param name="clearanceDistance">Distance to move away from dock</param>
        /// <returns>Sequence of BehaviorIntents for undocking (empty if not docked)</returns>
        public static IEnumerable<BehaviorIntent> EnsureUndocked(DroneContext ctx, double clearanceDistance = DEFAULT_CLEARANCE)
        {
            // Already undocked - nothing to do
            if (!ctx.IsDocked)
                yield break;

            // Store reference to connector before undocking
            var connector = ctx.ActiveConnector.OtherConnector;

            // Disconnect
            ctx.Undock();

            // Dampeners will auto-enable on first movement command (ThrusterController safety)
            // But let's be explicit
            ctx.SetDampeners(true);

            // Calculate clearance position:
            // 1. Move along connector's -Forward (away from dock face) in world space
            // 2. Convert that world position to leader-local coordinates for Move intent

            // Step 1: Calculate world target position from connector local offset
            Vector3D connectorLocalOffset = new Vector3D(0, 0, -clearanceDistance);
            Vector3D worldOffset = Vector3D.TransformNormal(connectorLocalOffset, connector.WorldMatrix);
            // Apply connector-facing offset relative to nav reference to avoid lateral jig
            // caused by connector/reference origin or axis misalignment.
            Vector3D targetWorld = ctx.Reference.GetPosition() + worldOffset;

            // Step 2: Convert world position to leader-local coordinates
            Vector3D worldDirection = targetWorld - ctx.LastLeaderState.Position;
            Vector3D leaderLocalOffset = Vector3D.TransformNormal(worldDirection, MatrixD.Transpose(ctx.LastLeaderState.WorldMatrix));

            Func<Vector3D> dynamicWorldTarget = () =>
            {
                var leader = ctx.LastLeaderState;
                return leader.Position + Vector3D.TransformNormal(leaderLocalOffset, leader.WorldMatrix);
            };

            // Move to clear position
            yield return new BehaviorIntent
            {
                Position = new Move(leaderLocalOffset, () => ctx.LastLeaderState),
                Orientation = new StayLevel(),
                ExitWhen = () => ctx.DistanceTo(dynamicWorldTarget()) < 3.0
            };
        }

        /// <summary>
        /// Quick check to see if pre-flight is needed.
        /// Useful for directives that want to skip the foreach if not needed.
        /// </summary>
        public static bool NeedsPreFlight(DroneContext ctx)
        {
            return ctx.IsDocked;
        }
    }
}
