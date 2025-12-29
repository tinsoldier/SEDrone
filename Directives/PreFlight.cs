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
            var connector = ctx.ActiveConnector;

            // Disconnect
            ctx.Undock();

            // Dampeners will auto-enable on first movement command (ThrusterController safety)
            // But let's be explicit
            ctx.SetDampeners(true);

            // Calculate clearance direction (away from connector's forward)
            // Connector forward points INTO the thing it's docking with, so we go opposite
            Vector3D connectorForward = connector.WorldMatrix.Forward;
            Vector3D clearDirection = -connectorForward;

            // If we can't determine direction, use gravity-up as fallback
            if (clearDirection.LengthSquared() < 0.5)
            {
                Vector3D gravity = ctx.Gravity;
                if (gravity.LengthSquared() > 0.1)
                {
                    clearDirection = -Vector3D.Normalize(gravity);
                }
                else
                {
                    clearDirection = ctx.WorldMatrix.Forward;
                }
            }

            clearDirection = Vector3D.Normalize(clearDirection);
            Vector3D startPosition = ctx.Position;
            Vector3D clearPosition = startPosition + clearDirection * clearanceDistance;

            // Move to clear position
            yield return new BehaviorIntent
            {
                Position = new Approach(clearPosition, speedLimit: ctx.Config.DockingApproachSpeed),
                Orientation = new StayLevel(),
                ExitWhen = () => ctx.DistanceTo(clearPosition) < 3.0
            };

            // Brief stabilization
            yield return new BehaviorIntent
            {
                Position = new Loiter(clearPosition, ctx.Config.StationRadius),
                Orientation = new StayLevel(),
                ExitWhen = () => ctx.Velocity.Length() < 1.0
            };
        }

        /// <summary>
        /// Ensures the drone is ready for combat operations.
        /// Includes undocking if docked, plus any combat-specific preparation.
        /// </summary>
        /// <param name="ctx">Drone context</param>
        /// <param name="clearanceDistance">Distance to move away from dock</param>
        /// <returns>Sequence of BehaviorIntents for combat preparation</returns>
        public static IEnumerable<BehaviorIntent> EnsureCombatReady(DroneContext ctx, double clearanceDistance = DEFAULT_CLEARANCE)
        {
            // First, ensure undocked
            foreach (var intent in EnsureUndocked(ctx, clearanceDistance))
                yield return intent;

            // Future: could add weapon arming, shield activation, etc.
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
