using System;
using System.Collections.Generic;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Escort directive: maintains formation with leader, responds to threats.
    /// 
    /// This replaces the SearchingMode → ApproachingMode → HoldingMode flow
    /// with a single declarative directive.
    /// 
    /// Behavior:
    /// - No leader: Loiter at current position, waiting for contact
    /// - Have leader, far from formation: Approach formation position
    /// - Have leader, in formation: FormationFollow with velocity matching
    /// - Threats detected: Face closest threat while maintaining formation
    /// </summary>
    public class EscortDirective : IDirective
    {
        public string Name => "Escort";

        // Threshold angle (degrees) above which we use LevelFirstApproach
        private const double LEVEL_FIRST_ANGLE_THRESHOLD = 90.0;

        public IEnumerable<BehaviorIntent> Execute(DroneContext ctx)
        {
            // === Pre-flight: Undock if docked ===
            foreach (var intent in PreFlight.EnsureUndocked(ctx))
                yield return intent;

            while (true)
            {
                // === No leader contact - wait and loiter ===
                while (!ctx.HasLeaderContact)
                {
                    yield return new BehaviorIntent
                    {
                        Position = new Loiter(ctx.Position, 30),
                        Orientation = new StayLevel(),
                        ExitWhen = () => ctx.HasLeaderContact
                    };
                }

                // === Have leader - escort ===
                while (ctx.HasLeaderContact)
                {
                    // Check if we need to approach or can hold formation
                    bool needsApproach = ctx.HasExitedFormation() || !ctx.IsInFormation();
                    bool hadThreats = ctx.Tactical.HasThreats;

                    if (needsApproach)
                    {
                        // Determine if we should use level-first approach
                        // (when formation is significantly above/below us)
                        double angleToFormation = GetAngleToFormation(ctx);
                        bool useLevelFirst = Math.Abs(angleToFormation) > LEVEL_FIRST_ANGLE_THRESHOLD;

                        if (useLevelFirst)
                        {
                            // Use coupled behavior - handles both position and orientation
                            yield return new BehaviorIntent
                            {
                                Position = new LevelFirstApproach(() => ctx.GetFormationPosition()),
                                Orientation = null,  // Coupled behavior handles orientation
                                ExitWhen = () => ctx.IsInFormation() || !ctx.HasLeaderContact || ctx.Tactical.HasThreats != hadThreats
                            };
                        }
                        else
                        {
                            // Normal approach - dynamic target via Func<>
                            yield return new BehaviorIntent
                            {
                                Position = new Approach(() => ctx.GetFormationPosition()),
                                Orientation = GetApproachOrientation(ctx),
                                ExitWhen = () => ctx.IsInFormation() || !ctx.HasLeaderContact || ctx.Tactical.HasThreats != hadThreats
                            };
                        }
                    }
                    else
                    {
                        // In formation - yield FormationFollow intent
                        yield return new BehaviorIntent
                        {
                            Position = new FormationFollow(ctx.Config.StationOffset),
                            Orientation = GetFormationOrientation(ctx),
                            ExitWhen = () => ctx.HasExitedFormation() || !ctx.HasLeaderContact || ctx.Tactical.HasThreats != hadThreats
                        };
                    }
                }
            }
        }

        /// <summary>
        /// Gets the angle (in degrees) between our forward vector and the direction to formation.
        /// Positive = above us, Negative = below us (relative to horizon).
        /// </summary>
        private double GetAngleToFormation(DroneContext ctx)
        {
            Vector3D toFormation = ctx.GetFormationPosition() - ctx.Position;
            if (toFormation.LengthSquared() < 1.0)
                return 0;

            toFormation.Normalize();
            
            // Get the vertical component (relative to gravity)
            Vector3D up = -Vector3D.Normalize(ctx.Gravity);
            double verticalComponent = Vector3D.Dot(toFormation, up);
            
            // Convert to angle in degrees
            return Math.Asin(MathHelper.Clamp(verticalComponent, -1.0, 1.0)) * (180.0 / Math.PI);
        }

        /// <summary>
        /// Determines orientation behavior while approaching.
        /// If threats exist, face them. Otherwise, look at destination.
        /// </summary>
        private IOrientationBehavior GetApproachOrientation(DroneContext ctx)
        {
            if (ctx.Tactical.HasThreats)
            {
                return new FaceClosestThreat();
            }

            // Look toward formation position during approach (dynamic)
            return new LookAt(() => ctx.GetFormationPosition());
        }

        /// <summary>
        /// Determines orientation behavior while in formation.
        /// If threats exist, face them. Otherwise, match leader heading.
        /// </summary>
        private IOrientationBehavior GetFormationOrientation(DroneContext ctx)
        {
            if (ctx.Tactical.HasThreats)
            {
                return new FaceClosestThreat();
            }

            return new MatchLeader();
        }
    }
}
