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

        public IEnumerable<BehaviorIntent> Execute(DroneContext ctx)
        {
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

                    if (needsApproach)
                    {
                        // Approaching formation - yield Approach intent
                        // The PositionExecutor handles all the phased approach logic
                        yield return new BehaviorIntent
                        {
                            Position = new Approach(ctx.GetFormationPosition()),
                            Orientation = GetApproachOrientation(ctx),
                            ExitWhen = () => ctx.IsInFormation() || !ctx.HasLeaderContact
                        };
                    }
                    else
                    {
                        // In formation - yield FormationFollow intent
                        yield return new BehaviorIntent
                        {
                            Position = new FormationFollow(ctx.Config.StationOffset),
                            Orientation = GetFormationOrientation(ctx),
                            ExitWhen = () => ctx.HasExitedFormation() || !ctx.HasLeaderContact
                        };
                    }
                }
            }
        }

        /// <summary>
        /// Determines orientation behavior while approaching.
        /// If threats exist, face them. Otherwise, look at destination.
        /// </summary>
        private OrientationBehavior GetApproachOrientation(DroneContext ctx)
        {
            if (ctx.Tactical.HasThreats)
            {
                return new FaceClosestThreat();
            }

            // Look toward formation position during approach
            return new LookAt(ctx.GetFormationPosition());
        }

        /// <summary>
        /// Determines orientation behavior while in formation.
        /// If threats exist, face them. Otherwise, match leader heading.
        /// </summary>
        private OrientationBehavior GetFormationOrientation(DroneContext ctx)
        {
            if (ctx.Tactical.HasThreats)
            {
                return new FaceClosestThreat();
            }

            return new MatchLeader();
        }
    }
}
