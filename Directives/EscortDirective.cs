using System;
using System.Collections.Generic;
using System.Runtime.Remoting.Channels;
using Sandbox.ModAPI.Ingame;
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
    /// - No leader: Hold position with Move, waiting for contact
    /// - Have leader, far from formation: Approach formation position
    /// - Have leader, in formation: Move with velocity matching
    /// - Threats detected: Face closest threat while maintaining formation
    /// </summary>
    public class EscortDirective : IDirective
    {
        public string Name => "Escort";

        // Threshold angle (degrees) above which we use LevelFirstApproach
        private const double LEVEL_FIRST_ANGLE_THRESHOLD = 30.0;

        public IEnumerable<BehaviorIntent> Execute(DroneContext ctx)
        {
            if(PreFlight.NeedsPreFlight(ctx))
            {
                ctx.Debug?.Log($"[{Name}] Docked, executing pre-flight.");
            }

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
                        Position = new Move(() => ctx.Position),
                        Orientation = new StayLevel(),
                        ExitWhen = () => ctx.HasLeaderContact
                    };
                }

                // === Have leader - escort ===
                while (ctx.HasLeaderContact)
                {
                    // Check if we need to approach or can hold formation
                    bool hadThreats = ctx.Tactical.HasThreats;
                    bool leaderHasTargets = ctx.LastLeaderState.TargetEntityId != 0;
                    var rigProvider = ctx.WeaponRigs;
                    var rig = rigProvider != null ? rigProvider.GetPrimaryFixedWeaponRig(ctx.GameTime) : null;
                    bool weaponsReady = rig != null && rig.IsWeaponReady;

                    // Debug: log leader target state
                    if (leaderHasTargets)
                    {
                        ctx.Debug?.Log($"Escort: Leader targeting entity {ctx.LastLeaderState.TargetEntityId}");
                    }

                    // Debug logging
                    // ctx.Debug?.Log($"Escort: approach={needsApproach} inFrm={ctx.IsInFormation()} dist={ctx.DistanceToFormation():F1}m");
                    // ctx.Debug?.Log($"  tgtVel={ctx.LastLeaderState.Velocity.Length():F1} m/s, threats={hadThreats}");

                    yield return new BehaviorIntent
                    {
                         Position = new Move(ctx.StationOffset, () => ctx.LastLeaderState),
                        Orientation = GetFormationOrientation(ctx, rigProvider, rig, leaderHasTargets, weaponsReady),
                        ExitWhen = () => 
                            !ctx.HasLeaderContact ||
                            ctx.Tactical.HasThreats != hadThreats ||
                            (ctx.LastLeaderState.TargetEntityId != 0) != leaderHasTargets ||
                            (rig != null && rig.IsWeaponReady) != weaponsReady
                    };
                }
            }
        }

        /// <summary>
        /// Determines orientation behavior while in formation.
        /// If threats exist, face them. Otherwise, match leader heading.
        /// </summary>
        private IOrientationBehavior GetFormationOrientation(DroneContext ctx, FixedWeaponRigProvider rigProvider, IFixedWeaponRig rig, bool leaderHasTarget, bool weaponsReady)
        {
            if (leaderHasTarget && weaponsReady)
            {
                return new AimFixedWeapons(
                    () =>
                    {
                        var weaponBlock = rigProvider.GetPrimaryWeaponBlock(ctx.GameTime);
                        return ctx.Tactical.GetTargetTelemetry(ctx.LastLeaderState.TargetEntityId, weaponBlock)
                            ?? ctx.Tactical.GetClosestEnemyTelemetry(ctx.Position);
                        // return ctx.Tactical.GetPredictedTargetTelemetry(ctx.LastLeaderState.TargetEntityId, ctx.WcApi, weaponBlock)
                        //     ?? ctx.Tactical.GetTargetTelemetry(ctx.LastLeaderState.TargetEntityId, weaponBlock)
                        //     ?? ctx.Tactical.GetClosestEnemyTelemetry(ctx.Position);
                    },
                    () => rig);
            }

            if (rigProvider != null)
            {
                rigProvider.StopAllWeapons();
            }

            if (ctx.Tactical.HasThreats)
            {
                return new FaceClosestThreat();
            }

            return new MatchLeader();
        }
    }
}
