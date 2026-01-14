using System;
using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Attack directive: maintain a flanking arc around a fixed target while respecting leader leash.
    /// </summary>
    public class AttackDirective : IDirective
    {
        public string Name => "Attack";

        public IEnumerable<BehaviorIntent> Execute(DroneContext ctx)
        {
            if (PreFlight.NeedsPreFlight(ctx))
            {
                ctx.Debug?.Log($"[{Name}] Docked, executing pre-flight.");
            }

            foreach (var intent in PreFlight.EnsureUndocked(ctx))
                yield return intent;

            while (!ctx.HasLeaderContact)
            {
                yield return new BehaviorIntent
                {
                    Position = new Move(() => ctx.Position),
                    Orientation = new StayLevel(),
                    ExitWhen = () => ctx.HasLeaderContact
                };
            }

            long targetId = ctx.LastLeaderState.TargetEntityId;
            if (targetId == 0)
            {
                yield return BehaviorIntent.Aborted(AbortReason.TargetLost, "Attack requested with no leader target");
                yield break;
            }

            var rigProvider = ctx.WeaponRigs;
            var rig = rigProvider != null ? rigProvider.GetPrimaryFixedWeaponRig(ctx.GameTime) : null;

            int formationCount = ctx.FormationCount > 0 ? ctx.FormationCount : 1;
            int formationIndex = ctx.FormationIndex >= 0 ? ctx.FormationIndex : 0;

            double lastSeenTime = -1;
            Vector3D lastKnownTargetPos = Vector3D.Zero;
            Vector3D lastKnownTargetVel = Vector3D.Zero;
            Vector3D lastKnownWorldAttackPos = Vector3D.Zero;
            ITargetTelemetry cachedTelemetry = null;

            Vector3D lastLocalOffset = Vector3D.Zero;
            AttackReference lastReference = new AttackReference(Vector3D.Zero, Vector3D.Forward, Vector3D.Up, Vector3D.Zero);

            double cachedMaxRange = rig != null ? rig.MaxRange : 0;

            while (ctx.HasLeaderContact)
            {
                if (!UpdateTelemetry(ctx, targetId, rigProvider, ref lastSeenTime, ref lastKnownTargetPos, ref lastKnownTargetVel, ref cachedTelemetry))
                {
                    if (lastSeenTime < 0)
                    {
                        yield return BehaviorIntent.Aborted(AbortReason.TargetLost, "No target telemetry available");
                        yield break;
                    }

                    double searchEnd = lastSeenTime + ctx.Config.AttackLostTargetGraceSeconds;
                    while (ctx.HasLeaderContact && ctx.GameTime < searchEnd)
                    {
                        yield return new BehaviorIntent
                        {
                            Position = new Move(() => lastKnownWorldAttackPos),
                            Orientation = new MatchLeader(),
                            ExitWhen = () => !ctx.HasLeaderContact || UpdateTelemetry(ctx, targetId, rigProvider, ref lastSeenTime, ref lastKnownTargetPos, ref lastKnownTargetVel, ref cachedTelemetry) || ctx.GameTime >= searchEnd
                        };
                    }

                    if (!ctx.HasLeaderContact)
                    {
                        yield return BehaviorIntent.Aborted(AbortReason.LostLeader, "Lost leader while searching for target");
                        yield break;
                    }

                    if (!UpdateTelemetry(ctx, targetId, rigProvider, ref lastSeenTime, ref lastKnownTargetPos, ref lastKnownTargetVel, ref cachedTelemetry))
                    {
                        yield return BehaviorIntent.Aborted(AbortReason.TargetLost, "Target lost");
                        yield break;
                    }
                }

                // Ammo/heat gating could be inserted here before committing to attack movement.
                yield return new BehaviorIntent
                {
                    Position = new Move(
                            () => lastLocalOffset,
                            () => {
                                UpdateAttackGeometry(ctx, rig, formationCount, formationIndex, lastKnownTargetPos, lastKnownTargetVel, ref lastKnownWorldAttackPos, ref lastLocalOffset, ref lastReference, ref cachedMaxRange);
                                return lastReference;
                            })
                        .WithLevelFormation()
                        .WithExclusion(() => ctx.LastLeaderState.EntityId),
                    Orientation = rig != null
                        ? (IOrientationBehavior)new AimFixedWeapons(() => cachedTelemetry, () => rig)
                        : (ctx.Tactical.HasThreats ? (IOrientationBehavior)new FaceClosestThreat() : new MatchLeader()),
                    ExitWhen = () => !ctx.HasLeaderContact || !UpdateTelemetry(ctx, targetId, rigProvider, ref lastSeenTime, ref lastKnownTargetPos, ref lastKnownTargetVel, ref cachedTelemetry)
                };
            }

            yield return BehaviorIntent.Aborted(AbortReason.LostLeader, "Lost leader contact during attack");
        }

        private static bool UpdateTelemetry(DroneContext ctx, long targetId, FixedWeaponRigProvider rigProvider, ref double lastSeenTime, ref Vector3D lastKnownTargetPos, ref Vector3D lastKnownTargetVel, ref ITargetTelemetry cachedTelemetry)
        {
            var weaponBlock = rigProvider != null ? rigProvider.GetPrimaryWeaponBlock(ctx.GameTime) : null;
            var telemetry = ctx.Tactical.GetTargetTelemetry(targetId, weaponBlock);
            if (telemetry != null && telemetry.IsValid)
            {
                cachedTelemetry = telemetry;
                lastSeenTime = ctx.GameTime;
                lastKnownTargetPos = telemetry.Position;
                lastKnownTargetVel = telemetry.Velocity;
                return true;
            }

            return false;

        }

        private static void UpdateAttackGeometry(DroneContext ctx, IFixedWeaponRig rig, int formationCount, int formationIndex, Vector3D lastKnownTargetPos, Vector3D lastKnownTargetVel, ref Vector3D lastKnownWorldAttackPos, ref Vector3D lastLocalOffset, ref AttackReference lastReference, ref double cachedMaxRange)
        {
            double desiredRange = GetDesiredRange(ctx, rig, ref cachedMaxRange);
            double leaderMax = ctx.Config.AttackMaxLeaderDistance;

            // Build the attack plane: use gravity up when available, otherwise leader up.
            Vector3D up = ctx.Gravity.LengthSquared() > 0.1
                ? -Vector3D.Normalize(ctx.Gravity)
                : ctx.LastLeaderState.Up;
            if (up.LengthSquared() < 0.1)
            {
                up = Vector3D.Up;
            }

            // Project leader position into the attack plane for ring intersection math.
            Vector3D toLeader = ctx.LastLeaderState.Position - lastKnownTargetPos;
            Vector3D planarToLeader = toLeader - up * Vector3D.Dot(toLeader, up);
            double planarDistance = planarToLeader.Length();

            // Forward is defined toward the leader in the attack plane.
            Vector3D forward = planarToLeader.LengthSquared() > 0.001
                ? Vector3D.Normalize(planarToLeader)
                : ctx.LastLeaderState.Forward;
            if (forward.LengthSquared() < 0.1)
            {
                forward = ctx.WorldMatrix.Forward;
            }

            double range = desiredRange;
            double sweepAngle;
            double baseAngle;

            // Decide which portion of the target ring is reachable while respecting the leader leash.
            if (leaderMax <= 0)
            {
                // Degenerate configuration
                sweepAngle = Math.PI;
                baseAngle = Math.PI;
            }
            else if (planarDistance <= 0.01 || planarDistance + range <= leaderMax)
            {
                // Distance from target to leader + desired range is within the leader leash: full arc available.
                // IOTW Leader leash fully contains the target ring: allow a full half-circle behind the target.
                sweepAngle = Math.PI;
                baseAngle = Math.PI;
            }
            else if (planarDistance > leaderMax + range)
            {
                // Degenerate case: target is beyond max leash + weapon range. Ideally drones shouldn't respond to this.
                // No intersection: collapse to a single point on the leader-facing line at the leash boundary.
                range = Math.Max(planarDistance - leaderMax, 0.0);
                sweepAngle = 0.0;
                baseAngle = 0.0;
            }
            else
            {
                // Partial intersection: compute the arc of the target ring that lies within the leader leash.
                double cosTheta = (planarDistance * planarDistance + range * range - leaderMax * leaderMax) / (2 * planarDistance * range);
                cosTheta = MathHelper.Clamp(cosTheta, -1, 1);
                double theta = Math.Acos(cosTheta);
                sweepAngle = Math.Max(0.0, 2 * theta);
                baseAngle = 0.0;
            }

            // Build local offset on the target ring arc and convert to world position.
            lastLocalOffset = FormationNavigator.GetArcOffset(
                formationIndex,
                formationCount,
                range,
                0,
                0,
                baseAngle,
                sweepAngle);

            lastReference = new AttackReference(lastKnownTargetPos, forward, up, lastKnownTargetVel);

            MatrixD refMatrix = MatrixD.CreateWorld(lastReference.Position, lastReference.Forward, lastReference.Up);
            Vector3D worldOffset = Vector3D.TransformNormal(lastLocalOffset, refMatrix);
            lastKnownWorldAttackPos = lastKnownTargetPos + worldOffset;

        }

        private static double GetDesiredRange(DroneContext ctx, IFixedWeaponRig rig, ref double cachedMaxRange)
        {
            if (cachedMaxRange <= 0 && rig != null)
            {
                cachedMaxRange = rig.MaxRange;
            }

            double baseRange = cachedMaxRange > 0 ? cachedMaxRange : ctx.Config.AttackMaxLeaderDistance;
            if (baseRange <= 0)
            {
                baseRange = 1.0;
            }

            double minRange = Math.Max(0.0, baseRange * ctx.Config.AttackMinRangeFactor);
            double maxRange = Math.Max(minRange, baseRange * ctx.Config.AttackMaxRangeFactor);
            double desired = (minRange + maxRange) * 0.5;
            return desired > 0.1 ? desired : Math.Max(1.0, maxRange);
        }

        private struct AttackReference : IOrientedReference
        {
            private readonly Vector3D _position;
            private readonly Vector3D _forward;
            private readonly Vector3D _up;
            private readonly Vector3D _velocity;

            public AttackReference(Vector3D position, Vector3D forward, Vector3D up, Vector3D velocity)
            {
                _position = position;
                _forward = forward.LengthSquared() > 0.001 ? Vector3D.Normalize(forward) : Vector3D.Forward;
                _up = up.LengthSquared() > 0.001 ? Vector3D.Normalize(up) : Vector3D.Up;
                _velocity = velocity;
            }

            public Vector3D Position => _position;
            public Vector3D Forward => _forward;
            public Vector3D Up => _up;
            public Vector3D Velocity => _velocity;
        }
    }
}
