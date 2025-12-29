using System;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Maintain formation offset relative to leader.
    /// Handles leader velocity matching and station-keeping corrections.
    /// </summary>
    public class FormationFollow : IPositionBehavior
    {
        private readonly Func<Vector3D> _offsetFunc;

        /// <summary>Offset from leader in leader's local space (evaluated each tick if dynamic).</summary>
        public Vector3D Offset => _offsetFunc();
        public bool MatchLeaderVelocity { get; }

        /// <summary>Follow at fixed offset.</summary>
        public FormationFollow(Vector3D offset, bool matchLeaderVelocity = true)
        {
            _offsetFunc = () => offset;
            MatchLeaderVelocity = matchLeaderVelocity;
        }

        /// <summary>Follow at dynamically-computed offset (evaluated each tick).</summary>
        public FormationFollow(Func<Vector3D> offsetFunc, bool matchLeaderVelocity = true)
        {
            _offsetFunc = offsetFunc;
            MatchLeaderVelocity = matchLeaderVelocity;
        }

        public void Execute(DroneContext ctx)
        {
            if (!ctx.HasLeaderContact)
            {
                ctx.Thrusters.Release();
                return;
            }

            Vector3D formationPos = ctx.Navigator.CalculateFormationPosition(ctx.LastLeaderState, Offset);
            formationPos = ctx.Navigator.AdjustForTerrainClearance(formationPos, ctx.Reference, ctx.Config.MinTerrainClearance);

            double distance = Vector3D.Distance(ctx.Position, formationPos);
            Vector3D toFormation = formationPos - ctx.Position;
            double safeSpeed = ctx.Thrusters.GetSafeApproachSpeed(distance, toFormation);

            Vector3D leaderVelocity = MatchLeaderVelocity ? ctx.LastLeaderState.Velocity : Vector3D.Zero;
            Vector3D desiredVelocity = ctx.Navigator.CalculateDesiredVelocity(
                ctx.Position, ctx.Velocity, formationPos, leaderVelocity, safeSpeed);

            ctx.Thrusters.MoveToward(formationPos, desiredVelocity, safeSpeed, ctx.Config.PrecisionRadius);
        }
    }
}
