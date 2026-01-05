namespace IngameScript
{
    /// <summary>
    /// Match leader's compass heading (yaw only, stays level to gravity).
    /// </summary>
    public class MatchLeader : IOrientationBehavior
    {
        public void Execute(DroneContext ctx)
        {
            if (!ctx.HasLeaderContact)
            {
                ctx.Gyros.OrientLevel();
                return;
            }

            if (!ctx.Gyros.MatchCompassHeading(ctx.LastLeaderState.Forward, ctx.LastLeaderState.Up))
            {
                ctx.Gyros.OrientLevel();
            }
        }
    }
}
