namespace IngameScript
{
    /// <summary>
    /// Maintain level orientation (no pitch/roll relative to gravity).
    /// </summary>
    public class StayLevel : IOrientationBehavior
    {
        public void Execute(DroneContext ctx)
        {
            ctx.Gyros.OrientLevel();
        }
    }
}
