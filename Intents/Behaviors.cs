namespace IngameScript
{
    /// <summary>
    /// Interface for position behaviors - describes WHERE the drone should be and HOW to get there.
    /// Behaviors now own their execution logic.
    /// </summary>
    public interface IPositionBehavior
    {
        /// <summary>
        /// Executes this position behavior for one tick.
        /// Behavior owns its logic and can maintain state across ticks.
        /// </summary>
        void Execute(DroneContext ctx);
    }

    /// <summary>
    /// Interface for orientation behaviors - describes WHERE the drone should look and HOW to orient.
    /// Behaviors now own their execution logic.
    /// </summary>
    public interface IOrientationBehavior
    {
        /// <summary>
        /// Executes this orientation behavior for one tick.
        /// Behavior owns its logic and can maintain state across ticks.
        /// </summary>
        void Execute(DroneContext ctx);
    }
}
