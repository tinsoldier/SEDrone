namespace IngameScript
{
    /// <summary>
    /// Represents a discrete operational mode for the drone.
    /// Each mode encapsulates its own behavior and transition logic.
    /// 
    /// This follows the State-as-Strategy pattern:
    /// - Each mode is a self-contained object that handles one behavior
    /// - Modes decide their own transitions by returning a different mode from Update()
    /// - Enter/Exit hooks provide lifecycle management for setup/teardown
    /// </summary>
    public interface IDroneMode
    {
        /// <summary>
        /// Human-readable name for status display and debugging.
        /// Should be short (e.g., "Searching", "Approaching", "Holding").
        /// </summary>
        string Name { get; }

        /// <summary>
        /// Called once when entering this mode.
        /// Use for mode-specific initialization:
        /// - Reset PID controllers
        /// - Set orientation behavior
        /// - Log mode entry
        /// - Configure controller parameters
        /// </summary>
        /// <param name="brain">The drone brain context</param>
        void Enter(DroneBrain brain);

        /// <summary>
        /// Called every tick while in this mode.
        /// Performs the mode's primary behavior and determines transitions.
        /// 
        /// Return values:
        /// - Return 'this' to stay in the current mode
        /// - Return a new mode instance to trigger a transition
        /// 
        /// The brain will handle calling Exit() on current mode and Enter() on the new mode.
        /// </summary>
        /// <param name="brain">The drone brain context</param>
        /// <returns>The mode to be active next tick (this or a new mode)</returns>
        IDroneMode Update(DroneBrain brain);

        /// <summary>
        /// Called once when leaving this mode.
        /// Use for cleanup:
        /// - Release controls if needed
        /// - Log mode exit
        /// - Clear mode-specific state
        /// </summary>
        /// <param name="brain">The drone brain context</param>
        void Exit(DroneBrain brain);
    }
}
