using System.Collections.Generic;

namespace IngameScript
{
    /// <summary>
    /// A directive is a high-level behavioral orchestrator.
    /// It yields BehaviorIntents describing what should happen each tick.
    /// 
    /// Directives use yield return to create coroutine-style sequences.
    /// The iterator position naturally tracks "where we are" in the sequence.
    /// Local variables persist across yields, providing implicit state.
    /// </summary>
    public interface IDirective
    {
        /// <summary>
        /// Human-readable name for status display.
        /// </summary>
        string Name { get; }

        /// <summary>
        /// Executes the directive, yielding behavior intents.
        /// Each yield return represents one behavioral step.
        /// The Brain advances to the next intent when ExitWhen returns true.
        /// 
        /// Yield BehaviorIntent.Complete() to signal successful completion.
        /// Yield BehaviorIntent.Aborted(...) to signal failure.
        /// </summary>
        /// <param name="ctx">Read-only context for drone state and tactical info</param>
        IEnumerable<BehaviorIntent> Execute(DroneContext ctx);
    }
}
