using System;

namespace IngameScript
{
    /// <summary>
    /// Reason for directive abort.
    /// </summary>
    public enum AbortReason
    {
        None,
        LostLeader,
        TargetLost,
        Timeout,
        Damaged
    }

    /// <summary>
    /// Detailed abort information.
    /// </summary>
    public class AbortInfo
    {
        public AbortReason Reason { get; }
        public string Details { get; }

        public AbortInfo(AbortReason reason, string details = null)
        {
            Reason = reason;
            Details = details;
        }
    }

    /// <summary>
    /// A single behavioral step yielded by a directive.
    /// Represents one "tick" of declarative intent: be here, look there, do this.
    /// 
    /// If Position implements IOrientationBehavior (coupled maneuver), Orientation is ignored.
    /// </summary>
    public class BehaviorIntent
    {
        /// <summary>
        /// Desired position behavior. Required - use Loiter for idle.
        /// If this also implements IOrientationBehavior, it's a coupled maneuver
        /// and the Orientation property is ignored.
        /// </summary>
        public IPositionBehavior Position { get; set; }

        /// <summary>
        /// Desired orientation behavior. Ignored if Position is a coupled maneuver.
        /// Use StayLevel or MatchLeader for default orientation.
        /// </summary>
        public IOrientationBehavior Orientation { get; set; }

        /// <summary>
        /// Optional non-locomotion action (docking, antenna, etc.)
        /// </summary>
        public ActionBehavior Action { get; set; }

        /// <summary>
        /// Condition that triggers advancement to the next intent.
        /// Evaluated every tick. When true, directive advances.
        /// </summary>
        public Func<bool> ExitWhen { get; set; }

        /// <summary>
        /// True if the directive has completed successfully.
        /// </summary>
        public bool IsComplete { get; private set; }

        /// <summary>
        /// True if the directive was aborted.
        /// </summary>
        public bool IsAborted { get; private set; }

        /// <summary>
        /// Details about why the directive was aborted.
        /// </summary>
        public AbortInfo Abort { get; private set; }

        /// <summary>
        /// Creates a terminal intent indicating successful completion.
        /// </summary>
        public static BehaviorIntent Complete()
        {
            return new BehaviorIntent { IsComplete = true };
        }

        /// <summary>
        /// Creates a terminal intent indicating abort.
        /// </summary>
        public static BehaviorIntent Aborted(AbortReason reason, string details = null)
        {
            return new BehaviorIntent
            {
                IsAborted = true,
                Abort = new AbortInfo(reason, details)
            };
        }
    }
}
