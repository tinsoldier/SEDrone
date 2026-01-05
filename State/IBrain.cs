using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;

namespace IngameScript
{
    /// <summary>
    /// Interface for pluggable "brain" modules that control grid behavior.
    /// Different brains can be swapped based on role (Leader/Drone) or situation.
    /// </summary>
    public interface IBrain
    {
        /// <summary>
        /// Human-readable name for status display.
        /// </summary>
        string Name { get; }

        /// <summary>
        /// Current operational state description.
        /// </summary>
        string Status { get; }
        IMyProgrammableBlock PB { get; set; }

        /// <summary>
        /// Initialize the brain with required dependencies.
        /// Called once when the brain is activated.
        /// </summary>
        /// <param name="context">Shared context containing grid systems and state</param>
        void Initialize(BrainContext context);

        /// <summary>
        /// Returns the state machine enumerator for this brain.
        /// Each yield represents one tick of work; return false to signal completion/error.
        /// </summary>
        IEnumerator<bool> Run();

        /// <summary>
        /// Called when the brain is being replaced or shut down.
        /// Clean up any overrides, release control of hardware.
        /// </summary>
        void Shutdown();
    }

    /// <summary>
    /// Shared context passed to all brains, providing access to grid systems.
    /// This decouples brains from direct Program dependencies.
    /// </summary>
    public class BrainContext
    {
        // === Grid Program Access ===
        public IMyGridTerminalSystem GridTerminalSystem { get; set; }
        public IMyProgrammableBlock Me { get; set; }
        public IMyIntergridCommunicationSystem IGC { get; set; }

        // === Configuration ===
        public DroneConfig Config { get; set; }

        // === Shared Components ===
        public IMyShipController Reference { get; set; }

        /// <summary>
        /// Grid entity ID associated with this brain.
        /// </summary>
        public long GridId { get; set; }

        /// <summary>
        /// Optional pre-collected hardware set (ref-hack mode).
        /// </summary>
        public DroneHardware Hardware { get; set; }

        // === Runtime ===
        public double DeltaTime { get; set; }   // Seconds since last update
        public double GameTime { get; set; }    // Total game time in seconds

        // === Status Output ===
        public System.Action<string> Echo { get; set; }

        // === Shared Messaging ===
        public ICommandBus CommandBus { get; set; }
        public IStateBus<LeaderStateMessage> LeaderStateBus { get; set; }

        // === Shared Tactical ===
        public TacticalCoordinator TacticalCoordinator { get; set; }
        public TacticalSnapshot SharedTacticalSnapshot { get; set; }

        // === Docking (refhack/local) ===
        public DockingPadManager LocalDockingManager { get; set; }
    }
}
