enum DroneState
{
    Initializing,   // Block discovery, config parse
    Idle,           // No target, waiting
    Acquiring,      // Searching for target
    Approaching,    // Moving toward station point
    StationKeeping, // Maintaining position
    Returning,      // Target lost, returning to last known
    Emergency,       // Critical failure, safe shutdown
    Docking,         // Performing docking maneuvers
    Undocking        // Performing launch maneuvers
}