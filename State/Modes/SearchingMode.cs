namespace IngameScript
{
    /// <summary>
    /// Initial mode: No leader contact, waiting for broadcasts.
    /// The drone releases all controls and waits passively for IGC messages.
    /// 
    /// Transitions:
    /// - → ApproachingMode: When leader contact is established
    /// </summary>
    public class SearchingMode : IDroneMode
    {
        public string Name => "Searching";

        public void Enter(DroneBrain brain)
        {
            // Release all controls - we're just waiting
            brain.Gyros.Release();
            brain.Thrusters.Release();
            
            brain.Echo?.Invoke($"[{Name}] Waiting for leader broadcasts on: {brain.Config.IGCChannel}");
        }

        public IDroneMode Update(DroneBrain brain)
        {
            // Transition condition: leader contact acquired
            if (brain.HasLeaderContact)
            {
                return new ApproachingMode();
            }

            // Stay in searching mode
            return this;
        }

        public void Exit(DroneBrain brain)
        {
            brain.Echo?.Invoke($"[{Name}] Leader acquired: {brain.LastLeaderState.GridName}");
        }
    }
}
