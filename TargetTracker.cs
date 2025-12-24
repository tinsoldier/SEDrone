using Sandbox.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    public abstract class TargetTracker
    {
        // Current target reference
        IMyCubeGrid TargetGrid;
        IMyTerminalBlock ReferenceBlock;  // For orientation

        // Target state (updated each tick)
        Vector3D Position;
        Vector3D Velocity;
        Vector3D Acceleration;      // For prediction
        MatrixD WorldMatrix;        // Orientation

        // Methods
        public abstract bool TryAcquireTarget();
        public abstract void UpdateTargetState();
        public abstract bool IsTargetValid();
        public abstract Vector3D PredictPosition(double seconds);
    }
}