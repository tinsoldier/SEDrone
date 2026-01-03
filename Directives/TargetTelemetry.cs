using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    public interface ITargetTelemetry
    {
        bool IsValid { get; }
        Vector3D Position { get; }
        Vector3D Velocity { get; }
        Vector3D Acceleration { get; }
    }

    public class TargetTelemetry : ITargetTelemetry
    {
        private readonly MyDetectedEntityInfo _info;

        public TargetTelemetry(MyDetectedEntityInfo info)
        {
            _info = info;
        }

        public bool IsValid => _info.EntityId != 0;
        public Vector3D Position => _info.Position;
        public Vector3D Velocity => _info.Velocity;
        public Vector3D Acceleration => Vector3D.Zero;
    }
}
