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
        Vector3D Size { get; }
    }

    public class TargetTelemetry : ITargetTelemetry
    {
        private readonly MyDetectedEntityInfo _info;

        public TargetTelemetry(MyDetectedEntityInfo info)
        {
            _info = info;
        }

        public bool IsValid => _info.EntityId != 0;
        public Vector3D Position => _info.BoundingBox.Center;
        public Vector3D Velocity => _info.Velocity;
        public Vector3D Acceleration => Vector3D.Zero;
        public Vector3D Size => _info.BoundingBox.Size;
    }

    public class PredictedTargetTelemetry : ITargetTelemetry
    {
        private readonly MyDetectedEntityInfo _info;
        private readonly Program.WcPbApi _wcApi;
        private readonly IMyTerminalBlock _weaponBlock;

        public PredictedTargetTelemetry(MyDetectedEntityInfo info, Program.WcPbApi wcApi, IMyTerminalBlock weaponBlock)
        {
            _info = info;
            _wcApi = wcApi;
            _weaponBlock = weaponBlock;
        }

        public bool IsValid => _info.EntityId != 0;
        public Vector3D Position
        {
            get
            {
                if (_wcApi == null || _weaponBlock == null)
                    return _info.BoundingBox.Center;

                Vector3D? predicted = _wcApi.GetPredictedTargetPosition(_weaponBlock, _info.EntityId, 0);
                return predicted ?? _info.BoundingBox.Center;
            }
        }
        public Vector3D Velocity => _info.Velocity;
        public Vector3D Acceleration => Vector3D.Zero;
        public Vector3D Size => _info.BoundingBox.Size;
    }
}
