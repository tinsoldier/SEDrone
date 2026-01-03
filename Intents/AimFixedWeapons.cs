using System;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    public interface IFixedWeaponRig
    {
        IMyCubeBlock AimBlock { get; }
        double ProjectileSpeed { get; } // 0 = hitscan
        bool CanFire { get; }
        void Fire(bool enable);
    }

    public interface ITargetTelemetry
    {
        bool IsValid { get; }
        Vector3D Position { get; }
        Vector3D Velocity { get; }
        Vector3D Acceleration { get; }
    }

    /// <summary>
    /// Aims a fixed weapon rig at a target using lead prediction.
    /// Orientation only - firing is handled by the directive/executor.
    /// </summary>
    public class AimFixedWeapons : IOrientationBehavior
    {
        private readonly Func<ITargetTelemetry> _targetFunc;
        private readonly Func<IFixedWeaponRig> _weaponFunc;
        private readonly Func<Vector3D> _desiredUpFunc;
        private readonly int _maxLeadIterations;
        private readonly double _fireAngleDeg;

        public bool IsAligned { get; private set; }
        public double AlignmentErrorDeg { get; private set; }

        public AimFixedWeapons(
            Func<ITargetTelemetry> targetFunc,
            Func<IFixedWeaponRig> weaponFunc,
            Func<Vector3D> desiredUpFunc = null,
            int maxLeadIterations = 4,
            double fireAngleDeg = 1.0)
        {
            _targetFunc = targetFunc;
            _weaponFunc = weaponFunc;
            _desiredUpFunc = desiredUpFunc;
            _maxLeadIterations = Math.Max(1, maxLeadIterations);
            _fireAngleDeg = Math.Max(0.1, fireAngleDeg);
        }

        public void Execute(DroneContext ctx)
        {
            var target = _targetFunc?.Invoke();
            var weapon = _weaponFunc?.Invoke();

            if (weapon == null || weapon.AimBlock == null || target == null || !target.IsValid)
            {
                IsAligned = false;
                return;
            }

            Vector3D aimDirection;
            if (!TryGetAimDirection(
                weapon.AimBlock.GetPosition(),
                ctx.Velocity,
                target,
                weapon.ProjectileSpeed,
                _maxLeadIterations,
                out aimDirection))
            {
                IsAligned = false;
                return;
            }

            Vector3D desiredUp = _desiredUpFunc != null
                ? _desiredUpFunc()
                : (ctx.Gravity.LengthSquared() > 0.1 ? -Vector3D.Normalize(ctx.Gravity) : ctx.WorldMatrix.Up);

            ctx.Gyros.AlignBlockToDirection(
                weapon.AimBlock,
                Vector3D.Normalize(aimDirection),
                Vector3D.Normalize(desiredUp));

            Vector3D blockForward = weapon.AimBlock.WorldMatrix.Forward;
            double dot = Vector3D.Dot(Vector3D.Normalize(blockForward), Vector3D.Normalize(aimDirection));
            AlignmentErrorDeg = Math.Acos(MathHelper.Clamp(dot, -1, 1)) * (180.0 / Math.PI);
            IsAligned = AlignmentErrorDeg <= _fireAngleDeg;
        }

        private static bool TryGetAimDirection(
            Vector3D shooterPos,
            Vector3D shooterVelocity,
            ITargetTelemetry target,
            double projectileSpeed,
            int maxIterations,
            out Vector3D aimDirection)
        {
            aimDirection = Vector3D.Zero;

            Vector3D toTarget = target.Position - shooterPos;
            if (toTarget.LengthSquared() < 0.001)
            {
                return false;
            }

            if (projectileSpeed <= 0.001)
            {
                aimDirection = toTarget;
                return true;
            }

            Vector3D relativeVelocity = target.Velocity - shooterVelocity;
            Vector3D acceleration = target.Acceleration;

            double time = toTarget.Length() / projectileSpeed;

            for (int i = 0; i < maxIterations; i++)
            {
                Vector3D predicted = target.Position
                    + relativeVelocity * time
                    + 0.5 * acceleration * time * time;

                Vector3D toPredicted = predicted - shooterPos;
                double newTime = toPredicted.Length() / projectileSpeed;

                aimDirection = toPredicted;

                if (Math.Abs(newTime - time) < 0.01 * Math.Max(time, 0.001))
                {
                    return aimDirection.LengthSquared() > 0.001;
                }

                time = newTime;
            }

            return aimDirection.LengthSquared() > 0.001;
        }
    }
}
