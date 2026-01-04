using System;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    public interface IFixedWeaponRig
    {
        IMyCubeBlock AimBlock { get; }
        double ProjectileSpeed { get; } // 0 = hitscan
        double MaxRange { get; } // 0 = unknown/unlimited
        bool IsWeaponReady { get; }
        bool CanFire { get; }
        void Fire(bool enable);
    }

    /// <summary>
    /// Aims a fixed weapon rig at a target using lead prediction.
    /// Orientation only - firing is handled by the directive/executor.
    /// </summary>
    public class AimFixedWeapons : IOrientationBehavior
    {
        private readonly Func<ITargetTelemetry> _targetFunc;
        private readonly Func<IFixedWeaponRig> _weaponFunc;
        private readonly bool _autoFire;

        private readonly int _maxLeadIterations;
        private readonly double _fireAngleDeg;
        private readonly int _stableTicksRequired;
        private readonly double _maxAngularRateRad;
        private int _alignedTicks;
        private long _lastTargetId;
        private bool _wasAligned;

        public bool IsAligned { get; private set; }
        public double AlignmentErrorDeg { get; private set; }
        public bool FireReady { get; private set; }

        public AimFixedWeapons(
            Func<ITargetTelemetry> targetFunc,
            Func<IFixedWeaponRig> weaponFunc,
            bool autoFire = true,
            int maxLeadIterations = 4,
            double fireAngleDeg = 1.0,
            int stableTicksRequired = 3,
            double maxAngularRateRad = 0.2)
        {
            _targetFunc = targetFunc;
            _weaponFunc = weaponFunc;
            _autoFire = autoFire;

            _maxLeadIterations = Math.Max(1, maxLeadIterations);
            _fireAngleDeg = Math.Max(0.1, fireAngleDeg);
            _stableTicksRequired = Math.Max(1, stableTicksRequired);
            _maxAngularRateRad = Math.Max(0.01, maxAngularRateRad);
        }

        public void Execute(DroneContext ctx)
        {
            var target = _targetFunc?.Invoke();
            var weapon = _weaponFunc?.Invoke();

            if (weapon == null || weapon.AimBlock == null || target == null || !target.IsValid)
            {
                IsAligned = false;
                FireReady = false;
                _alignedTicks = 0;
                if (weapon != null && _autoFire)
                {
                    weapon.Fire(false);
                }
                return;
            }

            if (_lastTargetId != target.EntityId)
            {
                _lastTargetId = target.EntityId;
                _alignedTicks = 0;
                FireReady = false;
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
                FireReady = false;
                _alignedTicks = 0;
                if (_autoFire)
                {
                    weapon.Fire(false);
                }
                return;
            }

            Vector3D desiredUp = ctx.Gravity.LengthSquared() > 0.1
                ? -Vector3D.Normalize(ctx.Gravity)
                : ctx.WorldMatrix.Up;

            ctx.Gyros.AlignBlockToDirection(
                weapon.AimBlock,
                Vector3D.Normalize(aimDirection),
                Vector3D.Normalize(desiredUp));

            Vector3D blockForward = weapon.AimBlock.WorldMatrix.Forward;
            double dot = Vector3D.Dot(Vector3D.Normalize(blockForward), Vector3D.Normalize(aimDirection));
            AlignmentErrorDeg = Math.Acos(MathHelper.Clamp(dot, -1, 1)) * (180.0 / Math.PI);

            double range = Vector3D.Distance(weapon.AimBlock.GetPosition(), target.Position);
            double dynamicFireAngleDeg = _fireAngleDeg;
            if (range > 0.1)
            {
                double maxSize = Math.Max(target.Size.X, Math.Max(target.Size.Y, target.Size.Z));
                double halfSize = Math.Max(0.1, maxSize * 0.5);
                double angularSizeDeg = Math.Atan(halfSize / range) * (180.0 / Math.PI);
                dynamicFireAngleDeg = Math.Max(0.05, Math.Min(_fireAngleDeg, angularSizeDeg));
            }

            double alignedThreshold = dynamicFireAngleDeg;
            if (_wasAligned)
            {
                alignedThreshold *= 1.5;
            }
            IsAligned = AlignmentErrorDeg <= alignedThreshold;
            _alignedTicks = IsAligned ? _alignedTicks + 1 : 0;
            _wasAligned = IsAligned;

            bool inRange = weapon.MaxRange <= 0 || range <= weapon.MaxRange;
            bool stable = _alignedTicks >= _stableTicksRequired;
            bool slowEnough = ctx.Gyros.AngularVelocity <= _maxAngularRateRad;
            FireReady = IsAligned && stable && slowEnough && weapon.IsWeaponReady && weapon.CanFire && inRange ;
            
            if (_autoFire)
            {
                ctx.Debug.Log($"(Aim) aligned={IsAligned}, stable={stable}, slowEnough={slowEnough}, inRange={inRange}, => FireReady={FireReady}");
                weapon.Fire(FireReady);
            }
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

            // Predict target position in world space
            // Use relative velocity because projectiles inherit shooter velocity.
            Vector3D targetVelocity = target.Velocity - shooterVelocity;
            Vector3D acceleration = target.Acceleration;

            double time = toTarget.Length() / projectileSpeed;

            for (int i = 0; i < maxIterations; i++)
            {
                // Predict where target will be at time T (in world space)
                Vector3D predicted = target.Position
                    + targetVelocity * time
                    + 0.5 * acceleration * time * time;

                // Calculate aim direction from shooter to predicted position
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
