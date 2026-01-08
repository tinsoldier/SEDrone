using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    public class FixedWeaponRigProvider
    {
        private readonly IMyProgrammableBlock _pb;
        private readonly IMyGridTerminalSystem _gridTerminalSystem;
        private readonly long _gridEntityId;
        private readonly IMyTerminalBlock _aimReference;
        private readonly Program.WcPbApi _wcApi;
        private readonly System.Action<string> _echo;
        private readonly List<IMyTerminalBlock> _fixedWeapons = new List<IMyTerminalBlock>();
        private readonly DroneHardware _hardware;
        private bool _initialized;

        public FixedWeaponRigProvider(IMyGridTerminalSystem gridTerminalSystem, IMyProgrammableBlock me, IMyTerminalBlock aimReference, Program.WcPbApi wcApi, System.Action<string> echo = null, DroneHardware hardware = null)
        {
            _pb = me;
            _gridTerminalSystem = gridTerminalSystem;
            _hardware = hardware;
            _gridEntityId = hardware?.GridId ?? me?.CubeGrid.EntityId ?? 0;
            _aimReference = aimReference;
            _wcApi = wcApi;
            _echo = echo;

            // Capture weapons immediately at construction (while docked, WC API works)
            // Don't refresh later - in RefHack mode, WC API won't work on undocked drone grids
            CaptureWeaponsOnce();
        }

        public IFixedWeaponRig GetPrimaryFixedWeaponRig(double gameTime)
        {
            if (_gridEntityId == 0)
                return null;

            if (_fixedWeapons.Count == 0)
                return null;

            if (_aimReference == null)
                return null;

            return new CompositeFixedWeaponRig(_aimReference, _fixedWeapons, _wcApi, _echo);
        }

        public IMyTerminalBlock GetPrimaryWeaponBlock(double gameTime)
        {
            if (_gridEntityId == 0)
                return null;

            if (_fixedWeapons.Count == 0)
                return null;

            return _fixedWeapons[0];
        }

        public void StopAllWeapons()
        {
            if (_fixedWeapons.Count == 0)
                return;
                    
            //_wcApi.ReleaseAiFocus(_pb, 0);

            for (int i = 0; i < _fixedWeapons.Count; i++)
            {
                var block = _fixedWeapons[i];
                if (block != null && block.IsFunctional)
                {
                    //_wcApi.ToggleWeaponFire(block, false, true, 0);
                    block.SetValue("WC_FocusFire", false);
                    block.SetValue("WC_Shoot", false);
                }
            }
        }

        /// <summary>
        /// Captures weapons once at initialization. Called from constructor while docked,
        /// when WC API still works via the leader's PB. After undocking, we rely on cached data.
        /// </summary>
        private void CaptureWeaponsOnce()
        {
            if (_initialized)
                return;
            _initialized = true;

            _fixedWeapons.Clear();

            if (_hardware == null)
            {
                _echo?.Invoke($"(WeaponRig) Hardware is null - cannot capture weapons");
                return;
            }

            // Refresh weapon definitions while we can (docked = WC API works)
            if (_gridTerminalSystem != null && _wcApi != null)
                _hardware.RefreshWeapons(_gridTerminalSystem, _wcApi);

            // Log weapon discovery summary
            //_echo?.Invoke($"(WeaponRig) {_hardware.GetWeaponSummary()}");

            var fixedWeapons = _hardware.FixedWeaponBlocks;
            if (fixedWeapons == null || fixedWeapons.Count == 0)
            {
                return;
            }

            for (int i = 0; i < fixedWeapons.Count; i++)
            {
                var block = fixedWeapons[i];
                if (block != null &&
                    block.IsFunctional &&
                    block.CubeGrid.EntityId == _gridEntityId)
                {
                    _fixedWeapons.Add(block);
                }
            }

            //_echo?.Invoke($"(WeaponRig) Captured {_fixedWeapons.Count} weapons for targeting");
        }

        public class CompositeFixedWeaponRig : IFixedWeaponRig
        {
            private readonly IMyTerminalBlock _aimBlock;
            public readonly List<IMyTerminalBlock> _weaponBlocks;
            private readonly Program.WcPbApi _wcApi;
            private readonly System.Action<string> _echo;
            //private bool _monitorRegistered;
            private bool? _lastFireState;
            private double? _cachedMaxRange;

            public CompositeFixedWeaponRig(IMyTerminalBlock aimBlock, List<IMyTerminalBlock> weaponBlocks, Program.WcPbApi wcApi, System.Action<string> echo)
            {
                _aimBlock = aimBlock;
                _weaponBlocks = weaponBlocks;
                _wcApi = wcApi;
                _echo = echo;
            }

            public IMyCubeBlock AimBlock => _aimBlock;
            public double ProjectileSpeed => 1700; // TODO: wire actual projectile speed
            public double MaxRange => GetCachedMaxRange();
            public bool IsWeaponReady => AnyWeaponReady();

            public void Fire(bool enable)
            {
                //_echo?.Invoke($"(Aim) Fire called with enable={enable}");
                if (_weaponBlocks == null || _weaponBlocks.Count == 0)
                    return;
                // if (enable && _lastFireState.HasValue && _lastFireState.Value == enable)
                //     return;
                _lastFireState = enable;
                // if (enable && !_monitorRegistered && _wcApi != null)
                // {
                //     _wcApi.MonitorProjectileCallback(_weaponBlocks[0], 0, OnProjectileUpdate);
                //     _monitorRegistered = true;
                // }
                //_echo?.Invoke($"(Aim) Setting fire={enable}");
                for (int i = 0; i < _weaponBlocks.Count; i++)
                {
                    var block = _weaponBlocks[i];
                    if (block != null && block.IsFunctional)
                    {
                        //_wcApi.ToggleWeaponFire(block, enable, true, 0);
                        block.SetValue("WC_FocusFire", enable);
                        block.SetValue("WC_Shoot", enable);
                    }
                }
            }

            // private void OnProjectileUpdate(long blockEntityId, int partId, ulong projectileId, long targetEntityId, Vector3D position, bool exists)
            // {
            //     _echo?.Invoke($"(Aim) Projectile callback");

            //     var state = _wcApi.GetProjectileState(projectileId);
            //     Vector3D velocity = state.Item2;
            //     double speed = velocity.Length();
            //     if(exists)
            //         _echo?.Invoke($"(Aim) Start Projectile speed={speed:F1} m/s");
            //     else
            //         _echo?.Invoke($"(Aim) End Projectile speed={speed:F1} m/s, dist to target={(position - AimBlock.GetPosition()).Length():F1} m");
            // }

            private double GetCachedMaxRange()
            {
                if (_cachedMaxRange.HasValue)
                    return _cachedMaxRange.Value;

                if (_weaponBlocks == null || _weaponBlocks.Count == 0)
                {
                    _cachedMaxRange = 0;
                    return 0;
                }

                double minRange = double.MaxValue;
                for (int i = 0; i < _weaponBlocks.Count; i++)
                {
                    var block = _weaponBlocks[i];
                    if (block == null || !block.IsFunctional)
                        continue;

                    double range = _wcApi.GetMaxWeaponRange(block, 0);
                    if (range > 0 && range < minRange)
                        minRange = range;
                }

                _cachedMaxRange = minRange == double.MaxValue ? 0 : minRange;
                return _cachedMaxRange.Value;
            }

            private bool AnyWeaponReady()
            {
                if (_weaponBlocks == null || _weaponBlocks.Count == 0)
                    return false;

                for (int i = 0; i < _weaponBlocks.Count; i++)
                {
                    var block = _weaponBlocks[i];
                    if (block == null || !block.IsFunctional)
                        continue;
                    if (_wcApi.IsWeaponReadyToFire(block))
                        return true;
                }
                return false;
            }
        }
    }
}
