using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRage.Game;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    public class FixedWeaponRigProvider
    {
        private static HashSet<MyDefinitionId> _staticLauncherDefs;

        private readonly IMyGridTerminalSystem _gridTerminalSystem;
        private readonly long _gridEntityId;
        private readonly IMyTerminalBlock _aimReference;
        private readonly Program.WcPbApi _wcApi;
        private readonly System.Action<string> _echo;
        private readonly List<IMyTerminalBlock> _fixedWeapons = new List<IMyTerminalBlock>();
        private double _lastRefreshTime;

        private const double REFRESH_INTERVAL = 2.0;

        public FixedWeaponRigProvider(IMyGridTerminalSystem gridTerminalSystem, IMyProgrammableBlock me, IMyTerminalBlock aimReference, Program.WcPbApi wcApi, System.Action<string> echo = null)
        {
            _gridTerminalSystem = gridTerminalSystem;
            _gridEntityId = me?.CubeGrid.EntityId ?? 0;
            _aimReference = aimReference;
            _wcApi = wcApi;
            _echo = echo;
        }

        public IFixedWeaponRig GetPrimaryFixedWeaponRig(double gameTime)
        {
            if (_wcApi == null || _gridEntityId == 0)
                return null;

            if ((gameTime - _lastRefreshTime) > REFRESH_INTERVAL || _fixedWeapons.Count == 0)
            {
                Refresh();
                _lastRefreshTime = gameTime;
            }

            if (_fixedWeapons.Count == 0)
                return null;

            if (_aimReference == null)
                return null;

            return new CompositeFixedWeaponRig(_aimReference, _fixedWeapons, _wcApi, _echo);
        }

        public IMyTerminalBlock GetPrimaryWeaponBlock(double gameTime)
        {
            if (_wcApi == null || _gridEntityId == 0)
                return null;

            if ((gameTime - _lastRefreshTime) > REFRESH_INTERVAL || _fixedWeapons.Count == 0)
            {
                Refresh();
                _lastRefreshTime = gameTime;
            }

            if (_fixedWeapons.Count == 0)
                return null;

            return _fixedWeapons[0];
        }

        private void Refresh()
        {
            _fixedWeapons.Clear();

            var wcDefs = GetStaticLauncherDefs();
            if (wcDefs == null || wcDefs.Count == 0)
                return;

            _gridTerminalSystem.GetBlocksOfType<IMyTerminalBlock>(
                _fixedWeapons,
                b => b.CubeGrid.EntityId == _gridEntityId && b.IsFunctional && wcDefs.Contains(b.BlockDefinition));
        }

        private HashSet<MyDefinitionId> GetStaticLauncherDefs()
        {
            if (_staticLauncherDefs != null)
                return _staticLauncherDefs;

            if (_wcApi == null)
                return null;

            var defs = new HashSet<MyDefinitionId>();
            _wcApi.GetAllCoreStaticLaunchers(defs);

            // remove defs with "Flare" in the subtype name (these are not aimed weapons)
            defs.RemoveWhere(d => d.SubtypeName.Contains("Flare"));

            _staticLauncherDefs = defs;
            return _staticLauncherDefs;
        }

        public class CompositeFixedWeaponRig : IFixedWeaponRig
        {
            private readonly IMyTerminalBlock _aimBlock;
            public readonly List<IMyTerminalBlock> _weaponBlocks;
            private readonly Program.WcPbApi _wcApi;
            private readonly System.Action<string> _echo;
            private readonly HashSet<ulong> _seenProjectiles = new HashSet<ulong>();
            private bool _monitorRegistered;
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
            public bool CanFire => AnyWeaponCanFire();

            public void Fire(bool enable)
            {
                //_echo?.Invoke($"(Aim) Fire called with enable={enable}");
                if (_weaponBlocks == null || _weaponBlocks.Count == 0)
                    return;
                if (_lastFireState.HasValue && _lastFireState.Value == enable)
                    return;
                _lastFireState = enable;
                // if (enable && !_monitorRegistered && _wcApi != null)
                // {
                //     _wcApi.MonitorProjectileCallback(_weaponBlocks[0], 0, OnProjectileUpdate);
                //     _monitorRegistered = true;
                // }
                for (int i = 0; i < _weaponBlocks.Count; i++)
                {
                    var block = _weaponBlocks[i];
                    if (block != null && block.IsFunctional)
                    {
                        _wcApi.ToggleWeaponFire(block, enable, true, 0);
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
                    if (_wcApi.IsWeaponReadyToFire(block, 0, true, false))
                        return true;
                }
                return false;
            }

            private bool AnyWeaponCanFire()
            {
                if (_weaponBlocks == null || _weaponBlocks.Count == 0)
                    return false;

                for (int i = 0; i < _weaponBlocks.Count; i++)
                {
                    var block = _weaponBlocks[i];
                    if (block != null && block.IsFunctional && block.IsWorking)
                        return true;
                }
                return false;
            }
        }
    }
}
