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
        private readonly Program.WcPbApi _wcApi;
        private readonly System.Action<string> _echo;
        private readonly List<IMyTerminalBlock> _fixedWeapons = new List<IMyTerminalBlock>();
        private double _lastRefreshTime;

        private const double REFRESH_INTERVAL = 2.0;

        public FixedWeaponRigProvider(IMyGridTerminalSystem gridTerminalSystem, IMyProgrammableBlock me, Program.WcPbApi wcApi, System.Action<string> echo = null)
        {
            _gridTerminalSystem = gridTerminalSystem;
            _gridEntityId = me?.CubeGrid.EntityId ?? 0;
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

            var block = _fixedWeapons[0];
            if (block == null || !block.IsFunctional)
                return null;

            return new FixedWeaponRig(block, _wcApi, _echo);
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
            _staticLauncherDefs = defs;
            return _staticLauncherDefs;
        }

        private class FixedWeaponRig : IFixedWeaponRig
        {
            private readonly IMyTerminalBlock _weaponBlock;
            private readonly Program.WcPbApi _wcApi;
            private readonly System.Action<string> _echo;
            private readonly HashSet<ulong> _seenProjectiles = new HashSet<ulong>();
            private bool _monitorRegistered;

            public FixedWeaponRig(IMyTerminalBlock weaponBlock, Program.WcPbApi wcApi, System.Action<string> echo)
            {
                _weaponBlock = weaponBlock;
                _wcApi = wcApi;
                _echo = echo;
            }

            public IMyCubeBlock AimBlock => _weaponBlock;
            public double ProjectileSpeed => 2025; // TODO: wire actual projectile speed
            public double MaxRange => _wcApi.GetMaxWeaponRange(_weaponBlock, 0);
            public bool IsWeaponReady => _wcApi.IsWeaponReadyToFire(_weaponBlock, 0, true, false);
            public bool CanFire => _weaponBlock != null && _weaponBlock.IsFunctional && _weaponBlock.IsWorking;

            public void Fire(bool enable)
            {
                //_echo?.Invoke($"(Aim) Fire called with enable={enable}");
                if (_weaponBlock == null)
                    return;
                // if (enable && !_monitorRegistered && _wcApi != null)
                // {
                //     _wcApi.MonitorProjectileCallback(_weaponBlock, 0, OnProjectileUpdate);
                //     _monitorRegistered = true;
                // }
                _wcApi.ToggleWeaponFire(_weaponBlock, enable, true, 0);
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
        }
    }
}
