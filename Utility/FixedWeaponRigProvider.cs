using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRage.Game;
using VRage.Game.ModAPI.Ingame;

namespace IngameScript
{
    public class FixedWeaponRigProvider
    {
        private static HashSet<MyDefinitionId> _staticLauncherDefs;

        private readonly IMyGridTerminalSystem _gridTerminalSystem;
        private readonly long _gridEntityId;
        private readonly Program.WcPbApi _wcApi;
        private readonly List<IMyTerminalBlock> _fixedWeapons = new List<IMyTerminalBlock>();
        private double _lastRefreshTime;

        private const double REFRESH_INTERVAL = 2.0;

        public FixedWeaponRigProvider(IMyGridTerminalSystem gridTerminalSystem, IMyProgrammableBlock me, Program.WcPbApi wcApi)
        {
            _gridTerminalSystem = gridTerminalSystem;
            _gridEntityId = me?.CubeGrid.EntityId ?? 0;
            _wcApi = wcApi;
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

            return new FixedWeaponRig(block, _wcApi);
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

            public FixedWeaponRig(IMyTerminalBlock weaponBlock, Program.WcPbApi wcApi)
            {
                _weaponBlock = weaponBlock;
                _wcApi = wcApi;
            }

            public IMyCubeBlock AimBlock => _weaponBlock;
            public double ProjectileSpeed => 0; // TODO: wire actual projectile speed
            public double MaxRange => _wcApi.GetMaxWeaponRange(_weaponBlock, 0);
            public bool IsWeaponReady => _wcApi.IsWeaponReadyToFire(_weaponBlock, 0, true, false);
            public bool CanFire => _weaponBlock != null && _weaponBlock.IsFunctional && _weaponBlock.IsWorking;

            public void Fire(bool enable)
            {
                if (_weaponBlock == null)
                    return;
                _wcApi.ToggleWeaponFire(_weaponBlock, enable, true, 0);
            }

            // public bool CanShootTarget(long targetEntityId)
            // {
            //     if (_weaponBlock == null || targetEntityId == 0)
            //         return false;

            //     /*
            //     _wcApi.CanShootTarget only checks:
            //     1.	Target distance (min/max range)
            //     2.	Target velocity prediction (lead calculation)
            //     3.	Turret azimuth/elevation constraints (can the turret physically rotate to aim there?)
            //     4.	Whether a ray from the pivot would intersect the target's OBB
            //     */
            //     return _wcApi.CanShootTarget(_weaponBlock, targetEntityId, 0); //0 weaponid is default weapon usually, call get weapon map if needed
            // }
        }
    }
}
