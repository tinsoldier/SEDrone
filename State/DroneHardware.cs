using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;
using VRage.Game;

namespace IngameScript
{
    /// <summary>
    /// Pre-collected block references for a detached drone grid.
    /// Intended for ref-hack scenarios where blocks are captured while docked.
    /// </summary>
    public class DroneHardware
    {
        private static HashSet<MyDefinitionId> _staticLauncherDefs;
        private static HashSet<MyDefinitionId> _staticFlareDefs;
        private static HashSet<MyDefinitionId> _staticTurretDefs;

        public long GridId { get; set; }
        public IMyShipController Reference { get; set; }

        public List<IMyGyro> Gyros { get; } = new List<IMyGyro>();
        public List<IMyThrust> Thrusters { get; } = new List<IMyThrust>();
        public List<IMyShipConnector> Connectors { get; } = new List<IMyShipConnector>();
        public List<IMyTerminalBlock> WeaponBlocks { get; } = new List<IMyTerminalBlock>();
        public List<IMyTerminalBlock> FixedWeaponBlocks { get; } = new List<IMyTerminalBlock>();
        public List<IMyTerminalBlock> FlareBlocks { get; } = new List<IMyTerminalBlock>();
        public List<IMyTerminalBlock> TurretBlocks { get; } = new List<IMyTerminalBlock>();

        public bool IsValid()
        {
            return Reference != null && Reference.CubeGrid != null;
        }

        public static DroneHardware Capture(IMyGridTerminalSystem gts, long gridId, IMyShipController reference, Program.WcPbApi wcApi = null)
        {
            var hardware = new DroneHardware
            {
                GridId = gridId != 0 ? gridId : reference?.CubeGrid.EntityId ?? 0,
                Reference = reference
            };

            if (gts == null || hardware.GridId == 0)
                return hardware;

            gts.GetBlocksOfType(hardware.Gyros, g => g.CubeGrid.EntityId == hardware.GridId);
            gts.GetBlocksOfType(hardware.Thrusters, t => t.CubeGrid.EntityId == hardware.GridId);
            gts.GetBlocksOfType(hardware.Connectors, c => c.CubeGrid.EntityId == hardware.GridId);
            hardware.RefreshWeapons(gts, wcApi);

            return hardware;
        }

        public void RefreshConnectors(IMyGridTerminalSystem gts)
        {
            if (gts == null || GridId == 0)
                return;

            Connectors.Clear();
            gts.GetBlocksOfType(Connectors, c => c.CubeGrid.EntityId == GridId);
        }

        public void RefreshWeapons(IMyGridTerminalSystem gts, Program.WcPbApi wcApi)
        {
            WeaponBlocks.Clear();
            FixedWeaponBlocks.Clear();
            FlareBlocks.Clear();
            TurretBlocks.Clear();

            if (gts == null || GridId == 0)
                return;

            if (!EnsureWeaponDefs(wcApi))
                return;

            gts.GetBlocksOfType<IMyTerminalBlock>(
                WeaponBlocks,
                b => b.CubeGrid.EntityId == GridId && IsWeaponDefinition(b.BlockDefinition));

            for (int i = 0; i < WeaponBlocks.Count; i++)
            {
                var block = WeaponBlocks[i];
                if (block == null)
                    continue;

                var def = block.BlockDefinition;
                if (_staticTurretDefs.Contains(def))
                {
                    TurretBlocks.Add(block);
                    continue;
                }

                if (_staticLauncherDefs.Contains(def))
                {
                    if (_staticFlareDefs.Contains(def))
                        FlareBlocks.Add(block);
                    else
                        FixedWeaponBlocks.Add(block);
                }
            }
        }

        private static bool EnsureWeaponDefs(Program.WcPbApi wcApi)
        {
            if (_staticLauncherDefs != null && _staticFlareDefs != null && _staticTurretDefs != null)
                return true;

            if (wcApi == null)
                return false;

            var launcherDefs = new HashSet<MyDefinitionId>();
            wcApi.GetAllCoreStaticLaunchers(launcherDefs);

            var turretDefs = new HashSet<MyDefinitionId>();
            wcApi.GetAllCoreTurrets(turretDefs);

            var flareDefs = new HashSet<MyDefinitionId>();
            foreach (var def in launcherDefs)
            {
                if (def.SubtypeName.Contains("Flare"))
                    flareDefs.Add(def);
            }

            _staticLauncherDefs = launcherDefs;
            _staticTurretDefs = turretDefs;
            _staticFlareDefs = flareDefs;
            return true;
        }

        private static bool IsWeaponDefinition(MyDefinitionId definitionId)
        {
            return _staticLauncherDefs.Contains(definitionId) || _staticTurretDefs.Contains(definitionId);
        }

        // TODO: Add Refresh(IMyGridTerminalSystem gts) for non-refhack mode to replace stale references.
    }
}
