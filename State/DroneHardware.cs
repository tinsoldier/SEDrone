using System.Collections.Generic;
using Sandbox.ModAPI.Ingame;

namespace IngameScript
{
    /// <summary>
    /// Pre-collected block references for a detached drone grid.
    /// Intended for ref-hack scenarios where blocks are captured while docked.
    /// </summary>
    public class DroneHardware
    {
        public long GridId { get; set; }
        public IMyShipController Reference { get; set; }

        public List<IMyGyro> Gyros { get; } = new List<IMyGyro>();
        public List<IMyThrust> Thrusters { get; } = new List<IMyThrust>();
        public List<IMyShipConnector> Connectors { get; } = new List<IMyShipConnector>();
        public List<IMyTerminalBlock> WeaponBlocks { get; } = new List<IMyTerminalBlock>();

        public bool IsValid()
        {
            return Reference != null && Reference.CubeGrid != null;
        }

        public static DroneHardware Capture(IMyGridTerminalSystem gts, long gridId, IMyShipController reference)
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
            gts.GetBlocksOfType<IMyTerminalBlock>(hardware.WeaponBlocks, b => b.CubeGrid.EntityId == hardware.GridId);

            return hardware;
        }

        // TODO: Add Refresh(IMyGridTerminalSystem gts) for non-refhack mode to replace stale references.
    }
}
