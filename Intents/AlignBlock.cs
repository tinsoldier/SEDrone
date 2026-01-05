using System;
using Sandbox.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{

    /// <summary>
    /// Align a specific block on the drone to face a target direction.
    /// Used for docking - orients the ship so the selected block faces
    /// opposite to the target connector's forward direction.
    /// 
    /// Inspired by SEAD2's AlignWithGravity which creates a virtual reference
    /// frame based on the connector orientation.
    /// </summary>
    public class AlignBlock : IOrientationBehavior
    {
        /// <summary>The drone's block to align.</summary>
        public IMyCubeBlock Block { get; private set; }

        /// <summary>
        /// Function returning the direction the drone connector should face (world space).
        /// Typically the negation of the target connector's forward.
        /// </summary>
        public Func<Vector3D> TargetDirection { get; private set; }

        /// <summary>
        /// Optional: desired "up" direction for the connector alignment.
        /// If null, uses gravity-aligned up.
        /// </summary>
        public Func<Vector3D> DesiredUp { get; private set; }

        public AlignBlock(
            IMyCubeBlock block,
            Func<Vector3D> targetDirection,
            Func<Vector3D> desiredUp = null)
        {
            Block = block;
            TargetDirection = targetDirection;
            DesiredUp = desiredUp;
        }

        public AlignBlock(
            IMyCubeBlock block,
            Vector3D targetDirection,
            Vector3D? desiredUp = null)
        {
            Block = block;
            TargetDirection = () => targetDirection;
            DesiredUp = desiredUp.HasValue ? (Func<Vector3D>)(() => desiredUp.Value) : null;
        }

        public void Execute(DroneContext ctx)
        {
            var block = Block;
            if (block == null || !block.IsFunctional)
            {
                ctx.Debug?.Log("AlignBlock: Block missing or not functional");
                ctx.Gyros.OrientLevel();
                return;
            }

            Vector3D targetDir = TargetDirection();
            if (targetDir.LengthSquared() < 0.1)
            {
                ctx.Debug?.Log("AlignBlock: Target direction invalid");
                ctx.Gyros.OrientLevel();
                return;
            }
            targetDir = Vector3D.Normalize(targetDir);

            // Get desired up direction (from behavior or gravity-based)
            Vector3D desiredUp;
            if (DesiredUp != null)
            {
                desiredUp = Vector3D.Normalize(DesiredUp());
            }
            else
            {
                // Use gravity-aligned up
                Vector3D gravity = ctx.Gravity;
                if (gravity.LengthSquared() > 0.1)
                {
                    desiredUp = -Vector3D.Normalize(gravity);
                }
                else
                {
                    desiredUp = ctx.WorldMatrix.Up;
                }
            }

            if (!ctx.Gyros.AlignBlockToDirection(block, targetDir, desiredUp))
            {
                ctx.Debug?.Log("AlignBlock: Gyro alignment failed");
                ctx.Gyros.OrientLevel();
            }
        }
    }
}
