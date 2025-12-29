using System;
using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    // Note: IOrientationBehavior interface is defined in PositionBehavior.cs
    // to allow coupled behaviors that implement both interfaces.

    /// <summary>
    /// Look at a specific world position.
    /// </summary>
    public class LookAt : IOrientationBehavior
    {
        private readonly Func<Vector3D> _targetFunc;

        /// <summary>Target position to look at (evaluated each tick if dynamic).</summary>
        public Vector3D Target => _targetFunc();

        /// <summary>Look at fixed position.</summary>
        public LookAt(Vector3D target)
        {
            _targetFunc = () => target;
        }

        /// <summary>Look at dynamically-computed position.</summary>
        public LookAt(Func<Vector3D> targetFunc)
        {
            _targetFunc = targetFunc;
        }
    }

    /// <summary>
    /// Look in a specific direction (normalized vector).
    /// </summary>
    public class LookDirection : IOrientationBehavior
    {
        private readonly Func<Vector3D> _directionFunc;

        /// <summary>Direction to look (evaluated each tick if dynamic).</summary>
        public Vector3D Direction => _directionFunc();

        /// <summary>Look in fixed direction.</summary>
        public LookDirection(Vector3D direction)
        {
            _directionFunc = () => direction;
        }

        /// <summary>Look in dynamically-computed direction.</summary>
        public LookDirection(Func<Vector3D> directionFunc)
        {
            _directionFunc = directionFunc;
        }
    }

    /// <summary>
    /// Match leader's compass heading (yaw only, stays level to gravity).
    /// </summary>
    public class MatchLeader : IOrientationBehavior { }

    /// <summary>
    /// Orient toward the closest detected threat.
    /// Executor handles threat lookup from TacticalContext.
    /// </summary>
    public class FaceClosestThreat : IOrientationBehavior { }

    /// <summary>
    /// Maintain level orientation (no pitch/roll relative to gravity).
    /// </summary>
    public class StayLevel : IOrientationBehavior { }

    /// <summary>
    /// Align a specific connector on the drone to face a target direction.
    /// Used for docking - orients the ship so the selected connector faces
    /// opposite to the target connector's forward direction.
    /// 
    /// Inspired by SEAD2's AlignWithGravity which creates a virtual reference
    /// frame based on the connector orientation.
    /// </summary>
    public class AlignConnector : IOrientationBehavior
    {
        /// <summary>The drone's connector to align.</summary>
        public IMyShipConnector DroneConnector { get; private set; }

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

        public AlignConnector(
            IMyShipConnector droneConnector, 
            Func<Vector3D> targetDirection,
            Func<Vector3D> desiredUp = null)
        {
            DroneConnector = droneConnector;
            TargetDirection = targetDirection;
            DesiredUp = desiredUp;
        }

        public AlignConnector(
            IMyShipConnector droneConnector,
            Vector3D targetDirection,
            Vector3D? desiredUp = null)
        {
            DroneConnector = droneConnector;
            TargetDirection = () => targetDirection;
            DesiredUp = desiredUp.HasValue ? (Func<Vector3D>)(() => desiredUp.Value) : null;
        }
    }
}
