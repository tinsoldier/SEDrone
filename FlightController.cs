using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// High-level flight controller that coordinates orientation and thrust.
    /// Wraps GyroController and ThrusterController for simplified usage.
    /// 
    /// Note: For Milestone 2, DroneBrain directly uses GyroController and 
    /// ThrusterController. This class may be used in future milestones for
    /// more complex flight modes (docking, terrain following, etc.)
    /// </summary>
    class FlightController
    {
        // Hardware controllers
        private GyroController _gyroController;
        private ThrusterController _thrusterController;
        private IMyShipController _reference;

        // State
        public Vector3D CurrentVelocity { get; private set; }
        public Vector3D CurrentPosition { get; private set; }
        public double CurrentSpeed { get; private set; }

        /// <summary>
        /// Initializes the flight controller with hardware.
        /// </summary>
        public void Initialize(IMyShipController reference, GyroController gyroController, 
                               ThrusterController thrusterController)
        {
            _reference = reference;
            _gyroController = gyroController;
            _thrusterController = thrusterController;
        }

        /// <summary>
        /// Updates state from ship sensors.
        /// </summary>
        public void UpdateState()
        {
            if (_reference == null) return;
            
            CurrentPosition = _reference.GetPosition();
            CurrentVelocity = _reference.GetShipVelocities().LinearVelocity;
            CurrentSpeed = _reference.GetShipSpeed();
        }

        /// <summary>
        /// Moves toward a position while facing it.
        /// </summary>
        public void MoveToAndFace(Vector3D targetPosition, Vector3D matchVelocity, 
                                   double maxSpeed, double precisionRadius)
        {
            UpdateState();
            
            _gyroController?.LookAt(targetPosition);
            _thrusterController?.MoveToward(targetPosition, matchVelocity, maxSpeed, precisionRadius);
        }

        /// <summary>
        /// Releases all hardware control.
        /// </summary>
        public void Release()
        {
            _gyroController?.Release();
            _thrusterController?.Release();
        }
    }
}