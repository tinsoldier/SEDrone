using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Helper struct that wraps connector position/orientation information
    /// into an IOrientedReference for use with Move behavior.
    ///
    /// This allows treating a moving connector as a reference frame,
    /// making docking logic cleaner and more aligned with formation flying.
    /// </summary>
    public struct ConnectorReference : IOrientedReference
    {
        private readonly Vector3D _position;
        private readonly Vector3D _forward;
        private readonly Vector3D _up;
        private readonly Vector3D _velocity;

        public ConnectorReference(Vector3D position, Vector3D forward, Vector3D up, Vector3D velocity)
        {
            _position = position;
            // Normalize orientation vectors to ensure validity
            _forward = forward.LengthSquared() > 0.001 ? Vector3D.Normalize(forward) : Vector3D.Forward;
            _up = up.LengthSquared() > 0.001 ? Vector3D.Normalize(up) : Vector3D.Up;
            _velocity = velocity;
        }

        public Vector3D Position => _position;
        public Vector3D Forward => _forward;
        public Vector3D Up => _up;
        public Vector3D Velocity => _velocity;

        /// <summary>
        /// Creates a ConnectorReference with zero velocity.
        /// </summary>
        public static ConnectorReference Stationary(Vector3D position, Vector3D forward, Vector3D up)
        {
            return new ConnectorReference(position, forward, up, Vector3D.Zero);
        }
    }
}
