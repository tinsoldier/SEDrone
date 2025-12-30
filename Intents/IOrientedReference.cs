using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Represents a reference frame with position, orientation, and velocity.
    /// Used for local-to-world coordinate transformations in formation flying.
    ///
    /// Any object that has a position and orientation can implement this interface,
    /// making it usable as a reference frame for relative positioning.
    /// </summary>
    public interface IOrientedReference
    {
        /// <summary>World-space position of the reference frame origin.</summary>
        Vector3D Position { get; }

        /// <summary>Forward direction (Z-axis) of the reference frame.</summary>
        Vector3D Forward { get; }

        /// <summary>Up direction (Y-axis) of the reference frame.</summary>
        Vector3D Up { get; }

        /// <summary>Velocity of the reference frame for velocity matching.</summary>
        Vector3D Velocity { get; }
    }
}
