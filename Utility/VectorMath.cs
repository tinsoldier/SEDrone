using System;
using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Static utility class for common vector math operations.
    /// Provides gravity-aware projections and other reusable calculations.
    /// </summary>
    public static class VectorMath
    {
        /// <summary>
        /// Projects a direction vector onto the horizontal plane (perpendicular to gravity).
        /// Useful for extracting compass heading from a 3D orientation.
        /// </summary>
        /// <param name="direction">The direction vector to project</param>
        /// <param name="gravity">The gravity vector (points down, doesn't need to be normalized)</param>
        /// <param name="fallbackRight">Optional fallback right vector if direction is parallel to gravity</param>
        /// <returns>The projected direction, normalized. Returns Vector3D.Zero if degenerate.</returns>
        public static Vector3D ProjectOntoHorizontalPlane(Vector3D direction, Vector3D gravity, Vector3D? fallbackRight = null)
        {
            // No gravity = can't define horizontal
            if (gravity.LengthSquared() < 0.1)
            {
                return direction.LengthSquared() > 0.001 ? Vector3D.Normalize(direction) : Vector3D.Zero;
            }

            Vector3D gravityNorm = Vector3D.Normalize(gravity);
            
            // Project direction onto plane perpendicular to gravity
            // Formula: v_projected = v - (v · n) * n, where n is the plane normal (gravity direction)
            Vector3D projected = direction - gravityNorm * Vector3D.Dot(direction, gravityNorm);

            if (projected.LengthSquared() > 0.001)
            {
                return Vector3D.Normalize(projected);
            }

            // Direction is parallel to gravity (pointing straight up/down)
            // Use fallback if provided
            if (fallbackRight.HasValue)
            {
                Vector3D fallbackForward = Vector3D.Cross(fallbackRight.Value, gravityNorm);
                if (fallbackForward.LengthSquared() > 0.001)
                {
                    return Vector3D.Normalize(fallbackForward);
                }
            }

            // Completely degenerate - return zero to signal caller should handle
            return Vector3D.Zero;
        }
    }
}
