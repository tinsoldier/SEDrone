using VRageMath;

namespace IngameScript
{
    /// <summary>
    /// Orient toward the closest detected threat.
    /// Handles threat lookup from TacticalContext.
    /// </summary>
    public class FaceClosestThreat : IOrientationBehavior
    {
        private Vector3D _lastThreatDirection = Vector3D.Zero;

        public void Execute(DroneContext ctx)
        {
            Vector3D? closestThreat = ctx.Tactical.GetClosestThreatPosition(ctx.Position);

            if (closestThreat.HasValue)
            {
                Vector3D threatDir = Vector3D.Normalize(closestThreat.Value - ctx.Position);
                _lastThreatDirection = threatDir;
                ctx.Gyros.LookAt(closestThreat.Value);
            }
            else if (_lastThreatDirection.LengthSquared() > 0.1)
            {
                Vector3D target = ctx.Position + _lastThreatDirection * 1000.0;
                ctx.Gyros.LookAt(target);
            }
            else
            {
                ctx.Gyros.OrientLevel();
            }
        }
    }
}
