using Sandbox.ModAPI.Ingame;
using VRageMath;

namespace IngameScript.Utility
{
    class TerrainScanner
    {
        IMyCameraBlock Camera;      // For raycasting
        double LastGroundDistance;
        Vector3D LastGroundNormal;

        bool TryScanTerrain(out double altitude, out Vector3D normal);
    }
}