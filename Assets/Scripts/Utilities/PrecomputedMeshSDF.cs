using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

// Pre-computed mesh SDF for high-quality flow field generation
public struct MeshSDFData : IComponentData
{
    public float3 BoundsMin;
    public float3 BoundsMax;
    public int3 Resolution;        // SDF grid resolution
    public float CellSize;
    public float MaxDistance;      // Distance beyond which SDF is clamped
}

// SDF grid data - stores signed distance for each cell
[InternalBufferCapacity(0)]
public struct MeshSDFCell : IBufferElementData
{
    public float SignedDistance; // Negative inside mesh, positive outside
    public float3 SurfaceNormal; // Direction to nearest surface
}

[BurstCompile]
public static class PrecomputedMeshSDF
{
    // Sample pre-computed mesh SDF for flow field generation
    public static float SampleMeshSDF(
        float3 worldPosition,
        Entity meshEntity,
        in MeshSDFData sdfData,
        in DynamicBuffer<MeshSDFCell> sdfCells)
    {
        // Convert world position to SDF grid coordinates
        float3 localPos = worldPosition - sdfData.BoundsMin;
        
        if (math.any(localPos < 0) || math.any(localPos >= (sdfData.BoundsMax - sdfData.BoundsMin)))
        {
            // Outside SDF bounds - return distance to bounds
            return DistanceToBounds(worldPosition, sdfData.BoundsMin, sdfData.BoundsMax);
        }
        
        float3 gridPosFloat = localPos / sdfData.CellSize;
        int3 gridPos = (int3)gridPosFloat;
        
        // Bounds check
        if (math.any(gridPos < 0) || math.any(gridPos >= sdfData.Resolution))
            return sdfData.MaxDistance;
        
        // Trilinear interpolation for smooth SDF sampling
        return SampleSDFTrilinear(gridPosFloat, gridPos, sdfData, sdfCells);
    }
    
    public static float3 SampleMeshSDFGradient(
        float3 worldPosition,
        Entity meshEntity,
        in MeshSDFData sdfData,
        in DynamicBuffer<MeshSDFCell> sdfCells)
    {
        float epsilon = sdfData.CellSize * 0.5f;
        
        // Calculate gradient using finite differences
        float dx = SampleMeshSDF(worldPosition + new float3(epsilon, 0, 0), meshEntity, sdfData, sdfCells) -
                   SampleMeshSDF(worldPosition - new float3(epsilon, 0, 0), meshEntity, sdfData, sdfCells);
                   
        float dy = SampleMeshSDF(worldPosition + new float3(0, epsilon, 0), meshEntity, sdfData, sdfCells) -
                   SampleMeshSDF(worldPosition - new float3(0, epsilon, 0), meshEntity, sdfData, sdfCells);
                   
        float dz = SampleMeshSDF(worldPosition + new float3(0, 0, epsilon), meshEntity, sdfData, sdfCells) -
                   SampleMeshSDF(worldPosition - new float3(0, 0, epsilon), meshEntity, sdfData, sdfCells);
        
        return math.normalize(new float3(dx, dy, dz) / (2f * epsilon));
    }
    
    private static float SampleSDFTrilinear(
        float3 gridPosFloat,
        int3 gridPos,
        in MeshSDFData sdfData,
        in DynamicBuffer<MeshSDFCell> sdfCells)
    {
        // Get fractional part for interpolation
        float3 t = gridPosFloat - gridPos;

        // Sample 8 neighboring cells - using individual variables for Burst compatibility
        float value0 = GetSDFValue(gridPos + new int3(0, 0, 0), sdfData, sdfCells);
        float value1 = GetSDFValue(gridPos + new int3(1, 0, 0), sdfData, sdfCells);
        float value2 = GetSDFValue(gridPos + new int3(0, 1, 0), sdfData, sdfCells);
        float value3 = GetSDFValue(gridPos + new int3(1, 1, 0), sdfData, sdfCells);
        float value4 = GetSDFValue(gridPos + new int3(0, 0, 1), sdfData, sdfCells);
        float value5 = GetSDFValue(gridPos + new int3(1, 0, 1), sdfData, sdfCells);
        float value6 = GetSDFValue(gridPos + new int3(0, 1, 1), sdfData, sdfCells);
        float value7 = GetSDFValue(gridPos + new int3(1, 1, 1), sdfData, sdfCells);

        // Trilinear interpolation
        float c00 = math.lerp(value0, value1, t.x);
        float c01 = math.lerp(value2, value3, t.x);
        float c10 = math.lerp(value4, value5, t.x);
        float c11 = math.lerp(value6, value7, t.x);

        float c0 = math.lerp(c00, c01, t.y);
        float c1 = math.lerp(c10, c11, t.y);

        return math.lerp(c0, c1, t.z);
    }
    
    private static float GetSDFValue(int3 gridPos, in MeshSDFData sdfData, in DynamicBuffer<MeshSDFCell> sdfCells)
    {
        // Bounds check
        if (math.any(gridPos < 0) || math.any(gridPos >= sdfData.Resolution))
            return sdfData.MaxDistance;
            
        int index = gridPos.x + gridPos.y * sdfData.Resolution.x + 
                   gridPos.z * sdfData.Resolution.x * sdfData.Resolution.y;
                   
        if (index >= sdfCells.Length)
            return sdfData.MaxDistance;
            
        return sdfCells[index].SignedDistance;
    }
    
    private static float DistanceToBounds(float3 point, float3 boundsMin, float3 boundsMax)
    {
        float3 center = (boundsMin + boundsMax) * 0.5f;
        float3 extents = (boundsMax - boundsMin) * 0.5f;
        float3 offset = math.abs(point - center);
        float3 distance = offset - extents;
        
        return math.length(math.max(distance, 0f)) + 
               math.min(math.max(distance.x, math.max(distance.y, distance.z)), 0f);
    }
}

// System to pre-compute mesh SDFs (run during initialization)
[BurstCompile]
[UpdateInGroup(typeof(InitializationSystemGroup))]
public partial struct PrecomputeMeshSDFSystem : ISystem
{
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<MeshSDFData>();
    }
    
    public void OnUpdate(ref SystemState state)
    {
        // This would generate SDF data for mesh colliders during initialization
        // Implementation would involve:
        // 1. Mesh voxelization
        // 2. Distance field computation
        // 3. Surface normal calculation
        // 
        // This is a complex process typically done offline or during loading
    }
}