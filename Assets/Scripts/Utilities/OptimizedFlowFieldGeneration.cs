using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

// CRITICAL OPTIMIZATION: Fix the naive SDF generation
[BurstCompile]
public static class OptimizedSpatialSampling
{
    // Replace the slow SampleNearestObstacle with octree traversal
    public static float SampleNearestObstacleOptimized(
        float3 worldPos, 
        in DynamicBuffer<OctreeNode> nodes, 
        in DynamicBuffer<SpatialObject> objects,
        float maxSearchRadius = 20f)
    {
        if (nodes.Length == 0)
            return float.MaxValue;
            
        float minDistance = float.MaxValue;
        
        // Use octree traversal instead of brute force
        TraverseOctreeForDistance(worldPos, maxSearchRadius, 0, nodes, objects, ref minDistance);
        
        return minDistance;
    }
    
    private static void TraverseOctreeForDistance(
        float3 queryPoint,
        float maxSearchRadius,
        int nodeIndex,
        in DynamicBuffer<OctreeNode> nodes,
        in DynamicBuffer<SpatialObject> objects,
        ref float minDistance)
    {
        if (nodeIndex >= nodes.Length)
            return;
            
        OctreeNode node = nodes[nodeIndex];
        
        // Early exit if node is too far away
        AABB nodeBounds = new AABB { Center = node.Center, Extents = new float3(node.HalfExtent) };
        float nodeDistance = DistanceToAABB(queryPoint, nodeBounds);
        if (nodeDistance > maxSearchRadius || nodeDistance > minDistance)
            return;
            
        if (node.IsLeaf)
        {
            // Check all objects in this leaf
            for (int i = node.ObjectStartIndex; i < node.ObjectStartIndex + node.ObjectCount; i++)
            {
                if (i >= objects.Length) break;
                
                SpatialObject obj = objects[i];
                // Use half extents as radius approximation
                float objRadius = math.length(obj.HalfExtents);
                float distance = math.length(obj.Position - queryPoint) - objRadius;
                minDistance = math.min(minDistance, distance);
            }
        }
        else
        {
            // Recursively check child nodes
            for (int childIndex = node.ChildStartIndex; childIndex < node.ChildStartIndex + 8; childIndex++)
            {
                TraverseOctreeForDistance(queryPoint, maxSearchRadius, childIndex, nodes, objects, ref minDistance);
            }
        }
    }
    
    private static float DistanceToAABB(float3 point, AABB aabb)
    {
        float3 center = (aabb.Min + aabb.Max) * 0.5f;
        float3 extents = (aabb.Max - aabb.Min) * 0.5f;
        
        float3 offset = math.abs(point - center);
        float3 distance = offset - extents;
        
        return math.length(math.max(distance, 0f)) + math.min(math.max(distance.x, math.max(distance.y, distance.z)), 0f);
    }
}

// PERFORMANCE OPTIMIZATION: Parallel SDF generation
[BurstCompile]
public struct ParallelSDFGenerationJobAlternate : IJobParallelFor
{
    [ReadOnly] public FlowField FlowField;
    [ReadOnly] public DynamicBuffer<OctreeNode> Nodes;
    [ReadOnly] public DynamicBuffer<SpatialObject> Objects;
    
    [NativeDisableParallelForRestriction]
    public DynamicBuffer<FlowCell> Cells;
    
    public void Execute(int index)
    {
        // Convert 1D index to 3D grid coordinates
        int3 gridPos = IndexToGrid(index, FlowField.Resolution);
        float3 worldPos = GridToWorldPos(gridPos, FlowField);
        
        // Use optimized octree sampling
        float sdfValue = OptimizedSpatialSampling.SampleNearestObstacleOptimized(worldPos, Nodes, Objects);
        
        FlowCell cell = Cells[index];
        cell.DistanceToObstacle = sdfValue;
        Cells[index] = cell;
    }
    
    private int3 IndexToGrid(int index, int3 resolution)
    {
        int x = index % resolution.x;
        int y = (index / resolution.x) % resolution.y;
        int z = index / (resolution.x * resolution.y);
        return new int3(x, y, z);
    }
    
    private float3 GridToWorldPos(int3 gridPos, FlowField flowField)
    {
        return flowField.BoundsMin + new float3(gridPos) * flowField.CellSize;
    }
}