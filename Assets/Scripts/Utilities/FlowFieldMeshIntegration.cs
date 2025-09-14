using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

// Enhanced flow field generation with proper mesh collider support
[BurstCompile]
public static class FlowFieldMeshIntegration
{
    // Enhanced SDF calculation that handles mesh colliders properly
    public static float CalculateEnhancedSDF(
        float3 queryPoint,
        SpatialObject spatialObj,
        ComponentLookup<MeshSDFData> meshSDFLookup,
        BufferLookup<MeshSDFCell> meshSDFCellLookup,
        ComponentLookup<ConvexHullData> convexLookup,
        BufferLookup<ConvexHullVertex> vertexLookup,
        BufferLookup<ConvexHullFace> faceLookup)
    {
        return spatialObj.ShapeType switch
        {
            CollisionShapeType.Point => math.length(spatialObj.Position - queryPoint),
            
            CollisionShapeType.Sphere => math.length(spatialObj.Position - queryPoint) - spatialObj.HalfExtents.x,
            
            CollisionShapeType.AABB => CalculateAABBDistance(queryPoint, spatialObj),
            
            CollisionShapeType.Mesh => CalculateMeshColliderDistance(
                queryPoint, spatialObj, meshSDFLookup, meshSDFCellLookup),
                
            CollisionShapeType.ConvexHull => CalculateConvexHullDistance(
                queryPoint, spatialObj, convexLookup, vertexLookup, faceLookup),
                
            CollisionShapeType.Compound => CalculateCompoundDistance(queryPoint, spatialObj),
            
            _ => math.length(spatialObj.Position - queryPoint)
        };
    }
    
    private static float CalculateMeshColliderDistance(
        float3 queryPoint,
        SpatialObject spatialObj,
        ComponentLookup<MeshSDFData> meshSDFLookup,
        BufferLookup<MeshSDFCell> meshSDFCellLookup)
    {
        // Check if mesh has pre-computed SDF
        if (meshSDFLookup.HasComponent(spatialObj.Entity) && 
            meshSDFCellLookup.HasBuffer(spatialObj.Entity))
        {
            // Use high-quality pre-computed SDF
            MeshSDFData sdfData = meshSDFLookup[spatialObj.Entity];
            DynamicBuffer<MeshSDFCell> sdfCells = meshSDFCellLookup[spatialObj.Entity];
            
            return PrecomputedMeshSDF.SampleMeshSDF(queryPoint, spatialObj.Entity, sdfData, sdfCells);
        }
        else
        {
            // Fallback to AABB approximation
            return CalculateAABBDistance(queryPoint, spatialObj);
        }
    }
    
    private static float CalculateConvexHullDistance(
        float3 queryPoint,
        SpatialObject spatialObj,
        ComponentLookup<ConvexHullData> convexLookup,
        BufferLookup<ConvexHullVertex> vertexLookup,
        BufferLookup<ConvexHullFace> faceLookup)
    {
        // Simplified convex hull distance calculation
        // In practice, this would use the Separating Axis Theorem
        
        if (convexLookup.HasComponent(spatialObj.Entity))
        {
            ConvexHullData hull = convexLookup[spatialObj.Entity];
            float distanceToCenter = math.length(queryPoint - hull.Center);
            
            // Simple approximation: distance to center minus max radius
            return math.max(0f, distanceToCenter - hull.MaxRadius);
        }
        
        // Fallback to AABB
        return CalculateAABBDistance(queryPoint, spatialObj);
    }
    
    private static float CalculateCompoundDistance(float3 queryPoint, SpatialObject spatialObj)
    {
        // For compound colliders, we'd need to check all child shapes
        // This is complex and expensive - fallback to AABB for now
        return CalculateAABBDistance(queryPoint, spatialObj);
    }
    
    private static float CalculateAABBDistance(float3 queryPoint, SpatialObject spatialObj)
    {
        float3 center = spatialObj.Position;
        float3 extents = spatialObj.HalfExtents;
        
        float3 offset = math.abs(queryPoint - center);
        float3 distance = offset - extents;
        
        return math.length(math.max(distance, 0f)) + 
               math.min(math.max(distance.x, math.max(distance.y, distance.z)), 0f);
    }
}

// Enhanced parallel SDF job with mesh collider support
[BurstCompile]
public struct EnhancedParallelSDFGenerationJob : IJobParallelFor
{
    [ReadOnly] public FlowField FlowField;
    [ReadOnly] public DynamicBuffer<OctreeNode> Nodes;
    [ReadOnly] public DynamicBuffer<SpatialObject> Objects;
    
    // Mesh collider support
    [ReadOnly] public ComponentLookup<MeshSDFData> MeshSDFLookup;
    [ReadOnly] public BufferLookup<MeshSDFCell> MeshSDFCellLookup;
    [ReadOnly] public ComponentLookup<ConvexHullData> ConvexLookup;
    [ReadOnly] public BufferLookup<ConvexHullVertex> VertexLookup;
    [ReadOnly] public BufferLookup<ConvexHullFace> FaceLookup;
    
    [NativeDisableParallelForRestriction]
    public DynamicBuffer<FlowCell> Cells;
    
    public void Execute(int index)
    {
        int3 gridPos = IndexToGrid(index, FlowField.Resolution);
        float3 worldPos = GridToWorldPos(gridPos, FlowField);
        
        float sdfValue = SampleNearestObstacleEnhanced(worldPos, Nodes, Objects);
        
        FlowCell cell = Cells[index];
        cell.DistanceToObstacle = sdfValue;
        Cells[index] = cell;
    }
    
    private float SampleNearestObstacleEnhanced(
        float3 worldPos, 
        in DynamicBuffer<OctreeNode> nodes, 
        in DynamicBuffer<SpatialObject> objects)
    {
        if (nodes.Length == 0)
            return float.MaxValue;
            
        float minDistance = float.MaxValue;
        float maxSearchRadius = 20f;
        
        TraverseOctreeEnhanced(worldPos, maxSearchRadius, 0, nodes, objects, ref minDistance);
        return minDistance;
    }
    
    private void TraverseOctreeEnhanced(
        float3 queryPoint,
        float maxSearchRadius,
        int nodeIndex,
        in DynamicBuffer<OctreeNode> nodes,
        in DynamicBuffer<SpatialObject> objects,
        ref float minDistance)
    {
        if (nodeIndex >= nodes.Length) return;
        
        OctreeNode node = nodes[nodeIndex];
        float nodeDistance = DistanceToOctreeNode(queryPoint, node);
        
        if (nodeDistance > maxSearchRadius || nodeDistance > minDistance) return;
        
        if (node.IsLeaf)
        {
            for (int i = node.ObjectStartIndex; i < node.ObjectStartIndex + node.ObjectCount; i++)
            {
                if (i >= objects.Length) break;
                
                SpatialObject obj = objects[i];
                
                // Use enhanced SDF calculation with mesh support
                float distance = FlowFieldMeshIntegration.CalculateEnhancedSDF(
                    queryPoint, obj, MeshSDFLookup, MeshSDFCellLookup, 
                    ConvexLookup, VertexLookup, FaceLookup);
                    
                minDistance = math.min(minDistance, distance);
                
                if (minDistance < 0.1f) return;
            }
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                int childIndex = node.ChildStartIndex + i;
                if (childIndex < nodes.Length)
                {
                    TraverseOctreeEnhanced(queryPoint, maxSearchRadius, childIndex, nodes, objects, ref minDistance);
                }
            }
        }
    }
    
    private float DistanceToOctreeNode(float3 point, OctreeNode node)
    {
        float3 offset = math.abs(point - node.Center);
        float3 distance = offset - new float3(node.HalfExtent);
        return math.length(math.max(distance, 0f)) + 
               math.min(math.max(distance.x, math.max(distance.y, distance.z)), 0f);
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