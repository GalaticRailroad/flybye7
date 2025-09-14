using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

// Navigation-specific octree node with field data storage
[InternalBufferCapacity(0)]
public struct NavigationOctreeNode : IBufferElementData
{
    public float3 Center;
    public float HalfExtent;
    public byte Depth;
    public bool IsLeaf;
    public int ParentIndex;

    // Child references (8 children stored consecutively, -1 if none)
    public int ChildStartIndex;

    // Navigation field data indices
    public int FieldDataIndex;      // Index into field data buffer
    public float Variance;           // How much the field varies in this region
    public NavigationTier Tier;     // Navigation tier classification

    // Statistics for adaptive subdivision
    public float MinDistance;        // Min distance to obstacle in this node
    public float MaxDistance;        // Max distance to obstacle in this node
    public float3 AverageFlow;      // Average flow direction in this node

    public static NavigationOctreeNode CreateLeaf(float3 center, float halfExtent, byte depth, int parentIndex)
    {
        return new NavigationOctreeNode
        {
            Center = center,
            HalfExtent = halfExtent,
            Depth = depth,
            IsLeaf = true,
            ParentIndex = parentIndex,
            ChildStartIndex = -1,
            FieldDataIndex = -1,
            Variance = 0f,
            Tier = NavigationTier.Far,
            MinDistance = float.MaxValue,
            MaxDistance = float.MaxValue,
            AverageFlow = float3.zero
        };
    }

    public static NavigationOctreeNode CreateBranch(float3 center, float halfExtent, byte depth, int parentIndex, int childStartIndex)
    {
        return new NavigationOctreeNode
        {
            Center = center,
            HalfExtent = halfExtent,
            Depth = depth,
            IsLeaf = false,
            ParentIndex = parentIndex,
            ChildStartIndex = childStartIndex,
            FieldDataIndex = -1,
            Variance = 0f,
            Tier = NavigationTier.Far,
            MinDistance = float.MaxValue,
            MaxDistance = float.MaxValue,
            AverageFlow = float3.zero
        };
    }
}

// Navigation tier classification for different navigation strategies
public enum NavigationTier : byte
{
    Far = 0,       // Far from obstacles - simple pathfinding
    Medium = 1,    // Medium distance - use flow fields
    Close = 2      // Close to obstacles - detailed avoidance
}

// Field data stored at each octree node
[InternalBufferCapacity(0)]
public struct NavigationFieldData : IBufferElementData
{
    public float SignedDistance;      // Distance to nearest obstacle (negative inside)
    public float3 VectorField;        // Pre-computed navigation direction
    public float3 GradientField;      // SDF gradient for obstacle avoidance
    public byte ObstacleFlags;        // Special obstacle properties

    // Corner values for trilinear interpolation (8 corners)
    public float Corner000Distance;
    public float Corner001Distance;
    public float Corner010Distance;
    public float Corner011Distance;
    public float Corner100Distance;
    public float Corner101Distance;
    public float Corner110Distance;
    public float Corner111Distance;
}

// Main navigation octree component
public struct NavigationOctree : IComponentData
{
    public float3 BoundsCenter;
    public float BoundsHalfExtent;
    public byte MaxDepth;
    public int RootNodeIndex;
    public int TotalNodes;
    public int NextFreeNodeIndex;
    public int NextFreeFieldIndex;
    public int PreallocatedNodeCapacity;
    public int PreallocatedFieldCapacity;

    // Subdivision parameters
    public float MinNodeSize;          // Minimum node size (based on actor size)
    public float SubdivisionThreshold; // Variance threshold for subdivision
    public float ProximityThreshold;   // Distance to obstacles that triggers subdivision

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float3 GetChildCenter(float3 parentCenter, float parentHalfExtent, int childIndex)
    {
        float quarter = parentHalfExtent * 0.5f;
        return parentCenter + new float3(
            (childIndex & 1) != 0 ? quarter : -quarter,
            (childIndex & 2) != 0 ? quarter : -quarter,
            (childIndex & 4) != 0 ? quarter : -quarter
        );
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int GetChildIndex(float3 parentCenter, float3 position)
    {
        int index = 0;
        if (position.x > parentCenter.x) index |= 1;
        if (position.y > parentCenter.y) index |= 2;
        if (position.z > parentCenter.z) index |= 4;
        return index;
    }
}

// Octree query operations
[BurstCompile]
public static class NavigationOctreeQueries
{
    // Find the leaf node containing a world position
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static int GetNodeContaining(float3 worldPosition, in NavigationOctree octree,
        in DynamicBuffer<NavigationOctreeNode> nodes)
    {
        // Check if position is within octree bounds
        float3 toPos = worldPosition - octree.BoundsCenter;
        if (math.any(math.abs(toPos) > octree.BoundsHalfExtent))
            return -1;

        int currentNode = octree.RootNodeIndex;

        while (currentNode >= 0 && currentNode < nodes.Length)
        {
            NavigationOctreeNode node = nodes[currentNode];

            if (node.IsLeaf)
                return currentNode;

            // Navigate to appropriate child
            int childIndex = NavigationOctree.GetChildIndex(node.Center, worldPosition);
            currentNode = node.ChildStartIndex + childIndex;
        }

        return -1;
    }

    // Find all nodes overlapping a sphere
    public static void GetNodesInRadius(float3 center, float radius, in NavigationOctree octree,
        in DynamicBuffer<NavigationOctreeNode> nodes, ref NativeList<int> resultNodes)
    {
        resultNodes.Clear();
        if (octree.TotalNodes > 0)
        {
            QueryNodeRadius(octree.RootNodeIndex, center, radius, nodes, ref resultNodes);
        }
    }

    private static void QueryNodeRadius(int nodeIndex, float3 center, float radius,
        in DynamicBuffer<NavigationOctreeNode> nodes, ref NativeList<int> resultNodes)
    {
        if (nodeIndex < 0 || nodeIndex >= nodes.Length)
            return;

        NavigationOctreeNode node = nodes[nodeIndex];

        // Check if sphere overlaps node bounds
        float3 closestPoint = math.clamp(center, node.Center - node.HalfExtent, node.Center + node.HalfExtent);
        float distSq = math.lengthsq(closestPoint - center);

        if (distSq > radius * radius)
            return;

        if (node.IsLeaf)
        {
            resultNodes.Add(nodeIndex);
        }
        else
        {
            // Check all children
            for (int i = 0; i < 8; i++)
            {
                QueryNodeRadius(node.ChildStartIndex + i, center, radius, nodes, ref resultNodes);
            }
        }
    }

    // Get octree depth at a position
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static byte GetNodeDepthAtPosition(float3 position, in NavigationOctree octree,
        in DynamicBuffer<NavigationOctreeNode> nodes)
    {
        int nodeIndex = GetNodeContaining(position, octree, nodes);
        if (nodeIndex >= 0 && nodeIndex < nodes.Length)
        {
            return nodes[nodeIndex].Depth;
        }
        return 0;
    }

    // Sample scalar field with trilinear interpolation
    public static float InterpolateScalarField(float3 position, in NavigationOctree octree,
        in DynamicBuffer<NavigationOctreeNode> nodes, in DynamicBuffer<NavigationFieldData> fieldData)
    {
        int nodeIndex = GetNodeContaining(position, octree, nodes);
        if (nodeIndex < 0)
            return float.MaxValue;

        NavigationOctreeNode node = nodes[nodeIndex];
        if (node.FieldDataIndex < 0 || node.FieldDataIndex >= fieldData.Length)
            return float.MaxValue;

        NavigationFieldData field = fieldData[node.FieldDataIndex];

        // Compute interpolation weights
        float3 localPos = (position - (node.Center - node.HalfExtent)) / (2f * node.HalfExtent);
        localPos = math.saturate(localPos);

        // Trilinear interpolation of corner values
        float c00 = math.lerp(field.Corner000Distance, field.Corner100Distance, localPos.x);
        float c01 = math.lerp(field.Corner001Distance, field.Corner101Distance, localPos.x);
        float c10 = math.lerp(field.Corner010Distance, field.Corner110Distance, localPos.x);
        float c11 = math.lerp(field.Corner011Distance, field.Corner111Distance, localPos.x);

        float c0 = math.lerp(c00, c10, localPos.y);
        float c1 = math.lerp(c01, c11, localPos.y);

        return math.lerp(c0, c1, localPos.z);
    }

    // Sample vector field with trilinear interpolation
    public static float3 InterpolateVectorField(float3 position, in NavigationOctree octree,
        in DynamicBuffer<NavigationOctreeNode> nodes, in DynamicBuffer<NavigationFieldData> fieldData)
    {
        int nodeIndex = GetNodeContaining(position, octree, nodes);
        if (nodeIndex < 0)
            return float3.zero;

        NavigationOctreeNode node = nodes[nodeIndex];
        if (node.FieldDataIndex < 0 || node.FieldDataIndex >= fieldData.Length)
            return float3.zero;

        NavigationFieldData field = fieldData[node.FieldDataIndex];

        // For now, return the node's vector field directly
        // Full trilinear interpolation of vectors would require corner vector values
        return field.VectorField;
    }
}

// Parallel batch queries for Job System
[BurstCompile]
public struct BatchQueryPositionsJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> QueryPositions;
    [ReadOnly] public NavigationOctree Octree;
    [ReadOnly] public NativeArray<NavigationOctreeNode> Nodes;
    [ReadOnly] public NativeArray<NavigationFieldData> FieldData;

    [WriteOnly] public NativeArray<float> DistanceResults;
    [WriteOnly] public NativeArray<float3> VectorResults;
    [WriteOnly] public NativeArray<NavigationTier> TierResults;

    public void Execute(int index)
    {
        float3 position = QueryPositions[index];

        // Find containing node
        int nodeIndex = NavigationOctreeQueries.GetNodeContaining(position, Octree, Nodes.AsReadOnlyDynamicBuffer());

        if (nodeIndex >= 0 && nodeIndex < Nodes.Length)
        {
            NavigationOctreeNode node = Nodes[nodeIndex];
            TierResults[index] = node.Tier;

            if (node.FieldDataIndex >= 0 && node.FieldDataIndex < FieldData.Length)
            {
                NavigationFieldData field = FieldData[node.FieldDataIndex];
                DistanceResults[index] = field.SignedDistance;
                VectorResults[index] = field.VectorField;
            }
            else
            {
                DistanceResults[index] = float.MaxValue;
                VectorResults[index] = float3.zero;
            }
        }
        else
        {
            DistanceResults[index] = float.MaxValue;
            VectorResults[index] = float3.zero;
            TierResults[index] = NavigationTier.Far;
        }
    }
}

// Extension methods for DynamicBuffer compatibility
public static class NavigationOctreeExtensions
{
    public static DynamicBuffer<NavigationOctreeNode> AsReadOnlyDynamicBuffer(this NativeArray<NavigationOctreeNode> array)
    {
        // This is a workaround for the job system - in practice, you'd pass the actual DynamicBuffer
        unsafe
        {
            var buffer = new DynamicBuffer<NavigationOctreeNode>();
            // Note: This is pseudocode - actual implementation would need proper buffer management
            return buffer;
        }
    }
}