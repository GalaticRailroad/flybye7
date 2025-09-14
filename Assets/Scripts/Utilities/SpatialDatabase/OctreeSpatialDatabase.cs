using System;
using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

[InternalBufferCapacity(0)]
public struct OctreeNode : IBufferElementData
{
    public float3 Center;
    public float HalfExtent;
    public byte Depth;
    public bool IsLeaf;
    
    // For leaf nodes - direct object storage
    public int ObjectStartIndex;
    public int ObjectCount;
    public int ObjectCapacity;
    
    // For branch nodes - child references (8 children stored consecutively)
    public int ChildStartIndex;
    
    public static OctreeNode CreateLeaf(float3 center, float halfExtent, byte depth, int objectStartIndex, int objectCapacity)
    {
        return new OctreeNode
        {
            Center = center,
            HalfExtent = halfExtent,
            Depth = depth,
            IsLeaf = true,
            ObjectStartIndex = objectStartIndex,
            ObjectCount = 0,
            ObjectCapacity = objectCapacity,
            ChildStartIndex = -1
        };
    }
    
    public static OctreeNode CreateBranch(float3 center, float halfExtent, byte depth, int childStartIndex)
    {
        return new OctreeNode
        {
            Center = center,
            HalfExtent = halfExtent,
            Depth = depth,
            IsLeaf = false,
            ObjectStartIndex = -1,
            ObjectCount = 0,
            ObjectCapacity = 0,
            ChildStartIndex = childStartIndex
        };
    }
}

[InternalBufferCapacity(0)]
public struct SpatialObject : IBufferElementData
{
    public Entity Entity;
    public float3 Position;
    public float3 HalfExtents;   // For AABB support
    public byte Team;
    public byte Type;
    public CollisionShapeType ShapeType;
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float3 GetMinBounds()
    {
        return ShapeType switch
        {
            CollisionShapeType.Point => Position,
            CollisionShapeType.AABB => Position - HalfExtents,
            CollisionShapeType.Sphere => Position - new float3(HalfExtents.x), // Use x as radius
            _ => Position
        };
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public float3 GetMaxBounds()
    {
        return ShapeType switch
        {
            CollisionShapeType.Point => Position,
            CollisionShapeType.AABB => Position + HalfExtents,
            CollisionShapeType.Sphere => Position + new float3(HalfExtents.x), // Use x as radius
            _ => Position
        };
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IntersectsAABB(float3 aabbMin, float3 aabbMax)
    {
        float3 objMin = GetMinBounds();
        float3 objMax = GetMaxBounds();
        
        return (objMin.x <= aabbMax.x && objMax.x >= aabbMin.x) &&
               (objMin.y <= aabbMax.y && objMax.y >= aabbMin.y) &&
               (objMin.z <= aabbMax.z && objMax.z >= aabbMin.z);
    }
}

public struct OctreeSpatialDatabase : IComponentData
{
    public float3 Center;
    public float HalfExtent;
    public byte MaxDepth;
    public byte MaxObjectsPerNode;
    public int RootNodeIndex;
    public int TotalNodes;
    public int TotalObjects;
    
    // Dynamic subdivision support
    public int NextFreeNodeIndex;    // Next available slot in nodes buffer
    public int NextFreeObjectIndex;  // Next available slot in objects buffer
    public int PreallocatedNodeCapacity;    // Total preallocated nodes
    public int PreallocatedObjectCapacity;  // Total preallocated objects
    
    public const float ObjectsCapacityGrowFactor = 1.5f;
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float3 GetChildCenter(float3 parentCenter, float parentHalfExtent, int childIndex)
    {
        float quarter = parentHalfExtent * 0.5f;
        return parentCenter + new float3(
            (childIndex & 1) != 0 ? quarter : -quarter,          // x
            (childIndex & 2) != 0 ? quarter : -quarter,          // y
            (childIndex & 4) != 0 ? quarter : -quarter           // z
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
    
    public static void Initialize(float3 center, float halfExtent, byte maxDepth, byte maxObjectsPerNode,
        ref OctreeSpatialDatabase octree, DynamicBuffer<OctreeNode> nodesBuffer,
        DynamicBuffer<SpatialObject> objectsBuffer)
    {
        // Calculate required buffer capacity for dynamic subdivision
        // Use reasonable buffer size instead of worst-case to prevent memory issues
        int maxPossibleNodes = math.min(CalculateMaxNodes(maxDepth), 8192); // Cap at 8K nodes
        int maxPossibleObjects = maxPossibleNodes * maxObjectsPerNode; // Remove 2x multiplier
        
        // Pre-allocate buffers for dynamic operations
        nodesBuffer.EnsureCapacity(maxPossibleNodes);
        objectsBuffer.EnsureCapacity(maxPossibleObjects);
        
        // Initialize octree parameters
        octree.Center = center;
        octree.HalfExtent = halfExtent;
        octree.MaxDepth = maxDepth;
        octree.MaxObjectsPerNode = maxObjectsPerNode;
        octree.TotalNodes = 0;
        octree.TotalObjects = 0;
        
        // Initialize buffer management
        octree.NextFreeNodeIndex = 0;
        octree.NextFreeObjectIndex = 0;
        octree.PreallocatedNodeCapacity = maxPossibleNodes;
        octree.PreallocatedObjectCapacity = maxPossibleObjects;
        
        // Create root node
        int initialObjectCapacity = maxObjectsPerNode * 2;
        octree.RootNodeIndex = CreateLeafNode(ref octree, nodesBuffer, objectsBuffer,
            center, halfExtent, 0, initialObjectCapacity);
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int CalculateMaxNodes(byte maxDepth)
    {
        // Cap depth to prevent memory issues
        maxDepth = (byte)math.min(maxDepth, 6); // Max depth 6 = 37K nodes max
        
        // Formula: sum of 8^i from i=0 to maxDepth
        int totalNodes = 0;
        int nodesAtDepth = 1;
        for (int depth = 0; depth <= maxDepth; depth++)
        {
            totalNodes += nodesAtDepth;
            nodesAtDepth *= 8;
            
            // Safety check to prevent overflow
            if (totalNodes > 100000) // 100K node limit
            {
                break;
            }
        }
        return totalNodes;
    }
    
    public static int CreateLeafNode(ref OctreeSpatialDatabase octree, DynamicBuffer<OctreeNode> nodesBuffer,
        DynamicBuffer<SpatialObject> objectsBuffer, float3 center, float halfExtent, byte depth, int objectCapacity)
    {
        // Check if we have space in pre-allocated buffers
        if (octree.NextFreeNodeIndex >= octree.PreallocatedNodeCapacity ||
            octree.NextFreeObjectIndex + objectCapacity >= octree.PreallocatedObjectCapacity)
        {
            return -1; // Out of pre-allocated space
        }
        
        int nodeIndex = octree.NextFreeNodeIndex;
        int objectStartIndex = octree.NextFreeObjectIndex;
        
        // Ensure buffers are large enough (they should be from pre-allocation)
        while (nodesBuffer.Length <= nodeIndex)
        {
            nodesBuffer.Add(default);
        }
        
        while (objectsBuffer.Length <= objectStartIndex + objectCapacity - 1)
        {
            objectsBuffer.Add(default);
        }
        
        // Create the node
        nodesBuffer[nodeIndex] = OctreeNode.CreateLeaf(center, halfExtent, depth, objectStartIndex, objectCapacity);
        
        // Update allocation tracking
        octree.NextFreeNodeIndex++;
        octree.NextFreeObjectIndex += objectCapacity;
        octree.TotalNodes++;
        
        return nodeIndex;
    }
    
    public static int CreateBranchNode(ref OctreeSpatialDatabase octree, DynamicBuffer<OctreeNode> nodesBuffer,
        float3 center, float halfExtent, byte depth)
    {
        // Check if we have space for 1 branch + 8 child nodes
        if (octree.NextFreeNodeIndex + 9 > octree.PreallocatedNodeCapacity)
        {
            return -1; // Out of pre-allocated space
        }
        
        int nodeIndex = octree.NextFreeNodeIndex;
        int childStartIndex = octree.NextFreeNodeIndex + 1; // Children will be added right after this node
        
        // Ensure buffer is large enough
        while (nodesBuffer.Length <= nodeIndex + 8)
        {
            nodesBuffer.Add(default);
        }
        
        // Create the branch node
        nodesBuffer[nodeIndex] = OctreeNode.CreateBranch(center, halfExtent, depth, childStartIndex);
        
        // Initialize 8 child slots (they'll be populated during subdivision)
        for (int i = 1; i <= 8; i++)
        {
            nodesBuffer[nodeIndex + i] = default;
        }
        
        // Update allocation tracking
        octree.NextFreeNodeIndex += 9; // Parent + 8 children
        octree.TotalNodes += 9;
        
        return nodeIndex;
    }
    
    public static void ClearAndResize(ref OctreeSpatialDatabase octree, DynamicBuffer<OctreeNode> nodesBuffer,
        DynamicBuffer<SpatialObject> objectsBuffer)
    {
        // Reset to just the root node and clear all objects
        octree.TotalObjects = 0;
        octree.TotalNodes = 1; // Just root node
        octree.NextFreeNodeIndex = 1; // Root is at index 0
        octree.NextFreeObjectIndex = octree.MaxObjectsPerNode * 2; // Root's objects already allocated
        
        // Keep only the root node, clear others
        if (nodesBuffer.Length > 0)
        {
            OctreeNode rootNode = nodesBuffer[octree.RootNodeIndex];
            rootNode.ObjectCount = 0; // Clear objects in root
            rootNode.IsLeaf = true;   // Make sure root is a leaf after clearing
            nodesBuffer[octree.RootNodeIndex] = rootNode;
            
            // Remove all nodes except root
            nodesBuffer.Resize(1, NativeArrayOptions.ClearMemory);
        }
        
        // Keep object buffer but clear content
        for (int i = 0; i < objectsBuffer.Length; i++)
        {
            objectsBuffer[i] = default;
        }
    }
}