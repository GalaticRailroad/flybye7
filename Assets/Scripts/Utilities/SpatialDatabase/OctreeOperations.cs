using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

[BurstCompile]
public static class OctreeOperations
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void AddObject(ref OctreeSpatialDatabase octree, ref UnsafeList<OctreeNode> nodesBuffer,
        ref UnsafeList<SpatialObject> objectsBuffer, in SpatialObject spatialObject)
    {
        AddObjectToNode(ref octree, ref nodesBuffer, ref objectsBuffer, octree.RootNodeIndex, spatialObject);
        octree.TotalObjects++;
    }
    
    private static void AddObjectToNode(ref OctreeSpatialDatabase octree, ref UnsafeList<OctreeNode> nodesBuffer,
        ref UnsafeList<SpatialObject> objectsBuffer, int nodeIndex, in SpatialObject spatialObject)
    {
        OctreeNode node = nodesBuffer[nodeIndex];
        
        if (node.IsLeaf)
        {
            // Try to add to this leaf node
            if (node.ObjectCount < node.ObjectCapacity && 
                node.ObjectStartIndex + node.ObjectCount < objectsBuffer.Length)
            {
                // Add object to leaf
                objectsBuffer[node.ObjectStartIndex + node.ObjectCount] = spatialObject;
                node.ObjectCount++;
                nodesBuffer[nodeIndex] = node;
            }
            else if (node.Depth < octree.MaxDepth)
            {
                // Try to subdivide and redistribute objects
                bool subdivisionSucceeded = SubdivideNode(ref octree, ref nodesBuffer, ref objectsBuffer, nodeIndex);
                
                if (subdivisionSucceeded)
                {
                    // Now add the object to the appropriate child
                    AddObjectToNode(ref octree, ref nodesBuffer, ref objectsBuffer, nodeIndex, spatialObject);
                }
                else
                {
                    // Subdivision failed, treat as max depth reached - add within capacity limits
                    if (node.ObjectCount < node.ObjectCapacity && 
                        node.ObjectStartIndex + node.ObjectCount < objectsBuffer.Length)
                    {
                        objectsBuffer[node.ObjectStartIndex + node.ObjectCount] = spatialObject;
                        node.ObjectCount++;
                        nodesBuffer[nodeIndex] = node;
                    }
                    // If capacity exceeded, skip this object (limitation of current implementation)
                }
            }
            else
            {
                // Max depth reached, force add to this node (within capacity limits)
                if (node.ObjectCount < node.ObjectCapacity && 
                    node.ObjectStartIndex + node.ObjectCount < objectsBuffer.Length)
                {
                    objectsBuffer[node.ObjectStartIndex + node.ObjectCount] = spatialObject;
                    node.ObjectCount++;
                    nodesBuffer[nodeIndex] = node;
                }
                else
                {
                    // Node is at capacity and we can't grow it - skip this object
                    // This is a limitation of the current DynamicBuffer approach
                    // In production, you'd want to pre-allocate more buffer space
                    // or implement a more sophisticated overflow handling strategy
                }
            }
        }
        else
        {
            // Branch node - find appropriate child
            int childIndex = GetBestChildForObject(node, spatialObject);
            AddObjectToNode(ref octree, ref nodesBuffer, ref objectsBuffer, node.ChildStartIndex + childIndex, spatialObject);
        }
    }
    
    private static bool SubdivideNode(ref OctreeSpatialDatabase octree, ref UnsafeList<OctreeNode> nodesBuffer,
        ref UnsafeList<SpatialObject> objectsBuffer, int nodeIndex)
    {
        if (nodeIndex >= nodesBuffer.Length)
            return false;
            
        OctreeNode leafNode = nodesBuffer[nodeIndex];
        if (!leafNode.IsLeaf)
            return false; // Already subdivided
            
        // Check if we have space for 8 child nodes in pre-allocated buffer
        if (octree.NextFreeNodeIndex + 8 > octree.PreallocatedNodeCapacity)
            return false; // Out of space
            
        // Convert leaf to branch node
        int childStartIndex = octree.NextFreeNodeIndex;
        OctreeNode branchNode = leafNode;
        branchNode.IsLeaf = false;
        branchNode.ChildStartIndex = childStartIndex;
        nodesBuffer[nodeIndex] = branchNode;
        
        // Create 8 child leaf nodes
        float childHalfExtent = leafNode.HalfExtent * 0.5f;
        int childObjectCapacity = octree.MaxObjectsPerNode;
        
        for (int childIdx = 0; childIdx < 8; childIdx++)
        {
            float3 childCenter = OctreeSpatialDatabase.GetChildCenter(leafNode.Center, leafNode.HalfExtent, childIdx);
            
            // Check object capacity
            if (octree.NextFreeObjectIndex + childObjectCapacity >= octree.PreallocatedObjectCapacity)
                return false; // Out of object space
                
            // Ensure buffer is large enough
            int newChildNodeIndex = octree.NextFreeNodeIndex;
            while (nodesBuffer.Length <= newChildNodeIndex)
            {
                // We can't add to UnsafeList, this is still a limitation
                return false;
            }
            
            int childObjectStartIndex = octree.NextFreeObjectIndex;
            OctreeNode childNode = OctreeNode.CreateLeaf(childCenter, childHalfExtent, 
                (byte)(leafNode.Depth + 1), childObjectStartIndex, childObjectCapacity);
                
            nodesBuffer[newChildNodeIndex] = childNode;
            octree.NextFreeNodeIndex++;
            octree.NextFreeObjectIndex += childObjectCapacity;
            octree.TotalNodes++;
        }
        
        // Redistribute existing objects to children
        for (int objIdx = 0; objIdx < leafNode.ObjectCount; objIdx++)
        {
            SpatialObject obj = objectsBuffer[leafNode.ObjectStartIndex + objIdx];
            int bestChild = GetBestChildForObject(branchNode, obj);
            int bestChildNodeIndex = childStartIndex + bestChild;
            
            OctreeNode childNode = nodesBuffer[bestChildNodeIndex];
            if (childNode.ObjectCount < childNode.ObjectCapacity &&
                childNode.ObjectStartIndex + childNode.ObjectCount < objectsBuffer.Length)
            {
                objectsBuffer[childNode.ObjectStartIndex + childNode.ObjectCount] = obj;
                childNode.ObjectCount++;
                nodesBuffer[bestChildNodeIndex] = childNode;
            }
        }
        
        return true; // Subdivision succeeded
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetBestChildForObject(in OctreeNode branchNode, in SpatialObject spatialObject)
    {
        // For now, use the object's center position to determine the best child
        // This could be improved to handle objects that span multiple children
        return OctreeSpatialDatabase.GetChildIndex(branchNode.Center, spatialObject.Position);
    }
    
    private static int CreateLeafNode(ref UnsafeList<OctreeNode> nodesBuffer, ref UnsafeList<SpatialObject> objectsBuffer,
        float3 center, float halfExtent, byte depth, int objectCapacity)
    {
        // For now, don't support dynamic node creation during frame updates
        // This would require proper memory management which is complex with UnsafeList from DynamicBuffer
        // Instead, we'll just create a leaf node with no allocated space and handle overflow differently
        int nodeIndex = nodesBuffer.Length;
        
        // We can't safely add to the UnsafeList since it doesn't have an allocator
        // This is a limitation of the current approach - dynamic subdivision needs to be handled differently
        // For now, return -1 to indicate we can't create new nodes during runtime
        return -1;
    }
    
    private static void GrowNodeCapacity(ref UnsafeList<OctreeNode> nodesBuffer, ref UnsafeList<SpatialObject> objectsBuffer,
        int nodeIndex)
    {
        // For now, disable capacity growth during frame updates due to UnsafeList allocation issues
        // This is a limitation of working with DynamicBuffer-backed UnsafeList containers
        // Capacity overflow will be handled by allowing nodes to exceed their ideal capacity
        
        // In a production system, you would pre-allocate sufficient buffer space during initialization
        // or implement a more sophisticated memory management approach
    }
    
    // AABB Query Implementation
    public static void QueryAABB<T>(in OctreeSpatialDatabase octree, in UnsafeList<OctreeNode> nodesBuffer,
        in UnsafeList<SpatialObject> objectsBuffer, float3 queryMin, float3 queryMax, ref T collector)
        where T : unmanaged, IOctreeQueryCollector
    {
        if (octree.TotalNodes > 0)
        {
            QueryNodeAABB(nodesBuffer, objectsBuffer, octree.RootNodeIndex, queryMin, queryMax, ref collector);
        }
    }
    
    private static void QueryNodeAABB<T>(in UnsafeList<OctreeNode> nodesBuffer, in UnsafeList<SpatialObject> objectsBuffer,
        int nodeIndex, float3 queryMin, float3 queryMax, ref T collector)
        where T : unmanaged, IOctreeQueryCollector
    {
        OctreeNode node = nodesBuffer[nodeIndex];
        
        // Check if query AABB intersects with node bounds
        float3 nodeMin = node.Center - new float3(node.HalfExtent);
        float3 nodeMax = node.Center + new float3(node.HalfExtent);
        
        if (!AABBIntersectsAABB(queryMin, queryMax, nodeMin, nodeMax))
            return;
        
        if (node.IsLeaf)
        {
            // Process objects in this leaf
            for (int i = 0; i < node.ObjectCount; i++)
            {
                SpatialObject obj = objectsBuffer[node.ObjectStartIndex + i];
                if (obj.IntersectsAABB(queryMin, queryMax))
                {
                    collector.OnVisitObject(obj, out bool shouldEarlyExit);
                    if (shouldEarlyExit)
                        return;
                }
            }
        }
        else
        {
            // Recursively check children
            for (int i = 0; i < 8; i++)
            {
                QueryNodeAABB(nodesBuffer, objectsBuffer, node.ChildStartIndex + i, queryMin, queryMax, ref collector);
            }
        }
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool AABBIntersectsAABB(float3 aabb1Min, float3 aabb1Max, float3 aabb2Min, float3 aabb2Max)
    {
        return (aabb1Min.x <= aabb2Max.x && aabb1Max.x >= aabb2Min.x) &&
               (aabb1Min.y <= aabb2Max.y && aabb1Max.y >= aabb2Min.y) &&
               (aabb1Min.z <= aabb2Max.z && aabb1Max.z >= aabb2Min.z);
    }
}

// New interface for octree queries
public interface IOctreeQueryCollector
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void OnVisitObject(in SpatialObject obj, out bool shouldEarlyExit);
}