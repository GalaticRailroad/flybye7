using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[BurstCompile]
public static class OctreeDynamicOperations
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void AddObject(ref OctreeSpatialDatabase octree, DynamicBuffer<OctreeNode> nodesBuffer,
        DynamicBuffer<SpatialObject> objectsBuffer, in SpatialObject spatialObject)
    {
        AddObjectToNode(ref octree, nodesBuffer, objectsBuffer, octree.RootNodeIndex, spatialObject);
        octree.TotalObjects++;
    }
    
    private static void AddObjectToNode(ref OctreeSpatialDatabase octree, DynamicBuffer<OctreeNode> nodesBuffer,
        DynamicBuffer<SpatialObject> objectsBuffer, int nodeIndex, in SpatialObject spatialObject)
    {
        if (nodeIndex >= nodesBuffer.Length)
            return;
            
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
                // Try subdivision
                if (SubdivideNode(ref octree, nodesBuffer, objectsBuffer, nodeIndex))
                {
                    // Subdivision succeeded, now add the object to the appropriate child
                    AddObjectToNode(ref octree, nodesBuffer, objectsBuffer, nodeIndex, spatialObject);
                }
                else
                {
                    // Subdivision failed, add to current node within limits
                    if (node.ObjectCount < node.ObjectCapacity && 
                        node.ObjectStartIndex + node.ObjectCount < objectsBuffer.Length)
                    {
                        objectsBuffer[node.ObjectStartIndex + node.ObjectCount] = spatialObject;
                        node.ObjectCount++;
                        nodesBuffer[nodeIndex] = node;
                    }
                    // If capacity exceeded, skip this object
                }
            }
            else
            {
                // Max depth reached, force add to this node within capacity limits
                if (node.ObjectCount < node.ObjectCapacity && 
                    node.ObjectStartIndex + node.ObjectCount < objectsBuffer.Length)
                {
                    objectsBuffer[node.ObjectStartIndex + node.ObjectCount] = spatialObject;
                    node.ObjectCount++;
                    nodesBuffer[nodeIndex] = node;
                }
            }
        }
        else
        {
            // Branch node - find appropriate child
            int childIndex = GetBestChildForObject(node, spatialObject);
            if (node.ChildStartIndex + childIndex < nodesBuffer.Length)
            {
                AddObjectToNode(ref octree, nodesBuffer, objectsBuffer, node.ChildStartIndex + childIndex, spatialObject);
            }
        }
    }
    
    private static bool SubdivideNode(ref OctreeSpatialDatabase octree, DynamicBuffer<OctreeNode> nodesBuffer,
        DynamicBuffer<SpatialObject> objectsBuffer, int nodeIndex)
    {
        if (nodeIndex >= nodesBuffer.Length)
            return false;
            
        OctreeNode leafNode = nodesBuffer[nodeIndex];
        if (!leafNode.IsLeaf)
            return false; // Already subdivided
            
        // Check if we have space for 8 child nodes
        if (octree.NextFreeNodeIndex + 8 > octree.PreallocatedNodeCapacity)
            return false; // Out of space
            
        // Convert leaf to branch node
        int childStartIndex = octree.NextFreeNodeIndex;
        OctreeNode branchNode = leafNode;
        branchNode.IsLeaf = false;
        branchNode.ChildStartIndex = childStartIndex;
        
        // Create 8 child leaf nodes
        float childHalfExtent = leafNode.HalfExtent * 0.5f;
        int childObjectCapacity = octree.MaxObjectsPerNode;
        
        for (int childIdx = 0; childIdx < 8; childIdx++)
        {
            float3 childCenter = OctreeSpatialDatabase.GetChildCenter(leafNode.Center, leafNode.HalfExtent, childIdx);
            
            // Check object capacity
            if (octree.NextFreeObjectIndex + childObjectCapacity >= octree.PreallocatedObjectCapacity)
                return false; // Out of object space
                
            int newChildNodeIndex = octree.NextFreeNodeIndex;
            int childObjectStartIndex = octree.NextFreeObjectIndex;
            
            // Ensure buffers are large enough
            while (nodesBuffer.Length <= newChildNodeIndex)
            {
                nodesBuffer.Add(default);
            }
            while (objectsBuffer.Length <= childObjectStartIndex + childObjectCapacity - 1)
            {
                objectsBuffer.Add(default);
            }
            
            OctreeNode childNode = OctreeNode.CreateLeaf(childCenter, childHalfExtent, 
                (byte)(leafNode.Depth + 1), childObjectStartIndex, childObjectCapacity);
                
            nodesBuffer[newChildNodeIndex] = childNode;
            octree.NextFreeNodeIndex++;
            octree.NextFreeObjectIndex += childObjectCapacity;
            octree.TotalNodes++;
        }
        
        // Update the parent node
        nodesBuffer[nodeIndex] = branchNode;
        
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
        return OctreeSpatialDatabase.GetChildIndex(branchNode.Center, spatialObject.Position);
    }
    
    // AABB Query Implementation
    public static void QueryAABB<T>(in OctreeSpatialDatabase octree, DynamicBuffer<OctreeNode> nodesBuffer,
        DynamicBuffer<SpatialObject> objectsBuffer, float3 queryMin, float3 queryMax, ref T collector)
        where T : unmanaged, IOctreeQueryCollector
    {
        if (octree.TotalNodes > 0 && octree.RootNodeIndex < nodesBuffer.Length)
        {
            QueryNodeAABB(nodesBuffer, objectsBuffer, octree.RootNodeIndex, queryMin, queryMax, ref collector);
        }
    }
    
    private static void QueryNodeAABB<T>(DynamicBuffer<OctreeNode> nodesBuffer, DynamicBuffer<SpatialObject> objectsBuffer,
        int nodeIndex, float3 queryMin, float3 queryMax, ref T collector)
        where T : unmanaged, IOctreeQueryCollector
    {
        if (nodeIndex >= nodesBuffer.Length)
            return;
            
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
                if (node.ObjectStartIndex + i < objectsBuffer.Length)
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
        }
        else
        {
            // Recursively check children
            for (int i = 0; i < 8; i++)
            {
                int childIndex = node.ChildStartIndex + i;
                if (childIndex < nodesBuffer.Length)
                {
                    QueryNodeAABB(nodesBuffer, objectsBuffer, childIndex, queryMin, queryMax, ref collector);
                }
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