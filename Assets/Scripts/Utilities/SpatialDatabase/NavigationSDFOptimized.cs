using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

// Optimized SDF generation with BVH and parallel processing
[BurstCompile]
public static class NavigationSDFOptimized
{
    // BVH for triangle acceleration
    public struct TriangleBVH
    {
        public NativeArray<BVHNode> Nodes;
        public NativeArray<int> TriangleIndices;
        public int RootIndex;
        public int NodeCount;

        public void Dispose()
        {
            if (Nodes.IsCreated) Nodes.Dispose();
            if (TriangleIndices.IsCreated) TriangleIndices.Dispose();
        }
    }

    public struct BVHNode
    {
        public float3 BoundsMin;
        public float3 BoundsMax;
        public int LeftChild;     // -1 if leaf
        public int RightChild;    // -1 if leaf
        public int TriangleStart; // For leaf nodes
        public int TriangleCount; // For leaf nodes
    }

    // Build BVH for mesh triangles
    public static TriangleBVH BuildBVH(ref SDFMeshData meshData, Allocator allocator)
    {
        int triangleCount = meshData.Triangles.Length;

        // Initialize triangle indices
        var triangleIndices = new NativeArray<int>(triangleCount, allocator);
        for (int i = 0; i < triangleCount; i++)
        {
            triangleIndices[i] = i;
        }

        // Allocate nodes (worst case: 2N-1 nodes)
        var nodes = new NativeArray<BVHNode>(triangleCount * 2, allocator);
        int nodeCount = 0;

        // Build BVH recursively
        int rootIndex = BuildBVHRecursive(ref meshData, triangleIndices, 0, triangleCount, nodes, ref nodeCount);

        return new TriangleBVH
        {
            Nodes = nodes,
            TriangleIndices = triangleIndices,
            RootIndex = rootIndex,
            NodeCount = nodeCount
        };
    }

    private static int BuildBVHRecursive(
        ref SDFMeshData meshData,
        NativeArray<int> triangleIndices,
        int start, int end,
        NativeArray<BVHNode> nodes,
        ref int nodeCount)
    {
        int nodeIndex = nodeCount++;
        BVHNode node = new BVHNode();

        // Calculate bounds for this node
        float3 boundsMin = new float3(float.MaxValue);
        float3 boundsMax = new float3(float.MinValue);

        for (int i = start; i < end; i++)
        {
            int triIndex = triangleIndices[i];
            int3 tri = meshData.Triangles[triIndex];

            float3 v0 = meshData.Vertices[tri.x];
            float3 v1 = meshData.Vertices[tri.y];
            float3 v2 = meshData.Vertices[tri.z];

            boundsMin = math.min(boundsMin, math.min(v0, math.min(v1, v2)));
            boundsMax = math.max(boundsMax, math.max(v0, math.max(v1, v2)));
        }

        node.BoundsMin = boundsMin;
        node.BoundsMax = boundsMax;

        int count = end - start;

        // Create leaf if few enough triangles
        if (count <= 4)
        {
            node.LeftChild = -1;
            node.RightChild = -1;
            node.TriangleStart = start;
            node.TriangleCount = count;
        }
        else
        {
            // Find split axis (longest extent)
            float3 extent = boundsMax - boundsMin;
            int axis = extent.x > extent.y ? (extent.x > extent.z ? 0 : 2) : (extent.y > extent.z ? 1 : 2);

            // Sort triangles along axis
            SortTrianglesAlongAxis(ref meshData, triangleIndices, start, end, axis);

            // Split in middle
            int mid = start + count / 2;

            // Recursively build children
            node.LeftChild = BuildBVHRecursive(ref meshData, triangleIndices, start, mid, nodes, ref nodeCount);
            node.RightChild = BuildBVHRecursive(ref meshData, triangleIndices, mid, end, nodes, ref nodeCount);
            node.TriangleStart = -1;
            node.TriangleCount = 0;
        }

        nodes[nodeIndex] = node;
        return nodeIndex;
    }

    private static void SortTrianglesAlongAxis(
        ref SDFMeshData meshData,
        NativeArray<int> triangleIndices,
        int start, int end,
        int axis)
    {
        // Simple bubble sort for now (can be optimized with quicksort)
        for (int i = start; i < end - 1; i++)
        {
            for (int j = start; j < end - 1 - (i - start); j++)
            {
                float3 center1 = GetTriangleCenter(ref meshData, triangleIndices[j]);
                float3 center2 = GetTriangleCenter(ref meshData, triangleIndices[j + 1]);

                if (center1[axis] > center2[axis])
                {
                    int temp = triangleIndices[j];
                    triangleIndices[j] = triangleIndices[j + 1];
                    triangleIndices[j + 1] = temp;
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float3 GetTriangleCenter(ref SDFMeshData meshData, int triangleIndex)
    {
        int3 tri = meshData.Triangles[triangleIndex];
        float3 v0 = meshData.Vertices[tri.x];
        float3 v1 = meshData.Vertices[tri.y];
        float3 v2 = meshData.Vertices[tri.z];
        return (v0 + v1 + v2) / 3f;
    }

    // Optimized distance query using BVH
    [BurstCompile]
    public static float DistanceToMeshBVH(
        in float3 point,
        ref SDFMeshData meshData,
        in TriangleBVH bvh,
        out bool isInside)
    {
        float minDistance = float.MaxValue;
        int closestTriangle = -1;

        // Traverse BVH
        TraverseBVH(point, ref meshData, bvh, bvh.RootIndex, ref minDistance, ref closestTriangle);

        // Determine inside/outside
        float3 localPoint = math.mul(math.inverse(meshData.LocalToWorld), new float4(point, 1f)).xyz;
        isInside = NavigationSDFGenerator.IsPointInsideMeshInternal(localPoint, ref meshData);

        return minDistance;
    }

    private static void TraverseBVH(
        in float3 point,
        ref SDFMeshData meshData,
        in TriangleBVH bvh,
        int nodeIndex,
        ref float minDistance,
        ref int closestTriangle)
    {
        if (nodeIndex < 0 || nodeIndex >= bvh.NodeCount)
            return;

        BVHNode node = bvh.Nodes[nodeIndex];

        // Early exit if node bounds are farther than current min distance
        float boundsDist = DistancePointToAABB(point, node.BoundsMin, node.BoundsMax);
        if (boundsDist > minDistance)
            return;

        if (node.LeftChild == -1) // Leaf node
        {
            // Test triangles in leaf
            for (int i = 0; i < node.TriangleCount; i++)
            {
                int triIndex = bvh.TriangleIndices[node.TriangleStart + i];
                int3 tri = meshData.Triangles[triIndex];

                float3 v0 = math.mul(meshData.LocalToWorld, new float4(meshData.Vertices[tri.x], 1f)).xyz;
                float3 v1 = math.mul(meshData.LocalToWorld, new float4(meshData.Vertices[tri.y], 1f)).xyz;
                float3 v2 = math.mul(meshData.LocalToWorld, new float4(meshData.Vertices[tri.z], 1f)).xyz;

                float distance = NavigationSDFGenerator.PointToTriangleDistance(point, v0, v1, v2);

                if (distance < minDistance)
                {
                    minDistance = distance;
                    closestTriangle = triIndex;
                }
            }
        }
        else
        {
            // Traverse children (visit closest first)
            BVHNode leftNode = bvh.Nodes[node.LeftChild];
            BVHNode rightNode = bvh.Nodes[node.RightChild];

            float leftDist = DistancePointToAABB(point, leftNode.BoundsMin, leftNode.BoundsMax);
            float rightDist = DistancePointToAABB(point, rightNode.BoundsMin, rightNode.BoundsMax);

            if (leftDist < rightDist)
            {
                TraverseBVH(point, ref meshData, bvh, node.LeftChild, ref minDistance, ref closestTriangle);
                TraverseBVH(point, ref meshData, bvh, node.RightChild, ref minDistance, ref closestTriangle);
            }
            else
            {
                TraverseBVH(point, ref meshData, bvh, node.RightChild, ref minDistance, ref closestTriangle);
                TraverseBVH(point, ref meshData, bvh, node.LeftChild, ref minDistance, ref closestTriangle);
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float DistancePointToAABB(in float3 point, in float3 boundsMin, in float3 boundsMax)
    {
        float3 closest = math.clamp(point, boundsMin, boundsMax);
        return math.length(point - closest);
    }

    // Parallel SDF generation with spatial hashing
    [BurstCompile]
    public struct ParallelSDFGenerationJob : IJobParallelFor
    {
        [ReadOnly] public NavigationOctree Octree;
        [ReadOnly] public NativeArray<NavigationOctreeNode> Nodes;
        [ReadOnly] public NativeArray<SDFColliderInfo> Colliders;
        [ReadOnly] public NativeArray<SDFMeshData> Meshes;
        [ReadOnly] public NativeParallelMultiHashMap<int3, int> SpatialHash;
        public float SpatialHashCellSize;

        [NativeDisableParallelForRestriction]
        public NativeArray<SDFNodeData> SDFData;

        public void Execute(int nodeIndex)
        {
            if (nodeIndex >= Nodes.Length)
                return;

            NavigationOctreeNode node = Nodes[nodeIndex];

            // Only process leaf nodes
            if (!node.IsLeaf)
                return;

            float3 nodeCenter = node.Center;

            // Use spatial hash to find nearby colliders
            int3 hashCell = GetHashCell(nodeCenter);
            float searchRadius = node.HalfExtent * 3f; // Search radius based on node size

            float minDistance = float.MaxValue;
            bool isInside = false;

            // Check neighboring cells
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        int3 neighborCell = hashCell + new int3(dx, dy, dz);

                        if (SpatialHash.TryGetFirstValue(neighborCell, out int colliderIndex, out var iterator))
                        {
                            do
                            {
                                SDFColliderInfo collider = Colliders[colliderIndex];
                                float distance = CalculateDistanceToCollider(nodeCenter, collider, Meshes);

                                if (math.abs(distance) < math.abs(minDistance))
                                {
                                    minDistance = distance;
                                    isInside = distance < 0;
                                }
                            } while (SpatialHash.TryGetNextValue(out colliderIndex, ref iterator));
                        }
                    }
                }
            }

            // Calculate distances at corners for better interpolation
            float halfExtent = node.HalfExtent;
            float maxError = 0f;

            for (int i = 0; i < 8; i++)
            {
                float3 cornerOffset = new float3(
                    (i & 1) != 0 ? halfExtent : -halfExtent,
                    (i & 2) != 0 ? halfExtent : -halfExtent,
                    (i & 4) != 0 ? halfExtent : -halfExtent
                );
                float3 cornerPos = nodeCenter + cornerOffset;

                float cornerDistance = CalculateDistanceAtPoint(cornerPos, hashCell, searchRadius);
                maxError = math.max(maxError, math.abs(cornerDistance - minDistance));
            }

            // Store SDF data
            SDFData[nodeIndex] = new SDFNodeData
            {
                Distance = isInside ? -math.abs(minDistance) : math.abs(minDistance),
                Gradient = float3.zero, // Will be computed in gradient job
                MaxError = maxError,
                NearestTriangleID = -1,
                IsInside = isInside,
                NearestPoint = nodeCenter // Placeholder
            };
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private int3 GetHashCell(float3 position)
        {
            return new int3(math.floor(position / SpatialHashCellSize));
        }

        private float CalculateDistanceAtPoint(float3 point, int3 baseHashCell, float searchRadius)
        {
            float minDistance = float.MaxValue;
            int searchCells = (int)math.ceil(searchRadius / SpatialHashCellSize);

            for (int dx = -searchCells; dx <= searchCells; dx++)
            {
                for (int dy = -searchCells; dy <= searchCells; dy++)
                {
                    for (int dz = -searchCells; dz <= searchCells; dz++)
                    {
                        int3 cell = baseHashCell + new int3(dx, dy, dz);

                        if (SpatialHash.TryGetFirstValue(cell, out int colliderIndex, out var iterator))
                        {
                            do
                            {
                                SDFColliderInfo collider = Colliders[colliderIndex];
                                float distance = CalculateDistanceToCollider(point, collider, Meshes);
                                minDistance = math.min(minDistance, math.abs(distance));
                            } while (SpatialHash.TryGetNextValue(out colliderIndex, ref iterator));
                        }
                    }
                }
            }

            return minDistance;
        }

        private float CalculateDistanceToCollider(float3 point, in SDFColliderInfo collider, NativeArray<SDFMeshData> meshes)
        {
            bool inside;
            float distance;

            switch (collider.Type)
            {
                case ColliderType.Box:
                    distance = NavigationSDFGenerator.DistanceToBox(point, collider, out inside);
                    break;
                case ColliderType.Sphere:
                    distance = NavigationSDFGenerator.DistanceToSphere(point, collider, out inside);
                    break;
                case ColliderType.Capsule:
                    distance = NavigationSDFGenerator.DistanceToCapsule(point, collider, out inside);
                    break;
                case ColliderType.Mesh:
                    if (collider.MeshDataIndex >= 0 && collider.MeshDataIndex < meshes.Length)
                    {
                        distance = NavigationSDFGenerator.DistanceToMeshFromArray(point, meshes, collider.MeshDataIndex, out inside);
                    }
                    else
                    {
                        distance = float.MaxValue;
                        inside = false;
                    }
                    break;
                default:
                    distance = float.MaxValue;
                    inside = false;
                    break;
            }

            return inside ? -math.abs(distance) : math.abs(distance);
        }
    }

    // Optimized gradient computation using central differences
    [BurstCompile]
    public struct OptimizedGradientJob : IJobParallelFor
    {
        [ReadOnly] public NavigationOctree Octree;
        [ReadOnly] public NativeArray<NavigationOctreeNode> Nodes;
        [ReadOnly] public NativeArray<SDFNodeData> SDFData;

        [WriteOnly]
        [NativeDisableParallelForRestriction]
        public NativeArray<float3> Gradients;

        public void Execute(int nodeIndex)
        {
            if (nodeIndex >= Nodes.Length)
                return;

            NavigationOctreeNode node = Nodes[nodeIndex];

            if (!node.IsLeaf)
            {
                Gradients[nodeIndex] = float3.zero;
                return;
            }

            float epsilon = node.HalfExtent * 0.1f;
            float3 center = node.Center;

            // Find neighboring nodes for gradient computation
            float dx_pos = SampleSDFNearby(center + new float3(epsilon, 0, 0));
            float dx_neg = SampleSDFNearby(center - new float3(epsilon, 0, 0));
            float dy_pos = SampleSDFNearby(center + new float3(0, epsilon, 0));
            float dy_neg = SampleSDFNearby(center - new float3(0, epsilon, 0));
            float dz_pos = SampleSDFNearby(center + new float3(0, 0, epsilon));
            float dz_neg = SampleSDFNearby(center - new float3(0, 0, epsilon));

            // Central difference gradient
            float3 gradient = new float3(
                (dx_pos - dx_neg) / (2f * epsilon),
                (dy_pos - dy_neg) / (2f * epsilon),
                (dz_pos - dz_neg) / (2f * epsilon)
            );

            // Normalize gradient
            if (math.lengthsq(gradient) > 1e-6f)
            {
                gradient = math.normalize(gradient);
            }

            Gradients[nodeIndex] = gradient;
        }

        private float SampleSDFNearby(float3 position)
        {
            // Find closest node using spatial locality
            float minDistSq = float.MaxValue;
            int closestNode = -1;

            for (int i = 0; i < Nodes.Length; i++)
            {
                if (!Nodes[i].IsLeaf)
                    continue;

                float3 toNode = position - Nodes[i].Center;
                float distSq = math.lengthsq(toNode);

                if (distSq < minDistSq && math.all(math.abs(toNode) <= Nodes[i].HalfExtent))
                {
                    minDistSq = distSq;
                    closestNode = i;
                }
            }

            if (closestNode >= 0)
            {
                return SDFData[closestNode].Distance;
            }

            return float.MaxValue;
        }
    }

    // Distance bound propagation for hierarchical computation
    [BurstCompile]
    public struct DistanceBoundPropagationJob : IJob
    {
        public NavigationOctree Octree;
        [ReadOnly] public NativeArray<NavigationOctreeNode> Nodes;
        public NativeArray<SDFNodeData> SDFData;

        public void Execute()
        {
            // Propagate distance bounds from leaves to root
            for (int depth = Octree.MaxDepth; depth >= 0; depth--)
            {
                for (int i = 0; i < Nodes.Length; i++)
                {
                    NavigationOctreeNode node = Nodes[i];

                    if (node.Depth != depth || node.IsLeaf)
                        continue;

                    // Update bounds based on children
                    float minDist = float.MaxValue;
                    float maxDist = float.MinValue;

                    for (int c = 0; c < 8; c++)
                    {
                        int childIndex = node.ChildStartIndex + c;
                        if (childIndex >= 0 && childIndex < Nodes.Length)
                        {
                            SDFNodeData childSDF = SDFData[childIndex];
                            minDist = math.min(minDist, childSDF.Distance - childSDF.MaxError);
                            maxDist = math.max(maxDist, childSDF.Distance + childSDF.MaxError);
                        }
                    }

                    // Update parent node
                    SDFNodeData parentSDF = SDFData[i];
                    parentSDF.Distance = (minDist + maxDist) * 0.5f;
                    parentSDF.MaxError = (maxDist - minDist) * 0.5f;
                    SDFData[i] = parentSDF;
                }
            }
        }
    }
}