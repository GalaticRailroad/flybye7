using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

// Enhanced SDF node data with detailed distance information
public struct SDFNodeData
{
    public float Distance;           // Signed distance at node center
    public float3 Gradient;          // Direction to nearest surface (normalized)
    public float MaxError;           // Maximum error within this node's volume
    public int NearestTriangleID;   // For debugging - which triangle is closest
    public bool IsInside;            // Quick check for inside/outside
    public float3 NearestPoint;     // Nearest point on surface
}

// Mesh data for SDF generation
public struct SDFMeshData
{
    public BlobArray<float3> Vertices;
    public BlobArray<int3> Triangles;
    public float3 BoundsMin;
    public float3 BoundsMax;
    public float4x4 LocalToWorld;
    public int MeshID;
}

// Collider types for distance calculation
public enum ColliderType : byte
{
    None = 0,
    Mesh = 1,
    Box = 2,
    Sphere = 3,
    Capsule = 4,
    Terrain = 5
}

// Collider information for SDF generation
public struct SDFColliderInfo
{
    public ColliderType Type;
    public float3 Position;
    public quaternion Rotation;
    public float3 Scale;

    // Box collider
    public float3 BoxHalfExtents;

    // Sphere collider
    public float SphereRadius;

    // Capsule collider
    public float CapsuleRadius;
    public float CapsuleHeight;
    public int CapsuleDirection; // 0=X, 1=Y, 2=Z

    // Mesh collider
    public int MeshDataIndex;
}

// Spatial acceleration structure for triangles
public struct TriangleBVHNode
{
    public float3 BoundsMin;
    public float3 BoundsMax;
    public int TriangleIndex;    // -1 for internal nodes
    public int LeftChild;         // Index of left child
    public int RightChild;        // Index of right child
}

// Main SDF generation system
[BurstCompile]
public static class NavigationSDFGenerator
{
    // Generate SDF for entire octree
    public static JobHandle GenerateSDF(
        ref NavigationOctree octree,
        DynamicBuffer<NavigationOctreeNode> nodes,
        DynamicBuffer<NavigationFieldData> fieldData,
        NativeArray<SDFColliderInfo> colliders,
        NativeArray<SDFMeshData> meshes,
        JobHandle dependency = default)
    {
        // Allocate temporary SDF data buffer
        int nodeCount = nodes.Length;
        NativeArray<SDFNodeData> sdfData = new NativeArray<SDFNodeData>(nodeCount, Allocator.TempJob);

        // Generate distance field for all nodes
        var generateJob = new GenerateSDFJob
        {
            Octree = octree,
            Nodes = nodes.AsNativeArray(),
            Colliders = colliders,
            Meshes = meshes,
            SDFData = sdfData
        };

        JobHandle generateHandle = generateJob.Schedule(nodeCount, 32, dependency);

        // Compute gradients using finite differences
        var gradientJob = new ComputeGradientsJob
        {
            Octree = octree,
            Nodes = nodes.AsNativeArray(),
            SDFData = sdfData
        };

        JobHandle gradientHandle = gradientJob.Schedule(nodeCount, 32, generateHandle);

        // Copy SDF data to field data buffer
        var copyJob = new CopySDFToFieldDataJob
        {
            SDFData = sdfData,
            FieldData = fieldData.AsNativeArray()
        };

        JobHandle copyHandle = copyJob.Schedule(nodeCount, 64, gradientHandle);

        // Dispose temporary buffer
        sdfData.Dispose(copyHandle);

        return copyHandle;
    }

    // Calculate exact signed distance to a point
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float CalculateSignedDistance(
        float3 point,
        NativeArray<SDFColliderInfo> colliders,
        NativeArray<SDFMeshData> meshes)
    {
        float minDistance = float.MaxValue;
        bool isInside = false;

        for (int i = 0; i < colliders.Length; i++)
        {
            SDFColliderInfo collider = colliders[i];
            float distance = float.MaxValue;
            bool inside = false;

            switch (collider.Type)
            {
                case ColliderType.Box:
                    distance = DistanceToBox(point, collider, out inside);
                    break;
                case ColliderType.Sphere:
                    distance = DistanceToSphere(point, collider, out inside);
                    break;
                case ColliderType.Capsule:
                    distance = DistanceToCapsule(point, collider, out inside);
                    break;
                case ColliderType.Mesh:
                    if (collider.MeshDataIndex >= 0 && collider.MeshDataIndex < meshes.Length)
                    {
                        distance = DistanceToMeshFromArray(point, meshes, collider.MeshDataIndex, out inside);
                    }
                    break;
            }

            if (distance < math.abs(minDistance))
            {
                minDistance = distance;
                isInside = inside;
            }
        }

        return isInside ? -minDistance : minDistance;
    }

    // Distance to box collider
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float DistanceToBox(in float3 point, in SDFColliderInfo box, out bool isInside)
    {
        // Transform point to box local space
        float3 localPoint = math.mul(math.inverse(box.Rotation), point - box.Position) / box.Scale;

        // Calculate distance in local space
        float3 d = math.abs(localPoint) - box.BoxHalfExtents;
        float outsideDistance = math.length(math.max(d, 0f));
        float insideDistance = math.min(math.cmax(d), 0f);

        isInside = insideDistance < 0;
        return outsideDistance + insideDistance;
    }

    // Distance to sphere collider
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float DistanceToSphere(in float3 point, in SDFColliderInfo sphere, out bool isInside)
    {
        float distance = math.length(point - sphere.Position) - sphere.SphereRadius;
        isInside = distance < 0;
        return math.abs(distance);
    }

    // Distance to capsule collider
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float DistanceToCapsule(in float3 point, in SDFColliderInfo capsule, out bool isInside)
    {
        // Transform point to capsule local space
        float3 localPoint = math.mul(math.inverse(capsule.Rotation), point - capsule.Position);

        // Get capsule axis
        float3 axis = float3.zero;
        axis[capsule.CapsuleDirection] = 1f;

        // Project point onto capsule axis
        float halfHeight = capsule.CapsuleHeight * 0.5f - capsule.CapsuleRadius;
        float3 p1 = axis * -halfHeight;
        float3 p2 = axis * halfHeight;

        float t = math.saturate(math.dot(localPoint - p1, p2 - p1) / math.lengthsq(p2 - p1));
        float3 closestOnLine = math.lerp(p1, p2, t);

        float distance = math.length(localPoint - closestOnLine) - capsule.CapsuleRadius;
        isInside = distance < 0;
        return math.abs(distance);
    }

    // Distance to mesh handling blob array access internally
    public static unsafe float DistanceToMeshFromArray(in float3 point, NativeArray<SDFMeshData> meshes, int meshIndex, out bool isInside)
    {
        // Use unsafe pointer arithmetic to get a proper ref without CS0206/CS1612 issues
        SDFMeshData* meshPtr = (SDFMeshData*)meshes.GetUnsafeReadOnlyPtr() + meshIndex;
        ref SDFMeshData mesh = ref *meshPtr;

        // Quick bounds check
        float3 localPoint = math.mul(math.inverse(mesh.LocalToWorld), new float4(point, 1f)).xyz;

        if (localPoint.x < mesh.BoundsMin.x || localPoint.x > mesh.BoundsMax.x ||
            localPoint.y < mesh.BoundsMin.y || localPoint.y > mesh.BoundsMax.y ||
            localPoint.z < mesh.BoundsMin.z || localPoint.z > mesh.BoundsMax.z)
        {
            isInside = false;
            return math.distance(localPoint, math.clamp(localPoint, mesh.BoundsMin, mesh.BoundsMax));
        }

        // Simple mesh distance calculation (simplified)
        float minDistance = float.MaxValue;

        // Access blob arrays through the mesh ref (this should work)
        ref var triangles = ref mesh.Triangles;
        ref var vertices = ref mesh.Vertices;

        // Iterate through triangles and find closest
        for (int i = 0; i < triangles.Length; i++)
        {
            int3 triangle = triangles[i];
            float3 v0 = vertices[triangle.x];
            float3 v1 = vertices[triangle.y];
            float3 v2 = vertices[triangle.z];

            float distance = DistancePointToTriangle(localPoint, v0, v1, v2);
            if (distance < minDistance)
            {
                minDistance = distance;
            }
        }

        // Determine inside/outside using ray casting inline to avoid CS0206
        int intersectionCount = 0;
        float3 rayDirection = new float3(1, 0, 0);
        float3 rayOrigin = localPoint;

        // Check intersections with all triangles
        for (int i = 0; i < triangles.Length; i++)
        {
            int3 triangle = triangles[i];
            float3 v0 = vertices[triangle.x];
            float3 v1 = vertices[triangle.y];
            float3 v2 = vertices[triangle.z];

            if (RayIntersectTriangle(rayOrigin, rayDirection, v0, v1, v2, out float t))
            {
                if (t > 0.001f) // Small epsilon to avoid self-intersection
                {
                    intersectionCount++;
                }
            }
        }

        // Odd number of intersections means inside
        isInside = (intersectionCount % 2) == 1;

        return minDistance;
    }

    // Distance to mesh (simplified - needs full implementation)
    public static float DistanceToMesh(in float3 point, ref SDFMeshData mesh, out bool isInside)
    {
        // Quick bounds check
        float3 localPoint = math.mul(math.inverse(mesh.LocalToWorld), new float4(point, 1f)).xyz;

        if (math.any(localPoint < mesh.BoundsMin - 0.1f) || math.any(localPoint > mesh.BoundsMax + 0.1f))
        {
            isInside = false;
            return DistanceToBounds(localPoint, mesh.BoundsMin, mesh.BoundsMax);
        }

        // Find closest triangle
        float minDistance = float.MaxValue;
        int closestTriangle = -1;

        for (int i = 0; i < mesh.Triangles.Length; i++)
        {
            int3 tri = mesh.Triangles[i];
            float3 v0 = mesh.Vertices[tri.x];
            float3 v1 = mesh.Vertices[tri.y];
            float3 v2 = mesh.Vertices[tri.z];

            // Transform vertices to world space
            v0 = math.mul(mesh.LocalToWorld, new float4(v0, 1f)).xyz;
            v1 = math.mul(mesh.LocalToWorld, new float4(v1, 1f)).xyz;
            v2 = math.mul(mesh.LocalToWorld, new float4(v2, 1f)).xyz;

            float distance = PointToTriangleDistance(point, v0, v1, v2);
            if (distance < minDistance)
            {
                minDistance = distance;
                closestTriangle = i;
            }
        }

        // Determine inside/outside using ray casting
        isInside = IsPointInsideMeshInternal(localPoint, ref mesh);

        return minDistance;
    }

    // Accurate point-to-triangle distance
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float PointToTriangleDistance(in float3 point, in float3 v0, in float3 v1, in float3 v2)
    {
        float3 edge0 = v1 - v0;
        float3 edge1 = v2 - v0;
        float3 v0ToPoint = point - v0;

        float a = math.dot(edge0, edge0);
        float b = math.dot(edge0, edge1);
        float c = math.dot(edge1, edge1);
        float d = math.dot(edge0, v0ToPoint);
        float e = math.dot(edge1, v0ToPoint);

        float det = a * c - b * b;
        float s = b * e - c * d;
        float t = b * d - a * e;

        if (s + t <= det)
        {
            if (s < 0f)
            {
                if (t < 0f)
                {
                    // Region 4
                    if (d < 0f)
                    {
                        t = 0f;
                        s = math.saturate(-d / a);
                    }
                    else
                    {
                        s = 0f;
                        t = math.saturate(-e / c);
                    }
                }
                else
                {
                    // Region 3
                    s = 0f;
                    t = math.saturate(-e / c);
                }
            }
            else if (t < 0f)
            {
                // Region 5
                t = 0f;
                s = math.saturate(-d / a);
            }
            else
            {
                // Region 0
                float invDet = 1f / det;
                s *= invDet;
                t *= invDet;
            }
        }
        else
        {
            if (s < 0f)
            {
                // Region 2
                float tmp0 = b + d;
                float tmp1 = c + e;
                if (tmp1 > tmp0)
                {
                    float numer = tmp1 - tmp0;
                    float denom = a - 2f * b + c;
                    s = math.saturate(numer / denom);
                    t = 1f - s;
                }
                else
                {
                    s = 0f;
                    t = math.saturate(-e / c);
                }
            }
            else if (t < 0f)
            {
                // Region 6
                float tmp0 = b + e;
                float tmp1 = a + d;
                if (tmp1 > tmp0)
                {
                    float numer = tmp1 - tmp0;
                    float denom = a - 2f * b + c;
                    t = math.saturate(numer / denom);
                    s = 1f - t;
                }
                else
                {
                    t = 0f;
                    s = math.saturate(-d / a);
                }
            }
            else
            {
                // Region 1
                float numer = (c + e) - (b + d);
                float denom = a - 2f * b + c;
                s = math.saturate(numer / denom);
                t = 1f - s;
            }
        }

        float3 closestPoint = v0 + s * edge0 + t * edge1;
        return math.length(point - closestPoint);
    }

    // Internal method that works directly with blob data
    public static bool IsPointInsideMeshInternal(in float3 point, ref SDFMeshData mesh)
    {
        // Simple winding number algorithm
        // Cast ray from point in +X direction and count intersections
        int intersectionCount = 0;
        float3 rayDirection = new float3(1, 0, 0);
        float3 rayOrigin = point;

        // Check intersections with all triangles
        for (int i = 0; i < mesh.Triangles.Length; i++)
        {
            int3 triangle = mesh.Triangles[i];
            float3 v0 = mesh.Vertices[triangle.x];
            float3 v1 = mesh.Vertices[triangle.y];
            float3 v2 = mesh.Vertices[triangle.z];

            if (RayIntersectTriangle(rayOrigin, rayDirection, v0, v1, v2, out float t))
            {
                if (t > 0.001f) // Small epsilon to avoid self-intersection
                {
                    intersectionCount++;
                }
            }
        }

        // Odd number of intersections means inside
        return (intersectionCount % 2) == 1;
    }

    // Distance from point to triangle
    private static float DistancePointToTriangle(in float3 point, in float3 v0, in float3 v1, in float3 v2)
    {
        float3 edge0 = v1 - v0;
        float3 edge1 = v2 - v0;
        float3 v0_to_point = point - v0;

        float a = math.dot(edge0, edge0);
        float b = math.dot(edge0, edge1);
        float c = math.dot(edge1, edge1);
        float d = math.dot(edge0, v0_to_point);
        float e = math.dot(edge1, v0_to_point);

        float det = a * c - b * b;
        float s = b * e - c * d;
        float t = b * d - a * e;

        if (s + t < det)
        {
            if (s < 0f)
            {
                if (t < 0f)
                {
                    if (d < 0f)
                    {
                        s = math.clamp(-d / a, 0f, 1f);
                        t = 0f;
                    }
                    else
                    {
                        s = 0f;
                        t = math.clamp(-e / c, 0f, 1f);
                    }
                }
                else
                {
                    s = 0f;
                    t = math.clamp(-e / c, 0f, 1f);
                }
            }
            else if (t < 0f)
            {
                s = math.clamp(-d / a, 0f, 1f);
                t = 0f;
            }
            else
            {
                float invDet = 1f / det;
                s *= invDet;
                t *= invDet;
            }
        }
        else
        {
            if (s < 0f)
            {
                float tmp0 = b + d;
                float tmp1 = c + e;
                if (tmp1 > tmp0)
                {
                    float numer = tmp1 - tmp0;
                    float denom = a - 2f * b + c;
                    s = math.clamp(numer / denom, 0f, 1f);
                    t = 1f - s;
                }
                else
                {
                    t = math.clamp(-e / c, 0f, 1f);
                    s = 0f;
                }
            }
            else if (t < 0f)
            {
                if (a + d > b + e)
                {
                    float numer = c + e - b - d;
                    float denom = a - 2f * b + c;
                    s = math.clamp(numer / denom, 0f, 1f);
                    t = 1f - s;
                }
                else
                {
                    s = math.clamp(-d / a, 0f, 1f);
                    t = 0f;
                }
            }
            else
            {
                float numer = c + e - b - d;
                float denom = a - 2f * b + c;
                s = math.clamp(numer / denom, 0f, 1f);
                t = 1f - s;
            }
        }

        float3 closestPoint = v0 + s * edge0 + t * edge1;
        return math.length(point - closestPoint);
    }

    // Ray-triangle intersection
    private static bool RayIntersectTriangle(in float3 rayOrigin, in float3 rayDirection, in float3 v0, in float3 v1, in float3 v2, out float t)
    {
        t = 0;
        float3 edge1 = v1 - v0;
        float3 edge2 = v2 - v0;
        float3 h = math.cross(rayDirection, edge2);
        float a = math.dot(edge1, h);

        if (math.abs(a) < 0.00001f)
            return false;

        float f = 1f / a;
        float3 s = rayOrigin - v0;
        float u = f * math.dot(s, h);

        if (u < 0f || u > 1f)
            return false;

        float3 q = math.cross(s, edge1);
        float v = f * math.dot(rayDirection, q);

        if (v < 0f || u + v > 1f)
            return false;

        t = f * math.dot(edge2, q);
        return t > 0.00001f;
    }

    // Check if point is inside mesh using ray casting
    public static bool IsPointInsideMesh(in float3 point, ref SDFMeshData mesh)
    {
        // Simple winding number algorithm
        // Cast ray from point in +X direction and count intersections
        int intersectionCount = 0;
        float3 rayDir = new float3(1f, 0f, 0f);

        for (int i = 0; i < mesh.Triangles.Length; i++)
        {
            int3 tri = mesh.Triangles[i];
            float3 v0 = mesh.Vertices[tri.x];
            float3 v1 = mesh.Vertices[tri.y];
            float3 v2 = mesh.Vertices[tri.z];

            if (RayTriangleIntersection(point, rayDir, v0, v1, v2, out float t) && t > 0f)
            {
                intersectionCount++;
            }
        }

        // Odd number of intersections means inside
        return (intersectionCount & 1) != 0;
    }

    // Ray-triangle intersection test
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool RayTriangleIntersection(
        float3 rayOrigin, float3 rayDir,
        float3 v0, float3 v1, float3 v2,
        out float t)
    {
        const float EPSILON = 1e-6f;
        t = 0f;

        float3 edge1 = v1 - v0;
        float3 edge2 = v2 - v0;
        float3 h = math.cross(rayDir, edge2);
        float a = math.dot(edge1, h);

        if (math.abs(a) < EPSILON)
            return false;

        float f = 1f / a;
        float3 s = rayOrigin - v0;
        float u = f * math.dot(s, h);

        if (u < 0f || u > 1f)
            return false;

        float3 q = math.cross(s, edge1);
        float v = f * math.dot(rayDir, q);

        if (v < 0f || u + v > 1f)
            return false;

        t = f * math.dot(edge2, q);
        return t > EPSILON;
    }

    // Distance to axis-aligned bounding box
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float DistanceToBounds(in float3 point, in float3 boundsMin, in float3 boundsMax)
    {
        float3 center = (boundsMin + boundsMax) * 0.5f;
        float3 extents = (boundsMax - boundsMin) * 0.5f;
        float3 offset = math.abs(point - center);
        float3 distance = offset - extents;

        return math.length(math.max(distance, 0f)) +
               math.min(math.max(distance.x, math.max(distance.y, distance.z)), 0f);
    }
}

// Job to generate SDF for all nodes
[BurstCompile]
public struct GenerateSDFJob : IJobParallelFor
{
    [ReadOnly] public NavigationOctree Octree;
    [ReadOnly] public NativeArray<NavigationOctreeNode> Nodes;
    [ReadOnly] public NativeArray<SDFColliderInfo> Colliders;
    [ReadOnly] public NativeArray<SDFMeshData> Meshes;

    [NativeDisableParallelForRestriction]
    public NativeArray<SDFNodeData> SDFData;

    public void Execute(int nodeIndex)
    {
        if (nodeIndex >= Nodes.Length)
            return;

        NavigationOctreeNode node = Nodes[nodeIndex];
        float3 nodeCenter = node.Center;

        // Calculate signed distance at node center
        float distance = NavigationSDFGenerator.CalculateSignedDistance(nodeCenter, Colliders, Meshes);

        // Calculate distances at corners for interpolation
        float halfExtent = node.HalfExtent;

        // Use individual variables instead of managed array for Burst compatibility
        float corner0, corner1, corner2, corner3, corner4, corner5, corner6, corner7;

        // Corner 0: (-x, -y, -z)
        float3 cornerPos = nodeCenter + new float3(-halfExtent, -halfExtent, -halfExtent);
        corner0 = NavigationSDFGenerator.CalculateSignedDistance(cornerPos, Colliders, Meshes);

        // Corner 1: (+x, -y, -z)
        cornerPos = nodeCenter + new float3(halfExtent, -halfExtent, -halfExtent);
        corner1 = NavigationSDFGenerator.CalculateSignedDistance(cornerPos, Colliders, Meshes);

        // Corner 2: (-x, +y, -z)
        cornerPos = nodeCenter + new float3(-halfExtent, halfExtent, -halfExtent);
        corner2 = NavigationSDFGenerator.CalculateSignedDistance(cornerPos, Colliders, Meshes);

        // Corner 3: (+x, +y, -z)
        cornerPos = nodeCenter + new float3(halfExtent, halfExtent, -halfExtent);
        corner3 = NavigationSDFGenerator.CalculateSignedDistance(cornerPos, Colliders, Meshes);

        // Corner 4: (-x, -y, +z)
        cornerPos = nodeCenter + new float3(-halfExtent, -halfExtent, halfExtent);
        corner4 = NavigationSDFGenerator.CalculateSignedDistance(cornerPos, Colliders, Meshes);

        // Corner 5: (+x, -y, +z)
        cornerPos = nodeCenter + new float3(halfExtent, -halfExtent, halfExtent);
        corner5 = NavigationSDFGenerator.CalculateSignedDistance(cornerPos, Colliders, Meshes);

        // Corner 6: (-x, +y, +z)
        cornerPos = nodeCenter + new float3(-halfExtent, halfExtent, halfExtent);
        corner6 = NavigationSDFGenerator.CalculateSignedDistance(cornerPos, Colliders, Meshes);

        // Corner 7: (+x, +y, +z)
        cornerPos = nodeCenter + new float3(halfExtent, halfExtent, halfExtent);
        corner7 = NavigationSDFGenerator.CalculateSignedDistance(cornerPos, Colliders, Meshes);

        // Calculate max error as variance in corner distances
        float minCorner = corner0;
        float maxCorner = corner0;

        minCorner = math.min(minCorner, corner1);
        maxCorner = math.max(maxCorner, corner1);
        minCorner = math.min(minCorner, corner2);
        maxCorner = math.max(maxCorner, corner2);
        minCorner = math.min(minCorner, corner3);
        maxCorner = math.max(maxCorner, corner3);
        minCorner = math.min(minCorner, corner4);
        maxCorner = math.max(maxCorner, corner4);
        minCorner = math.min(minCorner, corner5);
        maxCorner = math.max(maxCorner, corner5);
        minCorner = math.min(minCorner, corner6);
        maxCorner = math.max(maxCorner, corner6);
        minCorner = math.min(minCorner, corner7);
        maxCorner = math.max(maxCorner, corner7);
        float maxError = (maxCorner - minCorner) * 0.5f;

        // Store SDF data
        SDFData[nodeIndex] = new SDFNodeData
        {
            Distance = distance,
            Gradient = float3.zero, // Will be computed in gradient job
            MaxError = maxError,
            NearestTriangleID = -1,
            IsInside = distance < 0f,
            NearestPoint = nodeCenter // Placeholder
        };
    }
}

// Job to compute gradients using finite differences
[BurstCompile]
public struct ComputeGradientsJob : IJobParallelFor
{
    [ReadOnly] public NavigationOctree Octree;
    [ReadOnly] public NativeArray<NavigationOctreeNode> Nodes;

    [NativeDisableParallelForRestriction]
    public NativeArray<SDFNodeData> SDFData;

    public void Execute(int nodeIndex)
    {
        if (nodeIndex >= Nodes.Length)
            return;

        NavigationOctreeNode node = Nodes[nodeIndex];
        SDFNodeData sdfNode = SDFData[nodeIndex];

        // Use finite differences to compute gradient
        float epsilon = node.HalfExtent * 0.1f;
        float3 center = node.Center;

        // Sample neighboring points
        float dx_pos = SampleSDFAtPosition(center + new float3(epsilon, 0, 0));
        float dx_neg = SampleSDFAtPosition(center - new float3(epsilon, 0, 0));
        float dy_pos = SampleSDFAtPosition(center + new float3(0, epsilon, 0));
        float dy_neg = SampleSDFAtPosition(center - new float3(0, epsilon, 0));
        float dz_pos = SampleSDFAtPosition(center + new float3(0, 0, epsilon));
        float dz_neg = SampleSDFAtPosition(center - new float3(0, 0, epsilon));

        // Compute gradient
        float3 gradient = new float3(
            (dx_pos - dx_neg) / (2f * epsilon),
            (dy_pos - dy_neg) / (2f * epsilon),
            (dz_pos - dz_neg) / (2f * epsilon)
        );

        // Normalize gradient (except at distance = 0)
        if (math.lengthsq(gradient) > 1e-6f)
        {
            gradient = math.normalize(gradient);
        }

        sdfNode.Gradient = gradient;
        SDFData[nodeIndex] = sdfNode;
    }

    float SampleSDFAtPosition(float3 position)
    {
        // Find the node containing this position
        for (int i = 0; i < Nodes.Length; i++)
        {
            NavigationOctreeNode node = Nodes[i];
            float3 toPos = position - node.Center;

            if (math.all(math.abs(toPos) <= node.HalfExtent))
            {
                if (node.IsLeaf)
                {
                    return SDFData[i].Distance;
                }
            }
        }

        return float.MaxValue;
    }
}

// Job to copy SDF data to field data buffer
[BurstCompile]
public struct CopySDFToFieldDataJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<SDFNodeData> SDFData;
    [NativeDisableParallelForRestriction]
    public NativeArray<NavigationFieldData> FieldData;

    public void Execute(int index)
    {
        if (index >= SDFData.Length || index >= FieldData.Length)
            return;

        SDFNodeData sdf = SDFData[index];
        NavigationFieldData field = FieldData[index];

        field.SignedDistance = sdf.Distance;
        field.GradientField = sdf.Gradient;

        // Note: Corner distance values would need to be computed and stored separately
        // This is a simplified version

        FieldData[index] = field;
    }
}