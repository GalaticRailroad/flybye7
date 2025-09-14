using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[BurstCompile]
public static class HierarchicalCollisionQueries
{
    // Multi-level collision query for avoidance/steering
    public static bool QueryForAvoidance(
        float3 queryPosition, 
        float queryRadius,
        CollisionDetailLevel detailLevel,
        in DynamicBuffer<OctreeNode> nodesBuffer,
        in DynamicBuffer<SpatialObject> coarseObjects,
        ComponentLookup<ConvexHullData> convexHullLookup,
        BufferLookup<ConvexHullVertex> convexVertexLookup,
        BufferLookup<ConvexHullFace> convexFaceLookup,
        ComponentLookup<MeshColliderData> meshDataLookup,
        BufferLookup<MeshVertex> meshVertexLookup,
        BufferLookup<MeshTriangle> meshTriangleLookup,
        out AvoidanceResult result)
    {
        result = new AvoidanceResult();
        
        // Level 1: Coarse octree query for broad-phase detection
        NativeList<Entity> candidates = new NativeList<Entity>(16, Allocator.Temp);
        if (!QueryCoarsePhase(queryPosition, queryRadius, nodesBuffer, coarseObjects, ref candidates))
        {
            candidates.Dispose();
            return false; // No potential collisions
        }
        
        // Level 2: Medium detail with convex hulls (if requested)
        if (detailLevel >= CollisionDetailLevel.Medium)
        {
            FilterWithConvexHulls(queryPosition, queryRadius, candidates, 
                convexHullLookup, convexVertexLookup, convexFaceLookup, ref result);
        }
        
        // Level 3: Fine detail with mesh geometry (if requested)
        if (detailLevel >= CollisionDetailLevel.Fine)
        {
            RefineWithMeshData(queryPosition, queryRadius, candidates,
                meshDataLookup, meshVertexLookup, meshTriangleLookup, ref result);
        }
        
        candidates.Dispose();
        return result.HasCollisions;
    }
    
    private static bool QueryCoarsePhase(
        float3 queryPosition,
        float queryRadius,
        in DynamicBuffer<OctreeNode> nodesBuffer,
        in DynamicBuffer<SpatialObject> objects,
        ref NativeList<Entity> candidates)
    {
        // Use existing octree query for coarse detection
        CoarseCollisionCollector collector = new CoarseCollisionCollector
        {
            QueryPosition = queryPosition,
            QueryRadius = queryRadius,
            Candidates = candidates
        };
        
        float3 queryMin = queryPosition - queryRadius;
        float3 queryMax = queryPosition + queryRadius;
        
        // This would use our existing OctreeDynamicOperations.QueryAABB
        // but adapted for the collector
        QueryAABBForCollision(nodesBuffer, objects, queryMin, queryMax, ref collector);
        
        return candidates.Length > 0;
    }
    
    private static void FilterWithConvexHulls(
        float3 queryPosition,
        float queryRadius,
        NativeList<Entity> candidates,
        ComponentLookup<ConvexHullData> convexHullLookup,
        BufferLookup<ConvexHullVertex> vertexLookup,
        BufferLookup<ConvexHullFace> faceLookup,
        ref AvoidanceResult result)
    {
        for (int i = candidates.Length - 1; i >= 0; i--)
        {
            Entity candidate = candidates[i];
            
            // Check if entity has convex hull data
            if (!convexHullLookup.HasComponent(candidate))
            {
                continue; // Fall back to coarse bounds
            }
            
            ConvexHullData hullData = convexHullLookup[candidate];
            
            // Quick sphere-vs-sphere check first
            float distanceSq = math.distancesq(queryPosition, hullData.Center);
            float combinedRadius = queryRadius + hullData.MaxRadius;
            
            if (distanceSq > combinedRadius * combinedRadius)
            {
                candidates.RemoveAtSwapBack(i); // Not colliding
                continue;
            }
            
            // Detailed convex hull vs sphere test
            if (TestSphereVsConvexHull(queryPosition, queryRadius, candidate,
                hullData, vertexLookup, faceLookup, out AvoidanceContact contact))
            {
                result.Contacts.Add(contact);
                result.HasCollisions = true;
            }
            else
            {
                candidates.RemoveAtSwapBack(i); // Not colliding after detailed test
            }
        }
    }
    
    private static void RefineWithMeshData(
        float3 queryPosition,
        float queryRadius,
        NativeList<Entity> candidates,
        ComponentLookup<MeshColliderData> meshDataLookup,
        BufferLookup<MeshVertex> meshVertexLookup,
        BufferLookup<MeshTriangle> meshTriangleLookup,
        ref AvoidanceResult result)
    {
        // Fine-detail mesh collision testing
        for (int i = 0; i < candidates.Length; i++)
        {
            Entity candidate = candidates[i];
            
            if (!meshDataLookup.HasComponent(candidate))
                continue;
            
            MeshColliderData meshData = meshDataLookup[candidate];
            
            // Test against mesh triangles for precise collision
            if (TestSphereVsMesh(queryPosition, queryRadius, candidate,
                meshData, meshVertexLookup, meshTriangleLookup, out AvoidanceContact contact))
            {
                // Update existing contact or add new one
                UpdateOrAddContact(ref result, contact);
            }
        }
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TestSphereVsConvexHull(
        float3 sphereCenter,
        float sphereRadius,
        Entity hullEntity,
        ConvexHullData hullData,
        BufferLookup<ConvexHullVertex> vertexLookup,
        BufferLookup<ConvexHullFace> faceLookup,
        out AvoidanceContact contact)
    {
        contact = default;
        
        if (!faceLookup.HasBuffer(hullEntity))
            return false;
            
        var faces = faceLookup[hullEntity];
        float minSeparation = float.MaxValue;
        float3 bestNormal = float3.zero;
        
        // Test sphere against each face of the convex hull
        for (int i = 0; i < faces.Length; i++)
        {
            ConvexHullFace face = faces[i];
            float separation = math.dot(face.Normal, sphereCenter) - face.Distance - sphereRadius;
            
            if (separation > 0)
                return false; // Separated by this plane
                
            if (separation > minSeparation)
            {
                minSeparation = separation;
                bestNormal = face.Normal;
            }
        }
        
        // All face tests passed - we have collision
        contact = new AvoidanceContact
        {
            Entity = hullEntity,
            ContactPoint = sphereCenter - bestNormal * sphereRadius,
            Normal = bestNormal,
            PenetrationDepth = -minSeparation,
            ContactType = CollisionShapeType.ConvexHull
        };
        
        return true;
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool TestSphereVsMesh(
        float3 sphereCenter,
        float sphereRadius,
        Entity meshEntity,
        MeshColliderData meshData,
        BufferLookup<MeshVertex> meshVertexLookup,
        BufferLookup<MeshTriangle> meshTriangleLookup,
        out AvoidanceContact contact)
    {
        contact = default;
        
        if (!meshTriangleLookup.HasBuffer(meshEntity) || !meshVertexLookup.HasBuffer(meshEntity))
            return false;
        
        var vertices = meshVertexLookup[meshEntity];
        var triangles = meshTriangleLookup[meshEntity];
        
        float closestDistanceSq = float.MaxValue;
        float3 closestPoint = sphereCenter;
        float3 surfaceNormal = math.up();
        
        // Test sphere against each triangle
        for (int i = 0; i < triangles.Length; i++)
        {
            MeshTriangle tri = triangles[i];
            
            // Get triangle vertices
            float3 v0 = vertices[tri.VertexIndices.x].Position;
            float3 v1 = vertices[tri.VertexIndices.y].Position;
            float3 v2 = vertices[tri.VertexIndices.z].Position;
            
            // Find closest point on triangle to sphere center
            float3 pointOnTri = ClosestPointOnTriangle(sphereCenter, v0, v1, v2);
            float distanceSq = math.distancesq(sphereCenter, pointOnTri);
            
            if (distanceSq < closestDistanceSq)
            {
                closestDistanceSq = distanceSq;
                closestPoint = pointOnTri;
                surfaceNormal = tri.FaceNormal;
            }
        }
        
        float distance = math.sqrt(closestDistanceSq);
        if (distance < sphereRadius)
        {
            contact = new AvoidanceContact
            {
                Entity = meshEntity,
                ContactPoint = closestPoint,
                Normal = surfaceNormal,
                PenetrationDepth = sphereRadius - distance,
                ContactType = CollisionShapeType.Mesh
            };
            return true;
        }
        
        return false;
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float3 ClosestPointOnTriangle(float3 point, float3 a, float3 b, float3 c)
    {
        // Barycentric coordinate method for closest point on triangle
        float3 ab = b - a;
        float3 ac = c - a;
        float3 ap = point - a;

        float d1 = math.dot(ab, ap);
        float d2 = math.dot(ac, ap);
        if (d1 <= 0.0f && d2 <= 0.0f) return a;

        float3 bp = point - b;
        float d3 = math.dot(ab, bp);
        float d4 = math.dot(ac, bp);
        if (d3 >= 0.0f && d4 <= d3) return b;

        float vc = d1 * d4 - d3 * d2;
        if (vc <= 0.0f && d1 >= 0.0f && d3 <= 0.0f)
        {
            float v1 = d1 / (d1 - d3);
            return a + v1 * ab;
        }

        float3 cp = point - c;
        float d5 = math.dot(ab, cp);
        float d6 = math.dot(ac, cp);
        if (d6 >= 0.0f && d5 <= d6) return c;

        float vb = d5 * d2 - d1 * d6;
        if (vb <= 0.0f && d2 >= 0.0f && d6 <= 0.0f)
        {
            float w1 = d2 / (d2 - d6);
            return a + w1 * ac;
        }

        float va = d3 * d6 - d5 * d4;
        if (va <= 0.0f && (d4 - d3) >= 0.0f && (d5 - d6) >= 0.0f)
        {
            float w2 = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + w2 * (c - b);
        }

        float denom = 1.0f / (va + vb + vc);
        float v = vb * denom;
        float w = vc * denom;
        return a + ab * v + ac * w;
    }
    
    private static void UpdateOrAddContact(ref AvoidanceResult result, AvoidanceContact newContact)
    {
        // Find existing contact with same entity and update if penetration is deeper
        for (int i = 0; i < result.Contacts.Length; i++)
        {
            if (result.Contacts[i].Entity.Equals(newContact.Entity))
            {
                if (newContact.PenetrationDepth > result.Contacts[i].PenetrationDepth)
                {
                    result.Contacts[i] = newContact;
                }
                return;
            }
        }
        
        // Add new contact
        result.Contacts.Add(newContact);
    }
    
    // Simplified query for coarse phase (placeholder)
    private static void QueryAABBForCollision(
        in DynamicBuffer<OctreeNode> nodesBuffer,
        in DynamicBuffer<SpatialObject> objects,
        float3 queryMin,
        float3 queryMax,
        ref CoarseCollisionCollector collector)
    {
        // This would integrate with existing OctreeDynamicOperations
        // For now, simplified implementation
        for (int i = 0; i < objects.Length; i++)
        {
            var obj = objects[i];
            if (obj.IntersectsAABB(queryMin, queryMax))
            {
                collector.Candidates.Add(obj.Entity);
            }
        }
    }
}

// Result structure for avoidance queries
public struct AvoidanceResult
{
    public bool HasCollisions;
    public NativeList<AvoidanceContact> Contacts;
    
    public void Initialize(Allocator allocator)
    {
        Contacts = new NativeList<AvoidanceContact>(8, allocator);
        HasCollisions = false;
    }
    
    public void Dispose()
    {
        if (Contacts.IsCreated)
            Contacts.Dispose();
    }
}

// Individual collision contact for steering
public struct AvoidanceContact
{
    public Entity Entity;
    public float3 ContactPoint;
    public float3 Normal;           // Surface normal for steering direction
    public float PenetrationDepth;
    public CollisionShapeType ContactType;
}

// Collector for coarse-phase collision detection
public struct CoarseCollisionCollector
{
    public float3 QueryPosition;
    public float QueryRadius;
    public NativeList<Entity> Candidates;
}