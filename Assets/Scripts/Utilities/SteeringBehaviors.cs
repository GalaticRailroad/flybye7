using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[BurstCompile]
public static class SteeringBehaviorsUtility
{
    // Main avoidance steering function
    public static float3 ComputeAvoidanceForce(
        float3 currentPosition,
        float3 currentVelocity,
        float avoidanceRadius,
        float maxSpeed,
        CollisionDetailLevel detailLevel,
        in DynamicBuffer<OctreeNode> nodesBuffer,
        in DynamicBuffer<SpatialObject> coarseObjects,
        ComponentLookup<ConvexHullData> convexHullLookup,
        BufferLookup<ConvexHullVertex> convexVertexLookup,
        BufferLookup<ConvexHullFace> convexFaceLookup,
        ComponentLookup<MeshColliderData> meshDataLookup,
        BufferLookup<MeshVertex> meshVertexLookup,
        BufferLookup<MeshTriangle> meshTriangleLookup)
    {
        AvoidanceResult result = new AvoidanceResult();
        result.Initialize(Allocator.Temp);
        
        // Query for potential collisions
        bool hasCollisions = HierarchicalCollisionQueries.QueryForAvoidance(
            currentPosition,
            avoidanceRadius,
            detailLevel,
            nodesBuffer,
            coarseObjects,
            convexHullLookup,
            convexVertexLookup,
            convexFaceLookup,
            meshDataLookup,
            meshVertexLookup,
            meshTriangleLookup,
            out result);
        
        float3 steeringForce = float3.zero;
        
        if (hasCollisions && result.Contacts.Length > 0)
        {
            // Compute steering force based on collision contacts
            steeringForce = ComputeSteeringFromContacts(
                currentPosition,
                currentVelocity,
                result.Contacts,
                avoidanceRadius,
                maxSpeed);
        }
        
        result.Dispose();
        return steeringForce;
    }
    
    private static float3 ComputeSteeringFromContacts(
        float3 position,
        float3 velocity,
        NativeList<AvoidanceContact> contacts,
        float avoidanceRadius,
        float maxSpeed)
    {
        float3 totalForce = float3.zero;
        float3 velocityNorm = math.normalize(velocity);
        
        for (int i = 0; i < contacts.Length; i++)
        {
            AvoidanceContact contact = contacts[i];
            
            // Calculate avoidance force based on contact type and penetration
            float3 avoidanceForce = ComputeContactAvoidanceForce(
                position,
                velocity,
                velocityNorm,
                contact,
                avoidanceRadius,
                maxSpeed);
            
            totalForce += avoidanceForce;
        }
        
        // Limit the total steering force
        float forceMagnitude = math.length(totalForce);
        if (forceMagnitude > maxSpeed)
        {
            totalForce = math.normalize(totalForce) * maxSpeed;
        }
        
        return totalForce;
    }
    
    private static float3 ComputeContactAvoidanceForce(
        float3 position,
        float3 velocity,
        float3 velocityNorm,
        AvoidanceContact contact,
        float avoidanceRadius,
        float maxSpeed)
    {
        float3 toContact = contact.ContactPoint - position;
        float distance = math.length(toContact);
        
        if (distance < 0.001f)
            return float3.zero;
        
        float3 toContactNorm = toContact / distance;
        
        // Different avoidance strategies based on collision type
        switch (contact.ContactType)
        {
            case CollisionShapeType.Mesh:
                return ComputeMeshAvoidance(position, velocity, contact, avoidanceRadius, maxSpeed);
                
            case CollisionShapeType.ConvexHull:
                return ComputeConvexAvoidance(position, velocity, contact, avoidanceRadius, maxSpeed);
                
            case CollisionShapeType.Compound:
                return ComputeCompoundAvoidance(position, velocity, contact, avoidanceRadius, maxSpeed);
                
            default:
                // Fallback to simple sphere avoidance
                return ComputeSphereAvoidance(position, velocity, toContactNorm, distance, avoidanceRadius, maxSpeed);
        }
    }
    
    private static float3 ComputeMeshAvoidance(
        float3 position,
        float3 velocity,
        AvoidanceContact contact,
        float avoidanceRadius,
        float maxSpeed)
    {
        // Use surface normal for precise steering away from mesh surface
        float3 surfaceNormal = contact.Normal;
        
        // Project velocity onto surface to find sliding direction
        float3 velocityProjected = velocity - math.dot(velocity, surfaceNormal) * surfaceNormal;
        
        // Steering force combines normal repulsion with tangential sliding
        float penetrationFactor = math.saturate(contact.PenetrationDepth / avoidanceRadius);
        float3 repulsionForce = surfaceNormal * maxSpeed * penetrationFactor;
        float3 slidingForce = math.normalize(velocityProjected) * maxSpeed * (1.0f - penetrationFactor);
        
        return repulsionForce + slidingForce * 0.5f;
    }
    
    private static float3 ComputeConvexAvoidance(
        float3 position,
        float3 velocity,
        AvoidanceContact contact,
        float avoidanceRadius,
        float maxSpeed)
    {
        // Similar to mesh but potentially simplified
        float3 surfaceNormal = contact.Normal;
        float penetrationFactor = math.saturate(contact.PenetrationDepth / avoidanceRadius);
        
        return surfaceNormal * maxSpeed * penetrationFactor;
    }
    
    private static float3 ComputeCompoundAvoidance(
        float3 position,
        float3 velocity,
        AvoidanceContact contact,
        float avoidanceRadius,
        float maxSpeed)
    {
        // For compound colliders, use the contact normal from the closest primitive
        float3 avoidanceDir = contact.Normal;
        float strength = math.saturate(contact.PenetrationDepth / avoidanceRadius);
        
        return avoidanceDir * maxSpeed * strength;
    }
    
    private static float3 ComputeSphereAvoidance(
        float3 position,
        float3 velocity,
        float3 directionToObstacle,
        float distance,
        float avoidanceRadius,
        float maxSpeed)
    {
        // Classic sphere-based avoidance
        float avoidanceStrength = math.saturate((avoidanceRadius - distance) / avoidanceRadius);
        float3 avoidanceDirection = -directionToObstacle;
        
        return avoidanceDirection * maxSpeed * avoidanceStrength;
    }
    
    // Pathfinding integration - check if path segment intersects complex geometry
    public static bool IsPathClear(
        float3 startPos,
        float3 endPos,
        float agentRadius,
        CollisionDetailLevel detailLevel,
        in DynamicBuffer<OctreeNode> nodesBuffer,
        in DynamicBuffer<SpatialObject> coarseObjects,
        ComponentLookup<ConvexHullData> convexHullLookup,
        BufferLookup<ConvexHullVertex> convexVertexLookup,
        BufferLookup<ConvexHullFace> convexFaceLookup,
        ComponentLookup<MeshColliderData> meshDataLookup,
        BufferLookup<MeshVertex> meshVertexLookup,
        BufferLookup<MeshTriangle> meshTriangleLookup)
    {
        float3 pathDirection = endPos - startPos;
        float pathLength = math.length(pathDirection);
        
        if (pathLength < 0.001f)
            return true;
        
        pathDirection /= pathLength;
        
        // Sample points along the path and test for collisions
        int sampleCount = math.max(1, (int)(pathLength / agentRadius));
        float stepSize = pathLength / sampleCount;
        
        for (int i = 0; i <= sampleCount; i++)
        {
            float3 samplePos = startPos + pathDirection * (i * stepSize);
            
            AvoidanceResult result = new AvoidanceResult();
            result.Initialize(Allocator.Temp);
            
            bool hasCollision = HierarchicalCollisionQueries.QueryForAvoidance(
                samplePos,
                agentRadius,
                detailLevel,
                nodesBuffer,
                coarseObjects,
                convexHullLookup,
                convexVertexLookup,
                convexFaceLookup,
                meshDataLookup,
                meshVertexLookup,
                meshTriangleLookup,
                out result);
            
            result.Dispose();
            
            if (hasCollision)
                return false;
        }
        
        return true;
    }
}