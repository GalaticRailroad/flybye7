using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

// Enhanced ship navigation component with avoidance settings
public struct ComplexAvoidanceData : IComponentData
{
    public float AvoidanceRadius;
    public float MaxAvoidanceForce;
    public CollisionDetailLevel DetailLevel;
    public bool UseComplexGeometry;
}

// System that integrates complex avoidance into ship navigation
[BurstCompile]
// [UpdateAfter(typeof(ShipSystem))] // Commented out - ShipSystem reference
public partial struct ComplexAvoidanceSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<SpatialDatabaseSingleton>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        SpatialDatabaseSingleton spatialDatabaseSingleton = SystemAPI.GetSingleton<SpatialDatabaseSingleton>();
        Entity spatialDbEntity = spatialDatabaseSingleton.TargetablesSpatialDatabase;
        
        if (spatialDbEntity == Entity.Null || 
            !state.EntityManager.HasComponent<OctreeSpatialDatabase>(spatialDbEntity))
            return;
        
        // Get all the lookup tables needed for complex collision queries
        var convexHullLookup = SystemAPI.GetComponentLookup<ConvexHullData>(true);
        var convexVertexLookup = SystemAPI.GetBufferLookup<ConvexHullVertex>(true);
        var convexFaceLookup = SystemAPI.GetBufferLookup<ConvexHullFace>(true);
        var meshDataLookup = SystemAPI.GetComponentLookup<MeshColliderData>(true);
        var meshVertexLookup = SystemAPI.GetBufferLookup<MeshVertex>(true);
        var meshTriangleLookup = SystemAPI.GetBufferLookup<MeshTriangle>(true);
        
        DynamicBuffer<OctreeNode> nodesBuffer = state.EntityManager.GetBuffer<OctreeNode>(spatialDbEntity);
        DynamicBuffer<SpatialObject> objectsBuffer = state.EntityManager.GetBuffer<SpatialObject>(spatialDbEntity);

        ComplexAvoidanceJob avoidanceJob = new ComplexAvoidanceJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            NodesBuffer = nodesBuffer,
            ObjectsBuffer = objectsBuffer,
            ConvexHullLookup = convexHullLookup,
            ConvexVertexLookup = convexVertexLookup,
            ConvexFaceLookup = convexFaceLookup,
            MeshDataLookup = meshDataLookup,
            MeshVertexLookup = meshVertexLookup,
            MeshTriangleLookup = meshTriangleLookup,
        };
        
        state.Dependency = avoidanceJob.ScheduleParallel(state.Dependency);
    }
}

[BurstCompile]
public partial struct ComplexAvoidanceJob : IJobEntity
{
    public float DeltaTime;
    
    [ReadOnly] public DynamicBuffer<OctreeNode> NodesBuffer;
    [ReadOnly] public DynamicBuffer<SpatialObject> ObjectsBuffer;
    [ReadOnly] public ComponentLookup<ConvexHullData> ConvexHullLookup;
    [ReadOnly] public BufferLookup<ConvexHullVertex> ConvexVertexLookup;
    [ReadOnly] public BufferLookup<ConvexHullFace> ConvexFaceLookup;
    [ReadOnly] public ComponentLookup<MeshColliderData> MeshDataLookup;
    [ReadOnly] public BufferLookup<MeshVertex> MeshVertexLookup;
    [ReadOnly] public BufferLookup<MeshTriangle> MeshTriangleLookup;
    
    public void Execute(
        ref Ship ship,
        RefRW<LocalTransform> transform,
        in ComplexAvoidanceData avoidanceData)
    {
        if (!avoidanceData.UseComplexGeometry)
            return; // Use simple avoidance instead
            
        // Get current velocity
        float3 currentVelocity = ship.Velocity;
        float currentSpeed = math.length(currentVelocity);
        
        if (currentSpeed < 0.001f)
            return; // Not moving
        
        // Compute avoidance force using hierarchical collision detection
        float3 avoidanceForce = SteeringBehaviorsUtility.ComputeAvoidanceForce(
            transform.ValueRW.Position,
            currentVelocity,
            avoidanceData.AvoidanceRadius,
            avoidanceData.MaxAvoidanceForce,
            avoidanceData.DetailLevel,
            NodesBuffer,
            ObjectsBuffer,
            ConvexHullLookup,
            ConvexVertexLookup,
            ConvexFaceLookup,
            MeshDataLookup,
            MeshVertexLookup,
            MeshTriangleLookup);

        // Apply avoidance force to ship velocity
        float3 desiredVelocity = currentVelocity + avoidanceForce * DeltaTime;

        // Limit to max speed (using ship's current max speed)
        float desiredSpeed = math.length(desiredVelocity);
        float maxSpeed = 50f; // Default max speed, should be configured elsewhere
        if (desiredSpeed > maxSpeed)
        {
            desiredVelocity = math.normalize(desiredVelocity) * maxSpeed;
        }

        // Update ship velocity and position
        ship.Velocity = desiredVelocity;
        transform.ValueRW.Position += ship.Velocity * DeltaTime;

        // Update rotation to face movement direction
        if (math.lengthsq(ship.Velocity) > 0.001f)
        {
            transform.ValueRW.Rotation = quaternion.LookRotationSafe(math.normalize(ship.Velocity), math.up());
        }
    }
}

// Authoring component for complex avoidance settings
public class ComplexAvoidanceAuthoring : UnityEngine.MonoBehaviour
{
    [UnityEngine.Header("Complex Avoidance Settings")]
    public float AvoidanceRadius = 5.0f;
    public float MaxAvoidanceForce = 10.0f;
    public CollisionDetailLevel DetailLevel = CollisionDetailLevel.Medium;
    public bool UseComplexGeometry = true;
    
    class Baker : Baker<ComplexAvoidanceAuthoring>
    {
        public override void Bake(ComplexAvoidanceAuthoring authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent(entity, new ComplexAvoidanceData
            {
                AvoidanceRadius = authoring.AvoidanceRadius,
                MaxAvoidanceForce = authoring.MaxAvoidanceForce,
                DetailLevel = authoring.DetailLevel,
                UseComplexGeometry = authoring.UseComplexGeometry
            });
        }
    }
}