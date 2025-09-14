using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using System.Runtime.CompilerServices;

// Main steering behavior component - holds weights for all behaviors
public partial struct SteeringBehaviors : IComponentData
{
    // Individual behavior weights (0.0 = off, 1.0 = full strength)
    public float SeekWeight;
    public float FleeWeight;
    public float FlockCohesionWeight;
    public float FlockSeparationWeight;  
    public float FlockAlignmentWeight;
    public float WanderWeight;
    public float FlowFieldWeight;        // Integration with flow fields
    public float ObstacleAvoidanceWeight;
    public float LeaderFollowWeight;
    public float PathFollowWeight;
    
    // Behavior parameters
    public float MaxSpeed;
    public float MaxForce;
    public float SeparationRadius;
    public float CohesionRadius;
    public float AlignmentRadius;
    public float WanderRadius;
    public float WanderDistance;
    public float WanderJitter;
    public float ObstacleAvoidanceDistance;
    
    // Targets and state
    public Entity SeekTarget;
    public Entity FleeTarget;
    public Entity LeaderTarget;
    public Entity PathEntity;
    public float3 WanderTarget;     // Current wander target
    
    // Flow field integration
    public FlowFieldBlendWeights FlowFieldWeights;
    public CollisionDetailLevel AvoidanceDetailLevel;
}

// Flock member component - identifies entities that can flock together
public struct FlockMember : IComponentData
{
    public int FlockId;         // Different flocks (0 = neutral, 1+ = specific flocks)
    public float FlockRadius;   // How far to look for flock mates
}

// Main steering system that combines all behaviors
[BurstCompile]
// [UpdateAfter(typeof(ShipSystem))] // Commented out - ShipSystem reference
public partial struct SteeringBehaviorSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<SteeringBehaviors>();
    }

    [BurstCompile] 
    public void OnUpdate(ref SystemState state)
    {
        float deltaTime = SystemAPI.Time.DeltaTime;
        
        // Get spatial database for flocking and obstacle avoidance
        SpatialDatabaseSingleton spatialDatabaseSingleton = SystemAPI.GetSingleton<SpatialDatabaseSingleton>();
        Entity spatialDbEntity = spatialDatabaseSingleton.TargetablesSpatialDatabase;
        
        bool hasOctree = spatialDbEntity != Entity.Null && 
                        state.EntityManager.HasComponent<OctreeSpatialDatabase>(spatialDbEntity);
        
        // Get flow field collection if available
        FlowFieldCollection flowFields = default;
        bool hasFlowFields = SystemAPI.TryGetSingleton<FlowFieldCollection>(out flowFields);
        
        var transformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true);
        var shipLookup = SystemAPI.GetComponentLookup<Ship>(true);
        var flockMemberLookup = SystemAPI.GetComponentLookup<FlockMember>(true);
        
        // Flow field lookups (optional)
        var flowFieldLookup = SystemAPI.GetComponentLookup<FlowField>(true);
        var flowCellLookup = SystemAPI.GetBufferLookup<FlowCell>(true);
        
        // Octree spatial database for obstacle avoidance and flocking
        OctreeSpatialDatabase octreeDatabase = default;
        DynamicBuffer<OctreeNode> nodesBuffer = default;
        DynamicBuffer<SpatialObject> objectsBuffer = default;

        if (hasOctree)
        {
            octreeDatabase = state.EntityManager.GetComponentData<OctreeSpatialDatabase>(spatialDbEntity);
            nodesBuffer = state.EntityManager.GetBuffer<OctreeNode>(spatialDbEntity);
            objectsBuffer = state.EntityManager.GetBuffer<SpatialObject>(spatialDbEntity);
        }
        
        SteeringBehaviorJob steeringJob = new SteeringBehaviorJob
        {
            DeltaTime = deltaTime,
            TransformLookup = transformLookup,
            ShipLookup = shipLookup,
            FlockMemberLookup = flockMemberLookup,
            HasFlowFields = hasFlowFields,
            FlowFields = flowFields,
            FlowFieldLookup = flowFieldLookup,
            FlowCellLookup = flowCellLookup,
            HasOctree = hasOctree,
            OctreeDatabase = octreeDatabase,
            NodesBuffer = nodesBuffer,
            ObjectsBuffer = objectsBuffer,
        };
        
        state.Dependency = steeringJob.ScheduleParallel(state.Dependency);
    }
}

[BurstCompile]
public partial struct SteeringBehaviorJob : IJobEntity
{
    public float DeltaTime;

    [ReadOnly] public ComponentLookup<LocalTransform> TransformLookup;
    [ReadOnly] public ComponentLookup<Ship> ShipLookup;
    [ReadOnly] public ComponentLookup<FlockMember> FlockMemberLookup;

    // Flow field integration
    public bool HasFlowFields;
    public FlowFieldCollection FlowFields;
    [ReadOnly] public ComponentLookup<FlowField> FlowFieldLookup;
    [ReadOnly] public BufferLookup<FlowCell> FlowCellLookup;

    // Octree for flocking and obstacle avoidance
    public bool HasOctree;
    public OctreeSpatialDatabase OctreeDatabase;
    [ReadOnly] public DynamicBuffer<OctreeNode> NodesBuffer;
    [ReadOnly] public DynamicBuffer<SpatialObject> ObjectsBuffer;

    public void Execute(
        Entity entity,
        ref LocalTransform transform,
        ref Ship ship,
        RefRW<SteeringBehaviors> steering)
    {
        float3 totalForce = float3.zero;
        float3 currentVelocity = ship.Velocity;
        float3 position = transform.Position;
        
        // 1. SEEK behavior
        if (steering.ValueRW.SeekWeight > 0f && steering.ValueRW.SeekTarget != Entity.Null)
        {
            float3 seekForce = ComputeSeekForce(position, currentVelocity, steering.ValueRW);
            totalForce += seekForce * steering.ValueRW.SeekWeight;
        }
        
        // 2. FLEE behavior
        if (steering.ValueRW.FleeWeight > 0f && steering.ValueRW.FleeTarget != Entity.Null)
        {
            float3 fleeForce = ComputeFleeForce(position, currentVelocity, steering.ValueRW);
            totalForce += fleeForce * steering.ValueRW.FleeWeight;
        }
        
        // 3. FLOCKING behaviors (requires spatial queries)
        if (HasOctree && IsFlockingActive(steering.ValueRW))
        {
            FlockingForces flockForces = ComputeFlockingForces(entity, position, currentVelocity, steering.ValueRW);
            totalForce += flockForces.Cohesion * steering.ValueRW.FlockCohesionWeight;
            totalForce += flockForces.Separation * steering.ValueRW.FlockSeparationWeight;
            totalForce += flockForces.Alignment * steering.ValueRW.FlockAlignmentWeight;
        }
        
        // 4. WANDER behavior
        if (steering.ValueRW.WanderWeight > 0f)
        {
            float3 wanderForce = ComputeWanderForce(position, currentVelocity, ref steering.ValueRW);
            totalForce += wanderForce * steering.ValueRW.WanderWeight;
        }
        
        // 5. FLOW FIELD behavior (high-performance navigation)
        if (HasFlowFields && steering.ValueRW.FlowFieldWeight > 0f)
        {
            float3 flowForce = ComputeFlowFieldForce(position, steering.ValueRW);
            totalForce += flowForce * steering.ValueRW.FlowFieldWeight;
        }
        
        // 6. OBSTACLE AVOIDANCE behavior
        if (HasOctree && steering.ValueRO.ObstacleAvoidanceWeight > 0f)
        {
            float3 avoidanceForce = ComputeObstacleAvoidanceForce(position, currentVelocity, steering.ValueRO);
            totalForce += avoidanceForce * steering.ValueRO.ObstacleAvoidanceWeight;
        }
        
        // 7. LEADER FOLLOWING behavior
        if (steering.ValueRW.LeaderFollowWeight > 0f && steering.ValueRW.LeaderTarget != Entity.Null)
        {
            float3 followForce = ComputeLeaderFollowForce(position, currentVelocity, steering.ValueRW);
            totalForce += followForce * steering.ValueRW.LeaderFollowWeight;
        }
        
        // Apply force limits
        float forceMagnitude = math.length(totalForce);
        if (forceMagnitude > steering.ValueRW.MaxForce)
        {
            totalForce = math.normalize(totalForce) * steering.ValueRW.MaxForce;
        }
        
        // Update velocity
        ship.Velocity += totalForce * DeltaTime;
        
        // Apply speed limits
        float speed = math.length(ship.Velocity);
        if (speed > steering.ValueRW.MaxSpeed)
        {
            ship.Velocity = math.normalize(ship.Velocity) * steering.ValueRW.MaxSpeed;
        }
        
        // Update position
        transform.Position += ship.Velocity * DeltaTime;
        
        // Update rotation to face movement direction
        if (math.lengthsq(ship.Velocity) > 0.001f)
        {
            transform.Rotation = quaternion.LookRotationSafe(math.normalize(ship.Velocity), math.up());
        }
    }
    
    // SEEK: Move toward target
    private float3 ComputeSeekForce(float3 position, float3 velocity, SteeringBehaviors steering)
    {
        if (!TransformLookup.HasComponent(steering.SeekTarget))
            return float3.zero;
            
        float3 targetPos = TransformLookup[steering.SeekTarget].Position;
        float3 desired = math.normalize(targetPos - position) * steering.MaxSpeed;
        return desired - velocity;
    }
    
    // FLEE: Move away from target
    private float3 ComputeFleeForce(float3 position, float3 velocity, SteeringBehaviors steering)
    {
        if (!TransformLookup.HasComponent(steering.FleeTarget))
            return float3.zero;
            
        float3 targetPos = TransformLookup[steering.FleeTarget].Position;
        float3 desired = math.normalize(position - targetPos) * steering.MaxSpeed;
        return desired - velocity;
    }
    
    // FLOCKING: Cohesion, Separation, Alignment
    private FlockingForces ComputeFlockingForces(Entity entity, float3 position, float3 velocity, SteeringBehaviors steering)
    {
        if (!FlockMemberLookup.HasComponent(entity))
            return new FlockingForces();
            
        FlockMember flockMember = FlockMemberLookup[entity];
        
        // Query nearby flock members using spatial database
        FlockQuery flockQuery = new FlockQuery
        {
            QueryEntity = entity,
            FlockId = flockMember.FlockId,
            Position = position,
            SeparationRadius = steering.SeparationRadius,
            CohesionRadius = steering.CohesionRadius,
            AlignmentRadius = steering.AlignmentRadius,
        };
        
        // Use octree to find nearby flock members
        NativeList<FlockNeighbor> neighbors = new NativeList<FlockNeighbor>(32, Allocator.Temp);
        QueryFlockNeighbors(flockQuery, ref neighbors);
        
        FlockingForces forces = CalculateFlockingForces(position, velocity, neighbors, steering);
        
        neighbors.Dispose();
        return forces;
    }
    
    // WANDER: Random exploration
    private float3 ComputeWanderForce(float3 position, float3 velocity, ref SteeringBehaviors steering)
    {
        // Update wander target with jitter
        steering.WanderTarget += new float3(
            UnityEngine.Random.Range(-1f, 1f) * steering.WanderJitter,
            UnityEngine.Random.Range(-1f, 1f) * steering.WanderJitter,
            UnityEngine.Random.Range(-1f, 1f) * steering.WanderJitter
        );
        
        // Normalize and scale
        steering.WanderTarget = math.normalize(steering.WanderTarget) * steering.WanderRadius;
        
        // Project ahead of the agent
        float3 velocityNorm = math.normalize(velocity);
        float3 wanderCenter = position + velocityNorm * steering.WanderDistance;
        float3 wanderPos = wanderCenter + steering.WanderTarget;
        
        return math.normalize(wanderPos - position) * steering.MaxSpeed - velocity;
    }
    
    // FLOW FIELD: High-performance navigation around complex obstacles
    private float3 ComputeFlowFieldForce(float3 position, SteeringBehaviors steering)
    {
        if (!HasFlowFields)
            return float3.zero;
            
        float3 flowVelocity = FlowFieldNavigation.GetBlendedFlowVelocity(
            position,
            steering.FlowFieldWeights,
            FlowFieldLookup,
            FlowCellLookup,
            FlowFields,
            steering.MaxSpeed);
            
        return flowVelocity; // Flow field provides desired velocity directly
    }
    
    // OBSTACLE AVOIDANCE: Avoid static obstacles
    private float3 ComputeObstacleAvoidanceForce(float3 position, float3 velocity, SteeringBehaviors steering)
    {
        if (!HasOctree)
            return float3.zero;
            
        // Use simplified SDF-based avoidance for performance
        // This could be enhanced with the hierarchical collision system if needed
        float3 ahead = position + math.normalize(velocity) * steering.ObstacleAvoidanceDistance;
        
        // Query obstacles in front
        ObstacleQuery obstacleQuery = new ObstacleQuery
        {
            Position = ahead,
            Radius = steering.ObstacleAvoidanceDistance * 0.5f,
        };
        
        return QueryObstacleAvoidance(obstacleQuery, position, velocity, steering.MaxForce);
    }
    
    // LEADER FOLLOWING: Follow another entity
    private float3 ComputeLeaderFollowForce(float3 position, float3 velocity, SteeringBehaviors steering)
    {
        if (!TransformLookup.HasComponent(steering.LeaderTarget))
            return float3.zero;
            
        float3 leaderPos = TransformLookup[steering.LeaderTarget].Position;
        float3 leaderVel = float3.zero;
        
        if (ShipLookup.HasComponent(steering.LeaderTarget))
        {
            leaderVel = ShipLookup[steering.LeaderTarget].Velocity;
        }
        
        // Follow behind the leader
        float3 behindLeader = leaderPos - math.normalize(leaderVel) * 5f; // 5 units behind
        float3 desired = math.normalize(behindLeader - position) * steering.MaxSpeed;
        
        return desired - velocity;
    }
    
    // Helper functions
    private bool IsFlockingActive(SteeringBehaviors steering)
    {
        return steering.FlockCohesionWeight > 0f || 
               steering.FlockSeparationWeight > 0f || 
               steering.FlockAlignmentWeight > 0f;
    }
    
    private void QueryFlockNeighbors(FlockQuery query, ref NativeList<FlockNeighbor> neighbors)
    {
        if (!HasOctree || NodesBuffer.Length == 0 || ObjectsBuffer.Length == 0)
            return;
            
        // Get maximum search radius for all flocking behaviors
        float maxRadius = math.max(query.SeparationRadius, math.max(query.CohesionRadius, query.AlignmentRadius));
        
        // Create AABB query around the position
        AABB queryBounds = new AABB
        {
            Center = query.Position,
            Extents = new float3(maxRadius)
        };
        
        // Use a temporary list to collect spatial objects
        NativeList<SpatialObject> candidateObjects = new NativeList<SpatialObject>(32, Allocator.Temp);

        // Query octree for nearby objects
        float3 queryMin = queryBounds.Center - queryBounds.Extents;
        float3 queryMax = queryBounds.Center + queryBounds.Extents;

        var collector = new FlockQueryCollector
        {
            CandidateObjects = candidateObjects
        };

        OctreeDynamicOperations.QueryAABB(
            OctreeDatabase,
            NodesBuffer,
            ObjectsBuffer,
            queryMin,
            queryMax,
            ref collector);

        // Process candidates and filter for flock members
        for (int i = 0; i < candidateObjects.Length; i++)
        {
            SpatialObject spatialObject = candidateObjects[i];
            Entity candidateEntity = spatialObject.Entity;
            
            // Skip self
            if (candidateEntity == query.QueryEntity)
                continue;
                
            // Check if entity has flock member component and same flock ID
            if (!FlockMemberLookup.HasComponent(candidateEntity))
                continue;
                
            FlockMember candidateFlock = FlockMemberLookup[candidateEntity];
            if (candidateFlock.FlockId != query.FlockId)
                continue;
                
            // Get candidate position and velocity
            if (!TransformLookup.HasComponent(candidateEntity))
                continue;
                
            float3 candidatePos = TransformLookup[candidateEntity].Position;
            float distance = math.distance(query.Position, candidatePos);
            
            // Check if within any of the flocking radii
            if (distance <= maxRadius)
            {
                float3 candidateVel = float3.zero;
                if (ShipLookup.HasComponent(candidateEntity))
                {
                    candidateVel = ShipLookup[candidateEntity].Velocity;
                }
                
                FlockNeighbor neighbor = new FlockNeighbor
                {
                    Entity = candidateEntity,
                    Position = candidatePos,
                    Velocity = candidateVel,
                    Distance = distance
                };
                
                neighbors.Add(neighbor);
            }
        }
        
        candidateObjects.Dispose();
    }
    
    private FlockingForces CalculateFlockingForces(
        float3 position, 
        float3 velocity, 
        NativeList<FlockNeighbor> neighbors, 
        SteeringBehaviors steering)
    {
        FlockingForces forces = new FlockingForces();
        
        if (neighbors.Length == 0)
            return forces;
        
        // Cohesion: steer toward center of nearby flockmates
        float3 centerOfMass = float3.zero;
        int cohesionCount = 0;
        
        // Separation: steer away from nearby flockmates
        float3 separationForce = float3.zero;
        int separationCount = 0;
        
        // Alignment: match velocity of nearby flockmates
        float3 averageVelocity = float3.zero;
        int alignmentCount = 0;
        
        for (int i = 0; i < neighbors.Length; i++)
        {
            FlockNeighbor neighbor = neighbors[i];
            float distance = math.distance(position, neighbor.Position);
            
            // Cohesion
            if (distance < steering.CohesionRadius)
            {
                centerOfMass += neighbor.Position;
                cohesionCount++;
            }
            
            // Separation
            if (distance < steering.SeparationRadius)
            {
                float3 flee = position - neighbor.Position;
                flee = math.normalize(flee) / distance; // Weight by distance
                separationForce += flee;
                separationCount++;
            }
            
            // Alignment
            if (distance < steering.AlignmentRadius)
            {
                averageVelocity += neighbor.Velocity;
                alignmentCount++;
            }
        }
        
        // Calculate final forces
        if (cohesionCount > 0)
        {
            centerOfMass /= cohesionCount;
            forces.Cohesion = math.normalize(centerOfMass - position) * steering.MaxSpeed - velocity;
        }
        
        if (separationCount > 0)
        {
            separationForce /= separationCount;
            forces.Separation = math.normalize(separationForce) * steering.MaxSpeed;
        }
        
        if (alignmentCount > 0)
        {
            averageVelocity /= alignmentCount;
            forces.Alignment = math.normalize(averageVelocity) * steering.MaxSpeed - velocity;
        }
        
        return forces;
    }
    
    private float3 QueryObstacleAvoidance(ObstacleQuery query, float3 position, float3 velocity, float maxForce)
    {
        if (!HasOctree || NodesBuffer.Length == 0 || ObjectsBuffer.Length == 0)
            return float3.zero;
            
        // Create AABB query around the look-ahead position
        AABB queryBounds = new AABB
        {
            Center = query.Position,
            Extents = new float3(query.Radius)
        };
        
        // Use a temporary list to collect spatial objects
        NativeList<SpatialObject> obstacleObjects = new NativeList<SpatialObject>(16, Allocator.Temp);

        // Query octree for nearby obstacles
        float3 queryMin = queryBounds.Center - queryBounds.Extents;
        float3 queryMax = queryBounds.Center + queryBounds.Extents;

        var collector = new ObstacleQueryCollector
        {
            ObstacleObjects = obstacleObjects
        };

        OctreeDynamicOperations.QueryAABB(
            OctreeDatabase,
            NodesBuffer,
            ObjectsBuffer,
            queryMin,
            queryMax,
            ref collector);

        float3 totalAvoidanceForce = float3.zero;
        int obstacleCount = 0;

        // Process each potential obstacle
        for (int i = 0; i < obstacleObjects.Length; i++)
        {
            SpatialObject spatialObject = obstacleObjects[i];
            Entity obstacleEntity = spatialObject.Entity;
            
            // Get obstacle position (use transform lookup)
            if (!TransformLookup.HasComponent(obstacleEntity))
                continue;
                
            float3 obstaclePos = TransformLookup[obstacleEntity].Position;
            float3 toObstacle = obstaclePos - position;
            float distanceToObstacle = math.length(toObstacle);
            
            // Skip if obstacle is too far or behind us
            if (distanceToObstacle > query.Radius * 2f || distanceToObstacle < 0.001f)
                continue;
                
            float3 toObstacleNorm = toObstacle / distanceToObstacle;
            float3 velocityNorm = math.normalize(velocity);
            
            // Check if obstacle is in front of movement direction
            float dot = math.dot(velocityNorm, toObstacleNorm);
            if (dot < 0.3f) // Only avoid obstacles somewhat in front
                continue;
            
            // Calculate avoidance force based on obstacle bounding sphere
            // Use half extents as radius approximation
            float obstacleRadius = math.length(spatialObject.HalfExtents);
            float totalRadius = obstacleRadius + query.Radius;
            
            if (distanceToObstacle < totalRadius)
            {
                // We're inside or very close - strong repulsion perpendicular to approach
                float penetration = totalRadius - distanceToObstacle;
                float avoidanceStrength = math.saturate(penetration / query.Radius);
                
                // Find perpendicular direction for avoidance
                float3 avoidanceDir = ComputeAvoidanceDirection(position, velocity, obstaclePos, obstacleRadius);
                
                totalAvoidanceForce += avoidanceDir * maxForce * avoidanceStrength;
                obstacleCount++;
            }
        }
        
        obstacleObjects.Dispose();
        
        // Average and limit the avoidance force
        if (obstacleCount > 0)
        {
            totalAvoidanceForce /= obstacleCount;
            
            float forceMagnitude = math.length(totalAvoidanceForce);
            if (forceMagnitude > maxForce)
            {
                totalAvoidanceForce = math.normalize(totalAvoidanceForce) * maxForce;
            }
        }
        
        return totalAvoidanceForce;
    }
    
    private float3 ComputeAvoidanceDirection(float3 position, float3 velocity, float3 obstaclePos, float obstacleRadius)
    {
        float3 toObstacle = obstaclePos - position;
        float3 velocityNorm = math.normalize(velocity);
        
        // Project velocity onto the line to obstacle to find closest approach point
        float projection = math.dot(velocityNorm, toObstacle);
        float3 closestPoint = position + velocityNorm * projection;
        
        // Direction from obstacle to closest approach point
        float3 avoidanceDir = closestPoint - obstaclePos;
        float avoidanceDistance = math.length(avoidanceDir);
        
        if (avoidanceDistance < 0.001f)
        {
            // If we're heading directly at center, pick a perpendicular direction
            float3 up = math.up();
            if (math.abs(math.dot(velocityNorm, up)) > 0.9f)
                up = math.right(); // Use right if velocity is mostly vertical
                
            avoidanceDir = math.normalize(math.cross(velocityNorm, up));
        }
        else
        {
            avoidanceDir = math.normalize(avoidanceDir);
        }
        
        return avoidanceDir;
    }
}

// Supporting data structures
public struct FlockingForces
{
    public float3 Cohesion;
    public float3 Separation;
    public float3 Alignment;
}

public struct FlockNeighbor
{
    public Entity Entity;
    public float3 Position;
    public float3 Velocity;
    public float Distance;
}

public struct FlockQuery
{
    public Entity QueryEntity;
    public int FlockId;
    public float3 Position;
    public float SeparationRadius;
    public float CohesionRadius;  
    public float AlignmentRadius;
}

public struct ObstacleQuery
{
    public float3 Position;
    public float Radius;
}

// Query collectors for octree operations
[BurstCompile]
public struct FlockQueryCollector : IOctreeQueryCollector
{
    public NativeList<SpatialObject> CandidateObjects;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void OnVisitObject(in SpatialObject obj, out bool shouldEarlyExit)
    {
        CandidateObjects.Add(obj);
        shouldEarlyExit = false;
    }
}

[BurstCompile]
public struct ObstacleQueryCollector : IOctreeQueryCollector
{
    public NativeList<SpatialObject> ObstacleObjects;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void OnVisitObject(in SpatialObject obj, out bool shouldEarlyExit)
    {
        ObstacleObjects.Add(obj);
        shouldEarlyExit = false;
    }
}