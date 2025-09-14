using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

// High-speed AI component for chase/flee behaviors
public struct HighSpeedChaseAI : IComponentData
{
    public Entity TargetEntity;        // Player or target to chase/flee
    public float MaxSpeed;
    public float MaxAcceleration;
    public float ChaseRange;           // When to start chasing
    public float FleeRange;            // When to flee instead
    public float PredictionTime;       // How far ahead to predict target movement
    public AIBehaviorState BehaviorState;
    public float StateTimer;           // Time in current state
    public float NextDecisionTime;     // When to make next behavior decision
}

public enum AIBehaviorState : byte
{
    Idle = 0,
    Chase = 1,
    Flee = 2,
    Ambush = 3,
    Patrol = 4,
    LoseTarget = 5,  // Trying to break line of sight in complex environment
}

[BurstCompile]
// [UpdateAfter(typeof(ShipSystem))] // Commented out - ShipSystem reference
public partial struct HighSpeedChaseSystem : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<FlowFieldCollection>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        float deltaTime = SystemAPI.Time.DeltaTime;
        FlowFieldCollection flowFields = SystemAPI.GetSingleton<FlowFieldCollection>();
        
        var flowFieldLookup = SystemAPI.GetComponentLookup<FlowField>(true);
        var flowCellLookup = SystemAPI.GetBufferLookup<FlowCell>(true);
        var transformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true);
        
        HighSpeedNavigationJob navigationJob = new HighSpeedNavigationJob
        {
            DeltaTime = deltaTime,
            FlowFields = flowFields,
            FlowFieldLookup = flowFieldLookup,
            FlowCellLookup = flowCellLookup,
            TransformLookup = transformLookup,
        };
        
        state.Dependency = navigationJob.ScheduleParallel(state.Dependency);
    }
}

[BurstCompile]
public partial struct HighSpeedNavigationJob : IJobEntity
{
    public float DeltaTime;
    public FlowFieldCollection FlowFields;
    
    [ReadOnly] public ComponentLookup<FlowField> FlowFieldLookup;
    [ReadOnly] public BufferLookup<FlowCell> FlowCellLookup;
    [ReadOnly] public ComponentLookup<LocalTransform> TransformLookup;
    
    void Execute(
        ref LocalTransform transform,
        ref Ship ship,
        ref HighSpeedChaseAI chaseAI,
        in Team team)
    {
        // Get target position and predict future position
        float3 targetPosition = float3.zero;
        float3 targetVelocity = float3.zero;
        bool hasTarget = GetTargetInfo(chaseAI.TargetEntity, out targetPosition, out targetVelocity);
        
        if (!hasTarget)
        {
            chaseAI.BehaviorState = AIBehaviorState.Patrol;
        }
        
        // Update AI decision making
        UpdateBehaviorState(ref chaseAI, transform.Position, targetPosition, targetVelocity);
        
        // Get desired velocity from flow fields based on current behavior
        FlowFieldBlendWeights weights = GetBehaviorWeights(chaseAI.BehaviorState, chaseAI.StateTimer);
        
        float3 flowVelocity = FlowFieldNavigation.GetBlendedFlowVelocity(
            transform.Position,
            weights,
            FlowFieldLookup,
            FlowCellLookup,
            FlowFields,
            chaseAI.MaxSpeed);
        
        // Add predictive component for fast-moving targets
        float3 predictiveVelocity = ComputePredictiveVelocity(
            transform.Position,
            targetPosition,
            targetVelocity,
            chaseAI);
        
        // Blend flow field with predictive component
        float3 desiredVelocity = math.lerp(flowVelocity, predictiveVelocity, GetPredictiveWeight(chaseAI.BehaviorState));
        
        // Apply acceleration limits for realistic movement
        float3 acceleration = (desiredVelocity - ship.Velocity);
        float accelMagnitude = math.length(acceleration);
        
        if (accelMagnitude > chaseAI.MaxAcceleration)
        {
            acceleration = math.normalize(acceleration) * chaseAI.MaxAcceleration;
        }
        
        // Update velocity and position
        ship.Velocity += acceleration * DeltaTime;
        
        // Limit to max speed
        float currentSpeed = math.length(ship.Velocity);
        if (currentSpeed > chaseAI.MaxSpeed)
        {
            ship.Velocity = math.normalize(ship.Velocity) * chaseAI.MaxSpeed;
        }
        
        // Update position
        transform.Position += ship.Velocity * DeltaTime;
        
        // Update rotation to face movement direction
        if (math.lengthsq(ship.Velocity) > 0.001f)
        {
            transform.Rotation = quaternion.LookRotationSafe(math.normalize(ship.Velocity), math.up());
        }
        
        // Update timers
        chaseAI.StateTimer += DeltaTime;
    }
    
    private bool GetTargetInfo(Entity targetEntity, out float3 position, out float3 velocity)
    {
        position = float3.zero;
        velocity = float3.zero;
        
        if (targetEntity == Entity.Null || !TransformLookup.HasComponent(targetEntity))
            return false;
            
        position = TransformLookup[targetEntity].Position;
        
        // Try to get target velocity (assume Ship component)
        // In a real implementation, you'd have a more robust way to get velocity
        velocity = float3.zero; // Placeholder
        
        return true;
    }
    
    private void UpdateBehaviorState(
        ref HighSpeedChaseAI chaseAI,
        float3 currentPosition,
        float3 targetPosition,
        float3 targetVelocity)
    {
        float distanceToTarget = math.distance(currentPosition, targetPosition);
        
        // State transition logic
        switch (chaseAI.BehaviorState)
        {
            case AIBehaviorState.Idle:
            case AIBehaviorState.Patrol:
                if (distanceToTarget < chaseAI.ChaseRange)
                {
                    ChangeState(ref chaseAI, AIBehaviorState.Chase);
                }
                break;
                
            case AIBehaviorState.Chase:
                if (distanceToTarget > chaseAI.ChaseRange * 1.5f)
                {
                    ChangeState(ref chaseAI, AIBehaviorState.Patrol);
                }
                else if (distanceToTarget < chaseAI.FleeRange)
                {
                    ChangeState(ref chaseAI, AIBehaviorState.Flee);
                }
                // Occasionally switch to ambush for smarter behavior
                else if (chaseAI.StateTimer > 3f && UnityEngine.Random.value < 0.1f)
                {
                    ChangeState(ref chaseAI, AIBehaviorState.Ambush);
                }
                break;
                
            case AIBehaviorState.Flee:
                if (distanceToTarget > chaseAI.FleeRange * 2f)
                {
                    ChangeState(ref chaseAI, AIBehaviorState.Chase);
                }
                break;
                
            case AIBehaviorState.Ambush:
                if (chaseAI.StateTimer > 2f) // Ambush for 2 seconds then back to chase
                {
                    ChangeState(ref chaseAI, AIBehaviorState.Chase);
                }
                break;
                
            case AIBehaviorState.LoseTarget:
                if (chaseAI.StateTimer > 4f) // Try to lose target for 4 seconds
                {
                    ChangeState(ref chaseAI, AIBehaviorState.Patrol);
                }
                break;
        }
    }
    
    private void ChangeState(ref HighSpeedChaseAI chaseAI, AIBehaviorState newState)
    {
        chaseAI.BehaviorState = newState;
        chaseAI.StateTimer = 0f;
    }
    
    private FlowFieldBlendWeights GetBehaviorWeights(AIBehaviorState state, float stateTimer)
    {
        return state switch
        {
            AIBehaviorState.Chase => new FlowFieldBlendWeights { ChaseWeight = 1f },
            AIBehaviorState.Flee => new FlowFieldBlendWeights { FleeWeight = 1f },
            AIBehaviorState.Ambush => new FlowFieldBlendWeights { AmbushWeight = 0.7f, ChaseWeight = 0.3f },
            AIBehaviorState.LoseTarget => new FlowFieldBlendWeights { FleeWeight = 0.6f, PatrolWeight = 0.4f },
            _ => new FlowFieldBlendWeights { PatrolWeight = 1f }
        };
    }
    
    private float3 ComputePredictiveVelocity(
        float3 currentPosition,
        float3 targetPosition,
        float3 targetVelocity,
        HighSpeedChaseAI chaseAI)
    {
        // Predict where target will be
        float3 predictedTargetPos = targetPosition + targetVelocity * chaseAI.PredictionTime;
        
        // Direction to intercept point
        float3 toTarget = predictedTargetPos - currentPosition;
        float distance = math.length(toTarget);
        
        if (distance < 0.001f)
            return float3.zero;
            
        return math.normalize(toTarget) * chaseAI.MaxSpeed;
    }
    
    private float GetPredictiveWeight(AIBehaviorState state)
    {
        return state switch
        {
            AIBehaviorState.Chase => 0.3f,      // 30% predictive, 70% flow field
            AIBehaviorState.Ambush => 0.8f,     // 80% predictive for interception
            AIBehaviorState.Flee => 0.1f,       // Mostly flow field for safe escape
            _ => 0f
        };
    }
}