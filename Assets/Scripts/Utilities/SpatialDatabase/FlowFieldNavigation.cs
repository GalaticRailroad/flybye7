using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

// High-speed navigation using pre-computed flow fields
public struct FlowField : IComponentData
{
    public float3 BoundsMin;
    public float3 BoundsMax;
    public int3 Resolution; // Grid resolution (e.g., 64x64x64)
    public float CellSize;
    public FlowFieldType FieldType;
}

public enum FlowFieldType : byte
{
    Chase = 0,      // Flow toward target with obstacle avoidance
    Flee = 1,       // Flow away from threat through safe passages  
    Patrol = 2,     // Circular/patrol flows around areas
    Ambush = 3,     // Intercept flows to cut off escape routes
}

// Flow field cell data - very compact for performance
[InternalBufferCapacity(0)]
public struct FlowCell : IBufferElementData
{
    public float3 Velocity;         // Desired velocity vector
    public float DistanceToObstacle; // SDF - distance to nearest obstacle
    public byte Flags;              // Special properties (blocked, slow zone, etc.)
}

// Multiple flow fields for different behaviors
public struct FlowFieldCollection : IComponentData
{
    public Entity ChaseFieldEntity;
    public Entity FleeFieldEntity;
    public Entity PatrolFieldEntity;
    public Entity AmbushFieldEntity;
}

[BurstCompile]
public static class FlowFieldNavigation
{
    // Ultra-fast navigation query - just sample the flow field
    public static float3 GetNavigationVelocity(
        float3 worldPosition,
        FlowFieldType fieldType,
        in FlowField flowField,
        in DynamicBuffer<FlowCell> flowCells,
        float maxSpeed)
    {
        // Convert world position to grid coordinates
        float3 localPos = worldPosition - flowField.BoundsMin;
        int3 gridPos = (int3)(localPos / flowField.CellSize);
        
        // Bounds check
        if (math.any(gridPos < 0) || math.any(gridPos >= flowField.Resolution))
            return float3.zero;
            
        // Get cell index
        int cellIndex = GridToIndex(gridPos, flowField.Resolution);
        if (cellIndex >= flowCells.Length)
            return float3.zero;
            
        FlowCell cell = flowCells[cellIndex];
        
        // Return pre-computed velocity, scaled by max speed
        return cell.Velocity * maxSpeed;
    }
    
    // Get obstacle avoidance force using SDF gradient
    public static float3 GetObstacleAvoidanceForce(
        float3 worldPosition,
        float avoidanceRadius,
        in FlowField flowField,
        in DynamicBuffer<FlowCell> flowCells,
        float maxForce)
    {
        float3 localPos = worldPosition - flowField.BoundsMin;
        int3 gridPos = (int3)(localPos / flowField.CellSize);
        
        if (math.any(gridPos < 0) || math.any(gridPos >= flowField.Resolution))
            return float3.zero;
            
        int cellIndex = GridToIndex(gridPos, flowField.Resolution);
        if (cellIndex >= flowCells.Length)
            return float3.zero;
            
        FlowCell cell = flowCells[cellIndex];
        
        // If we're too close to an obstacle, compute repulsion force
        if (cell.DistanceToObstacle < avoidanceRadius)
        {
            // Compute SDF gradient for repulsion direction
            float3 gradient = ComputeSDFGradient(worldPosition, flowField, flowCells);
            float strength = math.saturate((avoidanceRadius - cell.DistanceToObstacle) / avoidanceRadius);
            
            return gradient * strength * maxForce;
        }
        
        return float3.zero;
    }
    
    // Get multiple flow field influences and blend them
    public static float3 GetBlendedFlowVelocity(
        float3 worldPosition,
        FlowFieldBlendWeights weights,
        ComponentLookup<FlowField> flowFieldLookup,
        BufferLookup<FlowCell> flowCellLookup,
        in FlowFieldCollection collection,
        float maxSpeed)
    {
        float3 totalVelocity = float3.zero;
        float totalWeight = 0f;
        
        // Blend chase flow
        if (weights.ChaseWeight > 0f && collection.ChaseFieldEntity != Entity.Null)
        {
            if (flowFieldLookup.HasComponent(collection.ChaseFieldEntity) && 
                flowCellLookup.HasBuffer(collection.ChaseFieldEntity))
            {
                FlowField chaseField = flowFieldLookup[collection.ChaseFieldEntity];
                DynamicBuffer<FlowCell> chaseCells = flowCellLookup[collection.ChaseFieldEntity];
                
                float3 chaseVel = GetNavigationVelocity(worldPosition, FlowFieldType.Chase, 
                    chaseField, chaseCells, maxSpeed);
                totalVelocity += chaseVel * weights.ChaseWeight;
                totalWeight += weights.ChaseWeight;
            }
        }
        
        // Blend flee flow
        if (weights.FleeWeight > 0f && collection.FleeFieldEntity != Entity.Null)
        {
            if (flowFieldLookup.HasComponent(collection.FleeFieldEntity) && 
                flowCellLookup.HasBuffer(collection.FleeFieldEntity))
            {
                FlowField fleeField = flowFieldLookup[collection.FleeFieldEntity];
                DynamicBuffer<FlowCell> fleeCells = flowCellLookup[collection.FleeFieldEntity];
                
                float3 fleeVel = GetNavigationVelocity(worldPosition, FlowFieldType.Flee, 
                    fleeField, fleeCells, maxSpeed);
                totalVelocity += fleeVel * weights.FleeWeight;
                totalWeight += weights.FleeWeight;
            }
        }
        
        // Blend patrol flow
        if (weights.PatrolWeight > 0f && collection.PatrolFieldEntity != Entity.Null)
        {
            if (flowFieldLookup.HasComponent(collection.PatrolFieldEntity) && 
                flowCellLookup.HasBuffer(collection.PatrolFieldEntity))
            {
                FlowField patrolField = flowFieldLookup[collection.PatrolFieldEntity];
                DynamicBuffer<FlowCell> patrolCells = flowCellLookup[collection.PatrolFieldEntity];
                
                float3 patrolVel = GetNavigationVelocity(worldPosition, FlowFieldType.Patrol, 
                    patrolField, patrolCells, maxSpeed);
                totalVelocity += patrolVel * weights.PatrolWeight;
                totalWeight += weights.PatrolWeight;
            }
        }
        
        // Normalize by total weight
        if (totalWeight > 0f)
        {
            totalVelocity /= totalWeight;
        }
        
        return totalVelocity;
    }
    
    // Compute SDF gradient for obstacle repulsion
    private static float3 ComputeSDFGradient(
        float3 worldPosition,
        in FlowField flowField,
        in DynamicBuffer<FlowCell> flowCells)
    {
        float epsilon = flowField.CellSize * 0.5f;
        
        // Sample SDF at neighboring points to compute gradient
        float distanceX0 = SampleSDF(worldPosition + new float3(-epsilon, 0, 0), flowField, flowCells);
        float distanceX1 = SampleSDF(worldPosition + new float3(epsilon, 0, 0), flowField, flowCells);
        float distanceY0 = SampleSDF(worldPosition + new float3(0, -epsilon, 0), flowField, flowCells);
        float distanceY1 = SampleSDF(worldPosition + new float3(0, epsilon, 0), flowField, flowCells);
        float distanceZ0 = SampleSDF(worldPosition + new float3(0, 0, -epsilon), flowField, flowCells);
        float distanceZ1 = SampleSDF(worldPosition + new float3(0, 0, epsilon), flowField, flowCells);
        
        float3 gradient = new float3(
            distanceX1 - distanceX0,
            distanceY1 - distanceY0,
            distanceZ1 - distanceZ0
        ) / (2f * epsilon);
        
        return math.normalize(gradient);
    }
    
    private static float SampleSDF(
        float3 worldPosition,
        in FlowField flowField,
        in DynamicBuffer<FlowCell> flowCells)
    {
        float3 localPos = worldPosition - flowField.BoundsMin;
        int3 gridPos = (int3)(localPos / flowField.CellSize);
        
        if (math.any(gridPos < 0) || math.any(gridPos >= flowField.Resolution))
            return float.MaxValue; // Outside bounds = very far from obstacles
            
        int cellIndex = GridToIndex(gridPos, flowField.Resolution);
        if (cellIndex >= flowCells.Length)
            return float.MaxValue;
            
        return flowCells[cellIndex].DistanceToObstacle;
    }
    
    private static int GridToIndex(int3 gridPos, int3 resolution)
    {
        return gridPos.x + gridPos.y * resolution.x + gridPos.z * resolution.x * resolution.y;
    }
}

// Blend weights for different flow field types
public struct FlowFieldBlendWeights
{
    public float ChaseWeight;    // How much to chase target
    public float FleeWeight;     // How much to flee from threat
    public float PatrolWeight;   // How much to follow patrol routes
    public float AmbushWeight;   // How much to use ambush tactics
    
    public static FlowFieldBlendWeights CreateChase() => new FlowFieldBlendWeights { ChaseWeight = 1f };
    public static FlowFieldBlendWeights CreateFlee() => new FlowFieldBlendWeights { FleeWeight = 1f };
    public static FlowFieldBlendWeights CreatePatrol() => new FlowFieldBlendWeights { PatrolWeight = 1f };
}