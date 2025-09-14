using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

// Job for adjusting steering behaviors based on navigation tier
[BurstCompile]
public partial struct AdjustSteeringByTierJob : IJobEntity
{
    [ReadOnly] public NavigationOctree Octree;
    [ReadOnly] public NativeArray<NavigationOctreeNode> NodesBuffer;
    [ReadOnly] public NativeArray<NavigationFieldData> FieldBuffer;

    // Direct implementation of GetNodeContaining for NativeArray
    private int GetNodeContainingDirect(float3 worldPosition, in NavigationOctree octree,
        in NativeArray<NavigationOctreeNode> nodes)
    {
        // Check if position is within octree bounds
        float3 toPos = worldPosition - octree.BoundsCenter;
        if (math.any(math.abs(toPos) > octree.BoundsHalfExtent))
            return -1;

        int currentNode = octree.RootNodeIndex;

        while (currentNode >= 0 && currentNode < nodes.Length)
        {
            NavigationOctreeNode node = nodes[currentNode];

            if (node.IsLeaf)
                return currentNode;

            // Navigate to appropriate child
            int childIndex = NavigationOctree.GetChildIndex(node.Center, worldPosition);
            currentNode = node.ChildStartIndex + childIndex;
        }

        return -1;
    }

    public void Execute(Entity entity, RefRW<SteeringBehaviors> steering, in Ship ship, in LocalToWorld transform)
    {
        float3 position = transform.Position;

        // Query octree for navigation tier at ship position
        int nodeIndex = GetNodeContainingDirect(position, Octree, NodesBuffer);

        if (nodeIndex >= 0 && nodeIndex < NodesBuffer.Length)
        {
            NavigationOctreeNode node = NodesBuffer[nodeIndex];

            // Adjust steering weights based on navigation tier
            switch (node.Tier)
            {
                case NavigationTier.Close:
                    // Close to obstacles - prioritize avoidance
                    steering.ValueRW.ObstacleAvoidanceWeight = 2.0f;
                    steering.ValueRW.FlowFieldWeight = 1.5f;
                    steering.ValueRW.SeekWeight = 0.5f;
                    break;

                case NavigationTier.Medium:
                    // Medium distance - balanced approach
                    steering.ValueRW.ObstacleAvoidanceWeight = 1.0f;
                    steering.ValueRW.FlowFieldWeight = 1.0f;
                    steering.ValueRW.SeekWeight = 1.0f;
                    break;

                case NavigationTier.Far:
                    // Far from obstacles - direct navigation
                    steering.ValueRW.ObstacleAvoidanceWeight = 0.2f;
                    steering.ValueRW.FlowFieldWeight = 0.5f;
                    steering.ValueRW.SeekWeight = 1.5f;
                    break;
            }

            // Sample distance field for additional avoidance
            if (node.FieldDataIndex >= 0 && node.FieldDataIndex < FieldBuffer.Length)
            {
                NavigationFieldData field = FieldBuffer[node.FieldDataIndex];

                // If very close to obstacle, add emergency avoidance
                if (field.SignedDistance < 2.0f)
                {
                    steering.ValueRW.EmergencyAvoidance = true;
                    steering.ValueRW.AvoidanceDirection = field.GradientField;
                }
                else
                {
                    steering.ValueRW.EmergencyAvoidance = false;
                }
            }
        }
    }
}

// Example integration showing how to use NavigationOctree with existing flow field system
[UpdateInGroup(typeof(SimulationSystemGroup))]
[UpdateAfter(typeof(NavigationOctreeBuilderSystem))]
[UpdateBefore(typeof(FlowFieldGenerationSystemOptimized))]
public partial class NavigationOctreeIntegrationSystem : SystemBase
{
    private EntityQuery _shipsQuery;
    private EntityQuery _octreeQuery;

    protected override void OnCreate()
    {
        _shipsQuery = GetEntityQuery(
            ComponentType.ReadOnly<Ship>(),
            ComponentType.ReadOnly<LocalToWorld>(),
            ComponentType.ReadWrite<SteeringBehaviors>()
        );

        _octreeQuery = GetEntityQuery(
            ComponentType.ReadOnly<NavigationOctree>(),
            ComponentType.ReadOnly<NavigationOctreeNode>(),
            ComponentType.ReadOnly<NavigationFieldData>()
        );
    }

    protected override void OnUpdate()
    {
        if (_octreeQuery.IsEmpty || _shipsQuery.IsEmpty)
            return;

        var octreeEntity = _octreeQuery.GetSingletonEntity();
        var octree = EntityManager.GetComponentData<NavigationOctree>(octreeEntity);
        var nodesBuffer = EntityManager.GetBuffer<NavigationOctreeNode>(octreeEntity);
        var fieldBuffer = EntityManager.GetBuffer<NavigationFieldData>(octreeEntity);

        // Process ships to determine their navigation tier and adjust behavior
        var adjustSteeringJob = new AdjustSteeringByTierJob
        {
            Octree = octree,
            NodesBuffer = nodesBuffer.AsNativeArray(),
            FieldBuffer = fieldBuffer.AsNativeArray()
        };

        Dependency = adjustSteeringJob.ScheduleParallel(_shipsQuery, Dependency);
    }
}

// Helper component for integration with flow field system
public struct NavigationFieldIntegration : IComponentData
{
    public Entity NavigationOctreeEntity;
    public Entity FlowFieldCollectionEntity;
    public float FieldBlendDistance; // Distance over which to blend fields
    public bool UseHierarchicalNavigation;
}

// Job for efficient batch navigation queries
[BurstCompile]
public struct HierarchicalNavigationQueryJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<float3> Positions;
    [ReadOnly] public NavigationOctree Octree;
    [ReadOnly] public NativeArray<NavigationOctreeNode> Nodes;
    [ReadOnly] public NativeArray<NavigationFieldData> Fields;

    // Flow field data
    [ReadOnly] public FlowField FlowFieldInfo;
    [ReadOnly] public NativeArray<FlowCell> FlowCells;

    // Output navigation data
    [WriteOnly] public NativeArray<float3> NavigationVelocities;
    [WriteOnly] public NativeArray<NavigationTier> NavigationTiers;
    [WriteOnly] public NativeArray<float> ObstacleDistances;

    public void Execute(int index)
    {
        float3 position = Positions[index];

        // First, query octree for coarse navigation
        int nodeIndex = GetNodeContaining(position);

        if (nodeIndex >= 0)
        {
            NavigationOctreeNode node = Nodes[nodeIndex];
            NavigationTiers[index] = node.Tier;

            // Get field data if available
            if (node.FieldDataIndex >= 0 && node.FieldDataIndex < Fields.Length)
            {
                NavigationFieldData field = Fields[node.FieldDataIndex];
                ObstacleDistances[index] = field.SignedDistance;

                // Choose navigation method based on tier
                if (node.Tier == NavigationTier.Close)
                {
                    // Use detailed flow field for close navigation
                    NavigationVelocities[index] = SampleFlowField(position);
                }
                else if (node.Tier == NavigationTier.Medium)
                {
                    // Blend octree vector field with flow field
                    float3 octreeVel = field.VectorField;
                    float3 flowVel = SampleFlowField(position);
                    NavigationVelocities[index] = math.normalize(octreeVel + flowVel);
                }
                else // Far
                {
                    // Use simple octree vector field
                    NavigationVelocities[index] = field.VectorField;
                }
            }
            else
            {
                ObstacleDistances[index] = float.MaxValue;
                NavigationVelocities[index] = float3.zero;
            }
        }
        else
        {
            // Outside octree bounds
            NavigationTiers[index] = NavigationTier.Far;
            ObstacleDistances[index] = float.MaxValue;
            NavigationVelocities[index] = float3.zero;
        }
    }

    private int GetNodeContaining(float3 position)
    {
        // Check bounds
        float3 toPos = position - Octree.BoundsCenter;
        if (math.any(math.abs(toPos) > Octree.BoundsHalfExtent))
            return -1;

        int currentNode = Octree.RootNodeIndex;

        while (currentNode >= 0 && currentNode < Nodes.Length)
        {
            NavigationOctreeNode node = Nodes[currentNode];

            if (node.IsLeaf)
                return currentNode;

            int childIndex = NavigationOctree.GetChildIndex(node.Center, position);
            currentNode = node.ChildStartIndex + childIndex;
        }

        return -1;
    }

    private float3 SampleFlowField(float3 worldPosition)
    {
        // Convert to flow field grid coordinates
        float3 localPos = worldPosition - FlowFieldInfo.BoundsMin;
        int3 gridPos = (int3)(localPos / FlowFieldInfo.CellSize);

        if (math.any(gridPos < 0) || math.any(gridPos >= FlowFieldInfo.Resolution))
            return float3.zero;

        int cellIndex = gridPos.x + gridPos.y * FlowFieldInfo.Resolution.x +
                       gridPos.z * FlowFieldInfo.Resolution.x * FlowFieldInfo.Resolution.y;

        if (cellIndex >= 0 && cellIndex < FlowCells.Length)
        {
            return FlowCells[cellIndex].Velocity;
        }

        return float3.zero;
    }
}

// Extension to SteeringBehaviors for octree integration
public partial struct SteeringBehaviors
{
    public bool EmergencyAvoidance;
    public float3 AvoidanceDirection;
}