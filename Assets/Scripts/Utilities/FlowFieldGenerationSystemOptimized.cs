using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

// Optimized flow field generation system with proper octree traversal and parallel processing
[BurstCompile]
[UpdateInGroup(typeof(InitializationSystemGroup))]
public partial struct FlowFieldGenerationSystemOptimized : ISystem
{
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        state.RequireForUpdate<SpatialDatabaseSingleton>();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        SpatialDatabaseSingleton spatialDb = SystemAPI.GetSingleton<SpatialDatabaseSingleton>();
        if (spatialDb.TargetablesSpatialDatabase == Entity.Null)
            return;

        // Get octree data for obstacle sampling
        DynamicBuffer<OctreeNode> nodes = state.EntityManager.GetBuffer<OctreeNode>(spatialDb.TargetablesSpatialDatabase);
        DynamicBuffer<SpatialObject> objects = state.EntityManager.GetBuffer<SpatialObject>(spatialDb.TargetablesSpatialDatabase);

        float currentTime = (float)SystemAPI.Time.ElapsedTime;
        var transformLookup = SystemAPI.GetComponentLookup<LocalTransform>(true);

        // Generate flow fields for all flow field entities that need updates
        foreach ((RefRO<FlowField> flowField, DynamicBuffer<FlowCell> cells, RefRW<FlowFieldGenerationControl> control, Entity entity)
                 in SystemAPI.Query<RefRO<FlowField>, DynamicBuffer<FlowCell>, RefRW<FlowFieldGenerationControl>>().WithEntityAccess())
        {
            // Check if this flow field needs regeneration
            if (ShouldRegenerateFlowField(control.ValueRO, currentTime, transformLookup))
            {
                GenerateFlowFieldOptimized(flowField.ValueRO, cells, nodes, objects, control.ValueRW, transformLookup);

                // Update control component
                control.ValueRW.LastGenerationTime = currentTime;
                control.ValueRW.NeedsRegeneration = false;

                // Store target positions for change detection
                UpdateTargetPositions(ref control.ValueRW, transformLookup);
            }
        }
    }

    private bool ShouldRegenerateFlowField(in FlowFieldGenerationControl control, float currentTime, ComponentLookup<LocalTransform> transformLookup)
    {
        if (control.NeedsRegeneration) return true;
        if (currentTime - control.LastGenerationTime > control.RegenerationInterval) return true;

        if (control.RegenerateWhenTargetMoves)
        {
            const float MOVEMENT_THRESHOLD = 5f;

            if (HasTargetMoved(control.ChaseTarget, control.LastChaseTargetPosition, MOVEMENT_THRESHOLD, transformLookup))
                return true;

            if (HasTargetMoved(control.FleeFromTarget, control.LastFleeTargetPosition, MOVEMENT_THRESHOLD, transformLookup))
                return true;
        }

        return false;
    }

    private bool HasTargetMoved(Entity target, float3 lastPosition, float threshold, ComponentLookup<LocalTransform> transformLookup)
    {
        if (target != Entity.Null && transformLookup.HasComponent(target))
        {
            float3 currentPos = transformLookup[target].Position;
            return math.distance(currentPos, lastPosition) > threshold;
        }
        return false;
    }

    private void UpdateTargetPositions(ref FlowFieldGenerationControl control, ComponentLookup<LocalTransform> transformLookup)
    {
        if (control.ChaseTarget != Entity.Null && transformLookup.HasComponent(control.ChaseTarget))
        {
            control.LastChaseTargetPosition = transformLookup[control.ChaseTarget].Position;
        }
        if (control.FleeFromTarget != Entity.Null && transformLookup.HasComponent(control.FleeFromTarget))
        {
            control.LastFleeTargetPosition = transformLookup[control.FleeFromTarget].Position;
        }
    }

    private void GenerateFlowFieldOptimized(
        in FlowField flowField,
        DynamicBuffer<FlowCell> cells,
        in DynamicBuffer<OctreeNode> nodes,
        in DynamicBuffer<SpatialObject> objects,
        FlowFieldGenerationControl control,
        ComponentLookup<LocalTransform> transformLookup)
    {
        int totalCells = flowField.Resolution.x * flowField.Resolution.y * flowField.Resolution.z;
        cells.ResizeUninitialized(totalCells);

        // Step 1: Generate SDF using parallel optimized job
        GenerateSDFParallel(flowField, cells, nodes, objects);

        // Step 2: Generate cost field
        NativeArray<float> costField = new NativeArray<float>(totalCells, Allocator.Temp);
        GenerateCostField(flowField, cells, costField);

        // Step 3: Generate integration field using optimized Dijkstra
        NativeArray<float> integrationField = new NativeArray<float>(totalCells, Allocator.Temp);
        GenerateIntegrationField(flowField, costField, integrationField, control, transformLookup);

        // Step 4: Generate flow vectors from gradients
        GenerateFlowVectors(flowField, integrationField, cells);

        costField.Dispose();
        integrationField.Dispose();
    }

    private void GenerateSDFParallel(
        in FlowField flowField,
        DynamicBuffer<FlowCell> cells,
        in DynamicBuffer<OctreeNode> nodes,
        in DynamicBuffer<SpatialObject> objects)
    {
        int totalCells = flowField.Resolution.x * flowField.Resolution.y * flowField.Resolution.z;

        // Use parallel job for SDF generation - provides significant speedup
        ParallelSDFGenerationJobOptimized sdfJob = new ParallelSDFGenerationJobOptimized
        {
            FlowField = flowField,
            Nodes = nodes,
            Objects = objects,
            Cells = cells
        };

        sdfJob.Schedule(totalCells, 64).Complete(); // Process 64 cells per batch
    }

    private void GenerateCostField(
        in FlowField flowField,
        DynamicBuffer<FlowCell> cells,
        NativeArray<float> costField)
    {
        for (int i = 0; i < costField.Length; i++)
        {
            FlowCell cell = cells[i];
            float baseCost = 1f;
            float obstacleCost = math.max(0f, 5f - cell.DistanceToObstacle);
            costField[i] = baseCost + obstacleCost;
        }
    }

    private void GenerateIntegrationField(
        in FlowField flowField,
        NativeArray<float> costField,
        NativeArray<float> integrationField,
        FlowFieldGenerationControl control,
        ComponentLookup<LocalTransform> transformLookup)
    {
        // Initialize with maximum values
        for (int i = 0; i < integrationField.Length; i++)
        {
            integrationField[i] = float.MaxValue;
        }

        // Set goals based on flow field type
        SetFlowFieldGoals(flowField, integrationField, control, transformLookup);

        // Dijkstra's algorithm with 26-neighbor connectivity
        NativeQueue<int> openSet = new NativeQueue<int>(Allocator.Temp);

        // Add all goal cells to the open set
        for (int i = 0; i < integrationField.Length; i++)
        {
            if (integrationField[i] == 0f)
            {
                openSet.Enqueue(i);
            }
        }

        // Process queue
        while (!openSet.IsEmpty())
        {
            int currentIndex = openSet.Dequeue();
            int3 currentGrid = IndexToGrid(currentIndex, flowField.Resolution);
            float currentDistance = integrationField[currentIndex];

            // Check all 26 neighbors for full 3D movement
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        if (dx == 0 && dy == 0 && dz == 0) continue;

                        int3 neighborGrid = currentGrid + new int3(dx, dy, dz);

                        if (math.any(neighborGrid < 0) || math.any(neighborGrid >= flowField.Resolution))
                            continue;

                        int neighborIndex = GridToIndex(neighborGrid, flowField.Resolution);

                        // Calculate 3D movement cost
                        float3 offset = new float3(dx, dy, dz);
                        float movementCost = math.length(offset);
                        float newDistance = currentDistance + costField[neighborIndex] * movementCost;

                        if (newDistance < integrationField[neighborIndex])
                        {
                            integrationField[neighborIndex] = newDistance;
                            openSet.Enqueue(neighborIndex);
                        }
                    }
                }
            }
        }

        openSet.Dispose();
    }

    private void GenerateFlowVectors(
        in FlowField flowField,
        NativeArray<float> integrationField,
        DynamicBuffer<FlowCell> cells)
    {
        for (int z = 0; z < flowField.Resolution.z; z++)
        {
            for (int y = 0; y < flowField.Resolution.y; y++)
            {
                for (int x = 0; x < flowField.Resolution.x; x++)
                {
                    int cellIndex = GridToIndex(new int3(x, y, z), flowField.Resolution);
                    float3 gradient = CalculateGradient(new int3(x, y, z), flowField, integrationField);

                    FlowCell cell = cells[cellIndex];
                    cell.Velocity = math.normalize(-gradient);
                    cells[cellIndex] = cell;
                }
            }
        }
    }

    private void SetFlowFieldGoals(in FlowField flowField, NativeArray<float> integrationField,
        FlowFieldGenerationControl control, ComponentLookup<LocalTransform> transformLookup)
    {
        switch (flowField.FieldType)
        {
            case FlowFieldType.Chase:
                SetChaseGoals(flowField, integrationField, control, transformLookup);
                break;
            case FlowFieldType.Flee:
                SetFleeGoals(flowField, integrationField);
                break;
            case FlowFieldType.Patrol:
                SetPatrolGoals(flowField, integrationField);
                break;
            case FlowFieldType.Ambush:
                SetAmbushGoals(flowField, integrationField, control, transformLookup);
                break;
        }
    }

    private void SetChaseGoals(in FlowField flowField, NativeArray<float> integrationField,
        FlowFieldGenerationControl control, ComponentLookup<LocalTransform> transformLookup)
    {
        if (control.ChaseTarget != Entity.Null && transformLookup.HasComponent(control.ChaseTarget))
        {
            float3 targetWorldPos = transformLookup[control.ChaseTarget].Position;
            float3 localPos = targetWorldPos - flowField.BoundsMin;
            int3 gridPos = (int3)(localPos / flowField.CellSize);
            gridPos = math.clamp(gridPos, int3.zero, flowField.Resolution - 1);

            int goalIndex = GridToIndex(gridPos, flowField.Resolution);
            integrationField[goalIndex] = 0f;
        }
        else
        {
            int3 goalGrid = flowField.Resolution / 2;
            int goalIndex = GridToIndex(goalGrid, flowField.Resolution);
            integrationField[goalIndex] = 0f;
        }
    }

    private void SetFleeGoals(in FlowField flowField, NativeArray<float> integrationField)
    {
        // Set goals on all boundary faces for 3D escape routes
        for (int y = 0; y < flowField.Resolution.y; y++)
        {
            for (int z = 0; z < flowField.Resolution.z; z++)
            {
                integrationField[GridToIndex(new int3(0, y, z), flowField.Resolution)] = 0f;
                integrationField[GridToIndex(new int3(flowField.Resolution.x - 1, y, z), flowField.Resolution)] = 0f;
            }
        }

        for (int x = 0; x < flowField.Resolution.x; x++)
        {
            for (int z = 0; z < flowField.Resolution.z; z++)
            {
                integrationField[GridToIndex(new int3(x, 0, z), flowField.Resolution)] = 0f;
                integrationField[GridToIndex(new int3(x, flowField.Resolution.y - 1, z), flowField.Resolution)] = 0f;
            }
        }

        for (int x = 0; x < flowField.Resolution.x; x++)
        {
            for (int y = 0; y < flowField.Resolution.y; y++)
            {
                integrationField[GridToIndex(new int3(x, y, 0), flowField.Resolution)] = 0f;
                integrationField[GridToIndex(new int3(x, y, flowField.Resolution.z - 1), flowField.Resolution)] = 0f;
            }
        }
    }

    private void SetPatrolGoals(in FlowField flowField, NativeArray<float> integrationField)
    {
        int3 center = flowField.Resolution / 2;
        float patrolRadius = math.min(flowField.Resolution.x, math.min(flowField.Resolution.y, flowField.Resolution.z)) * 0.3f;

        for (int i = 0; i < 16; i++)
        {
            float angle = (i / 16f) * 2f * math.PI;
            int3 patrolPoint = center + new int3(
                (int)(math.cos(angle) * patrolRadius),
                0,
                (int)(math.sin(angle) * patrolRadius)
            );

            if (math.all(patrolPoint >= 0) && math.all(patrolPoint < flowField.Resolution))
            {
                int goalIndex = GridToIndex(patrolPoint, flowField.Resolution);
                integrationField[goalIndex] = 0f;
            }
        }
    }

    private void SetAmbushGoals(in FlowField flowField, NativeArray<float> integrationField,
        FlowFieldGenerationControl control, ComponentLookup<LocalTransform> transformLookup)
    {
        if (control.ChaseTarget != Entity.Null && transformLookup.HasComponent(control.ChaseTarget))
        {
            float3 targetWorldPos = transformLookup[control.ChaseTarget].Position;
            float3 localPos = targetWorldPos - flowField.BoundsMin;
            int3 centerGrid = (int3)(localPos / flowField.CellSize);
            centerGrid = math.clamp(centerGrid, int3.zero, flowField.Resolution - 1);

            for (int yOffset = -1; yOffset <= 1; yOffset++)
            {
                int3 ambushPoint = centerGrid + new int3(0, yOffset * 3, 0);
                ambushPoint = math.clamp(ambushPoint, int3.zero, flowField.Resolution - 1);

                int goalIndex = GridToIndex(ambushPoint, flowField.Resolution);
                integrationField[goalIndex] = 0f;
            }
        }
        else
        {
            int3 center = flowField.Resolution / 2;
            for (int yOffset = -1; yOffset <= 1; yOffset++)
            {
                int3 ambushPoint = center + new int3(0, yOffset * (flowField.Resolution.y / 4), 0);
                if (math.all(ambushPoint >= 0) && math.all(ambushPoint < flowField.Resolution))
                {
                    int goalIndex = GridToIndex(ambushPoint, flowField.Resolution);
                    integrationField[goalIndex] = 0f;
                }
            }
        }
    }

    private float3 CalculateGradient(int3 gridPos, in FlowField flowField, NativeArray<float> integrationField)
    {
        float3 gradient = float3.zero;

        // X-axis gradient
        if (gridPos.x > 0 && gridPos.x < flowField.Resolution.x - 1)
        {
            int rightIndex = GridToIndex(gridPos + new int3(1, 0, 0), flowField.Resolution);
            int leftIndex = GridToIndex(gridPos + new int3(-1, 0, 0), flowField.Resolution);
            gradient.x = (integrationField[rightIndex] - integrationField[leftIndex]) * 0.5f;
        }
        else if (gridPos.x == 0)
        {
            int centerIndex = GridToIndex(gridPos, flowField.Resolution);
            int rightIndex = GridToIndex(gridPos + new int3(1, 0, 0), flowField.Resolution);
            gradient.x = integrationField[rightIndex] - integrationField[centerIndex];
        }
        else
        {
            int centerIndex = GridToIndex(gridPos, flowField.Resolution);
            int leftIndex = GridToIndex(gridPos + new int3(-1, 0, 0), flowField.Resolution);
            gradient.x = integrationField[centerIndex] - integrationField[leftIndex];
        }

        // Y-axis gradient (critical for 3D flight)
        if (gridPos.y > 0 && gridPos.y < flowField.Resolution.y - 1)
        {
            int upIndex = GridToIndex(gridPos + new int3(0, 1, 0), flowField.Resolution);
            int downIndex = GridToIndex(gridPos + new int3(0, -1, 0), flowField.Resolution);
            gradient.y = (integrationField[upIndex] - integrationField[downIndex]) * 0.5f;
        }
        else if (gridPos.y == 0)
        {
            int centerIndex = GridToIndex(gridPos, flowField.Resolution);
            int upIndex = GridToIndex(gridPos + new int3(0, 1, 0), flowField.Resolution);
            gradient.y = integrationField[upIndex] - integrationField[centerIndex];
        }
        else
        {
            int centerIndex = GridToIndex(gridPos, flowField.Resolution);
            int downIndex = GridToIndex(gridPos + new int3(0, -1, 0), flowField.Resolution);
            gradient.y = integrationField[centerIndex] - integrationField[downIndex];
        }

        // Z-axis gradient
        if (gridPos.z > 0 && gridPos.z < flowField.Resolution.z - 1)
        {
            int forwardIndex = GridToIndex(gridPos + new int3(0, 0, 1), flowField.Resolution);
            int backIndex = GridToIndex(gridPos + new int3(0, 0, -1), flowField.Resolution);
            gradient.z = (integrationField[forwardIndex] - integrationField[backIndex]) * 0.5f;
        }
        else if (gridPos.z == 0)
        {
            int centerIndex = GridToIndex(gridPos, flowField.Resolution);
            int forwardIndex = GridToIndex(gridPos + new int3(0, 0, 1), flowField.Resolution);
            gradient.z = integrationField[forwardIndex] - integrationField[centerIndex];
        }
        else
        {
            int centerIndex = GridToIndex(gridPos, flowField.Resolution);
            int backIndex = GridToIndex(gridPos + new int3(0, 0, -1), flowField.Resolution);
            gradient.z = integrationField[centerIndex] - integrationField[backIndex];
        }

        return gradient;
    }

    // Helper functions
    private int GridToIndex(int3 gridPos, int3 resolution)
    {
        return gridPos.x + gridPos.y * resolution.x + gridPos.z * resolution.x * resolution.y;
    }

    private int3 IndexToGrid(int index, int3 resolution)
    {
        int x = index % resolution.x;
        int y = (index / resolution.x) % resolution.y;
        int z = index / (resolution.x * resolution.y);
        return new int3(x, y, z);
    }

    private float3 GridToWorldPos(int3 gridPos, in FlowField flowField)
    {
        return flowField.BoundsMin + new float3(gridPos) * flowField.CellSize;
    }
}

// Optimized parallel SDF generation job
[BurstCompile]
public struct ParallelSDFGenerationJobOptimized : IJobParallelFor
{
    [ReadOnly] public FlowField FlowField;
    [ReadOnly] public DynamicBuffer<OctreeNode> Nodes;
    [ReadOnly] public DynamicBuffer<SpatialObject> Objects;

    [NativeDisableParallelForRestriction]
    public DynamicBuffer<FlowCell> Cells;

    public void Execute(int index)
    {
        int3 gridPos = IndexToGrid(index, FlowField.Resolution);
        float3 worldPos = GridToWorldPos(gridPos, FlowField);

        float sdfValue = SampleNearestObstacleOptimized(worldPos, Nodes, Objects);

        FlowCell cell = Cells[index];
        cell.DistanceToObstacle = sdfValue;
        Cells[index] = cell;
    }

    private float SampleNearestObstacleOptimized(float3 worldPos, in DynamicBuffer<OctreeNode> nodes, in DynamicBuffer<SpatialObject> objects)
    {
        if (nodes.Length == 0)
            return float.MaxValue;

        float minDistance = float.MaxValue;
        float maxSearchRadius = 20f;

        TraverseOctreeForDistance(worldPos, maxSearchRadius, 0, nodes, objects, ref minDistance);
        return minDistance;
    }

    private void TraverseOctreeForDistance(
        float3 queryPoint,
        float maxSearchRadius,
        int nodeIndex,
        in DynamicBuffer<OctreeNode> nodes,
        in DynamicBuffer<SpatialObject> objects,
        ref float minDistance)
    {
        if (nodeIndex >= nodes.Length) return;

        OctreeNode node = nodes[nodeIndex];
        float nodeDistance = DistanceToOctreeNode(queryPoint, node);

        if (nodeDistance > maxSearchRadius || nodeDistance > minDistance) return;

        if (node.IsLeaf)
        {
            for (int i = node.ObjectStartIndex; i < node.ObjectStartIndex + node.ObjectCount; i++)
            {
                if (i >= objects.Length) break;

                SpatialObject obj = objects[i];
                float distance = CalculateDistanceToSpatialObject(queryPoint, obj);
                minDistance = math.min(minDistance, distance);

                if (minDistance < 0.1f) return; // Early exit for very close objects
            }
        }
        else
        {
            // Recursively check children
            for (int i = 0; i < 8; i++)
            {
                int childIndex = node.ChildStartIndex + i;
                if (childIndex < nodes.Length)
                {
                    TraverseOctreeForDistance(queryPoint, maxSearchRadius, childIndex, nodes, objects, ref minDistance);
                }
            }
        }
    }

    private float DistanceToOctreeNode(float3 point, OctreeNode node)
    {
        float3 offset = math.abs(point - node.Center);
        float3 distance = offset - new float3(node.HalfExtent);
        return math.length(math.max(distance, 0f)) + math.min(math.max(distance.x, math.max(distance.y, distance.z)), 0f);
    }

    private float CalculateDistanceToSpatialObject(float3 point, SpatialObject obj)
    {
        return obj.ShapeType switch
        {
            CollisionShapeType.Point => math.length(obj.Position - point),
            CollisionShapeType.Sphere => math.length(obj.Position - point) - obj.HalfExtents.x,
            CollisionShapeType.AABB => DistanceToAABB(point, obj.GetMinBounds(), obj.GetMaxBounds()),
            _ => math.length(obj.Position - point)
        };
    }

    private float DistanceToAABB(float3 point, float3 aabbMin, float3 aabbMax)
    {
        float3 center = (aabbMin + aabbMax) * 0.5f;
        float3 extents = (aabbMax - aabbMin) * 0.5f;
        float3 offset = math.abs(point - center);
        float3 distance = offset - extents;
        return math.length(math.max(distance, 0f)) + math.min(math.max(distance.x, math.max(distance.y, distance.z)), 0f);
    }

    private int3 IndexToGrid(int index, int3 resolution)
    {
        int x = index % resolution.x;
        int y = (index / resolution.x) % resolution.y;
        int z = index / (resolution.x * resolution.y);
        return new int3(x, y, z);
    }

    private float3 GridToWorldPos(int3 gridPos, FlowField flowField)
    {
        return flowField.BoundsMin + new float3(gridPos) * flowField.CellSize;
    }
}