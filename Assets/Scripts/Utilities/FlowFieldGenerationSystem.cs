using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

// Flow field generation system - creates the actual flow field data
[BurstCompile]
[UpdateInGroup(typeof(InitializationSystemGroup))]
public partial struct FlowFieldGenerationSystem : ISystem
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
                GenerateFlowField(flowField.ValueRO, cells, nodes, objects, control.ValueRW, transformLookup);
                
                // Update control component
                control.ValueRW.LastGenerationTime = currentTime;
                control.ValueRW.NeedsRegeneration = false;
                
                // Store target positions for change detection
                if (control.ValueRO.ChaseTarget != Entity.Null && transformLookup.HasComponent(control.ValueRO.ChaseTarget))
                {
                    control.ValueRW.LastChaseTargetPosition = transformLookup[control.ValueRO.ChaseTarget].Position;
                }
                if (control.ValueRO.FleeFromTarget != Entity.Null && transformLookup.HasComponent(control.ValueRO.FleeFromTarget))
                {
                    control.ValueRW.LastFleeTargetPosition = transformLookup[control.ValueRO.FleeFromTarget].Position;
                }
            }
        }
    }
    
    private bool ShouldRegenerateFlowField(in FlowFieldGenerationControl control, float currentTime, ComponentLookup<LocalTransform> transformLookup)
    {
        // Force regeneration if needed
        if (control.NeedsRegeneration)
            return true;
            
        // Check time-based regeneration
        if (currentTime - control.LastGenerationTime > control.RegenerationInterval)
            return true;
            
        // Check if targets have moved significantly
        if (control.RegenerateWhenTargetMoves)
        {
            const float MOVEMENT_THRESHOLD = 5f; // Regenerate if target moves 5+ units
            
            if (control.ChaseTarget != Entity.Null && transformLookup.HasComponent(control.ChaseTarget))
            {
                float3 currentPos = transformLookup[control.ChaseTarget].Position;
                float distance = math.distance(currentPos, control.LastChaseTargetPosition);
                if (distance > MOVEMENT_THRESHOLD)
                    return true;
            }
            
            if (control.FleeFromTarget != Entity.Null && transformLookup.HasComponent(control.FleeFromTarget))
            {
                float3 currentPos = transformLookup[control.FleeFromTarget].Position;
                float distance = math.distance(currentPos, control.LastFleeTargetPosition);
                if (distance > MOVEMENT_THRESHOLD)
                    return true;
            }
        }
        
        return false;
    }
    
    private void GenerateFlowField(
        in FlowField flowField, 
        DynamicBuffer<FlowCell> cells,
        in DynamicBuffer<OctreeNode> nodes,
        in DynamicBuffer<SpatialObject> objects,
        FlowFieldGenerationControl control,
        ComponentLookup<LocalTransform> transformLookup)
    {
        int totalCells = flowField.Resolution.x * flowField.Resolution.y * flowField.Resolution.z;
        
        // Ensure buffer has correct size
        cells.ResizeUninitialized(totalCells);
        
        // Step 1: Generate SDF (distance to obstacles) for each cell
        GenerateSDF(flowField, cells, nodes, objects);
        
        // Step 2: Generate cost field based on SDF and field type
        NativeArray<float> costField = new NativeArray<float>(totalCells, Allocator.Temp);
        GenerateCostField(flowField, cells, costField);
        
        // Step 3: Generate integration field using Dijkstra's algorithm
        NativeArray<float> integrationField = new NativeArray<float>(totalCells, Allocator.Temp);
        GenerateIntegrationField(flowField, costField, integrationField, control, transformLookup);
        
        // Step 4: Generate flow vectors from integration field gradients
        GenerateFlowVectors(flowField, integrationField, cells);
        
        costField.Dispose();
        integrationField.Dispose();
    }
    
    private void GenerateSDF(
        in FlowField flowField,
        DynamicBuffer<FlowCell> cells,
        in DynamicBuffer<OctreeNode> nodes,
        in DynamicBuffer<SpatialObject> objects)
    {
        int totalCells = flowField.Resolution.x * flowField.Resolution.y * flowField.Resolution.z;
        
        // Use parallel job for SDF generation - much faster!
        ParallelSDFGenerationJob sdfJob = new ParallelSDFGenerationJob
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
            
            // Base cost depends on field type
            float baseCost = 1f;
            
            // Higher cost near obstacles
            float obstacleCost = math.max(0f, 5f - cell.DistanceToObstacle); // Cost increases as we get closer
            
            costField[i] = baseCost + obstacleCost;
        }
    }
    
    private void GenerateIntegrationField(
        in FlowField flowField,
        NativeArray<float> costField,
        NativeArray<float> integrationField,
        in FlowFieldGenerationControl control,
        ComponentLookup<LocalTransform> transformLookup)
    {
        // Initialize with maximum values
        for (int i = 0; i < integrationField.Length; i++)
        {
            integrationField[i] = float.MaxValue;
        }
        
        // Set goals based on flow field type for proper 3D flight
        SetFlowFieldGoals(flowField, integrationField, control, transformLookup);
        
        // Priority queue for Dijkstra's algorithm (simplified breadth-first for now)
        NativeQueue<int> openSet = new NativeQueue<int>(Allocator.Temp);
        
        // Add all goal cells to the open set
        for (int i = 0; i < integrationField.Length; i++)
        {
            if (integrationField[i] == 0f) // This is a goal cell
            {
                openSet.Enqueue(i);
            }
        }
        
        // Dijkstra's algorithm
        while (!openSet.IsEmpty())
        {
            int currentIndex = openSet.Dequeue();
            int3 currentGrid = IndexToGrid(currentIndex, flowField.Resolution);
            float currentDistance = integrationField[currentIndex];
            
            // Check all 26 neighbors for full 3D movement (6 cardinal + 12 face diagonal + 8 corner diagonal)
            for (int dx = -1; dx <= 1; dx++)
            {
                for (int dy = -1; dy <= 1; dy++)
                {
                    for (int dz = -1; dz <= 1; dz++)
                    {
                        if (dx == 0 && dy == 0 && dz == 0) continue; // Skip center cell
                        
                        int3 neighborGrid = currentGrid + new int3(dx, dy, dz);
                        
                        // Bounds check
                        if (math.any(neighborGrid < 0) || math.any(neighborGrid >= flowField.Resolution))
                            continue;
                            
                        int neighborIndex = GridToIndex(neighborGrid, flowField.Resolution);
                        
                        // Calculate actual 3D distance cost (diagonal movement costs more)
                        float3 offset = new float3(dx, dy, dz);
                        float movementCost = math.length(offset); // 1.0 for cardinal, ~1.41 for face diagonal, ~1.73 for corner diagonal
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
                    
                    // Calculate gradient from integration field
                    float3 gradient = CalculateGradient(new int3(x, y, z), flowField, integrationField);
                    
                    FlowCell cell = cells[cellIndex];
                    cell.Velocity = math.normalize(-gradient); // Flow toward lower integration values
                    cells[cellIndex] = cell;
                }
            }
        }
    }
    
    // Optimized obstacle sampling using octree traversal
    private float SampleNearestObstacle(float3 worldPos, in DynamicBuffer<OctreeNode> nodes, in DynamicBuffer<SpatialObject> objects)
    {
        if (nodes.Length == 0)
            return float.MaxValue;
            
        float minDistance = float.MaxValue;
        float maxSearchRadius = 20f; // Limit search radius for performance
        
        // Use octree traversal instead of brute force
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
        if (nodeIndex >= nodes.Length)
            return;
            
        OctreeNode node = nodes[nodeIndex];
        
        // Early exit if node is too far away
        float nodeDistance = DistanceToOctreeNode(queryPoint, node);
        if (nodeDistance > maxSearchRadius || nodeDistance > minDistance)
            return;
            
        if (node.IsLeaf)
        {
            // Check all objects in this leaf
            for (int i = node.ObjectStartIndex; i < node.ObjectStartIndex + node.ObjectCount; i++)
            {
                if (i >= objects.Length) break;
                
                SpatialObject obj = objects[i];
                float distance = CalculateDistanceToSpatialObject(queryPoint, obj);
                minDistance = math.min(minDistance, distance);
                
                // Early exit if we're very close
                if (minDistance < 0.1f)
                    return;
            }
        }
        else
        {
            // For branch nodes, sort children by distance and traverse closest first
            NativeArray<ChildDistance> childDistances = new NativeArray<ChildDistance>(8, Allocator.Temp);
            int validChildren = 0;
            
            for (int i = 0; i < 8; i++)
            {
                int childIndex = node.ChildStartIndex + i;
                if (childIndex < nodes.Length)
                {
                    OctreeNode childNode = nodes[childIndex];
                    float distance = DistanceToOctreeNode(queryPoint, childNode);
                    
                    if (distance <= maxSearchRadius && distance <= minDistance)
                    {
                        childDistances[validChildren] = new ChildDistance { Index = childIndex, Distance = distance };
                        validChildren++;
                    }
                }
            }
            
            // Sort children by distance (simple bubble sort for small arrays)
            for (int i = 0; i < validChildren - 1; i++)
            {
                for (int j = 0; j < validChildren - i - 1; j++)
                {
                    if (childDistances[j].Distance > childDistances[j + 1].Distance)
                    {
                        ChildDistance temp = childDistances[j];
                        childDistances[j] = childDistances[j + 1];
                        childDistances[j + 1] = temp;
                    }
                }
            }
            
            // Traverse children in order of distance (closest first)
            for (int i = 0; i < validChildren; i++)
            {
                // Check if this child can possibly contain a closer obstacle
                if (childDistances[i].Distance > minDistance)
                    break; // All remaining children are farther away
                    
                TraverseOctreeForDistance(queryPoint, maxSearchRadius, childDistances[i].Index, nodes, objects, ref minDistance);
            }
            
            childDistances.Dispose();
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
            CollisionShapeType.Sphere => math.length(obj.Position - point) - obj.HalfExtents.x, // x is radius for spheres
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
    
    private struct ChildDistance
    {
        public int Index;
        public float Distance;
    }

    private void SetFlowFieldGoals(in FlowField flowField, NativeArray<float> integrationField,
        in FlowFieldGenerationControl control, ComponentLookup<LocalTransform> transformLookup)
    {
        // Set goal cells based on flow field type
        switch (flowField.FieldType)
        {
            case FlowFieldType.Chase:
                if (control.ChaseTarget != Entity.Null && transformLookup.HasComponent(control.ChaseTarget))
                {
                    float3 targetPos = transformLookup[control.ChaseTarget].Position;
                    int3 gridPos = WorldToGrid(targetPos, flowField);
                    if (IsValidGridPos(gridPos, flowField.Resolution))
                    {
                        int index = GridToIndex(gridPos, flowField.Resolution);
                        integrationField[index] = 0f;
                    }
                }
                break;

            case FlowFieldType.Flee:
                if (control.FleeFromTarget != Entity.Null && transformLookup.HasComponent(control.FleeFromTarget))
                {
                    // For flee, set goals at the edges of the field
                    SetEdgeGoals(integrationField, flowField.Resolution);
                }
                break;

            case FlowFieldType.Patrol:
                // Set multiple patrol points as goals
                SetPatrolGoals(integrationField, flowField, control);
                break;

            case FlowFieldType.Ambush:
                // Set ambush positions around target
                SetAmbushGoals(integrationField, flowField, control, transformLookup);
                break;
        }
    }

    private void SetEdgeGoals(NativeArray<float> integrationField, int3 resolution)
    {
        // Set goals at the edges of the field for flee behavior
        for (int y = 0; y < resolution.y; y++)
        {
            for (int z = 0; z < resolution.z; z++)
            {
                // X edges
                integrationField[GridToIndex(new int3(0, y, z), resolution)] = 0f;
                integrationField[GridToIndex(new int3(resolution.x - 1, y, z), resolution)] = 0f;
            }
        }
    }

    private void SetPatrolGoals(NativeArray<float> integrationField, in FlowField flowField,
        in FlowFieldGenerationControl control)
    {
        // Simple patrol pattern - set corners as goals
        int3 res = flowField.Resolution;
        integrationField[GridToIndex(new int3(0, res.y / 2, 0), res)] = 0f;
        integrationField[GridToIndex(new int3(res.x - 1, res.y / 2, 0), res)] = 0f;
        integrationField[GridToIndex(new int3(0, res.y / 2, res.z - 1), res)] = 0f;
        integrationField[GridToIndex(new int3(res.x - 1, res.y / 2, res.z - 1), res)] = 0f;
    }

    private void SetAmbushGoals(NativeArray<float> integrationField, in FlowField flowField,
        in FlowFieldGenerationControl control, ComponentLookup<LocalTransform> transformLookup)
    {
        // Use ChaseTarget as the target to ambush
        if (control.ChaseTarget != Entity.Null && transformLookup.HasComponent(control.ChaseTarget))
        {
            float3 targetPos = transformLookup[control.ChaseTarget].Position;
            int3 centerGrid = WorldToGrid(targetPos, flowField);

            // Set ambush positions around target
            float ambushRadius = 10f;
            int gridRadius = (int)(ambushRadius / flowField.CellSize);

            for (int dx = -gridRadius; dx <= gridRadius; dx += gridRadius)
            {
                for (int dz = -gridRadius; dz <= gridRadius; dz += gridRadius)
                {
                    if (dx == 0 && dz == 0) continue;

                    int3 ambushPos = centerGrid + new int3(dx, 0, dz);
                    if (IsValidGridPos(ambushPos, flowField.Resolution))
                    {
                        integrationField[GridToIndex(ambushPos, flowField.Resolution)] = 0f;
                    }
                }
            }
        }
    }

    private int3 IndexToGrid(int index, int3 resolution)
    {
        int x = index % resolution.x;
        int y = (index / resolution.x) % resolution.y;
        int z = index / (resolution.x * resolution.y);
        return new int3(x, y, z);
    }

    private int GridToIndex(int3 gridPos, int3 resolution)
    {
        return gridPos.x + gridPos.y * resolution.x + gridPos.z * resolution.x * resolution.y;
    }

    private int3 WorldToGrid(float3 worldPos, in FlowField flowField)
    {
        float3 localPos = worldPos - flowField.BoundsMin;
        return (int3)(localPos / flowField.CellSize);
    }

    private bool IsValidGridPos(int3 gridPos, int3 resolution)
    {
        return math.all(gridPos >= 0) && math.all(gridPos < resolution);
    }

    private float3 CalculateGradient(int3 gridPos, in FlowField flowField, NativeArray<float> integrationField)
    {
        int3 resolution = flowField.Resolution;
        float3 gradient = float3.zero;

        // Sample neighbors for gradient
        for (int i = 0; i < 6; i++)
        {
            int3 neighborPos = gridPos + GetNeighborOffset(i);
            if (IsValidGridPos(neighborPos, resolution))
            {
                int neighborIndex = GridToIndex(neighborPos, resolution);
                float neighborValue = integrationField[neighborIndex];

                if (!math.isinf(neighborValue))
                {
                    float3 direction = math.normalize(new float3(GetNeighborOffset(i)));
                    gradient += direction * (1f / (neighborValue + 1f));
                }
            }
        }

        return math.normalizesafe(gradient);
    }

    private int3 GetNeighborOffset(int index)
    {
        switch (index)
        {
            case 0: return new int3(1, 0, 0);
            case 1: return new int3(-1, 0, 0);
            case 2: return new int3(0, 1, 0);
            case 3: return new int3(0, -1, 0);
            case 4: return new int3(0, 0, 1);
            case 5: return new int3(0, 0, -1);
            default: return int3.zero;
        }
    }
}

// Parallel job for SDF generation - provides significant speedup
[BurstCompile]
public struct ParallelSDFGenerationJob : IJobParallelFor
{
    [ReadOnly] public FlowField FlowField;
    [ReadOnly] public DynamicBuffer<OctreeNode> Nodes;
    [ReadOnly] public DynamicBuffer<SpatialObject> Objects;
    
    [NativeDisableParallelForRestriction]
    public DynamicBuffer<FlowCell> Cells;
    
    public void Execute(int index)
    {
        // Convert 1D index to 3D grid coordinates
        int x = index % FlowField.Resolution.x;
        int y = (index / FlowField.Resolution.x) % FlowField.Resolution.y;
        int z = index / (FlowField.Resolution.x * FlowField.Resolution.y);
        int3 gridPos = new int3(x, y, z);
        float3 worldPos = GridToWorldPos(gridPos, FlowField);

        // Use optimized octree sampling
        float sdfValue = SampleNearestObstacleOptimized(worldPos, Nodes, Objects);

        FlowCell cell = Cells[index];
        cell.DistanceToObstacle = sdfValue;
        Cells[index] = cell;
    }
    
    private float3 GridToWorldPos(int3 gridPos, FlowField flowField)
    {
        return flowField.BoundsMin + new float3(gridPos) * flowField.CellSize;
    }
    
    private float SampleNearestObstacleOptimized(float3 worldPos, in DynamicBuffer<OctreeNode> nodes, in DynamicBuffer<SpatialObject> objects)
    {
        if (nodes.Length == 0)
            return float.MaxValue;
            
        float minDistance = float.MaxValue;
        float maxSearchRadius = 20f; // Limit search radius for performance
        
        // Use octree traversal instead of brute force
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
        if (nodeIndex >= nodes.Length)
            return;
            
        OctreeNode node = nodes[nodeIndex];
        
        // Early exit if node is too far away
        float nodeDistance = DistanceToOctreeNode(queryPoint, node);
        if (nodeDistance > maxSearchRadius || nodeDistance > minDistance)
            return;
            
        if (node.IsLeaf)
        {
            // Check all objects in this leaf
            for (int i = node.ObjectStartIndex; i < node.ObjectStartIndex + node.ObjectCount; i++)
            {
                if (i >= objects.Length) break;
                
                SpatialObject obj = objects[i];
                float distance = CalculateDistanceToSpatialObject(queryPoint, obj);
                minDistance = math.min(minDistance, distance);
                
                // Early exit if we're very close
                if (minDistance < 0.1f)
                    return;
            }
        }
        else
        {
            // For branch nodes, traverse children (simplified for parallel context)
            for (int i = 0; i < 8; i++)
            {
                int childIndex = node.ChildStartIndex + i;
                if (childIndex < nodes.Length)
                {
                    OctreeNode childNode = nodes[childIndex];
                    float distance = DistanceToOctreeNode(queryPoint, childNode);
                    
                    if (distance <= maxSearchRadius && distance <= minDistance)
                    {
                        TraverseOctreeForDistance(queryPoint, maxSearchRadius, childIndex, nodes, objects, ref minDistance);
                    }
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
            CollisionShapeType.Sphere => math.length(obj.Position - point) - obj.HalfExtents.x, // x is radius for spheres
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

}
