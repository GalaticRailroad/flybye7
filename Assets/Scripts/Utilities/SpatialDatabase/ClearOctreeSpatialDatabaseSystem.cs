using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;

[BurstCompile]
[UpdateInGroup(typeof(BuildSpatialDatabaseGroup), OrderFirst = true)]
public unsafe partial struct ClearOctreeSpatialDatabaseSystem : ISystem
{
    private EntityQuery _octreeDatabasesQuery;

    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        _octreeDatabasesQuery = SystemAPI.QueryBuilder()
            .WithAll<OctreeSpatialDatabase, OctreeNode, SpatialObject>()
            .Build();
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        // Only proceed if we have octree database entities
        if (_octreeDatabasesQuery.CalculateEntityCount() > 0)
        {
            ComponentLookup<OctreeSpatialDatabase> octreeLookup = SystemAPI.GetComponentLookup<OctreeSpatialDatabase>(false);
            BufferLookup<OctreeNode> nodesBufferLookup = SystemAPI.GetBufferLookup<OctreeNode>(false);
            BufferLookup<SpatialObject> objectsBufferLookup = SystemAPI.GetBufferLookup<SpatialObject>(false);
            NativeArray<Entity> octreeEntities = _octreeDatabasesQuery.ToEntityArray(Allocator.Temp);

            JobHandle initialDep = state.Dependency;
            
            // Clear each octree database in a separate thread
            for (int i = 0; i < octreeEntities.Length; i++)
            {
                ClearOctreeSpatialDatabaseJob clearJob = new ClearOctreeSpatialDatabaseJob
                {
                    Entity = octreeEntities[i],
                    OctreeLookup = octreeLookup,
                    NodesBufferLookup = nodesBufferLookup,
                    ObjectsBufferLookup = objectsBufferLookup,
                };
                state.Dependency = JobHandle.CombineDependencies(state.Dependency, clearJob.Schedule(initialDep));
            }

            octreeEntities.Dispose();
        }
    }
    
    [BurstCompile]
    public struct ClearOctreeSpatialDatabaseJob : IJob
    {
        public Entity Entity;
        public ComponentLookup<OctreeSpatialDatabase> OctreeLookup;
        public BufferLookup<OctreeNode> NodesBufferLookup;
        public BufferLookup<SpatialObject> ObjectsBufferLookup;
        
        public void Execute()
        {
            if (OctreeLookup.HasComponent(Entity) &&
                NodesBufferLookup.TryGetBuffer(Entity, out DynamicBuffer<OctreeNode> nodesBuffer) &&
                ObjectsBufferLookup.TryGetBuffer(Entity, out DynamicBuffer<SpatialObject> objectsBuffer))
            {
                RefRW<OctreeSpatialDatabase> octreeRef = OctreeLookup.GetRefRW(Entity);
                OctreeSpatialDatabase.ClearAndResize(ref octreeRef.ValueRW, nodesBuffer, objectsBuffer);
            }
        }
    }
}