using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Jobs.LowLevel.Unsafe;
using Unity.Mathematics;
using Unity.Transforms;

[BurstCompile]
[UpdateInGroup(typeof(BuildSpatialDatabaseGroup))]
[UpdateAfter(typeof(ClearOctreeSpatialDatabaseSystem))]
public partial struct BuildOctreeSpatialDatabaseSystem : ISystem
{
    private EntityQuery _octreeDatabasesQuery;
    
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        _octreeDatabasesQuery = SystemAPI.QueryBuilder()
            .WithAll<OctreeSpatialDatabase, OctreeNode, SpatialObject>()
            .Build();
        
        state.RequireForUpdate<Config>();
        state.RequireForUpdate<SpatialDatabaseSingleton>();
        // Don't require the query here - check it manually in OnUpdate
    }

    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        Config config = SystemAPI.GetSingleton<Config>();
        SpatialDatabaseSingleton spatialDatabaseSingleton = SystemAPI.GetSingleton<SpatialDatabaseSingleton>();

        // Check if the spatial database entity exists and is an octree
        Entity spatialDbEntity = spatialDatabaseSingleton.TargetablesSpatialDatabase;
        if (spatialDbEntity == Entity.Null || 
            !state.EntityManager.HasComponent<OctreeSpatialDatabase>(spatialDbEntity) ||
            !state.EntityManager.HasBuffer<OctreeNode>(spatialDbEntity) ||
            !state.EntityManager.HasBuffer<SpatialObject>(spatialDbEntity))
        {
            // Skip this frame if the spatial database isn't an octree or isn't ready yet
            return;
        }

        // For now, use single-threaded building
        // TODO: Implement parallel building similar to the uniform grid version
        BuildOctreeSpatialDatabaseSingleJob buildJob = new BuildOctreeSpatialDatabaseSingleJob
        {
            CachedOctreeDatabase = new CachedOctreeSpatialDatabase
            {
                SpatialDatabaseEntity = spatialDbEntity,
                OctreeDatabaseLookup = SystemAPI.GetComponentLookup<OctreeSpatialDatabase>(false),
                NodesBufferLookup = SystemAPI.GetBufferLookup<OctreeNode>(false),
                ObjectsBufferLookup = SystemAPI.GetBufferLookup<SpatialObject>(false),
            },
        };
        state.Dependency = buildJob.Schedule(state.Dependency);
    }

    [BurstCompile]
    [WithAll(typeof(Targetable))]
    public partial struct BuildOctreeSpatialDatabaseSingleJob : IJobEntity, IJobEntityChunkBeginEnd
    {
        public CachedOctreeSpatialDatabase CachedOctreeDatabase;
        
        public void Execute(Entity entity, in LocalToWorld ltw, in Team team, in ActorType actorType)
        {
            // Convert to the old SpatialDatabaseElement format for compatibility
            SpatialDatabaseElement element = new SpatialDatabaseElement
            {
                Entity = entity,
                Position = ltw.Position,
                Team = (byte)team.Index,
                Type = actorType.Type,
            };
            
            // Use dynamic operations directly for full subdivision support
            DynamicBuffer<OctreeNode> nodesBuffer = CachedOctreeDatabase.NodesBufferLookup[CachedOctreeDatabase.SpatialDatabaseEntity];
            DynamicBuffer<SpatialObject> objectsBuffer = CachedOctreeDatabase.ObjectsBufferLookup[CachedOctreeDatabase.SpatialDatabaseEntity];
            
            // Convert to SpatialObject and add directly
            SpatialObject spatialObject = new SpatialObject
            {
                Entity = element.Entity,
                Position = element.Position,
                ShapeType = CollisionShapeType.Point,
                HalfExtents = float3.zero,
                Team = element.Team,
                Type = element.Type
            };
            
            OctreeDynamicOperations.AddObject(ref CachedOctreeDatabase._OctreeDatabase,
                nodesBuffer, objectsBuffer, spatialObject);
        }

        public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            CachedOctreeDatabase.CacheData();
            return true;
        }

        public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask,
            bool chunkWasExecuted)
        {
        }
    }
}