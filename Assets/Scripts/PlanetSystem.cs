using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

[BurstCompile]
[UpdateAfter(typeof(BuildSpatialDatabaseGroup))]
[UpdateBefore(typeof(BuildingSystem))]
public partial struct PlanetSystem : ISystem
{
    private EntityQuery _spatialDatabasesQuery;
    
    [BurstCompile]
    public void OnCreate(ref SystemState state)
    {
        _spatialDatabasesQuery = SystemAPI.QueryBuilder().WithAll<OctreeSpatialDatabase, OctreeNode, SpatialObject>().Build();
        
        state.RequireForUpdate<Config>();
        state.RequireForUpdate<SpatialDatabaseSingleton>();
        // Don't require spatial database query - check manually in OnUpdate
        state.RequireForUpdate<BeginSimulationEntityCommandBufferSystem.Singleton>();
        state.RequireForUpdate<TeamManagerReference>();
    }
    
    [BurstCompile]
    public void OnUpdate(ref SystemState state)
    {
        Config config = SystemAPI.GetSingleton<Config>();
        SpatialDatabaseSingleton spatialDatabaseSingleton = SystemAPI.GetSingleton<SpatialDatabaseSingleton>();
        
        // Check if spatial database exists and is ready
        Entity spatialDbEntity = spatialDatabaseSingleton.TargetablesSpatialDatabase;
        if (spatialDbEntity == Entity.Null || 
            !state.EntityManager.HasComponent<OctreeSpatialDatabase>(spatialDbEntity))
        {
            // Skip spatial queries this frame - spatial database not ready yet
            return;
        }
        
        EntityQuery planetsQuery = SystemAPI.QueryBuilder().WithAll<Planet>().Build();
        
        PlanetShipsAssessmentJob shipsAssessmentJob = new PlanetShipsAssessmentJob
        {
            TotalPlanetsCount = planetsQuery.CalculateEntityCount(),
            PlanetShipsAssessmentsPerUpdate = config.PlanetShipAssessmentsPerUpdate,
            CachedOctreeDatabase = new CachedOctreeSpatialDatabaseRO
            {
                SpatialDatabaseEntity = spatialDatabaseSingleton.TargetablesSpatialDatabase,
                OctreeDatabaseLookup = SystemAPI.GetComponentLookup<OctreeSpatialDatabase>(true),
                NodesBufferLookup = SystemAPI.GetBufferLookup<OctreeNode>(true),
                ObjectsBufferLookup = SystemAPI.GetBufferLookup<SpatialObject>(true),
            },
        };
        state.Dependency = shipsAssessmentJob.Schedule(state.Dependency);
        
        PlanetConversionJob conversionJob = new PlanetConversionJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
            ECB = SystemAPI.GetSingletonRW<BeginSimulationEntityCommandBufferSystem.Singleton>().ValueRW.CreateCommandBuffer(state.WorldUnmanaged),
            BuildingReferenceLookup = SystemAPI.GetComponentLookup<BuildingReference>(true),
            BuildingLookup = SystemAPI.GetComponentLookup<Building>(true),
        };
        state.Dependency = conversionJob.Schedule(state.Dependency);

        PlanetResourcesJob resourcesJob = new PlanetResourcesJob
        {
            DeltaTime = SystemAPI.Time.DeltaTime,
        };
        state.Dependency = resourcesJob.Schedule(state.Dependency);
        
        PlanetClearBuildingsDataJob planetClearBuildingsDataJob = new PlanetClearBuildingsDataJob
        { };
        state.Dependency = planetClearBuildingsDataJob.ScheduleParallel(state.Dependency);
    }

    [BurstCompile]
    public partial struct PlanetShipsAssessmentJob : IJobEntity, IJobEntityChunkBeginEnd
    {
        public int TotalPlanetsCount;
        public int PlanetShipsAssessmentsPerUpdate;
        public CachedOctreeSpatialDatabaseRO CachedOctreeDatabase;

        void Execute(Entity entity, in LocalTransform transform, ref Planet planet, in Team team,
            ref DynamicBuffer<PlanetShipsAssessment> shipsAssessmentBuffer,
            in DynamicBuffer<PlanetNetwork> planetNetworkBuffer)
        {
            // Ships assessment
            planet.ShipsAssessmentCounter -= math.min(TotalPlanetsCount, PlanetShipsAssessmentsPerUpdate);
            if (planet.ShipsAssessmentCounter < 0)
            {
                planet.ShipsAssessmentCounter += TotalPlanetsCount;

                // Clear assessment buffer
                for (int i = 0; i < shipsAssessmentBuffer.Length; i++)
                {
                    shipsAssessmentBuffer[i] = default;
                }

                // Query allied and enemy ships count using dynamic octree operations
                OctreePlanetAssessmentCollector collector = new OctreePlanetAssessmentCollector(team.Index, shipsAssessmentBuffer);
                float3 queryMin = transform.Position - planet.ShipsAssessmentExtents;
                float3 queryMax = transform.Position + planet.ShipsAssessmentExtents;
                
                DynamicBuffer<OctreeNode> nodesBuffer = CachedOctreeDatabase.NodesBufferLookup[CachedOctreeDatabase.SpatialDatabaseEntity];
                DynamicBuffer<SpatialObject> objectsBuffer = CachedOctreeDatabase.ObjectsBufferLookup[CachedOctreeDatabase.SpatialDatabaseEntity];
                
                OctreeDynamicOperations.QueryAABB(in CachedOctreeDatabase._OctreeDatabase,
                    nodesBuffer, objectsBuffer, queryMin, queryMax, ref collector);
            }
        }

        public bool OnChunkBegin(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            CachedOctreeDatabase.CacheData();
            return true;
        }

        public void OnChunkEnd(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask,
            bool chunkWasExecuted)
        { }
    }

    [BurstCompile]
    public partial struct PlanetConversionJob : IJobEntity
    {
        public float DeltaTime;
        public EntityCommandBuffer ECB;
        [ReadOnly]
        public ComponentLookup<BuildingReference> BuildingReferenceLookup;
        [ReadOnly]
        public ComponentLookup<Building> BuildingLookup;
        
        void Execute(Entity entity, ref Planet planet, in Team team, ref DynamicBuffer<CapturingWorker> capturingWorkers, in DynamicBuffer<MoonReference> moonReferences)
        {
            // Determine the majority team using CapturingWorker array
            int majorityTeamIndex = -1;
            float majorityTeamSpeed = 0;
            
            // Determine the majority team using capturingWorkers array
            for (int i = 0; i < capturingWorkers.Length; i++)
            {
                CapturingWorker capturingWorker = capturingWorkers[i];
                if (capturingWorker.CaptureSpeed > majorityTeamSpeed)
                {
                    majorityTeamIndex = i;
                    majorityTeamSpeed = capturingWorker.CaptureSpeed;
                }
            }

            // Reset conversion when there's a team change, or no unique team
            if (majorityTeamIndex < 0 || majorityTeamIndex != planet.LastConvertingTeam)
            {
                planet.LastConvertingTeam = -1;
                planet.CaptureProgress = 0;
            }
            
            // Handle conversion if there's a single team holds majority long enough
            if (majorityTeamIndex >= 0)
            {
                planet.LastConvertingTeam = majorityTeamIndex;
                planet.CaptureProgress += DeltaTime * majorityTeamSpeed;
                
                if (planet.CaptureProgress >= planet.CaptureTime)
                {
                    planet.CaptureProgress = 0;
                    GameUtilities.SetTeam(ECB, entity, planet.LastConvertingTeam);
                    
                    // Convert buildings
                    for (int i = 0; i < moonReferences.Length; i++)
                    {
                        Entity moonEntity = moonReferences[i].Entity;
                        Entity buildingEntity = BuildingReferenceLookup[moonEntity].BuildingEntity;
                        if (BuildingLookup.HasComponent(buildingEntity))
                        {
                            GameUtilities.SetTeam(ECB, buildingEntity, planet.LastConvertingTeam);
                        }
                    }
                }
            }

            // Clear capturingWorkers buffer
            for (int i = 0; i < capturingWorkers.Length; i++)
            {
                capturingWorkers[i] = default;
            }
        }
    }

    [BurstCompile]
    public partial struct PlanetResourcesJob : IJobEntity
    {
        public float DeltaTime;
        
        void Execute(ref Planet planet)
        {
            float3 finalGenerationRate =
                planet.ResourceGenerationRate + planet.ResearchBonuses.PlanetResourceGenerationRadeAdd;
            planet.ResourceCurrentStorage =
                math.clamp(planet.ResourceCurrentStorage + (finalGenerationRate * DeltaTime), float3.zero,
                    planet.ResourceMaxStorage);
        }
    }
    
    [BurstCompile]
    public partial struct PlanetClearBuildingsDataJob : IJobEntity
    {
        private void Execute(ref Planet planet)
        {
            planet.ResearchBonuses.Reset();
        }
    }
}
