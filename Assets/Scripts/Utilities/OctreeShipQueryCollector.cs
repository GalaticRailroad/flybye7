using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

// New octree-native query collector for better performance
public struct OctreeShipQueryCollector : IOctreeQueryCollector
{
    public Entity QuerierEntity;
    public float3 QuerierPosition;
    public byte QuerierTeam;
        
    public SpatialObject ClosestEnemy;
    public float ClosestEnemyDistanceSq;

    public OctreeShipQueryCollector(Entity querierEntity, float3 querierPosition, int querierTeam)
    {
        QuerierEntity = querierEntity;
        QuerierPosition = querierPosition;
        QuerierTeam = (byte)querierTeam;
            
        ClosestEnemy = default;
        ClosestEnemyDistanceSq = float.MaxValue;
    } 

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void OnVisitObject(in SpatialObject obj, out bool shouldEarlyExit)
    {
        shouldEarlyExit = false;

        if (obj.Team != QuerierTeam && obj.Entity.Index != QuerierEntity.Index)
        {
            float distSq = math.distancesq(QuerierPosition, obj.Position);
            if (distSq < ClosestEnemyDistanceSq)
            {
                ClosestEnemy = obj;
                ClosestEnemyDistanceSq = distSq;
                shouldEarlyExit = true; // Match original behavior - early exit when closer enemy found
            }
        }
    }
    
    // Conversion method to get the old SpatialDatabaseElement format
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public SpatialDatabaseElement GetClosestEnemyAsElement()
    {
        return new SpatialDatabaseElement
        {
            Entity = ClosestEnemy.Entity,
            Position = ClosestEnemy.Position,
            Team = ClosestEnemy.Team,
            Type = ClosestEnemy.Type
        };
    }
}