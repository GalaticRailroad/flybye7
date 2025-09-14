using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Rendering;
using Unity.Transforms;
using UnityEngine;

// System for building navigation octree from scene geometry
[UpdateInGroup(typeof(InitializationSystemGroup))]
[UpdateAfter(typeof(BuildOctreeSpatialDatabaseSystem))]
public partial class NavigationOctreeBuilderSystem : SystemBase
{
    private EntityQuery _meshCollidersQuery;
    private EntityQuery _rendererBoundsQuery;

    protected override void OnCreate()
    {
        // Query for entities with mesh colliders
        // NOTE: PhysicsCollider requires Unity.Physics package
        // _meshCollidersQuery = GetEntityQuery(
        //     ComponentType.ReadOnly<PhysicsCollider>(),
        //     ComponentType.ReadOnly<LocalToWorld>()
        // );

        // Query for entities with renderer bounds (fallback)
        _rendererBoundsQuery = GetEntityQuery(
            ComponentType.ReadOnly<RenderBounds>(),
            ComponentType.ReadOnly<LocalToWorld>()
        );
    }

    protected override void OnUpdate()
    {
        // Only build octree during initialization or when requested
        var octreeQuery = GetEntityQuery(ComponentType.ReadWrite<NavigationOctree>());
        if (octreeQuery.IsEmpty)
            return;

        var octreeEntity = octreeQuery.GetSingletonEntity();
        var octree = EntityManager.GetComponentData<NavigationOctree>(octreeEntity);
        var nodesBuffer = EntityManager.GetBuffer<NavigationOctreeNode>(octreeEntity);
        var fieldBuffer = EntityManager.GetBuffer<NavigationFieldData>(octreeEntity);

        // Check if octree needs rebuilding
        if (octree.TotalNodes > 1) // Already built
            return;

        // Collect all obstacle geometry
        var obstacles = new NativeList<ObstacleData>(Allocator.TempJob);
        CollectObstacles(ref obstacles);

        // Build the octree structure
        BuildOctreeStructure(ref octree, nodesBuffer, obstacles);

        // Populate field data
        var populateJob = new PopulateFieldDataJob
        {
            Octree = octree,
            Nodes = nodesBuffer.AsNativeArray(),
            FieldData = fieldBuffer.AsNativeArray(),
            Obstacles = obstacles
        };

        populateJob.Schedule(octree.TotalNodes, 32).Complete();

        // Update octree component
        EntityManager.SetComponentData(octreeEntity, octree);

        obstacles.Dispose();
    }

    private void CollectObstacles(ref NativeList<ObstacleData> obstacles)
    {
        // Collect from physics colliders using manual iteration
        // NOTE: Disabled - requires Unity.Physics package
        /*
        if (_meshCollidersQuery != null && !_meshCollidersQuery.IsEmpty)
        {
            var colliderEntities = _meshCollidersQuery.ToEntityArray(Allocator.Temp);
            var colliderArray = _meshCollidersQuery.ToComponentDataArray<PhysicsCollider>(Allocator.Temp);
            var transformArray = _meshCollidersQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);

            for (int i = 0; i < colliderArray.Length; i++)
            {
                if (colliderArray[i].IsValid)
                {
                    var bounds = colliderArray[i].Value.Value.CalculateAabb(new RigidTransform(transformArray[i].Rotation, transformArray[i].Position));
                    obstacles.Add(new ObstacleData
                    {
                        Center = bounds.Center,
                        HalfExtents = bounds.Extents * 0.5f,
                        Type = ObstacleType.Collider
                    });
                }
            }

            colliderEntities.Dispose();
            colliderArray.Dispose();
            transformArray.Dispose();
        }
        */

        // Fallback to renderer bounds if no colliders
        if (obstacles.IsEmpty)
        {
            var boundsArray = _rendererBoundsQuery.ToComponentDataArray<RenderBounds>(Allocator.Temp);
            var boundsTransformArray = _rendererBoundsQuery.ToComponentDataArray<LocalToWorld>(Allocator.Temp);

            for (int i = 0; i < boundsArray.Length; i++)
            {
                obstacles.Add(new ObstacleData
                {
                    Center = boundsTransformArray[i].Position,
                    HalfExtents = boundsArray[i].Value.Extents * 0.5f,
                    Type = ObstacleType.Renderer
                });
            }

            boundsArray.Dispose();
            boundsTransformArray.Dispose();
        }
    }

    private void BuildOctreeStructure(ref NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodesBuffer,
        NativeList<ObstacleData> obstacles)
    {
        // Clear existing nodes
        nodesBuffer.Clear();

        // Create root node
        var rootNode = NavigationOctreeNode.CreateLeaf(
            octree.BoundsCenter,
            octree.BoundsHalfExtent,
            0,
            -1 // No parent
        );
        nodesBuffer.Add(rootNode);
        octree.RootNodeIndex = 0;
        octree.TotalNodes = 1;
        octree.NextFreeNodeIndex = 1;

        // Adaptively subdivide based on obstacle proximity
        SubdivideAdaptively(ref octree, nodesBuffer, obstacles, 0);
    }

    private void SubdivideAdaptively(ref NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodesBuffer,
        NativeList<ObstacleData> obstacles, int nodeIndex)
    {
        if (nodeIndex >= nodesBuffer.Length)
            return;

        var node = nodesBuffer[nodeIndex];

        // Check if we should subdivide
        if (!ShouldSubdivide(node, octree, obstacles))
            return;

        // Convert leaf to branch
        node.IsLeaf = false;
        node.ChildStartIndex = octree.NextFreeNodeIndex;
        nodesBuffer[nodeIndex] = node;

        // Create 8 children
        float childHalfExtent = node.HalfExtent * 0.5f;
        for (int i = 0; i < 8; i++)
        {
            float3 childCenter = NavigationOctree.GetChildCenter(node.Center, node.HalfExtent, i);
            var childNode = NavigationOctreeNode.CreateLeaf(
                childCenter,
                childHalfExtent,
                (byte)(node.Depth + 1),
                nodeIndex
            );

            // Calculate distance to nearest obstacle for this child
            childNode.MinDistance = CalculateMinDistanceToObstacles(childCenter, childHalfExtent, obstacles);

            // Set navigation tier based on distance
            if (childNode.MinDistance < octree.ProximityThreshold * 0.5f)
                childNode.Tier = NavigationTier.Close;
            else if (childNode.MinDistance < octree.ProximityThreshold)
                childNode.Tier = NavigationTier.Medium;
            else
                childNode.Tier = NavigationTier.Far;

            nodesBuffer.Add(childNode);
            octree.NextFreeNodeIndex++;
            octree.TotalNodes++;

            // Recursively subdivide children
            SubdivideAdaptively(ref octree, nodesBuffer, obstacles, node.ChildStartIndex + i);
        }
    }

    private bool ShouldSubdivide(NavigationOctreeNode node, NavigationOctree octree, NativeList<ObstacleData> obstacles)
    {
        // Don't subdivide if at max depth
        if (node.Depth >= octree.MaxDepth)
            return false;

        // Don't subdivide if node is too small
        if (node.HalfExtent * 2f <= octree.MinNodeSize)
            return false;

        // Calculate minimum distance to obstacles
        float minDist = CalculateMinDistanceToObstacles(node.Center, node.HalfExtent, obstacles);

        // Subdivide if close to obstacles
        return minDist < octree.ProximityThreshold;
    }

    private float CalculateMinDistanceToObstacles(float3 center, float halfExtent, NativeList<ObstacleData> obstacles)
    {
        float minDist = float.MaxValue;

        for (int i = 0; i < obstacles.Length; i++)
        {
            var obstacle = obstacles[i];

            // Simple AABB distance calculation
            float3 delta = math.max(float3.zero, math.abs(center - obstacle.Center) - (halfExtent + obstacle.HalfExtents));
            float dist = math.length(delta);
            minDist = math.min(minDist, dist);
        }

        return minDist;
    }

    public struct ObstacleData
    {
        public float3 Center;
        public float3 HalfExtents;
        public ObstacleType Type;
    }

    public enum ObstacleType : byte
    {
        Collider,
        Renderer
    }
}

// Job to populate field data in octree nodes
[BurstCompile]
public struct PopulateFieldDataJob : IJobParallelFor
{
    [ReadOnly] public NavigationOctree Octree;
    [ReadOnly] public NativeArray<NavigationOctreeNode> Nodes;
    [WriteOnly] public NativeArray<NavigationFieldData> FieldData;
    [ReadOnly] public NativeList<NavigationOctreeBuilderSystem.ObstacleData> Obstacles;

    public void Execute(int index)
    {
        if (index >= Nodes.Length)
            return;

        var node = Nodes[index];
        if (!node.IsLeaf)
            return;

        // Allocate field data for this leaf node
        var field = new NavigationFieldData();

        // Calculate signed distance field
        field.SignedDistance = CalculateSignedDistance(node.Center, Obstacles);

        // Calculate gradient (for obstacle avoidance)
        field.GradientField = CalculateGradient(node.Center, node.HalfExtent);

        // Calculate vector field (simplified - would be more complex in production)
        field.VectorField = CalculateVectorField(node.Center, field.SignedDistance, field.GradientField);

        // Calculate corner distances for interpolation
        float extent = node.HalfExtent;
        field.Corner000Distance = CalculateSignedDistance(node.Center + new float3(-extent, -extent, -extent), Obstacles);
        field.Corner001Distance = CalculateSignedDistance(node.Center + new float3(-extent, -extent, extent), Obstacles);
        field.Corner010Distance = CalculateSignedDistance(node.Center + new float3(-extent, extent, -extent), Obstacles);
        field.Corner011Distance = CalculateSignedDistance(node.Center + new float3(-extent, extent, extent), Obstacles);
        field.Corner100Distance = CalculateSignedDistance(node.Center + new float3(extent, -extent, -extent), Obstacles);
        field.Corner101Distance = CalculateSignedDistance(node.Center + new float3(extent, -extent, extent), Obstacles);
        field.Corner110Distance = CalculateSignedDistance(node.Center + new float3(extent, extent, -extent), Obstacles);
        field.Corner111Distance = CalculateSignedDistance(node.Center + new float3(extent, extent, extent), Obstacles);

        // Store field data
        if (index < FieldData.Length)
        {
            FieldData[index] = field;
        }
    }

    private float CalculateSignedDistance(float3 position, NativeList<NavigationOctreeBuilderSystem.ObstacleData> obstacles)
    {
        float minDist = float.MaxValue;

        for (int i = 0; i < obstacles.Length; i++)
        {
            var obstacle = obstacles[i];

            // AABB signed distance
            float3 q = math.abs(position - obstacle.Center) - obstacle.HalfExtents;
            float dist = math.length(math.max(q, 0)) + math.min(math.max(q.x, math.max(q.y, q.z)), 0);
            minDist = math.min(minDist, dist);
        }

        return minDist;
    }

    private float3 CalculateGradient(float3 center, float halfExtent)
    {
        float epsilon = halfExtent * 0.1f;

        float dx = CalculateSignedDistance(center + new float3(epsilon, 0, 0), Obstacles) -
                   CalculateSignedDistance(center - new float3(epsilon, 0, 0), Obstacles);
        float dy = CalculateSignedDistance(center + new float3(0, epsilon, 0), Obstacles) -
                   CalculateSignedDistance(center - new float3(0, epsilon, 0), Obstacles);
        float dz = CalculateSignedDistance(center + new float3(0, 0, epsilon), Obstacles) -
                   CalculateSignedDistance(center - new float3(0, 0, epsilon), Obstacles);

        float3 gradient = new float3(dx, dy, dz) / (2f * epsilon);
        return math.normalizesafe(gradient);
    }

    private float3 CalculateVectorField(float3 position, float distance, float3 gradient)
    {
        // Simple vector field - point away from obstacles when close
        if (distance < 10f)
        {
            // Repulsion from obstacles
            return gradient * math.saturate(1f - distance / 10f);
        }

        // Default flow (could be toward a goal, etc.)
        return float3.zero;
    }

}