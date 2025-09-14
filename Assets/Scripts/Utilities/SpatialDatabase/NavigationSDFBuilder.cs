using System.Collections.Generic;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

// MonoBehaviour component for building SDF from Unity scene
public class NavigationSDFBuilder : MonoBehaviour
{
    [Header("SDF Generation Settings")]
    [SerializeField] private LayerMask obstacleLayerMask = -1;
    [SerializeField] private string obstacleTag = "NavigationObstacle";
    [SerializeField] private bool useTag = false;
    [SerializeField] private float maxProcessingDistance = 100f;

    [Header("Optimization Settings")]
    [SerializeField] private bool useSpatialHashing = true;
    [SerializeField] private float spatialHashCellSize = 5f;
    [SerializeField] private int maxTrianglesPerMesh = 10000;

    [Header("Debug Settings")]
    [SerializeField] private bool debugMode = false;
    [SerializeField] private float generationTimeLimit = 10f;

    private NativeList<SDFColliderInfo> colliderInfos;
    private NativeList<SDFMeshData> meshDataList;
    private Dictionary<Mesh, int> meshToIndex;

    // Analyze scene and extract collider information
    public void ExtractSceneColliders()
    {
        colliderInfos = new NativeList<SDFColliderInfo>(Allocator.Persistent);
        meshDataList = new NativeList<SDFMeshData>(Allocator.Persistent);
        meshToIndex = new Dictionary<Mesh, int>();

        // Find all colliders in scene
        Collider[] allColliders;
        if (useTag)
        {
            GameObject[] taggedObjects = GameObject.FindGameObjectsWithTag(obstacleTag);
            List<Collider> colliderList = new List<Collider>();

            for (int i = 0; i < taggedObjects.Length; i++)
            {
                Collider[] childColliders = taggedObjects[i].GetComponentsInChildren<Collider>();
                for (int j = 0; j < childColliders.Length; j++)
                {
                    colliderList.Add(childColliders[j]);
                }
            }

            allColliders = colliderList.ToArray();
        }
        else
        {
            allColliders = FindObjectsOfType<Collider>();
        }

        foreach (Collider collider in allColliders)
        {
            // Check layer mask
            if ((obstacleLayerMask & (1 << collider.gameObject.layer)) == 0)
                continue;

            // Skip disabled colliders
            if (!collider.enabled || !collider.gameObject.activeInHierarchy)
                continue;

            // Skip triggers
            if (collider.isTrigger)
                continue;

            // Process based on collider type
            if (collider is MeshCollider meshCollider)
            {
                ProcessMeshCollider(meshCollider);
            }
            else if (collider is BoxCollider boxCollider)
            {
                ProcessBoxCollider(boxCollider);
            }
            else if (collider is SphereCollider sphereCollider)
            {
                ProcessSphereCollider(sphereCollider);
            }
            else if (collider is CapsuleCollider capsuleCollider)
            {
                ProcessCapsuleCollider(capsuleCollider);
            }
            else if (collider is TerrainCollider terrainCollider)
            {
                ProcessTerrainCollider(terrainCollider);
            }
        }

        if (debugMode)
        {
            Debug.Log($"Extracted {colliderInfos.Length} colliders and {meshDataList.Length} unique meshes");
        }
    }

    private void ProcessMeshCollider(MeshCollider meshCollider)
    {
        Mesh mesh = meshCollider.sharedMesh;
        if (mesh == null)
            return;

        // Check if mesh is already processed
        int meshIndex;
        if (!meshToIndex.TryGetValue(mesh, out meshIndex))
        {
            // Add new mesh data
            meshIndex = meshDataList.Length;
            meshToIndex[mesh] = meshIndex;

            // Extract mesh data
            var vertices = mesh.vertices;
            var triangles = mesh.triangles;

            if (triangles.Length / 3 > maxTrianglesPerMesh)
            {
                Debug.LogWarning($"Mesh {mesh.name} has {triangles.Length / 3} triangles, which exceeds the limit. Consider simplifying.");
            }

            // Create blob asset for mesh data
            using (var builder = new BlobBuilder(Allocator.Temp))
            {
                ref var root = ref builder.ConstructRoot<SDFMeshData>();

                // Allocate arrays
                var vertexArray = builder.Allocate(ref root.Vertices, vertices.Length);
                var triangleArray = builder.Allocate(ref root.Triangles, triangles.Length / 3);

                // Copy vertices
                for (int i = 0; i < vertices.Length; i++)
                {
                    vertexArray[i] = vertices[i];
                }

                // Copy triangles
                for (int i = 0; i < triangles.Length; i += 3)
                {
                    triangleArray[i / 3] = new int3(triangles[i], triangles[i + 1], triangles[i + 2]);
                }

                // Set bounds
                root.BoundsMin = mesh.bounds.min;
                root.BoundsMax = mesh.bounds.max;
                root.LocalToWorld = meshCollider.transform.localToWorldMatrix;
                root.MeshID = meshIndex;

                // Create blob asset reference
                var blobAsset = builder.CreateBlobAssetReference<SDFMeshData>(Allocator.Persistent);
                meshDataList.Add(blobAsset.Value);
            }
        }

        // Add collider info
        Transform transform = meshCollider.transform;
        SDFColliderInfo info = new SDFColliderInfo
        {
            Type = ColliderType.Mesh,
            Position = transform.position,
            Rotation = transform.rotation,
            Scale = transform.lossyScale,
            MeshDataIndex = meshIndex
        };

        colliderInfos.Add(info);
    }

    private void ProcessBoxCollider(BoxCollider boxCollider)
    {
        Transform transform = boxCollider.transform;
        SDFColliderInfo info = new SDFColliderInfo
        {
            Type = ColliderType.Box,
            Position = transform.TransformPoint(boxCollider.center),
            Rotation = transform.rotation,
            Scale = transform.lossyScale,
            BoxHalfExtents = Vector3.Scale(boxCollider.size * 0.5f, transform.lossyScale)
        };

        colliderInfos.Add(info);
    }

    private void ProcessSphereCollider(SphereCollider sphereCollider)
    {
        Transform transform = sphereCollider.transform;
        float maxScale = Mathf.Max(transform.lossyScale.x, transform.lossyScale.y, transform.lossyScale.z);

        SDFColliderInfo info = new SDFColliderInfo
        {
            Type = ColliderType.Sphere,
            Position = transform.TransformPoint(sphereCollider.center),
            Rotation = transform.rotation,
            Scale = transform.lossyScale,
            SphereRadius = sphereCollider.radius * maxScale
        };

        colliderInfos.Add(info);
    }

    private void ProcessCapsuleCollider(CapsuleCollider capsuleCollider)
    {
        Transform transform = capsuleCollider.transform;
        Vector3 scale = transform.lossyScale;

        // Determine which axis the capsule is aligned with
        int direction = capsuleCollider.direction; // 0=X, 1=Y, 2=Z

        // Calculate scaled dimensions
        float radius = capsuleCollider.radius;
        float height = capsuleCollider.height;

        switch (direction)
        {
            case 0: // X-axis
                radius *= Mathf.Max(scale.y, scale.z);
                height *= scale.x;
                break;
            case 1: // Y-axis (default)
                radius *= Mathf.Max(scale.x, scale.z);
                height *= scale.y;
                break;
            case 2: // Z-axis
                radius *= Mathf.Max(scale.x, scale.y);
                height *= scale.z;
                break;
        }

        SDFColliderInfo info = new SDFColliderInfo
        {
            Type = ColliderType.Capsule,
            Position = transform.TransformPoint(capsuleCollider.center),
            Rotation = transform.rotation,
            Scale = scale,
            CapsuleRadius = radius,
            CapsuleHeight = height,
            CapsuleDirection = direction
        };

        colliderInfos.Add(info);
    }

    private void ProcessTerrainCollider(TerrainCollider terrainCollider)
    {
        // Terrain colliders require special handling
        // For now, we'll treat them as a heightfield mesh
        Terrain terrain = terrainCollider.GetComponent<Terrain>();
        if (terrain == null)
            return;

        TerrainData terrainData = terrain.terrainData;
        int resolution = 32; // Simplified resolution for SDF

        // Sample terrain heights and create a simplified mesh
        float[,] heights = new float[resolution, resolution];
        for (int z = 0; z < resolution; z++)
        {
            for (int x = 0; x < resolution; x++)
            {
                float normalizedX = (float)x / (resolution - 1);
                float normalizedZ = (float)z / (resolution - 1);
                heights[x, z] = terrainData.GetInterpolatedHeight(normalizedX, normalizedZ);
            }
        }

        // Create mesh from heightfield (simplified)
        List<Vector3> vertices = new List<Vector3>();
        List<int> triangles = new List<int>();

        float scaleX = terrainData.size.x / (resolution - 1);
        float scaleZ = terrainData.size.z / (resolution - 1);

        // Generate vertices
        for (int z = 0; z < resolution; z++)
        {
            for (int x = 0; x < resolution; x++)
            {
                vertices.Add(new Vector3(x * scaleX, heights[x, z], z * scaleZ));
            }
        }

        // Generate triangles
        for (int z = 0; z < resolution - 1; z++)
        {
            for (int x = 0; x < resolution - 1; x++)
            {
                int topLeft = z * resolution + x;
                int topRight = topLeft + 1;
                int bottomLeft = (z + 1) * resolution + x;
                int bottomRight = bottomLeft + 1;

                // First triangle
                triangles.Add(topLeft);
                triangles.Add(bottomLeft);
                triangles.Add(topRight);

                // Second triangle
                triangles.Add(topRight);
                triangles.Add(bottomLeft);
                triangles.Add(bottomRight);
            }
        }

        // Store as mesh data
        // (Similar to ProcessMeshCollider but with generated mesh)
        Debug.Log($"Processed terrain with {vertices.Count} vertices");
    }

    // Generate SDF for the octree
    public JobHandle GenerateSDFForOctree(Entity octreeEntity, EntityManager entityManager, JobHandle dependency = default)
    {
        if (!entityManager.HasComponent<NavigationOctree>(octreeEntity))
        {
            Debug.LogError("Entity does not have NavigationOctree component");
            return dependency;
        }

        var octree = entityManager.GetComponentData<NavigationOctree>(octreeEntity);
        var nodes = entityManager.GetBuffer<NavigationOctreeNode>(octreeEntity);
        var fieldData = entityManager.GetBuffer<NavigationFieldData>(octreeEntity);

        // Ensure field data buffer has correct size
        if (fieldData.Length < nodes.Length)
        {
            fieldData.ResizeUninitialized(nodes.Length);
        }

        // Convert lists to arrays for job
        var collidersArray = colliderInfos.AsArray();
        var meshesArray = meshDataList.AsArray();

        // Generate SDF using job system
        JobHandle sdfHandle = NavigationSDFGenerator.GenerateSDF(
            ref octree,
            nodes,
            fieldData,
            collidersArray,
            meshesArray,
            dependency
        );

        // Update octree component
        entityManager.SetComponentData(octreeEntity, octree);

        return sdfHandle;
    }

    // Build spatial hash for optimization
    public SpatialHashGrid BuildSpatialHash()
    {
        var spatialHash = new SpatialHashGrid(spatialHashCellSize, colliderInfos.Length);

        for (int i = 0; i < colliderInfos.Length; i++)
        {
            SDFColliderInfo info = colliderInfos[i];

            // Calculate bounds for collider
            float3 boundsMin, boundsMax;
            GetColliderBounds(info, out boundsMin, out boundsMax);

            // Insert into spatial hash
            spatialHash.Insert(i, boundsMin, boundsMax);
        }

        return spatialHash;
    }

    private static void GetMeshBounds(NativeArray<SDFMeshData> meshDataList, int meshIndex, SDFColliderInfo info, out float3 boundsMin, out float3 boundsMax)
    {
        // Access blob data directly to avoid CS0206
        float3 localMin = meshDataList[meshIndex].BoundsMin;
        float3 localMax = meshDataList[meshIndex].BoundsMax;

        // Transform bounds to world space (simplified)
        boundsMin = info.Position + localMin * info.Scale;
        boundsMax = info.Position + localMax * info.Scale;
    }

    private void GetColliderBounds(SDFColliderInfo info, out float3 boundsMin, out float3 boundsMax)
    {
        switch (info.Type)
        {
            case ColliderType.Box:
                boundsMin = info.Position - info.BoxHalfExtents;
                boundsMax = info.Position + info.BoxHalfExtents;
                break;

            case ColliderType.Sphere:
                float3 radiusVec = new float3(info.SphereRadius);
                boundsMin = info.Position - radiusVec;
                boundsMax = info.Position + radiusVec;
                break;

            case ColliderType.Capsule:
                float maxRadius = math.max(info.CapsuleRadius, info.CapsuleHeight * 0.5f);
                float3 maxRadiusVec = new float3(maxRadius);
                boundsMin = info.Position - maxRadiusVec;
                boundsMax = info.Position + maxRadiusVec;
                break;

            case ColliderType.Mesh:
                if (info.MeshDataIndex >= 0 && info.MeshDataIndex < meshDataList.Length)
                {
                    GetMeshBounds(meshDataList, info.MeshDataIndex, info, out boundsMin, out boundsMax);
                }
                else
                {
                    boundsMin = info.Position;
                    boundsMax = info.Position;
                }
                break;

            default:
                boundsMin = info.Position;
                boundsMax = info.Position;
                break;
        }
    }

    // Public API for integration
    public NativeArray<SDFColliderInfo> GetColliderInfos()
    {
        if (colliderInfos.IsCreated)
            return colliderInfos.AsArray();
        return new NativeArray<SDFColliderInfo>(0, Allocator.Temp);
    }

    public NativeArray<SDFMeshData> GetMeshDataList()
    {
        if (meshDataList.IsCreated)
            return meshDataList.AsArray();
        return new NativeArray<SDFMeshData>(0, Allocator.Temp);
    }

    // Cleanup
    private void OnDestroy()
    {
        if (colliderInfos.IsCreated)
            colliderInfos.Dispose();
        if (meshDataList.IsCreated)
            meshDataList.Dispose();
    }
}

// Spatial hash grid for optimization
public struct SpatialHashGrid
{
    private NativeParallelMultiHashMap<int3, int> hashMap;
    private float cellSize;
    private float inverseCellSize;

    public SpatialHashGrid(float cellSize, int capacity)
    {
        this.cellSize = cellSize;
        this.inverseCellSize = 1f / cellSize;
        hashMap = new NativeParallelMultiHashMap<int3, int>(capacity * 8, Allocator.Persistent);
    }

    public void Insert(int objectIndex, float3 boundsMin, float3 boundsMax)
    {
        int3 minCell = GetCellCoord(boundsMin);
        int3 maxCell = GetCellCoord(boundsMax);

        for (int z = minCell.z; z <= maxCell.z; z++)
        {
            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    int3 cell = new int3(x, y, z);
                    hashMap.Add(cell, objectIndex);
                }
            }
        }
    }

    public NativeList<int> Query(float3 position, float radius, Allocator allocator)
    {
        NativeList<int> results = new NativeList<int>(16, allocator);
        NativeHashSet<int> uniqueResults = new NativeHashSet<int>(16, Allocator.Temp);

        float3 minPos = position - radius;
        float3 maxPos = position + radius;

        int3 minCell = GetCellCoord(minPos);
        int3 maxCell = GetCellCoord(maxPos);

        for (int z = minCell.z; z <= maxCell.z; z++)
        {
            for (int y = minCell.y; y <= maxCell.y; y++)
            {
                for (int x = minCell.x; x <= maxCell.x; x++)
                {
                    int3 cell = new int3(x, y, z);

                    if (hashMap.TryGetFirstValue(cell, out int objectIndex, out var iterator))
                    {
                        do
                        {
                            if (uniqueResults.Add(objectIndex))
                            {
                                results.Add(objectIndex);
                            }
                        } while (hashMap.TryGetNextValue(out objectIndex, ref iterator));
                    }
                }
            }
        }

        uniqueResults.Dispose();
        return results;
    }

    private int3 GetCellCoord(float3 position)
    {
        return new int3(math.floor(position * inverseCellSize));
    }

    public void Dispose()
    {
        if (hashMap.IsCreated)
            hashMap.Dispose();
    }
}

// Extension methods (removed LINQ-style methods for performance)