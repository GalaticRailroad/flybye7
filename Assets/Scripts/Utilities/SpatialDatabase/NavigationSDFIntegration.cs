using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

// Integration component to tie SDF generation with navigation octree
[RequireComponent(typeof(NavigationOctreeAuthoring))]
[RequireComponent(typeof(NavigationSDFBuilder))]
public class NavigationSDFIntegration : MonoBehaviour
{
    [Header("SDF Generation Control")]
    [SerializeField] private bool generateOnStart = true;
    [SerializeField] private bool generateInEditor = false;
    [SerializeField] private float regenerationInterval = 0f; // 0 = no auto-regeneration

    [Header("Generation Settings")]
    [SerializeField] private bool useOptimizedGeneration = true;
    [SerializeField] private bool buildBVHForMeshes = true;
    [SerializeField] private int parallelBatchSize = 32;

    [Header("Generation Status")]
    [SerializeField] private bool isGenerating = false;
    [SerializeField] private float lastGenerationTime = 0f;
    [SerializeField] private float generationProgress = 0f;
    [SerializeField] private string generationStatus = "Not started";

    private NavigationOctreeAuthoring octreeAuthoring;
    private NavigationSDFBuilder sdfBuilder;
    private JobHandle currentGenerationJob;
    private float nextRegenerationTime = 0f;

    private void Awake()
    {
        octreeAuthoring = GetComponent<NavigationOctreeAuthoring>();
        sdfBuilder = GetComponent<NavigationSDFBuilder>();
    }

    private void Start()
    {
        if (generateOnStart && Application.isPlaying)
        {
            StartSDFGeneration();
        }
    }

    private void Update()
    {
        // Check for automatic regeneration
        if (regenerationInterval > 0 && Time.time >= nextRegenerationTime)
        {
            if (!isGenerating)
            {
                StartSDFGeneration();
                nextRegenerationTime = Time.time + regenerationInterval;
            }
        }

        // Update generation progress
        if (isGenerating && currentGenerationJob.IsCompleted)
        {
            CompleteSDFGeneration();
        }
    }

    private void OnValidate()
    {
        if (generateInEditor && !Application.isPlaying)
        {
#if UNITY_EDITOR
            UnityEditor.EditorApplication.delayCall += () =>
            {
                if (this != null)
                {
                    StartSDFGenerationInEditor();
                }
            };
#endif
        }
    }

    public void StartSDFGeneration()
    {
        if (isGenerating)
        {
            Debug.LogWarning("SDF generation already in progress");
            return;
        }

        StartCoroutine(GenerateSDFCoroutine());
    }

    private System.Collections.IEnumerator GenerateSDFCoroutine()
    {
        isGenerating = true;
        generationProgress = 0f;
        generationStatus = "Extracting colliders...";
        var startTime = Time.realtimeSinceStartup;

        // Step 1: Extract scene colliders
        sdfBuilder.ExtractSceneColliders();
        yield return null;

        generationProgress = 0.2f;
        generationStatus = "Building spatial hash...";

        // Step 2: Build spatial acceleration structure
        var spatialHash = sdfBuilder.BuildSpatialHash();
        yield return null;

        generationProgress = 0.3f;
        generationStatus = "Getting octree entity...";

        // Step 3: Get octree entity from ECS
        var world = World.DefaultGameObjectInjectionWorld;
        if (world == null)
        {
            Debug.LogError("No ECS world found");
            isGenerating = false;
            yield break;
        }

        var entityManager = world.EntityManager;
        Entity octreeEntity = Entity.Null;

        // Find or create octree entity
        var query = entityManager.CreateEntityQuery(typeof(NavigationOctree));
        if (query.CalculateEntityCount() > 0)
        {
            using (var entities = query.ToEntityArray(Allocator.Temp))
            {
                octreeEntity = entities[0];
            }
        }
        else
        {
            Debug.LogError("No octree entity found. Please ensure NavigationOctreeAuthoring has been properly set up.");
            isGenerating = false;
            yield break;
        }

        if (octreeEntity == Entity.Null)
        {
            Debug.LogError("Failed to get octree entity");
            isGenerating = false;
            yield break;
        }

        generationProgress = 0.4f;
        generationStatus = "Generating SDF...";

        // Step 4: Generate SDF
        if (useOptimizedGeneration)
        {
            currentGenerationJob = GenerateSDFOptimized(octreeEntity, entityManager, spatialHash);
        }
        else
        {
            currentGenerationJob = sdfBuilder.GenerateSDFForOctree(octreeEntity, entityManager);
        }

        // Wait for job completion
        while (!currentGenerationJob.IsCompleted)
        {
            generationProgress = 0.4f + (0.5f * GetJobProgress());
            yield return null;
        }

        currentGenerationJob.Complete();

        generationProgress = 0.9f;
        generationStatus = "Finalizing...";

        // Step 5: Update navigation tiers based on SDF
        UpdateNavigationTiers(octreeEntity, entityManager);
        yield return null;

        // Clean up
        spatialHash.Dispose();

        lastGenerationTime = Time.realtimeSinceStartup - startTime;
        generationProgress = 1f;
        generationStatus = $"Complete ({lastGenerationTime:F2}s)";
        isGenerating = false;

        Debug.Log($"SDF generation complete in {lastGenerationTime:F2} seconds");
    }

    private JobHandle GenerateSDFOptimized(Entity octreeEntity, EntityManager entityManager, SpatialHashGrid spatialHash)
    {
        var octree = entityManager.GetComponentData<NavigationOctree>(octreeEntity);
        var nodes = entityManager.GetBuffer<NavigationOctreeNode>(octreeEntity);
        var fieldData = entityManager.GetBuffer<NavigationFieldData>(octreeEntity);

        // Ensure field data buffer has correct size
        if (fieldData.Length < nodes.Length)
        {
            fieldData.ResizeUninitialized(nodes.Length);
        }

        // Get collider and mesh data
        var colliderInfos = sdfBuilder.GetColliderInfos();
        var meshDataList = sdfBuilder.GetMeshDataList();

        // Convert spatial hash to native format
        float cellSize = 5f; // Default cell size
        var nativeSpatialHash = new NativeParallelMultiHashMap<int3, int>(colliderInfos.Length * 8, Allocator.TempJob);
        PopulateNativeSpatialHash(spatialHash, colliderInfos, nativeSpatialHash);

        // Allocate SDF data
        int nodeCount = nodes.Length;
        var sdfData = new NativeArray<SDFNodeData>(nodeCount, Allocator.TempJob);

        // Create parallel SDF generation job
        var generateJob = new NavigationSDFOptimized.ParallelSDFGenerationJob
        {
            Octree = octree,
            Nodes = nodes.AsNativeArray(),
            Colliders = colliderInfos,
            Meshes = meshDataList,
            SpatialHash = nativeSpatialHash,
            SpatialHashCellSize = cellSize,
            SDFData = sdfData
        };

        var generateHandle = generateJob.Schedule(nodeCount, parallelBatchSize);

        // Compute gradients
        var gradients = new NativeArray<float3>(nodeCount, Allocator.TempJob);
        var gradientJob = new NavigationSDFOptimized.OptimizedGradientJob
        {
            Octree = octree,
            Nodes = nodes.AsNativeArray(),
            SDFData = sdfData,
            Gradients = gradients
        };

        var gradientHandle = gradientJob.Schedule(nodeCount, parallelBatchSize, generateHandle);

        // Copy results to field data
        var copyJob = new CopyOptimizedSDFToFieldDataJob
        {
            SDFData = sdfData,
            Gradients = gradients,
            FieldData = fieldData.AsNativeArray()
        };

        var copyHandle = copyJob.Schedule(nodeCount, 64, gradientHandle);

        // Dispose temporary data
        sdfData.Dispose(copyHandle);
        gradients.Dispose(copyHandle);
        nativeSpatialHash.Dispose(copyHandle);

        return copyHandle;
    }

    private void PopulateNativeSpatialHash(
        SpatialHashGrid managedHash,
        NativeArray<SDFColliderInfo> colliders,
        NativeParallelMultiHashMap<int3, int> nativeHash)
    {
        float cellSize = 5f; // Default cell size since we can't access from managedHash
        float inverseCellSize = 1f / cellSize;

        for (int i = 0; i < colliders.Length; i++)
        {
            SDFColliderInfo info = colliders[i];

            // Calculate bounds for collider
            float3 boundsMin, boundsMax;
            GetColliderBounds(info, out boundsMin, out boundsMax);

            // Insert into spatial hash
            int3 minCell = new int3(math.floor(boundsMin * inverseCellSize));
            int3 maxCell = new int3(math.floor(boundsMax * inverseCellSize));

            for (int z = minCell.z; z <= maxCell.z; z++)
            {
                for (int y = minCell.y; y <= maxCell.y; y++)
                {
                    for (int x = minCell.x; x <= maxCell.x; x++)
                    {
                        int3 cell = new int3(x, y, z);
                        nativeHash.Add(cell, i);
                    }
                }
            }
        }
    }

    private void GetColliderBounds(SDFColliderInfo info, out float3 boundsMin, out float3 boundsMax)
    {
        // Similar to NavigationSDFBuilder.GetColliderBounds
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

            default:
                boundsMin = info.Position;
                boundsMax = info.Position;
                break;
        }
    }

    private void UpdateNavigationTiers(Entity octreeEntity, EntityManager entityManager)
    {
        var nodes = entityManager.GetBuffer<NavigationOctreeNode>(octreeEntity);
        var fieldData = entityManager.GetBuffer<NavigationFieldData>(octreeEntity);

        // Update navigation tiers based on distance
        for (int i = 0; i < nodes.Length; i++)
        {
            if (!nodes[i].IsLeaf)
                continue;

            if (nodes[i].FieldDataIndex < 0 || nodes[i].FieldDataIndex >= fieldData.Length)
                continue;

            float distance = fieldData[nodes[i].FieldDataIndex].SignedDistance;
            NavigationOctreeNode node = nodes[i];

            // Classify tier based on distance
            if (math.abs(distance) < 2f)
            {
                node.Tier = NavigationTier.Close;
            }
            else if (math.abs(distance) < 10f)
            {
                node.Tier = NavigationTier.Medium;
            }
            else
            {
                node.Tier = NavigationTier.Far;
            }

            // Update min/max distance
            node.MinDistance = distance;
            node.MaxDistance = distance;

            nodes[i] = node;
        }
    }

    private void CompleteSDFGeneration()
    {
        currentGenerationJob.Complete();
        isGenerating = false;
        Debug.Log($"SDF generation completed in {lastGenerationTime:F2} seconds");
    }

    private float GetJobProgress()
    {
        // Estimate progress (actual progress tracking would require atomic counters)
        return 0.5f;
    }

#if UNITY_EDITOR
    private void StartSDFGenerationInEditor()
    {
        if (!UnityEditor.EditorApplication.isPlaying)
        {
            Debug.Log("Starting SDF generation in editor...");

            // Extract colliders
            sdfBuilder.ExtractSceneColliders();

            // Build spatial hash
            var spatialHash = sdfBuilder.BuildSpatialHash();

            // Note: Full ECS generation requires play mode
            Debug.LogWarning("Complete SDF generation requires play mode. Enter play mode to generate full SDF.");

            spatialHash.Dispose();
        }
    }
#endif

    // Public API
    public bool IsGenerating => isGenerating;
    public float GenerationProgress => generationProgress;
    public string GenerationStatus => generationStatus;
    public float LastGenerationTime => lastGenerationTime;

    // Context menu actions
    [ContextMenu("Generate SDF")]
    public void GenerateSDFFromMenu()
    {
        if (Application.isPlaying)
        {
            StartSDFGeneration();
        }
        else
        {
            Debug.LogWarning("SDF generation requires play mode");
        }
    }

    [ContextMenu("Clear SDF")]
    public void ClearSDF()
    {
        if (currentGenerationJob.IsCompleted)
        {
            currentGenerationJob.Complete();
        }

        isGenerating = false;
        generationProgress = 0f;
        generationStatus = "Cleared";
    }
}

// Job to copy optimized SDF data to field data
[Unity.Burst.BurstCompile]
public struct CopyOptimizedSDFToFieldDataJob : IJobParallelFor
{
    [ReadOnly] public NativeArray<SDFNodeData> SDFData;
    [ReadOnly] public NativeArray<float3> Gradients;
    [NativeDisableParallelForRestriction]
    public NativeArray<NavigationFieldData> FieldData;

    public void Execute(int index)
    {
        if (index >= SDFData.Length || index >= FieldData.Length)
            return;

        SDFNodeData sdf = SDFData[index];
        NavigationFieldData field = FieldData[index];

        field.SignedDistance = sdf.Distance;
        field.GradientField = Gradients[index];

        // TODO: Compute corner values for proper trilinear interpolation

        FieldData[index] = field;
    }
}

// Extension to add CellSize property to SpatialHashGrid
public static class SpatialHashGridExtensions
{
    public static float CellSize = 5f; // Default cell size
}