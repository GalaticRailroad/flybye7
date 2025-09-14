using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

// Test harness for navigation octree validation
public class NavigationOctreeTests : MonoBehaviour
{
    [Header("Test Configuration")]
    public bool RunTestsOnStart = true;
    public bool LogDetailedResults = false;

    [Header("Test Scene Setup")]
    public GameObject[] ObstacleObjects;
    public NavigationOctreeAuthoring OctreeAuthoring;

    [Header("Performance Tests")]
    public int QueryTestCount = 10000;
    public float QueryTestRadius = 50f;

    [Header("Test Results")]
    [SerializeField] private bool _allTestsPassed;
    [SerializeField] private float _queryPerformanceMs;
    [SerializeField] private float _buildPerformanceMs;
    [SerializeField] private int _totalNodesCreated;
    [SerializeField] private float _memoryUsageMB;

    private EntityManager _entityManager;
    private Entity _octreeEntity;

    private void Start()
    {
        if (RunTestsOnStart)
        {
            RunAllTests();
        }
    }

    public void RunAllTests()
    {
        Debug.Log("=== Starting Navigation Octree Tests ===");

        _entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
        _allTestsPassed = true;

        // Find or create octree entity
        SetupOctreeEntity();

        // Run test suite
        bool test1 = TestOctreeConstruction();
        bool test2 = TestSinglePointQueries();
        bool test3 = TestRadiusQueries();
        bool test4 = TestInterpolation();
        bool test5 = TestBatchQueries();
        bool test6 = TestPerformance();
        bool test7 = TestMemoryUsage();
        bool test8 = TestAdaptiveSubdivision();

        _allTestsPassed = test1 && test2 && test3 && test4 && test5 && test6 && test7 && test8;

        if (_allTestsPassed)
        {
            Debug.Log($"<color=green>✓ All Navigation Octree Tests PASSED</color>");
        }
        else
        {
            Debug.LogError($"<color=red>✗ Some Navigation Octree Tests FAILED</color>");
        }
    }

    private void SetupOctreeEntity()
    {
        // Create test octree entity if it doesn't exist
        var octreeQuery = _entityManager.CreateEntityQuery(ComponentType.ReadOnly<NavigationOctree>());

        if (octreeQuery.IsEmpty)
        {
            _octreeEntity = _entityManager.CreateEntity();
            _entityManager.AddComponentData(_octreeEntity, new NavigationOctree
            {
                BoundsCenter = float3.zero,
                BoundsHalfExtent = 100f,
                MaxDepth = 6,
                MinNodeSize = 1f,
                SubdivisionThreshold = 0.5f,
                ProximityThreshold = 10f,
                RootNodeIndex = 0,
                TotalNodes = 0,
                PreallocatedNodeCapacity = 4096,
                PreallocatedFieldCapacity = 2048
            });

            _entityManager.AddBuffer<NavigationOctreeNode>(_octreeEntity);
            _entityManager.AddBuffer<NavigationFieldData>(_octreeEntity);
        }
        else
        {
            _octreeEntity = octreeQuery.GetSingletonEntity();
        }
    }

    private bool TestOctreeConstruction()
    {
        Debug.Log("Test 1: Octree Construction");

        var startTime = Time.realtimeSinceStartup;

        // Get octree components
        var octree = _entityManager.GetComponentData<NavigationOctree>(_octreeEntity);
        var nodesBuffer = _entityManager.GetBuffer<NavigationOctreeNode>(_octreeEntity);
        var fieldBuffer = _entityManager.GetBuffer<NavigationFieldData>(_octreeEntity);

        // Clear and build octree
        nodesBuffer.Clear();
        fieldBuffer.Clear();

        // Create root node
        var rootNode = NavigationOctreeNode.CreateLeaf(
            octree.BoundsCenter,
            octree.BoundsHalfExtent,
            0, -1
        );
        nodesBuffer.Add(rootNode);
        octree.RootNodeIndex = 0;
        octree.TotalNodes = 1;

        // Simple test subdivision - divide root into 8 children
        rootNode.IsLeaf = false;
        rootNode.ChildStartIndex = 1;
        nodesBuffer[0] = rootNode;

        for (int i = 0; i < 8; i++)
        {
            float3 childCenter = NavigationOctree.GetChildCenter(
                octree.BoundsCenter,
                octree.BoundsHalfExtent,
                i
            );

            var childNode = NavigationOctreeNode.CreateLeaf(
                childCenter,
                octree.BoundsHalfExtent * 0.5f,
                1, 0
            );
            nodesBuffer.Add(childNode);
        }

        octree.TotalNodes = 9;
        _entityManager.SetComponentData(_octreeEntity, octree);

        _buildPerformanceMs = (Time.realtimeSinceStartup - startTime) * 1000f;
        _totalNodesCreated = octree.TotalNodes;

        bool passed = nodesBuffer.Length == 9;

        if (LogDetailedResults)
        {
            Debug.Log($"  - Created {octree.TotalNodes} nodes");
            Debug.Log($"  - Build time: {_buildPerformanceMs:F2}ms");
        }

        LogTestResult("Octree Construction", passed);
        return passed;
    }

    private bool TestSinglePointQueries()
    {
        Debug.Log("Test 2: Single Point Queries");

        var octree = _entityManager.GetComponentData<NavigationOctree>(_octreeEntity);
        var nodesBuffer = _entityManager.GetBuffer<NavigationOctreeNode>(_octreeEntity);

        bool allPointsFound = true;

        // Test center point
        float3 testPoint = octree.BoundsCenter;
        int nodeIndex = NavigationOctreeQueries.GetNodeContaining(testPoint, octree, nodesBuffer);
        if (nodeIndex < 0)
        {
            Debug.LogError($"  - Failed to find node for center point {testPoint}");
            allPointsFound = false;
        }

        // Test corner points
        float3[] cornerPoints = new float3[]
        {
            octree.BoundsCenter + new float3(-50, -50, -50),
            octree.BoundsCenter + new float3(50, 50, 50),
            octree.BoundsCenter + new float3(-50, 50, -50),
            octree.BoundsCenter + new float3(50, -50, 50),
        };

        foreach (var point in cornerPoints)
        {
            nodeIndex = NavigationOctreeQueries.GetNodeContaining(point, octree, nodesBuffer);
            if (nodeIndex < 0)
            {
                Debug.LogError($"  - Failed to find node for point {point}");
                allPointsFound = false;
            }
        }

        // Test out-of-bounds point
        float3 outOfBounds = octree.BoundsCenter + new float3(200, 200, 200);
        nodeIndex = NavigationOctreeQueries.GetNodeContaining(outOfBounds, octree, nodesBuffer);
        if (nodeIndex >= 0)
        {
            Debug.LogError($"  - Incorrectly found node for out-of-bounds point {outOfBounds}");
            allPointsFound = false;
        }

        LogTestResult("Single Point Queries", allPointsFound);
        return allPointsFound;
    }

    private bool TestRadiusQueries()
    {
        Debug.Log("Test 3: Radius Queries");

        var octree = _entityManager.GetComponentData<NavigationOctree>(_octreeEntity);
        var nodesBuffer = _entityManager.GetBuffer<NavigationOctreeNode>(_octreeEntity);

        var resultNodes = new NativeList<int>(Allocator.Temp);

        // Query at center with small radius
        NavigationOctreeQueries.GetNodesInRadius(octree.BoundsCenter, 10f, octree, nodesBuffer, ref resultNodes);
        bool centerQueryPassed = resultNodes.Length > 0;

        if (!centerQueryPassed)
        {
            Debug.LogError("  - Center radius query returned no nodes");
        }

        // Query with large radius should return multiple nodes
        resultNodes.Clear();
        NavigationOctreeQueries.GetNodesInRadius(octree.BoundsCenter, 60f, octree, nodesBuffer, ref resultNodes);
        bool largeQueryPassed = resultNodes.Length > 1;

        if (!largeQueryPassed)
        {
            Debug.LogError($"  - Large radius query returned only {resultNodes.Length} nodes");
        }

        resultNodes.Dispose();

        bool passed = centerQueryPassed && largeQueryPassed;
        LogTestResult("Radius Queries", passed);
        return passed;
    }

    private bool TestInterpolation()
    {
        Debug.Log("Test 4: Field Interpolation");

        var octree = _entityManager.GetComponentData<NavigationOctree>(_octreeEntity);
        var nodesBuffer = _entityManager.GetBuffer<NavigationOctreeNode>(_octreeEntity);
        var fieldBuffer = _entityManager.GetBuffer<NavigationFieldData>(_octreeEntity);

        // Ensure we have field data
        while (fieldBuffer.Length < nodesBuffer.Length)
        {
            var fieldData = new NavigationFieldData
            {
                SignedDistance = 10f,
                VectorField = new float3(0, 1, 0),
                GradientField = new float3(1, 0, 0),
                Corner000Distance = 5f,
                Corner001Distance = 6f,
                Corner010Distance = 7f,
                Corner011Distance = 8f,
                Corner100Distance = 9f,
                Corner101Distance = 10f,
                Corner110Distance = 11f,
                Corner111Distance = 12f
            };
            fieldBuffer.Add(fieldData);
        }

        // Link field data to nodes
        for (int i = 0; i < nodesBuffer.Length; i++)
        {
            if (nodesBuffer[i].IsLeaf)
            {
                var node = nodesBuffer[i];
                node.FieldDataIndex = i;
                nodesBuffer[i] = node;
            }
        }

        // Test interpolation at various points
        float3 testPoint = octree.BoundsCenter;
        float distance = NavigationOctreeQueries.InterpolateScalarField(testPoint, octree, nodesBuffer, fieldBuffer);
        bool distanceValid = !float.IsNaN(distance) && !float.IsInfinity(distance);

        float3 vector = NavigationOctreeQueries.InterpolateVectorField(testPoint, octree, nodesBuffer, fieldBuffer);
        bool vectorValid = math.all(math.isfinite(vector));

        bool passed = distanceValid && vectorValid;

        if (LogDetailedResults)
        {
            Debug.Log($"  - Interpolated distance: {distance}");
            Debug.Log($"  - Interpolated vector: {vector}");
        }

        LogTestResult("Field Interpolation", passed);
        return passed;
    }

    private bool TestBatchQueries()
    {
        Debug.Log("Test 5: Batch Queries");

        var octree = _entityManager.GetComponentData<NavigationOctree>(_octreeEntity);
        var nodesBuffer = _entityManager.GetBuffer<NavigationOctreeNode>(_octreeEntity);
        var fieldBuffer = _entityManager.GetBuffer<NavigationFieldData>(_octreeEntity);

        // Create batch of query positions
        int batchSize = 100;
        var queryPositions = new NativeArray<float3>(batchSize, Allocator.TempJob);
        var distanceResults = new NativeArray<float>(batchSize, Allocator.TempJob);
        var vectorResults = new NativeArray<float3>(batchSize, Allocator.TempJob);
        var tierResults = new NativeArray<NavigationTier>(batchSize, Allocator.TempJob);

        // Initialize random positions
        Unity.Mathematics.Random random = new Unity.Mathematics.Random(12345);
        for (int i = 0; i < batchSize; i++)
        {
            queryPositions[i] = octree.BoundsCenter + random.NextFloat3(-50f, 50f);
        }

        // Run batch query job
        var batchJob = new BatchQueryPositionsJob
        {
            QueryPositions = queryPositions,
            Octree = octree,
            Nodes = nodesBuffer.AsNativeArray(),
            FieldData = fieldBuffer.AsNativeArray(),
            DistanceResults = distanceResults,
            VectorResults = vectorResults,
            TierResults = tierResults
        };

        var handle = batchJob.Schedule(batchSize, 32);
        handle.Complete();

        // Validate results
        bool allValid = true;
        for (int i = 0; i < batchSize; i++)
        {
            if (float.IsNaN(distanceResults[i]) || !math.all(math.isfinite(vectorResults[i])))
            {
                allValid = false;
                break;
            }
        }

        queryPositions.Dispose();
        distanceResults.Dispose();
        vectorResults.Dispose();
        tierResults.Dispose();

        LogTestResult("Batch Queries", allValid);
        return allValid;
    }

    private bool TestPerformance()
    {
        Debug.Log("Test 6: Performance Test");

        var octree = _entityManager.GetComponentData<NavigationOctree>(_octreeEntity);
        var nodesBuffer = _entityManager.GetBuffer<NavigationOctreeNode>(_octreeEntity);

        var queryPositions = new NativeArray<float3>(QueryTestCount, Allocator.Temp);
        Unity.Mathematics.Random random = new Unity.Mathematics.Random(54321);

        for (int i = 0; i < QueryTestCount; i++)
        {
            queryPositions[i] = octree.BoundsCenter + random.NextFloat3(-QueryTestRadius, QueryTestRadius);
        }

        // Time queries
        var startTime = Time.realtimeSinceStartup;

        for (int i = 0; i < QueryTestCount; i++)
        {
            NavigationOctreeQueries.GetNodeContaining(queryPositions[i], octree, nodesBuffer);
        }

        _queryPerformanceMs = (Time.realtimeSinceStartup - startTime) * 1000f;

        queryPositions.Dispose();

        bool passed = _queryPerformanceMs < 1f; // Should complete in under 1ms

        Debug.Log($"  - {QueryTestCount} queries completed in {_queryPerformanceMs:F3}ms");
        Debug.Log($"  - Average time per query: {(_queryPerformanceMs / QueryTestCount) * 1000f:F3} microseconds");

        LogTestResult("Performance Test", passed);
        return passed;
    }

    private bool TestMemoryUsage()
    {
        Debug.Log("Test 7: Memory Usage");

        var octree = _entityManager.GetComponentData<NavigationOctree>(_octreeEntity);
        var nodesBuffer = _entityManager.GetBuffer<NavigationOctreeNode>(_octreeEntity);
        var fieldBuffer = _entityManager.GetBuffer<NavigationFieldData>(_octreeEntity);

        // Calculate memory usage
        int nodeSize = UnsafeUtility.SizeOf<NavigationOctreeNode>();
        int fieldSize = UnsafeUtility.SizeOf<NavigationFieldData>();

        int nodesMemory = nodesBuffer.Capacity * nodeSize;
        int fieldMemory = fieldBuffer.Capacity * fieldSize;
        int totalMemory = nodesMemory + fieldMemory;

        _memoryUsageMB = totalMemory / (1024f * 1024f);

        Debug.Log($"  - Nodes buffer: {nodesBuffer.Capacity} nodes × {nodeSize} bytes = {nodesMemory / 1024f:F2} KB");
        Debug.Log($"  - Field buffer: {fieldBuffer.Capacity} fields × {fieldSize} bytes = {fieldMemory / 1024f:F2} KB");
        Debug.Log($"  - Total memory: {_memoryUsageMB:F2} MB");

        bool passed = _memoryUsageMB < 10f; // Should use less than 10MB for test scene

        LogTestResult("Memory Usage", passed);
        return passed;
    }

    private bool TestAdaptiveSubdivision()
    {
        Debug.Log("Test 8: Adaptive Subdivision");

        var octree = _entityManager.GetComponentData<NavigationOctree>(_octreeEntity);
        var nodesBuffer = _entityManager.GetBuffer<NavigationOctreeNode>(_octreeEntity);

        // Count nodes at different depths
        int[] nodesPerDepth = new int[octree.MaxDepth + 1];
        int leafCount = 0;

        for (int i = 0; i < nodesBuffer.Length; i++)
        {
            var node = nodesBuffer[i];
            if (node.Depth <= octree.MaxDepth)
            {
                nodesPerDepth[node.Depth]++;
            }
            if (node.IsLeaf)
            {
                leafCount++;
            }
        }

        if (LogDetailedResults)
        {
            Debug.Log("  - Nodes per depth:");
            for (int d = 0; d <= octree.MaxDepth; d++)
            {
                if (nodesPerDepth[d] > 0)
                {
                    Debug.Log($"    Depth {d}: {nodesPerDepth[d]} nodes");
                }
            }
            Debug.Log($"  - Total leaf nodes: {leafCount}");
        }

        // Verify structure consistency
        bool structureValid = true;

        // Check that all points in space belong to exactly one leaf
        var testPoints = new NativeArray<float3>(100, Allocator.Temp);
        Unity.Mathematics.Random random = new Unity.Mathematics.Random(99999);

        for (int i = 0; i < testPoints.Length; i++)
        {
            testPoints[i] = octree.BoundsCenter + random.NextFloat3(-octree.BoundsHalfExtent * 0.9f, octree.BoundsHalfExtent * 0.9f);
            int nodeIdx = NavigationOctreeQueries.GetNodeContaining(testPoints[i], octree, nodesBuffer);

            if (nodeIdx >= 0 && nodeIdx < nodesBuffer.Length)
            {
                if (!nodesBuffer[nodeIdx].IsLeaf)
                {
                    Debug.LogError($"  - Point {testPoints[i]} mapped to non-leaf node!");
                    structureValid = false;
                }
            }
            else if (math.all(math.abs(testPoints[i] - octree.BoundsCenter) <= octree.BoundsHalfExtent))
            {
                Debug.LogError($"  - Point {testPoints[i]} within bounds but no node found!");
                structureValid = false;
            }
        }

        testPoints.Dispose();

        LogTestResult("Adaptive Subdivision", structureValid);
        return structureValid;
    }

    private void LogTestResult(string testName, bool passed)
    {
        if (passed)
        {
            Debug.Log($"  <color=green>✓ {testName} PASSED</color>");
        }
        else
        {
            Debug.LogError($"  <color=red>✗ {testName} FAILED</color>");
            _allTestsPassed = false;
        }
    }

    // Editor button for manual testing
    [ContextMenu("Run All Tests")]
    public void RunTestsManually()
    {
        RunAllTests();
    }

    [ContextMenu("Clear Test Results")]
    public void ClearResults()
    {
        _allTestsPassed = false;
        _queryPerformanceMs = 0;
        _buildPerformanceMs = 0;
        _totalNodesCreated = 0;
        _memoryUsageMB = 0;
        Debug.Log("Test results cleared");
    }
}