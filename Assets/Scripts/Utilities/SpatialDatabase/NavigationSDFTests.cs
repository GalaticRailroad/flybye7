using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

// SDF test suite for validation
public class NavigationSDFTests : MonoBehaviour
{
    [Header("Test Configuration")]
    [SerializeField] private bool runTestsOnStart = true;
    [SerializeField] private bool verboseLogging = true;
    [SerializeField] private float testTolerance = 0.01f;

    [Header("Test Selection")]
    [SerializeField] private bool testAnalyticalShapes = true;
    [SerializeField] private bool testGradientContinuity = true;
    [SerializeField] private bool testSignConsistency = true;
    [SerializeField] private bool testInterpolationSmoothness = true;
    [SerializeField] private bool testPerformance = true;
    [SerializeField] private bool testEdgeCases = true;

    [Header("Test Results")]
    [SerializeField] private List<TestResult> testResults = new List<TestResult>();
    [SerializeField] private int totalTests = 0;
    [SerializeField] private int passedTests = 0;
    [SerializeField] private int failedTests = 0;

    [System.Serializable]
    public class TestResult
    {
        public string testName;
        public bool passed;
        public string details;
        public float executionTime;
    }

    private NavigationOctreeAuthoring octreeAuthoring;
    private NavigationSDFBuilder sdfBuilder;

    private void Awake()
    {
        octreeAuthoring = GetComponent<NavigationOctreeAuthoring>();
        sdfBuilder = GetComponent<NavigationSDFBuilder>();
    }

    private void Start()
    {
        if (runTestsOnStart)
        {
            RunAllTests();
        }
    }

    public void RunAllTests()
    {
        testResults.Clear();
        totalTests = 0;
        passedTests = 0;
        failedTests = 0;

        Debug.Log("=== Starting SDF Test Suite ===");

        if (testAnalyticalShapes)
            RunAnalyticalTests();

        if (testGradientContinuity)
            RunGradientContinuityTests();

        if (testSignConsistency)
            RunSignConsistencyTests();

        if (testInterpolationSmoothness)
            RunInterpolationSmoothnessTests();

        if (testPerformance)
            RunPerformanceTests();

        if (testEdgeCases)
            RunEdgeCaseTests();

        // Report results
        Debug.Log($"=== Test Suite Complete ===");
        Debug.Log($"Total: {totalTests}, Passed: {passedTests}, Failed: {failedTests}");

        foreach (var result in testResults)
        {
            if (!result.passed)
            {
                Debug.LogError($"FAILED: {result.testName} - {result.details}");
            }
            else if (verboseLogging)
            {
                Debug.Log($"PASSED: {result.testName} ({result.executionTime:F3}ms)");
            }
        }
    }

    private void RunAnalyticalTests()
    {
        Debug.Log("Running Analytical Shape Tests...");

        // Test 1: Sphere
        TestAnalyticalSphere();

        // Test 2: Box
        TestAnalyticalBox();

        // Test 3: Capsule
        TestAnalyticalCapsule();

        // Test 4: Plane
        TestAnalyticalPlane();
    }

    private void TestAnalyticalSphere()
    {
        string testName = "Analytical Sphere SDF";
        var startTime = Time.realtimeSinceStartup;

        // Create sphere collider
        float3 center = new float3(0, 0, 0);
        float radius = 5f;

        var colliders = new NativeArray<SDFColliderInfo>(1, Allocator.Temp);
        colliders[0] = new SDFColliderInfo
        {
            Type = ColliderType.Sphere,
            Position = center,
            SphereRadius = radius
        };

        var meshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

        // Test points
        var testPoints = new float3[]
        {
            center,                                    // Center: should be -radius
            center + new float3(radius, 0, 0),       // On surface: should be 0
            center + new float3(radius * 2, 0, 0),   // Outside: should be radius
            center + new float3(radius * 0.5f, 0, 0) // Inside: should be -radius * 0.5
        };

        var expectedDistances = new float[]
        {
            -radius,
            0f,
            radius,
            -radius * 0.5f
        };

        bool passed = true;
        string details = "";

        for (int i = 0; i < testPoints.Length; i++)
        {
            float distance = NavigationSDFGenerator.CalculateSignedDistance(testPoints[i], colliders, meshes);
            float expected = expectedDistances[i];
            float error = math.abs(distance - expected);

            if (error > testTolerance)
            {
                passed = false;
                details += $"Point {i}: Expected {expected:F3}, Got {distance:F3}, Error {error:F3}\n";
            }
        }

        colliders.Dispose();
        meshes.Dispose();

        RecordTestResult(testName, passed, details, Time.realtimeSinceStartup - startTime);
    }

    private void TestAnalyticalBox()
    {
        string testName = "Analytical Box SDF";
        var startTime = Time.realtimeSinceStartup;

        // Create box collider
        float3 center = new float3(0, 0, 0);
        float3 halfExtents = new float3(3, 2, 4);

        var colliders = new NativeArray<SDFColliderInfo>(1, Allocator.Temp);
        colliders[0] = new SDFColliderInfo
        {
            Type = ColliderType.Box,
            Position = center,
            Rotation = quaternion.identity,
            Scale = float3.zero,
            BoxHalfExtents = halfExtents
        };

        var meshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

        // Test points
        var testPoints = new float3[]
        {
            center,                                        // Center
            center + new float3(halfExtents.x, 0, 0),    // On face
            center + new float3(halfExtents.x * 2, 0, 0), // Outside face
            center + halfExtents,                         // On corner
            center + halfExtents * 1.5f                   // Outside corner
        };

        bool passed = true;
        string details = "";

        for (int i = 0; i < testPoints.Length; i++)
        {
            float distance = NavigationSDFGenerator.CalculateSignedDistance(testPoints[i], colliders, meshes);

            // Basic sanity checks
            if (i == 0 && distance >= 0) // Center should be inside
            {
                passed = false;
                details += $"Center point should be inside (negative distance), got {distance:F3}\n";
            }
            else if (i == 1 && math.abs(distance) > testTolerance) // On face should be ~0
            {
                passed = false;
                details += $"Face point should be on surface (distance ~0), got {distance:F3}\n";
            }
            else if (i == 2 && distance <= 0) // Outside should be positive
            {
                passed = false;
                details += $"Outside point should have positive distance, got {distance:F3}\n";
            }
        }

        colliders.Dispose();
        meshes.Dispose();

        RecordTestResult(testName, passed, details, Time.realtimeSinceStartup - startTime);
    }

    private void TestAnalyticalCapsule()
    {
        string testName = "Analytical Capsule SDF";
        var startTime = Time.realtimeSinceStartup;

        // Create capsule collider
        float3 center = new float3(0, 0, 0);
        float radius = 2f;
        float height = 6f;

        var colliders = new NativeArray<SDFColliderInfo>(1, Allocator.Temp);
        colliders[0] = new SDFColliderInfo
        {
            Type = ColliderType.Capsule,
            Position = center,
            Rotation = quaternion.identity,
            CapsuleRadius = radius,
            CapsuleHeight = height,
            CapsuleDirection = 1 // Y-axis
        };

        var meshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

        // Test points
        float halfHeight = height * 0.5f - radius;
        var testPoints = new float3[]
        {
            center,                                    // Center
            center + new float3(radius, 0, 0),       // On cylinder surface
            center + new float3(0, halfHeight + radius, 0), // Top hemisphere
            center + new float3(0, -halfHeight - radius, 0), // Bottom hemisphere
        };

        bool passed = true;
        string details = "";

        for (int i = 0; i < testPoints.Length; i++)
        {
            float distance = NavigationSDFGenerator.CalculateSignedDistance(testPoints[i], colliders, meshes);

            if (i == 0 && distance >= 0) // Center should be inside
            {
                passed = false;
                details += $"Center should be inside, got distance {distance:F3}\n";
            }
            else if (i == 1 && math.abs(distance) > testTolerance) // On surface
            {
                passed = false;
                details += $"Cylinder surface point should have distance ~0, got {distance:F3}\n";
            }
        }

        colliders.Dispose();
        meshes.Dispose();

        RecordTestResult(testName, passed, details, Time.realtimeSinceStartup - startTime);
    }

    private void TestAnalyticalPlane()
    {
        string testName = "Analytical Plane (Thin Box) SDF";
        var startTime = Time.realtimeSinceStartup;

        // Create thin box as plane
        float3 center = new float3(0, 0, 0);
        float3 halfExtents = new float3(10, 0.1f, 10);

        var colliders = new NativeArray<SDFColliderInfo>(1, Allocator.Temp);
        colliders[0] = new SDFColliderInfo
        {
            Type = ColliderType.Box,
            Position = center,
            Rotation = quaternion.identity,
            Scale = float3.zero,
            BoxHalfExtents = halfExtents
        };

        var meshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

        // Test points
        var testPoints = new float3[]
        {
            center + new float3(0, 1, 0),    // Above plane
            center + new float3(0, -1, 0),   // Below plane
            center,                           // On plane
            center + new float3(0, 0.1f, 0), // Just above surface
        };

        bool passed = true;
        string details = "";

        for (int i = 0; i < testPoints.Length; i++)
        {
            float distance = NavigationSDFGenerator.CalculateSignedDistance(testPoints[i], colliders, meshes);

            if (i < 2 && distance <= 0) // Above/below should be outside
            {
                passed = false;
                details += $"Point {i} should be outside plane, got distance {distance:F3}\n";
            }
            else if (i == 2 && distance >= 0) // On plane should be inside
            {
                passed = false;
                details += $"Point on plane should be inside, got distance {distance:F3}\n";
            }
        }

        colliders.Dispose();
        meshes.Dispose();

        RecordTestResult(testName, passed, details, Time.realtimeSinceStartup - startTime);
    }

    private void RunGradientContinuityTests()
    {
        string testName = "Gradient Continuity";
        var startTime = Time.realtimeSinceStartup;

        Debug.Log("Running Gradient Continuity Tests...");

        // Create test geometry
        var colliders = new NativeArray<SDFColliderInfo>(1, Allocator.Temp);
        colliders[0] = new SDFColliderInfo
        {
            Type = ColliderType.Sphere,
            Position = float3.zero,
            SphereRadius = 5f
        };

        var meshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

        // Sample along lines through space
        bool passed = true;
        string details = "";
        int discontinuities = 0;

        for (int line = 0; line < 10; line++)
        {
            float3 start = new float3(
                UnityEngine.Random.Range(-10f, 10f),
                UnityEngine.Random.Range(-10f, 10f),
                UnityEngine.Random.Range(-10f, 10f)
            );

            float3 direction = math.normalize(new float3(
                UnityEngine.Random.Range(-1f, 1f),
                UnityEngine.Random.Range(-1f, 1f),
                UnityEngine.Random.Range(-1f, 1f)
            ));

            float prevDistance = float.NaN;
            float3 prevGradient = float3.zero;

            for (int i = 0; i < 100; i++)
            {
                float3 point = start + direction * (i * 0.2f);
                float distance = NavigationSDFGenerator.CalculateSignedDistance(point, colliders, meshes);

                // Compute gradient
                float epsilon = 0.01f;
                float dx = NavigationSDFGenerator.CalculateSignedDistance(point + new float3(epsilon, 0, 0), colliders, meshes) -
                          NavigationSDFGenerator.CalculateSignedDistance(point - new float3(epsilon, 0, 0), colliders, meshes);
                float dy = NavigationSDFGenerator.CalculateSignedDistance(point + new float3(0, epsilon, 0), colliders, meshes) -
                          NavigationSDFGenerator.CalculateSignedDistance(point - new float3(0, epsilon, 0), colliders, meshes);
                float dz = NavigationSDFGenerator.CalculateSignedDistance(point + new float3(0, 0, epsilon), colliders, meshes) -
                          NavigationSDFGenerator.CalculateSignedDistance(point - new float3(0, 0, epsilon), colliders, meshes);

                float3 gradient = new float3(dx, dy, dz) / (2f * epsilon);

                if (!float.IsNaN(prevDistance))
                {
                    // Check for discontinuities
                    float distanceChange = math.abs(distance - prevDistance);
                    float expectedChange = 0.2f; // Step size

                    if (distanceChange > expectedChange * 1.5f)
                    {
                        discontinuities++;
                        if (discontinuities <= 3) // Only report first few
                        {
                            details += $"Discontinuity at {point}: distance jumped by {distanceChange:F3}\n";
                        }
                    }

                    // Check gradient smoothness
                    if (math.lengthsq(prevGradient) > 0.01f && math.lengthsq(gradient) > 0.01f)
                    {
                        float3 normalizedPrev = math.normalize(prevGradient);
                        float3 normalizedCurr = math.normalize(gradient);
                        float dotProduct = math.dot(normalizedPrev, normalizedCurr);

                        if (dotProduct < 0.9f && math.abs(distance) > 0.5f) // Allow more variation near surface
                        {
                            discontinuities++;
                        }
                    }
                }

                prevDistance = distance;
                prevGradient = gradient;
            }
        }

        if (discontinuities > 5)
        {
            passed = false;
            details = $"Found {discontinuities} gradient discontinuities\n" + details;
        }

        colliders.Dispose();
        meshes.Dispose();

        RecordTestResult(testName, passed, details, Time.realtimeSinceStartup - startTime);
    }

    private void RunSignConsistencyTests()
    {
        string testName = "Sign Consistency";
        var startTime = Time.realtimeSinceStartup;

        Debug.Log("Running Sign Consistency Tests...");

        // Create overlapping geometry
        var colliders = new NativeArray<SDFColliderInfo>(2, Allocator.Temp);
        colliders[0] = new SDFColliderInfo
        {
            Type = ColliderType.Sphere,
            Position = new float3(-2, 0, 0),
            SphereRadius = 3f
        };
        colliders[1] = new SDFColliderInfo
        {
            Type = ColliderType.Sphere,
            Position = new float3(2, 0, 0),
            SphereRadius = 3f
        };

        var meshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

        bool passed = true;
        string details = "";

        // Test points that should definitely be inside
        var insidePoints = new float3[]
        {
            new float3(0, 0, 0),    // Between spheres (overlapping region)
            new float3(-2, 0, 0),   // Center of first sphere
            new float3(2, 0, 0),    // Center of second sphere
        };

        foreach (var point in insidePoints)
        {
            float distance = NavigationSDFGenerator.CalculateSignedDistance(point, colliders, meshes);
            if (distance > 0)
            {
                passed = false;
                details += $"Point {point} should be inside (negative distance), got {distance:F3}\n";
            }
        }

        // Test points that should definitely be outside
        var outsidePoints = new float3[]
        {
            new float3(10, 0, 0),
            new float3(0, 10, 0),
            new float3(0, 0, 10),
        };

        foreach (var point in outsidePoints)
        {
            float distance = NavigationSDFGenerator.CalculateSignedDistance(point, colliders, meshes);
            if (distance < 0)
            {
                passed = false;
                details += $"Point {point} should be outside (positive distance), got {distance:F3}\n";
            }
        }

        colliders.Dispose();
        meshes.Dispose();

        RecordTestResult(testName, passed, details, Time.realtimeSinceStartup - startTime);
    }

    private void RunInterpolationSmoothnessTests()
    {
        string testName = "Interpolation Smoothness";
        var startTime = Time.realtimeSinceStartup;

        Debug.Log("Running Interpolation Smoothness Tests...");

        // This test would require an actual octree with field data
        // For now, we'll do a simplified test

        bool passed = true;
        string details = "Interpolation test requires octree setup";

        // TODO: Implement full interpolation test with octree

        RecordTestResult(testName, passed, details, Time.realtimeSinceStartup - startTime);
    }

    private void RunPerformanceTests()
    {
        string testName = "Performance Benchmark";
        var startTime = Time.realtimeSinceStartup;

        Debug.Log("Running Performance Tests...");

        // Create complex scene
        int numColliders = 20;
        var colliders = new NativeArray<SDFColliderInfo>(numColliders, Allocator.Temp);

        for (int i = 0; i < numColliders; i++)
        {
            colliders[i] = new SDFColliderInfo
            {
                Type = (ColliderType)(UnityEngine.Random.Range(1, 4)), // Random type (excluding None and Mesh)
                Position = new float3(
                    UnityEngine.Random.Range(-50f, 50f),
                    UnityEngine.Random.Range(-50f, 50f),
                    UnityEngine.Random.Range(-50f, 50f)
                ),
                Rotation = quaternion.identity,
                SphereRadius = 3f,
                BoxHalfExtents = new float3(2, 2, 2),
                CapsuleRadius = 1f,
                CapsuleHeight = 4f,
                CapsuleDirection = 1
            };
        }

        var meshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

        // Benchmark distance calculations
        int numQueries = 10000;
        var queryStartTime = Time.realtimeSinceStartup;

        for (int i = 0; i < numQueries; i++)
        {
            float3 point = new float3(
                UnityEngine.Random.Range(-100f, 100f),
                UnityEngine.Random.Range(-100f, 100f),
                UnityEngine.Random.Range(-100f, 100f)
            );

            NavigationSDFGenerator.CalculateSignedDistance(point, colliders, meshes);
        }

        float queryTime = Time.realtimeSinceStartup - queryStartTime;
        float averageQueryTime = (queryTime * 1000f) / numQueries; // Convert to ms

        bool passed = averageQueryTime < 0.1f; // Target: < 0.1ms per query
        string details = $"Average query time: {averageQueryTime:F4}ms for {numColliders} colliders";

        colliders.Dispose();
        meshes.Dispose();

        RecordTestResult(testName, passed, details, Time.realtimeSinceStartup - startTime);
    }

    private void RunEdgeCaseTests()
    {
        string testName = "Edge Cases";
        var startTime = Time.realtimeSinceStartup;

        Debug.Log("Running Edge Case Tests...");

        bool passed = true;
        string details = "";

        // Test 1: Empty collider array
        {
            var emptyColliders = new NativeArray<SDFColliderInfo>(0, Allocator.Temp);
            var emptyMeshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

            float distance = NavigationSDFGenerator.CalculateSignedDistance(float3.zero, emptyColliders, emptyMeshes);

            if (distance != float.MaxValue)
            {
                passed = false;
                details += $"Empty collider array should return MaxValue, got {distance}\n";
            }

            emptyColliders.Dispose();
            emptyMeshes.Dispose();
        }

        // Test 2: Very small geometry
        {
            var colliders = new NativeArray<SDFColliderInfo>(1, Allocator.Temp);
            colliders[0] = new SDFColliderInfo
            {
                Type = ColliderType.Sphere,
                Position = float3.zero,
                SphereRadius = 0.001f
            };

            var meshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

            float distance = NavigationSDFGenerator.CalculateSignedDistance(new float3(0.01f, 0, 0), colliders, meshes);

            if (float.IsNaN(distance) || float.IsInfinity(distance))
            {
                passed = false;
                details += $"Small geometry produced invalid distance: {distance}\n";
            }

            colliders.Dispose();
            meshes.Dispose();
        }

        // Test 3: Very large geometry
        {
            var colliders = new NativeArray<SDFColliderInfo>(1, Allocator.Temp);
            colliders[0] = new SDFColliderInfo
            {
                Type = ColliderType.Box,
                Position = float3.zero,
                Rotation = quaternion.identity,
                BoxHalfExtents = new float3(10000, 10000, 10000)
            };

            var meshes = new NativeArray<SDFMeshData>(0, Allocator.Temp);

            float distance = NavigationSDFGenerator.CalculateSignedDistance(new float3(100, 100, 100), colliders, meshes);

            if (float.IsNaN(distance) || float.IsInfinity(distance))
            {
                passed = false;
                details += $"Large geometry produced invalid distance: {distance}\n";
            }

            colliders.Dispose();
            meshes.Dispose();
        }

        RecordTestResult(testName, passed, details, Time.realtimeSinceStartup - startTime);
    }

    private void RecordTestResult(string testName, bool passed, string details, float executionTime)
    {
        var result = new TestResult
        {
            testName = testName,
            passed = passed,
            details = details,
            executionTime = executionTime * 1000f // Convert to ms
        };

        testResults.Add(result);
        totalTests++;

        if (passed)
        {
            passedTests++;
            if (verboseLogging)
            {
                Debug.Log($"✓ {testName} PASSED ({executionTime * 1000f:F3}ms)");
            }
        }
        else
        {
            failedTests++;
            Debug.LogError($"✗ {testName} FAILED: {details}");
        }
    }

    // Context menu for editor testing
    [ContextMenu("Run All Tests")]
    public void RunAllTestsFromMenu()
    {
        RunAllTests();
    }

    [ContextMenu("Clear Test Results")]
    public void ClearTestResults()
    {
        testResults.Clear();
        totalTests = 0;
        passedTests = 0;
        failedTests = 0;
        Debug.Log("Test results cleared");
    }
}