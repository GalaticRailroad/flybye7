using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

[RequireComponent(typeof(Transform))]
public class NavigationOctreeAuthoring : MonoBehaviour
{
    [Header("Octree Bounds")]
    [Tooltip("Center of the octree in world space")]
    public float3 Center = float3.zero;

    [Tooltip("Half extents of the octree bounds")]
    public float HalfExtent = 100f;

    [Header("Subdivision Parameters")]
    [Range(1, 8)]
    [Tooltip("Maximum depth of octree subdivision")]
    public byte MaxDepth = 6;

    [Tooltip("Minimum node size based on actor size")]
    public float MinNodeSize = 1f;

    [Tooltip("Distance to obstacles that triggers subdivision")]
    public float ProximityThreshold = 10f;

    [Tooltip("Variance threshold for adaptive subdivision")]
    public float SubdivisionThreshold = 0.5f;

    [Header("Memory Preallocation")]
    [Tooltip("Initial capacity for octree nodes")]
    public int PreallocatedNodeCapacity = 4096;

    [Tooltip("Initial capacity for field data")]
    public int PreallocatedFieldCapacity = 2048;

    [Header("Visualization")]
    public bool ShowOctreeBounds = true;
    public bool ShowOctreeNodes = false;
    public bool ShowDistanceField = false;
    public bool ShowVectorField = false;
    public bool ShowNavigationTiers = false;

    [Range(0, 8)]
    public int MaxVisualizationDepth = 3;

    [Range(0, 100)]
    public float VisualizationDistance = 50f;

    public Color BoundsColor = Color.green;
    public Color[] DepthColors = new Color[]
    {
        Color.green,
        Color.yellow,
        new Color(1f, 0.5f, 0f), // Orange
        Color.red,
        Color.magenta,
        Color.blue,
        Color.cyan,
        Color.white
    };

    public Color[] TierColors = new Color[]
    {
        Color.green,  // Far
        Color.yellow, // Medium
        Color.red     // Close
    };

    public Gradient DistanceGradient;
    public float MaxDistanceVisualization = 20f;

    // Runtime visualization data (populated by system)
    private NavigationOctreeVisualizationData _visualizationData;

    class Baker : Baker<NavigationOctreeAuthoring>
    {
        public override void Bake(NavigationOctreeAuthoring authoring)
        {
            Entity entity = GetEntity(authoring, TransformUsageFlags.None);

            // Add octree component
            AddComponent(entity, new NavigationOctree
            {
                BoundsCenter = authoring.Center,
                BoundsHalfExtent = authoring.HalfExtent,
                MaxDepth = authoring.MaxDepth,
                MinNodeSize = authoring.MinNodeSize,
                SubdivisionThreshold = authoring.SubdivisionThreshold,
                ProximityThreshold = authoring.ProximityThreshold,
                RootNodeIndex = 0,
                TotalNodes = 0,
                NextFreeNodeIndex = 0,
                NextFreeFieldIndex = 0,
                PreallocatedNodeCapacity = authoring.PreallocatedNodeCapacity,
                PreallocatedFieldCapacity = authoring.PreallocatedFieldCapacity
            });

            // Add buffers for nodes and field data
            var nodesBuffer = AddBuffer<NavigationOctreeNode>(entity);
            nodesBuffer.EnsureCapacity(authoring.PreallocatedNodeCapacity);

            var fieldBuffer = AddBuffer<NavigationFieldData>(entity);
            fieldBuffer.EnsureCapacity(authoring.PreallocatedFieldCapacity);

            // Add visualization component
            AddComponent(entity, new NavigationOctreeVisualization
            {
                ShowNodes = authoring.ShowOctreeNodes,
                ShowDistanceField = authoring.ShowDistanceField,
                ShowVectorField = authoring.ShowVectorField,
                ShowNavigationTiers = authoring.ShowNavigationTiers,
                MaxDepth = authoring.MaxVisualizationDepth,
                MaxDistance = authoring.VisualizationDistance
            });
        }
    }

    private void OnDrawGizmos()
    {
        // Always show bounds
        if (ShowOctreeBounds)
        {
            Gizmos.color = BoundsColor;
            Gizmos.DrawWireCube(Center, new Vector3(HalfExtent * 2f, HalfExtent * 2f, HalfExtent * 2f));
        }
    }

    private void OnDrawGizmosSelected()
    {
        // Initialize gradient if needed
        if (DistanceGradient == null || DistanceGradient.colorKeys.Length == 0)
        {
            DistanceGradient = new Gradient();
            var colorKeys = new GradientColorKey[]
            {
                new GradientColorKey(Color.red, 0f),
                new GradientColorKey(Color.yellow, 0.5f),
                new GradientColorKey(Color.green, 1f)
            };
            var alphaKeys = new GradientAlphaKey[]
            {
                new GradientAlphaKey(0.5f, 0f),
                new GradientAlphaKey(0.5f, 1f)
            };
            DistanceGradient.SetKeys(colorKeys, alphaKeys);
        }

        // Draw detailed visualization when selected
        if (!Application.isPlaying)
        {
            // In edit mode, show preview structure
            DrawOctreePreview();
        }
        else if (_visualizationData != null && _visualizationData.IsValid)
        {
            // In play mode, show actual octree data
            DrawRuntimeOctree();
        }
    }

    private void DrawOctreePreview()
    {
        if (ShowOctreeNodes)
        {
            DrawOctreeNodeRecursive(Center, HalfExtent, 0, MaxVisualizationDepth);
        }

        if (ShowNavigationTiers)
        {
            // Show tier zones as colored regions
            float tierSize = ProximityThreshold;

            // Close tier
            Gizmos.color = TierColors[2] * new Color(1, 1, 1, 0.2f);
            Gizmos.DrawCube(Center, Vector3.one * tierSize);

            // Medium tier
            Gizmos.color = TierColors[1] * new Color(1, 1, 1, 0.1f);
            Gizmos.DrawCube(Center, Vector3.one * tierSize * 2f);

            // Far tier is everything else
        }
    }

    private void DrawOctreeNodeRecursive(float3 center, float halfExtent, int depth, int maxDepth)
    {
        if (depth > maxDepth)
            return;

        // Check distance to camera
        Vector3 cameraPos = Camera.current != null ? Camera.current.transform.position : Vector3.zero;
        float distToCamera = Vector3.Distance(cameraPos, center);
        if (distToCamera > VisualizationDistance)
            return;

        // Draw this node
        Color color = DepthColors[math.min(depth, DepthColors.Length - 1)];
        color.a = 0.3f;
        Gizmos.color = color;
        Gizmos.DrawWireCube(center, new Vector3(halfExtent * 2f, halfExtent * 2f, halfExtent * 2f));

        // Draw children (preview only)
        if (depth < maxDepth)
        {
            float childHalfExtent = halfExtent * 0.5f;

            // Only subdivide near "obstacles" in preview
            bool shouldSubdivide = distToCamera < ProximityThreshold * (maxDepth - depth);

            if (shouldSubdivide)
            {
                for (int i = 0; i < 8; i++)
                {
                    float3 childCenter = NavigationOctree.GetChildCenter(center, halfExtent, i);
                    DrawOctreeNodeRecursive(childCenter, childHalfExtent, depth + 1, maxDepth);
                }
            }
        }
    }

    private void DrawRuntimeOctree()
    {
        // This would draw actual runtime data from the ECS system
        // The system would populate _visualizationData with actual octree node data

        if (ShowDistanceField)
        {
            // Draw distance field visualization
            DrawDistanceField();
        }

        if (ShowVectorField)
        {
            // Draw vector field arrows
            DrawVectorField();
        }
    }

    private void DrawDistanceField()
    {
        // Sample points in a grid and color by distance
        int samples = 20;
        float step = (HalfExtent * 2f) / samples;

        for (int x = 0; x < samples; x++)
        {
            for (int y = 0; y < samples; y++)
            {
                for (int z = 0; z < samples; z++)
                {
                    float3 pos = Center - HalfExtent + new float3(x * step, y * step, z * step);

                    // In runtime, this would query actual distance field
                    // For preview, just use distance to center as example
                    float distance = math.length(pos - Center);
                    float normalizedDist = math.saturate(distance / MaxDistanceVisualization);

                    Gizmos.color = DistanceGradient.Evaluate(normalizedDist);
                    Gizmos.DrawCube(pos, Vector3.one * step * 0.5f);
                }
            }
        }
    }

    private void DrawVectorField()
    {
        // Draw arrows showing flow direction
        int samples = 10;
        float step = (HalfExtent * 2f) / samples;

        for (int x = 0; x < samples; x++)
        {
            for (int y = 0; y < samples; y++)
            {
                for (int z = 0; z < samples; z++)
                {
                    float3 pos = Center - HalfExtent + new float3(x * step, y * step, z * step);

                    // In runtime, this would query actual vector field
                    // For preview, show radial field as example
                    float3 direction = math.normalizesafe(pos - Center);

                    Gizmos.color = Color.cyan;
                    DrawArrow(pos, direction * step * 0.5f);
                }
            }
        }
    }

    private void DrawArrow(float3 position, float3 direction)
    {
        if (math.lengthsq(direction) < 0.001f)
            return;

        Gizmos.DrawLine(position, position + direction);

        // Draw arrowhead
        float3 right = math.cross(direction, new float3(0, 1, 0));
        if (math.lengthsq(right) < 0.001f)
            right = math.cross(direction, new float3(1, 0, 0));
        right = math.normalize(right);

        float3 arrowBase = position + direction * 0.8f;
        float arrowSize = math.length(direction) * 0.2f;

        Gizmos.DrawLine(position + direction, arrowBase + right * arrowSize);
        Gizmos.DrawLine(position + direction, arrowBase - right * arrowSize);
    }
}

// Component for runtime visualization settings
public struct NavigationOctreeVisualization : IComponentData
{
    public bool ShowNodes;
    public bool ShowDistanceField;
    public bool ShowVectorField;
    public bool ShowNavigationTiers;
    public int MaxDepth;
    public float MaxDistance;
}

// Runtime visualization data (populated by system)
public class NavigationOctreeVisualizationData
{
    public bool IsValid { get; set; }
    // Additional runtime data would go here
}