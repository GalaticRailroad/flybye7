using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

// SDF Visualization component
[RequireComponent(typeof(NavigationOctreeAuthoring))]
public class NavigationSDFVisualizer : MonoBehaviour
{
    // Visualization modes
    public enum VisualizationMode
    {
        DistanceField,      // Color by distance
        Gradient,           // Show gradient vectors as arrows
        IsoSurfaces,        // Show distance contours
        ErrorMetric,        // Highlight high-error regions
        InsideOutside,      // Binary visualization
        SliceView,          // 2D slice through 3D field
        NavigationTiers     // Show Far/Medium/Close tiers
    }

    [Header("Visualization Settings")]
    [SerializeField] private VisualizationMode visualizationMode = VisualizationMode.DistanceField;
    [SerializeField] private bool enableVisualization = true;
    [SerializeField] private float visualizationRadius = 50f;

    [Header("Distance Field Settings")]
    [SerializeField] private Gradient distanceGradient;
    [SerializeField] private float minDistance = -5f;
    [SerializeField] private float maxDistance = 20f;
    [SerializeField] private bool showNegativeDistances = true;

    [Header("Gradient Settings")]
    [SerializeField] private float arrowScale = 1f;
    [SerializeField] private int gradientSkip = 2; // Show every Nth node
    [SerializeField] private Color gradientColor = Color.cyan;

    [Header("Iso-Surface Settings")]
    [SerializeField] private float[] isoValues = new float[] { 0f, 1f, 5f, 10f };
    [SerializeField] private Color[] isoColors = new Color[] { Color.red, Color.yellow, Color.green, Color.blue };
    [SerializeField] private float isoLineWidth = 0.1f;

    [Header("Slice View Settings")]
    [SerializeField] private bool useSliceView = false;
    [SerializeField] private Vector3 sliceNormal = Vector3.up;
    [SerializeField] private float sliceOffset = 0f;
    [SerializeField] private float sliceThickness = 1f;

    [Header("Interactive Query")]
    [SerializeField] private bool enableInteractiveQuery = true;
    [SerializeField] private KeyCode queryKey = KeyCode.Mouse0;
    private Vector3 lastQueryPoint;
    private float lastQueryDistance;
    private Vector3 lastQueryGradient;

    [Header("Debug Info")]
    [SerializeField] private bool showDebugInfo = true;
    [SerializeField] private int totalNodesVisualized = 0;
    [SerializeField] private float averageError = 0f;

    private NavigationOctreeAuthoring octreeAuthoring;
    private EntityManager entityManager;
    private Entity octreeEntity;

    private void Awake()
    {
        octreeAuthoring = GetComponent<NavigationOctreeAuthoring>();

        // Initialize gradient if not set
        if (distanceGradient == null || distanceGradient.colorKeys.Length == 0)
        {
            distanceGradient = new Gradient();
            GradientColorKey[] colorKeys = new GradientColorKey[]
            {
                new GradientColorKey(Color.red, 0f),      // Inside/at surface
                new GradientColorKey(Color.yellow, 0.25f), // Close
                new GradientColorKey(Color.green, 0.5f),   // Medium
                new GradientColorKey(Color.blue, 0.75f),   // Far
                new GradientColorKey(Color.white, 1f)      // Very far
            };
            GradientAlphaKey[] alphaKeys = new GradientAlphaKey[]
            {
                new GradientAlphaKey(1f, 0f),
                new GradientAlphaKey(1f, 1f)
            };
            distanceGradient.SetKeys(colorKeys, alphaKeys);
        }
    }

    private void Start()
    {
        var world = World.DefaultGameObjectInjectionWorld;
        if (world != null)
        {
            entityManager = world.EntityManager;
        }
    }

    private void Update()
    {
        if (!enableVisualization || entityManager == null)
            return;

        // Handle interactive queries
        if (enableInteractiveQuery && Input.GetKeyDown(queryKey))
        {
            QueryPointUnderMouse();
        }
    }

    private void OnDrawGizmos()
    {
        if (!enableVisualization || !Application.isPlaying || entityManager == null)
            return;

        // Find octree entity
        if (octreeEntity == Entity.Null)
        {
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
                return;
            }
        }

        // Get octree components
        if (!entityManager.HasComponent<NavigationOctree>(octreeEntity))
            return;

        var octree = entityManager.GetComponentData<NavigationOctree>(octreeEntity);
        var nodes = entityManager.GetBuffer<NavigationOctreeNode>(octreeEntity);
        var fieldData = entityManager.GetBuffer<NavigationFieldData>(octreeEntity);

        // Visualize based on selected mode
        switch (visualizationMode)
        {
            case VisualizationMode.DistanceField:
                VisualizeDistanceField(octree, nodes, fieldData);
                break;
            case VisualizationMode.Gradient:
                VisualizeGradients(octree, nodes, fieldData);
                break;
            case VisualizationMode.IsoSurfaces:
                VisualizeIsoSurfaces(octree, nodes, fieldData);
                break;
            case VisualizationMode.ErrorMetric:
                VisualizeErrorMetric(octree, nodes);
                break;
            case VisualizationMode.InsideOutside:
                VisualizeInsideOutside(octree, nodes, fieldData);
                break;
            case VisualizationMode.SliceView:
                VisualizeSlice(octree, nodes, fieldData);
                break;
            case VisualizationMode.NavigationTiers:
                VisualizeNavigationTiers(octree, nodes);
                break;
        }

        // Draw interactive query result
        if (enableInteractiveQuery && lastQueryDistance != float.MaxValue)
        {
            DrawQueryResult();
        }

        // Draw debug info
        if (showDebugInfo)
        {
            DrawDebugInfo(octree, nodes);
        }
    }

    private void VisualizeDistanceField(NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodes, DynamicBuffer<NavigationFieldData> fieldData)
    {
        int nodesVisualized = 0;
        float3 viewerPos = transform.position;

        for (int i = 0; i < nodes.Length; i++)
        {
            var node = nodes[i];

            // Skip non-leaf nodes
            if (!node.IsLeaf)
                continue;

            // Distance culling
            if (math.distance(node.Center, viewerPos) > visualizationRadius)
                continue;

            // Check slice view
            if (useSliceView && !IsInSlice(node.Center, node.HalfExtent))
                continue;

            // Get field data
            if (node.FieldDataIndex < 0 || node.FieldDataIndex >= fieldData.Length)
                continue;

            var field = fieldData[node.FieldDataIndex];
            float distance = field.SignedDistance;

            // Skip negative distances if disabled
            if (!showNegativeDistances && distance < 0)
                continue;

            // Map distance to color
            float t = Mathf.InverseLerp(minDistance, maxDistance, distance);
            Color color = distanceGradient.Evaluate(t);

            // Draw node
            Gizmos.color = color;
            DrawNode(node);

            nodesVisualized++;
        }

        totalNodesVisualized = nodesVisualized;
    }

    private void VisualizeGradients(NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodes, DynamicBuffer<NavigationFieldData> fieldData)
    {
        float3 viewerPos = transform.position;
        Gizmos.color = gradientColor;

        for (int i = 0; i < nodes.Length; i += gradientSkip)
        {
            var node = nodes[i];

            if (!node.IsLeaf)
                continue;

            if (math.distance(node.Center, viewerPos) > visualizationRadius)
                continue;

            if (useSliceView && !IsInSlice(node.Center, node.HalfExtent))
                continue;

            if (node.FieldDataIndex < 0 || node.FieldDataIndex >= fieldData.Length)
                continue;

            var field = fieldData[node.FieldDataIndex];
            float3 gradient = field.GradientField;

            // Draw gradient arrow
            if (math.lengthsq(gradient) > 0.01f)
            {
                DrawArrow(node.Center, gradient * arrowScale * node.HalfExtent);
            }
        }
    }

    private void VisualizeIsoSurfaces(NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodes, DynamicBuffer<NavigationFieldData> fieldData)
    {
        float3 viewerPos = transform.position;

        for (int isoIndex = 0; isoIndex < isoValues.Length && isoIndex < isoColors.Length; isoIndex++)
        {
            float isoValue = isoValues[isoIndex];
            Gizmos.color = isoColors[isoIndex];

            for (int i = 0; i < nodes.Length; i++)
            {
                var node = nodes[i];

                if (!node.IsLeaf)
                    continue;

                if (math.distance(node.Center, viewerPos) > visualizationRadius)
                    continue;

                if (node.FieldDataIndex < 0 || node.FieldDataIndex >= fieldData.Length)
                    continue;

                var field = fieldData[node.FieldDataIndex];

                // Check if iso-surface passes through this node
                if (math.abs(field.SignedDistance - isoValue) < node.HalfExtent)
                {
                    // Draw node boundary to indicate iso-surface
                    Gizmos.DrawWireCube(node.Center, Vector3.one * node.HalfExtent * 2f);
                }
            }
        }
    }

    private void VisualizeErrorMetric(NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodes)
    {
        float3 viewerPos = transform.position;
        float totalError = 0f;
        int count = 0;

        for (int i = 0; i < nodes.Length; i++)
        {
            var node = nodes[i];

            if (!node.IsLeaf)
                continue;

            if (math.distance(node.Center, viewerPos) > visualizationRadius)
                continue;

            // Use variance as error metric
            float error = node.Variance;
            totalError += error;
            count++;

            // Color based on error
            float t = Mathf.Clamp01(error / 5f); // Normalize to 0-5 range
            Gizmos.color = Color.Lerp(Color.green, Color.red, t);

            DrawNode(node);
        }

        if (count > 0)
        {
            averageError = totalError / count;
        }
    }

    private void VisualizeInsideOutside(NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodes, DynamicBuffer<NavigationFieldData> fieldData)
    {
        float3 viewerPos = transform.position;

        for (int i = 0; i < nodes.Length; i++)
        {
            var node = nodes[i];

            if (!node.IsLeaf)
                continue;

            if (math.distance(node.Center, viewerPos) > visualizationRadius)
                continue;

            if (node.FieldDataIndex < 0 || node.FieldDataIndex >= fieldData.Length)
                continue;

            var field = fieldData[node.FieldDataIndex];

            // Binary coloring
            Gizmos.color = field.SignedDistance < 0 ? Color.red : Color.green;
            DrawNode(node);
        }
    }

    private void VisualizeSlice(NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodes, DynamicBuffer<NavigationFieldData> fieldData)
    {
        // Draw slice plane
        Gizmos.color = new Color(1f, 1f, 1f, 0.1f);
        Vector3 planeCenter = transform.position + sliceNormal * sliceOffset;
        DrawPlane(planeCenter, sliceNormal, visualizationRadius);

        // Visualize nodes in slice
        VisualizeDistanceField(octree, nodes, fieldData);
    }

    private void VisualizeNavigationTiers(NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodes)
    {
        float3 viewerPos = transform.position;
        Color[] tierColors = { Color.blue, Color.yellow, Color.red };

        for (int i = 0; i < nodes.Length; i++)
        {
            var node = nodes[i];

            if (!node.IsLeaf)
                continue;

            if (math.distance(node.Center, viewerPos) > visualizationRadius)
                continue;

            // Color by tier
            Gizmos.color = tierColors[(int)node.Tier];
            DrawNode(node);
        }
    }

    private void DrawNode(NavigationOctreeNode node)
    {
        Gizmos.DrawWireCube(node.Center, Vector3.one * node.HalfExtent * 2f);
    }

    private void DrawArrow(Vector3 origin, Vector3 direction)
    {
        Gizmos.DrawLine(origin, origin + direction);

        // Draw arrowhead
        Vector3 right = Vector3.Cross(direction, Vector3.up).normalized * 0.2f;
        Vector3 up = Vector3.Cross(direction, right).normalized * 0.2f;

        Vector3 tip = origin + direction;
        Vector3 arrowBase = tip - direction.normalized * 0.3f;

        Gizmos.DrawLine(tip, arrowBase + right);
        Gizmos.DrawLine(tip, arrowBase - right);
        Gizmos.DrawLine(tip, arrowBase + up);
        Gizmos.DrawLine(tip, arrowBase - up);
    }

    private void DrawPlane(Vector3 center, Vector3 normal, float size)
    {
        Vector3 tangent = Vector3.Cross(normal, Vector3.up);
        if (tangent.magnitude < 0.01f)
            tangent = Vector3.Cross(normal, Vector3.forward);
        tangent.Normalize();

        Vector3 bitangent = Vector3.Cross(normal, tangent).normalized;

        int gridSize = 10;
        float step = size * 2f / gridSize;

        for (int i = 0; i <= gridSize; i++)
        {
            float t = -size + i * step;

            // Draw lines along tangent
            Vector3 start = center + tangent * t - bitangent * size;
            Vector3 end = center + tangent * t + bitangent * size;
            Gizmos.DrawLine(start, end);

            // Draw lines along bitangent
            start = center - tangent * size + bitangent * t;
            end = center + tangent * size + bitangent * t;
            Gizmos.DrawLine(start, end);
        }
    }

    private bool IsInSlice(float3 point, float halfExtent)
    {
        float3 planePoint = (float3)transform.position + (float3)(sliceNormal * sliceOffset);
        float distance = math.dot(point - planePoint, (float3)sliceNormal);
        return math.abs(distance) <= sliceThickness + halfExtent;
    }

    private void QueryPointUnderMouse()
    {
        Ray ray = Camera.main.ScreenPointToRay(Input.mousePosition);
        RaycastHit hit;

        if (Physics.Raycast(ray, out hit, 1000f))
        {
            QueryPoint(hit.point);
        }
    }

    private void QueryPoint(Vector3 point)
    {
        if (entityManager == null || octreeEntity == Entity.Null)
            return;

        var octree = entityManager.GetComponentData<NavigationOctree>(octreeEntity);
        var nodes = entityManager.GetBuffer<NavigationOctreeNode>(octreeEntity);
        var fieldData = entityManager.GetBuffer<NavigationFieldData>(octreeEntity);

        // Find node containing point
        int nodeIndex = NavigationOctreeQueries.GetNodeContaining(point, octree, nodes);

        if (nodeIndex >= 0 && nodeIndex < nodes.Length)
        {
            var node = nodes[nodeIndex];

            if (node.FieldDataIndex >= 0 && node.FieldDataIndex < fieldData.Length)
            {
                var field = fieldData[node.FieldDataIndex];

                lastQueryPoint = point;
                lastQueryDistance = field.SignedDistance;
                lastQueryGradient = field.GradientField;

                Debug.Log($"SDF Query at {point}: Distance={lastQueryDistance:F3}, Gradient={lastQueryGradient}, Tier={node.Tier}");
            }
        }
    }

    private void DrawQueryResult()
    {
        // Draw query point
        Gizmos.color = lastQueryDistance < 0 ? Color.red : Color.green;
        Gizmos.DrawWireSphere(lastQueryPoint, 0.5f);

        // Draw distance text (requires Handles in editor)
#if UNITY_EDITOR
        UnityEditor.Handles.Label(lastQueryPoint + Vector3.up * 2f,
            $"Distance: {lastQueryDistance:F2}\nGradient: {lastQueryGradient}");
#endif

        // Draw gradient
        if (math.lengthsq(lastQueryGradient) > 0.01f)
        {
            Gizmos.color = Color.cyan;
            DrawArrow(lastQueryPoint, lastQueryGradient * 2f);
        }
    }

    private void DrawDebugInfo(NavigationOctree octree, DynamicBuffer<NavigationOctreeNode> nodes)
    {
#if UNITY_EDITOR
        Vector3 infoPos = transform.position + new Vector3(0, 10f, 0);
        string info = $"Octree SDF Debug Info\n" +
                     $"Total Nodes: {nodes.Length}\n" +
                     $"Nodes Visualized: {totalNodesVisualized}\n" +
                     $"Max Depth: {octree.MaxDepth}\n" +
                     $"Mode: {visualizationMode}\n";

        if (visualizationMode == VisualizationMode.ErrorMetric)
        {
            info += $"Average Error: {averageError:F3}\n";
        }

        UnityEditor.Handles.Label(infoPos, info);
#endif
    }
}