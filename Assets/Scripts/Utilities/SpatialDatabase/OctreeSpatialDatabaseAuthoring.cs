using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public class OctreeSpatialDatabaseAuthoring : MonoBehaviour
{
    [Header("Octree Configuration")]
    public float3 Center = float3.zero;
    public float HalfExtent = 100f;
    
    [Range(1, 10)]
    public byte MaxDepth = 6;
    
    [Range(1, 50)]
    public byte MaxObjectsPerNode = 10;
    
    [Header("Debug")]
    public bool DebugDrawOctree = false;
    public int DebugMaxDepthToDraw = 3;

    class Baker : Baker<OctreeSpatialDatabaseAuthoring>
    {
        public override void Bake(OctreeSpatialDatabaseAuthoring authoring)
        {
            Entity entity = GetEntity(authoring, TransformUsageFlags.None);

            OctreeSpatialDatabase octree = new OctreeSpatialDatabase();
            DynamicBuffer<OctreeNode> nodesBuffer = AddBuffer<OctreeNode>(entity);
            DynamicBuffer<SpatialObject> objectsBuffer = AddBuffer<SpatialObject>(entity);

            OctreeSpatialDatabase.Initialize(authoring.Center, authoring.HalfExtent, authoring.MaxDepth, 
                authoring.MaxObjectsPerNode, ref octree, nodesBuffer, objectsBuffer);
            AddComponent(entity, octree);
        }
    }

    private void OnDrawGizmosSelected()
    {
        if (DebugDrawOctree)
        {
            DrawOctreeNode(Center, HalfExtent, 0, DebugMaxDepthToDraw);
        }
        else
        {
            // Just draw the bounds
            Gizmos.color = Color.green;
            Gizmos.DrawWireCube(Center, new float3(HalfExtent * 2f));
        }
    }
    
    private void DrawOctreeNode(float3 center, float halfExtent, int depth, int maxDepth)
    {
        if (depth > maxDepth)
            return;
            
        // Set color based on depth
        float t = (float)depth / maxDepth;
        Color color = Color.Lerp(Color.green, Color.red, t);
        color.a = 0.3f;
        Gizmos.color = color;
        
        // Draw this node's bounds
        Gizmos.DrawWireCube(center, new float3(halfExtent * 2f));
        
        if (depth < maxDepth)
        {
            // Draw children
            float childHalfExtent = halfExtent * 0.5f;
            for (int i = 0; i < 8; i++)
            {
                float3 childCenter = OctreeSpatialDatabase.GetChildCenter(center, halfExtent, i);
                DrawOctreeNode(childCenter, childHalfExtent, depth + 1, maxDepth);
            }
        }
    }
}