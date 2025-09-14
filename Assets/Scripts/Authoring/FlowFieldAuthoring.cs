using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public class FlowFieldAuthoring : MonoBehaviour
{
    [Header("Flow Field Configuration")]
    public FlowFieldType FieldType = FlowFieldType.Chase;
    
    [Header("Grid Settings")]
    public int3 Resolution = new int3(64, 32, 64); // X, Y, Z resolution
    public float CellSize = 2f;
    public float3 BoundsMin = new float3(-50, 0, -50);
    
    [Header("Targets (for Chase/Flee)")]
    public GameObject ChaseTarget;
    public GameObject FleeFromTarget;
    
    [Header("Generation Settings")]
    public bool GenerateOnStart = true;
    public bool RegenerateWhenTargetMoves = true;
    public float RegenerationInterval = 1f; // Seconds between updates
    
    void Start()
    {
        // Calculate bounds max from min + resolution * cellSize
        float3 boundsMax = BoundsMin + new float3(Resolution) * CellSize;
        
        // Visualize bounds in editor
        Debug.DrawLine(new float3(BoundsMin.x, BoundsMin.y, BoundsMin.z), 
                      new float3(boundsMax.x, BoundsMin.y, BoundsMin.z), Color.green, 60f);
        Debug.DrawLine(new float3(BoundsMin.x, BoundsMin.y, BoundsMin.z), 
                      new float3(BoundsMin.x, boundsMax.y, BoundsMin.z), Color.red, 60f);
        Debug.DrawLine(new float3(BoundsMin.x, BoundsMin.y, BoundsMin.z), 
                      new float3(BoundsMin.x, BoundsMin.y, boundsMax.z), Color.blue, 60f);
    }
    
    class Baker : Baker<FlowFieldAuthoring>
    {
        public override void Bake(FlowFieldAuthoring authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.None);
            
            // Calculate bounds
            float3 boundsMax = authoring.BoundsMin + new float3(authoring.Resolution) * authoring.CellSize;
            
            // Add flow field component
            AddComponent(entity, new FlowField
            {
                BoundsMin = authoring.BoundsMin,
                BoundsMax = boundsMax,
                Resolution = authoring.Resolution,
                CellSize = authoring.CellSize,
                FieldType = authoring.FieldType
            });
            
            // Add flow cell buffer (will be populated by generation system)
            int totalCells = authoring.Resolution.x * authoring.Resolution.y * authoring.Resolution.z;
            DynamicBuffer<FlowCell> cellBuffer = AddBuffer<FlowCell>(entity);
            cellBuffer.ResizeUninitialized(totalCells);
            
            // Initialize cells with empty data
            for (int i = 0; i < totalCells; i++)
            {
                cellBuffer[i] = new FlowCell
                {
                    Velocity = float3.zero,
                    DistanceToObstacle = float.MaxValue,
                    Flags = 0
                };
            }
            
            // Add generation control component
            AddComponent(entity, new FlowFieldGenerationControl
            {
                GenerateOnStart = authoring.GenerateOnStart,
                RegenerateWhenTargetMoves = authoring.RegenerateWhenTargetMoves,
                RegenerationInterval = authoring.RegenerationInterval,
                LastGenerationTime = 0f,
                ChaseTarget = authoring.ChaseTarget ? GetEntity(authoring.ChaseTarget, TransformUsageFlags.Dynamic) : Entity.Null,
                FleeFromTarget = authoring.FleeFromTarget ? GetEntity(authoring.FleeFromTarget, TransformUsageFlags.Dynamic) : Entity.Null,
                NeedsRegeneration = authoring.GenerateOnStart
            });
        }
    }
}

// Component to control when flow fields should be generated/regenerated
public struct FlowFieldGenerationControl : IComponentData
{
    public bool GenerateOnStart;
    public bool RegenerateWhenTargetMoves;
    public float RegenerationInterval;
    public float LastGenerationTime;
    public Entity ChaseTarget;
    public Entity FleeFromTarget;
    public bool NeedsRegeneration;
    public float3 LastChaseTargetPosition;
    public float3 LastFleeTargetPosition;
}