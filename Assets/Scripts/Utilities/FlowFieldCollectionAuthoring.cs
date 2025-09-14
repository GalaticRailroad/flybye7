using Unity.Entities;
using UnityEngine;

// Authoring component to create a collection of flow fields
public class FlowFieldCollectionAuthoring : MonoBehaviour
{
    [Header("Flow Field References")]
    public GameObject ChaseFieldPrefab;
    public GameObject FleeFieldPrefab;
    public GameObject PatrolFieldPrefab;
    public GameObject AmbushFieldPrefab;
    
    class Baker : Baker<FlowFieldCollectionAuthoring>
    {
        public override void Bake(FlowFieldCollectionAuthoring authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.None);
            
            AddComponent(entity, new FlowFieldCollection
            {
                ChaseFieldEntity = authoring.ChaseFieldPrefab ? GetEntity(authoring.ChaseFieldPrefab, TransformUsageFlags.None) : Entity.Null,
                FleeFieldEntity = authoring.FleeFieldPrefab ? GetEntity(authoring.FleeFieldPrefab, TransformUsageFlags.None) : Entity.Null,
                PatrolFieldEntity = authoring.PatrolFieldPrefab ? GetEntity(authoring.PatrolFieldPrefab, TransformUsageFlags.None) : Entity.Null,
                AmbushFieldEntity = authoring.AmbushFieldPrefab ? GetEntity(authoring.AmbushFieldPrefab, TransformUsageFlags.None) : Entity.Null
            });
        }
    }
}