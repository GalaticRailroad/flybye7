using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

public class SteeringBehaviorAuthoring : MonoBehaviour
{
    [Header("Steering Behavior Weights")]
    [Range(0f, 2f)] public float SeekWeight = 0f;
    [Range(0f, 2f)] public float FleeWeight = 0f;
    [Range(0f, 2f)] public float FlockCohesionWeight = 0f;
    [Range(0f, 2f)] public float FlockSeparationWeight = 0f;
    [Range(0f, 2f)] public float FlockAlignmentWeight = 0f;
    [Range(0f, 2f)] public float WanderWeight = 0f;
    [Range(0f, 2f)] public float FlowFieldWeight = 0f;
    [Range(0f, 2f)] public float ObstacleAvoidanceWeight = 1f;
    [Range(0f, 2f)] public float LeaderFollowWeight = 0f;
    [Range(0f, 2f)] public float PathFollowWeight = 0f;
    
    [Header("Movement Parameters")]
    public float MaxSpeed = 10f;
    public float MaxForce = 5f;
    
    [Header("Flocking Parameters")]
    public float SeparationRadius = 2f;
    public float CohesionRadius = 5f;
    public float AlignmentRadius = 3f;
    
    [Header("Wander Parameters")]
    public float WanderRadius = 3f;
    public float WanderDistance = 5f;
    public float WanderJitter = 1f;
    
    [Header("Obstacle Avoidance")]
    public float ObstacleAvoidanceDistance = 5f;
    public CollisionDetailLevel AvoidanceDetailLevel = CollisionDetailLevel.Medium;
    
    [Header("Targets")]
    public GameObject SeekTarget;
    public GameObject FleeTarget;
    public GameObject LeaderTarget;
    public GameObject PathEntity;
    
    [Header("Flow Field Settings")]
    public float ChaseFlowWeight = 1f;
    public float FleeFlowWeight = 0f;
    public float PatrolFlowWeight = 0f;
    public float AmbushFlowWeight = 0f;
    
    [Header("Flock Settings")]
    public int FlockId = 1;
    public float FlockRadius = 10f;
    
    class Baker : Baker<SteeringBehaviorAuthoring>
    {
        public override void Bake(SteeringBehaviorAuthoring authoring)
        {
            Entity entity = GetEntity(TransformUsageFlags.Dynamic);
            
            // Add main steering behavior component
            AddComponent(entity, new SteeringBehaviors
            {
                SeekWeight = authoring.SeekWeight,
                FleeWeight = authoring.FleeWeight,
                FlockCohesionWeight = authoring.FlockCohesionWeight,
                FlockSeparationWeight = authoring.FlockSeparationWeight,
                FlockAlignmentWeight = authoring.FlockAlignmentWeight,
                WanderWeight = authoring.WanderWeight,
                FlowFieldWeight = authoring.FlowFieldWeight,
                ObstacleAvoidanceWeight = authoring.ObstacleAvoidanceWeight,
                LeaderFollowWeight = authoring.LeaderFollowWeight,
                PathFollowWeight = authoring.PathFollowWeight,
                
                MaxSpeed = authoring.MaxSpeed,
                MaxForce = authoring.MaxForce,
                SeparationRadius = authoring.SeparationRadius,
                CohesionRadius = authoring.CohesionRadius,
                AlignmentRadius = authoring.AlignmentRadius,
                WanderRadius = authoring.WanderRadius,
                WanderDistance = authoring.WanderDistance,
                WanderJitter = authoring.WanderJitter,
                ObstacleAvoidanceDistance = authoring.ObstacleAvoidanceDistance,
                
                SeekTarget = authoring.SeekTarget ? GetEntity(authoring.SeekTarget, TransformUsageFlags.Dynamic) : Entity.Null,
                FleeTarget = authoring.FleeTarget ? GetEntity(authoring.FleeTarget, TransformUsageFlags.Dynamic) : Entity.Null,
                LeaderTarget = authoring.LeaderTarget ? GetEntity(authoring.LeaderTarget, TransformUsageFlags.Dynamic) : Entity.Null,
                PathEntity = authoring.PathEntity ? GetEntity(authoring.PathEntity, TransformUsageFlags.Dynamic) : Entity.Null,
                WanderTarget = float3.zero,
                
                FlowFieldWeights = new FlowFieldBlendWeights
                {
                    ChaseWeight = authoring.ChaseFlowWeight,
                    FleeWeight = authoring.FleeFlowWeight,
                    PatrolWeight = authoring.PatrolFlowWeight,
                    AmbushWeight = authoring.AmbushFlowWeight
                },
                AvoidanceDetailLevel = authoring.AvoidanceDetailLevel
            });
            
            // Add flock member component if any flocking behaviors are enabled
            if (authoring.FlockCohesionWeight > 0f || authoring.FlockSeparationWeight > 0f || authoring.FlockAlignmentWeight > 0f)
            {
                AddComponent(entity, new FlockMember
                {
                    FlockId = authoring.FlockId,
                    FlockRadius = authoring.FlockRadius
                });
            }
        }
    }
}