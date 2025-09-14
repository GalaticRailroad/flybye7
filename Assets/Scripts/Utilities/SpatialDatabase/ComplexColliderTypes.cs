using Unity.Entities;
using Unity.Mathematics;

// Extended collision shape types for complex geometry
public enum CollisionShapeType : byte
{
    Point = 0,
    AABB = 1,
    Sphere = 2,
    Mesh = 3,           // Mesh colliders with detailed geometry
    Compound = 4,       // Compound colliders (multiple primitives)
    ConvexHull = 5,     // Simplified convex hull approximation
    OrientedBox = 6,    // Oriented bounding box (better than AABB)
}

// Collision detail levels for hierarchical detection
public enum CollisionDetailLevel : byte
{
    Coarse = 0,     // AABB/Sphere only (fastest)
    Medium = 1,     // Convex hulls and OBB (moderate speed)
    Fine = 2,       // Full mesh/compound geometry (slowest)
}

// Extended spatial object with multi-level collision data
[InternalBufferCapacity(0)]
public struct SpatialObjectMesh : IBufferElementData
{
    public Entity Entity;
    public float3 Position;
    public CollisionShapeType ShapeType;
    
    // Level 1: Coarse bounds (always available)
    public float3 BoundsMin;
    public float3 BoundsMax;
    
    // Level 2: Medium detail (optional)
    public Entity ConvexHullEntity;    // Reference to convex hull data
    public quaternion Rotation;       // For oriented shapes
    
    // Level 3: Fine detail (optional)
    public Entity MeshDataEntity;     // Reference to detailed mesh data
    
    public byte Team;
    public byte Type;
    public byte DetailLevels;         // Bitmask: which levels are available
}

// Convex hull data for medium-detail collision
public struct ConvexHullData : IComponentData
{
    public int VertexCount;
    public int FaceCount;
    public float3 Center;
    public float MaxRadius;
}

// References to convex hull geometry buffers
[InternalBufferCapacity(8)]
public struct ConvexHullVertex : IBufferElementData
{
    public float3 Position;
}

[InternalBufferCapacity(12)]
public struct ConvexHullFace : IBufferElementData
{
    public float3 Normal;
    public float Distance;      // Distance from origin along normal
    public int3 VertexIndices; // Triangle indices
}

// Mesh collision data for fine-detail collision
public struct MeshColliderData : IComponentData
{
    public int VertexCount;
    public int TriangleCount;
    public float3 BoundsMin;
    public float3 BoundsMax;
    public bool IsConvex;
}

// Mesh geometry buffers (only for fine-detail queries)
[InternalBufferCapacity(0)]
public struct MeshVertex : IBufferElementData
{
    public float3 Position;
    public float3 Normal;   // For surface-based steering
}

[InternalBufferCapacity(0)] 
public struct MeshTriangle : IBufferElementData
{
    public int3 VertexIndices;
    public float3 FaceNormal;
    public float3 EdgeNormals; // Perpendicular to each edge for avoidance
}