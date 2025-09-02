using System.Runtime.CompilerServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;

// Compatibility layer to maintain existing ISpatialQueryCollector interface
public unsafe struct OctreeSpatialDatabaseCompatibility
{
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void QueryAABB<T>(in OctreeSpatialDatabase octree,
        in DynamicBuffer<OctreeNode> nodesBuffer, in DynamicBuffer<SpatialObject> objectsBuffer,
        float3 center, float3 halfExtents, ref T collector)
        where T : unmanaged, ISpatialQueryCollector
    {
        float3 queryMin = center - halfExtents;
        float3 queryMax = center + halfExtents;
        
        CompatibilityCollector<T> compatCollector = new CompatibilityCollector<T>
        {
            OriginalCollector = collector
        };
        
        // Use dynamic operations instead of old UnsafeList operations
        OctreeDynamicOperations.QueryAABB(in octree, nodesBuffer, objectsBuffer, queryMin, queryMax, ref compatCollector);
        
        // Update the original collector with results
        collector = compatCollector.OriginalCollector;
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void QueryAABBCellProximityOrder<T>(in OctreeSpatialDatabase octree,
        in DynamicBuffer<OctreeNode> nodesBuffer, in DynamicBuffer<SpatialObject> objectsBuffer,
        float3 center, float3 halfExtents, ref T collector)
        where T : unmanaged, ISpatialQueryCollector
    {
        // For now, use the same implementation as regular AABB query
        // In the future, this could be optimized to process nodes in proximity order
        QueryAABB(in octree, in nodesBuffer, in objectsBuffer, center, halfExtents, ref collector);
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void AddToDynamicDatabase(ref OctreeSpatialDatabase octree,
        DynamicBuffer<OctreeNode> nodesBuffer, DynamicBuffer<SpatialObject> objectsBuffer,
        in SpatialDatabaseElement element)
    {
        // Convert old SpatialDatabaseElement to new SpatialObject
        SpatialObject spatialObject = new SpatialObject
        {
            Entity = element.Entity,
            Position = element.Position,
            ShapeType = CollisionShapeType.Point,
            HalfExtents = float3.zero,
            Team = element.Team,
            Type = element.Type
        };
        
        // Use the dynamic operations that support subdivision
        OctreeDynamicOperations.AddObject(ref octree, nodesBuffer, objectsBuffer, spatialObject);
    }
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void AddToDatabase(ref OctreeSpatialDatabase octree,
        ref UnsafeList<OctreeNode> nodesBuffer, ref UnsafeList<SpatialObject> objectsBuffer,
        in SpatialDatabaseElement element)
    {
        // DEPRECATED: This method should not be used anymore - use AddToDynamicDatabase instead
        // The old UnsafeList-based operations cause stack overflow due to subdivision issues
        UnityEngine.Debug.LogError("AddToDatabase called - this should not happen! Use OctreeDynamicOperations instead.");
    }
}

// Adapter to convert between IOctreeQueryCollector and ISpatialQueryCollector
public struct CompatibilityCollector<T> : IOctreeQueryCollector 
    where T : unmanaged, ISpatialQueryCollector
{
    public T OriginalCollector;
    
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void OnVisitObject(in SpatialObject obj, out bool shouldEarlyExit)
    {
        // Convert SpatialObject back to the old format for compatibility
        SpatialDatabaseElement element = new SpatialDatabaseElement
        {
            Entity = obj.Entity,
            Position = obj.Position,
            Team = obj.Team,
            Type = obj.Type
        };
        
        // Create a fake cell with a single element for the old interface
        SpatialDatabaseCell fakeCell = new SpatialDatabaseCell
        {
            StartIndex = 0,
            ElementsCount = 1,
            ElementsCapacity = 1,
            ExcessElementsCount = 0
        };
        
        // Create a temporary list with just this element
        UnsafeList<SpatialDatabaseElement> tempList = new UnsafeList<SpatialDatabaseElement>(1, Allocator.Temp);
        tempList.Add(element);
        
        OriginalCollector.OnVisitCell(in fakeCell, in tempList, out shouldEarlyExit);
        
        tempList.Dispose();
    }
}

// Cached access structures for octree (similar to the uniform grid ones)
public unsafe struct CachedOctreeSpatialDatabase
{
    public Entity SpatialDatabaseEntity;
    public ComponentLookup<OctreeSpatialDatabase> OctreeDatabaseLookup;
    public BufferLookup<OctreeNode> NodesBufferLookup;
    public BufferLookup<SpatialObject> ObjectsBufferLookup;

    public bool _IsInitialized;
    public OctreeSpatialDatabase _OctreeDatabase;
    public UnsafeList<OctreeNode> _NodesBuffer;
    public UnsafeList<SpatialObject> _ObjectsBuffer;

    public void CacheData()
    {
        if (!_IsInitialized)
        {
            _OctreeDatabase = OctreeDatabaseLookup[SpatialDatabaseEntity];
            // DISABLED: Do not create UnsafeList wrappers as they cause stack overflow in subdivision
            // Use OctreeDynamicOperations with DynamicBuffers directly instead
            // DynamicBuffer<OctreeNode> nodesBuffer = NodesBufferLookup[SpatialDatabaseEntity];
            // DynamicBuffer<SpatialObject> objectsBuffer = ObjectsBufferLookup[SpatialDatabaseEntity];
            // _NodesBuffer = new UnsafeList<OctreeNode>((OctreeNode*)nodesBuffer.GetUnsafePtr(), nodesBuffer.Length);
            // _ObjectsBuffer = new UnsafeList<SpatialObject>((SpatialObject*)objectsBuffer.GetUnsafePtr(), objectsBuffer.Length);
            _IsInitialized = true;
        }
    }
}

public unsafe struct CachedOctreeSpatialDatabaseUnsafe
{
    public Entity SpatialDatabaseEntity;
    [NativeDisableParallelForRestriction]
    [NativeDisableContainerSafetyRestriction]
    public ComponentLookup<OctreeSpatialDatabase> OctreeDatabaseLookup;
    [NativeDisableParallelForRestriction]
    [NativeDisableContainerSafetyRestriction]
    public BufferLookup<OctreeNode> NodesBufferLookup;
    [NativeDisableParallelForRestriction]
    [NativeDisableContainerSafetyRestriction]
    public BufferLookup<SpatialObject> ObjectsBufferLookup;

    public bool _IsInitialized;
    public OctreeSpatialDatabase _OctreeDatabase;
    public UnsafeList<OctreeNode> _NodesBuffer;
    public UnsafeList<SpatialObject> _ObjectsBuffer;

    public void CacheData()
    {
        if (!_IsInitialized)
        {
            _OctreeDatabase = OctreeDatabaseLookup[SpatialDatabaseEntity];
            // DISABLED: Do not create UnsafeList wrappers as they cause stack overflow in subdivision
            // Use OctreeDynamicOperations with DynamicBuffers directly instead
            // DynamicBuffer<OctreeNode> nodesBuffer = NodesBufferLookup[SpatialDatabaseEntity];
            // DynamicBuffer<SpatialObject> objectsBuffer = ObjectsBufferLookup[SpatialDatabaseEntity];
            // _NodesBuffer = new UnsafeList<OctreeNode>((OctreeNode*)nodesBuffer.GetUnsafePtr(), nodesBuffer.Length);
            // _ObjectsBuffer = new UnsafeList<SpatialObject>((SpatialObject*)objectsBuffer.GetUnsafePtr(), objectsBuffer.Length);
            _IsInitialized = true;
        }
    }
}

public unsafe struct CachedOctreeSpatialDatabaseRO
{
    public Entity SpatialDatabaseEntity;
    [ReadOnly]
    public ComponentLookup<OctreeSpatialDatabase> OctreeDatabaseLookup;
    [ReadOnly]
    public BufferLookup<OctreeNode> NodesBufferLookup;
    [ReadOnly]
    public BufferLookup<SpatialObject> ObjectsBufferLookup;

    public bool _IsInitialized;
    public OctreeSpatialDatabase _OctreeDatabase;
    public UnsafeList<OctreeNode> _NodesBuffer;
    public UnsafeList<SpatialObject> _ObjectsBuffer;

    public void CacheData()
    {
        if (!_IsInitialized)
        {
            _OctreeDatabase = OctreeDatabaseLookup[SpatialDatabaseEntity];
            DynamicBuffer<OctreeNode> nodesBuffer = NodesBufferLookup[SpatialDatabaseEntity];
            DynamicBuffer<SpatialObject> objectsBuffer = ObjectsBufferLookup[SpatialDatabaseEntity];
            _NodesBuffer = new UnsafeList<OctreeNode>((OctreeNode*)nodesBuffer.GetUnsafeReadOnlyPtr(), nodesBuffer.Length);
            _ObjectsBuffer = new UnsafeList<SpatialObject>((SpatialObject*)objectsBuffer.GetUnsafeReadOnlyPtr(), objectsBuffer.Length);
            _IsInitialized = true;
        }
    }
}