using System.Runtime.CompilerServices;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

public struct OctreePlanetAssessmentCollector : IOctreeQueryCollector
{
    public DynamicBuffer<PlanetShipsAssessment> ShipsAssessmentBuffer;

    public OctreePlanetAssessmentCollector(int querierTeam, DynamicBuffer<PlanetShipsAssessment> shipsAssessmentBuffer)
    {
        ShipsAssessmentBuffer = shipsAssessmentBuffer;
    } 

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void OnVisitObject(in SpatialObject obj, out bool shouldEarlyExit)
    {
        shouldEarlyExit = false;

        int elementTeam = (int)obj.Team;
        PlanetShipsAssessment planetShipsAssessment = ShipsAssessmentBuffer[elementTeam];
        switch (obj.Type)
        {
            case ActorType.FighterType:
                planetShipsAssessment.FighterCount++;
                break;
            case ActorType.WorkerType:
                planetShipsAssessment.WorkerCount++;
                break;
            case ActorType.TraderType:
                planetShipsAssessment.TraderCount++;
                break;
        }
        ShipsAssessmentBuffer[elementTeam] = planetShipsAssessment;
    }
}