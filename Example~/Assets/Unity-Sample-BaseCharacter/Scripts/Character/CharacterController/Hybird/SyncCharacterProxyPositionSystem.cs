using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Mathematics.math;
using UnityEngine;


[UpdateInGroup(typeof(PresentationSystemGroup))]
public class SyncCharacterProxyPositionSystem : ComponentSystem
{

    EntityQuery CharacterControllerQuery;

    protected override void OnCreate() {
        CharacterControllerQuery = GetEntityQuery(typeof(CharacterControllerComponentData),typeof(Translation));
    }
    protected override void OnUpdate() {
        Entities.ForEach(
            (Entity entity, Transform transform, ref CharacterProxy proxy) => {
                if(CharacterControllerQuery.CalculateEntityCount() > 0) {

                    var translationArray = CharacterControllerQuery.ToComponentDataArray<Translation>(Allocator.TempJob);

                    transform.position = translationArray[0].Value;

                    translationArray.Dispose();
                }
            });
    }
}