using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Mathematics.math;
using UnityEngine;


[UpdateInGroup(typeof(PresentationSystemGroup))]
public class SyncCharacterProxyPositionAndRotationSystem : ComponentSystem
{

    EntityQuery CharacterControllerQuery;

    protected override void OnCreate() {
        CharacterControllerQuery = GetEntityQuery(typeof(CharacterControllerComponentData),typeof(Translation),typeof(UserCommand));
    }
    protected override void OnUpdate() {
        Entities.ForEach(
            (Entity entity, Transform transform, ref CharacterProxy proxy) => {
                if(CharacterControllerQuery.CalculateEntityCount() > 0) {

                    var translationArray = CharacterControllerQuery.ToComponentDataArray<Translation>(Allocator.TempJob);
                    var userCommandArray = CharacterControllerQuery.ToComponentDataArray<UserCommand>(Allocator.TempJob);

                    transform.position = translationArray[0].Value;
                    transform.rotation =  Quaternion.LookRotation(Quaternion.Euler(-userCommandArray[0].lookPitch,0,0) * new Vector3(0,-1f,0),Vector3.up);

                    translationArray.Dispose();
                    userCommandArray.Dispose();
                }
            });
    }
}