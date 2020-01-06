using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;


[UpdateInGroup(typeof(PresentationSystemGroup))]
public class SyncCharacterProxyPositionAndRotationSystem : ComponentSystem
{

    EntityQuery CharacterControllerQuery;

    protected override void OnCreate() {
        CharacterControllerQuery = GetEntityQuery(typeof(CharacterControllerComponentData),typeof(LocalToWorld),typeof(Translation),typeof(UserCommand));
    }
    protected override void OnUpdate() {
        Entities.ForEach(
            (Entity entity, Transform transform, ref CharacterProxy proxy) => {
                if(CharacterControllerQuery.CalculateEntityCount() > 0) {

                    var localToWorldArray = CharacterControllerQuery.ToComponentDataArray<LocalToWorld>(Allocator.TempJob);
                    var translationArray = CharacterControllerQuery.ToComponentDataArray<Translation>(Allocator.TempJob);
                    var userCommandArray = CharacterControllerQuery.ToComponentDataArray<UserCommand>(Allocator.TempJob);

                    transform.position = translationArray[0].Value;
                    //var x = proxy.SyncPitchRotation ? math.radians(90 - userCommandArray[0].lookPitch) : 0;
                    //var y = proxy.SyncYawRotation ? math.radians(userCommandArray[0].lookYaw):0;
                    //transform.rotation = quaternion.Euler(new float3(x,y, 0));
                    transform.rotation = localToWorldArray[0].Rotation;

                    localToWorldArray.Dispose();
                    translationArray.Dispose();
                    userCommandArray.Dispose();
                }
            });
    }
}