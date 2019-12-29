using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Mathematics.math;
using UnityEngine;

[DisableAutoCreation]
public class TempCharacrerControllerSystem : ComponentSystem
{
    protected override void OnUpdate() {
        Entities.ForEach((Entity entity, ref CharacterControllerComponentData ccData, ref Translation translation) => {

            var vertical = Input.GetAxis("Vertical");
            var speed = 1;
            translation.Value.y += Time.DeltaTime * speed * vertical;
        });
    }
}