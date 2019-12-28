using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

[UpdateBefore(typeof(CharacterControllerStepSystem))]
public class ActiveUpdateSystem : ComponentSystem
{
    protected override void OnUpdate() {
        Entities.ForEach((Entity entity, ref CharacterControllerMoveQuery moveQuery, ref CharacterControllerVelocity velocity,ref Translation translation) => {
            var horizontal = Input.GetAxis("Horizontal");
            var vertical = Input.GetAxis("Vertical");

            var horizontalVelocity = new float3(horizontal, 0, vertical) * 10f;
            var verticalVelocity = new float3(0, velocity.Velocity.y - Time.DeltaTime * 9.81f, 0);

            velocity.Velocity = horizontalVelocity + verticalVelocity;

            moveQuery.StartPosition = translation.Value;
            //Debug.Log($"Set StartPosition为{moveQuery.StartPosition}");
        });
    }
}