using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Mathematics.math;
using UnityEngine;

[UpdateBefore(typeof(ActiveUpdateSystem))] //在应用输入写读取输入

public class InputSystem : ComponentSystem
{

    protected override void OnCreate() {
        
    }

    protected override void OnUpdate() {

        Entities.ForEach((Entity entity, ref UserCommand command) => {

            float invertY = 1;
            var deltaTime = Time.DeltaTime;

            Vector2 deltaMousePos = new Vector2(0, 0);
            if (deltaTime > 0.0f)
                deltaMousePos += new Vector2(Input.GetAxisRaw("Mouse X"), Input.GetAxisRaw("Mouse Y") * invertY);

            //command.lookPitch += deltaMousePos.y * configMouseSensitivity.FloatValue;
            command.lookPitch += deltaMousePos.y * 1.5f;
            command.lookPitch = Mathf.Clamp(command.lookPitch, 0, 180);
        });

    }
}