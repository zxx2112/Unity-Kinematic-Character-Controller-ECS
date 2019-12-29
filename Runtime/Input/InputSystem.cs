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

            var mouseSensitivity = 1.5f;

            command.lookYaw += deltaMousePos.x * mouseSensitivity;
            command.lookYaw = command.lookYaw % 360;
            while (command.lookYaw < 0.0f) command.lookYaw += 360.0f;

            command.lookPitch += deltaMousePos.y * mouseSensitivity;
            command.lookPitch = Mathf.Clamp(command.lookPitch, 0, 180);

            var horizontalInput = Input.GetAxis("Horizontal");
            var verticalInput = Input.GetAxis("Vertical");
            
            Vector2 moveInput = new Vector2(horizontalInput, verticalInput);
            float angle = Vector2.Angle(Vector2.up, moveInput);
            if (moveInput.x < 0)
                angle = 360 - angle;
            float magnitude = Mathf.Clamp(moveInput.magnitude, 0, 1);

            command.moveYaw = angle;
            command.moveMagnitude = magnitude;
        });

    }
}