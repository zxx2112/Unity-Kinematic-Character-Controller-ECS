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
        Entities.ForEach((Entity entity,ref UserCommand userCommand,ref CharacterControllerMoveQuery moveQuery, ref CharacterControllerVelocity velocity,ref Translation translation) => {
            
            var newVelocity = CalculateVelocity(Time.DeltaTime, userCommand,velocity);

            velocity.WorldVelocity = newVelocity;

            moveQuery.StartPosition = translation.Value;
        });
    }

    private float3 CalculateVelocity(float deltaTime, UserCommand command,CharacterControllerVelocity lastVelocity) {

        float playerSpeed = 5;
        float acceleration = 15;
        var friction = 12;

        var velocity = lastVelocity.WorldVelocity;

        //velocity.y = 0;
        //InAir
        {
            var playerGravity = 9.81f;
            var gravity = playerGravity;
            velocity += (float3)Vector3.down * gravity * deltaTime;
        }


        velocity = CalculateGroundVelocity(deltaTime, velocity, command,playerSpeed, friction, acceleration);

        return velocity;
    }

    /// <summary>
    /// 加入平台速度,暂时没有此功能
    /// </summary>
    /// <param name="velocity"></param>
    /// <param name="command"></param>
    /// <returns></returns>
    private Vector3 CalculateGroundVelocity(float deltaTime, Vector3 velocity, UserCommand command, float playerSpeed, float friction, float acceleration) {
        var moveYawRotation = Quaternion.Euler(0, command.lookYaw + command.moveYaw, 0);
        var moveVec = moveYawRotation * Vector3.forward * command.moveMagnitude;

        //摩擦力,会让角色逐渐停止
        var groundVelocity = new Vector3(velocity.x, 0, velocity.z);
        var groundSpeed = groundVelocity.magnitude;
        var frictionSpeed = Mathf.Max(groundSpeed, 1.0f) * deltaTime * friction;
        var newGroundSpeed = groundSpeed - frictionSpeed;
        if (newGroundSpeed < 0)
            newGroundSpeed = 0;
        if (groundSpeed > 0)
            groundVelocity *= (newGroundSpeed / groundSpeed);

        var wantedGroundVelocity = moveVec * playerSpeed;
        var wantedGroundDir = wantedGroundVelocity.normalized;
        var currentSpeed = Vector3.Dot(wantedGroundDir, groundVelocity);
        var wantedSpeed = playerSpeed * command.moveMagnitude;
        var deltaSpeed = wantedSpeed - currentSpeed;
        if (deltaSpeed > 0.0f) {
            var accel = deltaTime * acceleration * playerSpeed;
            var speed_adjustment = Mathf.Clamp(accel, 0.0f, deltaSpeed) * wantedGroundDir;
            groundVelocity += speed_adjustment;
        }

        velocity.x = groundVelocity.x;
        velocity.z = groundVelocity.z;

        return velocity;
    }
}