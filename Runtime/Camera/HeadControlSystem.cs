using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using UnityEngine;

//控制头部本地旋转
public class HeadControlSystem : ComponentSystem
{
    protected override void OnUpdate() {
        Entities.ForEach(
            (Entity entity,ref Head head ,ref Translation translation, ref Rotation rotation) => {
                //var currentYaw = ((Quaternion)rotation).eulerAngles.y;

                var currentPitch = ((Quaternion)rotation.Value).eulerAngles.x;

                currentPitch -= Input.GetAxisRaw("Mouse Y") * head.RotationSpeed;

                rotation.Value = Quaternion.Euler(currentPitch, 0, 0);
            }
        );
    }
}