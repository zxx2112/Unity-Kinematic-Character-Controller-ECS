using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

[UpdateBefore(typeof(CharacterControllerStepSystem))]
public class ActiveUpdateSystem : ComponentSystem
{
    protected override void OnUpdate() {
        Entities.ForEach((Entity entity, ref CharacterControllerMoveQuery moveQuery, ref LocalToWorld localToWorld) => {
            moveQuery.StartPosition = localToWorld.Position;
        });
    }
}