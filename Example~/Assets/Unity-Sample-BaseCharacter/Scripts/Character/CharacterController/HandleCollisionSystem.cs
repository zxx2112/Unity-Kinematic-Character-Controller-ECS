using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Mathematics.math;

[UpdateAfter(typeof(CharacterControllerStepSystem))]
public class HandleCollisionSystem : ComponentSystem
{
    protected override void OnUpdate() {
        Entities.ForEach((Entity entity,ref CharacterControllerMoveResult moveResult,ref Translation translation) => {
            translation.Value = moveResult.MoveResult;
        });
    }
}