using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using static Unity.Mathematics.math;

public class GameTimeSystem : ComponentSystem
{
    GameTime worldTime;
    Entity globalTimeEntiy;
    protected override void OnCreate() {
        base.OnCreate();
        globalTimeEntiy = EntityManager.CreateEntity(typeof(GlobalGameTime));
        worldTime = new GameTime(60);
    }
    protected override void OnUpdate() {
        
    }
}