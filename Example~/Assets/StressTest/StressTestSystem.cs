using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;

public class StressTestSystem : ComponentSystem
{
    public struct InitedTag : ISystemStateComponentData
    {

    }

    EntityQuery CharacterSpawnerQuery;

    float minCooldown = 3;
    float maxCooldown = 5;

    protected override void OnCreate() {
        CharacterSpawnerQuery = GetEntityQuery(typeof(CharacterSpawner),ComponentType.Exclude<InitedTag>());
    }

    protected override void OnUpdate() {
        var random = new Random((uint)UnityEngine.Time.frameCount);

        Entities.With(CharacterSpawnerQuery).ForEach((Entity ent, ref CharacterSpawner spawner) => {
            for (int i = 0; i < spawner.Count; i++) {
                var character = EntityManager.Instantiate(spawner.EntityPrefab);
                var randomPosition = random.NextFloat3(new float3(-1,0,-1),new float3(1,0,1)) * spawner.AreaSize/2;
                randomPosition.y = 1.5f;
                EntityManager.SetComponentData(character, new Translation { Value = randomPosition });
            }

            EntityManager.AddComponentData(ent, new InitedTag());
        });


        Entities.ForEach((Entity ent,ref StressTestComponent test ,ref CharacterControllerInternalData internalData) => {
            test.cooldown -= Time.DeltaTime;

            if (test.cooldown < 0) {
                test.cooldown = random.NextFloat(minCooldown,maxCooldown);

                var randomDirection = random.NextFloat3(new float3(-1,0,-1),new float3(1,0,1));
                randomDirection.y = 0;
                internalData.Input.LookInputVector = randomDirection;
                internalData.Input.Movement = new float2(0, 1);
            }
        });
    }
}