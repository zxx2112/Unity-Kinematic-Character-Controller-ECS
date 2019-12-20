using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Physics;
using Unity.Physics.Systems;

[DisableAutoCreation]
[AlwaysSynchronizeSystem]
[AlwaysUpdateSystem]
public class ChracterControllerCheckSupportSystem : JobComponentSystem
{
    BuildPhysicsWorld m_BuildPhysicsWorldSystem;
    EntityQuery m_GameTimeSingletonQuery;

    protected override void OnCreate() {
        m_BuildPhysicsWorldSystem = World.GetOrCreateSystem<BuildPhysicsWorld>();
        m_GameTimeSingletonQuery = GetEntityQuery(ComponentType.ReadOnly<GlobalGameTime>());
    }

    /// <summary>
    /// 需要unsafe来使用指针
    /// </summary>
    protected unsafe override JobHandle OnUpdate(JobHandle inputDependencies)
    {
        var physicsWorld = m_BuildPhysicsWorldSystem.PhysicsWorld;
        var time = m_GameTimeSingletonQuery.GetSingleton<GlobalGameTime>().gameTime;
        //var predictingTick = World.GetExistingSystem<GhostPredictionSystemGroup>().PredictingTick; 预测

        var constraints = new NativeList<SurfaceConstraintInfo>(Allocator.Temp);//表面约束信息?
        var castHits = new NativeList<ColliderCastHit>(Allocator.Temp);//射线击中信息?

        var deltaTime = Time.DeltaTime;
        Entities
            .WithName("CheckSupportJob")
            .ForEach((
                ref CharacterControllerComponentData ccData,
                ref CharacterControllerMoveQuery ccQuery,
                ref CharacterControllerMoveResult resultPosition,
                ref CharacterControllerVelocity velocity,
                ref CharacterControllerCollider ccCollider,
                ref CharacterControllerGroundSupportData ccGroundData
                //in PredictedGhostComponent predictedGhostComponent
                ) => {
                    //if(!GhostPredictionSystemGroup.ShouldPredict(PredictingTick,predictedGhostComponent))
                    //    return;

                    //查询到的信息写入状态信息
                    if (!ccQuery.CheckSupport) {
                        ccGroundData.SupportedState = CharacterControllerUtilities.CharacterSupportState.Unsupported;
                        ccGroundData.SurfaceVelocity = float3.zero;
                        ccGroundData.SurfaceNormal = float3.zero;
                    }

                    constraints.Clear();
                    castHits.Clear();

                    var collider = (Unity.Physics.Collider*)ccCollider.Collider.GetUnsafePtr();

                    var stepInput = new CharacterControllerUtilities.CharacterControllerStepInput {
                        World = physicsWorld,
                        //DeltaTime = time.tickInterval,
                        DeltaTime = deltaTime,
                        Up = math.up(),//设定角色永远头朝上
                        Gravity = new float3(0.0f, -9.8f, 0.0f),
                        MaxIterations = ccData.MaxIterations,
                        Tau = CharacterControllerUtilities.k_DefaultTau,
                        Damping = CharacterControllerUtilities.k_DefaultDamping,
                        SkinWidth = ccData.SkinWidth,
                        ContactTolerance = ccData.ContactTolearance * 2.0f,
                        MaxSlope = ccData.MaxSlope,
                        RigidBodyIndex = -1,
                        CurrentVelocity = velocity.Velocity,
                        MaxMovementSpeed = ccData.MaxMovementSpeed
                    };

                    var transform = new RigidTransform {
                        pos = resultPosition.MoveResult,
                        rot = quaternion.identity
                    };
                    //FollowGround为了解决上下坡穿入的问题
                    float probeFactor = ccQuery.FollowGroud ? 2 : 1;

                    CharacterControllerUtilities.CheckSupport(
                        ref physicsWorld,
                        collider,
                        stepInput,
                        ccData.GroundProbeVector * probeFactor,
                        transform,
                        ccData.MaxSlope,
                        ref constraints,
                        ref castHits,
                        out ccGroundData.SupportedState,
                        out ccGroundData.SurfaceNormal,
                        out ccGroundData.SurfaceVelocity);
                }).Run();


        constraints.Dispose();
        castHits.Dispose();

        return inputDependencies;
    }
}