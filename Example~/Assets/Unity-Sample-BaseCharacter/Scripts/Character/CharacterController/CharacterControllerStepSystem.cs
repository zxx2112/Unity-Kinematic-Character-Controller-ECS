using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Physics;
using Unity.Physics.Extensions;
using Unity.Physics.Systems;
using Unity.Transforms;

[AlwaysSynchronizeSystem]
public class CharacterControllerStepSystem : JobComponentSystem
{
    /// <summary>
    /// 应用冲量到其他接触的刚体
    /// </summary>
    [BurstCompile]
    struct ApplyDefferedImpulses : IJob
    {
        public NativeStream.Reader DeferredImpulseReader;

        public ComponentDataFromEntity<PhysicsVelocity> PhysicsVelocityData;
        public ComponentDataFromEntity<PhysicsMass> PhysicsMassData;
        public ComponentDataFromEntity<Translation> TranslationData;
        public ComponentDataFromEntity<Rotation> RotationData;

        public void Execute() {
            int index = 0;
            int maxIndex = DeferredImpulseReader.ForEachCount;
            DeferredImpulseReader.BeginForEachIndex(index++);
            while(DeferredImpulseReader.RemainingItemCount == 0 && index < maxIndex) {
                DeferredImpulseReader.BeginForEachIndex(index++);
            }

            while(DeferredImpulseReader.RemainingItemCount > 0) {
                var impulse = DeferredImpulseReader.Read<DefferedCharacterControllerImpulse>();
                while(DeferredImpulseReader.RemainingItemCount == 0 && index < maxIndex) {
                    DeferredImpulseReader.BeginForEachIndex(index++);
                }

                PhysicsVelocity pv = PhysicsVelocityData[impulse.Entity];
                PhysicsMass pm = PhysicsMassData[impulse.Entity];
                Translation t = TranslationData[impulse.Entity];
                Rotation r = RotationData[impulse.Entity];

                if(pm.InverseMass > 0.0f) {
                    pv.ApplyImpulse(pm, t, r, impulse.Impulse, impulse.Point);

                    PhysicsVelocityData[impulse.Entity] = pv;
                }
            }
        }
    }


    BuildPhysicsWorld m_BuildPhysicsWorld;

    EntityQuery m_CharacterControllerGroup;
    EntityQuery m_TimeSingletonQuery;

    protected override void OnCreate() {
        m_BuildPhysicsWorld = World.GetOrCreateSystem<BuildPhysicsWorld>();
        m_CharacterControllerGroup = GetEntityQuery(new EntityQueryDesc {
            All = new[]
            {
                ComponentType.ReadWrite<CharacterControllerComponentData>(),
            }
        });
        m_TimeSingletonQuery = GetEntityQuery(ComponentType.ReadOnly<GlobalGameTime>());
    }

    protected unsafe override JobHandle OnUpdate(JobHandle inputDeps) {

        var entityCount = m_CharacterControllerGroup.CalculateEntityCount();
        if (entityCount == 0)
            return inputDeps;

        var defferredImpulses = new NativeStream(entityCount, Allocator.TempJob);
        var time = m_TimeSingletonQuery.GetSingleton<GlobalGameTime>().gameTime;
        var physicWorld = m_BuildPhysicsWorld.PhysicsWorld;

        var writer = defferredImpulses.AsWriter();

        var constraints = new NativeList<SurfaceConstraintInfo>(Allocator.Temp);
        var castHits = new NativeList<ColliderCastHit>(Allocator.Temp);
        var distanceHits = new NativeList<DistanceHit>(Allocator.Temp);

        var input = new ColliderCastInput();
        var hit = new ColliderCastHit();

        //var predictingTick = World.GetExistingSystem<GhostPredictionSystemGrouo>().PredictingTick;

        var deltaTime = Time.DeltaTime;
        Entities
            .WithName("CharacterControllerStepSystem")
            .ForEach((
                ref CharacterControllerComponentData ccData,
                ref CharacterControllerCollider ccCollider,
                ref CharacterControllerMoveQuery moveQuery,
                ref CharacterControllerMoveResult moveResult,
                ref CharacterControllerVelocity velocity
                //in PredictedGhostComponent predictedGhostComponent
                ) => 
            {
                //if(!GhostPredictionSystemGroup.ShouldPredict(PredictingTick,predictedGhostComponent))
                //  return;

                var collider = (Collider*)ccCollider.Collider.GetUnsafePtr();

                var stepInput = new CharacterControllerUtilities.CharacterControllerStepInput {
                    World = physicWorld,
                    //DeltaTime = time.tickDuration,
                    DeltaTime = deltaTime,
                    Gravity = new float3(0.0f, -9.8f, 0.0f),
                    MaxIterations = ccData.MaxIterations,
                    Tau = CharacterControllerUtilities.k_DefaultTau,
                    Damping = CharacterControllerUtilities.k_DefaultDamping,
                    SkinWidth = ccData.SkinWidth,
                    ContactTolerance = ccData.ContactTolearance,
                    MaxSlope = ccData.MaxSlope,
                    RigidBodyIndex = -1,
                    CurrentVelocity = velocity.Velocity,
                    MaxMovementSpeed = ccData.MaxMovementSpeed,
                    FollowGroud = moveQuery.FollowGroud
                };

                var transform = new RigidTransform {
                    pos = moveQuery.StartPosition,
                    rot = quaternion.identity
                };

            CharacterControllerUtilities.CollideAndIntegrate(
                stepInput,
                ccData.CharacterMass,
                ccData.AffectsPhysicsBodies > 0,
                collider,
                ref transform,
                ref velocity.Velocity,
                ref writer,
                ref constraints,
                ref castHits,
                ref distanceHits,
                out input,
                out hit);

                moveResult.MoveResult = transform.pos;
            }).Run();

        var applyJob = new ApplyDefferedImpulses() {
            DeferredImpulseReader = defferredImpulses.AsReader(),
            PhysicsVelocityData = GetComponentDataFromEntity<PhysicsVelocity>(),
            PhysicsMassData = GetComponentDataFromEntity<PhysicsMass>(),
            TranslationData = GetComponentDataFromEntity<Translation>(),
            RotationData = GetComponentDataFromEntity<Rotation>()
        };
        applyJob.Run();

        CharacterControllerDebug.input = input;
        CharacterControllerDebug.hit = hit;

        defferredImpulses.Dispose();
        constraints.Dispose();
        castHits.Dispose();
        distanceHits.Dispose();
    
        return inputDeps;
    }
}