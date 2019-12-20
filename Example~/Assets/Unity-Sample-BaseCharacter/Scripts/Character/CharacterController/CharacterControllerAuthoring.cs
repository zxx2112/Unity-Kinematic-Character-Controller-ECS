using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using static CharacterControllerUtilities;
using UnityEngine;
using Unity.Jobs;
using Unity.Physics;

/// <summary>
/// 角色控制器的一些配置数据
/// </summary>
[Serializable]
public struct CharacterControllerComponentData : IComponentData
{
    public float3 GroundProbeVector;
    public float MaxSlope; //弧度制
    public int MaxIterations;
    public float CharacterMass;
    public float SkinWidth;
    public float ContactTolearance;
    public int AffectsPhysicsBodies;//影响的刚体数量
    public float MaxMovementSpeed;
}

/// <summary>
/// 地面状态
/// </summary>
public struct CharacterControllerGroundSupportData : IComponentData
{
    /// <summary>
    /// 地面法线方向
    /// </summary>
    public float3 SurfaceNormal;
    /// <summary>
    /// 地面移动速度
    /// </summary>
    public float3 SurfaceVelocity;
    /// <summary>
    /// 地面支持状态
    /// </summary>
    public CharacterSupportState SupportedState;
}

/// <summary>
/// 角色控制器移动查询
/// </summary>
public struct CharacterControllerMoveQuery : IComponentData
{
    public float3 StartPosition;
    public bool FollowGroud;
    public bool CheckSupport;
}

/// <summary>
/// 角色控制器移动结果
/// </summary>
public struct CharacterControllerMoveResult : IComponentData
{
    public float3 MoveResult;
}


/// <summary>
/// 角色控制器当前移动速度
/// </summary>
public struct CharacterControllerVelocity : IComponentData
{
    public float3 Velocity;
}

/// <summary>
/// 角色控制器初始化数据
/// </summary>
public struct CharacterControllerInitializationData : IComponentData
{
    public float3 CapsuleCenter;
    public float CapsuleRadius;
    public float CapsuleHeight;
}

/// <summary>
/// 角色控制器碰撞体,引用实际的碰撞体
/// </summary>
struct CharacterControllerCollider : ISystemStateComponentData
{
    public BlobAssetReference<Unity.Physics.Collider> Collider;
}


[Serializable]
public class CharacterControllerAuthoring : MonoBehaviour, IConvertGameObjectToEntity
{
    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        if(enabled) {
            dstManager.AddComponentData(entity, new CharacterControllerComponentData {
                GroundProbeVector = new float3(0,-1,0)
            });

            dstManager.AddComponentData(entity, new CharacterControllerInitializationData { 
                CapsuleCenter = new float3(0,1,0),
                CapsuleRadius = 0.5f,
                CapsuleHeight = 2.0f
            });

            dstManager.AddComponent(entity, typeof(CharacterControllerVelocity));
            dstManager.AddComponent(entity, typeof(CharacterControllerMoveQuery));
            dstManager.AddComponent(entity, typeof(CharacterControllerMoveResult));
            dstManager.AddComponent(entity, typeof(CharacterControllerGroundSupportData));
        }
    }
}

[AlwaysSynchronizeSystem]
[AlwaysUpdateSystem]
public class CharacterControllerInitAndCleanupSystem : JobComponentSystem
{
    protected override JobHandle OnUpdate(JobHandle inputDeps) {
        var ecb = new EntityCommandBuffer(Allocator.TempJob);

        //初始化角色控制器碰撞体(碰撞体为手动分配的内存,也需要手动释放)
        Entities
            .WithNone<CharacterControllerCollider>()
            .ForEach((Entity e,ref CharacterControllerInitializationData initData) => {
                //根据两点一半径构建胶囊几何体
                var capsule = new CapsuleGeometry {
                    Vertex0 = initData.CapsuleCenter + new float3(0, 0.5f * initData.CapsuleHeight - initData.CapsuleRadius, 0),
                    Vertex1 = initData.CapsuleCenter - new float3(0, 0.5f * initData.CapsuleHeight - initData.CapsuleRadius, 0),
                    Radius = initData.CapsuleRadius
                };
                //1为 1 << 0 第0层
                var filter = new CollisionFilter { BelongsTo = 1, CollidesWith = 1, GroupIndex = 0 };
                var collider = Unity.Physics.CapsuleCollider.Create(capsule, filter, new Unity.Physics.Material { Flags = new Unity.Physics.Material.MaterialFlags() });
                ecb.AddComponent(e, new CharacterControllerCollider { Collider = collider });
            }).Run();//使用Run来单线程运行?

        Entities
            .WithNone<CharacterControllerComponentData>()
            .ForEach((Entity e, ref CharacterControllerCollider collider) => {
                collider.Collider.Dispose();
                ecb.RemoveComponent<CharacterControllerCollider>(e);
            }).Run();

        ecb.Playback(EntityManager);
        ecb.Dispose();

        return inputDeps;
    }
}

