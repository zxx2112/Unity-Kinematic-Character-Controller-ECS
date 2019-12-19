using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using static CharacterControllerUtilities;
using UnityEngine;

/// <summary>
/// 角色控制器的一些配置数据
/// </summary>
[Serializable]
public struct CharacterControllerComponentData : IComponentData
{
    public float3 GroundProbeVector;
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
    public float3 CapsuleRadius;
    public float3 CapsuleHeight;
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

