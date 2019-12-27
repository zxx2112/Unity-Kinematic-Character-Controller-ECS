using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[Serializable]
public struct DebugComponent : IComponentData
{
    public float3 CurrentVelocity;
    public float3 SolvedVelocity;
}
