using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Physics;

[Serializable]
[GenerateAuthoringComponent]
public struct CharacterControllerDebugInfo : IComponentData
{
    public ColliderCastInput input;
    public ColliderCastHit hit;
}
