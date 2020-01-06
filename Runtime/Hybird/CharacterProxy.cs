using System;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

[Serializable]
public struct CharacterProxy : IComponentData
{
    public bool SyncPitchRotation;
    public bool SyncYawRotation;
}
