using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

public struct MeshContainerComponent : ISharedComponentData,IEquatable<MeshContainerComponent>
{
    public Mesh Mesh;

    public bool Equals(MeshContainerComponent other)
    {
        return Equals(Mesh, other.Mesh);
    }

    public override bool Equals(object obj)
    {
        return obj is MeshContainerComponent other && Equals(other);
    }

    public override int GetHashCode()
    {
        return (Mesh != null ? Mesh.GetHashCode() : 0);
    }
}

public class MeshContainerAuthoring : MonoBehaviour,IConvertGameObjectToEntity
{
    [SerializeField] private Mesh mesh = null;
    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem)
    {
        dstManager.AddSharedComponentData(entity, new MeshContainerComponent()
        {
            Mesh = mesh
        });
    }
}
