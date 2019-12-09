using System;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

[Serializable]
public struct CharacterSpawner : IComponentData
{
    public Entity EntityPrefab;
    public float AreaSize;
    public int Count;
}

public class CharacterSpawnerAuthoring : MonoBehaviour, IConvertGameObjectToEntity,IDeclareReferencedPrefabs
{
    public GameObject characterPrefab;

    public float areaSize = 1;
    public int count = 1;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, new CharacterSpawner() {
            EntityPrefab = conversionSystem.GetPrimaryEntity(characterPrefab),
            AreaSize = areaSize,
            Count = count
        }); ;
    }

    public void DeclareReferencedPrefabs(List<GameObject> referencedPrefabs) {
        referencedPrefabs.Add(characterPrefab);
    }

    private void OnDrawGizmos() {
        var color = Color.green;
        color.a = 0.5f;
        Gizmos.color = color;
        Gizmos.DrawCube(transform.position,new Vector3(areaSize,1,areaSize));
    }
}
