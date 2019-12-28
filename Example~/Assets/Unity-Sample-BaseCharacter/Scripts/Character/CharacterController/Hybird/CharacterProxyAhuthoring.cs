using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Entities;

[ExecuteInEditMode]
public class CharacterProxyAhuthoring : MonoBehaviour,IConvertGameObjectToEntity
{
    [SerializeField] private GameObject characterTarget;

    public void Convert(Entity entity, EntityManager dstManager, GameObjectConversionSystem conversionSystem) {
        dstManager.AddComponentData(entity, new CharacterProxy());
    }

    private void Update() {
        if (characterTarget == null) return;

        if(!Application.isPlaying) {
            transform.SetPositionAndRotation(characterTarget.transform.position,characterTarget.transform.rotation);
        }
    }
}
