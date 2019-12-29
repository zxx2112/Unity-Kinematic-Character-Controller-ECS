using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Transforms;
using UnityEngine;
using Unity.Mathematics;

public class EntityTracker : MonoBehaviour, IReceiveEntity
{
    [SerializeField] bool TrackPosition = true;
    [SerializeField] bool TrackRotation = true;

    private Entity EntityToTrack = Entity.Null;
    public void SetReceivedEntity(Entity entity)
    {
        EntityToTrack = entity;
    }

    // Update is called once per frame
    void LateUpdate()
    {
        if (EntityToTrack != Entity.Null)
        {
            try
            {
                var entityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
                var localToWorld = entityManager.GetComponentData<LocalToWorld>(EntityToTrack);
                var worldPosition = localToWorld.Position;
                if (float.IsNaN(worldPosition.x) || float.IsNaN(worldPosition.y) || float.IsNaN(worldPosition.z)) {
                    worldPosition = float3.zero;
                }

                if (TrackPosition)
                    transform.position = worldPosition;
                if(TrackRotation)
                    transform.rotation = localToWorld.Rotation;
            }
            catch
            {
                // Dirty way to check for an Entity that no longer exists.
                EntityToTrack = Entity.Null;
            }
        }
    }
}
