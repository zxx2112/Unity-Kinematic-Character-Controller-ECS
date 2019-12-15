//using System.Collections;
//using System.Collections.Generic;
//using UnityEngine;
//using Unity.Entities;
//using Unity.Mathematics;

//public class LookInputVectorUpdate : MonoBehaviour
//{
//    [SerializeField] Transform Camera;

//    EntityManager EntityManager;
//    EntityQuery CharacterControllerInternalDataQuery;
//    void Start()
//    {
//        EntityManager = World.DefaultGameObjectInjectionWorld.EntityManager;
//        CharacterControllerInternalDataQuery = EntityManager.CreateEntityQuery(typeof(CharacterControllerInternalData));
//    }

//    void Update()
//    {
//        if (Camera == null) return;

//        var entities = CharacterControllerInternalDataQuery.ToEntityArray(Unity.Collections.Allocator.TempJob);
//        var datas = CharacterControllerInternalDataQuery.ToComponentDataArray<CharacterControllerInternalData>(Unity.Collections.Allocator.TempJob);

//        for (int i = 0; i < entities.Length; i++) {
//            var data = datas[i];
//            var _lookInputVector = CameraRotationToLookInputVector(Camera.rotation);
//            data.Input.LookInputVector = _lookInputVector;
//            EntityManager.SetComponentData(entities[i], data);
//        }

//        entities.Dispose();
//        datas.Dispose();
//    }

//    private float3 CameraRotationToLookInputVector(Quaternion cameraRotation) {

//        Vector3 cameraPlanarDirection = Vector3.ProjectOnPlane(cameraRotation * Vector3.forward, Vector3.up).normalized;

//        return cameraPlanarDirection;
//    }
//}
