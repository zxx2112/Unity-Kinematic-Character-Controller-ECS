using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using UnityEngine;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;
using Unity.Physics;
using Math = System.Math;

public class CharacterControllerDebug : MonoBehaviour
{
    [SerializeField] private Mesh mesh = null;

    //public static List<ColliderCastInput> inputs = new List<ColliderCastInput>();
    //public static List<ColliderCastHit> hits = new List<ColliderCastHit>();

    public static ColliderCastInput input;
    public static ColliderCastHit hit;

    EntityQuery CharacterControllerQuery;


    bool Simulating;

    private void Start() {
        CharacterControllerQuery = World.DefaultGameObjectInjectionWorld.EntityManager.CreateEntityQuery(
            typeof(CharacterControllerComponentData),
            typeof(LocalToWorld),
            typeof(CharacterControllerInitializationData));

        Simulating = true;
    }

    private void OnDrawGizmos() {
        if (!Simulating) return;

        if (CharacterControllerQuery.CalculateEntityCount() == 0) return;

        var localToWorldArray = CharacterControllerQuery.ToComponentDataArray<LocalToWorld>(Allocator.TempJob);
        var initDataArray = CharacterControllerQuery.ToComponentDataArray<CharacterControllerInitializationData>(Allocator.TempJob);
        var localToWorld = localToWorldArray[0];
        var initData = initDataArray[0];

        try
        {
            //开始查询位置
            Gizmos.color = Color.red;
            Gizmos.DrawLine(input.Start, input.End);
            Gizmos.color = new Color(0.94f, 0.35f, 0.15f, 0.75f);
            Gizmos.DrawWireMesh(mesh, localToWorld.Position + initData.CapsuleCenter, quaternion.identity);

            //击中位置
            if (true) {
                Gizmos.color = Color.magenta;
                Gizmos.DrawSphere(hit.Position, 0.02f);
                // Gizmos.DrawWireMesh(mesh,
                //     math.lerp(input.Start, input.End, hit.Fraction),
                //     input.Orientation
                // );
            }
            else {
                if(Math.Abs(input.Orientation.value.w) < 0.001f)
                    input.Orientation = quaternion.identity;
            
                Gizmos.DrawWireMesh(mesh,
                    input.End,
                    input.Orientation
                );
            }

        }
        catch (Exception e)
        {
            Console.WriteLine(e);
            throw;
        }
        finally
        {
            localToWorldArray.Dispose();
            initDataArray.Dispose();
        }





    }
}
