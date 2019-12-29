using System.Collections;
using System.Collections.Generic;
using Unity.Entities;
using Unity.Physics;
using UnityEngine;
using Unity.Mathematics;
using Collider = Unity.Physics.Collider;
using SphereCollider = Unity.Physics.SphereCollider;
using BoxCollider = Unity.Physics.BoxCollider;
using CapsuleCollider = Unity.Physics.CapsuleCollider;
using System;
using Unity.Collections;

public class ColliderService
{
    public static BlobAssetReference<Collider> CreateCollider(UnityEngine.Mesh mesh, ColliderType type) {
        switch (type) {
            case ColliderType.Sphere: {
                    Bounds bounds = mesh.bounds;
                    return SphereCollider.Create(new SphereGeometry {
                        Center = bounds.center,
                        Radius = math.cmax(bounds.extents)
                    });
                }
            case ColliderType.Triangle: {
                    return PolygonCollider.CreateTriangle(mesh.vertices[0], mesh.vertices[1], mesh.vertices[2]);
                }
            case ColliderType.Quad: {
                    // We assume the first 2 triangles of the mesh are a quad with a shared edge
                    // Work out a correct ordering for the triangle
                    int[] orderedIndices = new int[4];

                    // Find the vertex in first triangle that is not on the shared edge
                    for (int i = 0; i < 3; i++) {
                        if ((mesh.triangles[i] != mesh.triangles[3]) &&
                            (mesh.triangles[i] != mesh.triangles[4]) &&
                            (mesh.triangles[i] != mesh.triangles[5])) {
                            // Push in order or prev, unique, next
                            orderedIndices[0] = mesh.triangles[(i - 1 + 3) % 3];
                            orderedIndices[1] = mesh.triangles[i];
                            orderedIndices[2] = mesh.triangles[(i + 1) % 3];
                            break;
                        }
                    }

                    // Find the vertex in second triangle that is not on a shared edge
                    for (int i = 3; i < 6; i++) {
                        if ((mesh.triangles[i] != orderedIndices[0]) &&
                            (mesh.triangles[i] != orderedIndices[1]) &&
                            (mesh.triangles[i] != orderedIndices[2])) {
                            orderedIndices[3] = mesh.triangles[i];
                            break;
                        }
                    }

                    return PolygonCollider.CreateQuad(
                        mesh.vertices[orderedIndices[0]],
                        mesh.vertices[orderedIndices[1]],
                        mesh.vertices[orderedIndices[2]],
                        mesh.vertices[orderedIndices[3]]);
                }
            case ColliderType.Box: {
                    Bounds bounds = mesh.bounds;
                    return BoxCollider.Create(new BoxGeometry {
                        Center = bounds.center,
                        Orientation = quaternion.identity,
                        Size = 2.0f * bounds.extents,
                        BevelRadius = 0.0f
                    });
                }
            case ColliderType.Capsule: {
                    Bounds bounds = mesh.bounds;
                    float min = math.cmin(bounds.extents);
                    float max = math.cmax(bounds.extents);
                    int x = math.select(math.select(2, 1, min == bounds.extents.y), 0, min == bounds.extents.x);
                    int z = math.select(math.select(2, 1, max == bounds.extents.y), 0, max == bounds.extents.x);
                    int y = math.select(math.select(2, 1, (1 != x) && (1 != z)), 0, (0 != x) && (0 != z));
                    float radius = bounds.extents[y];
                    float3 vertex0 = bounds.center; vertex0[z] = -(max - radius);
                    float3 vertex1 = bounds.center; vertex1[z] = (max - radius);
                    return CapsuleCollider.Create(new CapsuleGeometry {
                        Vertex0 = vertex0,
                        Vertex1 = vertex1,
                        Radius = radius
                    });
                }
            case ColliderType.Cylinder:
                // TODO: need someone to add
                throw new NotImplementedException();
            case ColliderType.Convex: {
                    NativeArray<float3> points = new NativeArray<float3>(mesh.vertices.Length, Allocator.TempJob);
                    for (int i = 0; i < mesh.vertices.Length; i++) {
                        points[i] = mesh.vertices[i];
                    }
                    BlobAssetReference<Collider> collider = ConvexCollider.Create(points, default, CollisionFilter.Default);
                    points.Dispose();
                    return collider;
                }
            default:
                throw new System.NotImplementedException();
        }
    }
}
