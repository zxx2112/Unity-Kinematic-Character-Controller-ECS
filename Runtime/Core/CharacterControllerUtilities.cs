using Unity.Physics;
using Unity.Mathematics;
using Unity.Collections;
using UnityEngine.Assertions;
using Unity.Physics.Extensions;
using Unity.Entities;

public struct DefferedCharacterControllerImpulse
{
    public Entity Entity;
    public float3 Impulse;
    public float3 Point;
}

public static class CharacterControllerUtilities
{
    const float k_SimpleSolverEpsion = 0.0001f;
    const float k_SimplexSolverEpsilonSq = k_SimpleSolverEpsion * k_SimpleSolverEpsion;

    public const float k_DefaultTau = 0.4f;
    public const float k_DefaultDamping = 0.9f;

    public enum CharacterSupportState : byte
    {
        Unsupported = 0,
        Sliding,
        Supported
    }

    /// <summary>
    /// 角色控制器每次解算开始时候的状态
    /// </summary>
    public struct CharacterControllerStepInput
    {
        public PhysicsWorld World;//所在物理世界
        public float DeltaTime;//距离上一帧的时间
        public float3 Gravity;//重力
        public float3 Up;//上方向
        public int MaxIterations;//最大迭代次数(迭代是为了找到有效的避障位置)
        public float Tau;//不清楚
        public float Damping;//阻尼(应该是移动阻尼)
        public float SkinWidth;//皮肤宽度,允许的与其他碰撞体的重叠程度
        public float ContactTolerance;//接触容忍度,距离多远算作接触
        public float MaxSlope;//能行走的最大坡度
        public int RigidBodyIndex;//刚体索引
        public float3 CurrentVelocity;//当前速度
        public float MaxMovementSpeed;//最大移动速度
        public bool FollowGroud;//是否跟随地面
    }

    public struct SelfFilteringAllHitsCollector<T> : ICollector<T> where T : struct,IQueryResult
    {
        private int m_selfRBIndex;

        public bool EarlyOutOnFirstHit => false;//只查询一个碰撞
        public float MaxFraction { get; }
        public int NumHits => AllHits.Length;

        public NativeList<T> AllHits;

        public SelfFilteringAllHitsCollector(int rbIndex,float maxFraction,ref NativeList<T> allHits) {
            MaxFraction = maxFraction;
            AllHits = allHits;
            m_selfRBIndex = rbIndex;
        }

        #region IQueryResult implementation

        public bool AddHit(T hit) {
            Assert.IsTrue(hit.Fraction < MaxFraction);
            AllHits.Add(hit);
            return true;
        }

        public void TransformNewHits(int oldNumHits,float oldFraction,Unity.Physics.Math.MTransform transform,uint numSubKeyBits,uint subKey) {
            for (int i = oldNumHits; i < NumHits; i ++) {
                T hit = AllHits[i];
                hit.Transform(transform, numSubKeyBits, subKey);
                AllHits[i] = hit;
            }

        }

        public void TransformNewHits(int oldNumHits, float oldFraction,Unity.Physics.Math.MTransform transform,int rigidIndex) {
            if(rigidIndex == m_selfRBIndex) {
                for (int i = oldNumHits; i < NumHits; i++) {
                    AllHits.RemoveAtSwapBack(oldNumHits);
                }
                return;
            }

            for (int i = oldNumHits; i < NumHits; i++) {
                T hit = AllHits[i];
                hit.Transform(transform, rigidIndex);
                AllHits[i] = hit;
            }
        }

        #endregion
    }

    public struct SelfFilteringClosestHitCollector<T> : ICollector<T> where T : struct,IQueryResult
    {
        public bool EarlyOutOnFirstHit => false;
        public float MaxFraction { get; private set; }
        public int NumHits { get; private set; }

        private T m_OldHit;
        private T m_ClosestHit;
        public T ClosestHit => m_ClosestHit;

        private int m_selfRBIndex;

        public SelfFilteringClosestHitCollector(int rbIndex, float maxFraction) {
            MaxFraction = maxFraction;
            m_OldHit = default(T);
            m_ClosestHit = default(T);
            NumHits = 0;
            m_selfRBIndex = rbIndex;
        }

        #region ICollector
        public bool AddHit(T hit) {
            Assert.IsTrue(hit.Fraction <= MaxFraction);
            MaxFraction = hit.Fraction;
            m_OldHit = m_ClosestHit;
            m_ClosestHit = hit;
            NumHits = 1;
            return true;
        }

        public void TransformNewHits(int oldNumHits,float oldFraction,Unity.Physics.Math.MTransform transform,uint numbSubKeyBits,uint subKey) {
            if(m_ClosestHit.Fraction < oldFraction) {
                m_ClosestHit.Transform(transform, numbSubKeyBits, subKey);
            }
        }

        public void TransformNewHits(int oldNumHits,float oldFraction,Unity.Physics.Math.MTransform transform,int rigidBodyIndex) {
            //检测到自己,回退
            if(rigidBodyIndex == m_selfRBIndex) {
                m_ClosestHit = m_OldHit;
                NumHits = 0;
                MaxFraction = oldFraction;
                m_OldHit = default(T);
                return;
            }

            if(m_ClosestHit.Fraction < oldFraction) {
                m_ClosestHit.Transform(transform, rigidBodyIndex);
            }
        }
        #endregion
    }
    public static unsafe void CheckSupport(
        ref PhysicsWorld world,ref PhysicsCollider collider,CharacterControllerStepInput stepInput,float3 groundProbeVector,RigidTransform transform,
        float maxSlope,ref NativeList<SurfaceConstraintInfo> constraints,ref NativeList<ColliderCastHit> castHits,out CharacterSupportState characterState,out float3 surfaceNormal,out float3 surfaceVelocity)
    {
        CheckSupport(ref world, collider.ColliderPtr, stepInput, groundProbeVector, transform, maxSlope, ref constraints, ref castHits, out characterState, out surfaceNormal, out surfaceVelocity);
    }

    public static unsafe void CheckSupport(
        ref PhysicsWorld world, Collider* collider, CharacterControllerStepInput stepInput, float3 groundProbeVector, RigidTransform transform,float maxSlope, 
        ref NativeList<SurfaceConstraintInfo> constraints, ref NativeList<ColliderCastHit> castHits, 
        out CharacterSupportState characterState, out float3 surfaceNormal, out float3 surfaceVelocity)
    {
        surfaceNormal = float3.zero;
        surfaceVelocity = float3.zero;

        //查询物理世界
        var displacement = groundProbeVector;
        SelfFilteringAllHitsCollector<ColliderCastHit> hitCollector = new SelfFilteringAllHitsCollector<ColliderCastHit>(stepInput.RigidBodyIndex, 1.0f, ref castHits);
        ColliderCastInput input = new ColliderCastInput() {
            Collider = collider,
            Orientation = transform.rot,
            Start = transform.pos,
            End = transform.pos + displacement
        };
        world.CastCollider(input, ref hitCollector);
        //如果没检测到任何碰撞体,那么可以肯定是非支持状态
        if(hitCollector.NumHits == 0) {
            characterState = CharacterSupportState.Unsupported;
            return;
        }

        float3 downwardsDirection = -stepInput.Up;
        Assert.IsTrue(Unity.Physics.Math.IsNormalized(downwardsDirection));

        float maxSlopCos = math.cos(maxSlope);

        for (int i = 0; i < hitCollector.NumHits; i++) {
            var hit = hitCollector.AllHits[i];
            CreateConstraint(stepInput.World, stepInput.Up,
                hit.RigidBodyIndex, hit.ColliderKey, hit.Position, hit.SurfaceNormal, hit.Fraction * math.length(displacement),
                stepInput.SkinWidth, maxSlopCos, ref constraints);
        }

        float3 initialVelocity = groundProbeVector * (1.0f / stepInput.DeltaTime);//初速度(期望在1秒内到达目标)
        UnityEngine.Debug.Log(initialVelocity.ToString());
        float3 outVelocity = initialVelocity;
        float3 outPosition = transform.pos;
        SimplexSolver.Solve(stepInput.World, stepInput.DeltaTime, stepInput.DeltaTime, stepInput.Up, stepInput.MaxMovementSpeed,
            constraints,ref outPosition, ref outVelocity, out float integratedTime, false);
        UnityEngine.Debug.Log(outVelocity.ToString());
        {
            int numSupportingPlanes = 0;
            for (int i = 0; i < constraints.Length; i++) {
                var constraint = constraints[i];
                if(constraint.Touched && !constraint.IsTooSteep) {
                    numSupportingPlanes ++;
                    surfaceNormal += constraint.Plane.Normal;
                    surfaceVelocity += constraint.Velocity;
                }
            }

            if(numSupportingPlanes > 0) {
                float invNumSupportingPlanes = 1.0f / numSupportingPlanes;
                surfaceNormal *= invNumSupportingPlanes;
                surfaceVelocity *= invNumSupportingPlanes;

                surfaceNormal = math.normalize(surfaceNormal);
            }
        }

        {

            if (math.lengthsq(initialVelocity - outVelocity) < k_SimplexSolverEpsilonSq) {
                //如果速度没有显著改变,那么肯定是未支持状态?
                characterState = CharacterSupportState.Unsupported;
            }
            else if(math.lengthsq(outVelocity) < k_SimplexSolverEpsilonSq) {
                //如果速度非常小,那么肯定是支持状态
                characterState = CharacterSupportState.Supported;
            }
            else {
                //滑行和支持状态
                outVelocity = math.normalize(outVelocity);
                float slopeAngleSin = math.max(0.0f, math.dot(outVelocity, downwardsDirection));
                float slopeAngleCosSq = 1 - slopeAngleSin * slopeAngleSin;
                if(slopeAngleCosSq < maxSlopCos * maxSlopCos) {
                    characterState = CharacterSupportState.Sliding;
                }
                else {
                    characterState = CharacterSupportState.Supported;
                }
            }
        }
    }

    public static unsafe void CollideAndIntegrate(
        CharacterControllerStepInput stepInput,float characterMass,bool affectBodies,Collider* collider,
        ref RigidTransform transform,ref float3 linearVelocity,ref NativeStream.Writer defferredImpulseWriter,
        ref NativeList<SurfaceConstraintInfo> constraints,ref NativeList<ColliderCastHit> castHits, ref NativeList<DistanceHit> distanceHits,out ColliderCastInput debugInput,out ColliderCastHit debugHit)
    {
        float deltaTime = stepInput.DeltaTime;
        float3 up = stepInput.Up;
        PhysicsWorld world = stepInput.World;

        float remaingTIme = deltaTime;

        float3 newPosition = transform.pos;
        quaternion orientation = transform.rot;
        float3 newVelocity = linearVelocity;

        float maxSlopCos = math.cos(stepInput.MaxSlope);

        debugInput = (default);
        debugHit = (default);


        const float timeEpsilon = 0.000001f;
        for (int i = 0; i < stepInput.MaxIterations  && remaingTIme > timeEpsilon; i++) {

            constraints.Clear();
            castHits.Clear();
            distanceHits.Clear();

            //碰撞检测
            {
                float3 displacement = newVelocity * remaingTIme;
                SelfFilteringAllHitsCollector<ColliderCastHit> collector = new SelfFilteringAllHitsCollector<ColliderCastHit>(stepInput.RigidBodyIndex,1.0f, ref castHits);
                ColliderCastInput input = new ColliderCastInput() {
                    Collider = collider,
                    Orientation = orientation,
                    Start = newPosition,
                    End = newPosition + displacement
                };
                if(i == 0) {
                    debugInput = input;
  
                    //UnityEngine.Debug.Log($"{input.End - input.Start}");
                }
                world.CastCollider(input, ref collector);

                for (int hitIndex = 0; hitIndex < collector.NumHits; hitIndex++) {
                    ColliderCastHit hit = collector.AllHits[hitIndex];
                    if(hitIndex == 0) {
                        debugHit = hit;
                    }
                    CreateConstraint(stepInput.World, stepInput.Up,
                        hit.RigidBodyIndex, hit.ColliderKey, hit.Position, hit.SurfaceNormal, hit.Fraction * math.length(displacement),
                        stepInput.SkinWidth, maxSlopCos, ref constraints);
                }
            }

            {
                SelfFilteringAllHitsCollector<DistanceHit> distanceHitsCollector = new SelfFilteringAllHitsCollector<DistanceHit>(
                    stepInput.RigidBodyIndex, stepInput.ContactTolerance, ref distanceHits);
                {
                    ColliderDistanceInput input = new ColliderDistanceInput() {
                        MaxDistance = stepInput.ContactTolerance,
                        Transform = transform,
                        Collider = collider
                    };
                    world.CalculateDistance(input, ref distanceHitsCollector);
                }

                int numConstraints = constraints.Length;
                for (int hitIndex = 0; hitIndex < distanceHitsCollector.NumHits; hitIndex++) {
                    DistanceHit hit = distanceHitsCollector.AllHits[hitIndex];
                    if(hit.Distance < stepInput.SkinWidth) {
                        bool found = false;

                        for (int constraintIndex = numConstraints - 1; constraintIndex >= 0; constraintIndex--) {
                            SurfaceConstraintInfo constraint = constraints[constraintIndex];
                            if(constraint.RigidBodyIndex == hit.RigidBodyIndex &&
                                constraint.ColliderKey.Equals(hit.ColliderKey))
                            {
                                {
                                    CreateConstraintFromHit(world, hit.RigidBodyIndex, hit.ColliderKey,
                                        hit.Position, hit.SurfaceNormal, hit.Distance,
                                        stepInput.SkinWidth, out SurfaceConstraintInfo newConstraint);

                                    ResolveConstraintPenetration(ref newConstraint);

                                    constraints[constraintIndex] = newConstraint;
                                }

                                found = true;
                                break;
                            }
                        }

                        if(!found) {
                            CreateConstraint(stepInput.World, stepInput.Up,
                                hit.RigidBodyIndex, hit.ColliderKey, hit.Position, hit.SurfaceNormal, hit.Distance,
                                stepInput.SkinWidth, maxSlopCos, ref constraints);
                        }
                    }
                }
            }

            float minDeltaTime = 0.0f;
            if(math.lengthsq(newVelocity) > k_SimplexSolverEpsilonSq) {
                minDeltaTime = 0.01f / math.length(newVelocity);
            }

            float3 preVelocity = newVelocity;
            float3 prePosition = newPosition;
            SimplexSolver.Solve(world, remaingTIme, minDeltaTime, up, stepInput.MaxMovementSpeed, constraints, ref newPosition, ref newVelocity, out float integratedTime);
            
            if(affectBodies) {
                CalculateAndStoreDefferedImpulse(stepInput, characterMass, preVelocity, ref constraints, ref defferredImpulseWriter);
            }

            float3 newDisplacement = newPosition - prePosition;

            if(math.length(newDisplacement) > k_SimpleSolverEpsion) {
                var newCollector = new SelfFilteringClosestHitCollector<ColliderCastHit>(stepInput.RigidBodyIndex, 1.0f);

                ColliderCastInput input = new ColliderCastInput() {
                    Collider = collider,
                    Orientation = orientation,
                    Start = prePosition,
                    End = prePosition + newDisplacement
                };

                world.CastCollider(input, ref newCollector);

                if(newCollector.NumHits > 0) {
                    //要到达解算后的位置仍然会与别的碰撞体相撞

                    ColliderCastHit hit = newCollector.ClosestHit;

                    bool found = false;
                    for (int constraintIndex = 0; constraintIndex < constraints.Length; constraintIndex++) {
                        SurfaceConstraintInfo constraint = constraints[constraintIndex];
                        if(constraint.RigidBodyIndex == hit.RigidBodyIndex && 
                            constraint.ColliderKey.Equals(hit.ColliderKey))
                        {
                            //如果撞到的是之前处理过的碰撞体,让其通过,解算位置有效

                            found = true;
                            break;
                        }
                    }

                    if(!found) {
                        //如果撞到的是未处理过的碰撞体,解算无效,下次迭代从碰到的位置开始

                        Assert.IsTrue(hit.Fraction >= 0.0f && hit.Fraction <= 1.0f);

                        integratedTime *= hit.Fraction;
                        newPosition = prePosition + newDisplacement * hit.Fraction;
                    }
                }
            }

            remaingTIme -= integratedTime;
        }

        transform.pos = newPosition;
        linearVelocity = newVelocity;
    }


    private static void CreateConstraintFromHit(PhysicsWorld world, int rigidBodyIndex, ColliderKey colliderKey,
    float3 hitPosition, float3 normal, float distance, float skinWidth, out SurfaceConstraintInfo constraint) {
        bool bodyIsDynamic = 0 <= rigidBodyIndex && rigidBodyIndex < world.NumDynamicBodies;

        constraint = new SurfaceConstraintInfo() {
            Plane = new Plane {
                Normal = normal,
                Distance = distance - skinWidth
            },
            RigidBodyIndex = rigidBodyIndex,
            ColliderKey = colliderKey,
            HitPosition = hitPosition,
            Velocity = bodyIsDynamic ?
                world.GetLinearVelocity(rigidBodyIndex, hitPosition) :
                float3.zero,
            Priority = bodyIsDynamic ? 1 : 0
        };
    }

    private static void CreateMaxSlopConstraint(float3 up,ref SurfaceConstraintInfo constraint,out SurfaceConstraintInfo maxSlopConstraint) {
        float verticalComponent = math.dot(constraint.Plane.Normal, up);

        SurfaceConstraintInfo newConstraint = constraint;
        newConstraint.Plane.Normal = math.normalize(newConstraint.Plane.Normal - verticalComponent * up);

        float distance = newConstraint.Plane.Distance;

        newConstraint.Plane.Distance = distance / math.max(math.dot(newConstraint.Plane.Normal, constraint.Plane.Normal), 0.5f);

        if(newConstraint.Plane.Distance < 0.0f) {
            constraint.Plane.Distance = 0.0f;

            ResolveConstraintPenetration(ref newConstraint);
        }

        maxSlopConstraint = newConstraint;
    }

    private static void ResolveConstraintPenetration(ref SurfaceConstraintInfo constraint) {
        if(constraint.Plane.Distance < 0.0f) {
            float3 newVal = constraint.Velocity - constraint.Plane.Normal * constraint.Plane.Distance;
            constraint.Velocity = newVal;
            constraint.Plane.Distance = 0.0f;
        }
    }

    private static void CreateConstraint(PhysicsWorld world, float3 up,
        int hitRigidBodyIndex, ColliderKey hitColliderKey, float3 hitPosition, float3 hitSurfaceNormal, float hitDistance,
        float skinWidth, float maxSlopCos, ref NativeList<SurfaceConstraintInfo> constraints) {
        CreateConstraintFromHit(world, hitRigidBodyIndex, hitColliderKey, hitPosition,
            hitSurfaceNormal, hitDistance, skinWidth, out SurfaceConstraintInfo constraint);

        float verticalComponent = math.dot(constraint.Plane.Normal, up);
        bool shouldAddPlane = verticalComponent > k_SimpleSolverEpsion && verticalComponent < maxSlopCos;//上坡
        if (shouldAddPlane) {
            constraint.IsTooSteep = true;
            CreateMaxSlopConstraint(up, ref constraint, out SurfaceConstraintInfo maxSlopeConstraint);
            constraints.Add(maxSlopeConstraint);
        }

        ResolveConstraintPenetration(ref constraint);

        constraints.Add(constraint);
    }

    private static unsafe void CalculateAndStoreDefferedImpulse(
        CharacterControllerStepInput stepInput,float characterMass,float3 linearVelocity,
        ref NativeList<SurfaceConstraintInfo> constraints,ref NativeStream.Writer deferredImpulseWrtier)
    {
        PhysicsWorld world = stepInput.World;
        for (int i = 0; i < constraints.Length; i++) {
            SurfaceConstraintInfo constraint = constraints[i];

            int rigidBodyIndex = constraint.RigidBodyIndex;
            if (rigidBodyIndex < 0 || rigidBodyIndex >= world.NumDynamicBodies) {
                continue;
            }

            RigidBody body = world.Bodies[rigidBodyIndex];

            float3 pointRelvel = world.GetLinearVelocity(rigidBodyIndex, constraint.HitPosition);
            pointRelvel -= linearVelocity;

            float projectedVelocity = math.dot(pointRelvel, constraint.Plane.Normal);

            float deltaVelocity = -projectedVelocity * stepInput.Damping;

            float distnace = constraint.Plane.Distance;
            if(distnace < 0.0f) {
                deltaVelocity += (distnace / stepInput.DeltaTime) * stepInput.Tau;
            }

            MotionVelocity mv = world.MotionVelocities[rigidBodyIndex];
            float3 impulse = float3.zero;
            if(deltaVelocity < 0.0f) {
                float impulseMagnitute = 0.0f;
                {
                    float objectMassInv = GetInvMassAtPoint(constraint.HitPosition,constraint.Plane.Normal,body,mv);
                    impulseMagnitute = deltaVelocity / objectMassInv;
                }

                impulse = impulseMagnitute * constraint.Plane.Normal; 
            }
            //增加重力
            {
                float3 charVelDown = stepInput.Gravity * stepInput.DeltaTime;
                float relVelN = math.dot(charVelDown, constraint.Plane.Normal);

                {
                    bool isSeparatingContact = projectedVelocity < 0.0f;
                    float newRelVelN = relVelN - projectedVelocity;
                    relVelN = math.select(relVelN, newRelVelN, isSeparatingContact);
                }

                {
                    float3 newImpulse = impulse;
                    newImpulse += relVelN * characterMass * constraint.Plane.Normal;
                    impulse = math.select(impulse, newImpulse, relVelN < 0.0f);
                }
            }

            deferredImpulseWrtier.Write(
                new DefferedCharacterControllerImpulse() {
                    Entity = body.Entity,
                    Impulse = impulse,
                    Point = constraint.HitPosition
                });
        }
    }

    private static float GetInvMassAtPoint(float3 point, float3 normal,RigidBody body,MotionVelocity mv) {
        float3 massCenter;
        unsafe {
            massCenter = math.transform(body.WorldFromBody, body.Collider->MassProperties.MassDistribution.Transform.pos);
        }
        float3 arm = point - massCenter;
        float3 jacAng = math.cross(arm, normal);
        float3 armC = jacAng * mv.InverseInertiaAndMass.xyz;

        float objectMassInv = math.dot(armC, jacAng);
        objectMassInv += mv.InverseInertiaAndMass.w;

        return objectMassInv;
    }
}
