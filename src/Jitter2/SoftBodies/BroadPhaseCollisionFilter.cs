/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace Jitter2.SoftBodies;

/// <summary>
/// 一个宽相过滤器，用于处理涉及软体形状的碰撞。
/// 它将碰撞检测委托给窄相，并将接触注册到软体
/// 最接近的刚体顶点上。
/// </summary>
public class BroadPhaseCollisionFilter : IBroadPhaseFilter
{
    private readonly World world;

    /// <summary>
    /// 初始化 <see cref="BroadPhaseCollisionFilter"/> 类的新实例。
    /// </summary>
    /// <param name="world">世界实例。</param>
    public BroadPhaseCollisionFilter(World world)
    {
        this.world = world;
    }

    /// <inheritdoc/>
    public bool Filter(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB)
    {
        SoftBodyShape? softShapeA = proxyA as SoftBodyShape;
        SoftBodyShape? softShapeB = proxyB as SoftBodyShape;

        if (softShapeA != null && softShapeB != null)
        {
            if (softShapeB.ShapeId < softShapeA.ShapeId)
            {
                (softShapeA, softShapeB) = (softShapeB, softShapeA);
            }

            if (!softShapeA.SoftBody.IsActive && !softShapeB.SoftBody.IsActive) return false;

            bool colliding = NarrowPhase.MprEpa(softShapeA, softShapeB,
                JQuaternion.Identity, JVector.Zero,
                out JVector pA, out JVector pB, out JVector normal, out _);

            if (!colliding) return false;

            var closestA = softShapeA.GetClosest(pA);
            var closestB = softShapeB.GetClosest(pB);

            world.RegisterContact(closestA.RigidBodyId, closestB.RigidBodyId, closestA, closestB,
                pA, pB, normal);

            return false;
        }

        if (softShapeA != null)
        {
            var rigidShapeB = (proxyB as RigidBodyShape)!;
            var rigidBodyB = rigidShapeB.RigidBody;

            if (!softShapeA.SoftBody.IsActive && !rigidBodyB.Data.IsActive) return false;

            bool colliding = NarrowPhase.MprEpa(softShapeA, rigidShapeB, rigidBodyB.Orientation, rigidBodyB.Position,
                out JVector pA, out JVector pB, out JVector normal, out _);

            if (!colliding) return false;

            var closest = softShapeA.GetClosest(pA);

            world.RegisterContact(closest.RigidBodyId, rigidBodyB.RigidBodyId, closest, rigidBodyB,
                pA, pB, normal, ContactData.SolveMode.AngularBody1);

            return false;
        }

        if (softShapeB != null)
        {
            var rigidShapeA = (proxyA as RigidBodyShape)!;
            var rigidBodyA = rigidShapeA.RigidBody;

            if (!softShapeB.SoftBody.IsActive && !rigidBodyA.Data.IsActive) return false;

            bool colliding = NarrowPhase.MprEpa(softShapeB, rigidShapeA, rigidBodyA.Orientation, rigidBodyA.Position,
                out JVector pA, out JVector pB, out JVector normal, out _);

            if (!colliding) return false;

            var closest = softShapeB.GetClosest(pA);

            world.RegisterContact(closest.RigidBodyId, rigidBodyA.RigidBodyId, closest, rigidBodyA,
                pA, pB, normal, ContactData.SolveMode.AngularBody1);

            return false;
        }

        return true;
    }
}
