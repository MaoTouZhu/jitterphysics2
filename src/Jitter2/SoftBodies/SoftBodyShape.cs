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
/// 软体模拟中所用形状的抽象基类。
/// </summary>
public abstract class SoftBodyShape : Shape
{
    /// <summary>
    /// 获取距离指定位置最近的刚体。
    /// </summary>
    /// <param name="pos">世界坐标系中的位置。</param>
    /// <returns>此形状中最近的刚体（顶点）。</returns>
    public abstract RigidBody GetClosest(in JVector pos);

    /// <summary>
    /// 获取此形状所属的软体实例。
    /// </summary>
    public SoftBody SoftBody { get; internal init; } = null!;

    /// <inheritdoc/>
    public override bool RayCast(in JVector origin, in JVector direction, out JVector normal, out Real lambda)
    {
        return NarrowPhase.RayCast(this, origin, direction, out lambda, out normal);
    }

    /// <inheritdoc/>
    public override bool Sweep<T>(in T support, in JQuaternion orientation, in JVector position, in JVector sweep,
        out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
    {
        bool hit = NarrowPhase.Sweep(this, support,
            orientation, position, sweep,
            out pointB, out pointA, out normal, out lambda);

        JVector.NegateInPlace(ref normal);
        return hit;
    }

    /// <inheritdoc/>
    public override bool Distance<T>(in T support, in JQuaternion orientation, in JVector position,
        out JVector pointA, out JVector pointB, out JVector normal, out Real distance)
    {
        return NarrowPhase.Distance(support, this,
            orientation, JQuaternion.Identity,
            position, JVector.Zero,
            out pointA, out pointB, out normal, out distance);
    }
}
