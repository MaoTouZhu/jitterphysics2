/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 表示可附加到刚体上的形状的抽象基类。
/// </summary>
public abstract class RigidBodyShape : Shape
{
    /// <summary>
    /// 此形状所附加到的 <see cref="RigidBody"/>。在形状被附加之前以及从刚体移除之后为 null。
    /// </summary>
    public RigidBody RigidBody { get; internal set; } = null!;

    /// <summary>使用其附加刚体的变换绘制此形状；未附加时使用单位变换。</summary>
    /// <inheritdoc cref="Shape.DebugDraw(IDebugDrawer, in DebugDrawSettings)" path="/remarks"/>
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        JQuaternion orientation = RigidBody?.Orientation ?? JQuaternion.Identity;
        JVector position = RigidBody?.Position ?? JVector.Zero;
        var sink = new ShapeDebugDraw.DrawerSink(drawer, orientation, position, settings.CurrentColor);
        DebugDrawLocal(ref sink, settings);
    }

    public sealed override JVector Velocity => RigidBody?.Velocity ?? JVector.Zero;

    public sealed override void UpdateWorldBoundingBox(Real dt = (Real)0.0)
    {
        JBoundingBox box;

        if (RigidBody == null)
        {
            CalculateBoundingBox(JQuaternion.Identity, JVector.Zero, out box);
            WorldBoundingBox = box;
            return;
        }

        ref var data = ref RigidBody.Data;
        CalculateBoundingBox(data.Orientation, data.Position, out box);

        if (RigidBody.EnableSpeculativeContacts)
        {
            // 推测扫掠使用刚体的物理速度来预测平移。
            Real swept = dt * data.Velocity.Length();
            box.Min -= new JVector(swept);
            box.Max += new JVector(swept);
        }

        WorldBoundingBox = box;
    }

    public virtual void CalculateBoundingBox(in JQuaternion orientation, in JVector position, out JBoundingBox box)
    {
        ShapeHelper.CalculateBoundingBox(this, orientation, position, out box);
    }

    /// <summary>
    /// 计算形状的质量和惯性。子类可重写以提升性能或精度。默认实现依赖于
    /// 使用支撑映射函数构造的形状近似。
    /// </summary>
    /// <remarks>
    /// 惯性张量是相对于坐标系原点 (0,0,0) 计算的，
    /// 而不是相对于质心。
    /// </remarks>
    [ReferenceFrame(ReferenceFrame.Local)]
    public virtual void CalculateMassInertia(out JSymmetricMatrix inertia, out JVector com, out Real mass)
    {
        ShapeHelper.CalculateMassInertia(this, out inertia, out com, out mass);
    }

    /// <summary>
    /// 对形状执行局部射线投射，检查从指定点出发、沿指定方向传播的射线是否与对象相交。
    /// 它不考虑关联刚体的变换。
    /// </summary>
    /// <param name="origin">射线的起点。</param>
    /// <param name="direction">
    /// 射线的方向。此向量无需归一化。
    /// </param>
    /// <param name="normal">
    /// 若发生相交，则为交点处的表面法线。
    /// </param>
    /// <param name="lambda">
    /// 从 <paramref name="origin"/> 到交点的射线参数。由于
    /// <paramref name="direction"/> 无需归一化，因此它不一定是距离。命中点为：
    /// <c>origin + lambda * direction</c>。
    /// </param>
    /// <returns>
    /// 若射线与对象相交则为 <c>true</c>；否则为 <c>false</c>。
    /// </returns>
    [ReferenceFrame(ReferenceFrame.Local)]
    public virtual bool LocalRayCast(in JVector origin, in JVector direction, out JVector normal, out Real lambda)
    {
        return NarrowPhase.RayCast(this, origin, direction, out lambda, out normal);
    }

    [ReferenceFrame(ReferenceFrame.World)]
    public sealed override bool RayCast(in JVector origin, in JVector direction, out JVector normal, out Real lambda)
    {
        if (RigidBody == null)
        {
            return LocalRayCast(origin, direction, out normal, out lambda);
        }

        ref var data = ref RigidBody.Data;

        // 将射线旋转到刚体的参考系中……
        JVector transformedDir = JVector.ConjugatedTransform(direction, data.Orientation);
        JVector transformedOrigin = JVector.ConjugatedTransform(origin - data.Position, data.Orientation);

        bool result = LocalRayCast(transformedOrigin, transformedDir, out normal, out lambda);

        // ……再旋转回来。
        JVector.Transform(normal, data.Orientation, out normal);

        return result;
    }

    [ReferenceFrame(ReferenceFrame.World)]
    public sealed override bool Sweep<T>(in T support, in JQuaternion orientation, in JVector position, in JVector sweep,
        out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
    {
        if (RigidBody == null)
        {
            bool hit = NarrowPhase.Sweep(this, support,
                orientation, position, sweep,
                out pointB, out pointA, out normal, out lambda);

            JVector.NegateInPlace(ref normal);
            return hit;
        }

        ref var data = ref RigidBody.Data;

        return NarrowPhase.Sweep(support, this,
            orientation, data.Orientation,
            position, data.Position,
            sweep, JVector.Zero,
            out pointA, out pointB, out normal, out lambda);
    }

    [ReferenceFrame(ReferenceFrame.World)]
    public sealed override bool Distance<T>(in T support, in JQuaternion orientation, in JVector position,
        out JVector pointA, out JVector pointB, out JVector normal, out Real distance)
    {
        if (RigidBody == null)
        {
            return NarrowPhase.Distance(support, this,
                orientation, JQuaternion.Identity,
                position, JVector.Zero,
                out pointA, out pointB, out normal, out distance);
        }

        ref var data = ref RigidBody.Data;

        return NarrowPhase.Distance(support, this,
            orientation, data.Orientation,
            position, data.Position,
            out pointA, out pointB, out normal, out distance);
    }
}
