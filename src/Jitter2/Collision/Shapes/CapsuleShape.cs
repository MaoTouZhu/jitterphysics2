/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 表示由半径及其圆柱段长度定义的胶囊形状。
/// </summary>
public class CapsuleShape : RigidBodyShape
{
    private Real radius;
    private Real halfLength;

    /// <inheritdoc/>
    protected internal override void DebugDrawLocal<TSink>(ref TSink triangleSink, in DebugDrawSettings settings)
    {
        ShapeDebugDraw.Capsule(ref triangleSink, radius, halfLength, settings.Segments);
    }

    /// <summary>
    /// 获取或设置胶囊的半径。
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="value"/> 小于或等于零时抛出。
    /// </exception>
    public Real Radius
    {
        get => radius;
        set
        {
            radius = ArgumentCheck.Positive(value, nameof(Radius));
            UpdateWorldBoundingBox();
        }
    }

    /// <summary>
    /// 获取或设置胶囊圆柱部分的长度，不包括两端的半球。
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="value"/> 为负数时抛出。
    /// </exception>
    public Real Length
    {
        get => (Real)2.0 * halfLength;
        set
        {
            halfLength = ArgumentCheck.NonNegative(value, nameof(Length)) / (Real)2.0;
            UpdateWorldBoundingBox();
        }
    }

    /// <summary>
    /// 使用指定的半径和长度初始化 CapsuleShape 类的新实例。胶囊的对称轴沿 Y 轴对齐。
    /// </summary>
    /// <param name="radius">胶囊的半径。</param>
    /// <param name="length">胶囊圆柱部分的长度，不包括两端的半球。</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="radius"/> 小于或等于零，或 <paramref name="length"/> 为负数时抛出。
    /// </exception>
    public CapsuleShape(Real radius = (Real)0.5, Real length = (Real)1.0)
    {
        this.radius = ArgumentCheck.Positive(radius, nameof(radius));
        halfLength = (Real)0.5 * ArgumentCheck.NonNegative(length, nameof(length));
        UpdateWorldBoundingBox();
    }

    /// <inheritdoc/>
    public override void SupportMap(in JVector direction, out JVector result)
    {
        // 胶囊 = 线段 + 球体

        // 球体
        result = JVector.Normalize(direction) * radius;

        // 线段的两个端点为
        // p_1 = (0, +length/2, 0)
        // p_2 = (0, -length/2, 0)

        // 我们需要计算与方向向量的点积，
        // 以判断 p_1 还是 p_2 是正确的支撑点
        result.Y += MathR.Sign(direction.Y) * halfLength;
    }

    /// <inheritdoc/>
    public override void GetCenter(out JVector point)
    {
        point = JVector.Zero;
    }

    /// <inheritdoc/>
    public override void CalculateBoundingBox(in JQuaternion orientation, in JVector position, out JBoundingBox box)
    {
        JVector delta = halfLength * orientation.GetBasisY();

        box.Max.X = +radius + MathR.Abs(delta.X);
        box.Max.Y = +radius + MathR.Abs(delta.Y);
        box.Max.Z = +radius + MathR.Abs(delta.Z);

        box.Min = -box.Max;

        box.Min += position;
        box.Max += position;
    }

    /// <inheritdoc/>
    public override void CalculateMassInertia(out JSymmetricMatrix inertia, out JVector com, out Real mass)
    {
        Real length = (Real)2.0 * halfLength;

        Real massSphere = (Real)(4.0 / 3.0) * MathR.PI * radius * radius * radius;
        Real massCylinder = MathR.PI * radius * radius * length;

        inertia = JSymmetricMatrix.Identity;

        inertia.M11 = massCylinder * ((Real)(1.0 / 12.0) * length * length + (Real)(1.0 / 4.0) * radius * radius) + massSphere *
            ((Real)(2.0 / 5.0) * radius * radius + (Real)(1.0 / 4.0) * length * length + (Real)(3.0 / 8.0) * length * radius);
        inertia.M22 = (Real)(1.0 / 2.0) * massCylinder * radius * radius + (Real)(2.0 / 5.0) * massSphere * radius * radius;
        inertia.M33 = massCylinder * ((Real)(1.0 / 12.0) * length * length + (Real)(1.0 / 4.0) * radius * radius) + massSphere *
            ((Real)(2.0 / 5.0) * radius * radius + (Real)(1.0 / 4.0) * length * length + (Real)(3.0 / 8.0) * length * radius);

        mass = massCylinder + massSphere;
        com = JVector.Zero;
    }
}
