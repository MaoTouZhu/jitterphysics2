/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 表示由高度和半径定义的圆柱形状。
/// </summary>
public class CylinderShape : RigidBodyShape
{
    private Real radius;
    private Real height;

    /// <inheritdoc/>
    protected internal override void DebugDrawLocal<TSink>(ref TSink triangleSink, in DebugDrawSettings settings)
    {
        ShapeDebugDraw.Cylinder(ref triangleSink, radius, height, settings.Segments);
    }

    /// <summary>
    /// 获取或设置圆柱的半径。
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
    /// 获取或设置圆柱的高度。
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="value"/> 小于或等于零时抛出。
    /// </exception>
    public Real Height
    {
        get => height;
        set
        {
            height = ArgumentCheck.Positive(value, nameof(Height));
            UpdateWorldBoundingBox();
        }
    }

    /// <summary>
    /// 初始化 <see cref="CylinderShape"/> 类的新实例，创建一个具有指定
    /// 高度和半径的圆柱形状。圆柱的对称轴沿 y 轴对齐。
    /// </summary>
    /// <param name="height">圆柱的高度。</param>
    /// <param name="radius">圆柱底面的半径。</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="height"/> 或 <paramref name="radius"/> 小于或等于零时抛出。
    /// </exception>
    public CylinderShape(Real height, Real radius)
    {
        this.radius = ArgumentCheck.Positive(radius, nameof(radius));
        this.height = ArgumentCheck.Positive(height, nameof(height));
        UpdateWorldBoundingBox();
    }

    /// <inheritdoc/>
    public override void GetCenter(out JVector point)
    {
        point = JVector.Zero;
    }

    /// <inheritdoc/>
    public override void SupportMap(in JVector direction, out JVector result)
    {
        Real sigma = (Real)Math.Sqrt(direction.X * direction.X + direction.Z * direction.Z);

        if (sigma > (Real)0.0)
        {
            result.X = direction.X / sigma * radius;
            result.Y = Math.Sign(direction.Y) * height * (Real)0.5;
            result.Z = direction.Z / sigma * radius;
        }
        else
        {
            result.X = (Real)0.0;
            result.Y = Math.Sign(direction.Y) * height * (Real)0.5;
            result.Z = (Real)0.0;
        }
    }

    /// <inheritdoc/>
    public override void CalculateBoundingBox(in JQuaternion orientation, in JVector position, out JBoundingBox box)
    {
        JVector upa = orientation.GetBasisY();

        Real xx = upa.X * upa.X;
        Real yy = upa.Y * upa.Y;
        Real zz = upa.Z * upa.Z;

        Real xExt = MathR.Sqrt(yy + zz) * radius;
        Real yExt = MathR.Sqrt(xx + zz) * radius;
        Real zExt = MathR.Sqrt(xx + yy) * radius;

        JVector p1 = -(Real)0.5 * height * upa;
        JVector p2 = +(Real)0.5 * height * upa;

        JVector delta = JVector.Max(p1, p2) + new JVector(xExt, yExt, zExt);

        box.Min = position - delta;
        box.Max = position + delta;
    }

    /// <inheritdoc/>
    public override void CalculateMassInertia(out JSymmetricMatrix inertia, out JVector com, out Real mass)
    {
        mass = MathR.PI * radius * radius * height;

        inertia = JSymmetricMatrix.Identity;

        inertia.M11 = (Real)(1.0 / 4.0) * mass * radius * radius + (Real)(1.0 / 12.0) * mass * height * height;
        inertia.M22 = (Real)(1.0 / 2.0) * mass * radius * radius;
        inertia.M33 = (Real)(1.0 / 4.0) * mass * radius * radius + (Real)(1.0 / 12.0) * mass * height * height;

        com = JVector.Zero;
    }
}
