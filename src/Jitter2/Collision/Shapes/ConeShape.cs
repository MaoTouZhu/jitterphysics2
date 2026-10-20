/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 表示由底面半径和高度定义的圆锥形状。
/// </summary>
public class ConeShape : RigidBodyShape
{
    private Real radius;
    private Real height;

    /// <inheritdoc/>
    protected internal override void DebugDrawLocal<TSink>(ref TSink triangleSink, in DebugDrawSettings settings)
    {
        ShapeDebugDraw.Cone(ref triangleSink, radius, height, settings.Segments);
    }

    /// <summary>
    /// 获取或设置圆锥底面的半径。
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当半径小于或等于零时抛出。
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
    /// 获取或设置圆锥的高度。
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
    /// 使用指定的半径和高度初始化 ConeShape 类的新实例。圆锥的对称轴沿 Y 轴对齐。
    /// </summary>
    /// <param name="radius">圆锥底面的半径。</param>
    /// <param name="height">圆锥的高度。</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="radius"/> 或 <paramref name="height"/> 小于或等于零时抛出。
    /// </exception>
    public ConeShape(Real radius = (Real)0.5, Real height = (Real)1.0)
    {
        this.radius = ArgumentCheck.Positive(radius, nameof(radius));
        this.height = ArgumentCheck.Positive(height, nameof(height));
        UpdateWorldBoundingBox();
    }

    /// <inheritdoc/>
    public override void SupportMap(in JVector direction, out JVector result)
    {
        const Real zeroEpsilon = (Real)1e-12;
        // 圆锥 = 圆盘与点的凸包

        // 质心位于 0.25 高度处。
        JVector baseDir = new JVector(direction.X, (Real)0.0, direction.Z);
        baseDir = JVector.NormalizeSafe(baseDir, zeroEpsilon) * radius;

        baseDir.Y = -(Real)0.25 * height;

        // 圆盘支撑点与 (0, 0.75 * height, 0) 比较
        if (JVector.Dot(direction, baseDir) >= direction.Y * (Real)0.75 * height)
        {
            result = baseDir;
        }
        else
        {
            result = new JVector(0, (Real)0.75 * height, 0);
        }
    }

    /// <inheritdoc/>
    public override void GetCenter(out JVector point)
    {
        point = JVector.Zero;
    }

    /// <inheritdoc/>
    public override void CalculateBoundingBox(in JQuaternion orientation, in JVector position, out JBoundingBox box)
    {
        JVector upa = orientation.GetBasisY();

        Real xx = upa.X * upa.X;
        Real yy = upa.Y * upa.Y;
        Real zz = upa.Z * upa.Z;

        Real xext = MathR.Sqrt(yy + zz) * radius;
        Real yext = MathR.Sqrt(xx + zz) * radius;
        Real zext = MathR.Sqrt(xx + yy) * radius;

        JVector p1 = -(Real)0.25 * height * upa;
        JVector p2 = +(Real)0.75 * height * upa;

        box.Min = p1 - new JVector(xext, yext, zext);
        box.Max = p1 + new JVector(xext, yext, zext);

        JBoundingBox.AddPointInPlace(ref box, p2);

        box.Min += position;
        box.Max += position;
    }

    /// <inheritdoc/>
    public override void CalculateMassInertia(out JSymmetricMatrix inertia, out JVector com, out Real mass)
    {
        mass = (Real)(1.0 / 3.0) * MathR.PI * radius * radius * height;

        inertia = JSymmetricMatrix.Identity;
        inertia.M11 = mass * ((Real)(3.0 / 20.0) * radius * radius + (Real)(3.0 / 80.0) * height * height);
        inertia.M22 = (Real)(3.0 / 10.0) * mass * radius * radius;
        inertia.M33 = mass * ((Real)(3.0 / 20.0) * radius * radius + (Real)(3.0 / 80.0) * height * height);

        com = JVector.Zero;
    }
}
