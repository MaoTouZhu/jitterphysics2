/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using Jitter2.DataStructures;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 表示由点云定义的凸包形状。与 <see cref="ConvexHullShape"/> 不同，
/// 这些点不必位于凸包上。为了优化性能，
/// 此形状最好用于少量点（约 300 个）的情况。
/// </summary>
public class PointCloudShape : RigidBodyShape, ICloneableShape<PointCloudShape>
{
    private JBoundingBox cachedBoundingBox;
    private JSymmetricMatrix cachedInertia;
    private Real cachedMass;
    private JVector cachedCenter;

    private VertexSupportMap supportMap;
    private JVector shifted;

    /// <inheritdoc cref="PointCloudShape(ReadOnlySpan{JVector})"/>
    public PointCloudShape(IEnumerable<JVector> vertices) :
        this(SpanHelper.AsReadOnlySpan(vertices, out _))
    {
    }

    /// <summary>
    /// 初始化 <see cref="PointCloudShape"/> 类的新实例。
    /// </summary>
    /// <param name="vertices">定义凸包的所有顶点。</param>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="vertices"/> 为空时抛出。
    /// </exception>
    /// <exception cref="InvalidOperationException">
    /// 当 <paramref name="vertices"/> 未定义非退化体积时抛出。
    /// </exception>
    public PointCloudShape(ReadOnlySpan<JVector> vertices)
    {
        supportMap = new VertexSupportMap(vertices);
        UpdateShape();
    }

    /// <summary>
    /// 初始化 <see cref="PointCloudShape"/> 类的新实例。
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// 当 <paramref name="supportMap"/> 未定义非退化体积时抛出。
    /// </exception>
    public PointCloudShape(VertexSupportMap supportMap)
    {
        this.supportMap = supportMap;
        UpdateShape();
    }

    private PointCloudShape()
    {
    }

    /// <summary>
    /// 创建此形状的副本。底层数据结构在各实例之间共享。
    /// </summary>
    public PointCloudShape Clone()
    {
        PointCloudShape result = new()
        {
            supportMap = supportMap,
            cachedBoundingBox = cachedBoundingBox,
            cachedCenter = cachedCenter,
            cachedInertia = cachedInertia,
            cachedMass = cachedMass,
            shifted = shifted
        };
        result.UpdateWorldBoundingBox();
        return result;
    }

    /// <summary>
    /// 获取或设置形状的偏移值。此属性可用于构造包含一个或多个形状的刚体。
    /// </summary>
    public JVector Shift
    {
        get => shifted;
        set
        {
            shifted = value;
            UpdateShape();
        }
    }

    /// <summary>
    /// 更新形状缓存的质量、惯性和包围盒。
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// 当点云未定义非退化体积时抛出。
    /// </exception>
    public void UpdateShape()
    {
        CalculateMassInertia();
        CalcInitBox();
        UpdateWorldBoundingBox();
    }

    /// <summary>
    /// 重新计算质量、质心和惯性张量。
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// 当点云未定义非退化体积时抛出。
    /// </exception>
    public void CalculateMassInertia()
    {
        ShapeHelper.CalculateMassInertia(this, out cachedInertia, out cachedCenter, out cachedMass);

        const Real minimumMass = (Real)1e-12;

        if (!Real.IsFinite(cachedMass) || cachedMass <= minimumMass)
        {
            throw new InvalidOperationException("Point cloud must define a non-degenerate volume.");
        }
    }

    /// <inheritdoc/>
    public override void CalculateMassInertia(out JSymmetricMatrix inertia, out JVector com, out Real mass)
    {
        inertia = cachedInertia;
        com = cachedCenter;
        mass = cachedMass;
    }

    /// <inheritdoc/>
    public override void CalculateBoundingBox(in JQuaternion orientation, in JVector position, out JBoundingBox box)
    {
        JVector halfSize = (Real)0.5 * (cachedBoundingBox.Max - cachedBoundingBox.Min);
        JVector center = (Real)0.5 * (cachedBoundingBox.Max + cachedBoundingBox.Min);

        JMatrix ori = JMatrix.CreateFromQuaternion(orientation);
        JMatrix.Absolute(in ori, out JMatrix abs);
        JVector.Transform(halfSize, abs, out JVector temp);
        JVector.Transform(center, orientation, out JVector temp2);

        box.Max = temp;
        JVector.Negate(temp, out box.Min);

        JVector.Add(box.Min, position + temp2, out box.Min);
        JVector.Add(box.Max, position + temp2, out box.Max);
    }

    private void CalcInitBox()
    {
        JVector vec = JVector.UnitX;
        SupportMap(vec, out JVector res);
        cachedBoundingBox.Max.X = res.X;

        vec = JVector.UnitY;
        SupportMap(vec, out res);
        cachedBoundingBox.Max.Y = res.Y;

        vec = JVector.UnitZ;
        SupportMap(vec, out res);
        cachedBoundingBox.Max.Z = res.Z;

        vec = -JVector.UnitX;
        SupportMap(vec, out res);
        cachedBoundingBox.Min.X = res.X;

        vec = -JVector.UnitY;
        SupportMap(vec, out res);
        cachedBoundingBox.Min.Y = res.Y;

        vec = -JVector.UnitZ;
        SupportMap(vec, out res);
        cachedBoundingBox.Min.Z = res.Z;
    }

    /// <inheritdoc/>
    public override void SupportMap(in JVector direction, out JVector result)
    {
        supportMap.SupportMap(direction, out result);
        result += shifted;
    }

    /// <inheritdoc/>
    public override void GetCenter(out JVector point)
    {
        point = cachedCenter;
    }
}
