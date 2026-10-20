/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 表示一个三角形形状，由对 <see cref="TriangleMesh"/> 的引用和一个索引定义。
/// </summary>
public class TriangleShape : RigidBodyShape
{
    /// <summary>
    /// 该三角形所属的三角形网格。
    /// </summary>
    public readonly TriangleMesh Mesh;

    /// <summary>
    /// 表示该三角形在网格中位置的索引。
    /// </summary>
    public readonly int Index;

    /// <inheritdoc/>
    protected internal override void DebugDrawLocal<TSink>(ref TSink triangleSink, in DebugDrawSettings settings)
    {
        ref readonly var triangle = ref Mesh.Indices[Index];
        triangleSink.Add(new JTriangle(Mesh.Vertices[triangle.IndexA],
            Mesh.Vertices[triangle.IndexB], Mesh.Vertices[triangle.IndexC]));
    }

    /// <summary>
    /// 初始化 TriangleShape 类的新实例。
    /// </summary>
    /// <param name="mesh">该三角形所属的三角形网格。</param>
    /// <param name="index">表示该三角形在网格中位置的索引。</param>
    /// <exception cref="ArgumentNullException">
    /// 当 <paramref name="mesh"/> 为 <c>null</c> 时抛出。
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="index"/> 为负数，或大于等于网格中的三角形数量时抛出。
    /// </exception>
    public TriangleShape(TriangleMesh mesh, int index)
    {
        ArgumentNullException.ThrowIfNull(mesh);
        ArgumentOutOfRangeException.ThrowIfNegative(index);
        ArgumentOutOfRangeException.ThrowIfGreaterThanOrEqual(index, mesh.Indices.Length);

        Mesh = mesh;
        Index = index;

        UpdateWorldBoundingBox();
    }

    /// <summary>
    /// 为给定的 <see cref="TriangleMesh"/> 创建并返回所有 <see cref="TriangleShape"/> 类型的实例。
    /// </summary>
    public static IEnumerable<TriangleShape> CreateAllShapes(TriangleMesh mesh)
    {
        for (int index = 0; index < mesh.Indices.Length; index++)
        {
            yield return new TriangleShape(mesh, index);
        }
    }

    /// <exception cref="NotSupportedException">
    /// 总是抛出，因为三角形没有体积，因此没有质量属性。
    /// </exception>
    public override void CalculateMassInertia(out JSymmetricMatrix inertia, out JVector com, out Real mass)
    {
        throw new NotSupportedException($"{nameof(TriangleShape)} has no mass properties. " +
                                        $"If you encounter this while calling RigidBody.AddShape, " +
                                        $"call AddShape with massInertiaMode set to {nameof(MassInertiaUpdateMode.Preserve)}.");
    }

    /// <summary>
    /// 获取经过所附刚体变换后的顶点。如果该形状未附着到刚体，
    /// 则返回未经世界变换的网格顶点。
    /// </summary>
    /// <param name="a">第一个顶点变换后的坐标。</param>
    /// <param name="b">第二个顶点变换后的坐标。</param>
    /// <param name="c">第三个顶点变换后的坐标。</param>
    public void GetWorldVertices(out JVector a, out JVector b, out JVector c)
    {
        ref readonly var triangle = ref Mesh.Indices[Index];
        a = Mesh.Vertices[triangle.IndexA];
        b = Mesh.Vertices[triangle.IndexB];
        c = Mesh.Vertices[triangle.IndexC];

        if (RigidBody == null) return;

        ref JQuaternion orientation = ref RigidBody.Data.Orientation;
        ref JVector position = ref RigidBody.Data.Position;

        JVector.Transform(a, orientation, out a);
        JVector.Transform(b, orientation, out b);
        JVector.Transform(c, orientation, out c);

        a += position;
        b += position;
        c += position;
    }

    /// <inheritdoc/>
    public override void CalculateBoundingBox(in JQuaternion orientation, in JVector position, out JBoundingBox box)
    {
        const Real extraMargin = (Real)0.01;

        ref readonly var triangle = ref Mesh.Indices[Index];
        var a = Mesh.Vertices[triangle.IndexA];
        var b = Mesh.Vertices[triangle.IndexB];
        var c = Mesh.Vertices[triangle.IndexC];

        JVector.Transform(a, orientation, out a);
        JVector.Transform(b, orientation, out b);
        JVector.Transform(c, orientation, out c);

        box = JBoundingBox.SmallBox;

        JBoundingBox.AddPointInPlace(ref box, a);
        JBoundingBox.AddPointInPlace(ref box, b);
        JBoundingBox.AddPointInPlace(ref box, c);

        // 防止生成退化的包围盒
        JVector extra = new JVector(extraMargin);
        box.Min += position - extra;
        box.Max += position + extra;
    }

    /// <inheritdoc/>
    /// <remarks>背面三角形会被剔除。</remarks>
    public override bool LocalRayCast(in JVector origin, in JVector direction, out JVector normal, out Real lambda)
    {
        ref readonly var meshTriangle = ref Mesh.Indices[Index];

        var triangle = new JTriangle(Mesh.Vertices[meshTriangle.IndexA],
            Mesh.Vertices[meshTriangle.IndexB], Mesh.Vertices[meshTriangle.IndexC]);

        if (triangle.RayIntersect(origin, direction, JTriangle.CullMode.BackFacing, out normal, out lambda) &&
            lambda >= (Real)0.0) return true;

        // RayIntersect 检测的是整条直线，因此位于原点后方的命中对射线而言应视为未命中。
        normal = JVector.Zero;
        lambda = Real.MaxValue;
        return false;
    }

    /// <inheritdoc/>
    public override void GetCenter(out JVector point)
    {
        ref readonly var triangle = ref Mesh.Indices[Index];

        JVector a = Mesh.Vertices[triangle.IndexA];
        JVector b = Mesh.Vertices[triangle.IndexB];
        JVector c = Mesh.Vertices[triangle.IndexC];

        point = (Real)(1.0 / 3.0) * (a + b + c);
    }

    /// <inheritdoc/>
    public override void SupportMap(in JVector direction, out JVector result)
    {
        ref readonly var triangle = ref Mesh.Indices[Index];

        JVector a = Mesh.Vertices[triangle.IndexA];
        JVector b = Mesh.Vertices[triangle.IndexB];
        JVector c = Mesh.Vertices[triangle.IndexC];

        Real min = JVector.Dot(a, direction);
        Real dot = JVector.Dot(b, direction);

        result = a;

        if (dot > min)
        {
            min = dot;
            result = b;
        }

        dot = JVector.Dot(c, direction);

        if (dot > min)
        {
            result = c;
        }
    }
}
