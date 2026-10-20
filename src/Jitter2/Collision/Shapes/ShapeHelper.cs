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
/// 提供用于计算隐式定义形状属性的辅助方法。
/// </summary>
public static class ShapeHelper
{
    private const Real GoldenRatio = (Real)1.6180339887498948482045;
    private static readonly JSymmetricMatrix canonicalTetrahedronInertia = new(
        (Real)(1.0 / 60.0), (Real)(1.0 / 120.0), (Real)(1.0 / 120.0),
        (Real)(1.0 / 60.0), (Real)(1.0 / 120.0),
        (Real)(1.0 / 60.0));

    private static readonly JVector[] icosahedronVertices =
    [
        new(0, +1, +GoldenRatio), new(0, -1, +GoldenRatio), new(0, +1, -GoldenRatio), new(0, -1, -GoldenRatio),
        new(+1, +GoldenRatio, 0), new(+1, -GoldenRatio, 0), new(-1, +GoldenRatio, 0), new(-1, -GoldenRatio, 0),
        new(+GoldenRatio, 0, +1), new(+GoldenRatio, 0, -1), new(-GoldenRatio, 0, +1), new(-GoldenRatio, 0, -1)
    ];

    private static readonly int[,] icosahedronIndices = new int[20, 3]
    {
        { 1, 0, 10 }, { 0, 1, 8 }, { 0, 4, 6 }, { 4, 0, 8 }, { 0, 6, 10 }, { 5, 1, 7 }, { 1, 5, 8 }, { 7, 1, 10 },
        { 2, 3, 11 }, { 3, 2, 9 }, { 4, 2, 6 }, { 2, 4, 9 }, { 6, 2, 11 }, { 3, 5, 7 }, { 5, 3, 9 }, { 3, 7, 11 },
        { 4, 8, 9 }, { 8, 5, 9 }, { 10, 6, 11 }, { 7, 10, 11 }
    };

    private static int GetTessellationTriangleCapacity(int subdivisions)
    {
        int capacity = 20;

        for (int i = 1; i < subdivisions; i++)
        {
            capacity = checked(capacity * 4);
        }

        return capacity;
    }

    /// <inheritdoc cref="Tessellate{TSupport}(in TSupport, int)"/>
    /// <param name="support">形状实现的支撑映射接口。</param>
    /// <param name="hullCollection">用于添加三角形的集合。</param>
    /// <param name="subdivisions">递归深度，包含最终输出层级。</param>
    public static void Tessellate<TSupport, TCollection>(in TSupport support, TCollection hullCollection, int subdivisions = 3)
        where TSupport : ISupportMappable
        where TCollection : class, ICollection<JTriangle>
    {
        var sink = new CollectionSink<JTriangle>(hullCollection);
        Tessellate(in support, ref sink, subdivisions);
    }

    /// <summary>
    /// 创建由支撑映射定义的形状的细分网格，并将所有生成的三角形追加到指定的接收器。
    /// </summary>
    /// <typeparam name="TSupport">支撑形状类型。</typeparam>
    /// <typeparam name="TSink">接收生成三角形的接收器类型。</typeparam>
    /// <param name="support">形状实现的支撑映射接口。</param>
    /// <param name="hullSink">接收生成三角形的接收器。</param>
    /// <param name="subdivisions">递归深度，包含最终输出层级。
    /// 值为 1 时输出初始面；默认值 3 会将每个面分割两次。</param>
    /// <remarks>
    /// <para>
    /// <b>警告：</b>细分后的凸包可能并非完全凸。因此不适用于
    /// <see cref="ConvexHullShape"/>。
    /// </para>
    /// <para>
    /// <b>警告：</b>细分网格可能包含重叠的三角形和折痕，几何法线
    /// 甚至对于光滑凸形状也可能朝内。
    /// </para>
    /// <para>
    /// 时间复杂度为 O(4^n)，其中 n 为细分次数。
    /// </para>
    /// </remarks>
    public static void Tessellate<TSupport, TSink>(in TSupport support, ref TSink hullSink, int subdivisions = 3)
        where TSupport : ISupportMappable
        where TSink : ISink<JTriangle>
    {
        for (int i = 0; i < 20; i++)
        {
            JVector v1 = icosahedronVertices[icosahedronIndices[i, 0]];
            JVector v2 = icosahedronVertices[icosahedronIndices[i, 1]];
            JVector v3 = icosahedronVertices[icosahedronIndices[i, 2]];

            support.SupportMap(v1, out JVector sv1);
            support.SupportMap(v2, out JVector sv2);
            support.SupportMap(v3, out JVector sv3);

            Subdivide(in support, ref hullSink, v1, v2, v3, sv1, sv2, sv3, subdivisions);
        }
    }

    private static void Subdivide<TSupport, TSink>(in TSupport support, ref TSink hullSink,
        JVector v1, JVector v2, JVector v3, JVector p1, JVector p2, JVector p3,
        int subdivisions) where TSupport : ISupportMappable where TSink : ISink<JTriangle>
    {
        if (subdivisions <= 1)
        {
            JVector n = (p3 - p1) % (p2 - p1);

            if (n.LengthSquared() > (Real)1e-16)
            {
                hullSink.Add(new JTriangle(p1, p2, p3));
            }

            return;
        }

        // 有意跳过将中点方向重新投影到球面上的步骤。
        // 由此得到的采样点对本次近似而言已足够好，且
        // 避免这些归一化调用可使细分保持低开销。
        JVector h1 = (v1 + v2) * (Real)0.5;
        JVector h2 = (v2 + v3) * (Real)0.5;
        JVector h3 = (v3 + v1) * (Real)0.5;

        support.SupportMap(h1, out JVector sp1);
        support.SupportMap(h2, out JVector sp2);
        support.SupportMap(h3, out JVector sp3);

        subdivisions -= 1;

        Subdivide(in support, ref hullSink, v1, h1, h3, p1, sp1, sp3, subdivisions);
        Subdivide(in support, ref hullSink, h1, v2, h2, sp1, p2, sp2, subdivisions);
        Subdivide(in support, ref hullSink, h3, h2, v3, sp3, sp2, p3, subdivisions);
        Subdivide(in support, ref hullSink, h2, h3, h1, sp2, sp3, sp1, subdivisions);
    }

    /// <summary>
    /// 创建由支撑映射定义的形状的细分网格。
    /// </summary>
    /// <param name="support">形状实现的支撑映射接口。</param>
    /// <param name="subdivisions">递归深度，包含最终输出层级。
    /// 值为 1 时输出初始面；默认值 3 会将每个面分割两次。</param>
    /// <remarks>
    /// <para>
    /// 细分后的凸包可能并非完全凸。因此不适用于
    /// <see cref="ConvexHullShape"/>。
    /// </para>
    /// <para>
    /// <b>警告：</b>细分网格可能包含重叠的三角形和折痕，几何法线
    /// 甚至对于光滑凸形状也可能朝内。
    /// </para>
    /// <para>
    /// 时间复杂度为 O(4^n)，其中 n 为细分次数。
    /// </para>
    /// </remarks>
    public static List<JTriangle> Tessellate<TSupport>(in TSupport support, int subdivisions = 3)
        where TSupport : ISupportMappable
    {
        List<JTriangle> triangles = new(GetTessellationTriangleCapacity(subdivisions));
        Tessellate(in support, triangles, subdivisions);
        return triangles;
    }

    /// <summary>
    /// 创建给定 3D 顶点集凸包的细分网格。
    /// </summary>
    /// <param name="vertices">用于近似凸包的顶点。</param>
    /// <param name="subdivisions">用于生成凸包的细分次数。</param>
    /// <returns>表示凸包的三角形列表。</returns>
    /// <remarks>
    /// <para>
    /// 细分后的凸包可能并非完全凸。因此不适用于
    /// <see cref="ConvexHullShape"/>。
    /// </para>
    /// <para>
    /// <b>警告：</b>细分网格可能包含重叠的三角形和折痕，几何法线
    /// 甚至对于光滑凸形状也可能朝内。
    /// </para>
    /// <para>
    /// 时间复杂度为 O(4^n)，其中 n 为细分次数。
    /// </para>
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="vertices"/> 为空时抛出。
    /// </exception>
    public static List<JTriangle> Tessellate(ReadOnlySpan<JVector> vertices, int subdivisions = 3)
    {
        return Tessellate(new VertexSupportMap(vertices), subdivisions);
    }

    /// <inheritdoc cref="Tessellate(System.ReadOnlySpan{Jitter2.LinearMath.JVector}, int)"/>
    public static List<JTriangle> Tessellate(IEnumerable<JVector> vertices, int subdivisions = 3)
    {
        return Tessellate(new VertexSupportMap(SpanHelper.AsReadOnlySpan(vertices, out _)), subdivisions);
    }

    #region Obsolete MakeHull - Use Tessellate instead

    [Obsolete("Use Tessellate instead.", true)]
    public static List<JTriangle> MakeHull(IEnumerable<JVector> vertices, int subdivisions = 3) =>
        Tessellate(vertices, subdivisions);

    [Obsolete("Use Tessellate instead.", true)]
    public static List<JTriangle> MakeHull(ReadOnlySpan<JVector> vertices, int subdivisions = 3) =>
        Tessellate(vertices, subdivisions);

    [Obsolete("Use Tessellate instead.", true)]
    public static List<JTriangle> MakeHull<TSupport>(in TSupport support, int subdivisions = 3)
        where TSupport : ISupportMappable =>
        Tessellate(in support, subdivisions);

    [Obsolete("Use Tessellate instead.", true)]
    public static void MakeHull<TSupport, TCollection>(in TSupport support, TCollection hullCollection, int subdivisions = 3)
        where TSupport : ISupportMappable
        where TCollection : class, ICollection<JTriangle> =>
        Tessellate(in support, hullCollection, subdivisions);

    #endregion

    /// <summary>
    /// 计算给定朝向和位置的形状的轴对齐包围盒。
    /// </summary>
    /// <param name="support">形状实现的支撑映射接口。</param>
    /// <param name="orientation">形状的朝向。</param>
    /// <param name="position">形状的位置。</param>
    /// <param name="box">得到的包围盒。</param>
    public static void CalculateBoundingBox<TSupport>(in TSupport support,
        in JQuaternion orientation, in JVector position, out JBoundingBox box)
        where TSupport : ISupportMappable
    {
        JMatrix oriT = JMatrix.Transpose(JMatrix.CreateFromQuaternion(orientation));

        JVector axisX = oriT.GetColumn(0);
        JVector axisY = oriT.GetColumn(1);
        JVector axisZ = oriT.GetColumn(2);

        support.SupportMap(axisX, out JVector res);
        box.Max.X = JVector.Dot(axisX, res);

        support.SupportMap(axisY, out res);
        box.Max.Y = JVector.Dot(axisY, res);

        support.SupportMap(axisZ, out res);
        box.Max.Z = JVector.Dot(axisZ, res);

        support.SupportMap(-axisX, out res);
        box.Min.X = JVector.Dot(axisX, res);

        support.SupportMap(-axisY, out res);
        box.Min.Y = JVector.Dot(axisY, res);

        support.SupportMap(-axisZ, out res);
        box.Min.Z = JVector.Dot(axisZ, res);

        JVector.Add(box.Min, position, out box.Min);
        JVector.Add(box.Max, position, out box.Max);
    }

    /// <summary>
    /// 通过对二十面体递归细分生成的支撑点进行采样，近似给定 3D 顶点集的凸包。
    /// </summary>
    /// <param name="vertices">用于近似凸包的顶点。</param>
    /// <param name="subdivisions">
    /// 对每个二十面体三角形施加的递归细分次数。
    /// 值越大，采样方向越多、覆盖越好，
    /// 但计算量也越大。默认为 3。
    /// </param>
    /// <returns>
    /// 凸包上的 <see cref="JVector"/> 点列表，这些点通过从
    /// 精细化的球面分布进行方向性支撑映射采样得到。
    /// </returns>
    /// <remarks>
    /// 此方法从正二十面体开始，递归地将每个三角面细分为更小的
    /// 三角形，并将新顶点投影到单位球面上。每个最终的顶点方向都会传给支撑
    /// 映射器以生成凸包点。时间复杂度为 O(4^n)，其中 n 为细分次数。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="vertices"/> 为空时抛出。
    /// </exception>
    public static List<JVector> SampleHull(ReadOnlySpan<JVector> vertices, int subdivisions = 3)
    {
        return SampleHull(new VertexSupportMap(vertices), subdivisions);
    }

    /// <inheritdoc cref="SampleHull(ReadOnlySpan{JVector}, int)"/>
    public static List<JVector> SampleHull(IEnumerable<JVector> vertices, int subdivisions = 3)
    {
        return SampleHull(new VertexSupportMap(vertices), subdivisions);
    }

    /// <summary>
    /// 通过对二十面体递归细分生成的支撑方向求值，采样凸形状的凸包。
    /// </summary>
    /// <param name="support">
    /// 实现 <see cref="ISupportMappable"/> 的对象，表示一个可用方向性支撑映射
    /// 查询的凸形状。
    /// </param>
    /// <param name="subdivisions">
    /// 对每个二十面体三角形施加的递归细分次数。值越大，采样
    /// 方向越多、覆盖越好，但计算量也越大。默认为 3。
    /// </param>
    /// <returns>
    /// 凸包上的 <see cref="JVector"/> 点列表，这些点通过从
    /// 精细化的球面分布进行方向性支撑映射采样得到。
    /// </returns>
    /// <remarks>
    /// 此方法从正二十面体开始，递归地将每个三角面细分为更小的
    /// 三角形，并将新顶点投影到单位球面上。每个最终的顶点方向都会传给支撑
    /// 映射器以生成凸包点。时间复杂度为 O(4^n)，其中 n 为细分次数。
    /// </remarks>
    public static List<JVector> SampleHull<TSupport>(in TSupport support, int subdivisions = 3)
        where TSupport : ISupportMappable
    {
        int triangleCapacity = GetTessellationTriangleCapacity(subdivisions);
        Stack<(JTriangle triangle, int depth)> stack = new(triangleCapacity);

        for (int i = 0; i < 20; i++)
        {
            JVector v1 = icosahedronVertices[icosahedronIndices[i, 0]];
            JVector v2 = icosahedronVertices[icosahedronIndices[i, 1]];
            JVector v3 = icosahedronVertices[icosahedronIndices[i, 2]];
            stack.Push((new JTriangle(v1, v2, v3), subdivisions));
        }

        HashSet<JVector> hull = new(triangleCapacity);

        while (stack.Count > 0)
        {
            var (tri, depth) = stack.Pop();

            if (depth <= 1)
            {
                support.SupportMap(tri.V0, out JVector sv0);
                support.SupportMap(tri.V1, out JVector sv1);
                support.SupportMap(tri.V2, out JVector sv2);
                hull.Add(sv0); hull.Add(sv1); hull.Add(sv2);
                continue;
            }

            JVector ab = JVector.Normalize((tri.V0 + tri.V1) * (Real)0.5);
            JVector bc = JVector.Normalize((tri.V1 + tri.V2) * (Real)0.5);
            JVector ca = JVector.Normalize((tri.V2 + tri.V0) * (Real)0.5);

            stack.Push((new JTriangle(tri.V0, ab, ca), depth - 1));
            stack.Push((new JTriangle(ab, tri.V1, bc), depth - 1));
            stack.Push((new JTriangle(ca, bc, tri.V2), depth - 1));
            stack.Push((new JTriangle(ab, bc, ca), depth - 1));
        }

        return new List<JVector>(hull);
    }

    private struct MassInertiaSink : ISink<JTriangle>
    {
        public JSymmetricMatrix Inertia;
        public JVector CenterOfMass;
        public Real Mass;

        public void Add(in JTriangle triangle)
        {
            JMatrix transformation = JMatrix.FromColumns(triangle.V0, triangle.V1, triangle.V2);
            Real detA = transformation.Determinant();

            JSymmetricMatrix tetrahedronInertia =
                JSymmetricMatrix.Transform(canonicalTetrahedronInertia, transformation) * detA;

            JVector tetrahedronCom = (Real)(1.0 / 4.0) * (triangle.V0 + triangle.V1 + triangle.V2);
            Real tetrahedronMass = (Real)(1.0 / 6.0) * detA;

            Inertia += tetrahedronInertia;
            CenterOfMass += tetrahedronMass * tetrahedronCom;
            Mass += tetrahedronMass;
        }
    }

    /// <summary>
    /// 计算隐式定义形状的质量属性，假定单位质量密度。
    /// </summary>
    /// <remarks>
    /// <para>
    /// 形状通过表面细分进行近似，细分次数由 <paramref name="subdivisions"/> 指定。
    /// </para>
    /// <para>
    /// <b>关于参考系的说明：</b>
    /// 计算得到的 <paramref name="inertia"/> 张量是相对于 <b>坐标系原点 (0,0,0)</b> 表示的，
    /// <em>而非</em>计算得到的 <paramref name="centerOfMass"/>。
    /// </para>
    /// </remarks>
    /// <param name="support">形状实现的支撑映射接口。</param>
    /// <param name="inertia">
    /// 相对于 <b>原点 (0,0,0)</b> 计算的惯性张量的输出参数。
    /// </param>
    /// <param name="centerOfMass">计算得到的质心向量的输出参数（相对于原点）。</param>
    /// <param name="mass">计算得到的质量的输出参数（体积 * 密度 1.0）。</param>
    /// <param name="subdivisions">表面细分的递归深度（默认 4）。</param>
    public static void CalculateMassInertia<TSupport>(in TSupport support, out JSymmetricMatrix inertia, out JVector centerOfMass,
        out Real mass, int subdivisions = 4)
        where TSupport : ISupportMappable
    {
        MassInertiaSink sink = default;
        // 保留生成的绕序，包括朝内的三角形。带符号的四面体
        // 贡献会抵消重叠的共面片；翻转三角形会破坏这种抵消。
        Tessellate(in support, ref sink, subdivisions);

        inertia = JSymmetricMatrix.Identity * sink.Inertia.Trace() - sink.Inertia;
        centerOfMass = sink.CenterOfMass * ((Real)1.0 / sink.Mass);
        mass = sink.Mass;
    }
}
