/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Jitter2.DataStructures;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 表示由一组三角形定义的凸包形状。
/// </summary>
public class ConvexHullShape : RigidBodyShape, ICloneableShape<ConvexHullShape>
{
    private struct CHullVector(in JVector vertex) : IEquatable<CHullVector>
    {
        public readonly JVector Vertex = vertex;
        // 扁平化后的邻接列表可能先于顶点列表超过 ushort.MaxValue。
        public int NeighborMinIndex = 0;
        public int NeighborMaxIndex = 0;

        public readonly override bool Equals(object? obj)
        {
            return obj is CHullVector other && Equals(other);
        }

        public readonly override int GetHashCode()
        {
            return Vertex.GetHashCode();
        }

        public readonly bool Equals(CHullVector other)
        {
            return Vertex.Equals(other.Vertex);
        }
    }

    private readonly struct CHullTriangle(ushort a, ushort b, ushort c)
    {
        public readonly ushort IndexA = a;
        public readonly ushort IndexB = b;
        public readonly ushort IndexC = c;
    }

    private JBoundingBox cachedBoundingBox;
    private JSymmetricMatrix cachedInertia;
    private Real cachedMass;
    private JVector cachedCenter;

    private CHullVector[] vertices = null!;
    private CHullTriangle[] indices = null!;
    private ushort[] neighborList = null!;

    private JVector shifted;

    /// <inheritdoc/>
    protected internal override void DebugDrawLocal<TSink>(ref TSink triangleSink, in DebugDrawSettings settings)
    {
        foreach (var triangle in indices)
        {
            JVector a = vertices[triangle.IndexA].Vertex + shifted;
            JVector b = vertices[triangle.IndexB].Vertex + shifted;
            JVector c = vertices[triangle.IndexC].Vertex + shifted;
            JVector normal = (b - a) % (c - a);
            if (normal.LengthSquared() == 0) continue;

            // 输入面的绕序可以是任意方向。质心位于凸包内部。
            if (JVector.Dot(normal, a - cachedCenter) < 0) (b, c) = (c, b);
            triangleSink.Add(new JTriangle(a, b, c));
        }
    }

    /// <summary>
    /// 初始化 ConvexHullShape 类的新实例，创建一个凸包。
    /// </summary>
    /// <param name="triangles">描述凸包表面的三角形。其顶点必须位于
    /// 凸包表面上，以避免错误结果或碰撞算法无限挂起。</param>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="triangles"/> 为空，或凸包包含超过
    /// <see cref="ushort.MaxValue"/> 个顶点时抛出。
    /// </exception>
    /// <exception cref="InvalidOperationException">
    /// 当 <paramref name="triangles"/> 未定义非退化体积时抛出。
    /// </exception>
    public ConvexHullShape(ReadOnlySpan<JTriangle> triangles)
    {
        if (triangles.Length == 0)
        {
            throw new ArgumentException("Triangle set must contain at least one triangle.", nameof(triangles));
        }

        int maxVertexCount = triangles.Length * 3;
        Dictionary<CHullVector, ushort> tmpIndices = new(maxVertexCount);
        List<CHullVector> tmpVertices = new(maxVertexCount);

        ushort PushVector(CHullVector v)
        {
            if (tmpIndices.TryGetValue(v, out ushort result)) return result;

            if (tmpVertices.Count >= ushort.MaxValue)
            {
                throw new ArgumentException(
                    $"The convex hull consists of too many vertices (>{ushort.MaxValue}).", nameof(triangles));
            }

            result = (ushort)tmpVertices.Count;
            tmpIndices.Add(v, result);
            tmpVertices.Add(v);

            return result;
        }

        foreach (var tti in triangles)
        {
            PushVector(new CHullVector(tti.V0));
            PushVector(new CHullVector(tti.V1));
            PushVector(new CHullVector(tti.V2));
        }

        var tmpNeighbors = new List<ushort>[tmpVertices.Count];
        indices = new CHullTriangle[triangles.Length];

        for (int i = 0; i < triangles.Length; i++)
        {
            JTriangle tti = triangles[i];

            ushort a = PushVector(new CHullVector(tti.V0));
            ushort b = PushVector(new CHullVector(tti.V1));
            ushort c = PushVector(new CHullVector(tti.V2));

            indices[i] = new CHullTriangle(a, b, c);

            (tmpNeighbors[a] ??= []).Add(b);
            tmpNeighbors[a].Add(c);

            (tmpNeighbors[b] ??= []).Add(a);
            tmpNeighbors[b].Add(c);

            (tmpNeighbors[c] ??= []).Add(a);
            tmpNeighbors[c].Add(b);
        }

        // 预估容量以避免扩容（欧拉示性数约为每个顶点 6 个邻居）
        var finalNeighbors = new List<ushort>(tmpVertices.Count * 6);

        var tmpVerticesSpan = CollectionsMarshal.AsSpan(tmpVertices);

        for (int i = 0; i < tmpVerticesSpan.Length; i++)
        {
            ref var element = ref tmpVerticesSpan[i];
            element.NeighborMinIndex = finalNeighbors.Count;
            AddDistinct(tmpNeighbors[i], finalNeighbors);
            element.NeighborMaxIndex = finalNeighbors.Count;

            // 帮助 GC 回收
            tmpNeighbors[i] = null!;
        }

        vertices = tmpVertices.ToArray();
        neighborList = finalNeighbors.ToArray();

        tmpIndices.Clear();
        tmpVertices.Clear();

        UpdateShape();
    }

    /// <inheritdoc cref="ConvexHullShape(ReadOnlySpan{JTriangle})"/>
    public ConvexHullShape(IEnumerable<JTriangle> triangles) :
        this(SpanHelper.AsReadOnlySpan(triangles, out _))
    {
    }

    private ConvexHullShape()
    {
    }

    /// <summary>
    /// 辅助方法：对源列表排序并将唯一元素添加到目标列表。
    /// 替代 LINQ Distinct()，以获得更好的性能和零分配。
    /// </summary>
    private static void AddDistinct(List<ushort>? source, List<ushort> destination)
    {
        if (source == null || source.Count == 0) return;

        source.Sort();

        ushort previous = source[0];
        destination.Add(previous);

        for (int i = 1; i < source.Count; i++)
        {
            ushort current = source[i];
            if (current != previous)
            {
                destination.Add(current);
                previous = current;
            }
        }
    }

    /// <summary>
    /// 创建凸包形状的克隆。请注意，底层数据结构在各实例之间共享。
    /// </summary>
    /// <returns>ConvexHullShape 类的新实例，它与原始实例共享相同的底层数据结构。
    /// </returns>
    public ConvexHullShape Clone()
    {
        ConvexHullShape result = new()
        {
            neighborList = neighborList,
            vertices = vertices,
            indices = indices,
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
    /// 获取或设置应用于凸包顶点的偏移。
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
    /// 当凸包未定义非退化体积时抛出。
    /// </exception>
    public void UpdateShape()
    {
        CalculateMassInertia();
        CalcInitBox();
        UpdateWorldBoundingBox();
    }

    /// <inheritdoc/>
    public override void CalculateMassInertia(out JSymmetricMatrix inertia, out JVector com, out Real mass)
    {
        inertia = cachedInertia;
        com = cachedCenter;
        mass = cachedMass;
    }

    /// <summary>
    /// 根据凸包三角形重新计算质量、质心和惯性张量。
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// 当凸包未定义非退化体积时抛出。
    /// </exception>
    public void CalculateMassInertia()
    {
        cachedCenter = JVector.Zero;
        cachedInertia = JSymmetricMatrix.Zero;
        cachedMass = 0;

        const Real a = (Real)(1.0 / 60.0);
        const Real b = (Real)(1.0 / 120.0);
        JSymmetricMatrix canonicalInertia = new(a, b, b, a, b, a);

        JVector pointWithin = JVector.Zero;

        for (int i = 0; i < vertices.Length; i++)
        {
            pointWithin += vertices[i].Vertex;
        }

        pointWithin = pointWithin * ((Real)1.0 / vertices.Length) + shifted;

        foreach (CHullTriangle tri in indices)
        {
            JVector column0 = vertices[tri.IndexA].Vertex + shifted;
            JVector column1 = vertices[tri.IndexB].Vertex + shifted;
            JVector column2 = vertices[tri.IndexC].Vertex + shifted;

            // 检查绕序
            {
                JVector normal = (column1 - column0) % (column2 - column0);
                Real dot = JVector.Dot(normal, column0 - pointWithin);
                if (dot < (Real)0.0)
                {
                    (column0, column1) = (column1, column0);
                }
            }

            JMatrix transformation = new(
                column0.X, column1.X, column2.X,
                column0.Y, column1.Y, column2.Y,
                column0.Z, column1.Z, column2.Z);

            Real detA = transformation.Determinant();

            JSymmetricMatrix tetrahedronInertia = JSymmetricMatrix.Transform(canonicalInertia, transformation) * detA;

            JVector tetrahedronCom = (Real)(1.0 / 4.0) * (column0 + column1 + column2);
            Real tetrahedronMass = (Real)(1.0 / 6.0) * detA;

            cachedInertia += tetrahedronInertia;
            cachedCenter += tetrahedronMass * tetrahedronCom;
            cachedMass += tetrahedronMass;
        }

        const Real minimumMass = (Real)1e-12;

        if (!Real.IsFinite(cachedMass) || cachedMass <= minimumMass)
        {
            throw new InvalidOperationException("Convex hull must define a non-degenerate volume.");
        }

        cachedInertia = JSymmetricMatrix.Identity * cachedInertia.Trace() - cachedInertia;
        cachedCenter *= (Real)1.0 / cachedMass;
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
        InternalSupportMap(vec, out JVector res);
        cachedBoundingBox.Max.X = res.X;

        vec = JVector.UnitY;
        InternalSupportMap(vec, out res);
        cachedBoundingBox.Max.Y = res.Y;

        vec = JVector.UnitZ;
        InternalSupportMap(vec, out res);
        cachedBoundingBox.Max.Z = res.Z;

        vec = -JVector.UnitX;
        InternalSupportMap(vec, out res);
        cachedBoundingBox.Min.X = res.X;

        vec = -JVector.UnitY;
        InternalSupportMap(vec, out res);
        cachedBoundingBox.Min.Y = res.Y;

        vec = -JVector.UnitZ;
        InternalSupportMap(vec, out res);
        cachedBoundingBox.Min.Z = res.Z;
    }

    private ushort InternalSupportMap(in JVector direction, out JVector result)
    {
        ushort current = 0;
        Real dotProduct = JVector.Dot(vertices[current].Vertex, direction);

    main:
        bool needsVerify = false;
        JVector verifyDir = JVector.Arbitrary;

        var min = vertices[current].NeighborMinIndex;
        var max = vertices[current].NeighborMaxIndex;

        const Real epsilonIncrement = (Real)1e-12;

        for (int i = min; i < max; i++)
        {
            ushort nb = neighborList[i];
            Real nbProduct = JVector.Dot(vertices[nb].Vertex, direction);

            if (MathR.Abs(nbProduct - dotProduct) < epsilonIncrement)
            {
                verifyDir = vertices[nb].Vertex - vertices[current].Vertex;
                needsVerify = true;
            }

            if (nbProduct > dotProduct + epsilonIncrement)
            {
                // 无需寻找“最佳”邻居——一旦找到更好的
                // 候选点就移动到那里。
                dotProduct = nbProduct;
                current = nb;
                goto main;
            }
        }

        // 在出现平台时使用的次级爬山算法。
        if (needsVerify)
        {
            Real d0 = JVector.Dot(verifyDir, vertices[current].Vertex);

        secondary:
            min = vertices[current].NeighborMinIndex;
            max = vertices[current].NeighborMaxIndex;

            for (int i = min; i < max; i++)
            {
                ushort nb = neighborList[i];
                Real nbProduct = JVector.Dot(vertices[nb].Vertex, direction);

                if (nbProduct > dotProduct + epsilonIncrement)
                {
                    dotProduct = nbProduct;
                    current = nb;
                    goto main;
                }

                if (MathR.Abs(nbProduct - dotProduct) < epsilonIncrement)
                {
                    Real d1 = JVector.Dot(verifyDir, vertices[nb].Vertex);

                    if (d1 > d0 + epsilonIncrement)
                    {
                        d0 = d1;
                        current = nb;
                        goto secondary;
                    }
                }
            }
        }

        result = vertices[current].Vertex + shifted;
        return current;
    }

    /// <inheritdoc/>
    public override void SupportMap(in JVector direction, out JVector result)
    {
        InternalSupportMap(direction, out result);
    }

    /// <inheritdoc/>
    public override void GetCenter(out JVector point)
    {
        point = cachedCenter;
    }
}
