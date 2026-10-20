/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Jitter2.LinearMath;
using Vertex = Jitter2.Collision.MinkowskiDifference.Vertex;

namespace Jitter2.Collision;

/// <summary>
/// 带过滤后精确可见性、朝向和退化判定的 EPA 多面体，
/// 这些判定基于给定的浮点坐标。
/// </summary>
/// <remarks>
/// 距离、重心坐标和接触输出仍然是近似的，并以双精度计算。
/// 托管存储由 <see cref="InitHeap"/> 分配并复用。实例不得在线程间共享。
/// 每个非零的 Minkowski 顶点坐标的绝对值必须介于 2^-200
/// 与 2^200 之间（含端点）。超出此范围的种子与扩展会被拒绝。
/// </remarks>
public unsafe struct ConvexPolytope
{
    [StructLayout(LayoutKind.Sequential)]
    public struct Triangle
    {
        public short A, B, C;
        public bool FacingOrigin;

        public short this[int i] => ((short*)Unsafe.AsPointer(ref this))[i];

        public JVector Normal;
        public JVector ClosestToOrigin;

        public Real NormalSq;
        public Real ClosestToOriginSq;
    }

    public enum AddVertexResult
    {
        Added,
        NoHorizon,
        Degenerate,
        CapacityExceeded,
        NumericalFailure
    }

    private const int MaxVertices = 128;
    private const int MaxTriangles = 2 * MaxVertices;

    private readonly struct Edge(short a, short b)
    {
        public readonly short A = a, B = b;
        public bool Matches(in Edge other) =>
            (A == other.A && B == other.B) || (A == other.B && B == other.A);
    }

    private readonly struct Double3(double x, double y, double z)
    {
        public readonly double X = x, Y = y, Z = z;
        public bool IsFinite => double.IsFinite(X) && double.IsFinite(Y) && double.IsFinite(Z);
        public JVector ToVector() => new((Real)X, (Real)Y, (Real)Z);
        public static Double3 FromVector(in JVector v) => new(v.X, v.Y, v.Z);
        public static double Dot(in Double3 a, in Double3 b) => a.X * b.X + a.Y * b.Y + a.Z * b.Z;
        public static Double3 Cross(in Double3 a, in Double3 b) =>
            new(a.Y * b.Z - a.Z * b.Y, a.Z * b.X - a.X * b.Z, a.X * b.Y - a.Y * b.X);
        public static Double3 operator +(Double3 a, Double3 b) => new(a.X + b.X, a.Y + b.Y, a.Z + b.Z);
        public static Double3 operator -(Double3 a, Double3 b) => new(a.X - b.X, a.Y - b.Y, a.Z - b.Z);
        public static Double3 operator *(Double3 a, double b) => new(a.X * b, a.Y * b, a.Z * b);
    }

    private struct FaceData
    {
        public PlaneFilter Filter;
        public Double3 Barycentric;
        public double DistanceSquared;
    }

    private Vertex[] vertices;
    private Triangle[] triangles;
    private FaceData[] faceData;
    private short vertexCount, triangleCount;
    private bool originEnclosed;

    public readonly Span<Triangle> HullTriangles => triangles.AsSpan(0, triangleCount);

    /// <summary>
    /// 指示原点是否被包围。由 <see cref="GetClosestTriangle"/> 更新。
    /// </summary>
    public readonly bool OriginEnclosed => originEnclosed;

    public ref Vertex GetVertex(int index)
    {
        Debug.Assert(index < MaxVertices, "Out of bounds.");
        return ref vertices[index];
    }

    /// <summary>一次性分配可复用的托管缓冲区。在设置顶点或初始化凸包之前调用。</summary>
    public void InitHeap()
    {
        if (vertices != null) return;
        vertices = new Vertex[MaxVertices];
        triangles = new Triangle[MaxTriangles];
        faceData = new FaceData[MaxTriangles];
    }

    /// <summary>从前四个顶点初始化。返回 false 表示种子无效或不受支持。</summary>
    public bool InitTetrahedron()
    {
        vertexCount = 4;
        triangleCount = 0;
        originEnclosed = false;
        for (int i = 0; i < 4; i++)
        {
            if (!PlaneFilter.IsSafe(vertices[i].V)) return false;
        }

        ReadOnlySpan<short> indices = [0, 2, 1, 0, 1, 3, 0, 3, 2, 1, 2, 3];
        for (int i = 0; i < 4; i++)
        {
            if (CreateTriangle(indices[3 * i], indices[3 * i + 1], indices[3 * i + 2],
                    out triangles[i], out faceData[i]) != AddVertexResult.Added) return false;
        }

        triangleCount = 4;
        return true;
    }

    /// <summary>以指定点为中心初始化一个小四面体。</summary>
    /// <returns>如果种子退化、不受支持或无法求值，则返回 false。</returns>
    public bool InitTetrahedron(in JVector point)
    {
        const Real scale = (Real)1e-2;
        vertices[0] = new Vertex(point + scale * new JVector(MathR.Sqrt((Real)(8.0 / 9.0)), 0, -(Real)(1.0 / 3.0)));
        vertices[1] = new Vertex(point + scale * new JVector(-MathR.Sqrt((Real)(2.0 / 9.0)), MathR.Sqrt((Real)(2.0 / 3.0)), -(Real)(1.0 / 3.0)));
        vertices[2] = new Vertex(point + scale * new JVector(-MathR.Sqrt((Real)(2.0 / 9.0)), -MathR.Sqrt((Real)(2.0 / 3.0)), -(Real)(1.0 / 3.0)));
        vertices[3] = new Vertex(point + scale * new JVector(0, 0, 1));
        return InitTetrahedron();
    }

    /// <summary>使用双精度距离查找最近的三角形，并更新原点是否被包围的状态。</summary>
    public ref Triangle GetClosestTriangle()
    {
        if (triangleCount == 0) throw new InvalidOperationException("The polytope is not initialized.");
        int closest = 0;
        double minimum = double.PositiveInfinity;
        bool skipTest = originEnclosed;
        originEnclosed = true;
        for (int i = 0; i < triangleCount; i++)
        {
            if (faceData[i].DistanceSquared < minimum)
            {
                minimum = faceData[i].DistanceSquared;
                closest = i;
            }

            if (!triangles[i].FacingOrigin) originEnclosed = skipTest;
        }

        return ref triangles[closest];
    }

    /// <summary>根据仍属于当前凸包的三角形计算形状接触点。</summary>
    public void CalculatePoints(in Triangle triangle, out JVector pointA, out JVector pointB)
    {
        for (int i = 0; i < triangleCount; i++)
        {
            if (triangles[i].A != triangle.A || triangles[i].B != triangle.B || triangles[i].C != triangle.C) continue;
            Double3 bc = faceData[i].Barycentric;
            pointA = (Double3.FromVector(vertices[triangle.A].A) * bc.X +
                      Double3.FromVector(vertices[triangle.B].A) * bc.Y +
                      Double3.FromVector(vertices[triangle.C].A) * bc.Z).ToVector();
            pointB = (Double3.FromVector(vertices[triangle.A].B) * bc.X +
                      Double3.FromVector(vertices[triangle.B].B) * bc.Y +
                      Double3.FromVector(vertices[triangle.C].B) * bc.Z).ToVector();
            return;
        }

        throw new ArgumentException("Triangle no longer belongs to this hull.", nameof(triangle));
    }

    /// <summary>添加一个顶点。使用 <see cref="AddVertexDetailed"/> 区分正常停止与失败。</summary>
    /// <remarks>
    /// 成功扩展会使先前的三角形引用失效。NoHorizon 会保留凸包；
    /// 其他任何失败都需要重新初始化后才能再次使用凸包。
    /// </remarks>
    public bool AddVertex(in Vertex vertex) => AddVertexDetailed(vertex) == AddVertexResult.Added;

    /// <summary>
    /// 就地扩展凸包。NoHorizon 表示正常地未发生扩展，活动凸包保持不变。
    /// 其他任何失败都需要重新初始化。
    /// </summary>
    [SkipLocalsInit]
    public AddVertexResult AddVertexDetailed(in Vertex vertex)
    {
        if (triangleCount == 0 || !PlaneFilter.IsSafe(vertex.V)) return AddVertexResult.NumericalFailure;
        if (vertexCount == MaxVertices) return AddVertexResult.CapacityExceeded;
        vertices[vertexCount] = vertex;

        Span<bool> lit = stackalloc bool[MaxTriangles];
        Span<Edge> edges = stackalloc Edge[3 * MaxTriangles];
        int edgeCount = 0, litCount = 0;
        for (int index = triangleCount; index-- > 0;)
        {
            ref readonly Triangle face = ref triangles[index];
            lit[index] = Side(faceData[index], face.A, face.B, face.C, vertexCount) > 0;
            if (!lit[index]) continue;
            litCount++;
            Triangle tr = triangles[index];
            ReadOnlySpan<short> corners = [tr.A, tr.B, tr.C];
            for (int k = 0; k < 3; k++)
            {
                Edge edge = new(corners[k], corners[(k + 1) % 3]);
                bool added = true;
                for (int e = edgeCount; e-- > 0;)
                {
                    if (!edges[e].Matches(edge)) continue;
                    edges[e] = edges[--edgeCount];
                    added = false;
                    break;
                }

                if (added) edges[edgeCount++] = edge;
            }
        }

        if (edgeCount == 0) return AddVertexResult.NoHorizon;
        if (triangleCount - litCount + edgeCount > MaxTriangles) return AddVertexResult.CapacityExceeded;

        int survivorCount = 0;
        for (int i = 0; i < triangleCount; i++)
        {
            if (lit[i]) continue;
            if (survivorCount != i)
            {
                triangles[survivorCount] = triangles[i];
                faceData[survivorCount] = faceData[i];
            }
            survivorCount++;
        }

        // 直接在最终槽位中构建替换用的扇形面。某个面创建失败
        // 会使凸包失效；EPA 调用方会放弃本次求解并重新初始化它。
        for (int i = 0; i < edgeCount; i++)
        {
            int index = survivorCount + i;
            AddVertexResult result = CreateTriangle(edges[i].A, edges[i].B, vertexCount,
                out triangles[index], out faceData[index]);
            if (result != AddVertexResult.Added)
            {
                triangleCount = 0;
                originEnclosed = false;
                return result;
            }
        }

        triangleCount = (short)(survivorCount + edgeCount);
        vertexCount++;
        return AddVertexResult.Added;
    }

    private AddVertexResult CreateTriangle(short a, short b, short c, out Triangle triangle, out FaceData data)
    {
        triangle = default;
        data = default;
        data.Filter = new PlaneFilter(vertices[a].V, vertices[b].V, vertices[c].V);
        // 四个种子顶点始终位于不断增长的凸包内部。其中任意一个具有非零
        // 侧值即可同时证明面积非零和面朝向正确；
        // 这避免了为每次查询构造精确质心。
        int orientation = 0;
        for (short i = 0; i < 4 && orientation == 0; i++)
        {
            if (i == a || i == b || i == c) continue;
            orientation = -Side(data, a, b, c, i);
        }
        if (orientation == 0) return AddVertexResult.Degenerate;
        if (orientation < 0)
        {
            (a, b) = (b, a);
            data.Filter.Negate();
        }

        bool facingOrigin = Side(data, a, b, c, -1) <= 0;
        Double3 rawNormal = new(data.Filter.NormalX, data.Filter.NormalY, data.Filter.NormalZ);
        if (!data.Filter.ReliableNormal)
        {
            ExpansionArithmetic.Normal(vertices[a].V, vertices[b].V, vertices[c].V, out double x, out double y, out double z);
            rawNormal = new(x, y, z);
        }
        double normalScale = Math.Max(Math.Abs(rawNormal.X), Math.Max(Math.Abs(rawNormal.Y), Math.Abs(rawNormal.Z)));
        if (!(normalScale > 0) || !double.IsFinite(normalScale)) return AddVertexResult.NumericalFailure;
        Double3 scaledNormal = new(rawNormal.X / normalScale, rawNormal.Y / normalScale, rawNormal.Z / normalScale);
        double normalSquared = Double3.Dot(scaledNormal, scaledNormal);

        Double3 va = Double3.FromVector(vertices[a].V), vb = Double3.FromVector(vertices[b].V), vc = Double3.FromVector(vertices[c].V);
        bool clamped = CalcBarycentric(va, vb, vc, scaledNormal, normalScale, normalSquared, out data.Barycentric);
        Double3 closest = clamped
            ? va * data.Barycentric.X + vb * data.Barycentric.Y + vc * data.Barycentric.Z
            : scaledNormal * (Double3.Dot(scaledNormal, va) / normalSquared);
        data.DistanceSquared = Double3.Dot(closest, closest);
        if (!closest.IsFinite || !data.Barycentric.IsFinite || !double.IsFinite(data.DistanceSquared)) return AddVertexResult.NumericalFailure;

        triangle.A = a;
        triangle.B = b;
        triangle.C = c;
        triangle.FacingOrigin = facingOrigin;
        triangle.Normal = scaledNormal.ToVector();
        triangle.NormalSq = triangle.Normal.LengthSquared();
        triangle.ClosestToOrigin = closest.ToVector();
        triangle.ClosestToOriginSq = (Real)data.DistanceSquared;
        return IsFinite(triangle.ClosestToOrigin) && Real.IsFinite(triangle.ClosestToOriginSq)
            ? AddVertexResult.Added : AddVertexResult.NumericalFailure;
    }

    private int Side(in FaceData data, short a, short b, short c, short pointIndex)
    {
        JVector point = pointIndex < 0 ? JVector.Zero : vertices[pointIndex].V;
        if (point == vertices[a].V || point == vertices[b].V || point == vertices[c].V) return 0;
        if (data.Filter.TrySide(vertices[a].V, point, out int sign)) return sign;
        return ExpansionArithmetic.Side(vertices[a].V, vertices[b].V, vertices[c].V, point);
    }

    private static bool CalcBarycentric(in Double3 a, in Double3 b, in Double3 c,
        in Double3 normal, double normalScale, double normalSquared, out Double3 result)
    {
        Double3 u = a - b, v = a - c;
        double gamma = (Double3.Dot(Double3.Cross(u, a), normal) / normalSquared) / normalScale;
        double beta = (Double3.Dot(Double3.Cross(a, v), normal) / normalSquared) / normalScale;
        double alpha = 1 - gamma - beta;
        bool clamped = false;
        if (alpha >= 0 && beta < 0)
        {
            double t = Double3.Dot(a, u);
            if (gamma < 0 && t > 0)
            {
                beta = Math.Min(1, t / Double3.Dot(u, u));
                alpha = 1 - beta;
                gamma = 0;
            }
            else
            {
                gamma = Math.Clamp(Double3.Dot(a, v) / Double3.Dot(v, v), 0, 1);
                alpha = 1 - gamma;
                beta = 0;
            }

            clamped = true;
        }
        else if (beta >= 0 && gamma < 0)
        {
            Double3 w = b - c;
            double t = Double3.Dot(b, w);
            if (alpha < 0 && t > 0)
            {
                gamma = Math.Min(1, t / Double3.Dot(w, w));
                beta = 1 - gamma;
                alpha = 0;
            }
            else
            {
                alpha = Math.Clamp(-Double3.Dot(b, u) / Double3.Dot(u, u), 0, 1);
                beta = 1 - alpha;
                gamma = 0;
            }

            clamped = true;
        }
        else if (gamma >= 0 && alpha < 0)
        {
            Double3 w = b - c;
            double t = -Double3.Dot(c, v);
            if (beta < 0 && t > 0)
            {
                alpha = Math.Min(1, t / Double3.Dot(v, v));
                gamma = 1 - alpha;
                beta = 0;
            }
            else
            {
                beta = Math.Clamp(-Double3.Dot(c, w) / Double3.Dot(w, w), 0, 1);
                gamma = 1 - beta;
                alpha = 0;
            }

            clamped = true;
        }

        result = new Double3(alpha, beta, gamma);
        return clamped;
    }

    private static bool IsFinite(in JVector v) => Real.IsFinite(v.X) && Real.IsFinite(v.Y) && Real.IsFinite(v.Z);
}
