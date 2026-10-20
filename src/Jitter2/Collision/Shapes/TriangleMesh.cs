/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Jitter2.DataStructures;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// 表示由一组顶点和三角形索引定义的三角形网格。
/// </summary>
public class TriangleMesh
{
    public sealed class DegenerateTriangleException : Exception
    {
        public DegenerateTriangleException()
        {
        }

        public DegenerateTriangleException(string message)
            : base(message)
        {
        }

        public DegenerateTriangleException(string message, Exception innerException)
            : base(message, innerException)
        {
        }

        public DegenerateTriangleException(JTriangle triangle)
            : base($"Degenerate triangle found: {triangle}.")
        {
            Triangle = triangle;
        }

        public JTriangle Triangle { get; }
    }

    private readonly struct Edge(int indexA, int indexB) : IEquatable<Edge>
    {
        public readonly int IndexA = indexA;
        public readonly int IndexB = indexB;
        public bool Equals(Edge other) => IndexA == other.IndexA && IndexB == other.IndexB;
        public override bool Equals(object? obj) => obj is Edge other && Equals(other);
        public override int GetHashCode() => HashCode.Combine(IndexA, IndexB);
    }

    /// <summary>网格中一个三角形的索引、邻居以及局部空间法线。</summary>
    public struct Triangle(int a, int b, int c)
    {
        /// <summary>三角形各顶点在 <see cref="Vertices"/> 中的索引。</summary>
        public readonly int IndexA = a, IndexB = b, IndexC = c;
        /// <summary>与对应顶点相对的邻居三角形索引，位于边界处时为 -1。</summary>
        public int NeighborA = -1, NeighborB = -1, NeighborC = -1;
        /// <summary>三角形在网格局部坐标中的法线。</summary>
        public JVector Normal = default;
    }

    private readonly JVector[] vertices = null!;
    private readonly Triangle[] indices = null!;

    /// <summary>
    /// 获取网格的顶点。
    /// </summary>
    public ReadOnlySpan<JVector> Vertices => vertices;

    /// <summary>
    /// 获取网格的三角形索引。
    /// </summary>
    public ReadOnlySpan<Triangle> Indices => indices;

    /// <summary>
    /// 从三角形“汤”创建网格。顶点会被自动识别并去重。
    /// </summary>
    /// <param name="soup">用于构建网格的三角形。</param>
    /// <param name="ignoreDegenerated">如果为 <see langword="true"/>，则跳过退化三角形。</param>
    /// <exception cref="DegenerateTriangleException">
    /// 当发现退化三角形且 <paramref name="ignoreDegenerated"/> 为 <see langword="false"/> 时抛出。
    /// </exception>
    public TriangleMesh(ReadOnlySpan<JTriangle> soup, bool ignoreDegenerated = false)
    {
        BuildFromSoup(soup, ignoreDegenerated);
    }

    /// <inheritdoc cref="TriangleMesh(ReadOnlySpan{JTriangle}, bool)"/>
    public TriangleMesh(IEnumerable<JTriangle> soup, bool ignoreDegenerated = false) :
        this(SpanHelper.AsReadOnlySpan(soup, out _), ignoreDegenerated)
    {
    }

    /// <summary>
    /// 从现有顶点和索引创建网格。
    /// </summary>
    /// <param name="vertices">顶点缓冲区。</param>
    /// <param name="indices">索引缓冲区（长度必须是 3 的倍数）。</param>
    /// <param name="ignoreDegenerated">如果为 <see langword="true"/>，则跳过退化三角形。</param>
    /// <remarks>
    /// 位置完全相同的顶点会被规范化，因此邻接检测在带索引的网格中也能跨重复的接缝顶点生效。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="indices"/> 的条目数不是三的倍数时抛出。
    /// </exception>
    /// <exception cref="IndexOutOfRangeException">
    /// 当某个索引引用了 <paramref name="vertices"/> 之外的元素时抛出。
    /// </exception>
    /// <exception cref="DegenerateTriangleException">
    /// 当发现退化三角形且 <paramref name="ignoreDegenerated"/> 为 <see langword="false"/> 时抛出。
    /// </exception>
    public TriangleMesh(ReadOnlySpan<JVector> vertices, ReadOnlySpan<int> indices, bool ignoreDegenerated = false)
    {
        BuildFromIndexed(vertices, indices, ignoreDegenerated);
    }

    /// <inheritdoc cref="TriangleMesh(ReadOnlySpan{JVector}, ReadOnlySpan{int}, bool)"/>
    public TriangleMesh(ReadOnlySpan<JVector> vertices, ReadOnlySpan<ushort> indices, bool ignoreDegenerated = false)
    {
        // 将 ushort 索引即时转换为 int
        int[] intIndices = new int[indices.Length];
        for (int i = 0; i < indices.Length; i++) intIndices[i] = indices[i];

        BuildFromIndexed(vertices, intIndices, ignoreDegenerated);
    }

    /// <inheritdoc cref="TriangleMesh(ReadOnlySpan{JVector}, ReadOnlySpan{int}, bool)"/>
    public TriangleMesh(ReadOnlySpan<JVector> vertices, ReadOnlySpan<uint> indices, bool ignoreDegenerated = false)
    {
        int[] intIndices = new int[indices.Length];
        for (int i = 0; i < indices.Length; i++) intIndices[i] = (int)indices[i];

        BuildFromIndexed(vertices, intIndices, ignoreDegenerated);
    }

    /// <summary>
    /// 从自定义顶点类型（例如 System.Numerics.Vector3）和索引创建网格。
    /// </summary>
    /// <typeparam name="TVertex">非托管顶点类型，其大小必须与 <see cref="JVector"/> 相同。</typeparam>
    /// <param name="vertices">顶点缓冲区。</param>
    /// <param name="indices">索引缓冲区（长度必须是 3 的倍数）。</param>
    /// <param name="ignoreDegenerated">如果为 <see langword="true"/>，则跳过退化三角形。</param>
    /// <exception cref="ArgumentException">
    /// 当 <typeparamref name="TVertex"/> 的大小与 <see cref="JVector"/> 不同时抛出。
    /// </exception>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="indices"/> 的条目数不是三的倍数时抛出。
    /// </exception>
    /// <exception cref="IndexOutOfRangeException">
    /// 当某个索引引用了 <paramref name="vertices"/> 之外的元素时抛出。
    /// </exception>
    /// <exception cref="DegenerateTriangleException">
    /// 当发现退化三角形且 <paramref name="ignoreDegenerated"/> 为 <see langword="false"/> 时抛出。
    /// </exception>
    public static TriangleMesh Create<TVertex>(ReadOnlySpan<TVertex> vertices, ReadOnlySpan<int> indices,
        bool ignoreDegenerated = false) where TVertex : unmanaged
    {
        return new TriangleMesh(CastVertices(vertices, nameof(vertices)), indices, ignoreDegenerated);
    }

    /// <inheritdoc cref="Create{TVertex}(ReadOnlySpan{TVertex}, ReadOnlySpan{int}, bool)"/>
    public static TriangleMesh Create<TVertex>(ReadOnlySpan<TVertex> vertices, ReadOnlySpan<uint> indices,
        bool ignoreDegenerated = false) where TVertex : unmanaged
    {
        return new TriangleMesh(CastVertices(vertices, nameof(vertices)), indices, ignoreDegenerated);
    }

    /// <inheritdoc cref="Create{TVertex}(ReadOnlySpan{TVertex}, ReadOnlySpan{int}, bool)"/>
    public static TriangleMesh Create<TVertex>(ReadOnlySpan<TVertex> vertices, ReadOnlySpan<ushort> indices,
        bool ignoreDegenerated = false) where TVertex : unmanaged
    {
        return new TriangleMesh(CastVertices(vertices, nameof(vertices)), indices, ignoreDegenerated);
    }

    // 辅助方法：将类型转换逻辑集中在一处
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static ReadOnlySpan<JVector> CastVertices<TVertex>(ReadOnlySpan<TVertex> vertices, string paramName)
        where TVertex : unmanaged
    {
        if (Unsafe.SizeOf<TVertex>() != Unsafe.SizeOf<JVector>())
        {
            throw new ArgumentException($"Size mismatch: {typeof(TVertex).Name} ({Unsafe.SizeOf<TVertex>()} bytes) " +
                                        $"does not match JVector ({Unsafe.SizeOf<JVector>()} bytes).", paramName);
        }

        return MemoryMarshal.Cast<TVertex, JVector>(vertices);
    }

    private void BuildFromSoup(ReadOnlySpan<JTriangle> triangles, bool ignoreDegenerated)
    {
        int maxVertexCount = triangles.Length * 3;
        var vertexMap = new Dictionary<JVector, int>(maxVertexCount);
        var vertexList = new List<JVector>(maxVertexCount);
        var triangleList = new List<Triangle>(triangles.Length);

        // 辅助方法：对顶点去重
        int GetOrAddVertex(JVector v)
        {
            if (!vertexMap.TryGetValue(v, out int index))
            {
                index = vertexList.Count;
                vertexMap[v] = index;
                vertexList.Add(v);
            }
            return index;
        }

        foreach (ref readonly var tri in triangles)
        {
            JVector normal = (tri.V1 - tri.V0) % (tri.V2 - tri.V0);

            if (MathHelper.CloseToZero(normal, (Real)1e-12))
            {
                if (ignoreDegenerated) continue;
                throw new DegenerateTriangleException(tri);
            }

            // 去重
            int a = GetOrAddVertex(tri.V0);
            int b = GetOrAddVertex(tri.V1);
            int c = GetOrAddVertex(tri.V2);

            var internalTri = new Triangle(a, b, c);
            JVector.Normalize(normal, out internalTri.Normal);
            triangleList.Add(internalTri);
        }

        Unsafe.AsRef(in vertices) = vertexList.ToArray();
        Unsafe.AsRef(in indices) = triangleList.ToArray();

        AssignNeighbors();
    }

    private void BuildFromIndexed(ReadOnlySpan<JVector> vertices, ReadOnlySpan<int> indices, bool ignoreDegenerated)
    {
        if (indices.Length % 3 != 0) throw new ArgumentException("Indices must be a multiple of 3.", nameof(indices));

        var deduplicatedVertices = new List<JVector>(vertices.Length);
        var vertexMap = new Dictionary<JVector, int>(vertices.Length);
        int[] remap = new int[vertices.Length];

        for (int i = 0; i < vertices.Length; i++)
        {
            JVector vertex = vertices[i];

            if (!vertexMap.TryGetValue(vertex, out int canonicalIndex))
            {
                canonicalIndex = deduplicatedVertices.Count;
                vertexMap[vertex] = canonicalIndex;
                deduplicatedVertices.Add(vertex);
            }

            remap[i] = canonicalIndex;
        }

        Unsafe.AsRef(in this.vertices) = deduplicatedVertices.ToArray();

        var triangleList = new List<Triangle>(indices.Length / 3);

        for (int i = 0; i < indices.Length; i += 3)
        {
            int inputI0 = indices[i];
            int inputI1 = indices[i + 1];
            int inputI2 = indices[i + 2];

            // 边界安全检查
            if ((uint)inputI0 >= vertices.Length || (uint)inputI1 >= vertices.Length || (uint)inputI2 >= vertices.Length)
                throw new IndexOutOfRangeException($"Indices {inputI0},{inputI1},{inputI2} out of bounds.");

            int i0 = remap[inputI0];
            int i1 = remap[inputI1];
            int i2 = remap[inputI2];

            JVector v0 = this.vertices[i0];
            JVector v1 = this.vertices[i1];
            JVector v2 = this.vertices[i2];

            JVector normal = (v1 - v0) % (v2 - v0);

            if (MathHelper.CloseToZero(normal, (Real)1e-12))
            {
                if (ignoreDegenerated) continue;
                throw new DegenerateTriangleException(new JTriangle(v0, v1, v2));
            }

            var tri = new Triangle(i0, i1, i2);
            JVector.Normalize(normal, out tri.Normal);
            triangleList.Add(tri);
        }

        Unsafe.AsRef(in this.indices) = triangleList.ToArray();
        AssignNeighbors();
    }

    private void AssignNeighbors()
    {
        var edgeToTriangle = new Dictionary<Edge, int>(indices.Length * 3);

        for (int i = 0; i < indices.Length; i++)
        {
            var tri = indices[i];
            edgeToTriangle.TryAdd(new Edge(tri.IndexA, tri.IndexB), i);
            edgeToTriangle.TryAdd(new Edge(tri.IndexB, tri.IndexC), i);
            edgeToTriangle.TryAdd(new Edge(tri.IndexC, tri.IndexA), i);
        }

        for (int i = 0; i < indices.Length; i++)
        {
            ref var tri = ref indices[i];
            tri.NeighborA = edgeToTriangle.GetValueOrDefault(new Edge(tri.IndexC, tri.IndexB), -1);
            tri.NeighborB = edgeToTriangle.GetValueOrDefault(new Edge(tri.IndexA, tri.IndexC), -1);
            tri.NeighborC = edgeToTriangle.GetValueOrDefault(new Edge(tri.IndexB, tri.IndexA), -1);
        }
    }
}
