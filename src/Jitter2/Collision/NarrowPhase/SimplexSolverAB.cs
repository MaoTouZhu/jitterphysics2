/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Jitter2.LinearMath;

using Vertex = Jitter2.Collision.MinkowskiDifference.Vertex;

#pragma warning disable CS0649 // Field is never assigned to, and will always have its default value

namespace Jitter2.Collision;

/// <summary>
/// 实现 Gilbert-Johnson-Keerthi (GJK) 单纯形求解器，并支持获取原始形状（A 空间和 B 空间）上的最近点。
/// </summary>
/// <remarks>
/// 与 <see cref="SimplexSolver"/> 不同，此求解器会记录重心坐标和
/// 原始支撑点，从而可以通过 <see cref="GetClosest"/> 提取每个形状上的最近点。
/// </remarks>
[StructLayout(LayoutKind.Sequential)]
public unsafe struct SimplexSolverAB
{
    private const Real Epsilon = (Real)1e-8;

    private struct Barycentric
    {
        public Real Lambda0;
        public Real Lambda1;
        public Real Lambda2;
        public Real Lambda3;

        public Real this[int i]
        {
            get => ((Real*)Unsafe.AsPointer(ref this.Lambda0))[i];
            set => ((Real*)Unsafe.AsPointer(ref this.Lambda0))[i] = value;
        }
    }

    private Vertex v0;
    private Vertex v1;
    private Vertex v2;
    private Vertex v3;

    private Barycentric barycentric;
    private uint usageMask;

    /// <summary>
    /// 将求解器重置为空单纯形。
    /// </summary>
    public void Reset()
    {
        usageMask = 0;
    }

    private JVector ClosestSegment(int i0, int i1, ref Barycentric bc, out uint mask)
    {
        var ptr = (Vertex*)Unsafe.AsPointer(ref this.v0);
        JVector a = ptr[i0].V;
        JVector b = ptr[i1].V;

        JVector v = b - a;
        Real vsq = v.LengthSquared();

        if (vsq < Epsilon)
        {
            mask = 1u << i1;
            bc[i0] = (Real)0.0;
            bc[i1] = (Real)1.0;
            return b;
        }

        Real t = -JVector.Dot(a, v) / vsq;

        if (t > (Real)1.0)
        {
            mask = 1u << i1;
            bc[i0] = (Real)0.0;
            bc[i1] = (Real)1.0;
            return b;
        }

        if (t < (Real)0.0)
        {
            mask = 1u << i0;
            bc[i0] = (Real)1.0;
            bc[i1] = (Real)0.0;
            return a;
        }

        bc[i0] = (Real)1.0 - t;
        bc[i1] = t;
        mask = (1u << i0) | (1u << i1);

        // 为见证点保留重心坐标，但直接计算搜索向量。
        // return lambda0 * a + lambda1 * b;
        return JVector.Cross(v, JVector.Cross(a, v)) * ((Real)1.0 / vsq);
    }

    private JVector ClosestTriangle(int i0, int i1, int i2, ref Barycentric bc, out uint mask)
    {
        mask = 0;

        var ptr = (Vertex*)Unsafe.AsPointer(ref this.v0);
        JVector a = ptr[i0].V;
        JVector b = ptr[i1].V;
        JVector c = ptr[i2].V;

        JVector.Subtract(a, b, out var u);
        JVector.Subtract(a, c, out var v);

        JVector normal = u % v;

        Real t = normal.LengthSquared();
        Real it = (Real)1.0 / t;

        bool degenerate = t < Epsilon;

        JVector.Cross(u, a, out var c1);
        JVector.Cross(a, v, out var c2);

        Real lambda2 = JVector.Dot(c1, normal) * it;
        Real lambda1 = JVector.Dot(c2, normal) * it;
        Real lambda0 = (Real)1.0 - lambda2 - lambda1;

        Real bestDistance = Real.MaxValue;
        Unsafe.SkipInit(out JVector closestPt);
        Unsafe.SkipInit(out Barycentric b0);

        if (lambda0 < (Real)0.0 || degenerate)
        {
            var closest = ClosestSegment(i1, i2, ref b0, out uint m);
            Real dist = closest.LengthSquared();
            if (dist < bestDistance)
            {
                bc = b0;
                mask = m;
                bestDistance = dist;
                closestPt = closest;
            }
        }

        if (lambda1 < (Real)0.0 || degenerate)
        {
            var closest = ClosestSegment(i0, i2, ref b0, out uint m);
            Real dist = closest.LengthSquared();
            if (dist < bestDistance)
            {
                bc = b0;
                mask = m;
                bestDistance = dist;
                closestPt = closest;
            }
        }

        if (lambda2 < (Real)0.0 || degenerate)
        {
            var closest = ClosestSegment(i0, i1, ref b0, out uint m);
            Real dist = closest.LengthSquared();
            if (dist < bestDistance)
            {
                bc = b0;
                mask = m;
                closestPt = closest;
            }
        }

        if (mask != 0) return closestPt;

        bc[i0] = lambda0;
        bc[i1] = lambda1;
        bc[i2] = lambda2;

        mask = (1u << i0) | (1u << i1) | (1u << i2);

        // 为见证点保留重心坐标，但直接计算搜索向量。
        // return lambda0 * a + lambda1 * b + lambda2 * c;
        return normal * (JVector.Dot(normal, a) * it);
    }

    private JVector ClosestTetrahedron(ref Barycentric bc, out uint mask)
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        static Real Determinant(in JVector a, in JVector b, in JVector c, in JVector d)
        {
            return JVector.Dot(b - a, JVector.Cross(c - a, d - a));
        }

        Real detT = Determinant(v0.V, v1.V, v2.V, v3.V);
        Real inverseDetT = (Real)1.0 / detT;

        bool degenerate = detT * detT < Epsilon;

        Real lambda0 = Determinant(JVector.Zero, v1.V, v2.V, v3.V) * inverseDetT;
        Real lambda1 = Determinant(v0.V, JVector.Zero, v2.V, v3.V) * inverseDetT;
        Real lambda2 = Determinant(v0.V, v1.V, JVector.Zero, v3.V) * inverseDetT;
        Real lambda3 = (Real)1.0 - lambda0 - lambda1 - lambda2;

        Real bestDistance = Real.MaxValue;

        Unsafe.SkipInit(out JVector closestPt);
        Unsafe.SkipInit(out Barycentric b0);

        mask = 0;

        if (lambda0 < (Real)0.0 || degenerate)
        {
            var closest = ClosestTriangle(1, 2, 3, ref b0, out uint m);
            Real dist = closest.LengthSquared();
            if (dist < bestDistance)
            {
                bc = b0;
                mask = m;
                bestDistance = dist;
                closestPt = closest;
            }
        }

        if (lambda1 < (Real)0.0 || degenerate)
        {
            var closest = ClosestTriangle(0, 2, 3, ref b0, out uint m);
            Real dist = closest.LengthSquared();
            if (dist < bestDistance)
            {
                bc = b0;
                mask = m;
                bestDistance = dist;
                closestPt = closest;
            }
        }

        if (lambda2 < (Real)0.0 || degenerate)
        {
            var closest = ClosestTriangle(0, 1, 3, ref b0, out uint m);
            Real dist = closest.LengthSquared();
            if (dist < bestDistance)
            {
                bc = b0;
                mask = m;
                bestDistance = dist;
                closestPt = closest;
            }
        }

        if (lambda3 < (Real)0.0 || degenerate)
        {
            var closest = ClosestTriangle(0, 1, 2, ref b0, out uint m);
            Real dist = closest.LengthSquared();
            if (dist < bestDistance)
            {
                bc = b0;
                mask = m;
                // bestDistance = dist;
                closestPt = closest;
            }
        }

        if (mask != 0) return closestPt;

        barycentric[0] = lambda0;
        barycentric[1] = lambda1;
        barycentric[2] = lambda2;
        barycentric[3] = lambda3;

        mask = 0b1111;
        return JVector.Zero;
    }

    /// <summary>
    /// 使用当前单纯形和重心坐标计算原始形状 A 和 B 上的最近点。
    /// </summary>
    /// <param name="pointA">形状 A 上的最近点。</param>
    /// <param name="pointB">形状 B 上的最近点。</param>
    public void GetClosest(out JVector pointA, out JVector pointB)
    {
        pointA = JVector.Zero;
        pointB = JVector.Zero;

        var ptr = (Vertex*)Unsafe.AsPointer(ref this.v0);

        for (int i = 0; i < 4; i++)
        {
            if ((usageMask & (1u << i)) == 0) continue;
            pointA += barycentric[i] * ptr[i].A;
            pointB += barycentric[i] * ptr[i].B;
        }
    }

    /// <summary>
    /// 向单纯形中添加一个顶点（包含完整的 A/B 支撑点数据），并计算
    /// 新的距原点最近点。
    /// </summary>
    /// <param name="vertex">闵可夫斯基差顶点，包含两个形状各自的支撑点。</param>
    /// <param name="closest">
    /// 此方法返回时，包含缩减后的单纯形上距原点最近的点。
    /// </param>
    /// <returns>
    /// 如果原点不包含在单纯形内，则为 <c>true</c>；
    /// 如果原点被四面体包围，则为 <c>false</c>。
    /// </returns>
    [SkipLocalsInit]
    public bool AddVertex(in Vertex vertex, out JVector closest)
    {
        Unsafe.SkipInit(out closest);

        var ptr = (Vertex*)Unsafe.AsPointer(ref this.v0);
        int* ix = stackalloc int[4];

        int useCount = 0;
        int freeSlot = 0;

        for (int i = 0; i < 4; i++)
        {
            if ((usageMask & (1u << i)) != 0) ix[useCount++] = i;
            else freeSlot = i;
        }

        ix[useCount++] = freeSlot;
        ptr[freeSlot] = vertex;

        switch (useCount)
        {
            case 1:
                {
                    int i0 = ix[0];
                    closest = ptr[i0].V;
                    usageMask = 1u << i0;
                    barycentric[i0] = (Real)1.0;
                    return true;
                }
            case 2:
                {
                    int i0 = ix[0], i1 = ix[1];
                    closest = ClosestSegment(i0, i1, ref barycentric, out usageMask);
                    return true;
                }
            case 3:
                {
                    int i0 = ix[0], i1 = ix[1], i2 = ix[2];
                    closest = ClosestTriangle(i0, i1, i2, ref barycentric, out usageMask);
                    return true;
                }
            case 4:
                {
                    closest = ClosestTetrahedron(ref barycentric, out usageMask);
                    return usageMask != 0b1111;
                }
        }

        Debug.Assert(false, "Unreachable.");
        return false;
    }
}
