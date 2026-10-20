/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Jitter2.LinearMath;

/// <summary>
/// 表示由三个顶点定义的三角形。
/// </summary>
[StructLayout(LayoutKind.Explicit, Size = 9 * sizeof(Real))]
public struct JTriangle(in JVector v0, in JVector v1, in JVector v2) : IEquatable<JTriangle>
{
    /// <summary>
    /// 指定基于绕序的三角形面剔除模式。
    /// 逆时针（CCW）绕序被视为正面朝向。
    /// </summary>
    public enum CullMode
    {
        /// <summary>
        /// 剔除正面朝向的三角形。
        /// 若三角形的顶点按逆时针（CCW）顺序排列，则该三角形为正面朝向。
        /// </summary>
        FrontFacing,

        /// <summary>
        /// 剔除背面朝向的三角形。
        /// 若三角形的顶点按顺时针（CW）顺序排列，则该三角形为背面朝向。
        /// 这是最常见的剔除模式。
        /// </summary>
        BackFacing,

        /// <summary>
        /// 不执行面剔除；正面和背面朝向的三角形都会被处理。
        /// </summary>
        None
    }

    /// <summary>三角形的第一个顶点。</summary>
    [FieldOffset(0 * sizeof(Real))] public JVector V0 = v0;

    /// <summary>三角形的第二个顶点。</summary>
    [FieldOffset(3 * sizeof(Real))] public JVector V1 = v1;

    /// <summary>三角形的第三个顶点。</summary>
    [FieldOffset(6 * sizeof(Real))] public JVector V2 = v2;

    /// <summary>
    /// 检查穿过射线原点的直线是否与三角形的严格内部相交。
    /// 此方法可能返回位于原点之后的交点。
    /// </summary>
    /// <param name="origin">射线的起点（原点）。</param>
    /// <param name="direction">射线的方向向量。</param>
    /// <param name="cullMode">决定是否根据绕序（正面/背面朝向）忽略三角形。</param>
    /// <param name="normal">输出：交点处的归一化表面法线。</param>
    /// <param name="lambda">输出：交点处的有符号射线参数
    /// （命中点 = origin + lambda * direction）。若 direction 未归一化，则它不是距离。</param>
    /// <returns>若直线与三角形内部相交并通过面剔除测试，则为 <c>true</c>；
    /// 否则为 <c>false</c>。恰好落在边上的命中会被排除。</returns>
    public readonly bool RayIntersect(in JVector origin, in JVector direction, CullMode cullMode,
        out JVector normal, out Real lambda)
    {
        JVector u = V0 - V1;
        JVector v = V0 - V2;

        normal = u % v;
        Real it = (Real)1.0 / normal.LengthSquared();

        Real denominator = JVector.Dot(direction, normal);

        if (Math.Abs(denominator) < (Real)1e-06)
        {
            // 三角形与射线平行。
            goto return_false;
        }

        lambda = JVector.Dot(V0 - origin, normal);

        switch (cullMode)
        {
            case CullMode.FrontFacing when lambda < (Real)0.0:
            case CullMode.BackFacing when lambda > (Real)0.0:
                goto return_false;
        }

        lambda /= denominator;

        // 射线与三角形所在平面的交点。
        JVector hitPoint = origin + lambda * direction;

        // 检查该点是否位于三角形内部
        JVector at = V0 - hitPoint;
        JVector.Cross(u, at, out JVector tmp);
        Real gamma = JVector.Dot(tmp, normal) * it;
        JVector.Cross(at, v, out tmp);
        Real beta = JVector.Dot(tmp, normal) * it;
        Real alpha = (Real)1.0 - gamma - beta;

        if (alpha > 0 && beta > 0 && gamma > 0)
        {
            normal *= -MathHelper.SignBit(denominator) * MathR.Sqrt(it);
            return true;
        }

    return_false:

        lambda = Real.MaxValue; normal = JVector.Zero;
        return false;
    }

    /// <summary>
    /// 返回 <see cref="JTriangle"/> 的字符串表示形式。
    /// </summary>
    public readonly override string ToString()
    {
        return $"V0={{{V0}}}, V1={{{V1}}}, V2={{{V2}}}";
    }

    /// <summary>
    /// 计算三角形的面法线。
    /// 方向遵循右手定则（逆时针绕序）。
    /// </summary>
    /// <returns>未归一化的法线向量。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly JVector GetNormal()
    {
        return (V1 - V0) % (V2 - V0);
    }

    /// <summary>
    /// 计算三角形的几何中心（质心）。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly JVector GetCenter()
    {
        return (V0 + V1 + V2) * (Real)(1.0 / 3.0);
    }

    /// <summary>
    /// 计算三角形的面积。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly Real GetArea()
    {
        return GetNormal().Length() * (Real)0.5;
    }

    /// <summary>
    /// 计算此三角形的轴对齐包围盒（AABB）。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly JBoundingBox GetBoundingBox()
    {
        JBoundingBox box = JBoundingBox.SmallBox;
        JBoundingBox.AddPointInPlace(ref box, V0);
        JBoundingBox.AddPointInPlace(ref box, V1);
        JBoundingBox.AddPointInPlace(ref box, V2);
        return box;
    }

    /// <summary>
    /// 查找三角形表面上距离指定点最近的点。
    /// </summary>
    /// <param name="point">查询点。</param>
    /// <returns>三角形上距离查询点最近的点。</returns>
    public readonly JVector ClosestPoint(JVector point)
    {
        JVector ab = V1 - V0;
        JVector ac = V2 - V0;
        JVector ap = point - V0;

        Real d1 = JVector.Dot(ab, ap);
        Real d2 = JVector.Dot(ac, ap);

        // 顶点区域 V0
        if (d1 <= 0 && d2 <= 0) return V0;

        JVector bp = point - V1;
        Real d3 = JVector.Dot(ab, bp);
        Real d4 = JVector.Dot(ac, bp);

        // 顶点区域 V1
        if (d3 >= 0 && d4 <= d3) return V1;

        // 边区域 V0-V1
        Real vc = d1 * d4 - d3 * d2;
        if (vc <= 0 && d1 >= 0 && d3 <= 0)
        {
            Real v = d1 / (d1 - d3);
            return V0 + v * ab;
        }

        JVector cp = point - V2;
        Real d5 = JVector.Dot(ab, cp);
        Real d6 = JVector.Dot(ac, cp);

        // 顶点区域 V2
        if (d6 >= 0 && d5 <= d6) return V2;

        // 边区域 V0-V2
        Real vb = d5 * d2 - d1 * d6;
        if (vb <= 0 && d2 >= 0 && d6 <= 0)
        {
            Real w = d2 / (d2 - d6);
            return V0 + w * ac;
        }

        // 边区域 V1-V2
        Real va = d3 * d6 - d5 * d4;
        if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0)
        {
            Real w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return V1 + w * (V2 - V1);
        }

        // 面区域
        Real denom = (Real)1.0 / (va + vb + vc);
        Real vbn = vb * denom;
        Real wn = vc * denom;

        return V0 + ab * vbn + ac * wn;
    }

    public readonly override int GetHashCode() => HashCode.Combine(V0, V1, V2);

    public readonly bool Equals(JTriangle other)
    {
        return V0.Equals(other.V0) && V1.Equals(other.V1) && V2.Equals(other.V2);
    }

    public readonly override bool Equals(object? obj)
    {
        return obj is JTriangle other && Equals(other);
    }

    public static bool operator ==(JTriangle left, JTriangle right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(JTriangle left, JTriangle right)
    {
        return !(left == right);
    }
}
