/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 表示一种具有 SIMD 友好内存布局的轴对齐包围盒，用于 <see cref="DynamicTree"/> 等加速结构中的空间划分。
/// </summary>
/// <remarks>
/// 该结构体经过显式布局，恰好占用 8 个 <see cref="Real"/> 字段，从而实现高效的 SIMD 运算和
/// 二进制比较。它同时包含盒子作为 <see cref="JVector"/> 的最小角与最大角，以及
/// 未使用的 W 分量（<c>MinW</c>、<c>MaxW</c>）以匹配内存对齐。
/// </remarks>
/// <seealso cref="JBoundingBox"/>
/// <seealso cref="JVector"/>
[StructLayout(LayoutKind.Explicit, Size = 8 * sizeof(Real))]
public struct TreeBox : IEquatable<TreeBox>
{
    /// <summary>
    /// 用于射线与盒相交测试的小 epsilon 值。
    /// </summary>
    public const Real Epsilon = (Real)1e-12;

    /// <summary>包围盒的最小角。</summary>
    [FieldOffset(0 * sizeof(Real))] public JVector Min;

    /// <summary>用于 SIMD 对齐的填充。不直接使用。</summary>
    [FieldOffset(3 * sizeof(Real))] public Real MinW;

    /// <summary>包围盒的最大角。</summary>
    [FieldOffset(4 * sizeof(Real))] public JVector Max;

    /// <summary>用于 SIMD 对齐的填充。不直接使用。</summary>
    [FieldOffset(7 * sizeof(Real))] public Real MaxW;

    /// <summary>
    /// 返回 <see cref="Min"/> 向量的 <see cref="VectorReal"/> 视图，用于 SIMD 运算。
    /// </summary>
    public readonly ref VectorReal VectorMin => ref Unsafe.As<JVector, VectorReal>(ref Unsafe.AsRef(in this.Min));

    /// <summary>
    /// 返回 <see cref="Max"/> 向量的 <see cref="VectorReal"/> 视图，用于 SIMD 运算。
    /// </summary>
    public readonly ref VectorReal VectorMax => ref Unsafe.As<JVector, VectorReal>(ref Unsafe.AsRef(in this.Max));

    /// <summary>
    /// 根据最小角与最大角向量创建新的 <see cref="TreeBox"/>。
    /// </summary>
    public TreeBox(in JVector min, in JVector max)
    {
        this.Min = min;
        this.Max = max;
        this.MinW = 0;
        this.MaxW = 0;
    }

    /// <summary>
    /// 根据现有的 <see cref="JBoundingBox"/> 创建新的 <see cref="TreeBox"/>。
    /// </summary>
    public TreeBox(in JBoundingBox box)
    {
        this.Min = box.Min;
        this.Max = box.Max;
        this.MinW = 0;
        this.MaxW = 0;
    }

    /// <summary>
    /// 将该 <see cref="TreeBox"/> 转换为 <see cref="JBoundingBox"/>。
    /// </summary>
    public readonly JBoundingBox AsJBoundingBox() => new(Min, Max);

    // ─── 与 JBox 中完全一致的辅助函数 ───────────────────────────

    /// <summary>
    /// 判断该盒是否包含指定点。
    /// </summary>
    /// <param name="point">要检查的点。</param>
    /// <returns>若点位于该盒边界内则为 <c>true</c>；否则为 <c>false</c>。</returns>
    public readonly bool Contains(in JVector point)
    {
        return Min.X <= point.X && point.X <= Max.X &&
               Min.Y <= point.Y && point.Y <= Max.Y &&
               Min.Z <= point.Z && point.Z <= Max.Z;
    }

    /// <summary>
    /// 获取包围盒的中心点。
    /// </summary>
    public readonly JVector Center => (Min + Max) * ((Real)(1.0 / 2.0));

    /// <summary>
    /// 判断该盒是否与指定盒相交或重叠。
    /// </summary>
    /// <param name="box">用于比较的盒。</param>
    /// <returns>若两盒重叠则为 <c>true</c>；若完全分离则为 <c>false</c>。</returns>
    [Obsolete($"Use !{nameof(Disjoint)} instead.", true)]
    public readonly bool NotDisjoint(in JBoundingBox box)
    {
        return Max.X >= box.Min.X && Min.X <= box.Max.X && Max.Y >= box.Min.Y && Min.Y <= box.Max.Y &&
               Max.Z >= box.Min.Z && Min.Z <= box.Max.Z;
    }

    /// <summary>
    /// 判断该盒是否与指定盒完全分离（不重叠）。
    /// </summary>
    /// <param name="box">用于比较的盒。</param>
    /// <returns>若两盒在至少一个轴上存在间隙而分离则为 <c>true</c>；否则为 <c>false</c>。</returns>
    public readonly bool Disjoint(in JBoundingBox box)
    {
        return Max.X < box.Min.X || Min.X > box.Max.X || Max.Y < box.Min.Y || Min.Y > box.Max.Y ||
               Max.Z < box.Min.Z || Min.Z > box.Max.Z;
    }

    /// <summary>
    /// 判断该盒是否完全包围指定盒。
    /// </summary>
    /// <param name="box">要检查是否被包含的盒。</param>
    /// <returns>若 <paramref name="box"/> 完全位于该盒内部则为 <c>true</c>；否则为 <c>false</c>。</returns>
    public readonly bool Contains(in JBoundingBox box)
    {
        return Min.X <= box.Min.X && Max.X >= box.Max.X && Min.Y <= box.Min.Y && Max.Y >= box.Max.Y &&
               Min.Z <= box.Min.Z && Max.Z >= box.Max.Z;
    }

    /// <summary>
    /// 判断该盒是否完全包围指定盒。
    /// </summary>
    /// <remarks>这是 <see cref="Contains(in JBoundingBox)"/> 的别名。</remarks>
    [Obsolete($"Use {nameof(Contains)} instead.", true)]
    public readonly bool Encompasses(in JBoundingBox box)
    {
        return Min.X <= box.Min.X && Max.X >= box.Max.X && Min.Y <= box.Min.Y && Max.Y >= box.Max.Y &&
               Min.Z <= box.Min.Z && Max.Z >= box.Max.Z;
    }

    private static bool Intersect1D(Real start, Real dir, Real min, Real max,
        ref Real enter, ref Real exit)
    {
        if (dir * dir < Epsilon * Epsilon) return start >= min && start <= max;

        Real t0 = (min - start) / dir;
        Real t1 = (max - start) / dir;

        if (t0 > t1)
        {
            (t0, t1) = (t1, t0);
        }

        if (t0 > exit || t1 < enter) return false;

        if (t0 > enter) enter = t0;
        if (t1 < exit) exit = t1;
        return true;
    }

    /// <summary>
    /// 检查有限线段是否与该包围盒相交。
    /// </summary>
    /// <param name="origin">线段的起点。</param>
    /// <param name="direction">定义线段方向与长度的向量（终点 = 起点 + 方向）。</param>
    /// <returns>若线段穿过该盒则为 <c>true</c>；否则为 <c>false</c>。</returns>
    public readonly bool SegmentIntersect(in JVector origin, in JVector direction)
    {
        Real enter = (Real)0.0, exit = (Real)1.0;

        if (!Intersect1D(origin.X, direction.X, Min.X, Max.X, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, ref enter, ref exit))
            return false;

        return true;
    }

    /// <summary>
    /// 检查无限射线是否与该包围盒相交。
    /// </summary>
    /// <param name="origin">射线的原点。</param>
    /// <param name="direction">射线的方向（不要求归一化）。</param>
    /// <returns>若射线与该盒相交则为 <c>true</c>；否则为 <c>false</c>。</returns>
    public readonly bool RayIntersect(in JVector origin, in JVector direction)
    {
        Real enter = (Real)0.0, exit = Real.MaxValue;

        if (!Intersect1D(origin.X, direction.X, Min.X, Max.X, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, ref enter, ref exit))
            return false;

        return true;
    }

    /// <summary>
    /// 检查无限射线是否与该包围盒相交，并计算进入距离。
    /// </summary>
    /// <param name="origin">射线的原点。</param>
    /// <param name="direction">射线的方向（不要求归一化）。</param>
    /// <param name="enter">输出沿方向向量的标量距离，即射线进入该盒的位置。若已在盒内，则为 0。</param>
    /// <returns>若射线与该盒相交则为 <c>true</c>；否则为 <c>false</c>。</returns>
    public readonly bool RayIntersect(in JVector origin, in JVector direction, out Real enter)
    {
        enter = (Real)0.0;
        Real exit = Real.MaxValue;

        if (!Intersect1D(origin.X, direction.X, Min.X, Max.X, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Y, direction.Y, Min.Y, Max.Y, ref enter, ref exit))
            return false;

        if (!Intersect1D(origin.Z, direction.Z, Min.Z, Max.Z, ref enter, ref exit))
            return false;

        return true;
    }

    public readonly override string ToString()
    {
        return $"Min={{{Min}}}, Max={{{Max}}}";
    }

    // ─── 支持 SIMD 的辅助函数 ───────────────────────────

    /// <summary>
    /// 判断该盒是否与指定的 <see cref="TreeBox"/> 完全分离。
    /// </summary>
    /// <param name="box">用于比较的盒。</param>
    /// <returns>若两盒不重叠则为 <c>true</c>；若接触或重叠则为 <c>false</c>。</returns>
    public readonly bool Disjoint(in TreeBox box) => TreeBox.Disjoint(this, box);

    /// <summary>
    /// 判断该盒是否完全包围指定的 <see cref="TreeBox"/>。
    /// </summary>
    /// <param name="box">要检查是否被包含的盒。</param>
    /// <returns>若 <paramref name="box"/> 完全位于该盒内部则为 <c>true</c>。</returns>
    public readonly bool Contains(in TreeBox box) => TreeBox.Contains(this, box);

    /// <summary>
    /// 计算包围盒的表面积。
    /// </summary>
    /// <returns>总表面积：2*(ex*ey + ey*ez + ez*ex)。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly double GetSurfaceArea()
    {
        var extent = Vector.Subtract(VectorMax, VectorMin);

        double ex = extent.GetElement(0);
        double ey = extent.GetElement(1);
        double ez = extent.GetElement(2);

        return 2.0 * (ex * ey + ey * ez + ez * ex);
    }

    /// <summary>
    /// 计算将两个指定盒合并后所得假想盒的表面积。
    /// </summary>
    /// <param name="first">第一个盒。</param>
    /// <param name="second">第二个盒。</param>
    /// <returns>并集盒的表面积。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static double MergedSurface(in TreeBox first, in TreeBox second)
    {
        var vMin = Vector.Min(first.VectorMin, second.VectorMin);
        var vMax = Vector.Max(first.VectorMax, second.VectorMax);
        var extent = Vector.Subtract(vMax, vMin);

        double ex = extent.GetElement(0);
        double ey = extent.GetElement(1);
        double ez = extent.GetElement(2);

        return 2.0d * (ex * ey + ex * ez + ey * ez);
    }

    /// <summary>
    /// 使用 SIMD 指令判断外部盒是否完全包含内部盒。
    /// </summary>
    /// <param name="outer">可能的容器盒。</param>
    /// <param name="inner">要检查是否被包含的盒。</param>
    /// <returns>若 <paramref name="inner"/> 完全位于 <paramref name="outer"/> 内则为 <c>true</c>。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Contains(in TreeBox outer, in TreeBox inner)
    {
        var leMin = Vector.LessThanOrEqual(outer.VectorMin, inner.VectorMin);
        var geMax = Vector.GreaterThanOrEqual(outer.VectorMax, inner.VectorMax);

        var mask = Vector.BitwiseAnd(leMin, geMax);
        return Vector.EqualsAll(mask.AsInt32(), Vector.Create(-1));
    }

    /// <summary>
    /// 使用 SIMD 指令判断两个盒是否重叠。
    /// </summary>
    /// <param name="first">第一个盒。</param>
    /// <param name="second">第二个盒。</param>
    /// <returns>若两盒相交则为 <c>true</c>；若相互分离则为 <c>false</c>。</returns>
    [Obsolete($"Use !{nameof(Disjoint)} instead.", true)]
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool NotDisjoint(in TreeBox first, in TreeBox second)
    {
        var geMin = Vector.GreaterThanOrEqual(first.VectorMax, second.VectorMin);
        var leMax = Vector.LessThanOrEqual(first.VectorMin, second.VectorMax);

        var mask = Vector.BitwiseAnd(geMin, leMax);
        return Vector.EqualsAll(mask.AsInt32(), Vector.Create(-1));
    }

    /// <summary>
    /// 使用 SIMD 指令判断两个盒是否完全分离（不相交）。
    /// </summary>
    /// <param name="first">第一个盒。</param>
    /// <param name="second">第二个盒。</param>
    /// <returns>若两盒在任何轴上都不重叠则为 <c>true</c>；若接触或相交则为 <c>false</c>。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Disjoint(in TreeBox first, in TreeBox second)
    {
        // 若在任一轴上 first.Max < second.Min 或 first.Min > second.Max，
        // 则两个盒不可能重叠。
        var ltMin = Vector.LessThan(first.VectorMax, second.VectorMin);
        var gtMax = Vector.GreaterThan(first.VectorMin, second.VectorMax);

        var mask = Vector.BitwiseOr(ltMin, gtMax);
        return !Vector.EqualsAll(mask.AsInt32(), Vector.Create(0));
    }

    /// <summary>
    /// 创建一个同时包含两个输入盒的新盒。
    /// </summary>
    /// <param name="first">第一个盒。</param>
    /// <param name="second">第二个盒。</param>
    /// <param name="result">生成的并集盒。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void CreateMerged(in TreeBox first, in TreeBox second, out TreeBox result)
    {
        Unsafe.SkipInit(out result);
        result.VectorMin = Vector.Min(first.VectorMin, second.VectorMin);
        result.VectorMax = Vector.Max(first.VectorMax, second.VectorMax);
    }

    /// <summary>
    /// 使用二进制比较判断两个盒是否相等。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Equals(in TreeBox first, in TreeBox second)
    {
        var a = MemoryMarshal.AsBytes(MemoryMarshal.CreateReadOnlySpan(ref Unsafe.AsRef(in first), 1));
        var b = MemoryMarshal.AsBytes(MemoryMarshal.CreateReadOnlySpan(ref Unsafe.AsRef(in second), 1));
        return a.SequenceEqual(b); // 在 .NET ≥ 5 中由 SIMD 加速
    }

    /// <inheritdoc />
    public readonly bool Equals(TreeBox other)
    {
        return Equals(this, other);
    }

    public readonly override bool Equals(object? obj)
    {
        return obj is TreeBox other && Equals(other);
    }

    public readonly override int GetHashCode()
    {
        return HashCode.Combine(Min, MinW, Max, MaxW);
    }

    public static bool operator ==(TreeBox left, TreeBox right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(TreeBox left, TreeBox right)
    {
        return !(left == right);
    }
}
