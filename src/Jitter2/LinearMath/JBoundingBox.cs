/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Jitter2.LinearMath;

/// <summary>
/// 表示轴对齐包围盒（AABB），即边与坐标轴平行的矩形包围盒。
/// </summary>
[StructLayout(LayoutKind.Explicit, Size = 6 * sizeof(Real))]
public struct JBoundingBox(JVector min, JVector max) : IEquatable<JBoundingBox>
{
    public const Real Epsilon = (Real)1e-12;

    /// <summary>
    /// 描述一个包围盒与另一个包围盒在空间上的关系。
    /// </summary>
    public enum ContainmentType
    {
        /// <summary>
        /// 两个盒子完全分离，既不接触也不重叠。
        /// </summary>
        Disjoint,

        /// <summary>
        /// 另一个盒子完全位于此盒子内部。
        /// </summary>
        Contains,

        /// <summary>
        /// 两个盒子重叠，但彼此都不完全包含对方。
        /// </summary>
        Intersects
    }

    /// <summary>
    /// 包围盒的最小角点（最小的 X、Y、Z 坐标）。
    /// </summary>
    [FieldOffset(0 * sizeof(Real))]
    public JVector Min = min;

    /// <summary>
    /// 包围盒的最大角点（最大的 X、Y、Z 坐标）。
    /// </summary>
    [FieldOffset(3 * sizeof(Real))]
    public JVector Max = max;

    /// <summary>
    /// 覆盖整个有效坐标范围的包围盒。
    /// </summary>
    public static readonly JBoundingBox LargeBox;

    /// <summary>
    /// 一个以 Min > Max 初始化的反向包围盒，适用于从零开始扩展一个盒子。
    /// </summary>
    public static readonly JBoundingBox SmallBox;

    static JBoundingBox()
    {
        LargeBox.Min = new JVector(Real.MinValue);
        LargeBox.Max = new JVector(Real.MaxValue);
        SmallBox.Min = new JVector(Real.MaxValue);
        SmallBox.Max = new JVector(Real.MinValue);
    }

    /// <summary>
    /// 返回 <see cref="JBoundingBox"/> 的字符串表示。
    /// </summary>
    public readonly override string ToString()
    {
        return $"Min={{{Min}}}, Max={{{Max}}}";
    }

    // 建议改用 CreateTransformed，以获得更清晰的语义并避免修改原对象。
    [Obsolete($"Use static {nameof(CreateTransformed)} instead.", true)]
    public void Transform(in JMatrix orientation)
    {
        JVector halfExtents = (Real)0.5 * (Max - Min);
        JVector center = (Real)0.5 * (Max + Min);

        JVector.Transform(center, orientation, out center);

        JMatrix.Absolute(orientation, out var abs);
        JVector.Transform(halfExtents, abs, out halfExtents);

        Max = center + halfExtents;
        Min = center - halfExtents;
    }

    /// <summary>
    /// 创建一个新的 AABB，用于包住原包围盒经过给定方向矩阵旋转后的结果。
    /// </summary>
    /// <remarks>
    /// 旋转 AABB 通常会产生一个更大的 AABB，以容纳旋转后的几何体。
    /// </remarks>
    /// <param name="box">原始包围盒。</param>
    /// <param name="orientation">要应用的旋转矩阵。</param>
    /// <returns>包住旋转后盒子的新 AABB。</returns>
    public static JBoundingBox CreateTransformed(in JBoundingBox box, in JMatrix orientation)
    {
        JVector halfExtents = (Real)0.5 * (box.Max - box.Min);
        JVector center = (Real)0.5 * (box.Max + box.Min);

        JVector.Transform(center, orientation, out center);

        JMatrix.Absolute(orientation, out var abs);
        JVector.Transform(halfExtents, abs, out halfExtents);

        JBoundingBox result;
        result.Max = center + halfExtents;
        result.Min = center - halfExtents;

        return result;
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
    /// 检查一条有限线段是否与此包围盒相交。
    /// </summary>
    /// <param name="origin">线段的起点。</param>
    /// <param name="direction">从起点指向终点的向量（End = Origin + Direction）。</param>
    /// <returns>如果线段穿过该盒子则返回 <c>true</c>；否则返回 <c>false</c>。</returns>
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
    /// 检查一条无限射线是否与此包围盒相交。
    /// </summary>
    /// <param name="origin">射线的起点。</param>
    /// <param name="direction">射线的方向（不要求归一化）。</param>
    /// <returns>如果射线与该盒子相交则返回 <c>true</c>；否则返回 <c>false</c>。</returns>
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
    /// 检查射线是否与此包围盒相交，并计算入射参数。
    /// </summary>
    /// <param name="origin">射线的起点。</param>
    /// <param name="direction">射线的方向（不要求归一化）。</param>
    /// <param name="enter">输出 <c>origin + enter * direction</c> 中的参数，即射线进入
    /// 盒子的位置。若起点在盒子内部则返回 0；仅当方向为单位向量时该值才是距离。</param>
    /// <returns>如果射线与该盒子相交则返回 <c>true</c>；否则返回 <c>false</c>。</returns>
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

    /// <summary>
    /// 判断包围盒是否包含指定点。
    /// </summary>
    public readonly bool Contains(in JVector point)
    {
        return Min.X <= point.X && point.X <= Max.X &&
               Min.Y <= point.Y && point.Y <= Max.Y &&
               Min.Z <= point.Z && point.Z <= Max.Z;
    }

    /// <summary>
    /// 获取包围盒的 8 个角点。
    /// </summary>
    /// <param name="destination">至少能容纳 8 个 JVector 的 span，用于存放角点。</param>
    public readonly void GetCorners(Span<JVector> destination)
    {
        destination[0] = new(Min.X, Max.Y, Max.Z);
        destination[1] = new(Max.X, Max.Y, Max.Z);
        destination[2] = new(Max.X, Min.Y, Max.Z);
        destination[3] = new(Min.X, Min.Y, Max.Z);
        destination[4] = new(Min.X, Max.Y, Min.Z);
        destination[5] = new(Max.X, Max.Y, Min.Z);
        destination[6] = new(Max.X, Min.Y, Min.Z);
        destination[7] = new(Min.X, Min.Y, Min.Z);
    }

    // 标记为过时，以引导用户使用更推荐的静态方法。
    [Obsolete($"Use static {nameof(AddPointInPlace)} instead.", true)]
    public void AddPoint(in JVector point)
    {
        JVector.Max(Max, point, out Max);
        JVector.Min(Min, point, out Min);
    }

    /// <summary>
    /// 扩展包围盒以包含指定点。
    /// </summary>
    /// <param name="box">要扩展的包围盒。</param>
    /// <param name="point">要包含的点。</param>
    public static void AddPointInPlace(ref JBoundingBox box, in JVector point)
    {
        JVector.Max(box.Max, point, out box.Max);
        JVector.Min(box.Min, point, out box.Min);
    }

    /// <summary>
    /// 创建一个恰好包住一组点的包围盒。
    /// </summary>
    /// <param name="points">要包含的点集合。</param>
    /// <returns>包含所有点的包围盒。</returns>
    public static JBoundingBox CreateFromPoints(IEnumerable<JVector> points)
    {
        JBoundingBox box = SmallBox;

        foreach (var point in points)
        {
            AddPointInPlace(ref box, point);
        }

        return box;
    }

    /// <summary>
    /// 确定此盒子与另一个盒子之间的关系。
    /// </summary>
    /// <param name="box">要测试的另一个包围盒。</param>
    /// <returns>
    /// 若两者不接触则返回 <see cref="ContainmentType.Disjoint"/>。<br/>
    /// 若 <paramref name="box"/> 被此盒子完全包含则返回 <see cref="ContainmentType.Contains"/>。<br/>
    /// 若两者重叠但此盒子未完全包含对方则返回 <see cref="ContainmentType.Intersects"/>。
    /// </returns>
    public readonly ContainmentType Contains(in JBoundingBox box)
    {
        ContainmentType result = ContainmentType.Disjoint;
        if (Max.X >= box.Min.X && Min.X <= box.Max.X && Max.Y >= box.Min.Y && Min.Y <= box.Max.Y &&
            Max.Z >= box.Min.Z && Min.Z <= box.Max.Z)
        {
            result = Min.X <= box.Min.X && box.Max.X <= Max.X && Min.Y <= box.Min.Y && box.Max.Y <= Max.Y &&
                     Min.Z <= box.Min.Z && box.Max.Z <= Max.Z
                ? ContainmentType.Contains
                : ContainmentType.Intersects;
        }

        return result;
    }

    /// <summary>
    /// 判断两个盒子是否相交或重叠。
    /// </summary>
    /// <returns>如果盒子重叠则返回 <c>true</c>；如果互不相交则返回 <c>false</c>。</returns>
    [Obsolete($"Use !{nameof(Disjoint)} instead.", true)]
    public static bool NotDisjoint(in JBoundingBox left, in JBoundingBox right)
    {
        return left.Max.X >= right.Min.X && left.Min.X <= right.Max.X && left.Max.Y >= right.Min.Y && left.Min.Y <= right.Max.Y &&
               left.Max.Z >= right.Min.Z && left.Min.Z <= right.Max.Z;
    }

    /// <summary>
    /// 判断两个盒子是否完全分离（互不相交）。
    /// </summary>
    /// <param name="left">第一个包围盒。</param>
    /// <param name="right">第二个包围盒。</param>
    /// <returns>如果两个盒子在至少一个轴上存在间隙则返回 <see langword="true"/>；否则返回 <see langword="false"/>。</returns>
    public static bool Disjoint(in JBoundingBox left, in JBoundingBox right)
    {
        return left.Max.X < right.Min.X || left.Min.X > right.Max.X || left.Max.Y < right.Min.Y || left.Min.Y > right.Max.Y ||
               left.Max.Z < right.Min.Z || left.Min.Z > right.Max.Z;
    }

    /// <summary>
    /// 判断 <paramref name="outer"/> 盒子是否完全包含 <paramref name="inner"/> 盒子。
    /// </summary>
    /// <param name="outer">外部包围盒。</param>
    /// <param name="inner">要测试的内部包围盒。</param>
    /// <returns>如果 <paramref name="inner"/> 完全位于 <paramref name="outer"/> 的边界之内则返回 <see langword="true"/>；否则返回 <see langword="false"/>。</returns>
    public static bool Contains(in JBoundingBox outer, in JBoundingBox inner)
    {
        return outer.Min.X <= inner.Min.X && outer.Max.X >= inner.Max.X && outer.Min.Y <= inner.Min.Y && outer.Max.Y >= inner.Max.Y &&
               outer.Min.Z <= inner.Min.Z && outer.Max.Z >= inner.Max.Z;
    }

    /// <summary>
    /// 判断 <paramref name="outer"/> 盒子是否完全包含 <paramref name="inner"/> 盒子。
    /// </summary>
    /// <remarks>这是 <see cref="Contains(in JBoundingBox, in JBoundingBox)"/> 的别名。</remarks>
    [Obsolete($"Use {nameof(Contains)} instead.", true)]
    public static bool Encompasses(in JBoundingBox outer, in JBoundingBox inner)
    {
        return outer.Min.X <= inner.Min.X && outer.Max.X >= inner.Max.X && outer.Min.Y <= inner.Min.Y && outer.Max.Y >= inner.Max.Y &&
               outer.Min.Z <= inner.Min.Z && outer.Max.Z >= inner.Max.Z;
    }

    /// <summary>
    /// 创建一个新包围盒，它是另外两个包围盒的并集。
    /// </summary>
    /// <param name="original">第一个包围盒。</param>
    /// <param name="additional">第二个包围盒。</param>
    /// <returns>包含这两个输入的包围盒。</returns>
    public static JBoundingBox CreateMerged(in JBoundingBox original, in JBoundingBox additional)
    {
        CreateMerged(original, additional, out JBoundingBox result);
        return result;
    }

    /// <summary>
    /// 创建一个新包围盒，它是另外两个包围盒的并集。
    /// </summary>
    /// <param name="original">第一个包围盒。</param>
    /// <param name="additional">第二个包围盒。</param>
    /// <param name="result">输出：包含这两个输入的包围盒。</param>
    public static void CreateMerged(in JBoundingBox original, in JBoundingBox additional, out JBoundingBox result)
    {
        JVector.Min(original.Min, additional.Min, out result.Min);
        JVector.Max(original.Max, additional.Max, out result.Max);
    }

    /// <summary>
    /// 获取包围盒的中心点。
    /// </summary>
    public readonly JVector Center => (Min + Max) * ((Real)(1.0 / 2.0));

    /// <summary>
    /// 计算包围盒的体积。
    /// </summary>
    public readonly Real GetVolume()
    {
        JVector len = Max - Min;
        return len.X * len.Y * len.Z;
    }

    /// <summary>
    /// 计算包围盒的表面积。
    /// </summary>
    public readonly Real GetSurfaceArea()
    {
        JVector len = Max - Min;
        return (Real)2.0 * (len.X * len.Y + len.Y * len.Z + len.Z * len.X);
    }

    public readonly bool Equals(JBoundingBox other)
    {
        return Min.Equals(other.Min) && Max.Equals(other.Max);
    }

    public readonly override bool Equals(object? obj)
    {
        return obj is JBoundingBox other && Equals(other);
    }

    public readonly override int GetHashCode() => HashCode.Combine(Min, Max);

    public static bool operator ==(JBoundingBox left, JBoundingBox right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(JBoundingBox left, JBoundingBox right)
    {
        return !(left == right);
    }
}
