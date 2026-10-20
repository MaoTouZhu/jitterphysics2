/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Constraints;

/// <summary>
/// 表示由最小角度和最大角度定义的角度限位。
/// 供约束使用，将旋转运动限制在指定范围内。
/// </summary>
/// <param name="from">限位的最小角度。</param>
/// <param name="to">限位的最大角度。</param>
public struct AngularLimit(JAngle from, JAngle to)
{
    /// <summary>
    /// 获取或设置限位的最小角度。
    /// </summary>
    public JAngle From { get; set; } = from;

    /// <summary>
    /// 获取或设置限位的最大角度。
    /// </summary>
    public JAngle To { get; set; } = to;

    /// <summary>
    /// 允许完整旋转的限位（-π 到 +π 弧度）。
    /// </summary>
    public static readonly AngularLimit Full =
        new(JAngle.FromRadian(-MathR.PI), JAngle.FromRadian(MathR.PI));

    /// <summary>
    /// 将角度锁定在原位（不允许旋转）的限位。
    /// </summary>
    public static readonly AngularLimit Fixed =
        new(JAngle.FromRadian(+(Real)1e-6), JAngle.FromRadian(-(Real)1e-6));

    /// <summary>
    /// 从角度值（度）创建角度限位。
    /// </summary>
    /// <param name="min">最小角度，单位为度。</param>
    /// <param name="max">最大角度，单位为度。</param>
    /// <returns>新的 <see cref="AngularLimit"/> 实例。</returns>
    public static AngularLimit FromDegree(Real min, Real max)
    {
        return new AngularLimit(JAngle.FromDegree(min), JAngle.FromDegree(max));
    }

    /// <summary>
    /// 将限位解构为最小角度和最大角度。
    /// </summary>
    /// <param name="limitMin">最小角度。</param>
    /// <param name="limitMax">最大角度。</param>
    public readonly void Deconstruct(out JAngle limitMin, out JAngle limitMax)
    {
        limitMin = From;
        limitMax = To;
    }
}

/// <summary>
/// 表示由最小距离和最大距离定义的线性限位。
/// 供约束使用，将平移运动限制在指定范围内。
/// </summary>
/// <param name="from">限位的最小距离。</param>
/// <param name="to">限位的最大距离。</param>
public struct LinearLimit(Real from, Real to)
{
    /// <summary>
    /// 获取或设置限位的最小距离。
    /// </summary>
    public Real From { get; set; } = from;

    /// <summary>
    /// 获取或设置限位的最大距离。
    /// </summary>
    public Real To { get; set; } = to;

    /// <summary>
    /// 允许无限制运动的限位（负无穷到正无穷）。
    /// </summary>
    public static readonly LinearLimit Full =
        new(Real.NegativeInfinity, Real.PositiveInfinity);

    /// <summary>
    /// 将位置锁定在原位（不允许平移）的限位。
    /// </summary>
    public static readonly LinearLimit Fixed =
        new((Real)1e-6, -(Real)1e-6);

    /// <summary>
    /// 从最小值和最大值创建线性限位。
    /// </summary>
    /// <param name="min">最小距离。</param>
    /// <param name="max">最大距离。</param>
    /// <returns>新的 <see cref="LinearLimit"/> 实例。</returns>
    public static LinearLimit FromMinMax(Real min, Real max)
    {
        return new LinearLimit(min, max);
    }

    /// <summary>
    /// 将限位解构为最小距离和最大距离。
    /// </summary>
    /// <param name="limitMin">最小距离。</param>
    /// <param name="limitMax">最大距离。</param>
    public readonly void Deconstruct(out Real limitMin, out Real limitMax)
    {
        limitMin = From;
        limitMax = To;
    }
}
