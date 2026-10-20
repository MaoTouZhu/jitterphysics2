/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2;

/// <summary>以 <c>0xRRGGBBAA</c> 形式打包的常用颜色。</summary>
public static class DebugDrawColors
{
    public const uint White = 0xFFFFFFFF;
    public const uint Green = 0x00FF00FF;
    public const uint Red = 0xFF0000FF;

    // 较浅的坐标轴颜色在深色背景下仍然清晰可读。
    public const uint AxisX = 0xFF6666FF;
    public const uint AxisY = 0x66EE88FF;
    public const uint AxisZ = 0x77AAFFFF;
}

/// <summary>
/// 控制调试可视化的大小、细节和颜色。
/// 默认初始化的值使用与 <see cref="Default"/> 相同的设置。
/// </summary>
public readonly struct DebugDrawSettings
{
    private readonly Real guideSize, frameSize, arrowHeadSize;
    private readonly int segments, arrowSegments, shapeSubdivisions;
    private readonly JVector? origin;
    private readonly uint? referenceColor, lowerLimitColor, upperLimitColor, currentColor;
    private readonly uint? axisXColor, axisYColor, axisZColor;
    private readonly bool? solidArrows;
    private readonly Real? arrowShaftRadius, pointRadius;

    /// <summary>
    /// 默认设置：引导线大小 1、坐标架大小 1、点半径 0.06、曲线段数 48、
    /// 箭头段数 12、形状细节 3、实心箭头、绿色引导线、红色下限、
    /// 白色当前姿态，以及 RGB 方向坐标架。
    /// </summary>
    public static DebugDrawSettings Default => default;

    /// <summary>
    /// 角度引导线的半径以及平面贴片的半长，单位为世界单位。
    /// 同时设置约束箭头长度和限位标记大小。坐标架改用 <see cref="FrameSize"/>。
    /// 箭头头部、箭杆和锚点球分别独立使用 <see cref="ArrowHeadSize"/>、
    /// <see cref="ArrowShaftRadius"/> 和 <see cref="PointRadius"/>。
    /// 物理距离和线性限位从不缩放。
    /// 必须为有限值且大于零。默认值为 1。
    /// </summary>
    public Real GuideSize
    {
        get => guideSize == 0 ? 1 : guideSize;
        init
        {
            if (!Real.IsFinite(value) || value <= 0) throw new ArgumentOutOfRangeException(nameof(GuideSize));
            guideSize = value;
        }
    }

    /// <summary>
    /// 每个坐标架轴的长度，单位为世界单位，与 <see cref="GuideSize"/> 无关。
    /// 箭头头部大小和箭杆半径也相互独立。
    /// 必须为有限值且大于零。默认值为 1。
    /// </summary>
    public Real FrameSize
    {
        get => frameSize == 0 ? 1 : frameSize;
        init
        {
            if (!Real.IsFinite(value) || value <= 0) throw new ArgumentOutOfRangeException(nameof(FrameSize));
            frameSize = value;
        }
    }

    /// <summary>
    /// 球形锚点标记的半径，单位为世界单位，与 <see cref="GuideSize"/> 无关。
    /// 默认值为 0.06。必须为有限值且非负。取零会隐藏这些标记。
    /// </summary>
    public Real PointRadius
    {
        get => pointRadius ?? (Real)0.06;
        init
        {
            if (!Real.IsFinite(value) || value < 0) throw new ArgumentOutOfRangeException(nameof(PointRadius));
            pointRadius = value;
        }
    }

    /// <summary>
    /// 箭头头部的长度，单位为世界单位，与箭头的总长度、<see cref="GuideSize"/>、
    /// <see cref="FrameSize"/> 以及箭杆粗细无关。默认值为 0.2。实心箭头和线框箭头都会使用。
    /// 箭头头部会被限制为箭头长度的一半；其底面半径为自身长度的 40%。
    /// 必须为有限值且大于零。
    /// </summary>
    public Real ArrowHeadSize
    {
        get => arrowHeadSize == 0 ? (Real)0.2 : arrowHeadSize;
        init
        {
            if (!Real.IsFinite(value) || value <= 0) throw new ArgumentOutOfRangeException(nameof(ArrowHeadSize));
            arrowHeadSize = value;
        }
    }

    /// <summary>每个圆、锥体边界或内置圆形形状的段数，范围为 8 到 256。默认值为 48。箭头使用 <see cref="ArrowSegments"/>。</summary>
    public int Segments
    {
        get => segments == 0 ? 48 : segments;
        init
        {
            if (value < 8 || value > 256) throw new ArgumentOutOfRangeException(nameof(Segments));
            segments = value;
        }
    }

    /// <summary>
    /// 每个箭头头部底面和实心箭杆周围的段数，范围为 3 到 256。默认值为 12。
    /// 取 4 会绘制方形底面；更大的值近似为圆形。
    /// 线框箭头将每个底面顶点连接到尖端。
    /// </summary>
    public int ArrowSegments
    {
        get => arrowSegments == 0 ? 12 : arrowSegments;
        init
        {
            if (value < 3 || value > 256) throw new ArgumentOutOfRangeException(nameof(ArrowSegments));
            arrowSegments = value;
        }
    }

    /// <summary>
    /// 将箭头和方向坐标架绘制为带锥形头部的实心箭杆。默认值为 true。
    /// 设为 false 可绘制线框箭头。
    /// </summary>
    public bool SolidArrows { get => solidArrows ?? true; init => solidArrows = value; }

    /// <summary>
    /// 实心箭杆的半径，单位为世界单位，与箭头长度、
    /// <see cref="GuideSize"/> 和 <see cref="FrameSize"/> 无关。默认值为 0.025。
    /// 必须为有限值且大于零。半径会被限制为箭头长度的 5% 以及
    /// 箭头头部底面半径的一半。线框箭头不受影响。
    /// </summary>
    public Real ArrowShaftRadius
    {
        get => arrowShaftRadius ?? (Real)0.025;
        init
        {
            if (!Real.IsFinite(value) || value <= 0) throw new ArgumentOutOfRangeException(nameof(ArrowShaftRadius));
            arrowShaftRadius = value;
        }
    }

    /// <summary>
    /// 点云和使用近似表面的自定义形状的绘制细节。
    /// 必须至少为 1。默认值为 3。每增加一级，绘制开销约增加四倍。
    /// 内置圆形形状改用 <see cref="Segments"/>。
    /// </summary>
    public int ShapeSubdivisions
    {
        get => shapeSubdivisions == 0 ? 3 : shapeSubdivisions;
        init
        {
            if (value < 1) throw new ArgumentOutOfRangeException(nameof(ShapeSubdivisions));
            shapeSubdivisions = value;
        }
    }

    /// <summary>
    /// 角度引导线可选的世界空间原点。默认使用刚体 1 的位置，
    /// 当刚体 1 为 World.NullBody 时使用刚体 2 的位置。带有球窝的关节使用其锚点。
    /// FixedAngle 的坐标架始终使用刚体的位置。
    /// 位置约束始终使用其真实锚点。
    /// </summary>
    public JVector? Origin
    {
        get => origin;
        init
        {
            if (value.HasValue) ArgumentCheck.Finite(value.Value, nameof(Origin));
            origin = value;
        }
    }

    /// <summary>参考引导线的颜色。默认值为绿色。</summary>
    public uint ReferenceColor { get => referenceColor ?? DebugDrawColors.Green; init => referenceColor = value; }

    /// <summary>下限的颜色。默认值为红色。</summary>
    public uint LowerLimitColor { get => lowerLimitColor ?? DebugDrawColors.Red; init => lowerLimitColor = value; }

    /// <summary>上限和固定限位的颜色。默认值为绿色。</summary>
    public uint UpperLimitColor { get => upperLimitColor ?? DebugDrawColors.Green; init => upperLimitColor = value; }

    /// <summary>当前锚点、方向和形状几何的颜色。默认值为白色。</summary>
    public uint CurrentColor { get => currentColor ?? DebugDrawColors.White; init => currentColor = value; }

    /// <summary>坐标架局部 X 轴的颜色。默认值为浅红色。</summary>
    public uint AxisXColor { get => axisXColor ?? DebugDrawColors.AxisX; init => axisXColor = value; }

    /// <summary>坐标架局部 Y 轴的颜色。默认值为浅绿色。</summary>
    public uint AxisYColor { get => axisYColor ?? DebugDrawColors.AxisY; init => axisYColor = value; }

    /// <summary>坐标架局部 Z 轴的颜色。默认值为浅蓝色。</summary>
    public uint AxisZColor { get => axisZColor ?? DebugDrawColors.AxisZ; init => axisZColor = value; }
}
