/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.LinearMath;

namespace Jitter2;

/// <summary>
/// 定义可进行调试绘制的对象接口。
/// </summary>
public interface IDebugDrawable
{
    /// <summary>
    /// 传入一个 <see cref="IDebugDrawer"/>，用于绘制该对象的基本调试信息。
    /// </summary>
    /// <param name="drawer">用于渲染调试信息的调试绘制器。</param>
    public void DebugDraw(IDebugDrawer drawer);

    /// <summary>
    /// 使用提供的可视化设置绘制该对象。
    /// 仅提供 <see cref="DebugDraw(IDebugDrawer)"/> 的实现使用其默认可视化方式。
    /// </summary>
    public void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings) => DebugDraw(drawer);
}

/// <summary>
/// 在世界空间中接收用于调试可视化的彩色线段和三角形。
/// </summary>
/// <remarks>
/// 两个绘制方法都必须实现。线框绘制器可以将三角形绘制为三条线段。
/// 使用 <see cref="DebugDrawGeometry"/> 生成点标记及其他绘制几何体。
/// </remarks>
public interface IDebugDrawer
{
    /// <summary>
    /// 绘制两点之间的线段，颜色以 <c>0xRRGGBBAA</c> 格式打包。
    /// </summary>
    /// <param name="pA">线段在世界空间中的起点。</param>
    /// <param name="pB">线段在世界空间中的终点。</param>
    /// <param name="color">以 <c>0xRRGGBBAA</c> 格式打包的颜色。</param>
    public void DrawSegment(in JVector pA, in JVector pB, uint color);

    /// <summary>
    /// 绘制一个三角形，颜色以 <c>0xRRGGBBAA</c> 格式打包。
    /// </summary>
    /// <remarks>
    /// 从正面观察时顶点按逆时针排列。正面法线指向
    /// <c>(pB - pA) x (pC - pA)</c>。实现可以使用背面剔除。
    /// </remarks>
    /// <param name="pA">世界空间中的第一个顶点。</param>
    /// <param name="pB">世界空间中的第二个顶点。</param>
    /// <param name="pC">世界空间中的第三个顶点。</param>
    /// <param name="color">以 <c>0xRRGGBBAA</c> 格式打包的颜色。</param>
    public void DrawTriangle(in JVector pA, in JVector pB, in JVector pC, uint color);
}
