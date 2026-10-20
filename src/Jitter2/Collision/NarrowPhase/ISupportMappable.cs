/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 定义了一个由支撑函数刻画的通用凸形状接口。
/// </summary>
/// <remarks>
/// 支撑函数是基于 GJK 的碰撞检测算法的基本操作。
/// 任何凸形状都可以通过其支撑映射隐式表示，而无需
/// 显式的顶点或面数据。
/// </remarks>
public interface ISupportMappable
{
    /// <summary>
    /// 计算形状上沿指定方向最远的点。
    /// </summary>
    /// <param name="direction">局部空间中的搜索方向。不需要归一化。</param>
    /// <param name="result">形状表面上沿 <paramref name="direction"/> 方向最远的点。</param>
    void SupportMap(in JVector direction, out JVector result);

    /// <summary>
    /// 计算凸包内的一个代表点，用作基于 GJK 算法中的初始搜索点。
    /// </summary>
    /// <param name="point">凸包内部的一个点。</param>
    void GetCenter(out JVector point);
}
