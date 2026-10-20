/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.Collision.Shapes;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 提供接入窄相碰撞检测管线的钩子。
/// </summary>
/// <remarks>
/// 实现此接口可在接触生成之后拦截碰撞。
/// 可用于修改接触数据、实现自定义碰撞响应，
/// 或过滤掉特定碰撞。
/// </remarks>
public interface INarrowPhaseFilter
{
    /// <summary>
    /// 对每个检测到的碰撞及其接触数据调用。
    /// </summary>
    /// <param name="shapeA">碰撞中的第一个形状。</param>
    /// <param name="shapeB">碰撞中的第二个形状。</param>
    /// <param name="pointA">形状 A 上的接触点（可修改）。</param>
    /// <param name="pointB">形状 B 上的接触点（可修改）。</param>
    /// <param name="normal">用于求解接触的碰撞法线（可修改）。正的法线
    /// 冲量会将形状 B 沿此方向推开，并将形状 A 反向推开。</param>
    /// <param name="penetration">窄相报告的带符号穿透深度。仅修改此值
    /// 不会影响已注册的接触；其分离量由接触点和法线推导得出。</param>
    /// <returns><c>true</c> 表示保留该碰撞；<c>false</c> 表示丢弃该碰撞。</returns>
    [CallbackThread(ThreadContext.Any)]
    bool Filter(RigidBodyShape shapeA, RigidBodyShape shapeB,
        ref JVector pointA, ref JVector pointB,
        ref JVector normal, ref Real penetration);
}
