/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

namespace Jitter2.Collision;

/// <summary>
/// 提供接入宽相碰撞检测管线的钩子。
/// </summary>
/// <remarks>
/// 实现此接口可在窄相检测之前拦截形状对。
/// 可用于过滤特定形状对、实现自定义碰撞层，
/// 或为自定义代理类型处理碰撞。参见 <see cref="World.BroadPhaseFilter"/>。
/// </remarks>
public interface IBroadPhaseFilter
{
    /// <summary>
    /// 对每一对包围盒重叠的代理调用。
    /// </summary>
    /// <param name="proxyA">第一个代理。</param>
    /// <param name="proxyB">第二个代理。</param>
    /// <returns><c>true</c> 表示继续窄相检测；<c>false</c> 表示跳过此形状对。</returns>
    [CallbackThread(ThreadContext.Any)]
    bool Filter(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB);
}
