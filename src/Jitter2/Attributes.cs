/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;

namespace Jitter2;

/// <summary>
/// 表示参考系的枚举。
/// </summary>
public enum ReferenceFrame
{
    Local,
    World
}

/// <summary>
/// 用于指定成员的参考系的特性。
/// </summary>
[AttributeUsage(AttributeTargets.All)]
public sealed class ReferenceFrameAttribute : Attribute
{
    /// <summary>
    /// 获取或设置参考系。
    /// </summary>
    public ReferenceFrame Frame { get; set; }

    /// <summary>
    /// 初始化 <see cref="ReferenceFrameAttribute"/> 类的新实例。
    /// </summary>
    /// <param name="frame">参考系。</param>
    public ReferenceFrameAttribute(ReferenceFrame frame)
    {
        Frame = frame;
    }
}

/// <summary>
/// 指定代码执行或回调被调用时所在的线程上下文。
/// </summary>
public enum ThreadContext
{
    /// <summary>
    /// 从主线程（或调用 World.Step 的线程）调用。
    /// 可以安全地访问全局状态。
    /// </summary>
    MainThread,

    /// <summary>
    /// 从后台工作线程调用。
    /// 代码必须是线程安全且无锁的。
    /// </summary>
    ParallelWorker,

    /// <summary>
    /// 可能从上述任意线程调用。使用时需谨慎。
    /// </summary>
    Any
}

/// <summary>
/// 指示回调或事件预期被调用时所在的线程上下文。
/// 此特性主要用于提供信息，并用于文档目的。
/// </summary>
[AttributeUsage(AttributeTargets.Method | AttributeTargets.Event)]
public sealed class CallbackThreadAttribute(ThreadContext context) : Attribute
{
    /// <summary>
    /// 获取预期调用所标注成员时所在的线程上下文。
    /// </summary>
    public ThreadContext Context { get; } = context;
}
