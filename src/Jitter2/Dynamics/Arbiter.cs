/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.Unmanaged;

namespace Jitter2.Dynamics;

/// <summary>
/// 管理两个不同刚体之间的接触信息。
/// </summary>
/// <remarks>
/// <para>
/// 仲裁器在接触被登记时创建（包括形状重叠之前的推测接触），并在二者分离
/// 或其中一个相关刚体从世界中移除时删除。每个仲裁器最多可保存
/// 四个缓存的接触点（参见 <see cref="ContactData"/>）。
/// </para>
/// <para>
/// 大多数情况下仲裁器以形状标识符为键。对于由引擎创建的仲裁器，顺序
/// 是规范的：ID 较小的形状始终排在前面。
/// </para>
/// <para>
/// <see cref="Handle"/> 属性提供对存储于非托管内存中的底层 <see cref="ContactData"/>
/// 的访问。该数据仅在仲裁器存在期间有效，不得
/// 与 <see cref="World.Step(Real, bool)"/> 并发访问。
/// </para>
/// <para>
/// 被移除的仲裁器由其所属的世界回收。不要在移除后继续保留仲裁器：
/// 同一实例随后可能表示另一对刚体。
/// </para>
/// </remarks>
public sealed class Arbiter
{
    /// <summary>
    /// 获取参与此接触的第一个刚体。
    /// </summary>
    public RigidBody Body1 { get; internal set; } = null!;

    /// <summary>
    /// 获取参与此接触的第二个刚体。
    /// </summary>
    public RigidBody Body2 { get; internal set; } = null!;

    /// <summary>
    /// 获取存储在非托管内存中的 <see cref="ContactData"/> 的句柄。
    /// </summary>
    /// <remarks>
    /// 底层数据仅在仲裁器注册于世界期间有效。
    /// 移除之后，仲裁器对象可能被回收用于另一对接触；在仲裁器的碰撞生命周期结束后，
    /// 不要保留或访问该句柄。
    /// </remarks>
    public JHandle<ContactData> Handle { get; internal set; }
}

/// <summary>
/// 表示用于查找 <see cref="Arbiter"/> 的有序标识符对。
/// </summary>
/// <remarks>
/// <paramref name="key1"/> 和 <paramref name="key2"/> 的顺序对相等比较有影响。
/// 对于由物理引擎创建的仲裁器，<paramref name="key1"/> 始终小于 <paramref name="key2"/>。
/// </remarks>
/// <param name="key1">第一个标识符（通常是较小的形状 ID）。</param>
/// <param name="key2">第二个标识符（通常是较大的形状 ID）。</param>
public readonly struct ArbiterKey(ulong key1, ulong key2) : IEquatable<ArbiterKey>
{
    /// <summary>
    /// 对中的第一个标识符。
    /// </summary>
    public readonly ulong Key1 = key1;

    /// <summary>
    /// 对中的第二个标识符。
    /// </summary>
    public readonly ulong Key2 = key2;

    public bool Equals(ArbiterKey other)
    {
        return Key1 == other.Key1 && Key2 == other.Key2;
    }

    public override bool Equals(object? obj)
    {
        return obj is ArbiterKey other && Equals(other);
    }

    public override int GetHashCode()
    {
        return HashCode.Combine(Key1, Key2);
    }

    public static bool operator ==(ArbiterKey left, ArbiterKey right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(ArbiterKey left, ArbiterKey right)
    {
        return !(left == right);
    }
}
