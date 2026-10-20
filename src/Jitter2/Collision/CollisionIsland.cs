/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;
using Jitter2.DataStructures;
using Jitter2.Dynamics;

namespace Jitter2.Collision;

/// <summary>
/// 表示通过接触或约束直接或间接连接的一组刚体。
/// </summary>
public sealed class Island : IPartitionedSetIndex
{
    internal readonly HashSet<RigidBody> InternalBodies = [];
    internal bool MarkedAsActive;

    /// <summary>
    /// 当岛处于活动状态但可能包含非活动刚体时，必须设置此标志。
    /// 例如，当一个非活动岛与一个活动岛合并时就会发生这种情况。
    /// </summary>
    internal bool NeedsUpdate;

    /// <summary>
    /// 获取此岛中所有刚体的集合。
    /// </summary>
    public ReadOnlyHashSet<RigidBody> Bodies => new(InternalBodies);

    int IPartitionedSetIndex.SetIndex { get; set; } = -1;

    /// <summary>
    /// 初始化 <see cref="Island"/> 类的新实例。
    /// </summary>
    public Island()
    {
    }

    /// <summary>
    /// 清除此岛内部列表中的所有刚体。
    /// </summary>
    internal void ClearLists()
    {
        InternalBodies.Clear();
    }

    internal void TrimLists()
    {
        InternalBodies.TrimExcess();
    }
}
