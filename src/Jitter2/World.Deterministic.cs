/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Jitter2.Collision;
using Jitter2.DataStructures;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Constraints;
using Jitter2.Parallelization;
using Jitter2.Unmanaged;

namespace Jitter2;

/// <summary>
/// 控制 <see cref="World.Step"/> 期间使用的求解器策略。
/// </summary>
public enum SolveMode
{
    /// <summary>
    /// 标准并行求解器。接触和约束以固定大小的批次分配到各个线程。
    /// 一次增量式、对缓存友好的重排序会让相关接触
    /// 在内存中相邻。速度快，但不具确定性。
    /// </summary>
    Regular,

    /// <summary>
    /// 基于岛的确定性求解器。每个模拟岛作为独立任务
    /// 依次求解。启用多线程时，各岛会分配到多个线程。
    /// 此模式可能比 <see cref="Regular"/> 慢得多。
    /// </summary>
    Deterministic
}

public sealed partial class World
{
    private SolveMode solveMode = Jitter2.SolveMode.Regular;

    /// <summary>
    /// <see cref="Step"/> 期间使用的求解器策略。默认为 <see cref="Jitter2.SolveMode.Regular"/>。
    /// </summary>
    /// <remarks>
    /// <see cref="Jitter2.SolveMode.Deterministic"/> 可能比
    /// <see cref="Jitter2.SolveMode.Regular"/> 慢得多。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">当赋予未定义的 <see cref="Jitter2.SolveMode"/>
    /// 值时抛出。</exception>
    public SolveMode SolveMode
    {
        get => solveMode;
        set
        {
            if (!Enum.IsDefined(value))
            {
                throw new ArgumentOutOfRangeException(nameof(value), value,
                    "The solve mode must be a defined enum value.");
            }

            solveMode = value;
        }
    }

    // -------------------------------------------------------------------------
    // 岛求解的内部状态
    // -------------------------------------------------------------------------

    private struct IslandRange
    {
        public int ContactStart, ContactEnd;
        public int SmallStart, SmallEnd;
        public int ConstraintStart, ConstraintEnd;
    }

    private struct ContactEntry
    {
        public int Index;
        public int IslandIndex;
        public ArbiterKey Key;
    }

    private struct SmallConstraintEntry
    {
        public JHandle<SmallConstraintData> Handle;
        public int IslandIndex;
        public ulong ConstraintId;
    }

    private struct ConstraintEntry
    {
        public JHandle<ConstraintData> Handle;
        public int IslandIndex;
        public ulong ConstraintId;
    }

    // 将 RigidBodyData._index 映射到岛的 SetIndex。每帧清空；容量按需增长，可通过 Trim() 缩减。
    private readonly Dictionary<int, int> handleToIsland = new();

    // 每帧根据排序后的缓冲区构建的岛范围。
    private readonly List<IslandRange> islandRanges = new();
    private readonly List<ContactEntry> sortedContacts = new();
    private readonly List<SmallConstraintEntry> sortedSmallConstraints = new();
    private readonly List<ConstraintEntry> sortedConstraints = new();

    private int islandSolverIterations;
    private int islandRelaxationIterations;

    // -------------------------------------------------------------------------
    // 构建岛查找表。
    // -------------------------------------------------------------------------

    private void BuildIslandLookup()
    {
        handleToIsland.Clear();

        // 将每个活动刚体的句柄索引映射到其活动岛分区索引。
        for (int i = 0; i < bodies.ActiveCount; i++)
        {
            RigidBody body = bodies[i];
            handleToIsland.Add(body.Data._index, ((IPartitionedSetIndex)body.Island).SetIndex);
        }
    }

    // -------------------------------------------------------------------------
    // 辅助方法
    // -------------------------------------------------------------------------

    private int IslandOf(JHandle<RigidBodyData> b1, JHandle<RigidBodyData> b2)
    {
        return b1.Data.MotionType != MotionType.Static
            ? handleToIsland[b1.Data._index]
            : handleToIsland[b2.Data._index];
    }

    // -------------------------------------------------------------------------
    // 为接触和约束构建排序后的附属列表。
    // -------------------------------------------------------------------------

    private void SortForIslands()
    {
        sortedContacts.Clear();
        var contacts = memContacts.Active;
        for (int i = 0; i < contacts.Length; i++)
        {
            ref ContactData contact = ref contacts[i];
            sortedContacts.Add(new ContactEntry
            {
                Index = i,
                IslandIndex = IslandOf(contact.Body1, contact.Body2),
                Key = contact.Key
            });
        }

        sortedContacts.Sort(static (a, b) =>
        {
            if (a.IslandIndex != b.IslandIndex) return a.IslandIndex.CompareTo(b.IslandIndex);
            if (a.Key.Key1 != b.Key.Key1) return a.Key.Key1 < b.Key.Key1 ? -1 : 1;
            if (a.Key.Key2 != b.Key.Key2) return a.Key.Key2 < b.Key.Key2 ? -1 : 1;
            return 0; // 接触必须相同。排序 *确实会* 将对象与自身比较。
        });

        sortedSmallConstraints.Clear();
        var small = memSmallConstraints.Active;
        for (int i = 0; i < small.Length; i++)
        {
            var handle = memSmallConstraints.GetHandle(ref small[i]);
            sortedSmallConstraints.Add(new SmallConstraintEntry
            {
                Handle = handle,
                IslandIndex = IslandOf(handle.Data.Body1, handle.Data.Body2),
                ConstraintId = handle.Data.ConstraintId
            });
        }

        sortedConstraints.Clear();
        var constraints = memConstraints.Active;
        for (int i = 0; i < constraints.Length; i++)
        {
            var handle = memConstraints.GetHandle(ref constraints[i]);
            sortedConstraints.Add(new ConstraintEntry
            {
                Handle = handle,
                IslandIndex = IslandOf(handle.Data.Body1, handle.Data.Body2),
                ConstraintId = handle.Data.ConstraintId
            });
        }

        // 按岛对约束分组，并使用存储的约束标识符
        // 作为每个岛内的确定性决胜条件。
        sortedSmallConstraints.Sort(static (a, b) =>
        {
            if (a.IslandIndex != b.IslandIndex) return a.IslandIndex.CompareTo(b.IslandIndex);
            return a.ConstraintId.CompareTo(b.ConstraintId);
        });
        sortedConstraints.Sort(static (a, b) =>
        {
            if (a.IslandIndex != b.IslandIndex) return a.IslandIndex.CompareTo(b.IslandIndex);
            return a.ConstraintId.CompareTo(b.ConstraintId);
        });
    }

    // -------------------------------------------------------------------------
    // 从排序后的缓冲区构建每个岛的范围（单次线性遍历）
    // -------------------------------------------------------------------------

    private void BuildIslandRanges()
    {
        islandRanges.Clear();

        var contacts = CollectionsMarshal.AsSpan(sortedContacts);
        var small = CollectionsMarshal.AsSpan(sortedSmallConstraints);
        var constraints = CollectionsMarshal.AsSpan(sortedConstraints);

        int ci = 0, si = 0, ri = 0;

        while (ci < contacts.Length || si < small.Length || ri < constraints.Length)
        {
            // 在三个当前扫描位置中找出最小的岛索引。
            int minIsland = int.MaxValue;
            if (ci < contacts.Length) minIsland = Math.Min(minIsland, contacts[ci].IslandIndex);
            if (si < small.Length) minIsland = Math.Min(minIsland, small[si].IslandIndex);
            if (ri < constraints.Length) minIsland = Math.Min(minIsland, constraints[ri].IslandIndex);

            IslandRange range = new();

            range.ContactStart = ci;
            while (ci < contacts.Length && contacts[ci].IslandIndex == minIsland) ci++;
            range.ContactEnd = ci;

            range.SmallStart = si;
            while (si < small.Length && small[si].IslandIndex == minIsland) si++;
            range.SmallEnd = si;

            range.ConstraintStart = ri;
            while (ri < constraints.Length && constraints[ri].IslandIndex == minIsland) ri++;
            range.ConstraintEnd = ri;

            islandRanges.Add(range);
        }
    }

    // -------------------------------------------------------------------------
    // 岛求解任务：依次为一段岛范围准备并迭代所有接触/约束。
    // 无需加锁——各岛相互独立。
    // -------------------------------------------------------------------------

    private void SolveIslandBatch(Parallel.Batch batch)
    {
        var allContacts = memContacts.Active;
        var orderedContacts = CollectionsMarshal.AsSpan(sortedContacts);
        var allSmall = CollectionsMarshal.AsSpan(sortedSmallConstraints);
        var allConstraints = CollectionsMarshal.AsSpan(sortedConstraints);

        for (int idx = batch.Start; idx < batch.End; idx++)
        {
            IslandRange range = islandRanges[idx];

            var contacts = orderedContacts[range.ContactStart..range.ContactEnd];
            var small = allSmall[range.SmallStart..range.SmallEnd];
            var constraints = allConstraints[range.ConstraintStart..range.ConstraintEnd];

            // --- 准备 ---

            for (int i = 0; i < contacts.Length; i++)
            {
                ref ContactData c = ref allContacts[contacts[i].Index];
                c.PrepareForIteration(timeStep);
            }

            for (int i = 0; i < constraints.Length; i++)
            {
                ref ConstraintData c = ref constraints[i].Handle.Data;
                if (c.IsEnabled)
                    c.PrepareForIteration(ref c, timeStep);
            }

            for (int i = 0; i < small.Length; i++)
            {
                ref SmallConstraintData c = ref small[i].Handle.Data;
                if (c.IsEnabled)
                    c.PrepareForIteration(ref c, timeStep);
            }

            // --- 迭代 ---

            for (int iter = 0; iter < islandSolverIterations; iter++)
            {
                for (int i = 0; i < contacts.Length; i++)
                {
                    ref ContactData c = ref allContacts[contacts[i].Index];
                    c.Iterate(true);
                }

                for (int i = 0; i < constraints.Length; i++)
                {
                    ref ConstraintData c = ref constraints[i].Handle.Data;
                    if (c.IsEnabled)
                        c.Iterate(ref c, timeStep);
                }

                for (int i = 0; i < small.Length; i++)
                {
                    ref SmallConstraintData c = ref small[i].Handle.Data;
                    if (c.IsEnabled)
                        c.Iterate(ref c, timeStep);
                }
            }

        }
    }

    private void RelaxIslandBatch(Parallel.Batch batch)
    {
        var allContacts = memContacts.Active;
        var orderedContacts = CollectionsMarshal.AsSpan(sortedContacts);

        for (int idx = batch.Start; idx < batch.End; idx++)
        {
            IslandRange range = islandRanges[idx];
            var contacts = orderedContacts[range.ContactStart..range.ContactEnd];

            for (int iter = 0; iter < islandRelaxationIterations; iter++)
            {
                for (int i = 0; i < contacts.Length; i++)
                {
                    ref ContactData c = ref allContacts[contacts[i].Index];
                    c.Iterate(false);
                }
            }
        }
    }

    // -------------------------------------------------------------------------
    // 入口点：构建岛查找 → 排序附属列表 → 构建范围。
    // -------------------------------------------------------------------------

    private void PrepareIslandSolveOrder()
    {
        BuildIslandLookup();
        SortForIslands();
        BuildIslandRanges();
    }

    private void RelaxIslands(bool multiThread, int iterations)
    {
        islandRelaxationIterations = iterations;

        int numIslands = islandRanges.Count;
        if (numIslands == 0) return;

        int numTasks = multiThread ? Math.Min(numIslands, ThreadPool.Instance.ThreadCount) : 1;

        if (numTasks <= 1)
        {
            RelaxIslandBatch(new Parallel.Batch(0, numIslands));
            return;
        }

        Parallel.ForBatch(0, numIslands, numTasks, relaxIsland);
    }

    private void SolveIslands(bool multiThread, int iterations)
    {
        islandSolverIterations = iterations;

        int numIslands = islandRanges.Count;
        if (numIslands == 0) return;

        int numTasks = multiThread ? Math.Min(numIslands, ThreadPool.Instance.ThreadCount) : 1;

        if (numTasks <= 1)
        {
            // 单线程回退：在调用线程上处理所有岛。
            SolveIslandBatch(new Parallel.Batch(0, numIslands));
            return;
        }

        // 将各岛范围均匀分配到各个线程。每个任务获得 islandRanges 的
        // 一段连续切片并依次求解。
        // 各任务之间的岛完全独立——无需加锁。
        Parallel.ForBatch(0, numIslands, numTasks, solveIsland);
    }
}
