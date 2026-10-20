/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.DataStructures;
using Jitter2.Unmanaged;

namespace Jitter2.Parallelization;

/// <summary>
/// 为物理引擎所用的集合提供并行批处理扩展方法。
/// </summary>
/// <remarks>
/// 这些方法将集合划分为批次，并分配到各工作线程。
/// 批次数由任务阈值与可用线程数决定。
/// </remarks>
public static class ParallelExtensions
{
    /// <summary>
    /// 以并行批次处理数组元素。
    /// </summary>
    /// <param name="array">要处理的数组。</param>
    /// <param name="taskThreshold">用于选择批次数的正目标批大小；
    /// 实际批次可能更小或更大。</param>
    /// <param name="action">要对每个批次调用的回调。</param>
    /// <param name="execute">如果为 <see langword="true"/>，则调用 <see cref="ThreadPool.Execute"/> 开始执行。</param>
    /// <returns>生成的批次数。</returns>
    public static int ParallelForBatch(this Array array, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true)
    {
        int numTasks = array.Length / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, array.Length, numTasks, action, execute);

        return numTasks;
    }

    /// <summary>
    /// 以并行批次处理 <see cref="PartitionedBuffer{T}"/> 的活动元素。
    /// </summary>
    /// <typeparam name="T">非托管元素类型。</typeparam>
    /// <param name="list">要处理的缓冲区。</param>
    /// <param name="taskThreshold">用于选择批次数的正目标批大小；
    /// 实际批次可能更小或更大。</param>
    /// <param name="action">要对每个批次调用的回调。</param>
    /// <param name="execute">如果为 <see langword="true"/>，则调用 <see cref="ThreadPool.Execute"/> 开始执行。</param>
    /// <returns>生成的批次数。</returns>
    public static int ParallelForBatch<T>(this PartitionedBuffer<T> list, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true) where T : unmanaged
    {
        int numTasks = list.Active.Length / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, list.Active.Length, numTasks, action, execute);

        return numTasks;
    }

    /// <summary>
    /// 以并行批次处理 <see cref="ReadOnlyPartitionedSet{T}"/> 的活动元素。
    /// </summary>
    /// <typeparam name="T">元素类型。</typeparam>
    /// <param name="list">要处理的集合。</param>
    /// <param name="taskThreshold">用于选择批次数的正目标批大小；
    /// 实际批次可能更小或更大。</param>
    /// <param name="action">要对每个批次调用的回调。</param>
    /// <param name="execute">如果为 <see langword="true"/>，则调用 <see cref="ThreadPool.Execute"/> 开始执行。</param>
    /// <returns>生成的批次数。</returns>
    public static int ParallelForBatch<T>(this ReadOnlyPartitionedSet<T> list, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true) where T : class, IPartitionedSetIndex
    {
        int numTasks = list.ActiveCount / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, list.ActiveCount, numTasks, action, execute);

        return numTasks;
    }

    /// <summary>
    /// 以并行批次处理 <see cref="PartitionedSet{T}"/> 的活动元素。
    /// </summary>
    /// <typeparam name="T">元素类型。</typeparam>
    /// <param name="partitionedSet">要处理的集合。</param>
    /// <param name="taskThreshold">用于选择批次数的正目标批大小；
    /// 实际批次可能更小或更大。</param>
    /// <param name="action">要对每个批次调用的回调。</param>
    /// <param name="execute">如果为 <see langword="true"/>，则调用 <see cref="ThreadPool.Execute"/> 开始执行。</param>
    /// <returns>生成的批次数。</returns>
    internal static int ParallelForBatch<T>(this PartitionedSet<T> partitionedSet, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true) where T : class, IPartitionedSetIndex
    {
        int numTasks = partitionedSet.ActiveCount / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, partitionedSet.ActiveCount, numTasks, action, execute);

        return numTasks;
    }

    /// <summary>
    /// 以并行批次处理 <see cref="SlimBag{T}"/> 的元素。
    /// </summary>
    /// <typeparam name="T">元素类型。</typeparam>
    /// <param name="list">要处理的袋。</param>
    /// <param name="taskThreshold">用于选择批次数的正目标批大小；
    /// 实际批次可能更小或更大。</param>
    /// <param name="action">要对每个批次调用的回调。</param>
    /// <param name="execute">如果为 <see langword="true"/>，则调用 <see cref="ThreadPool.Execute"/> 开始执行。</param>
    /// <returns>生成的批次数。</returns>
    internal static int ParallelForBatch<T>(this SlimBag<T> list, int taskThreshold,
        Action<Parallel.Batch> action, bool execute = true) where T : class, IPartitionedSetIndex
    {
        int numTasks = list.Count / taskThreshold + 1;
        numTasks = Math.Min(numTasks, ThreadPool.Instance.ThreadCount);

        Parallel.ForBatch(0, list.Count, numTasks, action, execute);

        return numTasks;
    }
}
