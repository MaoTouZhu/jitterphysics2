/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Diagnostics;

namespace Jitter2.Parallelization;

/// <summary>
/// 为 Jitter 物理引擎内的并行批处理提供方法与结构。
/// </summary>
/// <remarks>
/// 此类在内部使用，通过 <see cref="ThreadPool"/> 将工作分配到各工作线程。
/// </remarks>
public static class Parallel
{
    /// <summary>
    /// 表示由并行批次处理的半开索引区间。
    /// 该结构由 <see cref="ForBatch"/> 使用，将 for 循环拆分为工作线程任务。
    /// </summary>
    public readonly struct Batch(int start, int end)
    {
        /// <summary>返回批次的字符串表示形式。</summary>
        public override string ToString()
        {
            return $"Batch(Start: {Start}, End: {End})";
        }

        /// <summary>批次的起始索引（含）。</summary>
        public readonly int Start = start;

        /// <summary>批次的结束索引（不含）。</summary>
        public readonly int End = end;
    }

    /// <summary>
    /// 计算均分区间中某一部分的起始与结束索引。
    /// </summary>
    /// <param name="numElements">要划分的元素总数。</param>
    /// <param name="numDivisions">划分份数（部分数）。</param>
    /// <param name="part">部分的从零开始的索引。</param>
    /// <param name="start">指定部分的起始索引（含）。</param>
    /// <param name="end">指定部分的结束索引（不含）。</param>
    /// <remarks>
    /// 将余数元素分配到前面的部分。例如，14 个元素
    /// 和 4 个部分：部分 0 得到 [0,4)，部分 1 得到 [4,8)，部分 2 得到 [8,11)，部分 3 得到 [11,14)。
    /// </remarks>
    public static void GetBounds(int numElements, int numDivisions, int part, out int start, out int end)
    {
        Debug.Assert(part < numDivisions);

        int div = Math.DivRem(numElements, numDivisions, out int mod);

        start = div * part + Math.Min(part, mod);
        end = start + div + (part < mod ? 1 : 0);
    }

    private static readonly ThreadPool threadPool = ThreadPool.Instance;

    /// <summary>
    /// 通过使用 <see cref="ThreadPool"/> 将工作划分为批次，并行执行任务。
    /// </summary>
    /// <param name="lower">要处理区间的下界（含）。</param>
    /// <param name="upper">要处理区间的上界（不含）。</param>
    /// <param name="numTasks">要创建的批次数。</param>
    /// <param name="action">要对每个批次执行的回调。</param>
    /// <param name="execute">是否立即执行已排队的批次。</param>
    /// <remarks>
    /// 该方法将区间 [lower, upper) 拆分为 <paramref name="numTasks"/> 个批次，并并行处理每个批次。
    /// <paramref name="action"/> 回调会对每个批次调用，批次由 <see cref="Batch"/> 结构表示。
    /// 如果 <paramref name="execute"/> 为 true，方法会调用 <see cref="ThreadPool.Execute"/> 开始执行任务。
    /// </remarks>
    public static void ForBatch(int lower, int upper, int numTasks, Action<Batch> action, bool execute = true)
    {
        Debug.Assert(numTasks <= ushort.MaxValue);
        for (int i = 0; i < numTasks; i++)
        {
            GetBounds(upper - lower, numTasks, i, out int start, out int end);
            threadPool.AddTask(action, new Batch(start + lower, end + lower));
        }

        if (execute) threadPool.Execute();
    }
}
