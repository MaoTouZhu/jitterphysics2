/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;
using System.Threading;
using Jitter2.Unmanaged;

namespace Jitter2.Parallelization;

/// <summary>
/// 提供一个轻量级的读写锁，针对罕见的写操作进行了优化。
/// </summary>
/// <remarks>
/// <para>
/// 多个读取者可同时持有该锁，但写入者拥有独占访问权。
/// 此实现使用自旋等待，最适合较短的临界区。
/// </para>
/// <para>
/// 该锁不可重入且与线程绑定。每次成功获取都必须由获取它的同一线程
/// 恰好释放一次。不支持递归获取、
/// 读锁升级为写锁、写锁降级为读锁，以及释放由
/// 其他线程获取的锁。
/// </para>
/// <para>
/// 按此约定使用时是线程安全的。所有同步均通过
/// 原子操作和内存屏障完成。
/// </para>
/// </remarks>
public sealed class ReaderWriterLock
{
    private const int YieldThreshold = 32;
    private const int SleepThreshold = 128;
    private const int ReaderSlotCount = 128;

    // 为每个线程分配一个稳定的读取者槽位。这样读取者通常可以更新
    // 各自独立的计数器，而不是争用一个共享计数器，从而避免
    // 缓存行抖动。
    [ThreadStatic]
    private static int readerSlotPlusOne;

    // 槽位分配在多个锁实例之间共享，因此一个线程通常会在
    // 每个 ReaderWriterLock 实例中复用同一个槽位。
    private static int nextReaderSlot;

    private int writer;

    // 读取者计数器经过填充，使相邻槽位的计数器通常不会
    // 占用同一缓存行。结合按线程分配槽位，这
    // 减少了并发读取者之间的缓存行抖动。
    private readonly MemoryHelper.PaddedInt[] readers =
        new MemoryHelper.PaddedInt[ReaderSlotCount];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Backoff(ref int count)
    {
        int current = count;
        if (count < int.MaxValue) count++;

        if (current < YieldThreshold)
        {
            Thread.SpinWait(1);
        }
        else if (current < SleepThreshold)
        {
            Thread.Yield();
        }
        else
        {
            Thread.Sleep(0);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static int GetReaderSlot()
    {
        int slot = readerSlotPlusOne - 1;
        if (slot >= 0) return slot;

        // 如果参与的线程数超过 ReaderSlotCount，槽位将被
        // 共享。这会增加争用，但不影响正确性。
        slot = (int)((uint)(Interlocked.Increment(ref nextReaderSlot) - 1) %
                     ReaderSlotCount);

        readerSlotPlusOne = slot + 1;
        return slot;
    }

    /// <summary>
    /// 获取读锁。
    /// </summary>
    /// <remarks>
    /// <para>
    /// 多个线程可以同时持有读锁。当写入者持有锁时，
    /// 此方法会阻塞。
    /// </para>
    /// <para>
    /// 该锁必须由获取它的线程使用 <see cref="ExitReadLock"/>
    /// 恰好释放一次。
    /// </para>
    /// </remarks>
    public void EnterReadLock()
    {
        int slot = GetReaderSlot();
        int backoff = 0;

        while (true)
        {
            while (Volatile.Read(ref writer) == 1)
            {
                Backoff(ref backoff);
            }

            Interlocked.Increment(ref readers[slot].Value);

            if (Volatile.Read(ref writer) == 0)
            {
                break;
            }

            Interlocked.Decrement(ref readers[slot].Value);
            Backoff(ref backoff);
        }
    }

    /// <summary>
    /// 以独占方式获取写锁。
    /// </summary>
    /// <remarks>
    /// <para>
    /// 此方法会阻塞，直到所有读取者和其他写入者都已释放该锁。
    /// </para>
    /// <para>
    /// 该锁必须由获取它的线程使用 <see cref="ExitWriteLock"/>
    /// 恰好释放一次。
    /// </para>
    /// </remarks>
    public void EnterWriteLock()
    {
        int backoff = 0;

        while (true)
        {
            if (Interlocked.CompareExchange(ref writer, 1, 0) == 0)
            {
                while (HasReaders())
                {
                    Backoff(ref backoff);
                }

                break;
            }

            Backoff(ref backoff);
        }
    }

    private bool HasReaders()
    {
        for (int i = 0; i < readers.Length; i++)
        {
            if (Volatile.Read(ref readers[i].Value) != 0)
            {
                return true;
            }
        }

        return false;
    }

    /// <summary>
    /// 释放当前线程先前获取的读锁。
    /// </summary>
    /// <remarks>
    /// 每次成功调用 <see cref="EnterReadLock"/> 后必须恰好调用一次，
    /// 且必须在获取该锁的同一线程上调用。
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ExitReadLock()
    {
        Interlocked.Decrement(ref readers[GetReaderSlot()].Value);
    }

    /// <summary>
    /// 释放当前线程持有的写锁。
    /// </summary>
    /// <remarks>
    /// 成功调用 <see cref="EnterWriteLock"/> 后必须恰好调用一次，
    /// 且必须在获取该锁的同一线程上调用。
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void ExitWriteLock()
    {
        Volatile.Write(ref writer, 0);
    }
}
