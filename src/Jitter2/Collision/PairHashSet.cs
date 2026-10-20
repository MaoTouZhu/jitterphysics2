/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;

namespace Jitter2.Collision;

/// <summary>
/// 一种用于存储 (int, int) 无序对的哈希集合实现。
/// 该实现基于开放寻址，容量为 2 的幂。
/// </summary>
/// <remarks>
/// <para>
/// 对以规范形式存储，较小的 ID 在前。
/// <see cref="Pair.ID"/> 等于零的对被视为空槽。
/// </para>
/// <para>
/// 此类通常不是线程安全的。只有 <see cref="ConcurrentAdd"/> 可以被
/// 多个线程同时调用。所有其他方法都需要外部同步。
/// </para>
/// </remarks>
internal unsafe class PairHashSet : IEnumerable<PairHashSet.Pair>
{
    /// <summary>
    /// 表示以规范形式存储的整数 ID 无序对。
    /// </summary>
    [StructLayout(LayoutKind.Explicit, Size = 8)]
    public readonly struct Pair
    {
        /// <summary>该对的组合 64 位标识符。</summary>
        [FieldOffset(0)] public readonly long ID;

        /// <summary>两个 ID 中较小的一个。</summary>
        [FieldOffset(0)] public readonly int ID1;

        /// <summary>两个 ID 中较大的一个。</summary>
        [FieldOffset(4)] public readonly int ID2;

        /// <summary>表示空槽的零对。</summary>
        public static readonly Pair Zero = new();

        /// <summary>
        /// 由两个 ID 创建一对，并按规范顺序存储。
        /// </summary>
        /// <param name="id1">第一个 ID。</param>
        /// <param name="id2">第二个 ID。</param>
        public Pair(int id1, int id2)
        {
            if (id1 < id2)
            {
                (ID1, ID2) = (id1, id2);
            }
            else
            {
                (ID1, ID2) = (id2, id1);
            }
        }

        /// <summary>
        /// 计算该对的哈希码。
        /// </summary>
        /// <returns>哈希码。</returns>
        public int GetHash()
        {
            return HashCode.Combine(ID1, ID2);
        }
    }

    /// <summary>
    /// 枚举哈希集合中的非空对。
    /// </summary>
    public struct Enumerator(PairHashSet hashSet) : IEnumerator<Pair>
    {
        private int index = -1;

        /// <inheritdoc/>
        public readonly Pair Current => hashSet.Slots[index];

        readonly object IEnumerator.Current => Current;

        /// <inheritdoc/>
        public readonly void Dispose()
        {
        }

        /// <inheritdoc/>
        public bool MoveNext()
        {
            var slots = hashSet.Slots;

            while (index < slots.Length - 1)
            {
                if (slots[++index].ID != 0) return true;
            }

            return false;
        }

        /// <inheritdoc/>
        public void Reset()
        {
            index = -1;
        }
    }

    /// <summary>
    /// 对的内部存储数组。空槽的 <see cref="Pair.ID"/> 等于零。
    /// </summary>
    public Pair[] Slots = [];
    private int count;

    /// <summary>
    /// 哈希集合中的最小槽数（每槽 8 字节时为 128 KB）。
    /// </summary>
    public const int MinimumSize = 16384;

    /// <summary>
    /// 用于确定何时收缩哈希集合的因子。
    /// </summary>
    public const int TrimFactor = 8;

    /// <summary>
    /// 用于在收缩哈希集合后保留增长余量的因子。
    /// </summary>
    public const int TrimTargetFactor = 4;

    /// <summary>
    /// 获取哈希集合中的对数量。
    /// </summary>
    public int Count => count;

    private static int PickSize(int size = -1)
    {
        int p2 = MinimumSize;
        while (p2 < size)
        {
            p2 *= 2;
        }

        return p2;
    }

    /// <summary>
    /// 从哈希集合中移除所有对。
    /// </summary>
    /// <remarks>
    /// 此方法不是线程安全的。不要与任何其他方法并发调用。
    /// </remarks>
    public void Clear()
    {
        Array.Clear(Slots, 0, Slots.Length);
        count = 0;
    }

    /// <summary>
    /// 初始化 <see cref="PairHashSet"/> 类的新实例。
    /// </summary>
    public PairHashSet()
    {
        Resize(PickSize());
    }

    private void Resize(int size)
    {
        if (Slots.Length == size) return;

        Logger.Information("{0}: Resizing from {1} to {2} elements.", nameof(PairHashSet), Slots.Length, size);

        var newSlots = new Pair[size];

        foreach (var pair in Slots)
        {
            if (pair.ID != 0)
            {
                int hash = pair.GetHash();
                int hashIndex = FindSlot(newSlots, hash, pair.ID);
                newSlots[hashIndex] = pair;
            }
        }

        Volatile.Write(ref Slots, newSlots);
    }

    private static int FindSlot(Pair[] slots, int hash, long id)
    {
        int modder = slots.Length - 1;

        hash &= modder;

        while (true)
        {
            if (slots[hash].ID == 0 || slots[hash].ID == id) return hash;
            hash = (hash + 1) & modder;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static long ReadConcurrentId(Pair[] slots, int index)
    {
        // ConcurrentAdd 通过原子 64 位比较交换发布 ID。原生
        // 64 位加载是原子的，但普通的 long 读取在 32 位进程中可能发生撕裂。
        if (Environment.Is64BitProcess) return slots[index].ID;
        return Interlocked.Read(ref Unsafe.As<Pair, long>(ref slots[index]));
    }

    private static int FindSlotConcurrent(Pair[] slots, int hash, long id, out long slotId)
    {
        int modder = slots.Length - 1;

        hash &= modder;

        while (true)
        {
            slotId = ReadConcurrentId(slots, hash);
            if (slotId == 0 || slotId == id) return hash;
            hash = (hash + 1) & modder;
        }
    }

    /// <summary>
    /// 检查哈希集合是否包含指定的对。
    /// </summary>
    /// <param name="pair">要检查的对。</param>
    /// <returns>如果该对存在，则为 <c>true</c>；否则为 <c>false</c>。</returns>
    public bool Contains(Pair pair)
    {
        int hash = pair.GetHash();
        int hashIndex = FindSlot(Slots, hash, pair.ID);
        return Slots[hashIndex].ID != 0;
    }

    /// <summary>
    /// 向哈希集合中添加一个对。
    /// </summary>
    /// <param name="pair">要添加的对。</param>
    /// <returns>如果该对被添加，则为 <c>true</c>；如果它已存在，则为 <c>false</c>。</returns>
    /// <remarks>
    /// 此方法不是线程安全的。
    /// </remarks>
    public bool Add(Pair pair)
    {
        int hash = pair.GetHash();
        int hashIndex = FindSlot(Slots, hash, pair.ID);

        if (Slots[hashIndex].ID == 0)
        {
            Slots[hashIndex] = pair;
            Interlocked.Increment(ref count);

            if (Slots.Length < 2 * count)
            {
                Resize(PickSize(Slots.Length * 2));
            }

            return true;
        }

        return false;
    }

    private readonly Jitter2.Parallelization.ReaderWriterLock rwLock = new();

    /// <summary>
    /// 以线程安全的方式尝试向哈希集合中添加一个对。
    /// </summary>
    /// <remarks>
    /// 多个线程可以并发调用 <see cref="ConcurrentAdd"/>。
    /// 但是，不得与此方法并发调用任何其他方法（包括 <see cref="Add"/>、<see cref="Remove(Pair)"/>、
    /// <see cref="Clear"/> 或枚举）。
    /// </remarks>
    /// <param name="pair">要添加的对。</param>
    /// <returns>如果该对被添加，则为 <c>true</c>；如果它已存在，则为 <c>false</c>。</returns>
    public bool ConcurrentAdd(Pair pair)
    {
        int hash = pair.GetHash();

        // 快速路径：在频繁添加已存在条目时，这是一个*巨大*的优化。
        // 完全绕过任何锁或同步。
        Pair[] fpSlots = Volatile.Read(ref Slots);
        _ = FindSlotConcurrent(fpSlots, hash, pair.ID, out long fpSlotId);
        if (fpSlotId == pair.ID) return false;

        rwLock.EnterReadLock();

        Pair[] slots = Slots;
        fixed (Pair* slotsPtr = slots)
        {
            while (true)
            {
                var hashIndex = FindSlotConcurrent(slots, hash, pair.ID, out long slotId);
                var slotPtr = &slotsPtr[hashIndex];

                if (slotId == pair.ID)
                {
                    rwLock.ExitReadLock();
                    return false;
                }

                if (Interlocked.CompareExchange(ref *(long*)slotPtr, pair.ID, 0) != 0)
                {
                    continue;
                }

                Interlocked.Increment(ref count);
                rwLock.ExitReadLock();

                Pair[] currentSlots = Volatile.Read(ref Slots);
                if (currentSlots.Length < 2 * count)
                {
                    rwLock.EnterWriteLock();

                    try
                    {
                        // 检查另一个线程是否已经执行过扩容。
                        if (Slots.Length < 2 * count)
                        {
                            Resize(PickSize(Slots.Length * 2));
                        }
                    }
                    finally
                    {
                        rwLock.ExitWriteLock();
                    }
                }

                return true;
            } // while
        } // fixed
    }

    /// <summary>
    /// 移除指定槽索引处的对。
    /// </summary>
    /// <param name="slot">槽索引。</param>
    /// <returns>如果移除了一个对，则为 <c>true</c>；如果该槽为空，则为 <c>false</c>。</returns>
    public bool Remove(int slot)
    {
        int modder = Slots.Length - 1;

        if (Slots[slot].ID == 0)
        {
            return false;
        }

        int hashJ = slot;

        while (true)
        {
            hashJ = (hashJ + 1) & modder;

            if (Slots[hashJ].ID == 0)
            {
                break;
            }

            int hashK = Slots[hashJ].GetHash() & modder;

            // https://en.wikipedia.org/wiki/Open_addressing
            if ((hashJ > slot && (hashK <= slot || hashK > hashJ)) ||
                (hashJ < slot && hashK <= slot && hashK > hashJ))
            {
                Slots[slot] = Slots[hashJ];
                slot = hashJ;
            }
        }

        Slots[slot] = Pair.Zero;
        Interlocked.Decrement(ref count);

        if (Slots.Length > MinimumSize && count * TrimFactor < Slots.Length)
        {
            Resize(PickSize(count * TrimTargetFactor));
        }

        return true;
    }

    /// <summary>
    /// 从哈希集合中移除指定的对。
    /// </summary>
    /// <param name="pair">要移除的对。</param>
    /// <returns>如果移除了该对，则为 <c>true</c>；如果未找到该对，则为 <c>false</c>。</returns>
    public bool Remove(Pair pair)
    {
        int hash = pair.GetHash();
        int hashIndex = FindSlot(Slots, hash, pair.ID);
        return Remove(hashIndex);
    }

    /// <inheritdoc/>
    public Enumerator GetEnumerator()
    {
        return new Enumerator(this);
    }

    IEnumerator<Pair> IEnumerable<Pair>.GetEnumerator()
    {
        return GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }
}
