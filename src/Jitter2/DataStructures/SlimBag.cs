/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Threading;

namespace Jitter2.DataStructures;

/// <summary>
/// 一个由数组支持的轻量级无序集合。移除位置 n 处的元素会将最后一个元素
/// 移到位置 n 并递减 <see cref="Count"/>，从而提供 O(1) 移除。
/// </summary>
/// <typeparam name="T">包中元素的类型。</typeparam>
/// <remarks>
/// 关键操作：
/// <list type="bullet">
///   <item><description><see cref="Add"/>：分摊 O(1)，可能调整内部数组的大小。</description></item>
///   <item><description><see cref="RemoveAt"/>：O(1)，与最后一个元素交换。</description></item>
///   <item><description><see cref="Remove"/>：O(n)，需要线性搜索。</description></item>
///   <item><description><see cref="ConcurrentAdd"/>：使用读写锁的线程安全添加。</description></item>
/// </list>
/// </remarks>
internal class SlimBag<T> : IEnumerable<T>
{
    /// <summary>
    /// 用于 <see cref="SlimBag{T}"/> 的轻量级结构体枚举器。
    /// </summary>
    /// <remarks>
    /// <b>警告：</b>如果在枚举期间修改了包，则不安全。
    /// </remarks>
    public struct Enumerator : IEnumerator<T>
    {
        private readonly SlimBag<T> owner;
        private int index;

        internal Enumerator(SlimBag<T> owner)
        {
            this.owner = owner;
            index = -1;
        }

        public T Current => owner.array[index];
        object? IEnumerator.Current => Current;

        public bool MoveNext()
        {
            // 前进并与有效项的数量比较，而不是 array.Length
            return ++index < owner.counter;
        }

        // 无需释放资源
        public void Dispose() { }

        public void Reset() => index = -1;
    }

    private readonly int initialSize;
    private T[] array;
    private int counter;
    private int nullOut;
    private readonly EqualityComparer<T> comparer = EqualityComparer<T>.Default;

    /// <summary>
    /// 使用指定的初始大小初始化 <see cref="SlimBag{T}"/> 类的新实例。
    /// </summary>
    /// <param name="initialSize">内部数组的初始大小。如果未指定，则默认为 4。</param>
    public SlimBag(int initialSize = 4)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(initialSize, 1);

        this.initialSize = initialSize;
        array = new T[initialSize];
        nullOut = 0;
    }

    /// <summary>
    /// 获取内部数组的长度。
    /// </summary>
    public int InternalSize => array.Length;

    /// <summary>
    /// 将内部存储缩小到能够容纳当前元素的最小保留容量。
    /// </summary>
    /// <returns>如果调整了包的大小，则为 <see langword="true"/>；否则为 <see langword="false"/>。</returns>
    public bool Trim()
    {
        int newSize = initialSize;

        while (newSize < counter)
        {
            newSize = checked(newSize * 2);
        }

        if (newSize >= array.Length) return false;

        Array.Resize(ref array, newSize);
        nullOut = int.Min(nullOut, counter);
        return true;
    }

    /// <summary>
    /// 返回内部数组有效元素上的 span。
    /// </summary>
    public Span<T> AsSpan()
    {
        return new Span<T>(array, 0, counter);
    }

    /// <summary>
    /// 向 <see cref="SlimBag{T}"/> 添加一系列元素。
    /// </summary>
    /// <param name="list">要添加的元素集合。</param>
    public void AddRange(IEnumerable<T> list)
    {
        ArgumentNullException.ThrowIfNull(list);

        if (list is ICollection<T> collection)
        {
            EnsureCapacity(checked(counter + collection.Count));
        }
        else if (list is IReadOnlyCollection<T> readOnlyCollection)
        {
            EnsureCapacity(checked(counter + readOnlyCollection.Count));
        }

        foreach (T elem in list) Add(elem);
    }

    /// <summary>
    /// 向 <see cref="SlimBag{T}"/> 添加一个元素。
    /// </summary>
    /// <param name="item">要添加的元素。</param>
    public void Add(T item)
    {
        EnsureCapacity(counter + 1);

        array[counter++] = item;
    }

    private void EnsureCapacity(int capacity)
    {
        if (capacity <= array.Length) return;

        int newLength = array.Length;
        do
        {
            newLength = checked(newLength * 2);
        }
        while (newLength < capacity);

        Array.Resize(ref array, newLength);
    }

    private readonly Jitter2.Parallelization.ReaderWriterLock rwLock = new();

    /// <summary>
    /// 以线程安全的方式向 <see cref="SlimBag{T}"/> 添加一个元素。
    /// </summary>
    /// <param name="item">要添加的元素。</param>
    public void ConcurrentAdd(T item)
    {
        int lc = Interlocked.Increment(ref counter) - 1;

    again:

        rwLock.EnterReadLock();

        if (lc < array.Length)
        {
            array[lc] = item;
            rwLock.ExitReadLock();
        }
        else
        {
            rwLock.ExitReadLock();

            rwLock.EnterWriteLock();
            if (lc >= array.Length)
            {
                Array.Resize(ref array, array.Length * 2);
            }
            rwLock.ExitWriteLock();

            goto again;
        }
    }

    /// <summary>
    /// 从 <see cref="SlimBag{T}"/> 中移除特定元素的第一个匹配项。
    /// </summary>
    /// <param name="item">要移除的元素。</param>
    public void Remove(T item)
    {
        int index = -1;
        for (int i = 0; i < counter; i++)
        {
            if (comparer.Equals(item, array[i]))
            {
                index = i;
                break;
            }
        }

        if (index != -1) RemoveAt(index);
    }

    /// <summary>
    /// 从 <see cref="SlimBag{T}"/> 中移除指定索引处的元素。
    /// </summary>
    /// <param name="index">要移除的元素的从零开始的索引。</param>
    public void RemoveAt(int index)
    {
        counter -= 1;
        if (index != counter) array[index] = array[counter];
    }

    /// <summary>
    /// 获取或设置 <see cref="SlimBag{T}"/> 中包含的元素数量。
    /// </summary>
    public int Count
    {
        get => counter;
        set
        {
            Debug.Assert(value <= counter);
            counter = value;
        }
    }

    /// <summary>
    /// 获取指定索引处的元素。注意，索引应小于
    /// <see cref="Count"/>，但这一点并未强制。
    /// </summary>
    /// <param name="i">要获取的元素的从零开始的索引。</param>
    /// <returns>指定索引处的元素。</returns>
    public T this[int i] => array[i];

    /// <summary>
    /// 从 <see cref="SlimBag{T}"/> 中移除所有元素。
    /// </summary>
    public void Clear()
    {
        counter = 0;
    }

    /// <summary>
    /// 每次调用将一个过时的数组槽置空，以便回收已移除元素的垃圾。
    /// </summary>
    /// <remarks>
    /// 更新观察到的 <see cref="Count"/> 高水位标记，并在当前计数低于该标记时
    /// 清除一个过时槽。请在包中仍有元素时、收缩之前调用此方法，
    /// 以便后续调用能够一次一个槽地清除先前使用的范围。
    /// </remarks>
    public void TrackAndNullOutOne()
    {
        nullOut = Math.Max(nullOut, counter);
        if (nullOut <= counter) return;
        array[--nullOut] = default!;
    }

    public Enumerator GetEnumerator() => new Enumerator(this);

    IEnumerator<T> IEnumerable<T>.GetEnumerator() => GetEnumerator();
    IEnumerator IEnumerable.GetEnumerator() => GetEnumerator();
}
