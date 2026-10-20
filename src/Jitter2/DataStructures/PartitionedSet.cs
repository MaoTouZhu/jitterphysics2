/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace Jitter2.DataStructures;

/// <summary>
/// 定义 <see cref="PartitionedSet{T}"/> 用于跟踪元素位置的索引属性。
/// </summary>
public interface IPartitionedSetIndex
{
    /// <summary>
    /// 获取或设置元素在 <see cref="PartitionedSet{T}"/> 中的索引。
    /// 值为 -1 表示该元素不属于任何集合。
    /// </summary>
    int SetIndex { get; set; }
}

/// <summary>
/// <see cref="PartitionedSet{T}"/> 的只读包装器。
/// </summary>
/// <typeparam name="T">集合中元素的类型。</typeparam>
public readonly struct ReadOnlyPartitionedSet<T>(PartitionedSet<T> partitionedSet) : IEnumerable<T>
    where T : class, IPartitionedSetIndex
{
    /// <summary>获取活动元素的数量。</summary>
    public int ActiveCount => partitionedSet.ActiveCount;

    /// <summary>获取元素总数。</summary>
    public int Count => partitionedSet.Count;

    /// <summary>
    /// 返回集合中所有元素的只读跨度。
    /// </summary>
    public ReadOnlySpan<T> Elements => partitionedSet.Elements;

    /// <summary>
    /// 返回集合中活动元素的只读跨度。
    /// </summary>
    public ReadOnlySpan<T> Active => partitionedSet.Active;

    /// <summary>
    /// 返回集合中非活动元素的只读跨度。
    /// </summary>
    public ReadOnlySpan<T> Inactive => partitionedSet.Inactive;

    /// <summary>获取指定索引处的元素。</summary>
    public T this[int i] => partitionedSet[i];

    /// <summary>确定集合是否包含指定元素。</summary>
    public bool Contains(T element) => partitionedSet.Contains(element);

    /// <summary>确定指定元素是否位于活动分区中。</summary>
    public bool IsActive(T element) => partitionedSet.IsActive(element);

    public PartitionedSet<T>.Enumerator GetEnumerator()
    {
        return new PartitionedSet<T>.Enumerator(partitionedSet);
    }

    IEnumerator<T> IEnumerable<T>.GetEnumerator()
    {
        return GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }
}

/// <summary>
/// 表示一个可划分为活动与非活动子集的对象的集合。
/// </summary>
/// <typeparam name="T">集合中元素的类型，必须实现 <see cref="IPartitionedSetIndex"/>。</typeparam>
/// <remarks>
/// 方法 <see cref="Add(T, bool)"/>、<see cref="Remove(T)"/>、<see cref="Contains(T)"/>、<see cref="IsActive(T)"/>、
/// <see cref="MoveToActive(T)"/> 和 <see cref="MoveToInactive(T)"/> 的时间复杂度均为 O(1)，
/// 但当底层数组扩容时 <see cref="Add(T, bool)"/> 可能耗时 O(n)。
/// </remarks>
public class PartitionedSet<T> : IEnumerable<T> where T : class, IPartitionedSetIndex
{
    public struct Enumerator(PartitionedSet<T> partitionedSet) : IEnumerator<T>
    {
        private readonly int version = partitionedSet.version;
        private int index = -1;

        public readonly T Current => (index >= 0 ? partitionedSet[index] : null)!;

        readonly object IEnumerator.Current => Current;

        public readonly void Dispose()
        {
        }

        public bool MoveNext()
        {
            ThrowIfModified();

            if (index < partitionedSet.Count - 1)
            {
                index++;
                return true;
            }

            return false;
        }

        public void Reset()
        {
            ThrowIfModified();
            index = -1;
        }

        private readonly void ThrowIfModified()
        {
            if (version != partitionedSet.version)
                throw new InvalidOperationException("The set was modified during enumeration. Enumerate a copy to change it while looping.");
        }
    }

    private readonly int initialSize;
    private T[] elements;
    private int version;

    /// <summary>获取集合中活动元素的数量。</summary>
    public int ActiveCount { get; private set; }

    /// <summary>
    /// 初始化 <see cref="PartitionedSet{T}"/> 类的新实例。
    /// </summary>
    /// <param name="initialSize">内部数组的初始容量。</param>
    public PartitionedSet(int initialSize = 1024)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(initialSize, 1);

        this.initialSize = initialSize;
        elements = new T[initialSize];
    }

    /// <summary>获取集合在下一次扩容前可容纳的元素数量。</summary>
    public int Capacity => elements.Length;

    /// <summary>获取指定索引处的元素。</summary>
    public T this[int i] => elements[i];

    /// <summary>返回集合中所有元素的跨度。</summary>
    public ReadOnlySpan<T> Elements => elements.AsSpan()[..Count];

    /// <summary>返回集合中活动元素的跨度。</summary>
    public ReadOnlySpan<T> Active => elements.AsSpan()[..ActiveCount];

    /// <summary>返回集合中非活动元素的跨度。</summary>
    public ReadOnlySpan<T> Inactive => elements.AsSpan()[ActiveCount..Count];

    /// <summary>从集合中移除所有元素。</summary>
    public void Clear()
    {
        for (int i = 0; i < Count; i++)
        {
            elements[i].SetIndex = -1;
            elements[i] = null!;
        }

        Count = 0;
        ActiveCount = 0;
        version++;
    }

    /// <summary>获取集合中元素的总数。</summary>
    public int Count { get; private set; }

    /// <summary>
    /// 将内部存储收缩到能容纳当前元素的最小保留容量。
    /// </summary>
    /// <returns>如果集合被调整了大小，则为 <see langword="true"/>；否则为 <see langword="false"/>。</returns>
    public bool Trim()
    {
        int newSize = initialSize;

        while (newSize < Count)
        {
            newSize = checked(newSize * 2);
        }

        if (newSize >= elements.Length) return false;

        Array.Resize(ref elements, newSize);
        return true;
    }

    /// <summary>返回集合中所有元素的跨度。</summary>
    public Span<T> AsSpan() => this.elements.AsSpan(0, Count);

    /// <summary>
    /// 向集合中添加一个元素。
    /// </summary>
    /// <param name="element">要添加的元素。</param>
    /// <param name="active">如果为 <see langword="true"/>，则将该元素添加到活动分区。</param>
    public void Add(T element, bool active = false)
    {
        Debug.Assert(element.SetIndex == -1);

        if (Count == elements.Length)
        {
            Array.Resize(ref elements, elements.Length * 2);
        }

        element.SetIndex = Count;
        elements[Count++] = element;
        version++;

        if (active) MoveToActive(element);
    }

    private void Swap(int index0, int index1)
    {
        (elements[index0], elements[index1]) =
            (elements[index1], elements[index0]);

        elements[index0].SetIndex = index0;
        elements[index1].SetIndex = index1;
    }

    /// <summary>
    /// 确定指定元素是否位于活动分区中。
    /// </summary>
    /// <param name="element">要检查的元素。</param>
    /// <returns>如果该元素处于活动状态，则为 <see langword="true"/>；否则为 <see langword="false"/>。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public bool IsActive(T element)
    {
        Debug.Assert(element.SetIndex != -1);
        Debug.Assert(elements[element.SetIndex] == element);

        return (element.SetIndex < ActiveCount);
    }

    /// <summary>
    /// 将元素移动到活动分区。
    /// </summary>
    /// <param name="element">要移动的元素。</param>
    /// <returns>如果元素被移动，则为 <see langword="true"/>；如果它已经处于活动状态，则为 <see langword="false"/>。</returns>
    public bool MoveToActive(T element)
    {
        Debug.Assert(element.SetIndex != -1);
        Debug.Assert(elements[element.SetIndex] == element);

        int index = element.SetIndex;
        if (index < ActiveCount) return false;
        if (index != ActiveCount) Swap(ActiveCount, index);
        ActiveCount += 1;
        version++;
        return true;
    }

    /// <summary>
    /// 将元素移动到非活动分区。
    /// </summary>
    /// <param name="element">要移动的元素。</param>
    /// <returns>如果元素被移动，则为 <see langword="true"/>；如果它已经处于非活动状态，则为 <see langword="false"/>。</returns>
    public bool MoveToInactive(T element)
    {
        Debug.Assert(element.SetIndex != -1);
        Debug.Assert(elements[element.SetIndex] == element);

        int index = element.SetIndex;
        if (index >= ActiveCount) return false;
        ActiveCount -= 1;
        if (index != ActiveCount) Swap(ActiveCount, index);
        version++;
        return true;
    }

    /// <summary>
    /// 确定集合是否包含指定元素。
    /// </summary>
    /// <param name="element">要查找的元素。</param>
    /// <returns>如果找到该元素，则为 <see langword="true"/>；否则为 <see langword="false"/>。</returns>
    public bool Contains(T element)
    {
        if (element.SetIndex >= Count || element.SetIndex < 0) return false;
        return (elements[element.SetIndex] == element);
    }

    /// <summary>
    /// 从集合中移除指定元素。
    /// </summary>
    /// <param name="element">要移除的元素。</param>
    public void Remove(T element)
    {
        Debug.Assert(element.SetIndex != -1);
        Debug.Assert(elements[element.SetIndex] == element);

        MoveToInactive(element);

        int index = element.SetIndex;
        int lastIndex = Count - 1;

        Count = lastIndex;

        if (index != lastIndex)
        {
            T moved = elements[lastIndex];
            elements[index] = moved;
            moved.SetIndex = index;
        }

        elements[lastIndex] = null!;

        element.SetIndex = -1;
        version++;
    }

    public Enumerator GetEnumerator()
    {
        return new Enumerator(this);
    }

    IEnumerator<T> IEnumerable<T>.GetEnumerator()
    {
        return GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return GetEnumerator();
    }
}
