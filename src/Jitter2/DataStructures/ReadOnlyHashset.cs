/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections;
using System.Collections.Generic;

namespace Jitter2.DataStructures;

/// <summary>
/// <see cref="HashSet{T}"/> 的只读包装器，禁止修改，同时允许枚举和查找。
/// </summary>
/// <typeparam name="T">集合中元素的类型。</typeparam>
public readonly struct ReadOnlyHashSet<T>(HashSet<T> hashset) : IReadOnlyCollection<T>
{
    public HashSet<T>.Enumerator GetEnumerator()
    {
        return hashset.GetEnumerator();
    }

    IEnumerator<T> IEnumerable<T>.GetEnumerator()
    {
        return hashset.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return hashset.GetEnumerator();
    }

    /// <summary>确定集合是否包含指定元素。</summary>
    public bool Contains(T item) => hashset.Contains(item);

    /// <summary>将集合的元素复制到数组。</summary>
    public void CopyTo(T[] array) => hashset.CopyTo(array);

    /// <summary>将集合的元素复制到数组，从指定索引开始。</summary>
    public void CopyTo(T[] array, int arrayIndex) => hashset.CopyTo(array, arrayIndex);

    /// <summary>获取集合中元素的数量。</summary>
    public int Count => hashset.Count;
}
