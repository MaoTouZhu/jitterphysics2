/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections;
using System.Collections.Generic;

namespace Jitter2.DataStructures;

/// <summary>
/// 一个围绕 <see cref="List{T}"/> 的只读包装器，在允许索引访问和枚举的同时阻止修改。
/// </summary>
/// <typeparam name="T">列表中元素的类型。</typeparam>
public readonly struct ReadOnlyList<T>(List<T> list) : IReadOnlyList<T>
{
    /// <summary>获取指定索引处的元素。</summary>
    public T this[int i] => list[i];

    public List<T>.Enumerator GetEnumerator()
    {
        return list.GetEnumerator();
    }

    IEnumerator<T> IEnumerable<T>.GetEnumerator()
    {
        return list.GetEnumerator();
    }

    IEnumerator IEnumerable.GetEnumerator()
    {
        return list.GetEnumerator();
    }

    /// <summary>获取列表中的元素数量。</summary>
    public int Count => list.Count;
}
