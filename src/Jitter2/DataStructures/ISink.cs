/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

namespace Jitter2.DataStructures;

using System;
using System.Collections.Generic;

/// <summary>
/// 算法所产生值的只追加目标。为热路径设计，是
/// <see cref="ICollection{T}"/> 的极简、对结构体友好的替代方案。
/// </summary>
/// <typeparam name="T">接收器接受的值类型。</typeparam>
public interface ISink<T>
{
    /// <summary>向接收器追加一个值。</summary>
    void Add(in T item);
}

/// <summary>
/// 将 <see cref="ICollection{T}"/> 适配为 <see cref="ISink{T}"/> 接口。
/// </summary>
/// <typeparam name="T">元素类型。</typeparam>
public readonly struct CollectionSink<T> : ISink<T>
{
    private readonly ICollection<T> collection;

    /// <summary>创建一个转发到指定集合的接收器。</summary>
    /// <exception cref="ArgumentNullException">当 <paramref name="collection"/> 为 null 时抛出。</exception>
    public CollectionSink(ICollection<T> collection)
    {
        this.collection = collection ?? throw new ArgumentNullException(nameof(collection));
    }

    /// <summary>向所包装的集合追加一个值。</summary>
    public void Add(in T item)
    {
        collection.Add(item);
    }
}
