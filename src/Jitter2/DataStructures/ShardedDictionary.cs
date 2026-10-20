/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;

#if NET9_0_OR_GREATER
using Lock = System.Threading.Lock;
#else
using Lock = System.Object;
#endif

namespace Jitter2.DataStructures;

/// <summary>
/// 一个分片字典，其条目被分区以减少锁争用。
/// 每个分片都有自己的锁，允许调用方独立同步不同分片。
/// </summary>
/// <typeparam name="TKey">字典中键的类型。</typeparam>
/// <typeparam name="TValue">字典中值的类型。</typeparam>
/// <remarks>
/// <para>
/// 此实现避免了 <see cref="System.Collections.Concurrent.ConcurrentDictionary{TKey,TValue}"/> 的 GC 开销。
/// </para>
/// <para>
/// <b>线程：</b>单个操作不是线程安全的。在并发写入期间，调用方必须先通过
/// <see cref="GetLock"/> 获取分片锁，然后再调用 <see cref="Add"/>、<see cref="Remove"/>、
/// <see cref="TryGetValue"/> 或索引器。<see cref="TrimExcess"/>
/// 要求排除所有并发操作。
/// </para>
/// </remarks>
internal class ShardedDictionary<TKey, TValue> where TKey : notnull
{
    private readonly Lock[] locks;
    private readonly Dictionary<TKey, TValue>[] dictionaries;

    private static int ShardSuggestion(int threads)
    {
        int[] primes = [3, 5, 7, 11, 17, 23, 29, 37, 47, 59, 71];

        foreach (int prime in primes)
        {
            if (prime >= threads)
                return prime;
        }

        return primes[^1];
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private int GetShardIndex(TKey key)
    {
        return (key.GetHashCode() & 0x7FFFFFFF) % locks.Length;
    }

    /// <summary>获取与指定键关联的值。</summary>
    public TValue this[TKey key]
    {
        get
        {
            int index = GetShardIndex(key);
            return dictionaries[index][key];
        }
    }

    /// <summary>
    /// 获取包含指定键的分片的锁对象。
    /// </summary>
    /// <param name="key">用于定位分片的键。</param>
    /// <returns>该分片的锁对象。</returns>
    public Lock GetLock(TKey key)
    {
        return locks[GetShardIndex(key)];
    }

    /// <summary>
    /// 初始化 <see cref="ShardedDictionary{TKey, TValue}"/> 类的新实例。
    /// </summary>
    /// <param name="threads">预期的并发线程数，用于确定分片数量。</param>
    public ShardedDictionary(int threads)
    {
        int count = ShardSuggestion(threads);

        locks = new Lock[count];
        dictionaries = new Dictionary<TKey, TValue>[count];

        for (int i = 0; i < count; i++)
        {
            locks[i] = new Lock();
            dictionaries[i] = new Dictionary<TKey, TValue>();
        }
    }

    /// <summary>
    /// 将每个分片的容量设置为其实际包含的条目数。
    /// </summary>
    public void TrimExcess()
    {
        for (int i = 0; i < dictionaries.Length; i++)
        {
            dictionaries[i].TrimExcess();
        }
    }

    /// <summary>
    /// 尝试获取与指定键关联的值。
    /// </summary>
    /// <param name="key">要定位的键。</param>
    /// <param name="value">如果找到，则为与该键关联的值；否则为默认值。</param>
    /// <returns>如果找到该键，则为 <see langword="true"/>；否则为 <see langword="false"/>。</returns>
    public bool TryGetValue(TKey key, [MaybeNullWhen(false)] out TValue value)
    {
        return dictionaries[GetShardIndex(key)].TryGetValue(key, out value);
    }

    /// <summary>
    /// 向字典添加一个键值对。
    /// </summary>
    /// <param name="key">要添加的键。</param>
    /// <param name="value">要与该键关联的值。</param>
    public void Add(TKey key, TValue value)
    {
        dictionaries[GetShardIndex(key)].Add(key, value);
    }

    /// <summary>
    /// 从字典中移除具有指定键的条目。
    /// </summary>
    /// <param name="key">要移除的键。</param>
    public void Remove(TKey key)
    {
        dictionaries[GetShardIndex(key)].Remove(key);
    }
}
