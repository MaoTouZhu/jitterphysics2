/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;

namespace Jitter2.DataStructures;

/// <summary>
/// 提供将集合转换为 span 的实用方法。
/// </summary>
internal static class SpanHelper
{
    /// <summary>
    /// 将可枚举对象转换为 <see cref="ReadOnlySpan{T}"/>，尽可能使用零拷贝。
    /// </summary>
    /// <typeparam name="T">非托管元素类型。</typeparam>
    /// <param name="elements">要转换的集合。</param>
    /// <param name="backingArray">
    /// 当需要拷贝时，包含已分配的支撑数组；否则为 <see langword="null"/>。
    /// </param>
    /// <returns>元素上的只读 span。</returns>
    /// <remarks>
    /// 对 <see cref="Array"/> 和 <see cref="List{T}"/> 使用零拷贝路径。
    /// 其他集合类型需要分配。
    /// </remarks>
    public static ReadOnlySpan<T> AsReadOnlySpan<T>(IEnumerable<T> elements, out T[]? backingArray) where T : struct
    {
        ArgumentNullException.ThrowIfNull(elements);

        backingArray = null;

        switch (elements)
        {
            // 1. 数组：零拷贝，直接返回。
            case T[] array:
                return array;

            // 2. List<T>：通过 Marshal 进行零拷贝。
            case List<T> list:
                return CollectionsMarshal.AsSpan(list);

            // 3. ICollection<T>：快速拷贝（Count 已知）。
            // 涵盖 HashSet、LinkedList、Queue、Stack 等。
            case ICollection<T> collection:
                backingArray = new T[collection.Count];
                collection.CopyTo(backingArray, 0);
                return backingArray;

            // 4. IReadOnlyCollection<T>：优化分配（Count 已知）。
            // 涵盖仅公开只读接口的自定义集合。
            case IReadOnlyCollection<T> readOnlyCol:
                backingArray = new T[readOnlyCol.Count];
                int i = 0;
                foreach (var item in readOnlyCol)
                {
                    backingArray[i++] = item;
                }

                return backingArray;

            // 5. 通用回退：需要迭代（大小未知）。
            default:
                var buffer = new List<T>();

                foreach (var item in elements)
                {
                    buffer.Add(item);
                }

                backingArray = buffer.ToArray();
                return backingArray;
        }
    }

}
