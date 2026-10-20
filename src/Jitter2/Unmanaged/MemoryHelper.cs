/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace Jitter2.Unmanaged;

/// <summary>
/// 提供用于分配和管理非托管内存的辅助方法。
/// </summary>
/// <remarks>
/// <para>
/// <b>安全性：</b>分配方法返回的所有指针都必须使用
/// 对应的 <see cref="Free(void*)"/> 或 <see cref="AlignedFree"/> 方法释放。
/// 否则会导致内存泄漏。
/// </para>
/// <para>
/// <b>线程：</b>这些方法是线程安全的，但调用方负责
/// 同步对已分配内存的访问。
/// </para>
/// </remarks>
public static unsafe class MemoryHelper
{
    /// <summary>
    /// 表示一个在内存中隔离的整数值，用于防止多个线程操作相邻数据时
    /// 发生伪共享。
    /// </summary>
    /// <remarks>
    /// 该结构体会填充到 132 字节（值两侧各 64 字节填充），
    /// 以确保 <see cref="Value"/> 字段位于自己的缓存行上。
    /// </remarks>
    [StructLayout(LayoutKind.Explicit, Size = 132)]
    public struct IsolatedInt
    {
        [FieldOffset(0)]
        private fixed byte pad0[64];

        [FieldOffset(64)]
        public int Value;

        [FieldOffset(68)]
        private fixed byte pad1[64];
    }

    /// <summary>
    /// 表示一个带填充的数组元素，用于让相邻整数值不位于同一缓存行。
    /// </summary>
    /// <remarks>
    /// 此类型适用于计数器数组，其中相邻的 <see cref="Value"/> 字段
    /// 不应共享缓存行。对于独立字段，请使用 <see cref="IsolatedInt"/>。
    /// </remarks>
    [StructLayout(LayoutKind.Explicit, Size = 128)]
    public struct PaddedInt
    {
        [FieldOffset(0)]
        public int Value;
    }

    /// <summary>
    /// 一个大小相当于六个 <see cref="Real"/> 类型实例的内存块。
    /// </summary>
    /// <remarks>
    /// 该结构体使用顺序布局和固定大小，以提供一致的布局。
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 6 * sizeof(Real))]
    public struct MemBlock6Real { }

    /// <summary>
    /// 一个大小相当于九个 <see cref="Real"/> 类型实例的内存块。
    /// </summary>
    /// <remarks>
    /// 该结构体使用顺序布局和固定大小，以提供一致的布局。
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 9 * sizeof(Real))]
    public struct MemBlock9Real { }

    /// <summary>
    /// 一个大小相当于十二个 <see cref="Real"/> 类型实例的内存块。
    /// </summary>
    /// <remarks>
    /// 该结构体使用顺序布局和固定大小，以提供一致的布局。
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 12 * sizeof(Real))]
    public struct MemBlock12Real { }

    /// <summary>
    /// 一个大小相当于十六个 <see cref="Real"/> 类型实例的内存块。
    /// </summary>
    /// <remarks>
    /// 该结构体使用顺序布局和固定大小，以提供一致的布局。
    /// </remarks>
    [StructLayout(LayoutKind.Sequential, Size = 16 * sizeof(Real))]
    public struct MemBlock16Real { }

    /// <summary>
    /// 为指定类型的数组分配一块非托管内存。
    /// </summary>
    /// <typeparam name="T">要为其分配内存的元素所属的非托管类型。</typeparam>
    /// <param name="num">要为其分配内存的元素数量。</param>
    /// <returns>指向所分配内存块的指针。</returns>
    /// <remarks>
    /// <b>安全性：</b>调用方必须使用 <see cref="Free{T}"/> 释放返回的指针。
    /// 该内存不会被初始化为零。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">当 <paramref name="num"/> 为负数时抛出。</exception>
    /// <exception cref="OverflowException">当请求的字节数超出本机地址空间时抛出。</exception>
    public static T* AllocateHeap<T>(int num) where T : unmanaged
    {
        ArgumentOutOfRangeException.ThrowIfNegative(num);
        nuint byteCount = checked((nuint)num * (nuint)sizeof(T));
        return (T*)NativeMemory.Alloc(byteCount);
    }

    /// <summary>
    /// 为指定类型的数组分配一块对齐的非托管内存。
    /// </summary>
    /// <typeparam name="T">要为其分配内存的元素所属的非托管类型。</typeparam>
    /// <param name="num">要为其分配内存的元素数量。</param>
    /// <param name="alignment">内存对齐字节数（必须是 2 的幂）。</param>
    /// <returns>指向所分配内存块的指针。</returns>
    /// <remarks>
    /// <b>安全性：</b>调用方必须使用 <see cref="AlignedFree"/> 释放返回的指针。
    /// 该内存不会被初始化为零。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="num"/> 为负数或 <paramref name="alignment"/> 不是 2 的正整数次幂时抛出。
    /// </exception>
    /// <exception cref="OverflowException">当请求的字节数超出本机地址空间时抛出。</exception>
    public static T* AlignedAllocateHeap<T>(int num, int alignment) where T : unmanaged
    {
        ArgumentOutOfRangeException.ThrowIfNegative(num);
        ValidateAlignment(alignment);
        nuint byteCount = checked((nuint)num * (nuint)sizeof(T));
        return (T*)NativeMemory.AlignedAlloc(byteCount, (nuint)alignment);
    }

    /// <summary>
    /// 释放先前为指定类型的数组分配的一块非托管内存。
    /// </summary>
    /// <typeparam name="T">内存块中元素的非托管类型。</typeparam>
    /// <param name="ptr">指向要释放的内存块的指针。</param>
    public static void Free<T>(T* ptr) where T : unmanaged
    {
        Free((void*)ptr);
    }

    /// <summary>
    /// 分配一块指定字节长度的非托管内存。
    /// </summary>
    /// <param name="len">要分配的内存块长度，以字节为单位。</param>
    /// <returns>指向所分配内存块的指针。</returns>
    /// <remarks>
    /// <b>安全性：</b>调用方必须使用 <see cref="Free(void*)"/> 释放返回的指针。
    /// 该内存不会被初始化为零。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">当 <paramref name="len"/> 为负数时抛出。</exception>
    public static void* AllocateHeap(int len)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(len);
        return NativeMemory.Alloc((nuint)len);
    }

    /// <summary>
    /// 分配一块指定字节长度的对齐非托管内存。
    /// </summary>
    /// <param name="len">要分配的内存块长度，以字节为单位。</param>
    /// <param name="alignment">内存对齐字节数（必须是 2 的幂）。</param>
    /// <returns>指向所分配内存块的指针。</returns>
    /// <remarks>
    /// <b>安全性：</b>调用方必须使用 <see cref="AlignedFree"/> 释放返回的指针。
    /// 该内存不会被初始化为零。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="len"/> 为负数或 <paramref name="alignment"/> 不是 2 的正整数次幂时抛出。
    /// </exception>
    public static void* AlignedAllocateHeap(int len, int alignment)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(len);
        ValidateAlignment(alignment);
        return NativeMemory.AlignedAlloc((nuint)len, (nuint)alignment);
    }

    /// <summary>
    /// 释放先前分配的一块非托管内存。
    /// </summary>
    /// <param name="ptr">指向要释放的内存块的指针。</param>
    public static void Free(void* ptr) => NativeMemory.Free(ptr);

    /// <summary>
    /// 释放先前分配的一块对齐非托管内存。
    /// </summary>
    /// <param name="ptr">指向要释放的对齐内存块的指针。</param>
    public static void AlignedFree(void* ptr) => NativeMemory.AlignedFree(ptr);

    /// <summary>
    /// 将非托管内存清零。
    /// </summary>
    /// <param name="buffer">指向要清零的内存块的指针。</param>
    /// <param name="len">要清零的内存块长度，以字节为单位。</param>
    /// <exception cref="ArgumentOutOfRangeException">当 <paramref name="len"/> 为负数时抛出。</exception>
    public static void MemSet(void* buffer, int len)
    {
        ArgumentOutOfRangeException.ThrowIfNegative(len);
        Unsafe.InitBlockUnaligned(buffer, 0, (uint)len);
    }

    private static void ValidateAlignment(int alignment)
    {
        if (alignment <= 0 || (alignment & (alignment - 1)) != 0)
        {
            throw new ArgumentOutOfRangeException(nameof(alignment), alignment,
                "Alignment must be a positive power of two.");
        }
    }
}
