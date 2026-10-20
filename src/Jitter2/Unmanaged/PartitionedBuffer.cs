/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Jitter2.Parallelization;

namespace Jitter2.Unmanaged;

/// <summary>
/// 存储在 <see cref="PartitionedBuffer{T}"/> 中的非托管对象的句柄。
/// 即使底层内存被调整大小，该句柄也保持稳定。
/// </summary>
/// <typeparam name="T">数据的非托管类型。</typeparam>
/// <remarks>
/// <para>
/// <b>有效性：</b>只有在所属的 <see cref="PartitionedBuffer{T}"/>
/// 仍然存在且该元素尚未通过 <see cref="PartitionedBuffer{T}.Free"/> 释放时，句柄才有效。
/// </para>
/// <para>
/// <b>不要缓存</b>可能调整缓冲区大小的操作之间的 <see cref="Data"/> 引用。
/// 需要时使用句柄获取新的引用。
/// </para>
/// </remarks>
public readonly unsafe struct JHandle<T> : IEquatable<JHandle<T>> where T : unmanaged
{
    /// <summary>
    /// 表示空/无效引用的句柄。
    /// </summary>
    public static readonly JHandle<T> Zero = new(null);

    internal readonly T** Pointer;

    /// <summary>
    /// 获取对底层数据的引用。
    /// </summary>
    public ref T Data => ref Unsafe.AsRef<T>(*Pointer);

    internal JHandle(T** ptr)
    {
        Pointer = ptr;
    }

    /// <summary>
    /// 获取一个值，指示这是否为零句柄。已释放的句柄可能仍然非零；
    /// 该属性并不表示所引用的元素仍然有效。
    /// </summary>
    public readonly bool IsZero => Pointer == null;

    /// <summary>
    /// 将句柄重新解释为指向另一种类型的句柄。两种类型必须具有兼容的布局。
    /// </summary>
    /// <typeparam name="TConvert">要重新解释为的目标非托管类型。</typeparam>
    /// <param name="handle">要重新解释的句柄。</param>
    /// <returns>重新解释为目标类型的句柄。</returns>
    /// <remarks>
    /// <b>安全性：</b>调用方必须确保 <typeparamref name="T"/> 和 <typeparamref name="TConvert"/>
    /// 具有兼容的内存布局。不会执行任何运行时验证。
    /// </remarks>
    public static JHandle<TConvert> AsHandle<TConvert>(JHandle<T> handle) where TConvert : unmanaged
    {
        return new JHandle<TConvert>((TConvert**)handle.Pointer);
    }

    public readonly bool Equals(JHandle<T> other)
    {
        return Pointer == other.Pointer;
    }

    public readonly override bool Equals(object? obj)
    {
        return obj is JHandle<T> other && Equals(other);
    }

    public readonly override int GetHashCode()
    {
        return ((nint)Pointer).GetHashCode();
    }

    public static bool operator ==(JHandle<T> left, JHandle<T> right)
    {
        return left.Equals(right);
    }

    public static bool operator !=(JHandle<T> left, JHandle<T> right)
    {
        return !(left == right);
    }
}

/// <summary>
/// 管理非托管结构体的内存，将它们依次存储在连续的内存块中。
/// 每个结构体可以处于活动状态或非活动状态。
/// </summary>
/// <typeparam name="T">要存储的非托管类型。大小必须至少为 4 字节。</typeparam>
/// <remarks>
/// <para>
/// <b>内存布局要求：</b>类型 <typeparamref name="T"/> 必须为其前
/// 4 字节（sizeof(int)）预留用于内部记录。该内存区域用于存储
/// 将数据映射回其 <see cref="JHandle{T}"/> 的稳定 ID。
/// </para>
/// <para>
/// 不要手动修改这些字节。兼容的结构体应如下所示：
/// <code>
/// [StructLayout(LayoutKind.Sequential)]
/// public struct RigidBodyData
/// {
///     private readonly int _internalIndex; // 由 PartitionedBuffer 预留
///
///     public JVector Position;
///     public JQuaternion Orientation;
///     // ... 其他字段
/// }
/// </code>
/// </para>
/// <para>
/// <b>线程：</b>调用方必须串行化所有变更操作，包括调用 <see cref="Allocate"/>。
/// 使用 <see cref="ResizeLock"/> 在分配可能调整缓冲区大小时保护读取方；
/// 仅靠调整大小锁并不能使并发分配变得安全。
/// </para>
/// <para>
/// <b>释放：</b>此类拥有非托管内存，必须释放以避免内存泄漏。
/// </para>
/// </remarks>
public sealed unsafe class PartitionedBuffer<T> : IDisposable where T : unmanaged
{
    /// <summary>
    /// 当缓冲区的内部间接表达到最大容量时引发的异常。
    /// </summary>
    public class MaximumSizeException : Exception
    {
        public MaximumSizeException() { }
        public MaximumSizeException(string message) : base(message) { }
        public MaximumSizeException(string message, Exception inner) : base(message, inner) { }
    }

    // 间接表的分页常量
    private const int PageSize = 4096;
    private const int PageMask = PageSize - 1;
    private const int PageShift = 12; // 2^12 = 4096
    private const int MaxPages = 1024 * 16; // 可容纳约 6700 万个句柄
    private const int MaximumCapacity = MaxPages * PageSize;

    private T* memory;
    private T*** pages; // 指向各页的指针数组（间接表）

    private int activeCount;
    private int size;
    private int pageCount;
    private bool disposed;

    /// <summary>
    /// 读写锁。发生调整大小时由写入方加锁。
    /// 调整大小会移动连续数据的内存地址。如果存在对 Allocate 的并发调用，
    /// 请使用读取锁访问数据。
    /// </summary>
    public readonly ReaderWriterLock ResizeLock = new();

    /// <summary>
    /// 获取缓冲区中已分配元素的数量。
    /// </summary>
    public int Count { get; private set; }

    /// <summary>
    /// 指示已分配的内存是否对齐到 64 字节边界。
    /// </summary>
    public bool Aligned64 { get; }

    /// <summary>
    /// 初始化该类的新实例。
    /// </summary>
    /// <param name="initialSize">连续内存块的初始大小。</param>
    /// <param name="aligned64">指示内存是否应按照 64 字节对齐。</param>
    /// <exception cref="ArgumentException">
    /// 当 <typeparamref name="T"/> 太小而无法存储内部元素 ID 时引发。
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="initialSize"/> 小于 1 或超过最大容量时引发。
    /// </exception>
    public PartitionedBuffer(int initialSize = 1024, bool aligned64 = false)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(initialSize, 1);
        ArgumentOutOfRangeException.ThrowIfGreaterThan(initialSize, MaximumCapacity);

        if (sizeof(T) < sizeof(int))
        {
            throw new ArgumentException($"Type {typeof(T).Name} is too small. It must be at least {sizeof(int)} bytes to store the internal ID.");
        }

        size = initialSize;

        // 分配页指针的主列表
        pages = (T***)MemoryHelper.AllocateHeap(MaxPages * sizeof(IntPtr));

        if (aligned64)
        {
            try { memory = MemoryHelper.AlignedAllocateHeap<T>(size, 64); }
            catch
            {
                Logger.Warning("Could not allocate aligned memory. Falling back to unaligned memory.");
                aligned64 = false;
            }
        }

        if (!aligned64)
        {
            memory = MemoryHelper.AllocateHeap<T>(size);
        }

        this.Aligned64 = aligned64;

        EnsureHandleCapacity(size);

        // 初始化内存中的 ID 槽位
        for (int i = 0; i < size; i++)
        {
            Unsafe.AsRef<int>(&memory[i]) = i;
        }
    }

    /// <summary>
    /// 返回已分配的非托管内存总量（以字节为单位，数据 + 间接页 + 主页表）。
    /// </summary>
    public long TotalBytesAllocated => (long)size * sizeof(T) + (long)pageCount * PageSize * sizeof(IntPtr) + MaxPages * sizeof(IntPtr);

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private T** GetHandleSlot(int id)
    {
        return &pages[id >> PageShift][id & PageMask];
    }

    private void EnsureHandleCapacity(int requiredCount)
    {
        while (pageCount * PageSize < requiredCount)
        {
            if (pageCount >= MaxPages)
                throw new MaximumSizeException("Internal indirection table limit reached.");

            pages[pageCount] = (T**)MemoryHelper.AllocateHeap(PageSize * sizeof(IntPtr));
            pageCount++;
        }
    }

    /// <summary>
    /// 从缓冲区中移除关联的非托管元素，并使该句柄失效。
    /// </summary>
    /// <param name="handle">要释放的句柄。</param>
    /// <remarks>
    /// <b>安全性：</b>调用此方法后，句柄将失效。
    /// 不要使用该句柄或其数据的任何缓存引用。
    /// </remarks>
    public void Free(JHandle<T> handle)
    {
        Debug.Assert(!disposed);

        T* element = *handle.Pointer;
        int index = (int)(element - memory);
        int lastIndex = Count - 1;
        int freedId = Unsafe.Read<int>(element);

        Debug.Assert((uint)index < (uint)Count);

        // 只有存活下来的元素需要保留。先从活动边界处填补活动空洞，
        // 再用最后一个元素填补由此产生的非活动空洞。
        if (index < activeCount)
        {
            int lastActiveIndex = activeCount - 1;

            if (index != lastActiveIndex)
            {
                memory[index] = memory[lastActiveIndex];
                *GetHandleSlot(Unsafe.Read<int>(&memory[index])) = &memory[index];
            }

            activeCount = lastActiveIndex;

            if (lastActiveIndex != lastIndex)
            {
                memory[lastActiveIndex] = memory[lastIndex];
                *GetHandleSlot(Unsafe.Read<int>(&memory[lastActiveIndex])) = &memory[lastActiveIndex];
            }
        }
        else if (index != lastIndex)
        {
            memory[index] = memory[lastIndex];
            *GetHandleSlot(Unsafe.Read<int>(&memory[index])) = &memory[index];
        }

        Count = lastIndex;
        Unsafe.AsRef<int>(&memory[lastIndex]) = freedId;
        *handle.Pointer = &memory[lastIndex];
    }

    /// <summary>
    /// 所有标记为活动的元素的 span。
    /// </summary>
    /// <remarks>
    /// <b>不要缓存：</b>缓冲区调整大小时，此 span 将失效。
    /// </remarks>
    public Span<T> Active => new(memory, activeCount);

    /// <summary>
    /// 所有标记为非活动的元素的 span。
    /// </summary>
    /// <remarks>
    /// <b>不要缓存：</b>缓冲区调整大小时，此 span 将失效。
    /// </remarks>
    public Span<T> Inactive => new(&memory[activeCount], Count - activeCount);

    /// <summary>
    /// 所有元素的 span。
    /// </summary>
    /// <remarks>
    /// <b>不要缓存：</b>缓冲区调整大小时，此 span 将失效。
    /// </remarks>
    public Span<T> Elements => new(memory, Count);

    /// <summary>
    /// 返回对象的句柄。O(1) 操作。
    /// </summary>
    /// <param name="t">对缓冲区中元素的引用。</param>
    /// <returns>该元素的句柄。</returns>
    public JHandle<T> GetHandle(ref T t)
    {
        return new JHandle<T>(GetHandleSlot(Unsafe.Read<int>(Unsafe.AsPointer(ref t))));
    }

    /// <summary>
    /// 检查该元素是否作为活动元素存储。O(1)。
    /// </summary>
    /// <param name="handle">要检查的句柄。</param>
    /// <returns>如果元素处于活动状态则返回 <see langword="true"/>；否则返回 <see langword="false"/>。</returns>
    public bool IsActive(JHandle<T> handle)
    {
        Debug.Assert(*handle.Pointer - memory < Count);
        return (nint)(*handle.Pointer) - (nint)memory < activeCount * sizeof(T);
    }

    /// <summary>
    /// 将对象从非活动移动到活动。
    /// </summary>
    /// <param name="handle">要移动的元素的句柄。</param>
    public void MoveToActive(JHandle<T> handle)
    {
        Debug.Assert(*handle.Pointer - memory < Count);

        if ((nint)(*handle.Pointer) - (nint)memory < activeCount * sizeof(T)) return;

        (**handle.Pointer, memory[activeCount]) = (memory[activeCount], **handle.Pointer);

        *GetHandleSlot(Unsafe.Read<int>(*handle.Pointer)) = *handle.Pointer;
        *GetHandleSlot(Unsafe.Read<int>(&memory[activeCount])) = &memory[activeCount];

        activeCount += 1;
    }

    /// <summary>
    /// 将对象从活动移动到非活动。
    /// </summary>
    /// <param name="handle">要移动的元素的句柄。</param>
    public void MoveToInactive(JHandle<T> handle)
    {
        if ((nint)(*handle.Pointer) - (nint)memory >= activeCount * sizeof(T)) return;

        activeCount -= 1;
        (**handle.Pointer, memory[activeCount]) = (memory[activeCount], **handle.Pointer);

        *GetHandleSlot(Unsafe.Read<int>(*handle.Pointer)) = *handle.Pointer;
        *GetHandleSlot(Unsafe.Read<int>(&memory[activeCount])) = &memory[activeCount];
    }

    /// <summary>
    /// 根据索引交换两个条目，并相应地调整句柄。
    /// </summary>
    /// <param name="i">第一个元素的索引。</param>
    /// <param name="j">第二个元素的索引。</param>
    public void Swap(int i, int j)
    {
        (memory[i], memory[j]) = (memory[j], memory[i]);

        *GetHandleSlot(Unsafe.Read<int>(&memory[i])) = &memory[i];
        *GetHandleSlot(Unsafe.Read<int>(&memory[j])) = &memory[j];
    }

    /// <summary>
    /// 获取句柄所引用元素的索引。
    /// </summary>
    /// <param name="handle">要获取索引的句柄。</param>
    /// <returns>元素在缓冲区中的索引。</returns>
    public int GetIndex(JHandle<T> handle)
    {
        return (int)(((nint)(*handle.Pointer) - (nint)memory) / sizeof(T));
    }

    /// <summary>
    /// 分配一个非托管对象。容量会动态增长。
    /// </summary>
    /// <param name="active">如果为 <see langword="true"/>，则该元素被添加到活动分区。</param>
    /// <param name="clear">如果为 <see langword="true"/>，则将该元素的内存（不包括内部 ID）清零。</param>
    /// <returns>新分配元素的句柄。</returns>
    /// <remarks>
    /// <b>线程：</b>调用方必须将分配操作与其他变更操作串行化。此方法可能
    /// 调整缓冲区大小，从而移动所有数据；请使用 <see cref="ResizeLock"/> 保护并发读取方。
    /// </remarks>
    /// <exception cref="MaximumSizeException">
    /// 当达到内部间接表限制时引发。
    /// </exception>
    public JHandle<T> Allocate(bool active = false, bool clear = false)
    {
        Debug.Assert(!disposed);

        if (Count == size)
        {
            ResizeLock.EnterWriteLock();

            T* newMemory = null;
            bool published = false;

            try
            {
                int oldSize = size;
                if (size == MaximumCapacity)
                {
                    throw new MaximumSizeException("Internal indirection table limit reached.");
                }

                int newSize = Math.Min(checked(size * 2), MaximumCapacity);
                T* oldMemory = memory;

                if (Aligned64) newMemory = MemoryHelper.AlignedAllocateHeap<T>(newSize, 64);
                else newMemory = MemoryHelper.AllocateHeap<T>(newSize);

                // 在发布任何新的数据指针之前，确保句柄已就绪。
                EnsureHandleCapacity(newSize);

                for (int i = 0; i < oldSize; i++)
                {
                    newMemory[i] = oldMemory[i];
                }

                for (int i = oldSize; i < newSize; i++)
                {
                    Unsafe.AsRef<int>(&newMemory[i]) = i;
                }

                for (int i = 0; i < oldSize; i++)
                {
                    *GetHandleSlot(Unsafe.Read<int>(&newMemory[i])) = &newMemory[i];
                }

                memory = newMemory;
                size = newSize;
                published = true;

                if (Aligned64) MemoryHelper.AlignedFree(oldMemory);
                else MemoryHelper.Free(oldMemory);
            }
            finally
            {
                if (!published && newMemory != null)
                {
                    if (Aligned64) MemoryHelper.AlignedFree(newMemory);
                    else MemoryHelper.Free(newMemory);
                }

                ResizeLock.ExitWriteLock();
            }
        }

        int freeIndex = Count;
        int hdlId = Unsafe.Read<int>(&memory[freeIndex]);
        T** slot = GetHandleSlot(hdlId);

        int targetIndex = freeIndex;

        // 将新的活动元素直接放在活动边界处。从该位置被置换出去的
        // 非活动元素只需移动一次，即移入空闲槽位。
        if (active && activeCount != Count)
        {
            targetIndex = activeCount;
            memory[freeIndex] = memory[targetIndex];
            *GetHandleSlot(Unsafe.Read<int>(&memory[freeIndex])) = &memory[freeIndex];
            Unsafe.AsRef<int>(&memory[targetIndex]) = hdlId;
        }

        *slot = &memory[targetIndex];

        var handle = new JHandle<T>(slot);

        if (clear)
        {
            // 清零时跳过前 4 字节（ID）
            MemoryHelper.MemSet((byte*)*slot + sizeof(int), sizeof(T) - sizeof(int));
        }

        Count += 1;
        if (active) activeCount += 1;

        return handle;
    }

    private void FreeResources()
    {
        if (!disposed)
        {
            for (int i = 0; i < pageCount; i++)
            {
                MemoryHelper.Free(pages[i]);
            }
            MemoryHelper.Free(pages);
            pages = (T***)0;

            if (Aligned64) MemoryHelper.AlignedFree(memory);
            else MemoryHelper.Free(memory);
            memory = (T*)0;

            disposed = true;
        }
    }

    ~PartitionedBuffer() => FreeResources();

    public void Dispose()
    {
        FreeResources();
        GC.SuppressFinalize(this);
    }
}
