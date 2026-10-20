/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Numerics;
using System.Runtime.CompilerServices;

namespace Jitter2.LinearMath;

/// <summary>
/// JVector 与 JQuaternion 的互操作辅助方法。
/// 包含到 System.Numerics 类型的安全隐式转换，以及针对布局兼容结构体、
/// 用于高性能互操作的按需启用的非安全位重解释。
/// </summary>
internal static class UnsafeInterop
{
    [MethodImpl(MethodImplOptions.NoInlining)]
    public static void ThrowSizeMismatch<TTo, TFrom>()
        => throw new InvalidOperationException(
            $"UnsafeAs size mismatch: sizeof({typeof(TFrom).Name})={Unsafe.SizeOf<TFrom>()}, " +
            $"sizeof({typeof(TTo).Name})={Unsafe.SizeOf<TTo>()}.");

    [MethodImpl(MethodImplOptions.NoInlining)]
    public static void ThrowSizeMismatch(string from, int fromSize, string to, int toSize)
        => throw new InvalidOperationException(
            $"UnsafeAs size mismatch: sizeof({from})={fromSize}, sizeof({to})={toSize}.");
}

public partial struct JVector
{
    /// <summary>
    /// 将此 <see cref="JVector"/> 的位重新解释为 <typeparamref name="T"/>。
    /// </summary>
    /// <remarks>
    /// 仅当 <typeparamref name="T"/> 具有相同大小且布局兼容时有效。
    /// </remarks>
    /// <typeparam name="T">目标非托管类型。</typeparam>
    /// <returns>重新解释后的值。</returns>
    /// <exception cref="InvalidOperationException">
    /// 当 <typeparamref name="T"/> 与 <see cref="JVector"/> 的大小不同时抛出。
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public T UnsafeAs<T>() where T : unmanaged
    {
        if (Unsafe.SizeOf<T>() != Unsafe.SizeOf<JVector>())
        {
            UnsafeInterop.ThrowSizeMismatch<T, JVector>();
        }

        return Unsafe.As<JVector, T>(ref this);
    }

    /// <summary>
    /// 将 <paramref name="value"/> 的位重新解释为 <see cref="JVector"/>。
    /// </summary>
    /// <remarks>
    /// 仅当 <typeparamref name="T"/> 具有相同大小且布局兼容时有效。
    /// </remarks>
    /// <typeparam name="T">源非托管类型。</typeparam>
    /// <param name="value">要重新解释的值。</param>
    /// <returns>重新解释后的向量。</returns>
    /// <exception cref="InvalidOperationException">
    /// 当 <typeparamref name="T"/> 与 <see cref="JVector"/> 的大小不同时抛出。
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JVector UnsafeFrom<T>(in T value) where T : unmanaged
    {
        if (Unsafe.SizeOf<T>() != Unsafe.SizeOf<JVector>())
        {
            UnsafeInterop.ThrowSizeMismatch<JVector, T>();
        }

        return Unsafe.As<T, JVector>(ref Unsafe.AsRef(in value));
    }

    public static implicit operator JVector((Real x, Real y, Real z) tuple) => new(tuple.x, tuple.y, tuple.z);

    public static implicit operator Vector3(in JVector v) => new((float)v.X, (float)v.Y, (float)v.Z);

    public static implicit operator JVector(in Vector3 v) => new(v.X, v.Y, v.Z);
}

public partial struct JQuaternion
{
    /// <summary>
    /// 将此 <see cref="JQuaternion"/> 的位重新解释为 <typeparamref name="T"/>。
    /// </summary>
    /// <remarks>
    /// 仅当 <typeparamref name="T"/> 具有相同大小且布局兼容时有效。
    /// 内存顺序为 X、Y、Z、W（W 在最后）。
    /// </remarks>
    /// <typeparam name="T">目标非托管类型。</typeparam>
    /// <returns>重新解释后的值。</returns>
    /// <exception cref="InvalidOperationException">
    /// 当 <typeparamref name="T"/> 与 <see cref="JQuaternion"/> 的大小不同时抛出。
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public T UnsafeAs<T>() where T : unmanaged
    {
        if (Unsafe.SizeOf<T>() != Unsafe.SizeOf<JQuaternion>())
        {
            UnsafeInterop.ThrowSizeMismatch<T, JQuaternion>();
        }

        return Unsafe.As<JQuaternion, T>(ref this);
    }

    /// <summary>
    /// 将 <paramref name="value"/> 的位重新解释为 <see cref="JQuaternion"/>。
    /// </summary>
    /// <remarks>
    /// 仅当 <typeparamref name="T"/> 具有相同大小且布局兼容时有效。
    /// 要求内存顺序为 X、Y、Z、W（W 在最后）。
    /// </remarks>
    /// <typeparam name="T">源非托管类型。</typeparam>
    /// <param name="value">要重新解释的值。</param>
    /// <returns>重新解释后的四元数。</returns>
    /// <exception cref="InvalidOperationException">
    /// 当 <typeparamref name="T"/> 与 <see cref="JQuaternion"/> 的大小不同时抛出。
    /// </exception>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static JQuaternion UnsafeFrom<T>(in T value) where T : unmanaged
    {
        if (Unsafe.SizeOf<T>() != Unsafe.SizeOf<JQuaternion>())
        {
            UnsafeInterop.ThrowSizeMismatch<JQuaternion, T>();
        }

        return Unsafe.As<T, JQuaternion>(ref Unsafe.AsRef(in value));
    }

    public static implicit operator JQuaternion((Real x, Real y, Real z, Real w) tuple) => new(tuple.x, tuple.y, tuple.z, tuple.w);

    public static implicit operator Quaternion(in JQuaternion q) => new((float)q.X, (float)q.Y, (float)q.Z, (float)q.W);

    public static implicit operator JQuaternion(in Quaternion q) => new(q.X, q.Y, q.Z, q.W);
}

#if DEBUG

public static class UnsafeBase64Serializer<T> where T : unmanaged
{
    public static string Serialize(in T value)
    {
        int size = Unsafe.SizeOf<T>();
        byte[] buffer = new byte[size];

        unsafe
        {
            fixed (byte* ptr = buffer)
            {
                *(T*)ptr = value;
            }
        }

        return Convert.ToBase64String(buffer);
    }

    public static T Deserialize(string base64)
    {
        byte[] buffer = Convert.FromBase64String(base64);

        if (buffer.Length != Unsafe.SizeOf<T>())
        {
            UnsafeInterop.ThrowSizeMismatch("Base64Data", buffer.Length, typeof(T).Name, Unsafe.SizeOf<T>());
        }

        unsafe
        {
            fixed (byte* ptr = buffer)
            {
                return *(T*)ptr;
            }
        }
    }
}

#endif
