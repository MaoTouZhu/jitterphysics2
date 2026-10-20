using System;
using JitterDemo.Renderer.OpenGL.Native;

namespace JitterDemo.Renderer.OpenGL;

public enum BufferUsage : uint
{
    StaticDraw = GLC.STATIC_DRAW,
    DynamicDraw = GLC.DYNAMIC_DRAW,
    StreamDraw = GLC.STREAM_DRAW
}

public sealed class GpuBuffer
{
    public uint Handle { get; }
    public uint Target { get; }
    public int Capacity { get; private set; }

    private GpuBuffer(uint target)
    {
        Target = target;
        Handle = GL.GenBuffer();
    }

    public static GpuBuffer Vertex() => new(GLC.ARRAY_BUFFER);
    public static GpuBuffer Index() => new(GLC.ELEMENT_ARRAY_BUFFER);

    public void Bind() => GL.BindBuffer(Target, Handle);

    public void Upload<T>(ReadOnlySpan<T> data, BufferUsage usage = BufferUsage.StaticDraw) where T : unmanaged
    {
        Bind();
        unsafe
        {
            fixed (T* p = data)
            {
                int bytes = data.Length * sizeof(T);
                GL.BufferData(Target, bytes, (IntPtr)p, (uint)usage);
                Capacity = bytes;
            }
        }
    }

    public void Upload<T>(T[] data, BufferUsage usage = BufferUsage.StaticDraw) where T : unmanaged
        => Upload<T>(data.AsSpan(), usage);

    public void Upload<T>(T[] data, int count, BufferUsage usage = BufferUsage.StaticDraw) where T : unmanaged
        => Upload<T>(data.AsSpan(0, count), usage);

    public void Upload(IntPtr data, int bytes, BufferUsage usage = BufferUsage.StaticDraw)
    {
        Bind();
        GL.BufferData(Target, bytes, data, (uint)usage);
        Capacity = bytes;
    }

    // 孤儿化 + 子数据：把旧的后备存储交还给驱动，再写入新分配的内存。
    // 这样我们填充新缓冲区时，驱动可以让上一帧的缓冲区仍处于使用中。
    public void Stream<T>(ReadOnlySpan<T> data) where T : unmanaged
    {
        Bind();
        unsafe
        {
            fixed (T* p = data)
            {
                int bytes = data.Length * sizeof(T);
                GL.BufferData(Target, bytes, IntPtr.Zero, (uint)BufferUsage.DynamicDraw);
                if (bytes > 0) GL.BufferSubData(Target, 0, bytes, (IntPtr)p);
                Capacity = bytes;
            }
        }
    }

    public void Stream<T>(T[] data, int count) where T : unmanaged
        => Stream<T>(data.AsSpan(0, count));

    public void Stream(IntPtr data, int bytes)
    {
        Bind();
        GL.BufferData(Target, bytes, IntPtr.Zero, (uint)BufferUsage.DynamicDraw);
        if (bytes > 0) GL.BufferSubData(Target, 0, bytes, data);
        Capacity = bytes;
    }
}
