/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

/*
 * 本文件定义了用于在模拟过程中记录性能数据的 Tracer 工具。
 *
 * 当在编译时定义符号 PROFILE 时，会完整包含 Tracer 类。
 * 它使用线程本地缓冲区以最小开销记录事件，并可以导出为
 * Chrome Trace Event 格式以供外部分析。公开接口被有意设计得尽量精简：
 * 外部代码可以写出已记录的跟踪数据，但记录本身由引擎在内部
 * 控制。
 *
 * 当未定义 PROFILE 时，Tracer 类会被替换为存根实现。
 * 存根中的所有跟踪方法都带有 [Conditional("_NEVER")] 特性，确保
 * 对 Tracer.* 的每次调用都被编译器完全移除。这消除了非性能分析构建中
 * 跟踪系统的任何运行时开销或依赖。
 */

using System;
using System.Buffers;
using System.Collections.Concurrent;
using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Runtime.CompilerServices;
using System.Text;

namespace Jitter2;

internal enum TraceCategory : ushort
{
    General,
    Solver,
    Runtime,
    Invoke,
}

internal enum TraceName : long
{
    Scope,
    PruneInvalidPairs,
    UpdateBoundingBoxes,
    ScanMoved,
    UpdateProxies,
    ScanOverlaps,
    Gc,
    Step,
    NarrowPhase,
    AddArbiter,
    ReorderContacts,
    CheckDeactivation,
    Solve,
    RemoveArbiter,
    UpdateContacts,
    UpdateBodies,
    BroadPhase,
    Queue,
    PreStep,
    PostStep,
    OptimizeBroadPhase,
}

internal enum TracePhase : byte
{
    Begin,
    End,
    Instant,
    Complete
}

#if PROFILE

/// <summary>
/// 提供对性能跟踪功能的访问。
/// </summary>
/// <remarks>
/// 启用跟踪后，可以在模拟过程中记录性能数据，
/// 并导出以使用 Chrome 跟踪查看器等外部工具进行分析。
/// </remarks>
public static class Tracer
{
    private const string DefaultPath = "trace.json";

    static Tracer()
    {
        string message =
            $">>> PROFILING ENABLED! <<< Use {nameof(Tracer)}.{nameof(WriteToFile)} to dump trace to disk.";

        if (Logger.Listener == null) Console.WriteLine(message);
        else Logger.Warning(message);

        StartGcTracing();
    }

    private struct TraceEvent
    {
        public double TimestampMicro;
        public double DurationMicro;
        public int ThreadId;
        public TraceCategory Category;
        public TracePhase Phase;
        public TraceName Name;
    }

    private sealed class PerThreadBuffer(int initialCapacity)
    {
        public TraceEvent[] Events = ArrayPool<TraceEvent>.Shared.Rent(initialCapacity);
        public int Count = 0;
    }

    private static readonly ConcurrentBag<PerThreadBuffer> allBuffers = new();
    [ThreadStatic] private static PerThreadBuffer? _perThreadBuffer;

    private const int InitialCapacity = 16_384;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static PerThreadBuffer EnsureThreadBuffer()
    {
        var b = _perThreadBuffer;
        if (b is not null) return b;

        b = new PerThreadBuffer(InitialCapacity);
        _perThreadBuffer = b;
        allBuffers.Add(b);
        return b;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Record(ref TraceEvent e)
    {
        var b = EnsureThreadBuffer();
        int i = b.Count;
        if ((uint)i >= (uint)b.Events.Length)
        {
            Grow(b);
        }

        b.Events[i] = e;
        b.Count = i + 1;
    }

    private static void Grow(PerThreadBuffer b)
    {
        var oldArr = b.Events;
        int newLen = oldArr.Length << 1;
        var newArr = ArrayPool<TraceEvent>.Shared.Rent(newLen);
        Array.Copy(oldArr, 0, newArr, 0, oldArr.Length);
        b.Events = newArr;
        ArrayPool<TraceEvent>.Shared.Return(oldArr, clearArray: false);
    }

    private static readonly double ticksToMicro = 1_000_000.0 / Stopwatch.Frequency;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static double NowMicroseconds()
        => Stopwatch.GetTimestamp() * ticksToMicro;

    [ThreadStatic] private static double _scopeStartMicro;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ProfileBegin(TraceName name, TraceCategory cat = TraceCategory.Solver)
    {
        var evt = new TraceEvent
        {
            TimestampMicro = NowMicroseconds(),
            ThreadId = Environment.CurrentManagedThreadId,
            Category = cat,
            Phase = TracePhase.Begin,
            Name = name
        };
        Record(ref evt);
    }

    internal static void ProfileBegin(Delegate @delegate)
    {
        var evt = new TraceEvent
        {
            TimestampMicro = NowMicroseconds(),
            ThreadId = Environment.CurrentManagedThreadId,
            Category = TraceCategory.Invoke,
            Phase = TracePhase.Begin,
            Name = (TraceName)@delegate.Method.MethodHandle.Value,
        };
        Record(ref evt);
    }

    internal static void ProfileEnd(Delegate @delegate)
    {
        var evt = new TraceEvent
        {
            TimestampMicro = NowMicroseconds(),
            ThreadId = Environment.CurrentManagedThreadId,
            Category = TraceCategory.Invoke,
            Phase = TracePhase.End,
            Name = (TraceName)@delegate.Method.MethodHandle.Value,
        };
        Record(ref evt);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ProfileEnd(TraceName name, TraceCategory cat = TraceCategory.Solver)
    {
        var evt = new TraceEvent
        {
            TimestampMicro = NowMicroseconds(),
            ThreadId = Environment.CurrentManagedThreadId,
            Category = cat,
            Phase = TracePhase.End,
            Name = name
        };
        Record(ref evt);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ProfileEvent(TraceName name, TraceCategory cat = TraceCategory.Solver)
    {
        var evt = new TraceEvent
        {
            TimestampMicro = NowMicroseconds(),
            ThreadId = Environment.CurrentManagedThreadId,
            Category = cat,
            Phase = TracePhase.Instant,
            Name = name
        };
        Record(ref evt);
    }

    /// <summary>
    /// 开始测量当前线程上性能分析作用域的持续时间。
    /// </summary>
    /// <remarks>
    /// 每个线程同一时间只能有一个作用域测量处于活动状态。嵌套调用
    /// <see cref="ProfileScopeBegin"/> 会覆盖先前的起始时间戳，
    /// 从而导致持续时间测量不正确。
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ProfileScopeBegin()
    {
        _scopeStartMicro = NowMicroseconds();
    }

    /// <summary>
    /// 结束当前性能分析作用域，并在其持续时间超过给定阈值时记录该持续时间。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal static void ProfileScopeEnd(
        TraceName name = TraceName.Scope,
        TraceCategory cat = TraceCategory.Solver,
        double thresholdMicroseconds = 100.0)
    {
        double end = NowMicroseconds();
        double dur = end - _scopeStartMicro;
        if (dur < thresholdMicroseconds) return;

        var evt = new TraceEvent
        {
            TimestampMicro = _scopeStartMicro,
            DurationMicro = dur,
            ThreadId = Environment.CurrentManagedThreadId,
            Category = cat,
            Phase = TracePhase.Complete,
            Name = name
        };
        Record(ref evt);
    }

    private sealed class GcNotification
    {
        ~GcNotification()
        {
            var evt = new TraceEvent
            {
                TimestampMicro = NowMicroseconds(),
                ThreadId = Environment.CurrentManagedThreadId,
                Category = TraceCategory.Runtime,
                Phase = TracePhase.Instant,
                Name = TraceName.Gc
            };
            Record(ref evt);

            if (!Environment.HasShutdownStarted)
            {
                _ = new GcNotification();
            }
        }
    }

    private static void StartGcTracing()
    {
        _ = new GcNotification();
    }

    /// <summary>
    /// 将所有已记录的跟踪事件写入 JSON 文件。
    /// </summary>
    /// <param name="filename">要写入的文件路径。默认为 <c>trace.json</c>。</param>
    /// <param name="clear">
    /// 如果为 <see langword="true"/>，则写入后清除所有已记录的数据。
    /// 如果为 <see langword="false"/>，则记录的数据保留在内存中。
    /// </param>
    /// <remarks>
    /// 输出文件使用 Chrome Trace Event 格式，可以在
    /// <a href="chrome://tracing">chrome://tracing</a> 或兼容的查看器中打开。
    /// </remarks>
    public static void WriteToFile(string filename = DefaultPath, bool clear = true)
    {
        using var writer = new StreamWriter(filename, false, new UTF8Encoding(false));
        writer.Write('[');

        bool first = true;
        var inv = CultureInfo.InvariantCulture;

        foreach (var buf in allBuffers)
        {
            var arr = buf.Events;
            int count = buf.Count;

            for (int i = 0; i < count; i++)
            {
                ref readonly var e = ref arr[i];
                if (!first) writer.Write(',');
                first = false;

                writer.Write('{');

                writer.Write("\"name\":\"");

                if (e.Category == TraceCategory.Invoke)
                {
                    var method = System.Reflection.MethodBase.GetMethodFromHandle(
                        RuntimeMethodHandle.FromIntPtr((IntPtr)e.Name))!.Name;
                    writer.Write(method);
                }
                else
                {
                    writer.Write(e.Name.ToString());
                }

                writer.Write("\",");

                writer.Write("\"cat\":\"");
                writer.Write(e.Category.ToString());
                writer.Write("\",");

                writer.Write("\"ph\":\"");
                writer.Write(PhaseToChar(e.Phase));
                writer.Write("\",");

                writer.Write("\"ts\":");
                writer.Write(e.TimestampMicro.ToString(inv));

                if (e.Phase == TracePhase.Complete)
                {
                    writer.Write(",\"dur\":");
                    writer.Write(e.DurationMicro.ToString(inv));
                }

                writer.Write(",\"pid\":1,\"tid\":");
                writer.Write(e.ThreadId);

                writer.Write('}');
            }

            if (clear)
            {
                buf.Count = 0;
            }
        }

        writer.Write(']');

        if (clear)
        {
            foreach (var buf in allBuffers)
            {
                if (buf.Events.Length > InitialCapacity)
                {
                    var old = buf.Events;
                    buf.Events = ArrayPool<TraceEvent>.Shared.Rent(InitialCapacity);
                    buf.Count = 0;
                    ArrayPool<TraceEvent>.Shared.Return(old, clearArray: false);
                }
            }
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static char PhaseToChar(TracePhase p) => p switch
    {
        TracePhase.Begin => 'B',
        TracePhase.End => 'E',
        TracePhase.Instant => 'i',
        TracePhase.Complete => 'X',
        _ => '?'
    };
}

#else

internal static class Tracer
{
    [Conditional("_NEVER")]
    internal static void ProfileBegin(TraceName name, TraceCategory cat = TraceCategory.Solver)
    {
    }

    [Conditional("_NEVER")]
    internal static void ProfileEnd(TraceName name, TraceCategory cat = TraceCategory.Solver)
    {
    }

    [Conditional("_NEVER")]
    internal static void ProfileEvent(TraceName name, TraceCategory cat = TraceCategory.Solver)
    {
    }

    [Conditional("_NEVER")]
    internal static void ProfileScopeBegin()
    {
    }

    [Conditional("_NEVER")]
    internal static void ProfileBegin(Delegate @delegate)
    {
    }

    [Conditional("_NEVER")]
    internal static void ProfileEnd(Delegate @delegate)
    {
    }

    [Conditional("_NEVER")]
    internal static void ProfileScopeEnd(TraceName name = TraceName.Scope,
        TraceCategory cat = TraceCategory.Solver,
        double thresholdMicroseconds = 100.0)
    {

    }
}

#endif
