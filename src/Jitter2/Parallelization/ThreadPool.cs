/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Concurrent;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.ExceptionServices;
using System.Threading;
using Jitter2.DataStructures;
using Jitter2.Unmanaged;

namespace Jitter2.Parallelization;

/*
 * ---------------------------------------------------------------------------
 *  Jitter2 ThreadPool – 持久化工作线程模型
 * ---------------------------------------------------------------------------
 *
 *  该线程池的设计思路是：在仿真的整个生命周期内保持一组固定的工作线程
 *  存活。工作线程处于紧密循环中，仅在收到明确指令时挂起。
 *  这样就避免了在每个仿真步 *期间* 反复唤醒线程的开销。
 *
 * 主线程：
 *
 *  [ 1. 入队任务 ]
 *      任务通过 AddTask() 添加到临时列表。任务是包装 Action<T> 和参数的
 *      轻量级对象，会被池化以便复用。
 *
 *  [ 2. 执行 ]
 *      调用 Execute() 时，所有暂存任务会以轮询方式
 *      分发到各线程的队列。
 *
 *      随后主线程作为“工作线程 0”参与任务执行，
 *      清空自己的队列并从其他线程窃取工作，直到所有任务
 *      完成。
 *
 * 工作线程：
 *
 *  [ 3. 工作线程循环 ]
 *      每个工作线程反复执行：
 *
 *        - 完全清空自己的队列。
 *        - 如果它执行了任何工作，就尝试从其他队列窃取任务
 *          （并清空它访问的每个队列）。
 *        - 如果没有可用工作，则短暂自旋并在 ManualResetEventSlim 上等待。
 *
 *      工作线程永久留在此循环中。当事件被置位
 *      （通过 ResumeWorkers）时，它们继续运行；当事件被重置（通过 PauseWorkers）时，
 *      它们在无剩余工作时阻塞。
 *
 *  [ 4. 完成 ]
 *      Execute() 会阻塞直到所有任务完成（tasksLeft == 0），
 *      但线程本身保持热态。仿真循环可以在每一步中
 *      多次调用 Execute() 而无需唤醒线程。
 *
 *      在仿真步结束时，会调用 PauseWorkers() 让线程
 *      挂起，直到下一步。
 *
 *  ---------------------------------------------------------------------------
 *  为什么采用这种设计？
 *  ---------------------------------------------------------------------------
 *  - 仿真步期间没有线程唤醒开销（线程是持久化的）。
 *  - 任务倾向于在每个仿真步由同一线程拾取，从而改善
 *    缓存局部性。
 *  - 工作窃取改善了负载均衡。
 * ---------------------------------------------------------------------------
 */

/// <summary>
/// 提供一个持久化工作线程池，用于在物理引擎内并行执行任务。
/// </summary>
/// <remarks>
/// <para>
/// 工作线程在整个池的生命周期内保持存活，以避免唤醒开销。
/// 任务通过各线程的队列分发，并使用工作窃取实现负载均衡。
/// </para>
/// <para>
/// 在 <see cref="Execute"/> 调用期间，主线程作为“工作线程 0”参与执行。
/// 使用 <see cref="ResumeWorkers"/> 和 <see cref="PauseWorkers"/> 在仿真步之间
/// 控制线程活动。
/// </para>
/// </remarks>
public sealed class ThreadPool
{
    private interface ITask
    {
        public void Perform();
    }

    private sealed class Task<T> : ITask
    {
        public Action<T> Action = null!;
        public T Parameter = default!;

        public void Perform()
        {
            _counter = _total;

            Tracer.ProfileBegin(Action);
            try
            {
                Action(Parameter);
            }
            finally
            {
                Tracer.ProfileEnd(Action);
            }
        }

        private static readonly List<Task<T>> pool = new(32);

        private static int _counter;
        private static int _total;

        public static Task<T> GetFree()
        {
            if (_counter == 0)
            {
                _counter++;
                _total++;
                pool.Add(new Task<T>());
            }

            return pool[^_counter--];
        }
    }

    /// <summary>
    /// 用于工作线程的可用处理器比例。
    /// </summary>
    public const float ThreadsPerProcessor = 0.9f;

    private readonly ManualResetEventSlim mainResetEvent;
    private Thread[] threads = [];

    private readonly SlimBag<ITask> taskList = [];

    private ConcurrentQueue<ITask>[] queues = [];

    private volatile bool running = true;

    private MemoryHelper.IsolatedInt tasksLeft;
    private int threadCount;
    private ExceptionDispatchInfo? capturedException;

    private static ThreadPool? _instance;

    /// <summary>
    /// 获取此池管理的工作线程数。
    /// </summary>
    /// <value>包含作为工作线程 0 的主线程。</value>
    public int ThreadCount => threadCount;

    /// <summary>
    /// 获取是否已请求工作线程在空闲时暂停。
    /// </summary>
    /// <remarks>
    /// 值为 <see langword="true"/> 并不保证每个工作线程都已阻塞；
    /// 工作线程可能仍在完成先前排队的任务。
    /// </remarks>
    public bool IsPaused => !mainResetEvent.IsSet;

    private ThreadPool()
    {
        threadCount = 0;
        mainResetEvent = new ManualResetEventSlim(true);

        int initialThreadCount = ThreadCountSuggestion;

#if !NET9_0_OR_GREATER
        // 9.0 之前的 .NET 版本存在一个已知问题，在非 Windows 系统上调试时
        // 可能导致卡死或冻结。参见：https://github.com/dotnet/runtime/pull/95555
        // 为避免此问题，在非 Windows 系统上附加调试器时会禁用多线程。
        if (!OperatingSystem.IsWindows() && Debugger.IsAttached)
        {
            Debug.WriteLine(
                "Multi-threading disabled to prevent potential hangups: Debugger attached, " +
                ".NET version < 9.0, non-Windows system detected.");
            initialThreadCount = 1; // 强制使用单线程以避免卡死
        }
#endif

        ChangeThreadCount(initialThreadCount);
    }

    /// <summary>
    /// 根据可用处理器数获取建议的线程数。
    /// </summary>
    /// <value>至少为 1，计算方式为 <see cref="ThreadsPerProcessor"/> × 处理器数。</value>
    public static int ThreadCountSuggestion => Math.Max((int)(Environment.ProcessorCount * ThreadsPerProcessor), 1);

    /// <summary>
    /// 更改工作线程数。
    /// </summary>
    /// <param name="numThreads">新的线程数。</param>
    /// <remarks>
    /// 现有的工作线程会被停止并创建新线程。
    /// 此操作会阻塞，直到之前的所有线程都已终止。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="numThreads"/> 小于一时抛出。
    /// </exception>
    public void ChangeThreadCount(int numThreads)
    {
        ArgumentOutOfRangeException.ThrowIfLessThan(numThreads, 1);

        if (numThreads == threadCount) return;

        running = false;
        mainResetEvent.Set();

        for (int i = 0; i < threadCount - 1; i++)
        {
            threads[i].Join();
        }

        running = true;
        threadCount = numThreads;

        queues = new ConcurrentQueue<ITask>[threadCount];
        for (int i = 0; i < threadCount; i++)
            queues[i] = new ConcurrentQueue<ITask>();

        threads = new Thread[threadCount - 1];

        using AutoResetEvent initWaitHandle = new(false);

        for (int i = 0; i < threadCount - 1; i++)
        {
            int index = i;

            threads[i] = new Thread(() =>
            {
                initWaitHandle.Set();
                ThreadProc(index + 1);
            });

            threads[i].IsBackground = true;
            threads[i].Start();
            initWaitHandle.WaitOne();
        }

        PauseWorkers();
    }

    /// <summary>
    /// 将任务添加到队列中以便稍后执行。
    /// </summary>
    /// <typeparam name="T">任务参数的类型。</typeparam>
    /// <param name="action">要执行的操作。</param>
    /// <param name="parameter">传递给操作的参数。</param>
    /// <remarks>
    /// 在调用 <see cref="Execute"/> 之前任务不会执行。
    /// 此方法不是线程安全的，必须从单个线程调用。
    /// </remarks>
    /// <exception cref="ArgumentNullException">当 <paramref name="action"/> 为 <see langword="null"/> 时抛出。</exception>
    public void AddTask<T>(Action<T> action, T parameter)
    {
        ArgumentNullException.ThrowIfNull(action);

        var instance = Task<T>.GetFree();
        instance.Action = action;
        instance.Parameter = parameter;
        taskList.Add(instance);
    }

    /// <summary>
    /// 指示 <see cref="ThreadPool"/> 实例是否已初始化。
    /// </summary>
    /// <value>如果已初始化则为 <c>true</c>；否则为 <c>false</c>。</value>
    public static bool InstanceInitialized => _instance != null;

    /// <summary>
    /// 获取共享的 <see cref="ThreadPool"/> 实例。首次访问必须由调用方串行化；
    /// 并发初始化可能创建多个池。
    /// </summary>
    public static ThreadPool Instance
    {
        get
        {
            _instance ??= new ThreadPool();
            return _instance;
        }
    }

    /// <summary>
    /// 恢复所有工作线程，使其可以处理排队任务。
    /// </summary>
    /// <remarks>
    /// 由 <see cref="Execute"/> 以及世界在多线程步中自动调用。
    /// 直接管理线程池时，手动调用会很有用。
    /// </remarks>
    public void ResumeWorkers()
    {
        mainResetEvent.Set();
    }

    /// <summary>
    /// 在所有工作线程完成当前任务后暂停它们。
    /// </summary>
    /// <remarks>
    /// 工作线程将阻塞在等待句柄上，直到调用 <see cref="ResumeWorkers"/>。
    /// 这会减少仿真步之间的 CPU 占用。
    /// </remarks>
    public void PauseWorkers()
    {
        mainResetEvent.Reset();
    }

    private void ThreadProc(int index)
    {
        var myQueue = queues[index];

        while (running)
        {
            int performedTasks = 0;

            while (myQueue.TryDequeue(out var task))
            {
                PerformTask(task);
                performedTasks++;
            }

            // 已完成自身所有任务。现在才尝试从其他线程窃取工作。
            if (performedTasks > 0)
            {
                // 从其他队列窃取
                int queueCount = queues.Length;
                for (int i = 1; i < queueCount; i++)
                {
                    int queueIndex = (i + index) % queueCount;

                    while (queues[queueIndex].TryDequeue(out var task))
                    {
                        PerformTask(task);
                    }
                }
            }

            Thread.Sleep(0);
            mainResetEvent.Wait();
        }
    }

    private void PerformTask(ITask task)
    {
        try
        {
            task.Perform();
        }
        catch (Exception exception)
        {
            Interlocked.CompareExchange(
                ref capturedException,
                ExceptionDispatchInfo.Capture(exception),
                null);
        }
        finally
        {
            Interlocked.Decrement(ref tasksLeft.Value);
        }
    }

    /// <summary>
    /// 执行所有排队任务并阻塞直到完成。
    /// </summary>
    /// <remarks>
    /// <para>
    /// 任务按轮询顺序分发到各线程的队列。主线程
    /// 作为工作线程 0 参与，并从其他队列执行工作窃取。
    /// </para>
    /// <para>
    /// 此方法会在开始时自动调用 <see cref="ResumeWorkers"/>。
    /// </para>
    /// </remarks>
    public void Execute()
    {
        ResumeWorkers();

        Interlocked.Exchange(ref capturedException, null);

        int totalTasks = taskList.Count;
        int queueCount = ThreadCount;
        Volatile.Write(ref tasksLeft.Value, totalTasks);

        for (int i = 0; i < totalTasks; i++)
        {
            queues[i % queueCount].Enqueue(taskList[i]);
        }

        taskList.Clear();

        // 先处理主线程的队列。
        var myQueue = queues[0];

        while (myQueue.TryDequeue(out var task))
        {
            PerformTask(task);
        }

        // 从其他队列窃取。
        for (int i = 1; i < queues.Length; i++)
        {
            while (queues[i].TryDequeue(out var task))
            {
                PerformTask(task);
            }
        }

        while (Volatile.Read(ref tasksLeft.Value) > 0)
        {
            Thread.SpinWait(1);
        }

        var exception = Volatile.Read(ref capturedException);
        if (exception != null)
        {
            PauseWorkers();
            exception.Throw();
        }
    }
}
