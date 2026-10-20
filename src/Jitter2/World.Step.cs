/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.Intrinsics.X86;
using System.Threading;
using Jitter2.Collision;
using Jitter2.DataStructures;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Constraints;
using Jitter2.LinearMath;
using Jitter2.Parallelization;
using Jitter2.Unmanaged;
using ThreadPool = Jitter2.Parallelization.ThreadPool;

#if NET9_0_OR_GREATER
using Lock = System.Threading.Lock;
#else
using Lock = System.Object;
#endif

namespace Jitter2;

public sealed partial class World
{
    private static readonly Lock MultiThreadStepLock = new();

    // 注意：引用类型 'Arbiter' 的 SlimBag 不会引入 GC 问题（清空时未将所有元素置为 null），
    // 因为仲裁器的引用本来就来自池化。
    private readonly SlimBag<Arbiter> deferredArbiters = [];
    private readonly SlimBag<JHandle<ContactData>> brokenArbiters = [];
    private readonly List<Arbiter> dispatchArbiters = [];
    private readonly List<Arbiter> retiredArbiters = [];

    /// <summary>
    /// <see cref="DebugTimings"/> 的性能分析分桶，对应 <see cref="Step(Real, bool)"/> 的各个阶段。
    /// </summary>
    public enum Timings
    {
        /// <summary>在 <see cref="PreStep"/> 回调中花费的时间。</summary>
        PreStep,

        /// <summary>在窄相碰撞检测和接触生成中花费的时间。</summary>
        NarrowPhase,

        /// <summary>创建延迟仲裁器所花费的时间。</summary>
        AddArbiter,

        /// <summary>重排接触所花费的时间（用于缓存优化或确定性结果）。</summary>
        ReorderContacts,

        /// <summary>评估刚体停用（休眠）所花费的时间。</summary>
        CheckDeactivation,

        /// <summary>在子步中花费的时间：力积分、约束求解和速度积分。</summary>
        Solve,

        /// <summary>移除失效仲裁器所花费的时间。</summary>
        RemoveArbiter,

        /// <summary>求解后更新接触状态所花费的时间。</summary>
        UpdateContacts,

        /// <summary>最终确定刚体状态和宽相代理更新所花费的时间。</summary>
        UpdateBodies,

        /// <summary>更新动态树（宽相）所花费的时间。</summary>
        BroadPhase,

        /// <summary>在 <see cref="PostStep"/> 回调中花费的时间。</summary>
        PostStep,

        /// <summary>用于数组大小的哨兵值。不是真正的计时分桶。</summary>
        Last
    }

    private Action<Parallel.Batch> integrateVelocities;
    private Action<Parallel.Batch> integrateForces;
    private Action<Parallel.Batch> prepareContacts;
    private Action<Parallel.Batch> iterateContacts;
    private Action<Parallel.Batch> relaxVelocities;
    private Action<Parallel.Batch> updateContacts;
    private Action<Parallel.Batch> prepareConstraints;
    private Action<Parallel.Batch> iterateConstraints;
    private Action<Parallel.Batch> prepareSmallConstraints;
    private Action<Parallel.Batch> iterateSmallConstraints;
    private Action<Parallel.Batch> updateBodies;
    private Action<IDynamicTreeProxy, IDynamicTreeProxy> detect;
    private Action<Parallel.Batch> solveIsland;
    private Action<Parallel.Batch> relaxIsland;

    private void InitParallelCallbacks()
    {
        integrateVelocities = IntegrateVelocities;
        integrateForces = IntegrateForces;
        prepareContacts = PrepareContacts;
        iterateContacts = IterateContacts;
        relaxVelocities = RelaxVelocities;
        prepareConstraints = PrepareConstraints;
        iterateConstraints = IterateConstraints;
        prepareSmallConstraints = PrepareSmallConstraints;
        iterateSmallConstraints = IterateSmallConstraints;
        updateContacts = UpdateContacts;
        updateBodies = UpdateBodies;
        detect = Detect;
        solveIsland = SolveIslandBatch;
        relaxIsland = RelaxIslandBatch;
    }

    private readonly double[] debugTimings = new double[(int)Timings.Last];

    /// <summary>
    /// 包含最近一次调用 <see cref="World.Step(Real, bool)"/> 各阶段的计时信息。
    /// 数值单位为毫秒。使用 <c>(int)Timings.XYZ</c> 进行索引。
    /// </summary>
    public ReadOnlySpan<double> DebugTimings => debugTimings;

    /// <summary>
    /// 执行单个模拟步。
    /// </summary>
    /// <param name="dt">要模拟的时间长度，单位为秒。应保持固定且不超过 1/60 秒。</param>
    /// <param name="multiThread">若为 <see langword="true"/>，则使用内部线程池并行执行。
    /// 设为 <see langword="false"/> 可单线程执行（适用于调试或无线程支持的平台）。</param>
    /// <remarks>
    /// 该步被划分为 <see cref="SubstepCount"/> 个子步以提高稳定性。
    /// 每个 <see cref="World"/> 都不得从多个外部线程并发地步进或修改。
    /// 若将 <paramref name="multiThread"/> 设为 <see langword="false"/>，不同世界可以并发步进。
    /// 当 <paramref name="multiThread"/> 为 <see langword="true"/> 时，该步使用进程范围的工作线程池；
    /// 因此多线程步进会在所有世界之间于内部串行化。
    /// 回调（<see cref="PreStep"/>、<see cref="PostStep"/> 等）在调用线程上调用。
    /// 当 <paramref name="multiThread"/> 为 true 时，<see cref="BroadPhaseFilter"/> 和 <see cref="NarrowPhaseFilter"/>
    /// 可能从工作线程被并发调用。
    /// </remarks>
    /// <exception cref="ArgumentException">当 <paramref name="dt"/> 为负数或非有限值时抛出。</exception>
    public void Step(Real dt, bool multiThread = true)
    {
        if (!multiThread)
        {
            StepInternal(dt, false);
            return;
        }

        lock (MultiThreadStepLock)
            StepInternal(dt, true);
    }

    private void StepInternal(Real dt, bool multiThread)
    {
        ThrowIfDisposed();
        AssertNullBody();
        ArgumentCheck.Finite(dt, nameof(dt));

        switch (dt)
        {
            case < (Real)0.0:
                throw new ArgumentException("Time step cannot be negative.", nameof(dt));
            case < Real.Epsilon:
                return; // 无事可做
        }

        try
        {
            Tracer.ProfileBegin(TraceName.Step);

            long time;
            double invFrequency = 1.0d / Stopwatch.Frequency;

            void SetTime(Timings type)
            {
                long ctime = Stopwatch.GetTimestamp();
                double delta = (ctime - time) * 1000.0d;
                debugTimings[(int)type] = delta * invFrequency;
                time = ctime;
            }

            timeStep = new TimeStep(dt, substeps);


            if (multiThread)
            {
                // 通知线程池启动线程
                ThreadPool.Instance.ResumeWorkers();
            }

            // 启动计时器
            time = Stopwatch.GetTimestamp();

            Tracer.ProfileScopeBegin();
            PreStep?.Invoke(timeStep);
            Tracer.ProfileScopeEnd(TraceName.PreStep);
            SetTime(Timings.PreStep);

            // 执行窄相检测。
            Tracer.ProfileBegin(TraceName.NarrowPhase);
            DynamicTree.EnumerateOverlaps(detect, multiThread);
            Tracer.ProfileEnd(TraceName.NarrowPhase);
            SetTime(Timings.NarrowPhase);

            Tracer.ProfileBegin(TraceName.AddArbiter);
            HandleDeferredArbiters();
            Tracer.ProfileEnd(TraceName.AddArbiter);
            SetTime(Timings.AddArbiter);

            Tracer.ProfileBegin(TraceName.CheckDeactivation);
            CheckDeactivation();
            Tracer.ProfileEnd(TraceName.CheckDeactivation);
            SetTime(Timings.CheckDeactivation);
            AssertIslandActivationInvariants();

            Tracer.ProfileBegin(TraceName.ReorderContacts);

            switch (SolveMode)
            {
                case SolveMode.Regular:
                    ReorderContacts();
                    break;
                case SolveMode.Deterministic:
                    PrepareIslandSolveOrder();
                    break;
            }

            Tracer.ProfileEnd(TraceName.ReorderContacts);
            SetTime(Timings.ReorderContacts);

            Tracer.ProfileBegin(TraceName.Solve);

            // 子步进
            for (int i = 0; i < timeStep.SubstepCount; i++)
            {
                PreSubStep?.Invoke(timeStep);
                IntegrateForces(multiThread); // 快速扫掠

                if (SolveMode == Jitter2.SolveMode.Deterministic)
                {
                    SolveIslands(multiThread, solverIterations); // 快速扫掠
                    IntegrateVelocities(multiThread); // 快速扫掠
                    RelaxIslands(multiThread, velocityRelaxations); // 快速扫掠
                }
                else
                {
                    SolveVelocities(multiThread, solverIterations); // 快速扫掠
                    IntegrateVelocities(multiThread); // 快速扫掠
                    RelaxVelocities(multiThread, velocityRelaxations); // 快速扫掠
                }

                PostSubStep?.Invoke(timeStep);
            }

            Tracer.ProfileEnd(TraceName.Solve);
            SetTime(Timings.Solve);

            Tracer.ProfileBegin(TraceName.RemoveArbiter);
            RemoveBrokenArbiters();
            Tracer.ProfileEnd(TraceName.RemoveArbiter);
            SetTime(Timings.RemoveArbiter);
            AssertIslandActivationInvariants();

            Tracer.ProfileBegin(TraceName.UpdateContacts);
            UpdateContacts(multiThread); // 快速扫掠
            Tracer.ProfileEnd(TraceName.UpdateContacts);
            SetTime(Timings.UpdateContacts);

            Tracer.ProfileBegin(TraceName.UpdateBodies);
            ForeachActiveBody(multiThread);
            Tracer.ProfileEnd(TraceName.UpdateBodies);
            SetTime(Timings.UpdateBodies);

            Tracer.ProfileBegin(TraceName.BroadPhase);
            DynamicTree.Update(multiThread, timeStep.StepDt);
            broadphaseStepDt = timeStep.StepDt;
            Tracer.ProfileEnd(TraceName.BroadPhase);
            SetTime(Timings.BroadPhase);

            Tracer.ProfileScopeBegin();
            PostStep?.Invoke(timeStep);
            Tracer.ProfileScopeEnd(TraceName.PostStep);
            SetTime(Timings.PostStep);
        }
        finally
        {
            if ((ThreadModel == ThreadModelType.Regular || !multiThread)
                && ThreadPool.InstanceInitialized)
            {
                // 通知线程池，线程可以进入等待状态。
                ThreadPool.Instance.PauseWorkers();
            }

            Tracer.ProfileEnd(TraceName.Step);
        }
    }

    /// <summary>
    /// 在速度层面求解现有的接触和约束，而不推进刚体变换。
    /// </summary>
    /// <param name="dt">参考时间步长（秒），用于缩放偏置和柔软度项。</param>
    /// <param name="solverIterations">要执行的求解器迭代次数。</param>
    /// <param name="relaxationIterations">求解之后要执行的松弛迭代次数。</param>
    /// <param name="multiThread">若为 <see langword="true"/>，则使用内部线程池并行执行。
    /// 由于工作线程池是进程范围的，多线程调用会在所有世界之间于内部串行化。</param>
    /// <remarks>
    /// 与 <see cref="Step"/> 不同，此方法不执行宽相或窄相碰撞检测，
    /// 不积分力，也不积分位置或朝向。它只在速度层面处理世界中
    /// 已存在的活动接触和约束。
    /// 这主要用于加载先前保存的场景之后：先恢复保存的接触和约束，
    /// 然后调用 <see cref="Stabilize"/> 对恢复的系统进行热启动和求解，
    /// 之后再通过 <see cref="Step"/> 恢复正常模拟。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="dt"/> 为负数或非有限值、<paramref name="solverIterations"/>
    /// 小于 1，或 <paramref name="relaxationIterations"/> 为负数时抛出。
    /// </exception>
    public void Stabilize(Real dt, int solverIterations, int relaxationIterations = 0, bool multiThread = true)
    {
        if (!multiThread)
        {
            StabilizeInternal(dt, solverIterations, relaxationIterations, false);
            return;
        }

        lock (MultiThreadStepLock)
            StabilizeInternal(dt, solverIterations, relaxationIterations, true);
    }

    private void StabilizeInternal(Real dt, int solverIterations, int relaxationIterations, bool multiThread)
    {
        ThrowIfDisposed();
        AssertNullBody();
        ArgumentCheck.Finite(dt, nameof(dt));

        if (dt < (Real)0.0)
        {
            throw new ArgumentException("Time step cannot be negative.", nameof(dt));
        }

        if (solverIterations < 1)
        {
            throw new ArgumentException("Solver iterations cannot be smaller than one.", nameof(solverIterations));
        }

        if (relaxationIterations < 0)
        {
            throw new ArgumentException("Relaxation iterations cannot be smaller than zero.", nameof(relaxationIterations));
        }

        if (dt < Real.Epsilon) return; // 无事可做

        try
        {
            timeStep = new TimeStep(dt, substeps);

            if (multiThread)
            {
                ThreadPool.Instance.ResumeWorkers();
            }

            CheckDeactivation();

            if (SolveMode == Jitter2.SolveMode.Deterministic)
            {
                PrepareIslandSolveOrder();

                for (int i = 0; i < timeStep.SubstepCount; i++)
                {
                    SolveIslands(multiThread, solverIterations);
                    RelaxIslands(multiThread, relaxationIterations);
                }
            }
            else
            {
                for (int i = 0; i < timeStep.SubstepCount; i++)
                {
                    SolveVelocities(multiThread, solverIterations);
                    RelaxVelocities(multiThread, relaxationIterations);
                }
            }

        }
        finally
        {
            if ((ThreadModel == ThreadModelType.Regular || !multiThread)
                && ThreadPool.InstanceInitialized)
            {
                ThreadPool.Instance.PauseWorkers();
            }
        }
    }

    #region Prepare and Solve Contacts and Constraints

    private readonly ThreadLocal<Queue<int>> deferredContacts = new(() => new Queue<int>());
    private readonly ThreadLocal<Queue<int>> deferredConstraints = new(() => new Queue<int>());
    private readonly ThreadLocal<Queue<int>> deferredSmallConstraints = new(() => new Queue<int>());

    private const int SolverPrefetchDistance = 4;
    private const int SolverPrefetchCacheLineSize = 64;

    // 在求解当前接触或约束的同时，请求 CPU 预先取出即将处理的接触或约束所用的刚体。
    // 这些刚体的地址通过句柄获取，比顺序排列的接触和约束数据更难以让 CPU 预测。
    // 这些提示让预取获得先机；它们不会锁定刚体，也不会把刚体的值读入求解器。
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static unsafe void PrefetchBodies(JHandle<RigidBodyData> handle1, JHandle<RigidBodyData> handle2)
    {
        byte* body1 = (byte*)*handle1.Pointer;
        byte* body2 = (byte*)*handle2.Pointer;

        // 覆盖每个刚体的 64 字节缓存行，包括速度、惯性和运动标志。
        Sse.Prefetch0(body1);
        Sse.Prefetch0(body2);
        Sse.Prefetch0(body1 + SolverPrefetchCacheLineSize);
        Sse.Prefetch0(body2 + SolverPrefetchCacheLineSize);

        if (Precision.IsDoublePrecision)
        {
            Sse.Prefetch0(body1 + 2 * SolverPrefetchCacheLineSize);
            Sse.Prefetch0(body2 + 2 * SolverPrefetchCacheLineSize);
            Sse.Prefetch0(body1 + 3 * SolverPrefetchCacheLineSize);
            Sse.Prefetch0(body2 + 3 * SolverPrefetchCacheLineSize);
        }
    }

    private void PrepareContacts(Parallel.Batch batch)
    {
        var span = memContacts.Active[batch.Start..batch.End];
        var localQueue = deferredContacts.Value!;

        for (int i = 0; i < span.Length; i++)
        {
            if (Sse.IsSupported && i + SolverPrefetchDistance < span.Length)
            {
                ref ContactData next = ref span[i + SolverPrefetchDistance];
                PrefetchBodies(next.Body1, next.Body2);
            }

            ref ContactData contact = ref span[i];
            ref RigidBodyData b1 = ref contact.Body1.Data;
            ref RigidBodyData b2 = ref contact.Body2.Data;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            contact.PrepareForIteration(timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }

        Tracer.ProfileScopeBegin();

        while (localQueue.TryDequeue(out int i))
        {
            ref ContactData contact = ref span[i];
            ref RigidBodyData b1 = ref contact.Body1.Data;
            ref RigidBodyData b2 = ref contact.Body2.Data;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            contact.PrepareForIteration(timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }

        Tracer.ProfileScopeEnd(TraceName.Queue, TraceCategory.General, 10);
    }

    private void IterateContacts(Parallel.Batch batch) => IterateContacts(batch, applyBias: true);

    private void RelaxVelocities(Parallel.Batch batch) => IterateContacts(batch, applyBias: false);

    private void IterateContacts(Parallel.Batch batch, bool applyBias)
    {
        var span = memContacts.Active[batch.Start..batch.End];
        var localQueue = deferredContacts.Value!;

        for (int i = 0; i < span.Length; i++)
        {
            if (Sse.IsSupported && i + SolverPrefetchDistance < span.Length)
            {
                ref ContactData next = ref span[i + SolverPrefetchDistance];
                PrefetchBodies(next.Body1, next.Body2);
            }

            ref ContactData c = ref span[i];
            ref RigidBodyData b1 = ref c.Body1.Data;
            ref RigidBodyData b2 = ref c.Body2.Data;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            c.Iterate(applyBias);
            UnlockTwoBody(ref b1, ref b2);
        }

        Tracer.ProfileScopeBegin();

        while (localQueue.TryDequeue(out int i))
        {
            ref ContactData c = ref span[i];
            ref RigidBodyData b1 = ref c.Body1.Data;
            ref RigidBodyData b2 = ref c.Body2.Data;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            c.Iterate(applyBias);
            UnlockTwoBody(ref b1, ref b2);
        }

        Tracer.ProfileScopeEnd(TraceName.Queue, TraceCategory.General, 10);
    }

    private unsafe void PrepareSmallConstraints(Parallel.Batch batch)
    {
        var span = memSmallConstraints.Active[batch.Start..batch.End];
        var localQueue = deferredSmallConstraints.Value!;

        for (int i = 0; i < span.Length; i++)
        {
            if (Sse.IsSupported && i + SolverPrefetchDistance < span.Length)
            {
                ref SmallConstraintData next = ref span[i + SolverPrefetchDistance];
                if (next.IsEnabled) PrefetchBodies(next.Body1, next.Body2);
            }

            ref SmallConstraintData constraint = ref span[i];
            ref RigidBodyData b1 = ref constraint.Body1.Data;
            ref RigidBodyData b2 = ref constraint.Body2.Data;

            if (!constraint.IsEnabled) continue;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            constraint.PrepareForIteration(ref constraint, timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }

        while (localQueue.TryDequeue(out int i))
        {
            ref SmallConstraintData constraint = ref span[i];
            ref RigidBodyData b1 = ref constraint.Body1.Data;
            ref RigidBodyData b2 = ref constraint.Body2.Data;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            constraint.PrepareForIteration(ref constraint, timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }
    }

    private unsafe void IterateSmallConstraints(Parallel.Batch batch)
    {
        var span = memSmallConstraints.Active[batch.Start..batch.End];
        var localQueue = deferredSmallConstraints.Value!;

        for (int i = 0; i < span.Length; i++)
        {
            if (Sse.IsSupported && i + SolverPrefetchDistance < span.Length)
            {
                ref SmallConstraintData next = ref span[i + SolverPrefetchDistance];
                if (next.IsEnabled) PrefetchBodies(next.Body1, next.Body2);
            }

            ref SmallConstraintData constraint = ref span[i];
            ref RigidBodyData b1 = ref constraint.Body1.Data;
            ref RigidBodyData b2 = ref constraint.Body2.Data;

            if (!constraint.IsEnabled) continue;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            constraint.Iterate(ref constraint, timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }

        while (localQueue.TryDequeue(out int i))
        {
            ref SmallConstraintData constraint = ref span[i];
            ref RigidBodyData b1 = ref constraint.Body1.Data;
            ref RigidBodyData b2 = ref constraint.Body2.Data;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            constraint.Iterate(ref constraint, timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }
    }


    private unsafe void PrepareConstraints(Parallel.Batch batch)
    {
        var span = memConstraints.Active[batch.Start..batch.End];
        var localQueue = deferredConstraints.Value!;

        for (int i = 0; i < span.Length; i++)
        {
            if (Sse.IsSupported && i + SolverPrefetchDistance < span.Length)
            {
                ref ConstraintData next = ref span[i + SolverPrefetchDistance];
                if (next.IsEnabled) PrefetchBodies(next.Body1, next.Body2);
            }

            ref ConstraintData constraint = ref span[i];
            ref RigidBodyData b1 = ref constraint.Body1.Data;
            ref RigidBodyData b2 = ref constraint.Body2.Data;

            if (!constraint.IsEnabled) continue;

            Debug.Assert(b1.MotionType == MotionType.Dynamic || b2.MotionType == MotionType.Dynamic,
                "Invalid constraint: both bodies are non-dynamic.");

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            constraint.PrepareForIteration(ref constraint, timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }

        while (localQueue.TryDequeue(out int i))
        {
            ref ConstraintData constraint = ref span[i];
            ref RigidBodyData b1 = ref constraint.Body1.Data;
            ref RigidBodyData b2 = ref constraint.Body2.Data;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            constraint.PrepareForIteration(ref constraint, timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }
    }

    private unsafe void IterateConstraints(Parallel.Batch batch)
    {
        var span = memConstraints.Active[batch.Start..batch.End];
        var localQueue = deferredConstraints.Value!;

        for (int i = 0; i < span.Length; i++)
        {
            if (Sse.IsSupported && i + SolverPrefetchDistance < span.Length)
            {
                ref ConstraintData next = ref span[i + SolverPrefetchDistance];
                if (next.IsEnabled) PrefetchBodies(next.Body1, next.Body2);
            }

            ref ConstraintData constraint = ref span[i];
            ref RigidBodyData b1 = ref constraint.Body1.Data;
            ref RigidBodyData b2 = ref constraint.Body2.Data;

            if (!constraint.IsEnabled) continue;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            constraint.Iterate(ref constraint, timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }

        while (localQueue.TryDequeue(out int i))
        {
            ref ConstraintData constraint = ref span[i];
            ref RigidBodyData b1 = ref constraint.Body1.Data;
            ref RigidBodyData b2 = ref constraint.Body2.Data;

            if (!TryLockTwoBody(ref b1, ref b2))
            {
                localQueue.Enqueue(i);
                continue;
            }

            constraint.Iterate(ref constraint, timeStep);
            UnlockTwoBody(ref b1, ref b2);
        }
    }

    #endregion

    private void UpdateBodies(Parallel.Batch batch)
    {
        for (int i = batch.Start; i < batch.End; i++)
        {
            bodies[i].Update(timeStep);
        }
    }

    private JHandle<RigidBodyData> lastVisited = JHandle<RigidBodyData>.Zero;
    private int sortCounter;

    /// <summary>
    /// 此方法逐步优化 <see cref="memContacts"/> 的内存布局，把彼此相连的接触
    /// 移到相邻位置。这能改善缓存一致性。
    /// </summary>
    private void ReorderContacts()
    {
        int activeCount = memContacts.Active.Length;

        // 当集合较小时，根本不必进行接触排序。
        if (activeCount < 1024) return;

        const int fraction = 1024;

        // 这里的基本思路是逐步改善接触数组的空间局部性。
        // 每帧只访问一小部分接触（总量的 1/1024）。
        // 对每个被访问的接触，算法检查它是否已经与另一个共享其某个刚体的
        // 接触相邻。若不相邻，则尝试把涉及相同刚体的接触在内存中前移，
        // 从而把相互连接的接触归拢到一起。

        int iterations = activeCount / fraction;

        for (int iter = 0; iter < iterations; iter++)
        {
            if (sortCounter > activeCount - 2) sortCounter = 0;

            ref var current = ref memContacts.Active[sortCounter];
            ref var next = ref memContacts.Active[sortCounter + 1];

            if (current.Body1 == next.Body1 || current.Body1 == next.Body2)
            {
                lastVisited = current.Body1;
                sortCounter += 1;
                continue;
            }

            if (current.Body2 == next.Body1 || current.Body2 == next.Body2)
            {
                lastVisited = current.Body2;
                sortCounter += 1;
                continue;
            }

            int swaps = 0;

            if (arbiters.TryGetValue(current.Key, out var arbiter))
            {
                if (lastVisited != arbiter.Body1.Handle)
                {
                    foreach (var arb in arbiter.Body1.Contacts)
                    {
                        int index = memContacts.GetIndex(arb.Handle);
                        if (index <= sortCounter || index >= activeCount) continue;

                        swaps++;
                        memContacts.Swap(sortCounter + swaps, index);
                    }

                    lastVisited = arbiter.Body1.Handle;
                }
                else
                {
                    foreach (var arb in arbiter.Body2.Contacts)
                    {
                        int index = memContacts.GetIndex(arb.Handle);
                        if (index <= sortCounter || index >= activeCount) continue;

                        swaps++;
                        memContacts.Swap(sortCounter + swaps, index);
                    }

                    lastVisited = arbiter.Body2.Handle;
                }
            }

            sortCounter += Math.Max(1, swaps);
        }
    }

    private void AssertNullBody()
    {
        ref RigidBodyData rigidBody = ref NullBody.Data;
        Debug.Assert(rigidBody.MotionType == MotionType.Static);
        Debug.Assert(rigidBody.InverseInertiaWorld == JSymmetricMatrix.Zero);
        Debug.Assert(rigidBody.InverseMassVector == JVector.Zero);
    }

    private void ForeachActiveBody(bool multiThread)
    {
#if DEBUG
        foreach (var body in bodies)
        {
            if (body.Data.MotionType != MotionType.Dynamic)
            {
                Debug.Assert(body.Data.InverseInertiaWorld == JSymmetricMatrix.Zero);
                Debug.Assert(body.Data.InverseMassVector == JVector.Zero);
            }
        }
#endif

        if (multiThread)
        {
            bodies.ParallelForBatch(256, updateBodies);
        }
        else
        {
            Parallel.Batch batch = new(0, bodies.ActiveCount);
            UpdateBodies(batch);
        }
    }

    private void RemoveBrokenArbiters()
    {
        foreach (var handle in brokenArbiters)
        {
            if ((handle.Data.UsageMask & ContactData.MaskContactAll) != 0) continue;
            var arb = arbiters[handle.Data.Key];

            AddToActiveList(arb.Body1.InternalIsland);
            AddToActiveList(arb.Body2.InternalIsland);

            IslandHelper.ArbiterRemoved(islands, islandPool, arb);
            arbiters.Remove(handle.Data.Key);

            arb.Body1.RaiseEndCollide(arb);
            arb.Body2.RaiseEndCollide(arb);

            memContacts.Free(handle);
            ReturnArbiter(arb);
        }

        brokenArbiters.Clear();
    }

    private void UpdateContacts(Parallel.Batch batch)
    {
        var span = memContacts.Active[batch.Start..batch.End];

        for (int i = 0; i < span.Length; i++)
        {
            // 清除已失效的接触
            ref ContactData cq = ref span[i];
            cq.UpdatePosition();

            if ((cq.UsageMask & ContactData.MaskContactAll) == 0)
            {
                var h = memContacts.GetHandle(ref cq);
                brokenArbiters.ConcurrentAdd(h);
            }
        }
    }

    private void HandleDeferredArbiters()
    {
        while (deferredArbiters.Count > 0)
        {
            try
            {
                foreach (var arb in deferredArbiters)
                {
                    IslandHelper.ArbiterCreated(islands, islandPool, arb);

                    AddToActiveList(arb.Body1.InternalIsland);
                    AddToActiveList(arb.Body2.InternalIsland);

                    dispatchArbiters.Add(arb);
                }

                // 在处理器能够移除它们之前，所有接触都必须已链接且不再处于待定状态。
                deferredArbiters.Clear();

                foreach (var arb in dispatchArbiters)
                {
                    // 被移除的仲裁器在此批处理结束前无法复用，因此零句柄
                    // 也会使已排队的通知失效，即使处理器创建了新的接触。
                    if (!arb.Handle.IsZero) arb.Body1.RaiseBeginCollide(arb);
                    if (!arb.Handle.IsZero) arb.Body2.RaiseBeginCollide(arb);
                }
            }
            finally
            {
                dispatchArbiters.Clear();
                foreach (var arb in retiredArbiters) arbiterPool.Push(arb);
                retiredArbiters.Clear();
            }
        }
    }

    /// <summary>
    /// 尝试锁定两个刚体。遇到竞争时短暂等待，若仍不成功则退避。
    /// 锁定顺序由内存地址决定，以防止死锁。
    /// </summary>
    /// <param name="b1">第一个刚体数据的引用。</param>
    /// <param name="b2">第二个刚体数据的引用。</param>
    /// <returns>若两把锁均已获取则为 <see langword="true"/>；否则为 <see langword="false"/>。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool TryLockTwoBody(ref RigidBodyData b1, ref RigidBodyData b2)
    {
        const int spinCount = 10;

        if (Unsafe.IsAddressGreaterThan(ref b1, ref b2))
        {
            if (b1.MotionType == MotionType.Dynamic)
            {
                if (Interlocked.CompareExchange(ref b1._lockFlag, 1, 0) != 0)
                {
                    Thread.SpinWait(spinCount);
                    if (Interlocked.CompareExchange(ref b1._lockFlag, 1, 0) != 0)
                    {
                        return false;
                    }
                }
            }

            if (b2.MotionType == MotionType.Dynamic)
            {
                if (Interlocked.CompareExchange(ref b2._lockFlag, 1, 0) != 0)
                {
                    Thread.SpinWait(spinCount);
                    if (Interlocked.CompareExchange(ref b2._lockFlag, 1, 0) != 0)
                    {
                        // 退避
                        if (b1.MotionType == MotionType.Dynamic) Volatile.Write(ref b1._lockFlag, 0);
                        return false;
                    }
                }
            }
        }
        else
        {
            if (b2.MotionType == MotionType.Dynamic)
            {
                if (Interlocked.CompareExchange(ref b2._lockFlag, 1, 0) != 0)
                {
                    Thread.SpinWait(spinCount);
                    if (Interlocked.CompareExchange(ref b2._lockFlag, 1, 0) != 0)
                    {
                        return false;
                    }
                }
            }

            if (b1.MotionType == MotionType.Dynamic)
            {
                if (Interlocked.CompareExchange(ref b1._lockFlag, 1, 0) != 0)
                {
                    Thread.SpinWait(spinCount);
                    if (Interlocked.CompareExchange(ref b1._lockFlag, 1, 0) != 0)
                    {
                        // 退避
                        if (b2.MotionType == MotionType.Dynamic) Volatile.Write(ref b2._lockFlag, 0);
                        return false;
                    }
                }
            }
        }

        return true;
    }

    /// <summary>
    /// 使用自旋等待循环获取两个刚体上的锁。锁定顺序由内存地址决定，
    /// 以防止死锁。
    /// </summary>
    /// <param name="b1">第一个刚体数据的引用。</param>
    /// <param name="b2">第二个刚体数据的引用。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void LockTwoBody(ref RigidBodyData b1, ref RigidBodyData b2)
    {
        if (Unsafe.IsAddressGreaterThan(ref b1, ref b2))
        {
            if (b1.MotionType == MotionType.Dynamic)
            {
                SpinWait spinWait = new();
                while (Interlocked.CompareExchange(ref b1._lockFlag, 1, 0) != 0)
                {
                    spinWait.SpinOnce();
                }
            }

            if (b2.MotionType == MotionType.Dynamic)
            {
                SpinWait spinWait = new();
                while (Interlocked.CompareExchange(ref b2._lockFlag, 1, 0) != 0)
                {
                    spinWait.SpinOnce();
                }
            }
        }
        else
        {
            if (b2.MotionType == MotionType.Dynamic)
            {
                SpinWait spinWait = new();
                while (Interlocked.CompareExchange(ref b2._lockFlag, 1, 0) != 0)
                {
                    spinWait.SpinOnce();
                }
            }

            if (b1.MotionType == MotionType.Dynamic)
            {
                SpinWait spinWait = new();
                while (Interlocked.CompareExchange(ref b1._lockFlag, 1, 0) != 0)
                {
                    spinWait.SpinOnce();
                }
            }
        }
    }

    /// <summary>
    /// 释放此前由 <see cref="LockTwoBody"/>
    /// 或 <see cref="TryLockTwoBody"/> 获取的两个刚体上的锁。
    /// </summary>
    /// <param name="b1">第一个刚体数据的引用。</param>
    /// <param name="b2">第二个刚体数据的引用。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void UnlockTwoBody(ref RigidBodyData b1, ref RigidBodyData b2)
    {
        if (Unsafe.IsAddressGreaterThan(ref b1, ref b2))
        {
            if (b2.MotionType == MotionType.Dynamic) Interlocked.Decrement(ref b2._lockFlag);
            if (b1.MotionType == MotionType.Dynamic) Interlocked.Decrement(ref b1._lockFlag);
        }
        else
        {
            if (b1.MotionType == MotionType.Dynamic) Interlocked.Decrement(ref b1._lockFlag);
            if (b2.MotionType == MotionType.Dynamic) Interlocked.Decrement(ref b2._lockFlag);
        }
    }

    private void IntegrateForces(Parallel.Batch batch)
    {
        var span = memRigidBodies.Active[batch.Start..batch.End];

        for (int i = 0; i < span.Length; i++)
        {
            ref RigidBodyData rigidBody = ref span[i];
            if (rigidBody.MotionType != MotionType.Dynamic) continue;

            rigidBody.AngularVelocity += rigidBody.DeltaAngularVelocity;
            rigidBody.Velocity += rigidBody.DeltaVelocity;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static JVector SolveGyroscopic(in JSymmetricMatrix inertiaWorld, in JVector omega, Real dt)
    {
        // 本方法用单次牛顿迭代求解 ω_{n+1} 的方程：
        // I_{n+1}(ω_{n+1} - ω_{n}) + h ω_{n+1} x (I_{n+1}ω_{n+1})=0

        // 基于 Erin Catto 的 “Numerical Methods”（GDC 2015）幻灯片。
        // Catto 在**刚体空间**中积分陀螺项，其中惯性
        // 张量 I_b 为常量，然后用*前一*朝向 R_n 把更新后的角速度 ω′
        // 变换回世界空间。

        // 在本实现中，我们只保留世界空间下的逆惯性，因此
        // 直接在**世界空间**中使用下式求解同一隐式方程：
        //   I_w = R_n I_b R_nᵀ（由 t_n 时刻的朝向组装而成）。
        //
        // 两种方法在代数上等价：
        // - Catto 保持 I_b 固定，并用 R_n 旋转 omega'。
        // - 本方法在求解时保持 I_w 固定（= R_n I_b R_n^T）。
        // 两者都引入相同的一阶 O(h) 近似：要么冻结
        // 惯性张量，要么用滞后一步的朝向旋转 omega'。

        JVector f = dt * (omega % JVector.Transform(omega, inertiaWorld));

        JMatrix inertiaMatrix = inertiaWorld.ToMatrix();
        JMatrix jacobian = inertiaMatrix + dt * (JMatrix.CreateCrossProduct(omega) * inertiaMatrix -
                                               JMatrix.CreateCrossProduct(JVector.Transform(omega, inertiaWorld)));

        if (!JMatrix.Inverse(jacobian, out var invJacobian)) return omega;

        return omega - JVector.Transform(f, invJacobian);
    }

    private void IntegrateVelocities(Parallel.Batch batch)
    {
        var span = memRigidBodies.Active[batch.Start..batch.End];

        for (int i = 0; i < span.Length; i++)
        {
            ref RigidBodyData rigidBody = ref span[i];

            // 只有动态和运动学对象具有速度。
            if (rigidBody.MotionType == MotionType.Static) continue;

            JVector linearVelocity = rigidBody.Velocity;
            JVector angularVelocity = rigidBody.AngularVelocity;

            rigidBody.Position += linearVelocity * timeStep.SubstepDt;

            JQuaternion quat = MathHelper.RotationQuaternion(angularVelocity, timeStep.SubstepDt);
            rigidBody.Orientation = JQuaternion.Normalize(quat * rigidBody.Orientation);

            if (!rigidBody.EnableGyroscopicForces || rigidBody.HasMotionLocks) continue;

            // 注意：此处不执行辛欧拉更新（即不用*旧*角速度计算新朝向），
            // 因为陀螺项确实会引入不稳定性。
            // 我们用隐式欧拉处理陀螺项。这被称为辛分裂方法。
            JSymmetricMatrix.Inverse(rigidBody.InverseInertiaWorld, out var inertiaWorld);
            rigidBody.AngularVelocity = SolveGyroscopic(inertiaWorld, angularVelocity, timeStep.SubstepDt);
        }
    }

    private void RelaxVelocities(bool multiThread, int iterations)
    {
        if (multiThread)
        {
            for (int iter = 0; iter < iterations; iter++)
            {
                memContacts.ParallelForBatch(64, relaxVelocities);
            }
        }
        else
        {
            var batchContacts = new Parallel.Batch(0, memContacts.Active.Length);

            for (int iter = 0; iter < iterations; iter++)
            {
                RelaxVelocities(batchContacts);
            }
        }
    }

    private void SolveVelocities(bool multiThread, int iterations)
    {
        if (multiThread)
        {
            memContacts.ParallelForBatch(64, prepareContacts, false);
            memConstraints.ParallelForBatch(64, prepareConstraints, false);
            memSmallConstraints.ParallelForBatch(64, prepareSmallConstraints, false);

            ThreadPool.Instance.Execute();

            for (int iter = 0; iter < iterations; iter++)
            {
                memContacts.ParallelForBatch(64, iterateContacts, false);
                memConstraints.ParallelForBatch(64, iterateConstraints, false);
                memSmallConstraints.ParallelForBatch(64, iterateSmallConstraints, false);

                ThreadPool.Instance.Execute();
            }
        }
        else
        {
            Parallel.Batch batchContacts = new(0, memContacts.Active.Length);
            Parallel.Batch batchConstraints = new(0, memConstraints.Active.Length);
            Parallel.Batch batchSmallConstraints = new(0, memSmallConstraints.Active.Length);

            PrepareContacts(batchContacts);
            PrepareConstraints(batchConstraints);
            PrepareSmallConstraints(batchSmallConstraints);

            for (int iter = 0; iter < iterations; iter++)
            {
                IterateContacts(batchContacts);
                IterateConstraints(batchConstraints);
                IterateSmallConstraints(batchSmallConstraints);
            }
        }
    }

    private void UpdateContacts(bool multiThread)
    {
        if (multiThread)
        {
            memContacts.ParallelForBatch(256, updateContacts);
        }
        else
        {
            Parallel.Batch batch = new(0, memContacts.Active.Length);
            UpdateContacts(batch);
        }
    }

    private void IntegrateForces(bool multiThread)
    {
        if (multiThread)
        {
            memRigidBodies.ParallelForBatch(256, integrateForces);
        }
        else
        {
            IntegrateForces(new Parallel.Batch(0, memRigidBodies.Active.Length));
        }
    }

    private void IntegrateVelocities(bool multiThread)
    {
        if (multiThread)
        {
            memRigidBodies.ParallelForBatch(256, integrateVelocities);
        }
        else
        {
            IntegrateVelocities(new Parallel.Batch(0, memRigidBodies.Active.Length));
        }
    }

    private readonly Stack<(Island Island, bool RaiseEvent)> inactivateIslands = new();

    private static void ClearMotionForSleep(RigidBody body)
    {
        ref RigidBodyData rigidBody = ref body.Data;

        rigidBody.Velocity = JVector.Zero;
        rigidBody.AngularVelocity = JVector.Zero;
        rigidBody.DeltaVelocity = JVector.Zero;
        rigidBody.DeltaAngularVelocity = JVector.Zero;

        body.Force = JVector.Zero;
        body.Torque = JVector.Zero;
        body.InternalSleepTime = Real.PositiveInfinity;
    }

    private bool DeactivateBodyForSleep(RigidBody body)
    {
        ref RigidBodyData rigidBody = ref body.Data;
        bool wasActive = rigidBody.IsActive;

        rigidBody.IsActive = false;

        memRigidBodies.MoveToInactive(body.Handle);
        bodies.MoveToInactive(body);

        bool deactivatedBody = wasActive && body.MotionType != MotionType.Static;

        // 静态刚体具有接触和约束，但它们不构成
        // 碰撞岛。当相连的动态岛进入休眠时，不要停用它们的接触或约束。
        if (body.MotionType != MotionType.Static)
        {
            foreach (var c in body.InternalContacts)
            {
                memContacts.MoveToInactive(c.Handle);
            }

            foreach (var c in body.InternalConstraints)
            {
                if (c.IsSmallConstraint)
                {
                    memSmallConstraints.MoveToInactive(c.SmallHandle);
                }
                else
                {
                    memConstraints.MoveToInactive(c.Handle);
                }
            }
        }

        foreach (var s in body.InternalShapes)
        {
            DynamicTree.DeactivateProxy(s);
        }

        return deactivatedBody;
    }

    private void CheckDeactivation()
    {
        for (int i = 0; i < islands.ActiveCount; i++)
        {
            Island island = islands[i];

            bool deactivateIsland = !island.MarkedAsActive;
            if (!AllowDeactivation) deactivateIsland = false;

            // 将该岛标记为非活动
            // 下一帧只要有一个活动刚体就足以把
            // MarkedAsActive 重新置为 true；
            island.MarkedAsActive = false;

            bool needsUpdate = island.NeedsUpdate;
            island.NeedsUpdate = false;

            if (!deactivateIsland && !needsUpdate) continue;

            bool activatedBody = false;
            bool deactivatedBody = false;

            foreach (RigidBody body in island.InternalBodies)
            {
                ref RigidBodyData rigidBody = ref body.Data;

                if (rigidBody.IsActive != deactivateIsland)
                {
                    if (!needsUpdate) break;
                    continue;
                }

                if (deactivateIsland)
                {
                    deactivatedBody |= DeactivateBodyForSleep(body);
                }
                else
                {
                    // 完全不要激活静态刚体。
                    if (rigidBody.MotionType == MotionType.Static) continue;

                    rigidBody.IsActive = true;
                    activatedBody = true;

                    body.InternalSleepTime = 0;

                    memRigidBodies.MoveToActive(body.Handle);
                    bodies.MoveToActive(body);

                    foreach (var c in body.InternalContacts)
                    {
                        memContacts.MoveToActive(c.Handle);
                    }

                    foreach (var c in body.InternalConstraints)
                    {
                        UpdateConstraintActivation(c);
                    }

                    foreach (var s in body.InternalShapes)
                    {
                        DynamicTree.ActivateProxy(s);
                    }
                }
            }

            if (deactivateIsland)
            {
                inactivateIslands.Push((island, deactivatedBody));
            }
            else if (activatedBody)
            {
                IslandActivated?.Invoke(island);
            }
        }

        while (inactivateIslands.Count > 0)
        {
            var (island, raiseEvent) = inactivateIslands.Pop();
            islands.MoveToInactive(island);
            if (raiseEvent) IslandDeactivated?.Invoke(island);
        }
    }
}
