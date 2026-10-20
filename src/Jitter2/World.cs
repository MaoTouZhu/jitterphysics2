/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Threading;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.DataStructures;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Constraints;
using Jitter2.LinearMath;
using Jitter2.Unmanaged;

#pragma warning disable CS8618 // 退出构造函数时，不可为 null 的字段必须包含非 null 值。

namespace Jitter2;

/// <summary>
/// 当两个刚体参数被要求引用不同的刚体时引发。
/// </summary>
public sealed class SameBodyException : ArgumentException
{
    private const string DefaultMessage = "Body arguments must be different.";
    private const string DefaultParamName = "body2";

    public SameBodyException()
        : base(DefaultMessage, DefaultParamName)
    {
    }

    public SameBodyException(string message)
        : base(message)
    {
    }

    public SameBodyException(string message, Exception innerException)
        : base(message, innerException)
    {
    }

    public SameBodyException(string message, string paramName)
        : base(message, paramName)
    {
    }

    public SameBodyException(string message, string paramName, Exception innerException)
        : base(message, paramName, innerException)
    {
    }
}

/// <summary>
/// 表示一个模拟环境，用于保存并管理所有模拟对象的状态。
/// </summary>
/// <remarks>
/// 单个 <see cref="World"/> 实例不得从多个外部线程并发地步进或修改。不同的世界可以在
/// <c>multiThread: false</c> 时并发步进。
/// 使用 <c>multiThread: true</c> 的调用会在所有世界之间内部串行化，因为它们共享
/// 进程级的 Jitter 工作线程池。
/// </remarks>
public sealed partial class World : IDisposable
{
    /// <summary>
    /// 控制内部工作线程在两次 <see cref="Step(Real, bool)"/> 调用之间的行为。
    /// </summary>
    public enum ThreadModelType
    {
        /// <summary>
        /// 引擎空闲时工作线程可以让出。降低后台 CPU 占用。
        /// </summary>
        Regular,

        /// <summary>
        /// 工作线程在两次步进之间保持活动，以减少唤醒延迟，
        /// 代价是更高的后台 CPU 占用。
        /// </summary>
        Persistent
    }

    /// <summary>
    /// 提供对非托管内存中对象的访问。此操作可能不安全。
    /// </summary>
    /// <remarks>
    /// 返回的 span 由非托管内存支撑，仅在下次可能调整内部缓冲区大小的世界修改之前有效
    /// （例如创建或移除刚体、约束、接触，或调用 <see cref="Step"/>）。不要缓存这些 span。
    /// 不得与 <see cref="Step(Real, bool)"/> 并发使用。
    /// </remarks>
    public readonly struct SpanData(World world)
    {
        /// <summary>
        /// 返回已分配的非托管内存总量，单位为字节。
        /// </summary>
        public long TotalBytesAllocated =>
            world.memRigidBodies.TotalBytesAllocated +
            world.memContacts.TotalBytesAllocated +
            world.memConstraints.TotalBytesAllocated +
            world.memSmallConstraints.TotalBytesAllocated;

        /// <summary>活动（唤醒）刚体数据的 span。</summary>
        public Span<RigidBodyData> ActiveRigidBodies => world.memRigidBodies.Active;

        /// <summary>非活动（休眠）刚体数据的 span。</summary>
        public Span<RigidBodyData> InactiveRigidBodies => world.memRigidBodies.Inactive;

        /// <summary>所有刚体数据（活动与非活动）的 span。</summary>
        public Span<RigidBodyData> RigidBodies => world.memRigidBodies.Elements;

        /// <summary>活动接触数据的 span。</summary>
        public Span<ContactData> ActiveContacts => world.memContacts.Active;

        /// <summary>非活动接触数据的 span。</summary>
        public Span<ContactData> InactiveContacts => world.memContacts.Inactive;

        /// <summary>所有接触数据（活动与非活动）的 span。</summary>
        public Span<ContactData> Contacts => world.memContacts.Elements;

        /// <summary>活动约束数据的 span。</summary>
        public Span<ConstraintData> ActiveConstraints => world.memConstraints.Active;

        /// <summary>非活动约束数据的 span。</summary>
        public Span<ConstraintData> InactiveConstraints => world.memConstraints.Inactive;

        /// <summary>所有约束数据（活动与非活动）的 span。</summary>
        public Span<ConstraintData> Constraints => world.memConstraints.Elements;

        /// <summary>活动小型约束数据的 span。</summary>
        public Span<SmallConstraintData> ActiveSmallConstraints => world.memSmallConstraints.Active;

        /// <summary>非活动小型约束数据的 span。</summary>
        public Span<SmallConstraintData> InactiveSmallConstraints => world.memSmallConstraints.Inactive;

        /// <summary>所有小型约束数据（活动与非活动）的 span。</summary>
        public Span<SmallConstraintData> SmallConstraints => world.memSmallConstraints.Elements;
    }

    private readonly PartitionedBuffer<ContactData> memContacts;
    private readonly PartitionedBuffer<RigidBodyData> memRigidBodies;
    private readonly PartitionedBuffer<ConstraintData> memConstraints;
    private readonly PartitionedBuffer<SmallConstraintData> memSmallConstraints;

    /// <summary>
    /// 用于逐步和逐子步回调的委托。
    /// </summary>
    /// <param name="timeStep">完整的步与子步计时信息。</param>
    public delegate void WorldStep(TimeStep timeStep);

    // 步进回调。

    /// <summary>
    /// 在模拟步开始时引发，此时尚未执行任何碰撞检测、
    /// 约束求解或积分。
    /// </summary>
    /// <remarks>
    /// 每次调用 <see cref="Step"/> 时此事件引发一次，并接收完整的
    /// 步与子步计时信息。可用于施加外力、修改刚体，
    /// 或在模拟推进前收集逐步诊断信息。
    /// </remarks>
    [CallbackThread(ThreadContext.MainThread)]
    public event WorldStep? PreStep;

    /// <summary>
    /// 在模拟步结束时引发，此时所有子步、碰撞处理
    /// 和积分均已完成。
    /// </summary>
    /// <remarks>
    /// 每次调用 <see cref="Step"/> 时此事件引发一次，并接收完整的
    /// 步与子步计时信息。此时所有刚体状态即为该步的
    /// 最终结果。
    /// </remarks>
    [CallbackThread(ThreadContext.MainThread)]
    public event WorldStep? PostStep;

    /// <summary>
    /// 在模拟步期间每个子步开始时引发。
    /// </summary>
    /// <remarks>
    /// 为提高稳定性，一个模拟步可划分为多个子步。
    /// 此事件在每个子步引发一次，并接收完整的步与子步
    /// 计时信息。它在子步的力积分
    /// 和约束求解之前立即调用。
    /// 不要在此处执行会改变拓扑的世界修改（例如
    /// 添加或移除刚体、约束或接触，或修改刚体
    /// 运动类型）。
    /// </remarks>
    [CallbackThread(ThreadContext.MainThread)]
    public event WorldStep? PreSubStep;

    /// <summary>
    /// 在模拟步期间每个子步结束时引发。
    /// </summary>
    /// <remarks>
    /// 此事件在每个子步引发一次，并接收完整的步与子步计时信息。
    /// 它在子步的积分和约束求解
    /// 完成之后调用。
    /// 不要在此处执行会改变拓扑的世界修改（例如
    /// 添加或移除刚体、约束或接触，或修改刚体
    /// 运动类型）。
    /// </remarks>
    [CallbackThread(ThreadContext.MainThread)]
    public event WorldStep? PostSubStep;

    /// <summary>
    /// 当模拟岛中的非活动非静态刚体在模拟步期间被激活时引发。
    /// </summary>
    /// <remarks>
    /// 此事件在岛的非静态刚体、接触、约束和
    /// 形状被移动到活动分区之后调用。岛实例是内部
    /// 连通性对象，之后可能拆分、合并或被复用；请在回调期间
    /// 复制所需数据，而不要缓存岛引用。
    /// 不要在此处执行会改变拓扑的世界修改（例如
    /// 添加或移除刚体、约束或接触，或修改刚体
    /// 运动类型）。
    /// </remarks>
    [CallbackThread(ThreadContext.MainThread)]
    public event Action<Island>? IslandActivated;

    /// <summary>
    /// 当模拟岛中的活动非静态刚体在模拟步期间被停用时引发。
    /// </summary>
    /// <remarks>
    /// 此事件在岛的非静态刚体、接触、约束和
    /// 形状被移动到非活动分区之后调用。岛实例是内部
    /// 连通性对象，之后可能拆分、合并或被复用；请在回调期间
    /// 复制所需数据，而不要缓存岛引用。
    /// 不要在此处执行会改变拓扑的世界修改（例如
    /// 添加或移除刚体、约束或接触，或修改刚体
    /// 运动类型）。
    /// </remarks>
    [CallbackThread(ThreadContext.MainThread)]
    public event Action<Island>? IslandDeactivated;

    /// <summary>
    /// 在约束被永久从世界中移除之后引发。
    /// </summary>
    /// <remarks>
    /// 无论是显式移除，还是因移除刚体或清空世界而导致的自动移除，都会引发此事件。
    /// 约束被禁用、刚体的运动类型改变或世界被释放时不会引发。
    /// 回调运行时约束已无效，但其 <see cref="Constraint.Body1"/> 和
    /// <see cref="Constraint.Body2"/> 引用仍然可用。
    /// </remarks>
    [CallbackThread(ThreadContext.MainThread)]
    public event Action<Constraint>? ConstraintRemoved;

    /// <summary>
    /// 授予对驻留在非托管内存中的对象的访问权限。此操作可能不安全。在可能的情况下
    /// 请使用对应的托管属性以降低风险。
    /// </summary>
    /// <exception cref="ObjectDisposedException">若此世界已被释放则引发。</exception>
    public SpanData RawData
    {
        get
        {
            ThrowIfDisposed();
            return new SpanData(this);
        }
    }

    private readonly ShardedDictionary<ArbiterKey, Arbiter> arbiters =
        new(Parallelization.ThreadPool.ThreadCountSuggestion);

    // 在创建/回滚期间于锁（memContacts）内访问。移除是独占的：
    // 要么在碰撞工作线程结束后的 Step 期间，要么在两次模拟步之间。
    private readonly Stack<Arbiter> arbiterPool = [];

    private readonly PartitionedSet<Island> islands = [];
    private readonly PartitionedSet<RigidBody> bodies = [];
    private readonly Stack<Island> islandPool = [];

    private static ulong _idCounter;

    /// <summary>
    /// 生成一个唯一 ID。
    /// </summary>
    /// <returns>一个单调递增的唯一标识符。</returns>
    public static ulong RequestId()
    {
        return Interlocked.Increment(ref _idCounter);
    }

    /// <summary>
    /// 生成一段范围的唯一 ID。
    /// </summary>
    /// <param name="count">要生成的 ID 数量。</param>
    /// <returns>一个元组，包含所生成范围内的最小和最大请求 ID。
    /// 上限为排他边界。</returns>
    /// <exception cref="ArgumentOutOfRangeException">当 count 小于 1 时引发。</exception>
    public static (ulong min, ulong max) RequestId(int count)
    {
        if (count < 1) throw new ArgumentOutOfRangeException(nameof(count), "Count must be greater than zero.");
        ulong count64 = (ulong)count;
        ulong max = Interlocked.Add(ref _idCounter, count64) + 1;
        return (max - count64, max);
    }

    /// <summary>
    /// 获取或设置两次 <see cref="Step(Real, bool)"/> 调用之间使用的线程模型。
    /// </summary>
    public ThreadModelType ThreadModel { get; set; } = ThreadModelType.Regular;

    /// <summary>
    /// 此世界中的所有碰撞岛。
    /// </summary>
    public ReadOnlyPartitionedSet<Island> Islands => new(islands);

    /// <summary>
    /// 此世界中的所有刚体。
    /// </summary>
    public ReadOnlyPartitionedSet<RigidBody> RigidBodies => new(bodies);

    /// <summary>
    /// 访问 <see cref="DynamicTree"/> 实例。调用方可以管理自己的代理，并在
    /// 修改形状几何体后更新形状。Jitter 会为已附加的刚体形状管理代理。
    /// </summary>
    public DynamicTree DynamicTree { get; }

    /// <summary>
    /// 一个固定刚体，钉扎在世界中。可用于创建约束。
    /// </summary>
    public RigidBody NullBody { get; }

    /// <summary>
    /// 指定是否启用 Jitter 的停用机制。
    /// 设为 false 时不会激活非活动对象。
    /// </summary>
    public bool AllowDeactivation { get; set; } = true;

    /// <summary>
    /// 获取或设置约束求解器和速度松弛每个子步的迭代次数。
    /// </summary>
    /// <remarks>
    /// 更高的求解器迭代次数可提高约束精度，但会牺牲性能。
    /// 松弛迭代有助于减少求解后的速度误差。
    /// 默认值：(solver: 6, relaxation: 4)。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 若 <c>solver</c> 小于 1 或 <c>relaxation</c> 为负则引发。
    /// </exception>
    public (int solver, int relaxation) SolverIterations
    {
        get => (solverIterations, velocityRelaxations);
        set
        {
            if (value.solver < 1)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(value.solver), value.solver,
                    "Solver iterations cannot be smaller than one.");
            }

            if (value.relaxation < 0)
            {
                throw new ArgumentOutOfRangeException(
                    nameof(value.relaxation), value.relaxation,
                    "Relaxation iterations cannot be smaller than zero.");
            }

            solverIterations = value.solver;
            velocityRelaxations = value.relaxation;
        }
    }

    /// <summary>
    /// 每次调用 <see cref="World.Step(Real, bool)"/> 时的子步数量。
    /// 设为 1 时停用子步。
    /// 默认值：1。
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当所赋的值小于 1 时引发。
    /// </exception>
    public int SubstepCount
    {
        get => substeps;
        set
        {
            if (value < 1)
            {
                throw new ArgumentOutOfRangeException(nameof(value),
                    "The number of substeps must be greater than zero.");
            }

            substeps = value;
        }
    }

    /// <summary>
    /// 默认重力，另请参见 <see cref="RigidBody.AffectedByGravity"/>。
    /// 默认值：(0, -9.81, 0)。
    /// </summary>
    public JVector Gravity
    {
        get => gravity;
        set
        {
            DebugCheck.IsFinite(value, nameof(value));
            gravity = value;
        }
    }

    // 将其设为全局，因为 World.Step 中调用的几乎每个方法都会用到它。
    private JVector gravity = new(0, -(Real)9.81, 0);
    private int solverIterations = 6;
    private int velocityRelaxations = 4;
    private int substeps = 1;

    private TimeStep timeStep = new((Real)0.01);
    // 最近一次宽相更新所用的 dt；在首次 Step 之前，允许进行一次保守的扫掠。
    private Real broadphaseStepDt = (Real)0.1;

    internal Real BroadphaseStepDt => broadphaseStepDt;

    /// <summary>
    /// 创建 World 类的实例。
    /// </summary>
    public World()
    {
        Debug.Assert(Unsafe.SizeOf<RigidBodyData>() == SolverPrefetchCacheLineSize * (Precision.IsDoublePrecision ? 4 : 2),
            "RigidBodyData size should match the cache lines prefetched by the solver.");

        Logger.Information($"Creating new world.");

        memRigidBodies = new PartitionedBuffer<RigidBodyData>(aligned64: true);
        memContacts = new PartitionedBuffer<ContactData>();
        memConstraints = new PartitionedBuffer<ConstraintData>();
        memSmallConstraints = new PartitionedBuffer<SmallConstraintData>();

        NullBody = CreateRigidBody();
        NullBody.MotionType = MotionType.Static;

        DynamicTree = new DynamicTree(DefaultDynamicTreeFilter);

        InitParallelCallbacks();
    }

    /// <summary>
    /// DynamicTree 的默认过滤器函数。拒绝两个附加到同一刚体的 <see cref="RigidBodyShape"/> 代理；
    /// 接受其他代理对。
    /// </summary>
    public static bool DefaultDynamicTreeFilter(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB)
    {
        if (proxyA is RigidBodyShape rbsA && proxyB is RigidBodyShape rbsB)
        {
            return rbsA.RigidBody != rbsB.RigidBody;
        }

        return true;
    }

    /// <summary>
    /// 从模拟世界中移除所有实体。同时清除动态树中的所有代理，
    /// 包括不属于该世界的用户自行添加的代理。
    /// </summary>
    public void Clear()
    {
        ThrowIfDisposed();

        // 创建副本，因为我们要修改列表
        Stack<RigidBody> bodyStack = new(bodies);
        while (bodyStack.Count > 0) Remove(bodyStack.Pop());

        if (NullBody.Shapes.Count > 0)
        {
            NullBody.RemoveShapes(NullBody.Shapes, MassInertiaUpdateMode.Preserve);
        }

        // 未与任何刚体关联的剩余形状。
        Stack<IDynamicTreeProxy> proxies = new(DynamicTree.Proxies);
        while (proxies.Count > 0) DynamicTree.RemoveProxy(proxies.Pop());

    }

    /// <summary>
    /// 缩减世界内部存储中的多余容量。
    /// </summary>
    /// <remarks>
    /// 此方法会修剪世界级别的集合、宽相临时存储、岛池、确定性
    /// 求解器临时容器，以及每个刚体/每个岛的集合底层存储。
    ///
    /// <para>
    /// 现有刚体、约束、接触和仲裁器保持有效。此方法不会压缩
    /// 非托管模拟缓冲区。
    /// </para>
    ///
    /// <para>
    /// 应将修剪视为独占的维护操作。不要与 <see cref="Step(Real, bool)"/>、
    /// 接触注册、刚体/约束/形状修改或宽相查询并发调用。
    /// </para>
    /// </remarks>
    /// <exception cref="ObjectDisposedException">若此世界已被释放则引发。</exception>
    public void Trim()
    {
        ThrowIfDisposed();

        bodies.Trim();
        islands.Trim();

        deferredArbiters.Trim();
        brokenArbiters.Trim();
        dispatchArbiters.TrimExcess();
        retiredArbiters.TrimExcess();

        arbiters.TrimExcess();

        foreach (Island island in islandPool)
        {
            island.TrimLists();
        }

        islandPool.TrimExcess();

        DynamicTree.Trim();

        handleToIsland.TrimExcess();
        islandRanges.TrimExcess();
        sortedContacts.TrimExcess();
        sortedSmallConstraints.TrimExcess();
        sortedConstraints.TrimExcess();

        foreach (RigidBody body in bodies)
        {
            body.TrimLists();
        }

        foreach (Island island in islands)
        {
            island.TrimLists();
        }
    }

    /// <summary>
    /// 从世界中移除指定的刚体。此操作还会自动丢弃所有关联的接触
    /// 和约束。
    /// </summary>
    /// <param name="body">要移除的刚体。</param>
    /// <exception cref="ArgumentNullException">若 <paramref name="body"/> 为 <see langword="null"/> 则引发。</exception>
    /// <exception cref="ArgumentException">若 <paramref name="body"/> 不属于此世界则引发。</exception>
    /// <exception cref="InvalidOperationException">若 <paramref name="body"/> 已被移除则引发。</exception>
    /// <exception cref="ObjectDisposedException">若此世界已被释放则引发。</exception>
    public void Remove(RigidBody body)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(body);

        if (body.World != this)
            throw new ArgumentException("The body does not belong to this world.", nameof(body));

        if (body == NullBody) return;

        if (!body.IsValid)
            throw new InvalidOperationException("The body has already been removed from this world.");

        // 无需先复制 hashset 的内容。边迭代边移除已不再
        // 使枚举器失效，参见 https://github.com/dotnet/runtime/pull/37180
        // 这对我们非常方便。

        foreach (var constraint in body.InternalConstraints)
        {
            Remove(constraint);
        }

        foreach (var shape in body.Shapes)
        {
            DynamicTree.RemoveProxy(shape);
            shape.RigidBody = null!;
        }

        for (int i = deferredArbiters.Count - 1; i >= 0; i--)
        {
            Arbiter arbiter = deferredArbiters[i];
            if (arbiter.Body1 == body || arbiter.Body2 == body) Remove(arbiter);
        }

        foreach (var contact in body.InternalContacts)
        {
            Remove(contact);
        }

        memRigidBodies.Free(body.Handle);

        // 我们必须是自己的岛。
        Debug.Assert(body.InternalIsland is { InternalBodies.Count: 1 });

        body.Handle = JHandle<RigidBodyData>.Zero;

        IslandHelper.BodyRemoved(islands, islandPool, body);

        body.InternalIsland = null!;

        bodies.Remove(body);
    }

    /// <summary>
    /// 从世界中移除指定的约束。如需临时停用约束，请考虑使用
    /// <see cref="Constraint.IsEnabled"/> 属性。
    /// </summary>
    /// <param name="constraint">要移除的约束。</param>
    /// <exception cref="ArgumentNullException">若 <paramref name="constraint"/> 为 <see langword="null"/> 则引发。</exception>
    /// <exception cref="ArgumentException">
    /// 若 <paramref name="constraint"/> 不属于此世界则引发。
    /// </exception>
    /// <exception cref="InvalidOperationException">若该约束已被移除则引发。</exception>
    /// <exception cref="ObjectDisposedException">若此世界已被释放则引发。</exception>
    public void Remove(Constraint constraint)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(constraint);

        if (constraint.Body1.World != this)
            throw new ArgumentException("The constraint does not belong to this world.", nameof(constraint));

        if (!constraint.IsValid)
            throw new InvalidOperationException("The constraint has already been removed from this world.");

        ActivateBodyNextStep(constraint.Body1);
        ActivateBodyNextStep(constraint.Body2);

        IslandHelper.ConstraintRemoved(islands, islandPool, constraint);

        if (constraint.IsSmallConstraint)
        {
            memSmallConstraints.Free(constraint.SmallHandle);
        }
        else
        {
            memConstraints.Free(constraint.Handle);
        }

        constraint.Handle = JHandle<ConstraintData>.Zero;
        ConstraintRemoved?.Invoke(constraint);
    }

    /// <summary>
    /// 从世界中移除特定的仲裁器。
    /// </summary>
    /// <param name="arbiter">要移除的仲裁器。</param>
    /// <exception cref="ArgumentNullException">若 <paramref name="arbiter"/> 为 <see langword="null"/> 则引发。</exception>
    /// <exception cref="ArgumentException">
    /// 若 <paramref name="arbiter"/> 不属于此世界则引发。
    /// </exception>
    /// <exception cref="ObjectDisposedException">若此世界已被释放则引发。</exception>
    public void Remove(Arbiter arbiter)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(arbiter);

        if (arbiter.Body1.World != this)
            throw new ArgumentException("The arbiter does not belong to this world.", nameof(arbiter));

        ActivateBodyNextStep(arbiter.Body1);
        ActivateBodyNextStep(arbiter.Body2);

        bool pending = false;
        for (int i = 0; i < deferredArbiters.Count; i++)
        {
            if (deferredArbiters[i] != arbiter) continue;
            deferredArbiters.RemoveAt(i);
            pending = true;
            break;
        }

        if (!pending) IslandHelper.ArbiterRemoved(islands, islandPool, arbiter);
        arbiters.Remove(arbiter.Handle.Data.Key);

        brokenArbiters.Remove(arbiter.Handle);
        memContacts.Free(arbiter.Handle);

        ReturnArbiter(arbiter);
    }

    internal void RemoveDeferredArbitersForShape(RigidBody body, ulong shapeId)
    {
        for (int i = deferredArbiters.Count - 1; i >= 0; i--)
        {
            Arbiter arbiter = deferredArbiters[i];
            if (arbiter.Body1 != body && arbiter.Body2 != body) continue;

            ArbiterKey key = arbiter.Handle.Data.Key;
            if (key.Key1 == shapeId || key.Key2 == shapeId) Remove(arbiter);
        }
    }

    /// <summary>
    /// 标记一个刚体在下一步开始时被激活。
    /// </summary>
    /// <param name="body">要激活的刚体。</param>
    /// <param name="wakeUpStatic">当意图是移动静态刚体，或从其他 MotionType 切换为静态时，
    /// 设为 true。此时会激活所有相连的刚体。</param>
    internal void ActivateBodyNextStep(RigidBody body, bool wakeUpStatic = false)
    {
        body.InternalSleepTime = 0;

        if (body.IsActive) return;

        if (body.MotionType == MotionType.Static && !wakeUpStatic) return;

        AddToActiveList(body.InternalIsland);

        if (body.MotionType == MotionType.Static)
        {
            foreach (var c in body.Constraints)
            {
                ActivateBodyNextStep(c.Body1 == body ? c.Body2 : c.Body1);
            }

            foreach (var c in body.Contacts)
            {
                ActivateBodyNextStep(c.Body1 == body ? c.Body2 : c.Body1);
            }
        }

        body.Island.NeedsUpdate = true;
    }

    /// <summary>
    /// 移除连接两个非动态刚体的接触。
    /// </summary>
    /// <param name="body">应移除其非动态接触的刚体。</param>
    internal void RemoveStaticStaticContacts(RigidBody body)
    {
        foreach (var arbiter in body.InternalContacts)
        {
            if (arbiter.Body1.Data.MotionType != MotionType.Dynamic &&
                arbiter.Body2.Data.MotionType != MotionType.Dynamic)
            {
                Remove(arbiter);
            }
        }
    }

    /// <summary>
    /// 仅当动态端点处于唤醒状态时，才让约束保留在求解器的活动分区中。
    /// 这不会改变约束的启用状态或其岛连接。
    /// </summary>
    internal void UpdateConstraintActivation(Constraint constraint)
    {
        RigidBody body1 = constraint.Body1;
        RigidBody body2 = constraint.Body2;
        bool active = (body1.MotionType == MotionType.Dynamic && body1.IsActive) ||
                      (body2.MotionType == MotionType.Dynamic && body2.IsActive);

        if (constraint.IsSmallConstraint)
        {
            if (active) memSmallConstraints.MoveToActive(constraint.SmallHandle);
            else memSmallConstraints.MoveToInactive(constraint.SmallHandle);
        }
        else
        {
            if (active) memConstraints.MoveToActive(constraint.Handle);
            else memConstraints.MoveToInactive(constraint.Handle);
        }
    }

    /// <summary>
    /// 根据刚体现有的接触和约束重建岛连接。
    /// </summary>
    /// <param name="body">应重建其连接的刚体。</param>
    internal void BuildConnectionsFromExistingContacts(RigidBody body)
    {
        foreach (var constraint in body.InternalConstraints)
        {
            IslandHelper.AddConnection(islands, islandPool, constraint.Body1, constraint.Body2);
        }

        foreach (var contact in body.InternalContacts)
        {
            IslandHelper.AddConnection(islands, islandPool, contact.Body1, contact.Body2);
        }
    }

    /// <summary>
    /// 移除刚体的所有岛连接。
    /// </summary>
    /// <param name="body">应移除其连接的刚体。</param>
    internal void RemoveConnections(RigidBody body)
    {
        while (body.InternalConnections.Count > 0)
        {
            IslandHelper.RemoveConnection(islands, islandPool, body, body.InternalConnections[^1]);
        }

        Debug.Assert(body.InternalConnections.Count == 0);
        Debug.Assert(body.InternalIsland.InternalBodies.Count == 1);
    }

    /// <summary>
    /// 标记一个刚体在下一步开始时被停用。
    /// </summary>
    /// <param name="body">要停用的刚体。</param>
    internal void DeactivateBodyNextStep(RigidBody body)
    {
        body.InternalSleepTime = Real.PositiveInfinity;
    }

    /// <summary>
    /// 立即停用指定的岛，并清除其中所有刚体的运动状态。
    /// </summary>
    /// <remarks>
    /// 此操作忽略 <see cref="AllowDeactivation"/>，且不等待下一个
    /// 模拟步。岛中的动态和运动学刚体的线速度与角速度、
    /// 排队中的力、排队中的力矩以及由力引起的速度增量都会被置零。
    /// </remarks>
    /// <param name="island">要停用的岛。</param>
    /// <exception cref="ArgumentNullException">若 <paramref name="island"/> 为 <see langword="null"/> 则引发。</exception>
    /// <exception cref="ArgumentException">若 <paramref name="island"/> 不属于此世界则引发。</exception>
    public void ForceSleepIsland(Island island)
    {
        ThrowIfDisposed();
        ArgumentNullException.ThrowIfNull(island);

        if (!islands.Contains(island))
        {
            throw new ArgumentException("The island does not belong to this world.", nameof(island));
        }

        foreach (RigidBody body in island.InternalBodies)
        {
            ClearMotionForSleep(body);
        }

        island.MarkedAsActive = false;
        island.NeedsUpdate = false;

        if (!islands.IsActive(island)) return;

        bool deactivatedBody = false;

        foreach (RigidBody body in island.InternalBodies)
        {
            deactivatedBody |= DeactivateBodyForSleep(body);
        }

        islands.MoveToInactive(island);

        if (deactivatedBody)
        {
            IslandDeactivated?.Invoke(island);
        }
    }

    /// <summary>
    /// 构造指定类型的约束。创建后，通过调用其 <c>Initialize</c> 方法
    /// 初始化该约束。
    /// </summary>
    /// <typeparam name="T">要创建的约束的具体类型。</typeparam>
    /// <param name="body1">参与该约束的第一个刚体。</param>
    /// <param name="body2">参与该约束的第二个刚体。</param>
    /// <returns>指定约束类型的新实例，且已向世界注册。</returns>
    /// <exception cref="ArgumentException">
    /// 若任一刚体不属于此世界则引发。
    /// </exception>
    /// <exception cref="SameBodyException">
    /// 若 <paramref name="body1"/> 和 <paramref name="body2"/> 相同则引发。
    /// </exception>
    /// <exception cref="PartitionedBuffer{T}.MaximumSizeException">当超出约束缓冲区最大大小时引发。</exception>
    public T CreateConstraint<T>(RigidBody body1, RigidBody body2) where T : Constraint, new()
    {
        ThrowIfDisposed();

        if (body1.World != this)
            throw new ArgumentException("The body does not belong to this world.", nameof(body1));
        if (body2.World != this)
            throw new ArgumentException("The body does not belong to this world.", nameof(body2));
        if (!body1.IsValid || !body2.IsValid)
            throw new InvalidOperationException("The body has been removed from this world.");
        if (ReferenceEquals(body1, body2))
            throw new SameBodyException();

        T constraint = new();
        ulong constraintId = RequestId();

        if (constraint.IsSmallConstraint)
        {
            constraint.Create(memSmallConstraints.Allocate(false, true), body1, body2);
            constraint.SmallHandle.Data.ConstraintId = constraintId;
        }
        else
        {
            constraint.Create(memConstraints.Allocate(false, true), body1, body2);
            constraint.Handle.Data.ConstraintId = constraintId;
        }

        IslandHelper.ConstraintCreated(islands, islandPool, constraint);

        AddToActiveList(body1.InternalIsland);
        AddToActiveList(body2.InternalIsland);

        UpdateConstraintActivation(constraint);

        return constraint;
    }

    /// <summary>
    /// 将岛标记为活动，并将其移动到活动分区。
    /// </summary>
    /// <param name="island">要激活的岛。</param>
    private void AddToActiveList(Island island)
    {
        bool wasActive = islands.IsActive(island);
        island.MarkedAsActive = true;
        if (!wasActive) island.NeedsUpdate = true;

        islands.MoveToActive(island);
    }

    [Conditional("DEBUG")]
    private void AssertIslandActivationInvariant(Island island)
    {
        if (island.NeedsUpdate) return;

        bool shouldBeActive = islands.IsActive(island);

        foreach (RigidBody body in island.InternalBodies)
        {
            if (body.Data.MotionType == MotionType.Static) continue;

            Debug.Assert(body.Data.IsActive == shouldBeActive,
                "Islands without pending updates must not contain mixed dynamic body activation state.");
        }
    }

    [Conditional("DEBUG")]
    private void AssertIslandActivationInvariants()
    {
        for (int i = 0; i < islands.Count; i++)
        {
            AssertIslandActivationInvariant(islands[i]);
        }
    }

    /// <summary>
    /// 创建新的刚体并将其添加到模拟世界。
    /// </summary>
    /// <returns>新创建的 <see cref="RigidBody"/> 实例。</returns>
    /// <exception cref="PartitionedBuffer{T}.MaximumSizeException">当超出刚体缓冲区最大大小时引发。</exception>
    public RigidBody CreateRigidBody()
    {
        ThrowIfDisposed();
        RigidBody body = new(memRigidBodies.Allocate(true, true), this);
        body.Data.IsActive = true;

        bodies.Add(body, true);

        IslandHelper.BodyAdded(islands, islandPool, body);

        AddToActiveList(body.InternalIsland);

        return body;
    }

    private bool disposed;

    /// <summary>
    /// 获取一个值，指示此世界是否已被释放。
    /// </summary>
    public bool IsDisposed => disposed;

    private void ThrowIfDisposed()
    {
        ObjectDisposedException.ThrowIf(disposed, this);
    }

    /// <summary>
    /// 释放此模拟世界使用的所有非托管内存缓冲区。
    /// </summary>
    /// <remarks>
    /// 释放后，世界实例不可再使用。所有刚体、约束和接触
    /// 都将失效。多次调用 <see cref="Dispose"/> 是安全的。
    /// </remarks>
    public void Dispose()
    {
        if (disposed) return;
        disposed = true;

        memContacts.Dispose();
        memRigidBodies.Dispose();
        memConstraints.Dispose();
        memSmallConstraints.Dispose();

        deferredContacts.Dispose();
        deferredConstraints.Dispose();
        deferredSmallConstraints.Dispose();
    }
}
