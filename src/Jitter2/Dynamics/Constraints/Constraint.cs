/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Threading;
using Jitter2.LinearMath;
using Jitter2.Unmanaged;

namespace Jitter2.Dynamics.Constraints;

/// <summary>
/// 适用于占用空间不超过 <see cref="Precision.ConstraintSizeSmall"/> 字节的约束的底层数据。
/// </summary>
/// <remarks>
/// <para>
/// 此结构存储于非托管内存中，通过 <see cref="Constraint.SmallHandle"/> 访问。
/// 它包含求解器调度 id、稳定的约束标识符，以及所连接刚体的句柄。
/// </para>
/// <para>
/// 仅当该约束已注册到世界时数据才有效。不要跨模拟步缓存引用。
/// 不可与 <see cref="World.Step(Real, bool)"/> 并发访问。
/// </para>
/// </remarks>
[StructLayout(LayoutKind.Sequential, Size = Precision.ConstraintSizeSmall)]
public unsafe struct SmallConstraintData
{
    internal int _internal;
    /// <summary>此约束所注册的求解器调度对的标识符。</summary>
    public uint DispatchId;
    /// <summary>用于确定性排序的稳定创建顺序标识符。</summary>
    public ulong ConstraintId;

    /// <summary>第一个刚体模拟数据的句柄。</summary>
    public JHandle<RigidBodyData> Body1;
    /// <summary>第二个刚体模拟数据的句柄。</summary>
    public JHandle<RigidBodyData> Body2;

    /// <summary>获取此约束是否已启用。</summary>
    public readonly bool IsEnabled => DispatchId != 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void PrepareForIteration(ref SmallConstraintData constraint, in TimeStep timeStep)
    {
        ref readonly var dispatch = ref ConstraintDispatchTable.Get(DispatchId);
        ((delegate*<ref SmallConstraintData, in TimeStep, void>)dispatch.Prepare)(ref constraint, timeStep);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void Iterate(ref SmallConstraintData constraint, in TimeStep timeStep)
    {
        ref readonly var dispatch = ref ConstraintDispatchTable.Get(DispatchId);
        ((delegate*<ref SmallConstraintData, in TimeStep, void>)dispatch.Iterate)(ref constraint, timeStep);
    }
}

/// <summary>
/// 约束的底层数据，存储于非托管内存中。
/// </summary>
/// <remarks>
/// <para>
/// 此结构存储于非托管内存中，通过 <see cref="Constraint.Handle"/> 访问。
/// 它包含求解器调度 id、稳定的约束标识符，以及所连接刚体的句柄。
/// </para>
/// <para>
/// 仅当该约束已注册到世界时数据才有效。不要跨模拟步缓存引用。
/// 不可与 <see cref="World.Step(Real, bool)"/> 并发访问。
/// </para>
/// </remarks>
[StructLayout(LayoutKind.Sequential, Size = Precision.ConstraintSizeFull)]
public unsafe struct ConstraintData
{
    internal int _internal;
    /// <summary>此约束所注册的求解器调度对的标识符。</summary>
    public uint DispatchId;
    /// <summary>用于确定性排序的稳定创建顺序标识符。</summary>
    public ulong ConstraintId;

    /// <summary>第一个刚体模拟数据的句柄。</summary>
    public JHandle<RigidBodyData> Body1;
    /// <summary>第二个刚体模拟数据的句柄。</summary>
    public JHandle<RigidBodyData> Body2;

    /// <summary>获取此约束是否已启用。</summary>
    public readonly bool IsEnabled => DispatchId != 0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void PrepareForIteration(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref readonly var dispatch = ref ConstraintDispatchTable.Get(DispatchId);
        ((delegate*<ref ConstraintData, in TimeStep, void>)dispatch.Prepare)(ref constraint, timeStep);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public readonly void Iterate(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref readonly var dispatch = ref ConstraintDispatchTable.Get(DispatchId);
        ((delegate*<ref ConstraintData, in TimeStep, void>)dispatch.Iterate)(ref constraint, timeStep);
    }
}

internal static unsafe class ConstraintDispatchTable
{
    internal readonly struct Entry(nint prepare, nint iterate)
    {
        public readonly nint Prepare = prepare;
        public readonly nint Iterate = iterate;
    }

    private static readonly object Sync = new();
    private static Entry[] entries = [default];

    public static uint Register(
        delegate*<ref ConstraintData, in TimeStep, void> prepare,
        delegate*<ref ConstraintData, in TimeStep, void> iterate)
    {
        if (prepare == null) throw new ArgumentNullException(nameof(prepare));
        if (iterate == null) throw new ArgumentNullException(nameof(iterate));

        lock (Sync)
        {
            return Register((nint)prepare, (nint)iterate);
        }
    }

    public static uint Register(
        delegate*<ref SmallConstraintData, in TimeStep, void> prepare,
        delegate*<ref SmallConstraintData, in TimeStep, void> iterate)
    {
        if (prepare == null) throw new ArgumentNullException(nameof(prepare));
        if (iterate == null) throw new ArgumentNullException(nameof(iterate));

        lock (Sync)
        {
            return Register((nint)prepare, (nint)iterate);
        }
    }

    private static uint Register(nint prepare, nint iterate)
    {
        Entry[] current = entries;
        if (current.Length == int.MaxValue)
        {
            throw new InvalidOperationException("Too many registered constraint dispatch entries.");
        }

        Entry[] next = new Entry[current.Length + 1];
        Array.Copy(current, next, current.Length);
        next[current.Length] = new Entry(prepare, iterate);

        Volatile.Write(ref entries, next);
        return (uint)current.Length;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static ref readonly Entry Get(uint dispatchId)
    {
        Entry[] table = Volatile.Read(ref entries);
        return ref table[(int)dispatchId];
    }
}

/// <summary>
/// 用于存储类型为 <typeparamref name="T"/> 的自定义数据的约束的泛型基类。
/// </summary>
/// <typeparam name="T">
/// 包含约束特定状态的非托管数据结构。必须能容纳于
/// <see cref="ConstraintData"/>（即 <see cref="Precision.ConstraintSizeFull"/> 字节）之内。
/// </typeparam>
/// <remarks>
/// 从此类派生可创建具有自定义数据布局的约束。<see cref="Data"/>
/// 属性提供对约束非托管内存的类型化访问。
/// </remarks>
public abstract class Constraint<T> : Constraint where T : unmanaged
{
    /// <summary>
    /// 获取对存储于非托管内存中的约束类型化数据的引用。
    /// </summary>
    public ref T Data
    {
        get
        {
            Debug.Assert(IsValid);
            return ref JHandle<ConstraintData>.AsHandle<T>(Handle).Data;
        }
    }

    /// <inheritdoc />
    public override unsafe bool IsSmallConstraint => sizeof(T) <= sizeof(SmallConstraintData);

    /// <inheritdoc />
    protected override void Create() => Constraint.CheckDataSize<T>();
}

/// <summary>
/// 连接两个刚体并限制其相对运动的约束的基类。
/// </summary>
public abstract class Constraint : IDebugDrawable
{
    /// <summary>角度约束的默认柔软度（柔量）。</summary>
    public const Real DefaultAngularSoftness = (Real)0.001;

    /// <summary>角度误差修正的默认偏置因子。</summary>
    public const Real DefaultAngularBias = (Real)0.2;

    /// <summary>角度限位施加的默认柔软度。</summary>
    public const Real DefaultAngularLimitSoftness = (Real)0.001;

    /// <summary>角度限位修正的默认偏置因子。</summary>
    public const Real DefaultAngularLimitBias = (Real)0.1;

    /// <summary>线性约束的默认柔软度（柔量）。</summary>
    public const Real DefaultLinearSoftness = (Real)0.00001;

    /// <summary>线性误差修正的默认偏置因子。</summary>
    public const Real DefaultLinearBias = (Real)0.2;

    /// <summary>线性限位施加的默认柔软度。</summary>
    public const Real DefaultLinearLimitSoftness = (Real)0.0001;

    /// <summary>线性限位修正的默认偏置因子。</summary>
    public const Real DefaultLinearLimitBias = (Real)0.2;


    /// <summary>获取此约束连接的第一个刚体。</summary>
    public RigidBody Body1 { private set; get; } = null!;

    /// <summary>获取此约束连接的第二个刚体。</summary>
    public RigidBody Body2 { private set; get; } = null!;

    /// <summary>
    /// 指向自定义用户数据的托管指针。引擎不使用此项。
    /// </summary>
    public object? Tag { get; set; }

    /// <summary>获取此约束是否使用较小的数据布局。</summary>
    public virtual bool IsSmallConstraint { get; } = false;

    /// <summary>
    /// 用于访问原始约束数据的句柄。
    /// </summary>
    public JHandle<ConstraintData> Handle { internal set; get; }

    /// <summary>
    /// 获取一个值，指示底层约束数据是否仍然有效。
    /// </summary>
    public bool IsValid => !Handle.IsZero && !Body1.World.IsDisposed;

    /// <summary>
    /// 获取重新解释为 <see cref="SmallConstraintData"/> 的约束数据句柄。
    /// </summary>
    public JHandle<SmallConstraintData> SmallHandle => JHandle<ConstraintData>.AsHandle<SmallConstraintData>(Handle);

    /// <summary>
    /// 检查约束数据类型是否能容纳于 <see cref="ConstraintData"/> 之内。
    /// </summary>
    /// <typeparam name="T">约束特定的数据类型。</typeparam>
    /// <exception cref="InvalidOperationException">当数据类型过大时抛出。</exception>
    protected static unsafe void CheckDataSize<T>() where T : unmanaged
    {
        if (sizeof(T) > sizeof(ConstraintData))
        {
            throw new InvalidOperationException("The size of the constraint data is too large.");
        }
    }

    /// <summary>
    /// 验证此约束是否已通过 World 类正确创建。
    /// </summary>
    /// <exception cref="InvalidOperationException">
    /// 当约束是直接实例化而非通过
    /// <see cref="World.CreateConstraint{T}(RigidBody, RigidBody)"/> 创建时抛出。
    /// </exception>
    protected void VerifyNotZero()
    {
        if (Handle.IsZero)
        {
            throw new InvalidOperationException(
                $"The constraint has not been created by the world. " +
                $"Use World.CreateConstraint<{GetType().Name}>() to create constraints.");
        }
    }

    /// <summary>
    /// 设置此实例使用的求解器调度 id。在派生类中重写此方法
    /// 以指定正确的已注册求解器对。当约束启用时，该 id 稍后会被写入
    /// 非托管头部。
    /// </summary>
    protected virtual void Create()
    {
    }

    /// <summary>
    /// 重置求解器为此约束缓存的热启动状态。
    /// </summary>
    /// <remarks>
    /// 此操作仅清除持久化的求解器冲量。约束配置保持不变。
    /// 在恢复快照或其他不连续状态变化之后，若不希望保留
    /// 热启动，则此方法很有用。
    /// </remarks>
    public virtual void ResetWarmStart()
    {
    }

    protected uint DispatchId { get; set; }

    protected static unsafe uint RegisterFullConstraint(
        delegate*<ref ConstraintData, in TimeStep, void> prepare,
        delegate*<ref ConstraintData, in TimeStep, void> iterate)
    {
        return ConstraintDispatchTable.Register(prepare, iterate);
    }

    protected static unsafe uint RegisterSmallConstraint(
        delegate*<ref SmallConstraintData, in TimeStep, void> prepare,
        delegate*<ref SmallConstraintData, in TimeStep, void> iterate)
    {
        return ConstraintDispatchTable.Register(prepare, iterate);
    }

    /// <summary>
    /// 获取或设置此约束是否已启用。
    /// </summary>
    /// <remarks>
    /// 若要永久移除约束，请使用 <see cref="World.Remove(Constraint)"/>。
    /// 已启用的约束仅当至少一个所连接刚体为动态且处于唤醒状态时才会被求解。
    /// 更改刚体运动类型或激活状态不会改变此属性。
    /// </remarks>
    public unsafe bool IsEnabled
    {
        get
        {
            Debug.Assert(IsValid);
            return Handle.Data.DispatchId != 0;
        }
        set
        {
            Debug.Assert(IsValid);

            if (value && DispatchId == 0)
            {
                throw new InvalidOperationException(
                    $"The constraint has no registered solver dispatch. " +
                    $"Set {nameof(DispatchId)} in {nameof(Create)}().");
            }

            Handle.Data.DispatchId = value ? DispatchId : 0u;
        }
    }

    internal void Create(JHandle<SmallConstraintData> handle, RigidBody body1, RigidBody body2)
    {
        var cd = JHandle<SmallConstraintData>.AsHandle<ConstraintData>(handle);
        Create(cd, body1, body2);
    }

    internal void Create(JHandle<ConstraintData> handle, RigidBody body1, RigidBody body2)
    {
        Body1 = body1;
        Body2 = body2;
        Handle = handle;

        handle.Data.Body1 = body1.Handle;
        handle.Data.Body2 = body2.Handle;

        Create();

        IsEnabled = true;
    }

    /// <summary>
    /// 绘制此约束的调试可视化。
    /// </summary>
    /// <param name="drawer">接收可视化图元的调试绘制器。</param>
    /// <exception cref="NotImplementedException">当派生类未重写此方法时抛出。</exception>
    public virtual void DebugDraw(IDebugDrawer drawer)
    {
        throw new NotImplementedException();
    }

    /// <summary>
    /// 使用提供的可视化设置绘制此约束。
    /// 仅重写 <see cref="DebugDraw(IDebugDrawer)"/> 的自定义约束将使用其默认可视化。
    /// </summary>
    public virtual void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings) => DebugDraw(drawer);

    /// <summary>获取角度约束的显示原点，使世界相对参考线保持在刚体 2 附近。</summary>
    protected JVector GetDebugDrawOrigin(in DebugDrawSettings settings) =>
        settings.Origin ?? (Body1 == Body1.World.NullBody ? Body2.Position : Body1.Position);
}
