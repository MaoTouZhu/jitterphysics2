/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.DataStructures;
using Jitter2.Dynamics.Constraints;
using Jitter2.LinearMath;
using Jitter2.Unmanaged;

namespace Jitter2.Dynamics;

/// <summary>
/// 允许的平移与绕世界空间坐标轴的旋转。
/// 由 <see cref="RigidBody.AllowedMotion"/> 使用。
/// </summary>
[Flags]
public enum MotionAxes
{
    /// <summary>无模拟平移或旋转。</summary>
    None = 0,
    /// <summary>沿世界 X 轴的平移。</summary>
    LinearX = 1 << 0,
    /// <summary>沿世界 Y 轴的平移。</summary>
    LinearY = 1 << 1,
    /// <summary>沿世界 Z 轴的平移。</summary>
    LinearZ = 1 << 2,
    /// <summary>绕世界 X 轴的旋转。</summary>
    AngularX = 1 << 3,
    /// <summary>绕世界 Y 轴的旋转。</summary>
    AngularY = 1 << 4,
    /// <summary>绕世界 Z 轴的旋转。</summary>
    AngularZ = 1 << 5,
    /// <summary>沿所有世界轴的平移。</summary>
    Linear = LinearX | LinearY | LinearZ,
    /// <summary>绕所有世界轴的旋转。</summary>
    Angular = AngularX | AngularY | AngularZ,
    /// <summary>在 XY 平面内平移并绕 Z 轴旋转。</summary>
    PlaneXY = LinearX | LinearY | AngularZ,
    /// <summary>在 XZ 平面内平移并绕 Y 轴旋转。</summary>
    PlaneXZ = LinearX | LinearZ | AngularY,
    /// <summary>在 YZ 平面内平移并绕 X 轴旋转。</summary>
    PlaneYZ = LinearY | LinearZ | AngularX,
    /// <summary>无限制的平移与旋转（默认值）。</summary>
    All = Linear | Angular
}

/// <summary>
/// 指定添加/移除形状的操作是否重新计算刚体的质量和惯性。
/// </summary>
public enum MassInertiaUpdateMode
{
    /// <summary>
    /// 根据当前附加的形状重新计算质量和惯性。
    /// </summary>
    Update = 0,

    /// <summary>
    /// 保持当前质量和惯性不变。
    /// </summary>
    Preserve = 1
}

/// <summary>
/// 指定刚体参与模拟的方式。
/// </summary>
public enum MotionType
{
    /// <summary>
    /// 完全参与模拟；响应力、冲量、接触和约束。
    /// </summary>
    Dynamic = 0,

    /// <summary>
    /// 由用户控制的刚体，不受力或碰撞影响，但可以影响动态刚体。
    /// 在求解器中视为具有无限质量。可由用户代码设置非零速度。
    /// 参与碰撞岛的构建。
    /// </summary>
    Kinematic = 1,

    /// <summary>
    /// 不可移动的刚体（速度为零），在求解器中视为具有无限质量。
    /// 位置和朝向可由用户代码直接修改，这会更新宽相，
    /// 并可能影响下一步的接触。
    /// </summary>
    Static = 2
}

/// <summary>
/// <see cref="RigidBody"/> 的底层模拟状态，存储在非托管内存中。
/// </summary>
/// <remarks>
/// 此结构对布局敏感，仅供引擎内部使用。请优先使用
/// <see cref="RigidBody"/> 的属性，而不是直接访问字段。
/// 所有空间值（位置、速度、朝向、惯性）均在世界空间中。
/// <see cref="Flags"/> 字段是一个位域：位 0–1 编码 <see cref="MotionType"/>，
/// 位 2 表示活动状态，位 3 启用陀螺力，位 4–9 编码运动锁定。
/// </remarks>
[StructLayout(LayoutKind.Explicit, Size = Precision.RigidBodyDataSize)]
public struct RigidBodyData
{
    /// <summary>
    /// 引擎用于句柄管理的内部索引。在不同帧之间不稳定。
    /// </summary>
    [FieldOffset(0)]
    public int _index;

    /// <summary>
    /// 引擎使用的内部同步标志。请勿修改。
    /// </summary>
    [FieldOffset(4)]
    public int _lockFlag;

    /// <summary>
    /// 刚体的世界空间位置（质心）。
    /// </summary>
    [FieldOffset(8 + 0 * sizeof(Real))]
    public JVector Position;

    /// <summary>
    /// 世界空间中的线性速度，单位为每秒长度单位。
    /// </summary>
    [FieldOffset(8 + 3 * sizeof(Real))]
    public JVector Velocity;

    /// <summary>
    /// 世界空间中的角速度，单位为弧度每秒。向量方向
    /// 为旋转轴，其模长为旋转速率。
    /// </summary>
    [FieldOffset(8 + 6 * sizeof(Real))]
    public JVector AngularVelocity;

    /// <summary>
    /// 每个子步施加的线性速度变化（来自力和重力），在上一步结束时准备好。
    /// 仅供内部使用。
    /// </summary>
    [FieldOffset(8 + 9 * sizeof(Real))]
    public JVector DeltaVelocity;

    /// <summary>
    /// 每个子步施加的角速度变化（来自力矩），在上一步结束时准备好。
    /// 仅供内部使用。
    /// </summary>
    [FieldOffset(8 + 12 * sizeof(Real))]
    public JVector DeltaAngularVelocity;

    /// <summary>
    /// 刚体的世界空间朝向。
    /// </summary>
    [FieldOffset(8 + 15 * sizeof(Real))]
    public JQuaternion Orientation;

    /// <summary>
    /// 世界空间中的逆惯性张量。对于动态刚体，每步都会根据
    /// 刚体空间的逆惯性和当前朝向重新计算。被锁定的角度轴
    /// 通过 Schur 补消除，以保留剩余的惯性响应。
    /// 对于静态和运动学刚体，此值为零（表示无限惯性）。
    /// </summary>
    [FieldOffset(8 + 19 * sizeof(Real))]
    public JSymmetricMatrix InverseInertiaWorld;

    /// <summary>
    /// 世界空间中的对角逆质量，包含刚体的平移锁定。
    /// 静态和运动学刚体在每个轴上的逆质量均为零。
    /// </summary>
    /// <remarks>
    /// 自定义约束应使用
    /// <see cref="JVector.Multiply(in JVector, in JVector)"/> 配合 <see cref="InverseMassVector"/>
    /// 来处理冲量。沿某方向的等效逆质量的线性贡献
    /// 为 <c>JVector.Multiply(direction, InverseMassVector) * direction</c>。
    /// </remarks>
    [FieldOffset(8 + 25 * sizeof(Real))]
    public JVector InverseMassVector;

    /// <summary>
    /// 编码运动类型（位 0–1）、活动状态（位 2）、陀螺力（位 3）
    /// 以及被锁定的世界空间自由度（位 4–9）的位域。
    /// 请使用相应的属性，而不要直接操作此字段。
    /// </summary>
    [FieldOffset(8 + 28 * sizeof(Real))]
    public int Flags;

    /// <summary>允许的世界空间运动。通过 <see cref="RigidBody.AllowedMotion"/> 设置。</summary>
    public MotionAxes AllowedMotion
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => MotionAxes.All ^ (MotionAxes)((Flags >> 4) & (int)MotionAxes.All);
        internal set => Flags = (Flags & ~((int)MotionAxes.All << 4)) | ((int)(MotionAxes.All ^ value) << 4);
    }

    internal readonly bool HasLinearLocks => (Flags & (7 << 4)) != 0;
    internal readonly bool HasAngularLocks => (Flags & (56 << 4)) != 0;
    internal readonly bool HasMotionLocks => (Flags & (63 << 4)) != 0;

    /// <summary>
    /// 获取或设置刚体是否处于活动（唤醒）状态并参与模拟。
    /// 非活动刚体被视为休眠，在重新激活之前跳过积分。
    /// </summary>
    public bool IsActive
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => (Flags & 4) != 0;
        set
        {
            if (value) Flags |= 4;
            else Flags &= ~4;
        }
    }

    /// <summary>
    /// 获取或设置是否启用隐式陀螺力矩求解器。
    /// 详见 <see cref="RigidBody.EnableGyroscopicForces"/>。
    /// </summary>
    public bool EnableGyroscopicForces
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        readonly get => (Flags & 8) != 0;
        set
        {
            if (value) Flags |= 8;
            else Flags &= ~8;
        }
    }

    /// <summary>
    /// 如果刚体不是动态的或当前处于非活动（休眠）状态，则返回 true。
    /// </summary>
    [Obsolete($"Use {nameof(MotionType)} directly.", true)]
    public bool IsStaticOrInactive => MotionType != MotionType.Dynamic || !IsActive;

    /// <summary>
    /// 获取或设置此刚体如何参与模拟。
    /// 编码在 <see cref="Flags"/> 的位 0–1 中。
    /// </summary>
    public MotionType MotionType
    {
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        get => (MotionType)(Flags & 0b11);
        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        set => Flags = (Flags & ~0b11) | (int)value;
    }
}

/// <summary>
/// 表示 Jitter 物理世界中的主要实体。
/// </summary>
public sealed class RigidBody : IPartitionedSetIndex, IDebugDrawable
{
    private JHandle<RigidBodyData> handle;

    /// <summary>
    /// 此刚体的唯一标识符，创建时由 <see cref="World"/> 分配。
    /// 此 ID 在刚体的整个生命周期内保持稳定。
    /// </summary>
    public readonly ulong RigidBodyId;

    private Real restitution = (Real)0.0;
    private Real friction = (Real)0.2;

    /// <summary>
    /// 返回此刚体非托管模拟状态的按引用视图。
    /// </summary>
    /// <remarks>
    /// 出于性能考虑，模拟数据（位置、速度等）存储在一块连续的非托管内存中。
    /// 此属性提供对该数据的直接访问。
    /// <para>
    /// <strong>使用说明：</strong>
    /// <list type="bullet">
    /// <item><description>请优先使用 <see cref="RigidBody"/> 的属性，而不是直接访问字段。</description></item>
    /// <item><description>不要跨模拟步缓存返回的引用。</description></item>
    /// <item><description>除非你清楚自己在做什么，否则直接修改字段可能破坏不变量（例如世界空间惯性）。</description></item>
    /// </list>
    /// </para>
    /// </remarks>
    public ref RigidBodyData Data
    {
        get
        {
            Debug.Assert(IsValid);
            return ref handle.Data;
        }
    }

    /// <summary>
    /// 获取指向刚体数据的句柄，参见 <see cref="Data"/>。
    /// </summary>
    public JHandle<RigidBodyData> Handle
    {
        get => handle; internal set => handle = value;
    }

    /// <summary>
    /// 获取一个值，指示底层的刚体数据是否仍然有效。
    /// </summary>
    public bool IsValid => !handle.IsZero && !World.IsDisposed;

    // 创建刚体只有一种方式：world.CreateRigidBody。在那里我们会为新刚体添加一个岛。
    // 它永远不应为 null。
    internal Island InternalIsland = null!;

    internal readonly List<RigidBodyShape> InternalShapes = new(capacity: 1);
    internal readonly List<RigidBody> InternalConnections = new(capacity: 0);
    internal readonly HashSet<Arbiter> InternalContacts = new(capacity: 0);
    internal readonly HashSet<Constraint> InternalConstraints = new(capacity: 0);

    internal int InternalIslandMarker;
    internal Real InternalSleepTime = (Real)0.0;

    /// <summary>
    /// 获取与此刚体关联的碰撞岛。
    /// </summary>
    public Island Island => InternalIsland;

    /// <summary>
    /// 当创建新的仲裁器时触发的事件，包括在刚体发生重叠之前的推测接触。
    /// </summary>
    /// <remarks>
    /// 此事件提供一个 <see cref="Arbiter"/> 对象，其中包含碰撞的详细信息。
    /// 使用此事件来处理两个刚体之间开始接触跟踪时的逻辑。
    /// </remarks>
    [CallbackThread(ThreadContext.MainThread)]
    public event Action<Arbiter>? BeginCollide;

    /// <summary>
    /// 当仲裁器在模拟步中因接触结束而过期时触发的事件。
    /// 在此调用之后，对该仲裁器的引用将失效。
    /// </summary>
    /// <remarks>
    /// 此事件提供一个 <see cref="Arbiter"/> 对象，其中包含已结束碰撞的详细信息。
    /// 显式移除仲裁器、刚体或形状不会触发此事件。
    /// </remarks>
    [CallbackThread(ThreadContext.MainThread)]
    public event Action<Arbiter>? EndCollide;

    internal void RaiseBeginCollide(Arbiter arbiter)
    {
        BeginCollide?.Invoke(arbiter);
    }

    internal void RaiseEndCollide(Arbiter arbiter)
    {
        EndCollide?.Invoke(arbiter);
    }

    internal void TrimLists()
    {
        InternalShapes.TrimExcess();
        InternalConnections.TrimExcess();
        InternalContacts.TrimExcess();
        InternalConstraints.TrimExcess();
    }

    /// <summary>
    /// 包含此刚体接触到的或与之共享约束的所有刚体。
    /// </summary>
    public ReadOnlyList<RigidBody> Connections => new(InternalConnections);

    /// <summary>
    /// 包含此刚体参与的所有接触。
    /// </summary>
    public ReadOnlyHashSet<Arbiter> Contacts => new(InternalContacts);

    /// <summary>
    /// 清除此刚体所有当前接触的缓存接触流形状态。
    /// </summary>
    /// <remarks>
    /// 这会丢弃缓存的接触点和累积冲量，强制在下一个模拟步重建接触
    /// 流形。这在用户驱动的非连续变换变更（例如传送）之后非常有用。
    /// 不要与 <see cref="World.Step(Real, bool)"/> 并发调用此方法。
    /// </remarks>
    public void ClearContactCache()
    {
        foreach (var arbiter in InternalContacts)
        {
            arbiter.Handle.Data.UsageMask = 0;
        }
    }

    /// <summary>
    /// 包含连接到此刚体的所有约束。
    /// </summary>
    public ReadOnlyHashSet<Constraint> Constraints => new(InternalConstraints);

    /// <summary>
    /// 获取添加到此刚体的形状列表。
    /// </summary>
    public ReadOnlyList<RigidBodyShape> Shapes => new(InternalShapes);

    private Real inactiveThresholdLinearSq = (Real)0.1;
    private Real inactiveThresholdAngularSq = (Real)0.1;
    private Real deactivationTimeThreshold = (Real)1.0;

    private Real linearDampingMultiplier = (Real)0.998;
    private Real angularDampingMultiplier = (Real)0.995;

    private JVector force;
    private JVector torque;

    private JSymmetricMatrix inverseInertia = JSymmetricMatrix.Identity;
    private Real inverseMass = (Real)1.0;

    /// <summary>
    /// 获取或设置用于接触求解的摩擦系数。
    /// </summary>
    /// <remarks>
    /// 摩擦系数决定对滑动运动的阻力。
    /// 值通常从 0（无摩擦）开始向上取值。
    /// 较高的值表示强摩擦或粘附效应。
    /// 默认值：0.2。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">如果值为负则抛出。</exception>
    public Real Friction
    {
        get => friction;
        set
        {
            friction = ArgumentCheck.NonNegative(value, nameof(value));
        }
    }

    /// <summary>
    /// 获取或设置此对象的恢复系数（弹性）。
    /// </summary>
    /// <remarks>
    /// 恢复系数决定碰撞后保留多少能量，
    /// 0 表示非弹性碰撞（无反弹），1 表示完全弹性碰撞（完全反弹）。
    /// 0 到 1 之间的值产生部分弹性碰撞效果。
    /// 默认值：0.0。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">如果值不在 0 到 1 之间则抛出。</exception>
    public Real Restitution
    {
        get => restitution;
        set
        {
            restitution = ArgumentCheck.InRange(value, (Real)0.0, (Real)1.0, nameof(value));
        }
    }

    /// <summary>
    /// 获取分配给此刚体的世界。
    /// </summary>
    public World World { get; }

    internal RigidBody(JHandle<RigidBodyData> handle, World world)
    {
        this.handle = handle;
        World = world;

        Data.Orientation = JQuaternion.Identity;
        SetDefaultMassInertia();

        RigidBodyId = World.RequestId();

        Data._lockFlag = 0;
    }

    /// <summary>
    /// 获取或设置失活时间。动态刚体必须在此时间内保持在
    /// <see cref="DeactivationThreshold"/> 以下，其所在的岛才能被停用。
    /// 运动学刚体必须在此时间内保持线速度和角速度为零。
    /// 默认值：1 秒。
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当赋值为负数时抛出。
    /// </exception>
    public TimeSpan DeactivationTime
    {
        get => TimeSpan.FromSeconds(deactivationTimeThreshold);
        set
        {
            deactivationTimeThreshold = ArgumentCheck.NonNegative((Real)value.TotalSeconds, nameof(value));
        }
    }

    /// <summary>
    /// 获取或设置动态刚体的失活阈值。如果角速度和线速度的模长在
    /// <see cref="DeactivationTime"/> 持续时间内都保持在指定值以下，其所在的岛就可以被停用。
    /// 阈值的单位分别为 rad/s 和长度单位/s。
    /// </summary>
    /// <remarks>
    /// 值必须为非负。
    /// 默认值：角速度和线速度均约为 0.316。
    /// 运动学刚体忽略这些阈值，仅在两个速度都恰好为零时才静止。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 如果线速度或角速度阈值为负则抛出。
    /// </exception>
    public (Real angular, Real linear) DeactivationThreshold
    {
        get => (MathR.Sqrt(inactiveThresholdAngularSq), MathR.Sqrt(inactiveThresholdLinearSq));
        set
        {
            ArgumentCheck.NonNegative(value.linear, nameof(value.linear));
            ArgumentCheck.NonNegative(value.angular, nameof(value.angular));

            inactiveThresholdLinearSq = value.linear * value.linear;
            inactiveThresholdAngularSq = value.angular * value.angular;
        }
    }

    /// <summary>
    /// 获取或设置线性和角运动的阻尼系数。阻尼系数为 0 表示刚体不受
    /// 阻尼，而 1 会立即使刚体停止运动。阻尼在调用
    /// <see cref="World.Step(Real, bool)"/> 时施加。Jitter 每步将相应的速度乘以 1 减去阻尼
    /// 系数。注意这些值不按时间缩放；<see cref="World.Step(Real, bool)"/> 中较小的时间步
    /// 会导致阻尼增加。
    /// </summary>
    /// <remarks>阻尼系数必须在 [0, 1] 范围内。
    /// 默认值：线性 = 0.002，角 = 0.005。</remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 如果线性或角阻尼值小于 0 或大于 1 则抛出。
    /// </exception>
    public (Real linear, Real angular) Damping
    {
        get => ((Real)1.0 - linearDampingMultiplier, (Real)1.0 - angularDampingMultiplier);
        set
        {
            ArgumentCheck.InRange(value.linear, (Real)0.0, (Real)1.0, nameof(value.linear));
            ArgumentCheck.InRange(value.angular, (Real)0.0, (Real)1.0, nameof(value.angular));

            linearDampingMultiplier = (Real)1.0 - value.linear;
            angularDampingMultiplier = (Real)1.0 - value.angular;
        }
    }

    private void SetDefaultMassInertia()
    {
        inverseInertia = JSymmetricMatrix.Identity;
        inverseMass = (Real)1.0;
        UpdateWorldInertia();
    }

    /// <summary>
    /// 获取刚体（局部）空间中的逆惯性张量。
    /// </summary>
    /// <remarks>
    /// 关于世界空间逆惯性，请参见 <see cref="RigidBodyData.InverseInertiaWorld"/>。
    /// 对于非动态刚体，无论此值如何，求解器都将惯性视为无限大。
    /// <see cref="AllowedMotion"/> 保留此张量，并限制求解器的等效惯性。
    /// </remarks>
    public JSymmetricMatrix InverseInertia => inverseInertia;

    /// <summary>获取或设置刚体在世界空间中允许的平移和旋转。</summary>
    /// <value>默认为 <see cref="MotionAxes.All"/>。</value>
    /// <remarks>
    /// 每次赋值都会替换之前的限制。例如，
    /// <see cref="MotionAxes.PlaneXY"/> 允许沿 X 和 Y 平移并绕 Z 旋转。
    /// 受限方向在求解器中具有零等效逆质量或逆惯性。
    /// 限制会保留标量 <see cref="Mass"/> 和刚体空间的 <see cref="InverseInertia"/>。
    /// 对质量属性或形状的更改会在保留限制的同时更新响应。赋值为
    /// <see cref="MotionAxes.All"/> 会恢复由这些原始属性推导出的响应。
    /// <para>
    /// 更改会立即清除被禁止的速度分量、使缓存的冲量失效，
    /// 并安排激活。限制也适用于运动学速度。它们管的是
    /// 模拟运动；位置和朝向仍可直接赋值。设置
    /// <see cref="MotionAxes.None"/> 不会改变 <see cref="MotionType"/>。
    /// </para>
    /// <para>不要与 <see cref="World.Step(Real, bool)"/> 并发更改此属性。</para>
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">当标志位未定义时抛出。</exception>
    public MotionAxes AllowedMotion
    {
        get => Data.AllowedMotion;
        set
        {
            if ((value & ~MotionAxes.All) != 0)
            {
                throw new ArgumentOutOfRangeException(nameof(value), value, "Undefined motion axes.");
            }

            ref RigidBodyData data = ref Data;
            if (value == data.AllowedMotion) return;

            data.AllowedMotion = value;
            RestrictLinearMotion(ref data.Velocity, value);
            RestrictLinearMotion(ref data.DeltaVelocity, value);
            RestrictAngularMotion(ref data.AngularVelocity, value);
            RestrictAngularMotion(ref data.DeltaAngularVelocity, value);
            UpdateWorldInertia();

            ClearContactCache();
            foreach (Constraint constraint in InternalConstraints) constraint.ResetWarmStart();
            World.ActivateBodyNextStep(this, true);
        }
    }

    /// <summary>
    /// 获取或设置刚体的世界空间位置。
    /// </summary>
    /// <remarks>
    /// 设置此属性会更新所有附加形状的宽相代理，
    /// 并安排动态或运动学刚体在下一步激活。移动静态刚体会
    /// 激活与其连接的刚体。涉及此刚体的所有当前活动
    /// 缓存接触都会失效。
    /// </remarks>
    public JVector Position
    {
        get => Data.Position;
        set
        {
            DebugCheck.IsFinite(value, nameof(value));
            Data.Position = value;
            Move();
        }
    }

    /// <summary>
    /// 获取或设置刚体的世界空间朝向。
    /// </summary>
    /// <remarks>
    /// 设置此属性会更新所有附加形状的宽相代理，
    /// 并安排动态或运动学刚体在下一步激活。移动静态刚体会
    /// 激活与其连接的刚体。涉及此刚体的所有当前活动
    /// 缓存接触都会失效。
    /// </remarks>
    public JQuaternion Orientation
    {
        get => Data.Orientation;
        set
        {
            DebugCheck.IsUnitQuaternion(value, nameof(value));
            Data.Orientation = value;
            Move();
        }
    }

    private void Move()
    {
        UpdateWorldInertia();
        ClearContactCache();

        foreach (var shape in InternalShapes)
        {
            World.DynamicTree.Update(shape, EnableSpeculativeContacts ? World.BroadphaseStepDt : (Real)0.0);
        }

        World.ActivateBodyNextStep(this, true);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void RestrictLinearMotion(ref JVector vector, MotionAxes flags)
    {
        if ((flags & MotionAxes.Linear) == MotionAxes.Linear) return;
        if ((flags & MotionAxes.LinearX) == 0) vector.X = 0;
        if ((flags & MotionAxes.LinearY) == 0) vector.Y = 0;
        if ((flags & MotionAxes.LinearZ) == 0) vector.Z = 0;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void RestrictAngularMotion(ref JVector vector, MotionAxes flags)
    {
        if ((flags & MotionAxes.Angular) == MotionAxes.Angular) return;
        if ((flags & MotionAxes.AngularX) == 0) vector.X = 0;
        if ((flags & MotionAxes.AngularY) == 0) vector.Y = 0;
        if ((flags & MotionAxes.AngularZ) == 0) vector.Z = 0;
    }

    private static void RestrictAngularMotion(ref JSymmetricMatrix inertia, MotionAxes flags)
    {
        if ((flags & MotionAxes.Angular) == 0)
        {
            inertia = JSymmetricMatrix.Zero;
            return;
        }

        // 使用 Schur 补从逆惯性中消除被锁定的速度。
        if ((flags & MotionAxes.AngularX) == 0)
        {
            if (inertia.M11 > 0)
            {
                Real xy = inertia.M12 / inertia.M11;
                Real xz = inertia.M13 / inertia.M11;
                inertia.M22 -= xy * inertia.M12;
                inertia.M23 -= xy * inertia.M13;
                inertia.M33 -= xz * inertia.M13;
            }
            inertia.M11 = inertia.M12 = inertia.M13 = 0;
        }
        if ((flags & MotionAxes.AngularY) == 0)
        {
            if (inertia.M22 > 0)
            {
                Real xy = inertia.M12 / inertia.M22;
                Real yz = inertia.M23 / inertia.M22;
                inertia.M11 -= xy * inertia.M12;
                inertia.M13 -= xy * inertia.M23;
                inertia.M33 -= yz * inertia.M23;
            }
            inertia.M12 = inertia.M22 = inertia.M23 = 0;
        }
        if ((flags & MotionAxes.AngularZ) == 0)
        {
            if (inertia.M33 > 0)
            {
                Real xz = inertia.M13 / inertia.M33;
                Real yz = inertia.M23 / inertia.M33;
                inertia.M11 -= xz * inertia.M13;
                inertia.M12 -= xz * inertia.M23;
                inertia.M22 -= yz * inertia.M23;
            }
            inertia.M13 = inertia.M23 = inertia.M33 = 0;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    internal void Update(in TimeStep timeStep)
    {
        ref RigidBodyData rigidBody = ref Data;
        MotionAxes axes = rigidBody.AllowedMotion;

        RestrictLinearMotion(ref rigidBody.Velocity, axes);
        RestrictAngularMotion(ref rigidBody.AngularVelocity, axes);

        // 运动学速度由调用方规定，因此任何运动都会阻止休眠。
        bool isKinematic = rigidBody.MotionType == MotionType.Kinematic;
        bool resting = isKinematic
            ? rigidBody.AngularVelocity == JVector.Zero && rigidBody.Velocity == JVector.Zero
            : rigidBody.AngularVelocity.LengthSquared() < inactiveThresholdAngularSq &&
              rigidBody.Velocity.LengthSquared() < inactiveThresholdLinearSq;

        if (resting)
        {
            InternalSleepTime += timeStep.StepDt;
        }
        else
        {
            InternalSleepTime = 0;
        }

        if ((isKinematic && !resting) || InternalSleepTime < deactivationTimeThreshold)
        {
            InternalIsland.MarkedAsActive = true;
        }

        if (rigidBody.MotionType == MotionType.Dynamic)
        {
            UpdateWorldInertia();
            rigidBody.AngularVelocity *= angularDampingMultiplier;
            rigidBody.Velocity *= linearDampingMultiplier;

            rigidBody.DeltaVelocity = JVector.Multiply(Force, rigidBody.InverseMassVector) * timeStep.SubstepDt;
            rigidBody.DeltaAngularVelocity = JVector.Transform(Torque, rigidBody.InverseInertiaWorld) * timeStep.SubstepDt;

            if (AffectedByGravity)
            {
                JVector gravity = World.Gravity;
                RestrictLinearMotion(ref gravity, axes);
                rigidBody.DeltaVelocity += gravity * timeStep.SubstepDt;
            }

            Force = JVector.Zero;
            Torque = JVector.Zero;
        }
    }

    /// <summary>
    /// 获取或设置刚体在世界空间中的线性速度。
    /// </summary>
    /// <remarks>
    /// 单位为每秒长度单位。设置非零运动学速度会安排刚体
    /// 在下一步激活。对于动态刚体，激活检查会应用一个很小的容差。
    /// 被锁定的平移分量会被清除。
    /// </remarks>
    /// <exception cref="InvalidOperationException">
    /// 如果刚体的 <see cref="MotionType"/> 为 <see cref="MotionType.Static"/> 则抛出。
    /// </exception>
    public JVector Velocity
    {
        get => Data.Velocity;
        set
        {
            DebugCheck.IsFinite(value, nameof(value));

            ref RigidBodyData data = ref Data;
            if (data.MotionType == MotionType.Static)
            {
                throw new InvalidOperationException(
                    $"Cannot set velocity for static objects, objects must be kinematic or dynamic. See {nameof(MotionType)}.");
            }

            RestrictLinearMotion(ref value, data.AllowedMotion);
            JVector previousVelocity = data.Velocity;
            data.Velocity = value;

            if (EnableSpeculativeContacts && previousVelocity != value)
            {
                UpdateSpeculativeBroadphase();
                // 自定义形状包围盒的更新可能会重新定位刚体缓冲区。
                data = ref Data;
            }

            if (data.MotionType == MotionType.Kinematic
                ? value != JVector.Zero
                : !MathHelper.CloseToZero(value))
            {
                World.ActivateBodyNextStep(this);
            }
        }
    }

    /// <summary>
    /// 获取或设置刚体在世界空间中的角速度。
    /// </summary>
    /// <remarks>
    /// 单位为弧度每秒。向量方向为旋转轴，
    /// 其模长为旋转速率。设置非零运动学速度会安排
    /// 刚体在下一步激活。对于动态刚体，激活检查会应用一个很小的容差。
    /// 被锁定的旋转分量会被清除。
    /// </remarks>
    /// <exception cref="InvalidOperationException">
    /// 如果刚体的 <see cref="MotionType"/> 为 <see cref="MotionType.Static"/> 则抛出。
    /// </exception>
    public JVector AngularVelocity
    {
        get => Data.AngularVelocity;
        set
        {
            DebugCheck.IsFinite(value, nameof(value));

            ref RigidBodyData data = ref Data;
            if (data.MotionType == MotionType.Static)
            {
                throw new InvalidOperationException(
                    $"Cannot set angular velocity for static objects, objects must be kinematic or dynamic. See {nameof(MotionType)}.");
            }

            RestrictAngularMotion(ref value, data.AllowedMotion);
            data.AngularVelocity = value;

            if (data.MotionType == MotionType.Kinematic
                ? value != JVector.Zero
                : !MathHelper.CloseToZero(value))
            {
                World.ActivateBodyNextStep(this);
            }
        }
    }

    /// <summary>
    /// 获取或设置刚体在积分过程中是否受世界重力影响。
    /// </summary>
    /// <remarks>
    /// 仅在 <see cref="MotionType"/> 为 <see cref="MotionType.Dynamic"/> 时适用。
    /// 默认为 <see langword="true"/>。
    /// </remarks>
    public bool AffectedByGravity { get; set; } = true;

    /// <summary>
    /// 指向自定义用户数据的托管指针。引擎不使用此数据。
    /// </summary>
    public object? Tag { get; set; }

    private bool enableSpeculativeContacts;

    /// <summary>
    /// 获取或设置此刚体是否启用仅线性的推测接触。
    /// </summary>
    /// <remarks>
    /// 推测接触通过在真正发生穿透之前生成接触，帮助防止快速线性运动
    /// 造成的穿透。预测和推测包围盒
    /// 扩展仅使用线性速度，扫掠过程中形状朝向保持不变。
    /// 推测约束忽略表面角速度，仅更新线性速度。
    /// 它们无法防止由旋转导致的穿透。刚体仍会正常旋转，普通
    /// 重叠接触保留其角度响应。启用推测接触
    /// 可能增加接触数量和求解器开销。默认为 <see langword="false"/>。
    /// </remarks>
    public bool EnableSpeculativeContacts
    {
        get => enableSpeculativeContacts;
        set
        {
            if (enableSpeculativeContacts == value) return;
            enableSpeculativeContacts = value;
            if (IsValid && Data.Velocity.LengthSquared() > (Real)0.0)
            {
                foreach (var shape in InternalShapes)
                {
                    World.DynamicTree.Update(shape, value ? World.BroadphaseStepDt : (Real)0.0);
                }
            }
        }
    }

    private void UpdateSpeculativeBroadphase()
    {
        foreach (var shape in InternalShapes)
        {
            World.DynamicTree.Update(shape, World.BroadphaseStepDt);
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void UpdateWorldInertia()
    {
        ref RigidBodyData data = ref Data;
        if (data.MotionType == MotionType.Dynamic)
        {
            var bodyOrientation = JMatrix.CreateFromQuaternion(data.Orientation);
            JSymmetricMatrix.Transform(inverseInertia, bodyOrientation,
                out data.InverseInertiaWorld);
            if (data.HasAngularLocks)
            {
                RestrictAngularMotion(ref data.InverseInertiaWorld, data.AllowedMotion);
            }
            data.InverseMassVector = new JVector(inverseMass);
            RestrictLinearMotion(ref data.InverseMassVector, data.AllowedMotion);
        }
        else
        {
            data.InverseInertiaWorld = JSymmetricMatrix.Zero;
            data.InverseMassVector = JVector.Zero;
        }
    }

    private void ClearQueuedForces()
    {
        Force = JVector.Zero;
        Torque = JVector.Zero;

        ref RigidBodyData data = ref Data;
        data.DeltaVelocity = JVector.Zero;
        data.DeltaAngularVelocity = JVector.Zero;
    }

    /// <summary>
    /// 获取或设置刚体如何参与模拟。
    /// </summary>
    /// <remarks>
    /// 更改此属性会立即产生副作用：
    /// <list type="bullet">
    /// <item><description>切换到 <see cref="MotionType.Static"/> 会将速度清零、移除连接并停用刚体。</description></item>
    /// <item><description>从 <see cref="MotionType.Static"/> 切换出来会根据现有接触重建连接。</description></item>
    /// <item><description>切换到 <see cref="MotionType.Kinematic"/> 会保留速度。如果希望刚体静止，请将两个速度都设为零。</description></item>
    /// <item><description>约束仍保持注册，并在两个相连刚体都不是动态刚体时挂起。</description></item>
    /// <item><description>任何排队的力、力矩以及由力推导出的速度变化都会被丢弃。</description></item>
    /// <item><description>动态刚体使用其质量和惯性；静态和运动学刚体在求解器中视为具有无限质量。</description></item>
    /// </list>
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">如果值不是有效的 <see cref="MotionType"/> 则抛出。</exception>
    public MotionType MotionType
    {
        get => Data.MotionType;
        set
        {
            if (Data.MotionType == value) return;

            switch (value)
            {
                case MotionType.Static:
                    ClearQueuedForces();
                    // 切换为静态
                    World.RemoveConnections(this);
                    Data.MotionType = MotionType.Static;
                    World.RemoveStaticStaticContacts(this);
                    World.DeactivateBodyNextStep(this);
                    Data.Velocity = JVector.Zero;
                    Data.AngularVelocity = JVector.Zero;
                    UpdateWorldInertia();
                    break;
                case MotionType.Kinematic:
                    {
                        ClearQueuedForces();
                        // 切换为运动学
                        if (Data.MotionType == MotionType.Static)
                        {
                            Data.MotionType = MotionType.Kinematic;
                            World.BuildConnectionsFromExistingContacts(this);
                        }

                        Data.MotionType = MotionType.Kinematic;
                        World.RemoveStaticStaticContacts(this);
                        World.ActivateBodyNextStep(this, true);
                        UpdateWorldInertia();
                        break;
                    }
                case MotionType.Dynamic:
                    {
                        ClearQueuedForces();
                        // 切换为动态
                        if (Data.MotionType == MotionType.Static)
                        {
                            Data.MotionType = MotionType.Dynamic;
                            World.BuildConnectionsFromExistingContacts(this);
                        }

                        Data.MotionType = MotionType.Dynamic;
                        World.ActivateBodyNextStep(this, true);
                        UpdateWorldInertia();
                        break;
                    }
                default:
                    throw new ArgumentOutOfRangeException(nameof(value), value, null);
            }

            foreach (Constraint constraint in InternalConstraints)
            {
                World.UpdateConstraintActivation(constraint);
            }
        }
    }

    [Obsolete($"Use the {nameof(MotionType)} property instead.", true)]
    public bool IsStatic
    {
        set => MotionType = value ? MotionType.Static : MotionType.Dynamic;
        get => MotionType == MotionType.Static;
    }

    /// <summary>
    /// 指示刚体是处于活动状态还是被视为休眠状态。
    /// 使用 <see cref="SetActivationState"/> 更改激活状态。
    /// </summary>
    public bool IsActive => Data.IsActive;

    /// <summary>
    /// 指示引擎在下一个时间步开始时激活或停用刚体。
    /// 当前状态不会立即改变。
    /// </summary>
    /// <remarks>
    /// 激活和停用通过刚体的模拟岛生效。
    /// 静态刚体不能通过此方法激活，不过移动静态刚体可以唤醒相连的
    /// 非静态刚体。
    /// </remarks>
    /// <param name="active">
    /// 如果为 <see langword="true"/>，刚体将被激活；如果为 <see langword="false"/>，则被停用。
    /// </param>
    public void SetActivationState(bool active)
    {
        if (active) World.ActivateBodyNextStep(this);
        else World.DeactivateBodyNextStep(this);
    }

    private void AttachToShape(RigidBodyShape shape)
    {
        if (shape.RigidBody != null)
        {
            throw new ArgumentException("Shape has already been added to a body.", nameof(shape));
        }

        shape.RigidBody = this;
        try
        {
            shape.UpdateWorldBoundingBox(EnableSpeculativeContacts ? World.BroadphaseStepDt : (Real)0.0);
            World.DynamicTree.AddProxy(shape, IsActive);
        }
        catch
        {
            shape.RigidBody = null!;
            throw;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool ShouldUpdateMassInertia(MassInertiaUpdateMode massInertiaMode)
    {
        return massInertiaMode switch
        {
            MassInertiaUpdateMode.Update => true,
            MassInertiaUpdateMode.Preserve => false,
            _ => throw new ArgumentOutOfRangeException(nameof(massInertiaMode), massInertiaMode, null)
        };
    }

    /// <summary>
    /// 一次向刚体添加多个形状。如有需要，质量属性
    /// 只重新计算一次。
    /// </summary>
    /// <param name="shapes">要添加的形状。</param>
    /// <exception cref="ArgumentNullException">
    /// 如果 <paramref name="shapes"/> 或其中任一形状为 <see langword="null"/> 则抛出。
    /// </exception>
    /// <exception cref="ArgumentException">如果任一形状已附加到刚体则抛出。</exception>
    public void AddShapes(IEnumerable<RigidBodyShape> shapes)
        => AddShapes(shapes, MassInertiaUpdateMode.Update);

    /// <summary>
    /// 一次向刚体添加多个形状。如有需要，质量属性
    /// 只重新计算一次。
    /// </summary>
    /// <remarks>
    /// 形状按顺序添加。如果抛出异常，失败之前已处理的形状
    /// 仍保持附加到刚体。
    /// </remarks>
    /// <param name="shapes">要添加的形状。</param>
    /// <param name="massInertiaMode">
    /// 控制添加形状后是否重新计算刚体的质量和惯性。
    /// </param>
    /// <exception cref="ArgumentNullException">
    /// 如果 <paramref name="shapes"/> 或其中任一形状为 <see langword="null"/> 则抛出。
    /// </exception>
    /// <exception cref="ArgumentException">如果任一形状已附加到刚体则抛出。</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="massInertiaMode"/> 不是有效的 <see cref="MassInertiaUpdateMode"/> 值时抛出。
    /// </exception>
    public void AddShapes(IEnumerable<RigidBodyShape> shapes, MassInertiaUpdateMode massInertiaMode)
    {
        ArgumentNullException.ThrowIfNull(shapes);
        bool updateMassInertia = ShouldUpdateMassInertia(massInertiaMode);

        foreach (RigidBodyShape shape in shapes)
        {
            ArgumentNullException.ThrowIfNull(shape);

            if (shape.IsRegistered)
            {
                throw new ArgumentException("Shape cannot be added. Shape already registered elsewhere.", nameof(shapes));
            }

            AttachToShape(shape);
            InternalShapes.Add(shape);
        }

        if (updateMassInertia) SetMassInertia();
    }

    /// <summary>
    /// 为此 <see cref="RigidBody"/> 启用隐式陀螺力矩求解器。
    /// </summary>
    /// <remarks>
    /// 当为 <see langword="true"/> 且 <see cref="AllowedMotion"/> 为
    /// <see cref="MotionAxes.All"/> 时，每个子步都会执行一次额外的 Newton 迭代，
    /// 以隐式求解 <c>ω × (I ω)</c>。当任何轴被锁定时，求解器将被跳过。
    ///
    /// 对于具有高惯性各向异性或非常快自旋速率的刚体，收益会变得明显。
    /// 典型例子包括细长杆、陀螺、螺旋桨以及其他主惯性值
    /// 相差一个数量级的物体。在这些情况下，该标志会消除人为的
    /// 进动。
    /// </remarks>
    /// <value>
    /// <see langword="true"/> 表示每步积分陀螺力矩；否则为
    /// <see langword="false"/>（默认值）。
    /// </value>
    public bool EnableGyroscopicForces
    {
        get => Data.EnableGyroscopicForces;
        set => Data.EnableGyroscopicForces = value;
    }

    /// <summary>
    /// 向刚体添加一个形状。
    /// </summary>
    /// <remarks>
    /// 如果注册或质量计算失败，形状将保持分离，刚体的
    /// 质量属性保持不变。
    /// </remarks>
    /// <param name="shape">要添加的形状。</param>
    /// <exception cref="ArgumentNullException">
    /// 如果 <paramref name="shape"/> 为 <see langword="null"/> 则抛出。
    /// </exception>
    /// <exception cref="ArgumentException">
    /// 如果形状已注册到其他地方则抛出。
    /// </exception>
    public void AddShape(RigidBodyShape shape)
        => AddShape(shape, MassInertiaUpdateMode.Update);

    /// <summary>
    /// 向刚体添加一个形状。
    /// </summary>
    /// <remarks>
    /// 如果注册或质量计算失败，形状将保持分离，刚体的
    /// 质量属性保持不变。
    /// </remarks>
    /// <param name="shape">要添加的形状。</param>
    /// <param name="massInertiaMode">
    /// 控制添加形状后是否重新计算刚体的质量和惯性。
    /// </param>
    /// <exception cref="ArgumentException">
    /// 如果形状已注册到其他地方则抛出。
    /// </exception>
    /// <exception cref="ArgumentNullException">
    /// 如果 <paramref name="shape"/> 为 <see langword="null"/> 则抛出。
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="massInertiaMode"/> 不是有效的 <see cref="MassInertiaUpdateMode"/> 值时抛出。
    /// </exception>
    public void AddShape(RigidBodyShape shape, MassInertiaUpdateMode massInertiaMode)
    {
        ArgumentNullException.ThrowIfNull(shape);
        bool updateMassInertia = ShouldUpdateMassInertia(massInertiaMode);

        if (shape.IsRegistered)
        {
            throw new ArgumentException("Shape cannot be added. Shape already registered elsewhere.", nameof(shape));
        }

        AttachToShape(shape);
        try
        {
            InternalShapes.Add(shape);
            if (updateMassInertia) SetMassInertia();
        }
        catch
        {
            InternalShapes.Remove(shape);
            World.DynamicTree.RemoveProxy(shape);
            shape.RigidBody = null!;
            throw;
        }
    }

    [Obsolete($"Use {nameof(AddShapes)} with {nameof(MassInertiaUpdateMode)} instead.", true)]
    public void AddShape(IEnumerable<RigidBodyShape> shapes, bool setMassInertia = true)
        => AddShapes(shapes, setMassInertia ? MassInertiaUpdateMode.Update : MassInertiaUpdateMode.Preserve);

    [Obsolete($"Use {nameof(AddShape)} with {nameof(MassInertiaUpdateMode)} instead.", true)]
    public void AddShape(RigidBodyShape shape, bool setMassInertia = true)
        => AddShape(shape, setMassInertia ? MassInertiaUpdateMode.Update : MassInertiaUpdateMode.Preserve);

    /// <summary>
    /// 累积的力，在下次
    /// <see cref="World.Step(Real, bool)"/> 调用结束时转换为速度变化。该变化在随后的步中施加，
    /// 转换后此值重置为零。
    /// </summary>
    public JVector Force
    {
        get => force;
        set
        {
            DebugCheck.IsFinite(value, nameof(value));
            force = value;
        }
    }

    /// <summary>
    /// 累积的力矩，在下次
    /// <see cref="World.Step(Real, bool)"/> 调用结束时转换为角速度变化。该变化在随后的步中施加，
    /// 转换后此值重置为零。
    /// </summary>
    public JVector Torque
    {
        get => torque;
        set
        {
            DebugCheck.IsFinite(value, nameof(value));
            torque = value;
        }
    }

    /// <summary>
    /// 对刚体施加一个力，从而改变其速度。
    /// </summary>
    /// <param name="force">
    /// 要累积的力。它在下次 <see cref="World.Step(Real, bool)"/> 调用结束时
    /// 转换为速度变化，并在随后的步中施加。
    /// </param>
    /// <param name="wakeup">
    /// 如果为 <c>true</c>（默认），刚体当前处于休眠状态时将被激活。
    /// 如果为 <c>false</c>，仅当刚体已经处于活动状态时才施加该力；休眠的
    /// 刚体将保持休眠并忽略该力。
    /// </param>
    public void AddForce(in JVector force, bool wakeup = true)
    {
        DebugCheck.IsFinite(force, nameof(force));

        if ((Data.MotionType != MotionType.Dynamic) || MathHelper.CloseToZero(force)) return;

        if (wakeup) SetActivationState(true);
        else if (!IsActive) return;

        Force += force;
    }

    /// <summary>
    /// 在世界空间中的某位置对刚体施加一个力。该力及其产生的力矩
    /// 在下次 <see cref="World.Step(Real, bool)"/> 调用结束时转换为速度变化，
    /// 并在随后的步中施加。
    /// </summary>
    /// <param name="force">要施加的力。</param>
    /// <param name="position">施力的位置。</param>
    /// <param name="wakeup">
    /// 如果为 <c>true</c>（默认），刚体当前处于休眠状态时将被激活。
    /// 如果为 <c>false</c>，仅当刚体已经处于活动状态时才施加该力；休眠的
    /// 刚体将保持休眠并忽略该力。
    /// </param>
    [ReferenceFrame(ReferenceFrame.World)]
    public void AddForce(in JVector force, in JVector position, bool wakeup = true)
    {
        DebugCheck.IsFinite(force, nameof(force));
        DebugCheck.IsFinite(position, nameof(position));

        if ((Data.MotionType != MotionType.Dynamic) || MathHelper.CloseToZero(force)) return;

        if (wakeup) SetActivationState(true);
        else if (!IsActive) return;

        ref RigidBodyData data = ref Data;
        JVector.Subtract(position, data.Position, out JVector torque);
        JVector.Cross(torque, force, out torque);

        Force += force;
        Torque += torque;
    }

    /// <summary>
    /// 对刚体施加瞬时冲量，直接改变其速度。
    /// </summary>
    /// <param name="impulse">要施加的冲量。</param>
    /// <param name="wakeup">
    /// 如果为 <c>true</c>（默认），刚体当前处于休眠状态时将被激活。
    /// 如果为 <c>false</c>，仅当刚体已经处于活动状态时才施加该冲量；休眠的
    /// 刚体将保持休眠并忽略该冲量。
    /// </param>
    public void ApplyImpulse(in JVector impulse, bool wakeup = true)
    {
        DebugCheck.IsFinite(impulse, nameof(impulse));

        ref RigidBodyData data = ref Data;
        if ((data.MotionType != MotionType.Dynamic) || MathHelper.CloseToZero(impulse)) return;
        if (!wakeup && !data.IsActive) return;

        World.ActivateBodyNextStep(this);
        JVector previousVelocity = data.Velocity;
        data.Velocity += JVector.Multiply(impulse, data.InverseMassVector);
        if (EnableSpeculativeContacts && previousVelocity != data.Velocity)
        {
            UpdateSpeculativeBroadphase();
        }
    }

    /// <summary>
    /// 在世界空间中的某位置施加瞬时冲量，直接改变线速度
    /// 和角速度。
    /// </summary>
    /// <param name="impulse">要施加的冲量。</param>
    /// <param name="position">施加冲量的位置。</param>
    /// <param name="wakeup">
    /// 如果为 <c>true</c>（默认），刚体当前处于休眠状态时将被激活。
    /// 如果为 <c>false</c>，仅当刚体已经处于活动状态时才施加该冲量；休眠的
    /// 刚体将保持休眠并忽略该冲量。
    /// </param>
    [ReferenceFrame(ReferenceFrame.World)]
    public void ApplyImpulse(in JVector impulse, in JVector position, bool wakeup = true)
    {
        DebugCheck.IsFinite(impulse, nameof(impulse));
        DebugCheck.IsFinite(position, nameof(position));

        if ((Data.MotionType != MotionType.Dynamic) || MathHelper.CloseToZero(impulse)) return;
        if (!wakeup && !IsActive) return;

        World.ActivateBodyNextStep(this);

        ref RigidBodyData data = ref Data;
        JVector.Subtract(position, data.Position, out JVector angularImpulse);
        JVector.Cross(angularImpulse, impulse, out angularImpulse);

        JVector previousVelocity = data.Velocity;
        data.Velocity += JVector.Multiply(impulse, data.InverseMassVector);
        data.AngularVelocity += JVector.Transform(angularImpulse, data.InverseInertiaWorld);
        if (EnableSpeculativeContacts && previousVelocity != data.Velocity)
        {
            UpdateSpeculativeBroadphase();
        }
    }

    [Obsolete("Use ApplyImpulse instead.", true)]
    public void AddImpulse(in JVector impulse, bool wakeup = true) => ApplyImpulse(impulse, wakeup);

    [Obsolete("Use ApplyImpulse instead.", true)]
    public void AddImpulse(in JVector impulse, in JVector position, bool wakeup = true) => ApplyImpulse(impulse, position, wakeup);

    /// <summary>
    /// 使用线性外推预测刚体在给定时间步之后的位置。
    /// 这不会模拟力或碰撞——它假设速度恒定。
    /// </summary>
    /// <param name="dt">要向前外推的时间步。</param>
    /// <returns><paramref name="dt"/> 之后的预测位置。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public JVector PredictPosition(Real dt) => Data.Position + Data.Velocity * dt;

    /// <summary>
    /// 使用角速度预测刚体在给定时间步之后的朝向。
    /// 这不会模拟力或碰撞——它假设角速度恒定。
    /// </summary>
    /// <param name="dt">要向前外推的时间步。</param>
    /// <returns><paramref name="dt"/> 之后的预测朝向。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public JQuaternion PredictOrientation(Real dt) =>
        JQuaternion.Normalize(MathHelper.RotationQuaternion(Data.AngularVelocity, dt) * Data.Orientation);

    /// <summary>
    /// 使用简单外推预测刚体在给定时间步之后的位姿（位置和朝向）。
    /// 此方法用于渲染目的，不会修改模拟状态。
    /// </summary>
    /// <param name="dt">要向前外推的时间步。</param>
    /// <param name="position"><paramref name="dt"/> 之后的预测位置。</param>
    /// <param name="orientation"><paramref name="dt"/> 之后的预测朝向。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void PredictPose(Real dt, out JVector position, out JQuaternion orientation)
    {
        position = PredictPosition(dt);
        orientation = PredictOrientation(dt);
    }

    /// <summary>
    /// 从刚体中移除指定的形状。
    /// </summary>
    /// <param name="shape">要从刚体中移除的形状。</param>
    /// <exception cref="ArgumentNullException">
    /// 如果 <paramref name="shape"/> 为 <see langword="null"/> 则抛出。
    /// </exception>
    /// <exception cref="ArgumentException">
    /// 如果指定的形状不属于此刚体则抛出。
    /// </exception>
    public void RemoveShape(RigidBodyShape shape)
        => RemoveShape(shape, MassInertiaUpdateMode.Update);

    /// <summary>
    /// 从刚体中移除指定的形状。
    /// </summary>
    /// <param name="shape">要从刚体中移除的形状。</param>
    /// <param name="massInertiaMode">
    /// 控制移除形状后是否重新计算刚体的质量和惯性。
    /// </param>
    /// <exception cref="ArgumentException">
    /// 如果指定的形状不属于此刚体则抛出。
    /// </exception>
    /// <exception cref="ArgumentNullException">
    /// 如果 <paramref name="shape"/> 为 <see langword="null"/> 则抛出。
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="massInertiaMode"/> 不是有效的 <see cref="MassInertiaUpdateMode"/> 值时抛出。
    /// </exception>
    public void RemoveShape(RigidBodyShape shape, MassInertiaUpdateMode massInertiaMode)
    {
        ArgumentNullException.ThrowIfNull(shape);
        bool updateMassInertia = ShouldUpdateMassInertia(massInertiaMode);

        if (!InternalShapes.Remove(shape))
        {
            throw new ArgumentException("Shape is not part of this body.", nameof(shape));
        }

        World.RemoveDeferredArbitersForShape(this, shape.ShapeId);

        foreach (var arbiter in InternalContacts)
        {
            if (arbiter.Handle.Data.Key.Key1 == shape.ShapeId || arbiter.Handle.Data.Key.Key2 == shape.ShapeId)
            {
                // 从 Contacts（即我们正在迭代的 HashSet）中移除当前正在迭代的元素，
                // 即正在迭代的集合被修改。这是允许的。
                World.Remove(arbiter);
            }
        }

        World.DynamicTree.RemoveProxy(shape);
        shape.RigidBody = null!;

        if (updateMassInertia) SetMassInertia();
    }

    /// <summary>
    /// 从刚体中移除多个形状。
    /// </summary>
    /// <param name="shapes">要从刚体中移除的形状。</param>
    /// <exception cref="ArgumentNullException">
    /// 如果 <paramref name="shapes"/> 或其中任一形状为 <see langword="null"/> 则抛出。
    /// </exception>
    /// <exception cref="ArgumentException">如果至少有一个形状不属于此刚体则抛出。</exception>
    public void RemoveShapes(IEnumerable<RigidBodyShape> shapes)
        => RemoveShapes(shapes, MassInertiaUpdateMode.Update);

    /// <summary>
    /// 从刚体中移除多个形状。
    /// </summary>
    /// <param name="shapes">要从刚体中移除的形状。</param>
    /// <param name="massInertiaMode">
    /// 控制移除形状后是否重新计算刚体的质量和惯性。
    /// </param>
    /// <exception cref="ArgumentNullException">
    /// 如果 <paramref name="shapes"/> 或其中任一形状为 <see langword="null"/> 则抛出。
    /// </exception>
    /// <exception cref="ArgumentException">如果至少有一个形状不属于此刚体则抛出。</exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="massInertiaMode"/> 不是有效的 <see cref="MassInertiaUpdateMode"/> 值时抛出。
    /// </exception>
    public void RemoveShapes(IEnumerable<RigidBodyShape> shapes, MassInertiaUpdateMode massInertiaMode)
    {
        ArgumentNullException.ThrowIfNull(shapes);
        bool updateMassInertia = ShouldUpdateMassInertia(massInertiaMode);

        HashSet<ulong> sids = new();

        foreach (var shape in shapes)
        {
            ArgumentNullException.ThrowIfNull(shape);

            if (shape.RigidBody != this)
            {
                throw new ArgumentException($"Shape {shape} is not attached to this body.", nameof(shapes));
            }

            sids.Add(shape.ShapeId);
        }

        foreach (ulong shapeId in sids) World.RemoveDeferredArbitersForShape(this, shapeId);

        foreach (var arbiter in InternalContacts)
        {
            if (sids.Contains(arbiter.Handle.Data.Key.Key1) || sids.Contains(arbiter.Handle.Data.Key.Key2))
            {
                // 从 Contacts（即我们正在迭代的 HashSet）中移除当前正在迭代的元素，
                // 即正在迭代的集合被修改。这是允许的。
                World.Remove(arbiter);
            }
        }

        for (int i = InternalShapes.Count; i-- > 0;)
        {
            var shape = InternalShapes[i];

            if (sids.Contains(shape.ShapeId))
            {
                World.DynamicTree.RemoveProxy(shape);
                shape.RigidBody = null!;
                InternalShapes.RemoveAt(i);
            }
        }

        if (updateMassInertia) SetMassInertia();

        sids.Clear();
    }

    [Obsolete($"Use {nameof(RemoveShape)} with {nameof(MassInertiaUpdateMode)} instead.", true)]
    public void RemoveShape(RigidBodyShape shape, bool setMassInertia = true)
        => RemoveShape(shape, setMassInertia ? MassInertiaUpdateMode.Update : MassInertiaUpdateMode.Preserve);

    [Obsolete($"Use {nameof(RemoveShapes)} with {nameof(MassInertiaUpdateMode)} instead.", true)]
    public void RemoveShape(IEnumerable<RigidBodyShape> shapes, bool setMassInertia = true)
        => RemoveShapes(shapes, setMassInertia ? MassInertiaUpdateMode.Update : MassInertiaUpdateMode.Preserve);

    /// <summary>
    /// 移除与刚体关联的所有形状。
    /// </summary>
    public void ClearShapes()
        => ClearShapes(MassInertiaUpdateMode.Update);

    /// <summary>
    /// 移除与刚体关联的所有形状。
    /// </summary>
    /// <param name="massInertiaMode">
    /// 控制移除形状后是否重新计算刚体的质量和惯性。
    /// </param>
    public void ClearShapes(MassInertiaUpdateMode massInertiaMode)
    {
        RemoveShapes(InternalShapes, massInertiaMode);
    }

    [Obsolete($"Use {nameof(ClearShapes)} with {nameof(MassInertiaUpdateMode)} instead.", true)]
    public void ClearShapes(bool setMassInertia = true)
    {
        ClearShapes(setMassInertia ? MassInertiaUpdateMode.Update : MassInertiaUpdateMode.Preserve);
    }

    /// <summary>
    /// 根据所有附加的形状计算此刚体的质量和惯性，假设密度为单位密度。
    /// </summary>
    /// <remarks>
    /// 所有形状的质量贡献会求和。如果没有附加任何形状，刚体
    /// 将被赋予质量 1 和单位惯性张量。
    /// </remarks>
    /// <exception cref="InvalidOperationException">
    /// 如果计算出的惯性矩阵不可逆则抛出。这可能发生在某个形状具有无效质量属性时。
    /// </exception>
    public void SetMassInertia()
    {
        if (InternalShapes.Count == 0)
        {
            inverseInertia = JSymmetricMatrix.Identity;
            inverseMass = (Real)1.0;
            UpdateWorldInertia();
            return;
        }

        JSymmetricMatrix inertia = JSymmetricMatrix.Zero;
        Real mass = (Real)0.0;

        foreach (var rbs in InternalShapes)
        {
            rbs.CalculateMassInertia(out var shapeInertia, out _, out var shapeMass);

            inertia += shapeInertia;
            mass += shapeMass;
        }

        if (!JSymmetricMatrix.Inverse(inertia, out JSymmetricMatrix newInverseInertia))
        {
            throw new InvalidOperationException("Inertia matrix is not invertible. This might happen if a shape has " +
                                                "invalid mass properties. If you encounter this while calling " +
                                                "AddShape or AddShapes, call the method with massInertiaMode set to " +
                                                nameof(MassInertiaUpdateMode.Preserve) + ".");
        }

        inverseInertia = newInverseInertia;
        inverseMass = (Real)1.0 / mass;

        UpdateWorldInertia();
    }

    /// <summary>
    /// 根据所有附加的形状计算惯性，然后对其统一缩放以匹配指定的质量。
    /// </summary>
    /// <remarks>
    /// 这等价于调用 <see cref="SetMassInertia()"/>，然后缩放得到的
    /// 惯性张量，使刚体具有所需的总质量。当你希望惯性比例来自形状，
    /// 但需要特定总质量（例如用于玩法调优）时，请使用此方法。
    /// </remarks>
    /// <param name="mass">刚体所需的总质量。必须为正。</param>
    /// <exception cref="ArgumentException">如果指定的质量为零或负数则抛出。</exception>
    public void SetMassInertia(Real mass)
    {
        ArgumentCheck.Finite(mass, nameof(mass));

        if (mass <= (Real)0.0)
        {
            throw new ArgumentException("Mass cannot be zero or negative.", nameof(mass));
        }

        SetMassInertia();
        inverseInertia = JSymmetricMatrix.Multiply(inverseInertia, (Real)1.0 / (inverseMass * mass));
        inverseMass = (Real)1.0 / mass;
        UpdateWorldInertia();
    }

    /// <summary>
    /// 通过直接指定惯性和质量来设置此刚体的新质量属性。
    /// </summary>
    /// <param name="inertia">
    /// 刚体（局部）空间中关于质心的对称惯性张量
    /// （如果 <paramref name="setAsInverse"/> 为 true，则为逆惯性张量）。
    /// </param>
    /// <param name="mass">
    /// 质量（如果 <paramref name="setAsInverse"/> 为 true，则为逆质量）。设置逆质量时，
    /// 值为零表示无限质量。
    /// </param>
    /// <param name="setAsInverse">
    /// 如果为 <see langword="true"/>，<paramref name="inertia"/> 和 <paramref name="mass"/> 将被
    /// 解释为逆值。
    /// </param>
    /// <exception cref="ArgumentException">
    /// 在以下情况下抛出：
    /// <list type="bullet">
    /// <item><description>当 <paramref name="setAsInverse"/> 为 false 时，<paramref name="mass"/> 为零或负数。</description></item>
    /// <item><description>当 <paramref name="setAsInverse"/> 为 true 时，<paramref name="mass"/> 为负数或无限大。</description></item>
    /// <item><description>当 <paramref name="setAsInverse"/> 为 false 时，<paramref name="inertia"/> 不可逆。</description></item>
    /// </list>
    /// </exception>
    public void SetMassInertia(in JSymmetricMatrix inertia, Real mass, bool setAsInverse = false)
    {
        ArgumentCheck.Finite(inertia, nameof(inertia));
        ArgumentCheck.Finite(mass, nameof(mass));

        if (setAsInverse)
        {
            if (Real.IsInfinity(mass) || mass < (Real)0.0)
            {
                throw new ArgumentException("Inverse mass must be finite and not negative.", nameof(mass));
            }

            inverseInertia = inertia;
            inverseMass = mass;
        }
        else
        {
            if (mass <= (Real)0.0)
            {
                throw new ArgumentException("Mass cannot be zero or negative.", nameof(mass));
            }

            if (!JSymmetricMatrix.Inverse(inertia, out JSymmetricMatrix newInverseInertia))
            {
                throw new ArgumentException("Inertia matrix is not invertible.", nameof(inertia));
            }

            inverseInertia = newInverseInertia;
            inverseMass = (Real)1.0 / mass;
        }

        UpdateWorldInertia();
    }

    /// <summary>
    /// 在形状当前的位置和旋转处绘制刚体的碰撞形状。
    /// </summary>
    /// <remarks>
    /// 通过 <see cref="IDebugDrawer.DrawTriangle"/> 输出世界空间三角形。
    /// 点云和自定义形状可能出现缺口或重叠的面。
    /// </remarks>
    /// <param name="drawer">接收形状的调试绘制器。</param>
    public void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <summary>
    /// 使用提供的设置绘制刚体的碰撞形状。
    /// </summary>
    /// <remarks>
    /// 内置曲面使用 <see cref="DebugDrawSettings.Segments"/>，
    /// 近似曲面使用 <see cref="DebugDrawSettings.ShapeSubdivisions"/>，
    /// 三角形使用 <see cref="DebugDrawSettings.CurrentColor"/>。
    /// </remarks>
    /// <param name="drawer">接收形状的调试绘制器。</param>
    /// <param name="settings">可视化设置。</param>
    public void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        foreach (var shape in InternalShapes)
        {
            shape.DebugDraw(drawer, settings);
        }
    }

    /// <summary>
    /// 获取刚体的质量。要修改质量，请使用
    /// <see cref="RigidBody.SetMassInertia(Real)"/> 或
    /// <see cref="RigidBody.SetMassInertia(in JSymmetricMatrix, Real, bool)"/>。
    /// </summary>
    /// <remarks>
    /// 此值仅对 <see cref="MotionType.Dynamic"/> 刚体有意义。
    /// 无论此值为何，静态和运动学刚体在求解器中都被视为具有无限质量。
    /// <see cref="AllowedMotion"/> 会改变沿各个轴的等效响应，
    /// 而不会改变此标量质量。
    /// </remarks>
    public Real Mass => (Real)1.0 / inverseMass;

    int IPartitionedSetIndex.SetIndex { get; set; } = -1;
}
