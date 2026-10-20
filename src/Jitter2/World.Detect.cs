/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Diagnostics;
using System.Diagnostics.CodeAnalysis;
using System.Runtime.CompilerServices;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;
using Jitter2.Unmanaged;

namespace Jitter2;

public sealed partial class World
{
    /// <summary>
    /// 当窄相遇到一对无法处理的代理类型时抛出。
    /// </summary>
    /// <remarks>
    /// 这通常表示非 <see cref="RigidBodyShape"/> 代理被插入了世界的 <see cref="DynamicTree"/> 中。
    /// 使用 <see cref="BroadPhaseFilter"/> 过滤此类配对，
    /// 或确保只添加受支持的代理类型。
    /// </remarks>
    public class InvalidCollisionTypeException : Exception
    {
        private static string CreateMessage(Type proxyA, Type proxyB) =>
            $"Don't know how to handle collision between {proxyA} and {proxyB}." +
            " Register a BroadPhaseFilter to handle and/or filter out these collision types.";

        public InvalidCollisionTypeException()
        {
        }

        public InvalidCollisionTypeException(string message)
            : base(message)
        {
        }

        public InvalidCollisionTypeException(string message, Exception innerException)
            : base(message, innerException)
        {
        }

        public InvalidCollisionTypeException(Type proxyA, Type proxyB)
            : base(CreateMessage(proxyA, proxyB))
        {
            ProxyA = proxyA;
            ProxyB = proxyB;
        }

        public Type? ProxyA { get; }

        public Type? ProxyB { get; }
    }

    /// <summary>
    /// 接入窄相碰撞检测管线。
    /// 默认实例的类型为 <see cref="TriangleEdgeCollisionFilter"/>。
    /// </summary>
    /// <remarks>
    /// 用于在生成接触后拦截碰撞、修改接触数据，
    /// 或实现自定义碰撞响应。当以 <c>multiThread=true</c> 调用 <see cref="Step(Real, bool)"/> 时，
    /// 此对象可能被并发调用。实现必须线程安全。
    /// </remarks>
    public INarrowPhaseFilter? NarrowPhaseFilter { get; set; } = new TriangleEdgeCollisionFilter();

    /// <summary>
    /// 接入宽相碰撞检测管线。默认值为 null。
    /// </summary>
    /// <remarks>
    /// 用于在窄相检测前拦截形状配对、实现自定义碰撞层，
    /// 或处理自定义代理类型的碰撞。当以 <c>multiThread=true</c> 调用 <see cref="Step(Real, bool)"/> 时，
    /// 此对象可能被并发调用。实现必须线程安全。
    /// </remarks>
    public IBroadPhaseFilter? BroadPhaseFilter { get; set; }

    /// <summary>
    /// 启用为相互接触的平坦表面生成额外接触点。
    /// 传统上，碰撞系统只报告两个物体之间最深的碰撞点。
    /// 随后借助接触缓存、经过多个时间步生成完整的接触流形，这种方式
    /// 可能不稳定。本方法尝试在单个时间步内构建更完整乃至完整的接触流形。
    /// </summary>
    public bool EnableAuxiliaryContactPoints { set; get; } = true;

    /// <summary>
    /// 启用时（默认），接触点及其累积冲量会在帧之间缓存。
    /// 这使求解器能够从上一帧的解热启动，并让流形
    /// 在多个步中逐步增长，从而提高静止接触的稳定性。
    /// 禁用时，所有接触数据会在每帧结束时丢弃，每个接触都被
    /// 视为全新的。禁用此选项会清除所有帧间接触记忆，代价是
    /// 求解器收敛速度下降。
    /// </summary>
    public bool PersistentContactManifold { get; set; } = true;

    /// <summary>
    /// 推测接触会限制相对线性接近速度，以帮助防止物体在一帧内穿透
    /// 障碍物。它不考虑旋转运动。
    /// <see cref="SpeculativeRelaxationFactor"/> 会缩放减速程度，取值范围从 0
    /// （物体在本帧内立即停止）到 1（物体与障碍物在下一次速度积分后
    /// 刚好接触）。建议使用小于 1 的值，因为剩余速度
    /// 可能足以在下一帧触发另一次推测接触。
    /// 默认值：0.9。
    /// </summary>
    public Real SpeculativeRelaxationFactor { get; set; } = (Real)0.9;

    /// <summary>
    /// 当两个刚体之间的相对线速度超过阈值时，会生成推测接触。为防止直径为 D 的刚体
    /// 穿透薄墙，该阈值应设置为大约 D / 时间步，例如对于单位立方体和
    /// 0.01 的时间步，阈值取 100。
    /// 默认值：10.0。
    /// </summary>
    /// <remarks>
    /// 角速度不计入该阈值。内置的推测接触
    /// 在固定朝向下预测平移，无法防止由旋转引起的穿透。
    /// </remarks>
    public Real SpeculativeVelocityThreshold { get; set; } = (Real)10.0;

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void Detect(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB)
    {
        IBroadPhaseFilter? broadPhaseFilter = BroadPhaseFilter;
        if (broadPhaseFilter != null)
        {
            if (!broadPhaseFilter.Filter(proxyA, proxyB))
            {
                return;
            }
        }

        if (proxyA is not RigidBodyShape sA || proxyB is not RigidBodyShape sB)
        {
            throw new InvalidCollisionTypeException(proxyA.GetType(), proxyB.GetType());
        }

        if (sB.ShapeId < sA.ShapeId)
        {
            (sA, sB) = (sB, sA);
        }

        RigidBody bodyA = sA.RigidBody;
        RigidBody bodyB = sB.RigidBody;

        Debug.Assert(bodyA != null);
        Debug.Assert(bodyB != null);
        Debug.Assert(bodyA != bodyB);
        Debug.Assert(bodyA.World == this);
        Debug.Assert(bodyB.World == this);

        ref RigidBodyData b1 = ref bodyA.Data;
        ref RigidBodyData b2 = ref bodyB.Data;

        if (!b1.IsActive && !b2.IsActive) return;

        if (b1.MotionType != MotionType.Dynamic && b2.MotionType != MotionType.Dynamic) return;

        bool speculative = bodyA.EnableSpeculativeContacts || bodyB.EnableSpeculativeContacts;
        INarrowPhaseFilter? narrowPhaseFilter = NarrowPhaseFilter;

        var colliding = NarrowPhase.MprEpa(sA, sB, b1.Orientation, b2.Orientation, b1.Position, b2.Position,
            out JVector pA, out JVector pB, out JVector normal, out var penetration);

        if (!colliding)
        {
            if (!speculative) return;

            JVector dv = b2.Velocity - b1.Velocity;

            if (dv.LengthSquared() < SpeculativeVelocityThreshold * SpeculativeVelocityThreshold) return;

            bool success = NarrowPhase.Sweep(sA, sB, b1.Orientation, b2.Orientation,
                b1.Position, b2.Position, b1.Velocity, b2.Velocity,
                out pA, out pB, out normal, out Real toi);

            // ReSharper disable once CompareOfFloatsByEqualityOperator
            if (!success || toi > timeStep.StepDt || toi == (Real)0.0) return;

            penetration = normal * (pA - pB) * SpeculativeRelaxationFactor;

            if (narrowPhaseFilter != null)
            {
                if (!narrowPhaseFilter.Filter(sA, sB, ref pA, ref pB, ref normal, ref penetration))
                {
                    return;
                }
            }

            RegisterContact(sA.ShapeId, sB.ShapeId, bodyA, bodyB,
                pA, pB, normal, ContactData.SolveMode.Angular);

            return;
        }

        if (narrowPhaseFilter != null)
        {
            if (!narrowPhaseFilter.Filter(sA, sB, ref pA, ref pB, ref normal, ref penetration))
            {
                return;
            }
        }

        if (EnableAuxiliaryContactPoints)
        {
            Unsafe.SkipInit(out CollisionManifold manifold);
            manifold.BuildManifold(sA, sB, pA, pB, normal);

            RegisterContact(sA.ShapeId, sB.ShapeId, bodyA, bodyB, normal, ref manifold);
        }
        else
        {
            RegisterContact(sA.ShapeId, sB.ShapeId, bodyA, bodyB, pA, pB, normal);
        }
    }

    /// <summary>
    /// 将一个接触点注册到已有的 <see cref="Arbiter"/> 中。
    /// </summary>
    /// <remarks>
    /// 此方法使用提供的接触点和法线，把一个接触点添加到指定的 <paramref name="arbiter"/> 中。
    /// 所有输入向量必须位于世界空间。<paramref name="normal"/> 向量必须已归一化。
    /// 此方法假定 <paramref name="arbiter"/> 已有效，且已映射到正确的刚体对。
    /// <para>
    /// 创建或注册接触的调用可以彼此并发执行。它们不得与 <see cref="Step(Real, bool)"/>、
    /// <see cref="Remove(Arbiter)"/>、刚体或形状移除、<see cref="Clear"/>、<see cref="Dispose"/>
    /// 或其他改变拓扑的操作并发执行。
    /// 如果自定义接触生成使用了外部工作线程，则所有这些工作都必须在世界
    /// 步进或修改之前完成。
    /// </para>
    /// <para>
    /// 不要在仲裁器已从世界中移除后传入它；已移除的仲裁器可能被回收。
    /// </para>
    /// </remarks>
    /// <param name="arbiter">要添加接触的现有 <see cref="Arbiter"/> 实例。</param>
    /// <param name="point1">第一个刚体上的接触点，位于世界空间。</param>
    /// <param name="point2">第二个刚体上的接触点，位于世界空间。</param>
    /// <param name="normal">接触法线，位于世界空间。必须已归一化；正的法线
    /// 冲量会沿此方向推动仲裁器的第二个刚体，并沿相反方向推动其第一个刚体。</param>
    /// <param name="removeFlags"><see cref="ContactData.SolveMode"/> 标志的位掩码，将从完整的
    /// 接触求解中移除（参见 <see cref="ContactData.SolveMode.Full"/>）。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RegisterContact(Arbiter arbiter, in JVector point1, in JVector point2,
        in JVector normal, ContactData.SolveMode removeFlags = ContactData.SolveMode.None)
    {
        lock (arbiter)
        {
            memContacts.ResizeLock.EnterReadLock();
            try
            {
                if (!PersistentContactManifold) arbiter.Handle.Data.UsageMask = 0;
                arbiter.Handle.Data.ResetMode(removeFlags);
                arbiter.Handle.Data.AddContact(point1, point2, normal);
            }
            finally
            {
                memContacts.ResizeLock.ExitReadLock();
            }
        }
    }

    /// <summary>
    /// 使用 <see cref="CollisionManifold"/> 在两个刚体之间注册一个或多个接触点，
    /// 若 <see cref="Arbiter"/> 尚不存在则创建它。
    /// </summary>
    /// <remarks>
    /// 此方法确保指定 ID 对之间的接触信息由 <see cref="Arbiter"/> 跟踪。
    /// 如果给定 ID 尚无仲裁器，则使用 <paramref name="body1"/> 和 <paramref name="body2"/> 创建一个。
    ///
    /// <para>
    /// 创建或注册接触的调用可以彼此并发执行。它们不得与 <see cref="Step(Real, bool)"/>、
    /// <see cref="Remove(Arbiter)"/>、刚体或形状移除、<see cref="Clear"/>、<see cref="Dispose"/>
    /// 或其他改变拓扑的操作并发执行。
    /// 如果自定义接触生成使用了外部工作线程，则所有这些工作都必须在世界
    /// 步进或修改之前完成。
    /// </para>
    ///
    /// <para><b>Note：</b><paramref name="id0"/> 与 <paramref name="id1"/> 的顺序<i>确实重要</i>。</para>
    /// </remarks>
    /// <param name="id0">与接触关联的第一个标识符（例如形状或特征 ID）。</param>
    /// <param name="id1">与接触关联的第二个标识符。</param>
    /// <param name="body1">参与接触的第一个刚体。</param>
    /// <param name="body2">参与接触的第二个刚体。</param>
    /// <param name="normal">
    /// 接触法线，位于世界空间。必须是单位向量，方向从 <paramref name="body1"/> 指向 <paramref name="body2"/>。
    /// </param>
    /// <param name="manifold">包含世界空间中接触点对的 <see cref="CollisionManifold"/>。</param>
    /// <param name="removeFlags"><see cref="ContactData.SolveMode"/> 标志的位掩码，将从完整的
    /// 接触求解中移除（参见 <see cref="ContactData.SolveMode.Full"/>）。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RegisterContact(ulong id0, ulong id1, RigidBody body1, RigidBody body2, in JVector normal,
        ref CollisionManifold manifold, ContactData.SolveMode removeFlags = ContactData.SolveMode.None)
    {
        GetOrCreateArbiter(id0, id1, body1, body2, out Arbiter arbiter);

        lock (arbiter)
        {
            // 在接触可能被调整大小时不要添加接触
            memContacts.ResizeLock.EnterReadLock();

            try
            {
                if (!PersistentContactManifold) arbiter.Handle.Data.UsageMask = 0;
                arbiter.Handle.Data.ResetMode(removeFlags);

                ReadOnlySpan<JVector> manifoldA = manifold.ManifoldA;
                ReadOnlySpan<JVector> manifoldB = manifold.ManifoldB;

                for (int e = 0; e < manifold.Count; e++)
                {
                    JVector mfA = manifoldA[e];
                    JVector mfB = manifoldB[e];
                    arbiter.Handle.Data.AddContact(mfA, mfB, normal);
                }
            }
            finally
            {
                memContacts.ResizeLock.ExitReadLock();
            }
        }
    }

    /// <summary>
    /// 在两个刚体之间注册一个接触点，若 <see cref="Arbiter"/> 尚不存在则创建它。
    /// </summary>
    /// <remarks>
    /// 此方法确保指定 ID 对之间的接触信息由 <see cref="Arbiter"/> 跟踪。
    /// 如果给定 ID 尚无仲裁器，则使用 <paramref name="body1"/> 和 <paramref name="body2"/> 创建一个。
    /// 提供的接触点和法线必须位于世界空间。<paramref name="normal"/> 向量必须已归一化。
    ///
    /// <para>
    /// 创建或注册接触的调用可以彼此并发执行。它们不得与 <see cref="Step(Real, bool)"/>、
    /// <see cref="Remove(Arbiter)"/>、刚体或形状移除、<see cref="Clear"/>、<see cref="Dispose"/>
    /// 或其他改变拓扑的操作并发执行。
    /// 如果自定义接触生成使用了外部工作线程，则所有这些工作都必须在世界
    /// 步进或修改之前完成。
    /// </para>
    ///
    /// <para><b>Note：</b><paramref name="id0"/> 与 <paramref name="id1"/> 的顺序<i>确实重要</i>。</para>
    /// </remarks>
    /// <param name="id0">与接触关联的第一个标识符（例如形状或特征 ID）。</param>
    /// <param name="id1">与接触关联的第二个标识符。</param>
    /// <param name="body1">参与接触的第一个刚体。</param>
    /// <param name="body2">参与接触的第二个刚体。</param>
    /// <param name="point1"><paramref name="body1"/> 上的接触点，位于世界空间。</param>
    /// <param name="point2"><paramref name="body2"/> 上的接触点，位于世界空间。</param>
    /// <param name="normal">
    /// 接触法线，位于世界空间。必须是单位向量，方向从 <paramref name="body1"/> 指向 <paramref name="body2"/>。
    /// </param>
    /// <param name="removeFlags"><see cref="ContactData.SolveMode"/> 标志的位掩码，将从完整的
    /// 接触求解中移除（参见 <see cref="ContactData.SolveMode.Full"/>）。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void RegisterContact(ulong id0, ulong id1, RigidBody body1, RigidBody body2,
        in JVector point1, in JVector point2, in JVector normal,
        ContactData.SolveMode removeFlags = ContactData.SolveMode.None)
    {
        GetOrCreateArbiter(id0, id1, body1, body2, out Arbiter arbiter);
        RegisterContact(arbiter, point1, point2, normal, removeFlags);
    }

    /// <summary>
    /// 获取给定 ID 对对应的现有 <see cref="Arbiter"/> 实例。
    /// </summary>
    /// <param name="id0">第一个标识符（例如形状 ID）。</param>
    /// <param name="id1">第二个标识符。</param>
    /// <param name="arbiter">该有序 ID 对的仲裁器；若不存在则为 <see langword="null"/>。</param>
    /// <returns>如果该有序 ID 对存在仲裁器则返回 <see langword="true"/>；否则返回 <see langword="false"/>。</returns>
    /// <remarks>
    /// <paramref name="id0"/> 与 <paramref name="id1"/> 的顺序很重要。
    /// 对于引擎创建的仲裁器，<see cref="RigidBodyShape"/> 满足
    /// <paramref name="id0"/> &lt; <paramref name="id1"/>。
    /// <para>
    /// 此方法可以与接触创建和注册并发执行。它不得与 <see cref="Step(Real, bool)"/>、
    /// <see cref="Remove(Arbiter)"/>、刚体或形状移除、<see cref="Clear"/>、<see cref="Dispose"/>
    /// 或其他改变拓扑的操作并发执行。
    /// </para>
    /// </remarks>
    public bool GetArbiter(ulong id0, ulong id1, [MaybeNullWhen(false)] out Arbiter arbiter)
    {
        ArbiterKey arbiterKey = new(id0, id1);

        lock (arbiters.GetLock(arbiterKey))
        {
            return arbiters.TryGetValue(arbiterKey, out arbiter!);
        }
    }

    /// <summary>
    /// 获取给定 ID 对对应的现有 <see cref="Arbiter"/> 实例，
    /// 若不存在则创建一个新的。
    /// </summary>
    /// <remarks>
    /// 此方法确保每个有序 ID 对都有唯一的 <see cref="Arbiter"/>。
    /// 如果仲裁器已存在，则通过 <paramref name="arbiter"/> 输出参数返回它。
    /// 否则，分配一个新的仲裁器，用提供的 <paramref name="body1"/> 和 <paramref name="body2"/> 初始化，
    /// 并在内部注册。仅当创建新仲裁器时才会使用刚体参数。
    ///
    /// <para>
    /// 创建或注册接触的调用可以彼此并发执行。它们不得与 <see cref="Step(Real, bool)"/>、
    /// <see cref="Remove(Arbiter)"/>、刚体或形状移除、<see cref="Clear"/>、<see cref="Dispose"/>
    /// 或其他改变拓扑的操作并发执行。
    /// 如果自定义接触生成使用了外部工作线程，则所有这些工作都必须在世界
    /// 步进或修改之前完成。
    /// </para>
    ///
    /// <para><b>Note：</b><paramref name="id0"/> 与 <paramref name="id1"/> 的顺序<i>确实重要</i>。</para>
    /// </remarks>
    /// <param name="id0">与接触关联的第一个标识符（例如形状或特征 ID）。</param>
    /// <param name="id1">与接触关联的第二个标识符。</param>
    /// <param name="body1">第一个刚体。仅在创建新仲裁器时使用。</param>
    /// <param name="body2">第二个刚体。仅在创建新仲裁器时使用。</param>
    /// <param name="arbiter">与该 ID 对关联的结果 <see cref="Arbiter"/> 实例。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public void GetOrCreateArbiter(ulong id0, ulong id1, RigidBody body1, RigidBody body2, out Arbiter arbiter)
    {
        ArbiterKey arbiterKey = new(id0, id1);

        lock (arbiters.GetLock(arbiterKey))
        {
            if (arbiters.TryGetValue(arbiterKey, out arbiter!))
            {
                Debug.Assert(arbiter.Body1 == body1 && arbiter.Body2 == body2);
                return;
            }

            arbiter = CreateArbiter(arbiterKey, body1, body2);
        }
    }

    private Arbiter RentArbiter()
    {
        return arbiterPool.TryPop(out Arbiter? arbiter) ? arbiter : new Arbiter();
    }

    private void ReturnArbiter(Arbiter arbiter)
    {
        arbiter.Handle = JHandle<ContactData>.Zero;
        arbiter.Body1 = null!;
        arbiter.Body2 = null!;

        // 已排队的 BeginCollide 通知可能仍引用此实例。
        if (dispatchArbiters.Count > 0) retiredArbiters.Add(arbiter);
        else arbiterPool.Push(arbiter);
    }

    // 调用方在初始化和发布完成之前一直持有分片锁。
    private Arbiter CreateArbiter(ArbiterKey arbiterKey, RigidBody body1, RigidBody body2)
    {
        Debug.Assert(body1 != body2);
        Debug.Assert(body1.IsValid);
        Debug.Assert(body1.World == this);
        Debug.Assert(body2.IsValid);
        Debug.Assert(body2.World == this);

        Arbiter? arbiter = null;

        try
        {
            lock (memContacts)
            {
                arbiter = RentArbiter();

                var handle = memContacts.Allocate(true);
                arbiter.Handle = handle;
                handle.Data.Init(body1, body2);
                handle.Data.Key = arbiterKey;
                arbiter.Body1 = body1;
                arbiter.Body2 = body2;

                deferredArbiters.Add(arbiter);

                Debug.Assert(memContacts.IsActive(arbiter.Handle));
            }

            // 字典扩容只会阻塞本分片，而不会阻塞所有接触创建者。
            arbiters.Add(arbiterKey, arbiter);
            return arbiter;
        }
        catch
        {
            if (arbiter != null)
            {
                lock (memContacts)
                {
                    deferredArbiters.Remove(arbiter);

                    if (!arbiter.Handle.IsZero)
                    {
                        // Free 可能移动另一个已发布的接触。在其数据被移动期间
                        // 排除并发的 RegisterContact 调用。
                        memContacts.ResizeLock.EnterWriteLock();
                        try
                        {
                            memContacts.Free(arbiter.Handle);
                        }
                        finally
                        {
                            memContacts.ResizeLock.ExitWriteLock();
                        }
                    }

                    ReturnArbiter(arbiter);
                }
            }

            throw;
        }
    }
}
