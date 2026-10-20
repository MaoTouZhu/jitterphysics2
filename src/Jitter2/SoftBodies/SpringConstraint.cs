/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Constraints;
using Jitter2.LinearMath;
using Jitter2.Unmanaged;

namespace Jitter2.SoftBodies;

/// <summary>
/// 约束两个刚体，在锚点之间维持目标距离，
/// 并施加类似弹簧的力。沿连接两个锚点的直线
/// 移除一个平移自由度。
/// </summary>
/// <remarks>
/// 此约束专为软体顶点设计，这些顶点充当质点。
/// 不考虑相连刚体的角速度。
/// 弹簧行为由 <see cref="Softness"/> 和 <see cref="Bias"/> 控制，
/// 既可以直接设置，也可以通过 <see cref="SetSpringParameters"/>
/// 由物理参数计算得出。
/// </remarks>
public unsafe class SpringConstraint : Constraint<SpringConstraint.SpringData>
{
    /// <summary>
    /// 弹簧约束的底层数据，存储于非托管内存中。
    /// </summary>
    [StructLayout(LayoutKind.Sequential)]
    public struct SpringData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAnchor1;
        public JVector LocalAnchor2;

        public Real BiasFactor;
        public Real Softness;
        public Real Distance;

        public Real EffectiveMass;
        public Real AccumulatedImpulse;
        public Real Bias;

        public JVector Jacobian;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationSpringConstraint, &IterateSpringConstraint);

    /// <inheritdoc/>
    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = (Real)0.0;

    /// <summary>
    /// 从世界空间中的锚点初始化该约束。
    /// </summary>
    /// <param name="anchor1">第一个刚体上的锚点，位于世界空间。</param>
    /// <param name="anchor2">第二个刚体上的锚点，位于世界空间。</param>
    /// <remarks>
    /// 根据当前刚体位置计算局部锚点偏移。
    /// 默认值：<see cref="Softness"/> = <see cref="Constraint.DefaultLinearSoftness"/>，<see cref="Bias"/> = <see cref="Constraint.DefaultLinearBias"/>。
    /// <see cref="TargetDistance"/> 设为初始化时两锚点之间的初始距离。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="anchor1"/> 或 <paramref name="anchor2"/> 包含非有限值时抛出。
    /// </exception>
    public void Initialize(JVector anchor1, JVector anchor2)
    {
        VerifyNotZero();
        ArgumentCheck.Finite(anchor1, nameof(anchor1));
        ArgumentCheck.Finite(anchor2, nameof(anchor2));

        ref SpringData data = ref Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Subtract(anchor1, body1.Position, out data.LocalAnchor1);
        JVector.Subtract(anchor2, body2.Position, out data.LocalAnchor2);

        data.Softness = Constraint.DefaultLinearSoftness;
        data.BiasFactor = Constraint.DefaultLinearBias;
        data.Distance = (anchor2 - anchor1).Length();
    }

    /// <summary>
    /// 使用物理属性设置弹簧参数。此方法会计算并设置
    /// <see cref="Softness"/> 和 <see cref="Bias"/> 属性。它假定所涉及的刚体的质量、
    /// 其平移锁定、锚点方向以及时间步长均不发生变化。
    /// </summary>
    /// <remarks>
    /// 当锚点重合时，响应最快的平移轴
    /// 决定用于配置弹簧的有效质量。
    /// </remarks>
    /// <param name="frequency">频率，单位为 Hz。</param>
    /// <param name="damping">阻尼比（0 = 无阻尼，1 = 临界阻尼）。</param>
    /// <param name="dt">模拟的时间步长。</param>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="frequency"/> 或 <paramref name="dt"/> 小于等于零、
    /// 当 <paramref name="damping"/> 为负数，或当任一值非有限值时抛出。
    /// </exception>
    public void SetSpringParameters(Real frequency, Real damping, Real dt)
    {
        ArgumentCheck.Positive(frequency, nameof(frequency));
        ArgumentCheck.NonNegative(damping, nameof(damping));
        ArgumentCheck.Positive(dt, nameof(dt));

        ref SpringData data = ref Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector direction = body2.Position + data.LocalAnchor2 - body1.Position - data.LocalAnchor1;
        Real distance = direction.Length();
        Real inverseEffectiveMass;
        if (distance > 0)
        {
            direction *= (Real)1.0 / distance;
            inverseEffectiveMass = JVector.Multiply(direction, body1.InverseMassVector) * direction +
                                   JVector.Multiply(direction, body2.InverseMassVector) * direction;
        }
        else
        {
            // 方向未定义并不意味着质量无限大。对于不受限制的刚体，
            // 这会恢复其标量逆质量；受限制的刚体则使用响应最快的轴。
            inverseEffectiveMass = JVector.MaxAbs(body1.InverseMassVector + body2.InverseMassVector);
        }

        Real omega = (Real)2.0 * MathR.PI * frequency;
        if (!(inverseEffectiveMass > 0))
        {
            data.Softness = 0;
            data.BiasFactor = dt * omega / ((Real)2.0 * damping + dt * omega);
            return;
        }

        Real effectiveMass = (Real)1.0 / inverseEffectiveMass;
        Real dampingCoefficient = (Real)2.0 * effectiveMass * damping * omega;
        Real springStiffness = effectiveMass * omega * omega;

        Real h = dt;
        data.Softness = (Real)1.0 / (dampingCoefficient + h * springStiffness);
        data.BiasFactor = h * springStiffness * data.Softness;
    }

    /// <summary>
    /// 获取上一个已求解子步累积的求解器冲量。
    /// </summary>
    public Real Impulse
    {
        get
        {
            ref SpringData data = ref Data;
            return data.AccumulatedImpulse;
        }
    }

    /// <summary>
    /// 获取或设置第一个刚体上的锚点，位于世界空间。
    /// </summary>
    public JVector Anchor1
    {
        set
        {
            DebugCheck.IsFinite(value, nameof(value));

            ref SpringData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;
            JVector.Subtract(value, body1.Position, out data.LocalAnchor1);
        }
        get
        {
            ref SpringData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;
            JVector.Add(data.LocalAnchor1, body1.Position, out JVector result);
            return result;
        }
    }

    /// <summary>
    /// 获取或设置第二个刚体上的锚点，位于世界空间。
    /// </summary>
    public JVector Anchor2
    {
        set
        {
            DebugCheck.IsFinite(value, nameof(value));

            ref SpringData data = ref Data;
            ref RigidBodyData body2 = ref data.Body2.Data;
            JVector.Subtract(value, body2.Position, out data.LocalAnchor2);
        }
        get
        {
            ref SpringData data = ref Data;
            ref RigidBodyData body2 = ref data.Body2.Data;
            JVector.Add(data.LocalAnchor2, body2.Position, out JVector result);
            return result;
        }
    }

    /// <summary>
    /// 获取或设置弹簧的目标静止距离。
    /// </summary>
    /// <value>单位：米。默认值为初始化时两锚点之间的初始距离。</value>
    public Real TargetDistance
    {
        set
        {
            DebugCheck.IsNonNegative(value, nameof(value));

            ref SpringData data = ref Data;
            data.Distance = value;
        }
        get => Data.Distance;
    }

    /// <summary>
    /// 获取两锚点之间的当前距离。
    /// </summary>
    public Real Distance
    {
        get
        {
            ref SpringData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;
            ref RigidBodyData body2 = ref data.Body2.Data;

            JVector.Add(body1.Position, data.LocalAnchor1, out JVector p1);
            JVector.Add(body2.Position, data.LocalAnchor2, out JVector p2);

            JVector.Subtract(p2, p1, out JVector dp);

            return dp.Length();
        }
    }

    /// <summary>
    /// 为迭代准备弹簧约束。
    /// </summary>
    /// <param name="constraint">约束数据引用。</param>
    /// <param name="timeStep">完整步和子步的计时信息。</param>
    public static void PrepareForIterationSpringConstraint(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, SpringData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector r1 = data.LocalAnchor1;
        JVector r2 = data.LocalAnchor2;

        JVector.Add(body1.Position, r1, out JVector p1);
        JVector.Add(body2.Position, r2, out JVector p2);

        JVector.Subtract(p2, p1, out JVector dp);

        Real distance = dp.Length();
        Real error = distance - data.Distance;

        if (distance == 0)
        {
            data.Jacobian = JVector.Zero;
            data.EffectiveMass = data.AccumulatedImpulse = data.Bias = 0;
            return;
        }

        JVector n = dp * ((Real)1.0 / distance);

        data.Jacobian = n;
        data.EffectiveMass = JVector.Multiply(n, body1.InverseMassVector) * n +
                             JVector.Multiply(n, body2.InverseMassVector) * n;
        data.EffectiveMass += data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass = data.EffectiveMass > 0 ? (Real)1.0 / data.EffectiveMass : 0;
        if (data.EffectiveMass == 0) data.AccumulatedImpulse = 0;

        data.Bias = error * data.BiasFactor * timeStep.InverseStepDt;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity -= JVector.Multiply(data.AccumulatedImpulse * data.Jacobian, body1.InverseMassVector);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(data.AccumulatedImpulse * data.Jacobian, body2.InverseMassVector);
        }
    }

    /// <summary>
    /// 获取或设置弹簧约束的柔软度（柔顺性）。
    /// </summary>
    /// <value>
    /// 默认值为 <see cref="Constraint.DefaultLinearSoftness"/>。值越大，允许的位置误差越大，但弹簧越软。
    /// 求解时按时间步长的倒数进行缩放。
    /// </value>
    public Real Softness
    {
        get => Data.Softness;
        set
        {
            DebugCheck.IsNonNegative(value, nameof(value));
            Data.Softness = value;
        }
    }

    /// <summary>
    /// 获取或设置弹簧约束的偏置因子（误差修正强度）。
    /// </summary>
    /// <value>
    /// 默认值为 0.2。值越大，修正距离误差越激进。
    /// </value>
    public Real Bias
    {
        get => Data.BiasFactor;
        set
        {
            DebugCheck.IsNonNegative(value, nameof(value));
            Data.BiasFactor = value;
        }
    }

    /// <summary>
    /// 执行弹簧约束求解器的一次迭代。
    /// </summary>
    /// <param name="constraint">约束数据引用。</param>
    /// <param name="timeStep">完整步和子步的计时信息。</param>
    public static void IterateSpringConstraint(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, SpringData>(ref constraint);
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        Real jv = (body2.Velocity - body1.Velocity) * data.Jacobian;

        Real softnessScalar = data.AccumulatedImpulse * data.Softness * timeStep.InverseSubstepDt;

        Real lambda = -data.EffectiveMass * (jv + data.Bias + softnessScalar);

        Real oldAccumulatedImpulse = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;
        lambda = data.AccumulatedImpulse - oldAccumulatedImpulse;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity -= JVector.Multiply(lambda * data.Jacobian, body1.InverseMassVector);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(lambda * data.Jacobian, body2.InverseMassVector);
        }
    }

    /// <inheritdoc/>
    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        JVector reference = JVector.UnitX;
        DebugDraw(drawer, ref reference, settings);
    }

    /// <summary>
    /// 使用调用方持有的螺旋参考方向绘制弹簧。在各帧之间保留此方向，
    /// 可使螺旋在任意运动（包括弹簧轴反向）中保持连续。
    /// </summary>
    public void DebugDraw(IDebugDrawer drawer, ref JVector reference, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        JVector first = Anchor1, second = Anchor2;
        JVector displacement = second - first;
        drawer.DrawSpring(first, second, ref reference, settings.GuideSize * (Real)0.12, settings.CurrentColor,
            Math.Max(32, settings.Segments * 2));
        drawer.DrawPoint(first, settings.PointRadius, settings.ReferenceColor);
        drawer.DrawPoint(second, settings.PointRadius, settings.CurrentColor);
        if (displacement.LengthSquared() > (Real)1e-20)
        {
            JVector direction = JVector.Normalize(displacement);
            drawer.DrawStop(first + direction * TargetDistance, direction,
                settings.GuideSize * (Real)0.12, settings.ReferenceColor);
        }
    }
}
