/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Jitter2.LinearMath;
using Jitter2.Unmanaged;

namespace Jitter2.Dynamics.Constraints;

/// <summary>
/// 一种马达约束，驱动相对线速度沿各自刚体参考系中固定的轴运动。
/// </summary>
public unsafe class LinearMotor : Constraint<LinearMotor.LinearMotorData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct LinearMotorData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAxis1;
        public JVector LocalAxis2;

        public Real Velocity;
        public Real MaxForce;
        public Real MaxLambda;

        public Real EffectiveMass;

        public Real AccumulatedImpulse;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationLinearMotor, &IterateLinearMotor);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = (Real)0.0;

    /// <summary>
    /// 获取或设置第一个刚体在局部空间中的马达轴。
    /// </summary>
    /// <exception cref="ArgumentException">
    /// 当赋值的值不是有限值或未归一化时抛出。
    /// </exception>
    public JVector LocalAxis1
    {
        get => Data.LocalAxis1;
        set
        {
            Data.LocalAxis1 = ArgumentCheck.UnitVector(value, nameof(value));
        }
    }

    /// <summary>
    /// 获取或设置第二个刚体在局部空间中的马达轴。
    /// </summary>
    /// <exception cref="ArgumentException">
    /// 当赋值的值不是有限值或未归一化时抛出。
    /// </exception>
    public JVector LocalAxis2
    {
        get => Data.LocalAxis2;
        set
        {
            Data.LocalAxis2 = ArgumentCheck.UnitVector(value, nameof(value));
        }
    }

    /// <summary>
    /// 使用每个刚体的轴初始化马达。
    /// </summary>
    /// <param name="axis1">第一个刚体在世界空间中的马达轴。</param>
    /// <param name="axis2">第二个刚体在世界空间中的马达轴。</param>
    /// <remarks>
    /// 将轴存储在局部坐标系中。两个轴都会在内部归一化。
    /// 默认值：<see cref="TargetVelocity"/> = 0，<see cref="MaximumForce"/> = 0。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="axis1"/> 或 <paramref name="axis2"/> 为零或包含非有限值时抛出。
    /// </exception>
    public void Initialize(JVector axis1, JVector axis2)
    {
        VerifyNotZero();
        ArgumentCheck.NonZero(axis1, nameof(axis1));
        ArgumentCheck.NonZero(axis2, nameof(axis2));

        ref LinearMotorData data = ref Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.NormalizeInPlace(ref axis1);
        JVector.NormalizeInPlace(ref axis2);

        JVector.ConjugatedTransform(axis1, body1.Orientation, out data.LocalAxis1);
        JVector.ConjugatedTransform(axis2, body2.Orientation, out data.LocalAxis2);

        data.MaxForce = 0;
        data.Velocity = 0;
    }

    /// <summary>
    /// 获取或设置目标线速度，单位为每秒。
    /// </summary>
    /// <value>默认值为 0。</value>
    public Real TargetVelocity
    {
        get => Data.Velocity;
        set
        {
            DebugCheck.IsFinite(value, nameof(value));
            Data.Velocity = value;
        }
    }

    /// <summary>
    /// 获取或设置马达可施加的最大力。
    /// </summary>
    /// <value>默认值为 0。必须为非负值。</value>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="value"/> 为负时抛出。
    /// </exception>
    public Real MaximumForce
    {
        get => Data.MaxForce;
        set
        {
            Data.MaxForce = ArgumentCheck.NonNegative(value, nameof(value));
        }
    }

    /// <summary>
    /// 获取上一次已求解子步中累积的求解器冲量。
    /// </summary>
    public Real Impulse => Data.AccumulatedImpulse;

    public static void PrepareForIterationLinearMotor(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, LinearMotorData>(ref constraint);

        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Transform(data.LocalAxis1, body1.Orientation, out JVector j1);
        JVector.Transform(data.LocalAxis2, body2.Orientation, out JVector j2);

        data.EffectiveMass = JVector.Multiply(j1, body1.InverseMassVector) * j1 +
                             JVector.Multiply(j2, body2.InverseMassVector) * j2;
        data.EffectiveMass = data.EffectiveMass > 0 ? (Real)1.0 / data.EffectiveMass : 0;
        if (data.EffectiveMass == 0) data.AccumulatedImpulse = 0;
        data.MaxLambda = timeStep.SubstepDt * data.MaxForce;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity -= JVector.Multiply(j1 * data.AccumulatedImpulse, body1.InverseMassVector);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(j2 * data.AccumulatedImpulse, body2.InverseMassVector);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        ref LinearMotorData data = ref Data;
        JVector first = JVector.Transform(data.LocalAxis1, Body1.Orientation);
        JVector second = JVector.Transform(data.LocalAxis2, Body2.Orientation);
        JVector referenceOrigin = Body1 == Body1.World.NullBody ? Body2.Position : Body1.Position;
        JVector secondOrigin = Body2 == Body2.World.NullBody ? Body1.Position : Body2.Position;
        drawer.DrawSegment(referenceOrigin - first * settings.GuideSize, referenceOrigin + first * settings.GuideSize, settings.ReferenceColor);
        drawer.DrawSegment(secondOrigin - second * settings.GuideSize, secondOrigin + second * settings.GuideSize, settings.CurrentColor);
        if (data.Velocity != 0)
        {
            Real sign = data.Velocity > 0 ? 1 : -1;
            // 求解器将 dot(axis2, v2) - dot(axis1, v1) 驱向 TargetVelocity。
            drawer.DrawArrow(referenceOrigin, -first * (sign * settings.GuideSize * (Real)0.5),
                settings.ReferenceColor, settings);
            drawer.DrawArrow(secondOrigin, second * (sign * settings.GuideSize * (Real)0.5),
                settings.CurrentColor, settings);
        }
    }

    public static void IterateLinearMotor(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, LinearMotorData>(ref constraint);
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        JVector.Transform(data.LocalAxis1, body1.Orientation, out JVector j1);
        JVector.Transform(data.LocalAxis2, body2.Orientation, out JVector j2);

        Real jv = -j1 * body1.Velocity + j2 * body2.Velocity;

        Real lambda = -(jv - data.Velocity) * data.EffectiveMass;

        Real oldAccumulated = data.AccumulatedImpulse;

        data.AccumulatedImpulse += lambda;

        data.AccumulatedImpulse = Math.Clamp(data.AccumulatedImpulse, -data.MaxLambda, data.MaxLambda);

        lambda = data.AccumulatedImpulse - oldAccumulated;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity -= JVector.Multiply(j1 * lambda, body1.InverseMassVector);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(j2 * lambda, body2.InverseMassVector);
        }
    }
}
