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
/// 使用受限的力矩驱动相对扭转朝向目标角度。
/// 控制一个角度自由度；平移和摆动保持自由。
/// </summary>
/// <remarks>
/// 使用 [-pi, pi] 范围内的有符号关节角，不追踪圈数。
/// 应与将运动限制在此角度区间内的机械限位配合使用。
/// </remarks>
public unsafe class TwistServo : Constraint<TwistServo.TwistServoData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct TwistServoData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAxis2;
        public JQuaternion ReferenceOrientation;
        public Real TargetAngle;
        public Real MaximumTorque;
        public Real Softness;
        public Real BiasFactor;
        public Real EffectiveMass;
        public Real AccumulatedImpulse;
        public Real Bias;
        public JVector Jacobian;
        public short Hemisphere;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationTwistServo, &IterateTwistServo);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = 0;

    /// <summary>以同一世界空间方向初始化两个扭转轴。</summary>
    public void Initialize(JVector axis, JAngle targetAngle = default) => Initialize(axis, axis, targetAngle);

    /// <summary>初始化两个独立的世界空间扭转轴，以及相对于初始位姿的目标。</summary>
    /// <remarks>
    /// 使两个轴对齐的最短旋转定义零扭转。最大力矩默认为零。
    /// 非零目标角会使伺服起始于其目标构型之外。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">目标角不是有限值，或超出 [-pi, pi]。</exception>
    public void Initialize(JVector axis1, JVector axis2, JAngle targetAngle = default)
    {
        VerifyNotZero();
        ArgumentCheck.NonZero(axis1, nameof(axis1));
        ArgumentCheck.NonZero(axis2, nameof(axis2));
        Real target = ArgumentCheck.InRange((Real)targetAngle, -MathR.PI, MathR.PI, nameof(targetAngle));
        JVector.NormalizeInPlace(ref axis1);
        JVector.NormalizeInPlace(ref axis2);
        ref var data = ref Data;
        data.LocalAxis2 = JVector.ConjugatedTransform(axis2, Body2.Orientation);
        JVector firstInBody2 = JVector.ConjugatedTransform(axis1, Body2.Orientation);
        data.ReferenceOrientation = JQuaternion.CreateFromToRotation(firstInBody2, data.LocalAxis2) *
                                    (Body2.Orientation.Conjugate() * Body1.Orientation);
        data.TargetAngle = target;
        data.MaximumTorque = 0;
        data.Softness = Constraint.DefaultAngularSoftness;
        data.BiasFactor = Constraint.DefaultAngularBias;
        data.Hemisphere = 0;
        ResetWarmStart();
    }

    /// <summary>获取或设置相对于初始零扭转位姿的期望扭转角。</summary>
    /// <remarks>
    /// 目标必须位于 [-pi, pi] 内。有符号角度误差不会被回绕：从
    /// -170 度移动到 +170 度会经过零。不支持圈数计数。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">该值不是有限值，或超出 [-pi, pi]。</exception>
    public JAngle TargetAngle
    {
        get => (JAngle)Data.TargetAngle;
        set
        {
            Data.TargetAngle = ArgumentCheck.InRange((Real)value, -MathR.PI, MathR.PI, nameof(value));
            ResetWarmStart();
            ServoMath.WakeBodies(this);
        }
    }

    /// <summary>获取 [-pi, pi] 范围内的当前相对扭转角。不追踪圈数。</summary>
    public JAngle Angle
    {
        get
        {
            ref var data = ref Data;
            return (JAngle)AngularConstraintMath.CalculateTwist(
                data.ReferenceOrientation * Body1.Orientation.Conjugate(), Body2.Orientation,
                data.LocalAxis2, out _, out _);
        }
    }

    /// <summary>获取或设置伺服可施加的最大总力矩。</summary>
    /// <value>默认为零。正无穷表示不限制力矩。</value>
    public Real MaximumTorque
    {
        get => Data.MaximumTorque;
        set
        {
            Data.MaximumTorque = ServoMath.CheckMaximum(value);
            ServoMath.WakeBodies(this);
        }
    }

    /// <summary>获取或设置伺服的柔软度（柔顺度）。</summary>
    /// <value>默认为 <see cref="Constraint.DefaultAngularSoftness"/>。</value>
    public Real Softness
    {
        get => Data.Softness;
        set => Data.Softness = ArgumentCheck.NonNegative(value, nameof(value));
    }

    /// <summary>获取或设置伺服修正其位置误差的强度。</summary>
    /// <value>默认为 <see cref="Constraint.DefaultAngularBias"/>。</value>
    public Real Bias
    {
        get => Data.BiasFactor;
        set => Data.BiasFactor = ArgumentCheck.NonNegative(value, nameof(value));
    }

    /// <summary>获取上一个已求解子步中的有符号角冲量。</summary>
    public Real Impulse => Data.AccumulatedImpulse;

    public static void PrepareForIterationTwistServo(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, TwistServoData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;
        Real angle = AngularConstraintMath.CalculateTwist(data.ReferenceOrientation * body1.Orientation.Conjugate(),
            body2.Orientation, data.LocalAxis2, out data.Jacobian, out short hemisphere);
        if (data.Hemisphere != 0 && data.Hemisphere != hemisphere) data.AccumulatedImpulse = 0;
        data.Hemisphere = hemisphere;
        Real mass = JVector.Transform(data.Jacobian, body1.InverseInertiaWorld + body2.InverseInertiaWorld) * data.Jacobian +
                    data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass = mass > 0 ? (Real)1 / mass : 0;
        if (data.EffectiveMass == 0) data.AccumulatedImpulse = 0;
        // 关节角目标必须沿其角度区间到达，即使跨越 -pi/pi 边界
        // 会给出更短的旋转。
        Real error = angle - data.TargetAngle;
        data.Bias = error * data.BiasFactor * timeStep.InverseStepDt;
        Real maximum = ServoMath.MaximumImpulse(data.MaximumTorque, timeStep.SubstepDt);
        data.AccumulatedImpulse = Math.Clamp(data.AccumulatedImpulse, -maximum, maximum);
        ApplyImpulse(ref data, data.AccumulatedImpulse);
    }

    public static void IterateTwistServo(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, TwistServoData>(ref constraint);
        Real jv = (data.Body1.Data.AngularVelocity - data.Body2.Data.AngularVelocity) * data.Jacobian;
        Real lambda = -data.EffectiveMass * (jv + data.Bias +
            data.AccumulatedImpulse * data.Softness * timeStep.InverseSubstepDt);
        Real previous = data.AccumulatedImpulse;
        Real maximum = ServoMath.MaximumImpulse(data.MaximumTorque, timeStep.SubstepDt);
        data.AccumulatedImpulse = Math.Clamp(previous + lambda, -maximum, maximum);
        ApplyImpulse(ref data, data.AccumulatedImpulse - previous);
    }

    private static void ApplyImpulse(ref TwistServoData data, Real impulse)
    {
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;
        if (body1.MotionType == MotionType.Dynamic)
            body1.AngularVelocity += JVector.Transform(data.Jacobian * impulse, body1.InverseInertiaWorld);
        if (body2.MotionType == MotionType.Dynamic)
            body2.AngularVelocity -= JVector.Transform(data.Jacobian * impulse, body2.InverseInertiaWorld);
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        ref var data = ref Data;
        JQuaternion reference = Body1.Orientation * data.ReferenceOrientation.Conjugate();
        JVector axis = JVector.Transform(data.LocalAxis2, reference);
        JVector direction = JVector.Transform(MathHelper.CreateOrthonormal(data.LocalAxis2), reference);
        DebugDrawGeometry.AngularRange(drawer, GetDebugDrawOrigin(settings), axis, direction,
            data.TargetAngle, data.TargetAngle, Angle.Radian, settings);
    }
}
