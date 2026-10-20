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
/// 使用受限的力矩驱动刚体 2 朝向相对于刚体 1 的目标朝向。
/// 控制三个角度自由度；平移保持自由。
/// </summary>
public unsafe class AngularServo : Constraint<AngularServo.AngularServoData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct AngularServoData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JQuaternion TargetRelativeOrientation;
        public Real MaximumTorque;
        public Real Softness;
        public Real BiasFactor;
        public JVector AccumulatedImpulse;
        public JVector Bias;
        public JMatrix EffectiveMass;
        public JMatrix Jacobian;
        public short Hemisphere;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationAngularServo, &IterateAngularServo);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = JVector.Zero;

    /// <summary>以当前相对朝向作为目标来初始化伺服。</summary>
    public void Initialize()
    {
        VerifyNotZero();
        Initialize(Body1.Orientation.Conjugate() * Body2.Orientation);
    }

    /// <summary>初始化刚体 2 相对于刚体 1 的目标朝向。</summary>
    /// <param name="targetRelativeOrientation">刚体 1 坐标系下的归一化朝向。</param>
    /// <remarks>刚体可以起始于目标位姿之外。最大力矩默认为零。</remarks>
    public void Initialize(JQuaternion targetRelativeOrientation)
    {
        VerifyNotZero();
        ArgumentCheck.UnitQuaternion(targetRelativeOrientation, nameof(targetRelativeOrientation));
        ref var data = ref Data;
        data.TargetRelativeOrientation = targetRelativeOrientation;
        data.MaximumTorque = 0;
        data.Softness = Constraint.DefaultAngularSoftness;
        data.BiasFactor = Constraint.DefaultAngularBias;
        data.Hemisphere = 0;
        ResetWarmStart();
    }

    /// <summary>
    /// 获取或设置刚体 2 相对于刚体 1 的目标朝向。取单位四元数时两者的局部坐标系对齐。
    /// 该值必须归一化。修改它不会移动任一刚体。
    /// </summary>
    /// <remarks>伺服使用到目标朝向的最短旋转。不支持圈数计数。</remarks>
    public JQuaternion TargetRelativeOrientation
    {
        get => Data.TargetRelativeOrientation;
        set
        {
            Data.TargetRelativeOrientation = ArgumentCheck.UnitQuaternion(value, nameof(value));
            Data.Hemisphere = 0;
            ResetWarmStart();
            ServoMath.WakeBodies(this);
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

    /// <summary>获取上一个已求解子步中施加到刚体 1 的世界空间角冲量。</summary>
    public JVector Impulse => JVector.TransposedTransform(Data.AccumulatedImpulse, Data.Jacobian);

    public static void PrepareForIterationAngularServo(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, AngularServoData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        data.Jacobian = -(Real)2 * AngularConstraintMath.CalculateJacobian(
            data.TargetRelativeOrientation.Conjugate() * body1.Orientation.Conjugate(),
            body2.Orientation, out JVector error, out short hemisphere);
        if (data.Hemisphere != 0 && data.Hemisphere != hemisphere) data.AccumulatedImpulse = JVector.Zero;
        data.Hemisphere = hemisphere;

        data.EffectiveMass = JSymmetricMatrix.Transform(body1.InverseInertiaWorld + body2.InverseInertiaWorld,
            data.Jacobian).ToMatrix();
        Real softness = data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass.M11 += softness;
        data.EffectiveMass.M22 += softness;
        data.EffectiveMass.M33 += softness;
        data.EffectiveMass = MathHelper.InverseSymmetric(data.EffectiveMass);
        data.Bias = -error * data.BiasFactor * timeStep.InverseStepDt;

        ServoMath.ClampAngular(ref data.AccumulatedImpulse, data.Jacobian,
            ServoMath.MaximumImpulse(data.MaximumTorque, timeStep.SubstepDt));
        ApplyImpulse(ref data, JVector.TransposedTransform(data.AccumulatedImpulse, data.Jacobian));
    }

    public static void IterateAngularServo(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, AngularServoData>(ref constraint);
        JVector jv = JVector.Transform(data.Body1.Data.AngularVelocity - data.Body2.Data.AngularVelocity, data.Jacobian);
        JVector softness = data.AccumulatedImpulse * (data.Softness * timeStep.InverseSubstepDt);
        JVector lambda = -JVector.Transform(jv + data.Bias + softness, data.EffectiveMass);
        JVector previous = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;
        ServoMath.ClampAngular(ref data.AccumulatedImpulse, data.Jacobian,
            ServoMath.MaximumImpulse(data.MaximumTorque, timeStep.SubstepDt));
        ApplyImpulse(ref data, JVector.TransposedTransform(data.AccumulatedImpulse - previous, data.Jacobian));
    }

    private static void ApplyImpulse(ref AngularServoData data, in JVector impulse)
    {
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;
        if (body1.MotionType == MotionType.Dynamic)
            body1.AngularVelocity += JVector.Transform(impulse, body1.InverseInertiaWorld);
        if (body2.MotionType == MotionType.Dynamic)
            body2.AngularVelocity -= JVector.Transform(impulse, body2.InverseInertiaWorld);
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        JVector origin = GetDebugDrawOrigin(settings);
        drawer.DrawFrame(origin, Body1.Orientation * TargetRelativeOrientation, settings with
        {
            AxisXColor = settings.ReferenceColor,
            AxisYColor = settings.ReferenceColor,
            AxisZColor = settings.ReferenceColor
        });
        drawer.DrawFrame(origin, Body2.Orientation, settings);
    }
}
