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
/// 约束两个刚体之间的相对朝向，消除三个转动自由度。
/// </summary>
public unsafe class FixedAngle : Constraint<FixedAngle.FixedAngleData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct FixedAngleData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public Real MinAngle;
        public Real MaxAngle;

        public Real BiasFactor;
        public Real Softness;

        public JQuaternion Q0;

        public JVector AccumulatedImpulse;
        public JVector Bias;

        public JMatrix EffectiveMass;
        public JMatrix Jacobian;

        public ushort Clamp;
        public short Hemisphere;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationFixedAngle, &IterateFixedAngle);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = JVector.Zero;

    /// <summary>
    /// 使用两个刚体当前的相对朝向初始化约束。
    /// </summary>
    /// <remarks>
    /// 将当前相对朝向记录为目标朝向。
    /// 默认值：<see cref="Softness"/> = <see cref="Constraint.DefaultAngularSoftness"/>，<see cref="Bias"/> = <see cref="Constraint.DefaultAngularBias"/>。
    /// </remarks>
    public void Initialize()
    {
        VerifyNotZero();
        ref FixedAngleData data = ref Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        data.Softness = Constraint.DefaultAngularSoftness;
        data.BiasFactor = Constraint.DefaultAngularBias;

        JQuaternion q1 = body1.Orientation;
        JQuaternion q2 = body2.Orientation;

        data.Q0 = q2.Conjugate() * q1;
    }

    public static void PrepareForIterationFixedAngle(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, FixedAngleData>(ref constraint);

        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JQuaternion q1 = body1.Orientation;
        JQuaternion q2 = body2.Orientation;

        data.Clamp = 1024;

        // 在大角度旋转时，使用带有非奇异旋转坐标的小角度缩放，
        // 而不是四元数的向量部分。
        data.Jacobian = -(Real)2 * AngularConstraintMath.CalculateJacobian(
            data.Q0 * q1.Conjugate(), q2, out JVector error, out short hemisphere);
        if (data.Hemisphere != 0 && data.Hemisphere != hemisphere)
            data.AccumulatedImpulse = JVector.Zero;
        data.Hemisphere = hemisphere;

        data.EffectiveMass = JSymmetricMatrix.Transform(body1.InverseInertiaWorld + body2.InverseInertiaWorld,
            data.Jacobian).ToMatrix();

        data.EffectiveMass.M11 += data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass.M22 += data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass.M33 += data.Softness * timeStep.InverseSubstepDt;

        data.EffectiveMass = MathHelper.InverseSymmetric(data.EffectiveMass);

        data.Bias = -error * data.BiasFactor * timeStep.InverseStepDt;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.AngularVelocity += JVector.Transform(JVector.TransposedTransform(data.AccumulatedImpulse, data.Jacobian), body1.InverseInertiaWorld);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.AngularVelocity -= JVector.Transform(JVector.TransposedTransform(data.AccumulatedImpulse, data.Jacobian), body2.InverseInertiaWorld);
        }
    }

    /// <summary>
    /// 获取或设置约束的柔软度（柔顺度）。
    /// </summary>
    /// <value>
    /// 默认值为 0.001。值越大允许的角度误差越大，但稳定性更好。
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
    /// 获取或设置偏置因子，控制角度误差被修正的积极程度。
    /// </summary>
    /// <value>
    /// 默认值为 0.2。值越大误差修正越快，但可能导致不稳定。
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
    /// 获取上次求解子步累积的求解器冲量。
    /// </summary>
    public JVector Impulse => Data.AccumulatedImpulse;

    public static void IterateFixedAngle(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, FixedAngleData>(ref constraint);
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        JVector jv = JVector.Transform(body1.AngularVelocity - body2.AngularVelocity, data.Jacobian);
        JVector softness = data.AccumulatedImpulse * (data.Softness * timeStep.InverseSubstepDt);
        JVector lambda = -(Real)1.0 * JVector.Transform(jv + data.Bias + softness, data.EffectiveMass);

        data.AccumulatedImpulse += lambda;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.AngularVelocity += JVector.Transform(JVector.TransposedTransform(lambda, data.Jacobian), body1.InverseInertiaWorld);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.AngularVelocity -= JVector.Transform(JVector.TransposedTransform(lambda, data.Jacobian), body2.InverseInertiaWorld);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        if (Body1 != Body1.World.NullBody)
            drawer.DrawFrame(Body1.Position, Body1.Orientation, settings);
        if (Body2 != Body2.World.NullBody)
            drawer.DrawFrame(Body2.Position, Body2.Orientation * Data.Q0, settings);
    }
}
