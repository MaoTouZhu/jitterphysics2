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
/// 约束两个刚体绕单一轴相对旋转，消除两个角度自由度。
/// 可选择地施加角度限位。
/// </summary>
public unsafe class HingeAngle : Constraint<HingeAngle.HingeAngleData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct HingeAngleData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public Real MinAngle;
        public Real MaxAngle;
        public Real FixedAngle;

        public Real BiasFactor;
        public Real LimitBias;

        public Real LimitSoftness;
        public Real Softness;

        public JVector Axis;
        public JQuaternion Q0;

        public JVector AccumulatedImpulse;
        public JVector Bias;

        public JMatrix EffectiveMass;
        public JMatrix Jacobian;

        public JVector BilateralMass;
        public Real CouplingX, CouplingY;

        public ushort Clamp;
        public short Hemisphere;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationHingeAngle, &IterateHingeAngle);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = JVector.Zero;

    /// <summary>
    /// 使用旋转轴和角度限位初始化约束。
    /// </summary>
    /// <param name="axis">世界空间中的铰链轴，允许绕其旋转。</param>
    /// <param name="limit">定义允许旋转范围的角度限位。</param>
    /// <remarks>
    /// 将轴存储到刚体 2 的局部坐标系中，并记录初始相对朝向。
    /// 限位相等或反向时将铰链固定在其中点。
    /// 默认值：<see cref="Softness"/> = <see cref="Constraint.DefaultAngularSoftness"/>，<see cref="LimitSoftness"/> = <see cref="Constraint.DefaultAngularLimitSoftness"/>，
    /// <see cref="Bias"/> = <see cref="Constraint.DefaultAngularBias"/>，<see cref="LimitBias"/> = <see cref="Constraint.DefaultAngularLimitBias"/>。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="axis"/> 为零或包含非有限值，或
    /// <paramref name="limit"/> 中的任一值不是有限值时抛出。
    /// </exception>
    public void Initialize(JVector axis, AngularLimit limit)
    {
        VerifyNotZero();
        ArgumentCheck.NonZero(axis, nameof(axis));
        ArgumentCheck.Finite(limit.From, nameof(limit.From));
        ArgumentCheck.Finite(limit.To, nameof(limit.To));

        ref HingeAngleData data = ref Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        data.Softness = Constraint.DefaultAngularSoftness;
        data.LimitSoftness = Constraint.DefaultAngularLimitSoftness;
        data.BiasFactor = Constraint.DefaultAngularBias;
        data.LimitBias = Constraint.DefaultAngularLimitBias;

        data.MinAngle = (Real)limit.From / (Real)2.0;
        data.MaxAngle = (Real)limit.To / (Real)2.0;
        data.FixedAngle = (Real)0.25 * (Real)limit.From + (Real)0.25 * (Real)limit.To;

        JVector.NormalizeInPlace(ref axis);
        data.Axis = JVector.ConjugatedTransform(axis, body2.Orientation);

        JQuaternion q1 = body1.Orientation;
        JQuaternion q2 = body2.Orientation;

        data.Q0 = q2.Conjugate() * q1;
    }

    /// <summary>
    /// 设置铰链旋转的角度限位。
    /// </summary>
    /// <exception cref="ArgumentException">
    /// 当所赋限值中的任一值不是有限值时抛出。
    /// </exception>
    public AngularLimit Limit
    {
        set
        {
            ArgumentCheck.Finite(value.From, nameof(value.From));
            ArgumentCheck.Finite(value.To, nameof(value.To));

            ref HingeAngleData data = ref Data;
            data.MinAngle = (Real)value.From / (Real)2.0;
            data.MaxAngle = (Real)value.To / (Real)2.0;
            data.FixedAngle = (Real)0.25 * (Real)value.From + (Real)0.25 * (Real)value.To;
        }
    }

    public static void PrepareForIterationHingeAngle(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, HingeAngleData>(ref constraint);

        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JQuaternion q1 = body1.Orientation;
        JQuaternion q2 = body2.Orientation;

        JVector p0 = MathHelper.CreateOrthonormal(data.Axis);
        JVector p1 = data.Axis % p0;

        JMatrix m0 = AngularConstraintMath.CalculateJacobian(
            data.Q0 * q1.Conjugate(), q2, out JVector rotation, out short hemisphere);
        if (data.Hemisphere != 0 && data.Hemisphere != hemisphere)
            data.AccumulatedImpulse = JVector.Zero;
        data.Hemisphere = hemisphere;
        JVector error = new(JVector.Dot(p0, rotation), JVector.Dot(p1, rotation),
            JVector.Dot(data.Axis, rotation));

        data.Clamp = 0;

        data.Jacobian.UnsafeGet(0) = JVector.TransposedTransform(p0, m0);
        data.Jacobian.UnsafeGet(1) = JVector.TransposedTransform(p1, m0);
        data.Jacobian.UnsafeGet(2) = JVector.TransposedTransform(data.Axis, m0);

        data.EffectiveMass = JSymmetricMatrix.Transform(body1.InverseInertiaWorld + body2.InverseInertiaWorld,
            JMatrix.Transpose(data.Jacobian)).ToMatrix();

        data.EffectiveMass.M11 += data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass.M22 += data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass.M33 += data.LimitSoftness * timeStep.InverseSubstepDt;

        Real maxA = data.MaxAngle;
        Real minA = data.MinAngle;
        bool fullRange = minA <= -MathR.PI / 2 && maxA >= MathR.PI / 2;

        if (minA >= maxA)
        {
            data.Clamp = 3;
            error.Z = AngularConstraintMath.ShortestAngle(error.Z - data.FixedAngle, MathR.PI / 2);
        }
        else if (!fullRange && error.Z >= maxA)
        {
            data.Clamp = 1;
            error.Z -= maxA;
        }
        else if (!fullRange && error.Z <= minA)
        {
            data.Clamp = 2;
            error.Z -= minA;
        }
        else
        {
            data.AccumulatedImpulse.Z = 0;
            data.EffectiveMass.M33 = 0;
            data.EffectiveMass.M31 = data.EffectiveMass.M13 = 0;
            data.EffectiveMass.M32 = data.EffectiveMass.M23 = 0;

            data.Jacobian.M13 = data.Jacobian.M23 = data.Jacobian.M33 = 0;
        }

        if (data.Clamp == 1 || data.Clamp == 2)
        {
            data.AccumulatedImpulse.Z = data.Clamp == 1 ?
                MathR.Min(data.AccumulatedImpulse.Z, 0) : MathR.Max(data.AccumulatedImpulse.Z, 0);
            data.BilateralMass = MathHelper.InverseBilateralBlock(data.EffectiveMass);
            data.CouplingX = data.EffectiveMass.M13;
            data.CouplingY = data.EffectiveMass.M23;
        }

        data.EffectiveMass = MathHelper.InverseSymmetric(data.EffectiveMass);

        data.Bias = error * timeStep.InverseStepDt;
        data.Bias.X *= data.BiasFactor;
        data.Bias.Y *= data.BiasFactor;
        data.Bias.Z *= data.LimitBias;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.AngularVelocity += JVector.Transform(JVector.Transform(data.AccumulatedImpulse, data.Jacobian), body1.InverseInertiaWorld);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.AngularVelocity -= JVector.Transform(JVector.Transform(data.AccumulatedImpulse, data.Jacobian), body2.InverseInertiaWorld);
        }
    }

    /// <summary>
    /// 获取相对于初始姿态绕铰链轴的当前旋转角。
    /// </summary>
    /// <remarks>
    /// 当铰链轴未对齐时，该值是最近相对旋转向量在铰链轴上的投影，
    /// 与限位坐标保持一致。
    /// </remarks>
    public JAngle Angle
    {
        get
        {
            ref HingeAngleData data = ref Data;
            JQuaternion q1 = data.Body1.Data.Orientation;
            JQuaternion q2 = data.Body2.Data.Orientation;

            JQuaternion quat0 = data.Q0 * q1.Conjugate() * q2;

            return (JAngle)((Real)2.0 * JVector.Dot(data.Axis,
                AngularConstraintMath.RotationLog(quat0)));
        }
    }

    /// <summary>
    /// 获取或设置角度约束的柔软度（柔顺度）。
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
    /// 获取或设置角度限位生效时应用的柔软度（柔顺度）。
    /// </summary>
    /// <value>
    /// 默认值为 0.001。值越大允许的限位违反量越大，但稳定性更好。
    /// </value>
    public Real LimitSoftness
    {
        get => Data.LimitSoftness;
        set
        {
            DebugCheck.IsNonNegative(value, nameof(value));
            Data.LimitSoftness = value;
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
    /// 获取或设置角度限位修正的偏置因子。
    /// </summary>
    /// <value>
    /// 默认值为 0.1。值越大限位违反修正越快。
    /// </value>
    public Real LimitBias
    {
        get => Data.LimitBias;
        set
        {
            DebugCheck.IsNonNegative(value, nameof(value));
            Data.LimitBias = value;
        }
    }

    /// <summary>
    /// 获取上次求解子步累积的求解器冲量。
    /// </summary>
    public JVector Impulse => Data.AccumulatedImpulse;

    public static void IterateHingeAngle(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, HingeAngleData>(ref constraint);
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        JVector jv = JVector.TransposedTransform(body1.AngularVelocity - body2.AngularVelocity, data.Jacobian);

        JVector softness = data.AccumulatedImpulse * timeStep.InverseSubstepDt;
        softness.X *= data.Softness;
        softness.Y *= data.Softness;
        softness.Z *= data.LimitSoftness;

        JVector residual = jv + data.Bias + softness;
        JVector lambda = -(Real)1.0 * JVector.Transform(residual, data.EffectiveMass);

        JVector origAcc = data.AccumulatedImpulse;

        data.AccumulatedImpulse += lambda;

        if ((data.Clamp == 1 && data.AccumulatedImpulse.Z > 0) ||
            (data.Clamp == 2 && data.AccumulatedImpulse.Z < 0))
        {
            // 耦合块假定其限位冲量会被施加。
            // 在单边边界处，固定该冲量（包括移除任何缓存的限位冲量）
            // 后重新求解双边行。
            lambda.Z = -origAcc.Z;
            Real x = residual.X + data.CouplingX * lambda.Z;
            Real y = residual.Y + data.CouplingY * lambda.Z;
            lambda.X = -(data.BilateralMass.X * x + data.BilateralMass.Y * y);
            lambda.Y = -(data.BilateralMass.Y * x + data.BilateralMass.Z * y);
            data.AccumulatedImpulse.X = origAcc.X + lambda.X;
            data.AccumulatedImpulse.Y = origAcc.Y + lambda.Y;
            data.AccumulatedImpulse.Z = 0;
        }
        else if (data.Clamp == 0)
        {
            origAcc.Z = 0;
            data.AccumulatedImpulse.Z = 0;
        }

        lambda = data.AccumulatedImpulse - origAcc;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.AngularVelocity += JVector.Transform(JVector.Transform(lambda, data.Jacobian), body1.InverseInertiaWorld);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.AngularVelocity -= JVector.Transform(JVector.Transform(lambda, data.Jacobian), body2.InverseInertiaWorld);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        ref HingeAngleData data = ref Data;
        JQuaternion referenceOrientation = Body1.Orientation * data.Q0.Conjugate();
        JVector axis = JVector.Transform(data.Axis, referenceOrientation);
        JVector reference = JVector.Transform(-MathHelper.CreateOrthonormal(data.Axis), referenceOrientation);
        JVector origin = GetDebugDrawOrigin(settings);
        DebugDrawGeometry.AngularRange(drawer, origin, axis, reference, 2 * data.MinAngle,
            2 * data.MaxAngle, Angle.Radian, settings);
        drawer.DrawArrow(origin, JVector.Transform(data.Axis, Body2.Orientation) * (settings.GuideSize * (Real)0.4),
            settings.CurrentColor, settings);
    }
}
