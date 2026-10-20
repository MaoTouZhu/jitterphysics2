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
/// 使用受限的力驱动两个锚点沿附着于刚体 1 的轴线的有符号间距。
/// 控制一个平动自由度；垂直方向的移动和旋转保持自由。
/// </summary>
public unsafe class LinearAxisServo : Constraint<LinearAxisServo.LinearAxisServoData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct LinearAxisServoData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAxis;
        public JVector LocalAnchor1, LocalAnchor2;
        public Real TargetPosition;
        public Real MaximumForce;
        public Real Softness;
        public Real BiasFactor;
        public Real EffectiveMass;
        public Real AccumulatedImpulse;
        public Real Bias;
        public MemoryHelper.MemBlock12Real J0;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationLinearAxisServo, &IterateLinearAxisServo);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = 0;

    /// <summary>初始化世界空间锚点以及附着于刚体 1 的轴线。</summary>
    /// <param name="axis">伺服作用的轴线方向，内部会归一化。</param>
    /// <param name="anchor1">附着于刚体 1 的原点。</param>
    /// <param name="anchor2">附着于刚体 2 的锚点。</param>
    /// <param name="targetPosition">沿轴线的期望有符号间距。</param>
    /// <remarks>最大力默认为零。当前间距可以与目标不同。</remarks>
    public void Initialize(JVector axis, JVector anchor1, JVector anchor2, Real targetPosition = 0)
    {
        VerifyNotZero();
        ArgumentCheck.NonZero(axis, nameof(axis));
        ArgumentCheck.Finite(anchor1, nameof(anchor1));
        ArgumentCheck.Finite(anchor2, nameof(anchor2));
        ArgumentCheck.Finite(targetPosition, nameof(targetPosition));
        JVector.NormalizeInPlace(ref axis);
        ref var data = ref Data;
        data.LocalAxis = JVector.ConjugatedTransform(axis, Body1.Orientation);
        data.LocalAnchor1 = JVector.ConjugatedTransform(anchor1 - Body1.Position, Body1.Orientation);
        data.LocalAnchor2 = JVector.ConjugatedTransform(anchor2 - Body2.Position, Body2.Orientation);
        data.TargetPosition = targetPosition;
        data.MaximumForce = 0;
        data.Softness = Constraint.DefaultLinearSoftness;
        data.BiasFactor = Constraint.DefaultLinearBias;
        ResetWarmStart();
    }

    /// <summary>获取或设置附着于刚体 1 的世界空间锚点。</summary>
    [ReferenceFrame(ReferenceFrame.World)]
    public JVector Anchor1
    {
        get => Body1.Position + JVector.Transform(Data.LocalAnchor1, Body1.Orientation);
        set
        {
            ArgumentCheck.Finite(value, nameof(value));
            Data.LocalAnchor1 = JVector.ConjugatedTransform(value - Body1.Position, Body1.Orientation);
            ResetWarmStart();
            ServoMath.WakeBodies(this);
        }
    }

    /// <summary>获取或设置附着于刚体 2 的世界空间锚点。</summary>
    [ReferenceFrame(ReferenceFrame.World)]
    public JVector Anchor2
    {
        get => Body2.Position + JVector.Transform(Data.LocalAnchor2, Body2.Orientation);
        set
        {
            ArgumentCheck.Finite(value, nameof(value));
            Data.LocalAnchor2 = JVector.ConjugatedTransform(value - Body2.Position, Body2.Orientation);
            ResetWarmStart();
            ServoMath.WakeBodies(this);
        }
    }

    /// <summary>获取或设置附着于刚体 1 的世界空间方向，内部会归一化。</summary>
    [ReferenceFrame(ReferenceFrame.World)]
    public JVector Axis
    {
        get => JVector.Transform(Data.LocalAxis, Body1.Orientation);
        set
        {
            ArgumentCheck.NonZero(value, nameof(value));
            JVector.NormalizeInPlace(ref value);
            Data.LocalAxis = JVector.ConjugatedTransform(value, Body1.Orientation);
            ResetWarmStart();
            ServoMath.WakeBodies(this);
        }
    }

    /// <summary>获取两个锚点沿轴线的当前有符号间距。</summary>
    public Real Position => (Anchor2 - Anchor1) * Axis;

    /// <summary>获取或设置沿轴线的期望有符号间距。默认为零。</summary>
    public Real TargetPosition
    {
        get => Data.TargetPosition;
        set
        {
            Data.TargetPosition = ArgumentCheck.Finite(value, nameof(value));
            ResetWarmStart();
            ServoMath.WakeBodies(this);
        }
    }

    /// <summary>获取或设置伺服可施加的最大总力。</summary>
    /// <value>默认为零。正无穷表示不限制力。</value>
    public Real MaximumForce
    {
        get => Data.MaximumForce;
        set
        {
            Data.MaximumForce = ServoMath.CheckMaximum(value);
            ServoMath.WakeBodies(this);
        }
    }

    /// <summary>获取或设置伺服的柔软度（柔顺度）。</summary>
    /// <value>默认为 <see cref="Constraint.DefaultLinearSoftness"/>。</value>
    public Real Softness
    {
        get => Data.Softness;
        set => Data.Softness = ArgumentCheck.NonNegative(value, nameof(value));
    }

    /// <summary>获取或设置伺服修正其位置误差的强度。</summary>
    /// <value>默认为 <see cref="Constraint.DefaultLinearBias"/>。</value>
    public Real Bias
    {
        get => Data.BiasFactor;
        set => Data.BiasFactor = ArgumentCheck.NonNegative(value, nameof(value));
    }

    /// <summary>获取上一个已求解子步中沿轴线施加到刚体 2 的有符号冲量。</summary>
    public Real Impulse => Data.AccumulatedImpulse;

    public static void PrepareForIterationLinearAxisServo(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, LinearAxisServoData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;
        JVector axis = JVector.Transform(data.LocalAxis, body1.Orientation);
        JVector r1 = JVector.Transform(data.LocalAnchor1, body1.Orientation);
        JVector r2 = JVector.Transform(data.LocalAnchor2, body2.Orientation);
        JVector separation = body2.Position + r2 - body1.Position - r1;
        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 4);
        jacobian[0] = -axis;
        jacobian[1] = -((r1 + separation) % axis);
        jacobian[2] = axis;
        jacobian[3] = r2 % axis;
        Real mass = JVector.Multiply(jacobian[0], body1.InverseMassVector) * jacobian[0] +
                    JVector.Multiply(jacobian[2], body2.InverseMassVector) * jacobian[2] +
                    JVector.Transform(jacobian[1], body1.InverseInertiaWorld) * jacobian[1] +
                    JVector.Transform(jacobian[3], body2.InverseInertiaWorld) * jacobian[3] +
                    data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass = mass > 0 ? (Real)1 / mass : 0;
        if (data.EffectiveMass == 0) data.AccumulatedImpulse = 0;
        data.Bias = (separation * axis - data.TargetPosition) * data.BiasFactor * timeStep.InverseStepDt;
        Real maximum = ServoMath.MaximumImpulse(data.MaximumForce, timeStep.SubstepDt);
        data.AccumulatedImpulse = Math.Clamp(data.AccumulatedImpulse, -maximum, maximum);
        ApplyImpulse(ref data, data.AccumulatedImpulse);
    }

    public static void IterateLinearAxisServo(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, LinearAxisServoData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;
        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 4);
        Real jv = jacobian[0] * body1.Velocity + jacobian[1] * body1.AngularVelocity +
                  jacobian[2] * body2.Velocity + jacobian[3] * body2.AngularVelocity;
        Real lambda = -(jv + data.Bias + data.AccumulatedImpulse * data.Softness * timeStep.InverseSubstepDt) * data.EffectiveMass;
        Real previous = data.AccumulatedImpulse;
        Real maximum = ServoMath.MaximumImpulse(data.MaximumForce, timeStep.SubstepDt);
        data.AccumulatedImpulse = Math.Clamp(previous + lambda, -maximum, maximum);
        ApplyImpulse(ref data, data.AccumulatedImpulse - previous);
    }

    private static void ApplyImpulse(ref LinearAxisServoData data, Real impulse)
    {
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;
        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 4);
        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity += JVector.Multiply(jacobian[0] * impulse, body1.InverseMassVector);
            body1.AngularVelocity += JVector.Transform(jacobian[1] * impulse, body1.InverseInertiaWorld);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(jacobian[2] * impulse, body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(jacobian[3] * impulse, body2.InverseInertiaWorld);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        JVector origin = Anchor1, axis = Axis;
        JVector target = origin + axis * TargetPosition;
        JVector current = origin + axis * Position;
        drawer.DrawSegment(origin - axis * settings.GuideSize, origin + axis * settings.GuideSize, settings.ReferenceColor);
        drawer.DrawSegment(target, current, settings.CurrentColor);
        drawer.DrawPoint(target, settings.PointRadius, settings.ReferenceColor);
        drawer.DrawPoint(current, settings.PointRadius, settings.CurrentColor);
    }
}
