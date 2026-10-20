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
/// 使用受限的力将刚体 2 上的锚点驱向附着于刚体 1 的目标位置。
/// 控制三个平动自由度；相对朝向保持自由。
/// </summary>
public unsafe class LinearServo : Constraint<LinearServo.LinearServoData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct LinearServoData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAnchor1;
        public JVector LocalAnchor2;
        public JVector TargetPosition;
        public Real MaximumForce;
        public Real Softness;
        public Real BiasFactor;
        public JVector R1, R2;
        public JVector AccumulatedImpulse;
        public JVector Bias;
        public JMatrix EffectiveMass;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationLinearServo, &IterateLinearServo);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = JVector.Zero;

    /// <summary>将两个刚体的锚点初始化到同一世界空间点。</summary>
    public void Initialize(JVector anchor) => Initialize(anchor, anchor);

    /// <summary>初始化两个独立的世界空间锚点，伺服将驱动它们趋向重合。</summary>
    /// <remarks>目标位置和最大力默认为零。锚点可以起始于分离状态。</remarks>
    public void Initialize(JVector anchor1, JVector anchor2)
    {
        VerifyNotZero();
        ArgumentCheck.Finite(anchor1, nameof(anchor1));
        ArgumentCheck.Finite(anchor2, nameof(anchor2));
        ref var data = ref Data;
        data.LocalAnchor1 = JVector.ConjugatedTransform(anchor1 - Body1.Position, Body1.Orientation);
        data.LocalAnchor2 = JVector.ConjugatedTransform(anchor2 - Body2.Position, Body2.Orientation);
        data.TargetPosition = JVector.Zero;
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

    /// <summary>
    /// 获取或设置从锚点 1 指向锚点 2 的期望偏移量，以刚体 1 的局部坐标系表示。
    /// 零值驱动两个锚点趋向重合。修改它不会移动任一刚体。
    /// </summary>
    public JVector TargetPosition
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

    /// <summary>获取上一个已求解子步中施加到刚体 2 的世界空间冲量。</summary>
    public JVector Impulse => Data.AccumulatedImpulse;

    public static void PrepareForIterationLinearServo(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, LinearServoData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;
        data.R1 = JVector.Transform(data.LocalAnchor1 + data.TargetPosition, body1.Orientation);
        data.R2 = JVector.Transform(data.LocalAnchor2, body2.Orientation);
        JMatrix cr1 = JMatrix.CreateCrossProduct(data.R1), cr2 = JMatrix.CreateCrossProduct(data.R2);
        data.EffectiveMass = (JSymmetricMatrix.CreateScale(body1.InverseMassVector) +
                              JSymmetricMatrix.Transform(body1.InverseInertiaWorld, cr1) +
                              JSymmetricMatrix.CreateScale(body2.InverseMassVector) +
                              JSymmetricMatrix.Transform(body2.InverseInertiaWorld, cr2)).ToMatrix();
        Real softness = data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass.M11 += softness;
        data.EffectiveMass.M22 += softness;
        data.EffectiveMass.M33 += softness;
        data.EffectiveMass = MathHelper.InverseSymmetric(data.EffectiveMass);
        data.Bias = (body2.Position + data.R2 - body1.Position - data.R1) *
                    (data.BiasFactor * timeStep.InverseStepDt);
        ServoMath.ClampMagnitude(ref data.AccumulatedImpulse,
            ServoMath.MaximumImpulse(data.MaximumForce, timeStep.SubstepDt));
        ApplyImpulse(ref data, data.AccumulatedImpulse);
    }

    public static void IterateLinearServo(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, LinearServoData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;
        JVector jv = -body1.Velocity - (body1.AngularVelocity % data.R1) +
                     body2.Velocity + (body2.AngularVelocity % data.R2);
        JVector softness = data.AccumulatedImpulse * (data.Softness * timeStep.InverseSubstepDt);
        JVector lambda = -JVector.Transform(jv + data.Bias + softness, data.EffectiveMass);
        JVector previous = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;
        ServoMath.ClampMagnitude(ref data.AccumulatedImpulse,
            ServoMath.MaximumImpulse(data.MaximumForce, timeStep.SubstepDt));
        ApplyImpulse(ref data, data.AccumulatedImpulse - previous);
    }

    private static void ApplyImpulse(ref LinearServoData data, in JVector impulse)
    {
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;
        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity -= JVector.Multiply(impulse, body1.InverseMassVector);
            body1.AngularVelocity -= JVector.Transform(data.R1 % impulse, body1.InverseInertiaWorld);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(impulse, body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(data.R2 % impulse, body2.InverseInertiaWorld);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        JVector target = Anchor1 + JVector.Transform(TargetPosition, Body1.Orientation);
        JVector current = Anchor2;
        drawer.DrawSegment(target, current, settings.CurrentColor);
        drawer.DrawPoint(target, settings.PointRadius, settings.ReferenceColor);
        drawer.DrawPoint(current, settings.PointRadius, settings.CurrentColor);
        if (Body2 != Body2.World.NullBody) drawer.DrawSegment(Body2.Position, current, settings.CurrentColor);
    }
}
