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
/// 实现球窝关节，将每个刚体上的一个点锚定在一起，
/// 从而移除三个平移自由度。
/// </summary>
public unsafe class BallSocket : Constraint<BallSocket.BallSocketData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct BallSocketData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAnchor1;
        public JVector LocalAnchor2;

        public JVector U;
        public JVector R1;
        public JVector R2;

        public Real BiasFactor;
        public Real Softness;

        public JMatrix EffectiveMass;
        public JVector AccumulatedImpulse;
        public JVector Bias;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationBallSocket, &IterateBallSocket);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = JVector.Zero;

    /// <summary>
    /// 使用世界空间中的锚点初始化约束。
    /// </summary>
    /// <param name="anchor">世界空间中的锚点，由两个刚体共享。</param>
    /// <remarks>
    /// 根据各刚体当前的姿态计算其局部锚点。
    /// 默认值：<see cref="Bias"/> = <see cref="Constraint.DefaultLinearBias"/>，<see cref="Softness"/> = <see cref="Constraint.DefaultLinearSoftness"/>。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="anchor"/> 包含非有限值时抛出。
    /// </exception>
    public void Initialize(JVector anchor)
    {
        VerifyNotZero();
        ArgumentCheck.Finite(anchor, nameof(anchor));

        ref BallSocketData data = ref Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Subtract(anchor, body1.Position, out data.LocalAnchor1);
        JVector.Subtract(anchor, body2.Position, out data.LocalAnchor2);

        JVector.ConjugatedTransform(data.LocalAnchor1, body1.Orientation, out data.LocalAnchor1);
        JVector.ConjugatedTransform(data.LocalAnchor2, body2.Orientation, out data.LocalAnchor2);

        data.BiasFactor = Constraint.DefaultLinearBias;
        data.Softness = Constraint.DefaultLinearSoftness;
    }

    /// <summary>
    /// 获取或设置第一个刚体上的锚点，位于世界空间。该锚点
    /// 固定于第一个刚体的局部参考系中。
    /// </summary>
    [ReferenceFrame(ReferenceFrame.World)]
    public JVector Anchor1
    {
        set
        {
            DebugCheck.IsFinite(value, nameof(value));

            ref BallSocketData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;
            JVector.Subtract(value, body1.Position, out data.LocalAnchor1);
            JVector.ConjugatedTransform(data.LocalAnchor1, body1.Orientation, out data.LocalAnchor1);
        }
        get
        {
            ref BallSocketData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;
            JVector.Transform(data.LocalAnchor1, body1.Orientation, out JVector result);
            JVector.Add(result, body1.Position, out result);
            return result;
        }
    }

    /// <summary>
    /// 获取或设置第二个刚体上的锚点，位于世界空间。该锚点
    /// 固定于第二个刚体的局部参考系中。
    /// </summary>
    [ReferenceFrame(ReferenceFrame.World)]
    public JVector Anchor2
    {
        set
        {
            DebugCheck.IsFinite(value, nameof(value));

            ref BallSocketData data = ref Data;
            ref RigidBodyData body2 = ref data.Body2.Data;
            JVector.Subtract(value, body2.Position, out data.LocalAnchor2);
            JVector.ConjugatedTransform(data.LocalAnchor2, body2.Orientation, out data.LocalAnchor2);
        }
        get
        {
            ref BallSocketData data = ref Data;
            ref RigidBodyData body2 = ref data.Body2.Data;
            JVector.Transform(data.LocalAnchor2, body2.Orientation, out JVector result);
            JVector.Add(result, body2.Position, out result);
            return result;
        }
    }

    public static void PrepareForIterationBallSocket(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, BallSocketData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Transform(data.LocalAnchor1, body1.Orientation, out data.R1);
        JVector.Transform(data.LocalAnchor2, body2.Orientation, out data.R2);

        JVector.Add(body1.Position, data.R1, out JVector p1);
        JVector.Add(body2.Position, data.R2, out JVector p2);

        JMatrix cr1 = JMatrix.CreateCrossProduct(data.R1);
        JMatrix cr2 = JMatrix.CreateCrossProduct(data.R2);

        data.EffectiveMass = (JSymmetricMatrix.CreateScale(body1.InverseMassVector) +
                              JSymmetricMatrix.Transform(body1.InverseInertiaWorld, cr1) +
                              JSymmetricMatrix.CreateScale(body2.InverseMassVector) +
                              JSymmetricMatrix.Transform(body2.InverseInertiaWorld, cr2)).ToMatrix();

        Real softness = data.Softness * timeStep.InverseSubstepDt;

        data.EffectiveMass.M11 += softness;
        data.EffectiveMass.M22 += softness;
        data.EffectiveMass.M33 += softness;

        data.EffectiveMass = MathHelper.InverseSymmetric(data.EffectiveMass);

        data.Bias = (p2 - p1) * data.BiasFactor * timeStep.InverseStepDt;

        JVector acc = data.AccumulatedImpulse;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity -= JVector.Multiply(acc, body1.InverseMassVector);
            body1.AngularVelocity -= JVector.Transform(JVector.Transform(acc, cr1), body1.InverseInertiaWorld);
        }

        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(acc, body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(JVector.Transform(acc, cr2), body2.InverseInertiaWorld);
        }
    }

    /// <summary>
    /// 获取或设置约束的柔软度（柔顺度）。
    /// </summary>
    /// <value>
    /// 默认值为 <see cref="Constraint.DefaultLinearSoftness"/>。值越高允许的位置误差越大，但稳定性更好。
    /// 求解时按时间步的倒数进行缩放。
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
    /// 获取或设置偏置因子，控制位置误差被修正的激进程度。
    /// </summary>
    /// <value>
    /// 默认值为 0.2。值越高误差修正越快，但可能引起不稳定。
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
    /// 获取上一次求解的子步中累积的求解器冲量。
    /// </summary>
    public JVector Impulse => Data.AccumulatedImpulse;

    public static void IterateBallSocket(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, BallSocketData>(ref constraint);
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        JMatrix cr1 = JMatrix.CreateCrossProduct(data.R1);
        JMatrix cr2 = JMatrix.CreateCrossProduct(data.R2);

        JVector softnessVector = data.AccumulatedImpulse * data.Softness * timeStep.InverseSubstepDt;

        JVector jv = -body1.Velocity + JVector.Transform(body1.AngularVelocity, cr1) + body2.Velocity -
                     JVector.Transform(body2.AngularVelocity, cr2);

        JVector lambda = -(Real)1.0 * JVector.Transform(jv + data.Bias + softnessVector, data.EffectiveMass);

        data.AccumulatedImpulse += lambda;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity -= JVector.Multiply(lambda, body1.InverseMassVector);
            body1.AngularVelocity -= JVector.Transform(JVector.Transform(lambda, cr1), body1.InverseInertiaWorld);
        }

        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(lambda, body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(JVector.Transform(lambda, cr2), body2.InverseInertiaWorld);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        JVector first = Anchor1, second = Anchor2;
        if (Body1 != Body1.World.NullBody) drawer.DrawSegment(Body1.Position, first, settings.ReferenceColor);
        if (Body2 != Body2.World.NullBody) drawer.DrawSegment(Body2.Position, second, settings.CurrentColor);
        drawer.DrawSegment(first, second, settings.CurrentColor);
        drawer.DrawPoint(first, settings.PointRadius, settings.ReferenceColor);
        drawer.DrawPoint(second, settings.PointRadius, settings.CurrentColor);
    }
}
