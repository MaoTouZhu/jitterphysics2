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
/// 将一个刚体参考系中的固定点约束到另一个刚体参考系中固定的平面上。若启用限位，
/// 该约束移除一个平动自由度。
/// </summary>
public unsafe class PointOnPlane : Constraint<PointOnPlane.SliderData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct SliderData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAxis;

        public JVector LocalAnchor1;
        public JVector LocalAnchor2;

        public Real BiasFactor;
        public Real Softness;

        public Real EffectiveMass;
        public Real AccumulatedImpulse;
        public Real Bias;

        public Real Min;
        public Real Max;

        // 0：未启用，1：上限，2：下限，3：双边固定限位。
        public ushort Clamp;

        public MemoryHelper.MemBlock12Real J0;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationPointOnPlane, &IteratePointOnPlane);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = (Real)0.0;

    /// <inheritdoc cref="Initialize(JVector, JVector, JVector, LinearLimit)"/>
    public void Initialize(JVector axis, JVector anchor1, JVector anchor2)
    {
        Initialize(axis, anchor1, anchor2, LinearLimit.Fixed);
    }

    /// <summary>
    /// 使用世界空间参数初始化约束。
    /// </summary>
    /// <param name="axis">世界空间中的平面法线，固定于刚体 1 的参考系。</param>
    /// <param name="anchor1">刚体 1 上的锚点，在世界空间中定义平面原点。</param>
    /// <param name="anchor2">刚体 2 上的锚点，在世界空间中约束到该平面。</param>
    /// <param name="limit">到平面的距离限位。</param>
    /// <remarks>
    /// 根据当前刚体位姿计算局部锚点和轴。
    /// 相等或反向的距离限位会将点固定在其中点。这包括
    /// <see cref="LinearLimit.Fixed"/>，并允许两个方向上的修正冲量。
    /// 默认值：<see cref="Bias"/> = <see cref="Constraint.DefaultLinearBias"/>，<see cref="Softness"/> = <see cref="Constraint.DefaultLinearSoftness"/>。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="axis"/> 为零或包含非有限值、当
    /// <paramref name="anchor1"/> 或 <paramref name="anchor2"/> 包含非有限值，
    /// 或当 <paramref name="limit"/> 中任一值为 NaN 时抛出。
    /// </exception>
    public void Initialize(JVector axis, JVector anchor1, JVector anchor2, LinearLimit limit)
    {
        VerifyNotZero();
        ArgumentCheck.NonZero(axis, nameof(axis));
        ArgumentCheck.Finite(anchor1, nameof(anchor1));
        ArgumentCheck.Finite(anchor2, nameof(anchor2));
        ArgumentCheck.NotNaN(limit.From, nameof(limit.From));
        ArgumentCheck.NotNaN(limit.To, nameof(limit.To));

        ref SliderData data = ref Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.NormalizeInPlace(ref axis);

        JVector.Subtract(anchor1, body1.Position, out data.LocalAnchor1);
        JVector.Subtract(anchor2, body2.Position, out data.LocalAnchor2);

        JVector.ConjugatedTransform(data.LocalAnchor1, body1.Orientation, out data.LocalAnchor1);
        JVector.ConjugatedTransform(data.LocalAnchor2, body2.Orientation, out data.LocalAnchor2);

        JVector.ConjugatedTransform(axis, body1.Orientation, out data.LocalAxis);

        data.BiasFactor = Constraint.DefaultLinearBias;
        data.Softness = Constraint.DefaultLinearSoftness;

        (data.Min, data.Max) = limit;
    }

    public static void PrepareForIterationPointOnPlane(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, SliderData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Transform(data.LocalAxis, body1.Orientation, out JVector axis);

        JVector.Transform(data.LocalAnchor1, body1.Orientation, out JVector r1);
        JVector.Transform(data.LocalAnchor2, body2.Orientation, out JVector r2);

        JVector.Add(body1.Position, r1, out JVector p1);
        JVector.Add(body2.Position, r2, out JVector p2);

        data.Clamp = 0;

        JVector u = p2 - p1;

        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 4);

        jacobian[0] = -axis;
        jacobian[1] = -((r1 + u) % axis);
        jacobian[2] = axis;
        jacobian[3] = r2 % axis;

        Real error = JVector.Dot(u, axis);

        data.EffectiveMass = (Real)1.0;

        if (data.Min >= data.Max)
        {
            error -= (Real)0.5 * data.Min + (Real)0.5 * data.Max;
            data.Clamp = 3;
        }
        else if (error >= data.Max)
        {
            error -= data.Max;
            data.Clamp = 1;
        }
        else if (error <= data.Min)
        {
            error -= data.Min;
            data.Clamp = 2;
        }
        else
        {
            data.AccumulatedImpulse = 0;
            return;
        }

        data.EffectiveMass = JVector.Multiply(jacobian[0], body1.InverseMassVector) * jacobian[0] +
                             JVector.Multiply(jacobian[2], body2.InverseMassVector) * jacobian[2] +
                             JVector.Transform(jacobian[1], body1.InverseInertiaWorld) * jacobian[1] +
                             JVector.Transform(jacobian[3], body2.InverseInertiaWorld) * jacobian[3];

        data.EffectiveMass += (data.Softness * timeStep.InverseSubstepDt);
        data.EffectiveMass = data.EffectiveMass > 0 ? (Real)1.0 / data.EffectiveMass : 0;
        if (data.EffectiveMass == 0) data.AccumulatedImpulse = 0;

        data.Bias = error * data.BiasFactor * timeStep.InverseStepDt;

        Real acc = data.AccumulatedImpulse;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity += JVector.Multiply(jacobian[0] * acc, body1.InverseMassVector);
            body1.AngularVelocity += JVector.Transform(jacobian[1] * acc, body1.InverseInertiaWorld);
        }

        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(jacobian[2] * acc, body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(jacobian[3] * acc, body2.InverseInertiaWorld);
        }
    }

    /// <summary>
    /// 获取或设置约束的柔软度（柔顺性）。
    /// </summary>
    /// <value>
    /// 默认值为 0.00001。较高的值允许更大的位置误差，但可提升稳定性。
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
    /// 获取或设置偏置因子，控制位置误差的修正强度。
    /// </summary>
    /// <value>
    /// 默认值为 <see cref="Constraint.DefaultLinearBias"/>。较高的值修正误差更快，但可能导致不稳定。
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
    /// 获取上一次已求解子步中累积的求解器冲量。
    /// </summary>
    public Real Impulse => Data.AccumulatedImpulse;

    public static void IteratePointOnPlane(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, SliderData>(ref constraint);
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        if (data.Clamp == 0) return;

        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 4);

        Real jv = jacobian[0] * body1.Velocity + jacobian[1] * body1.AngularVelocity + jacobian[2] * body2.Velocity +
                   jacobian[3] * body2.AngularVelocity;

        Real softness = data.AccumulatedImpulse * data.Softness * timeStep.InverseSubstepDt;

        Real lambda = -(Real)1.0 * (jv + data.Bias + softness) * data.EffectiveMass;

        Real origAcc = data.AccumulatedImpulse;

        data.AccumulatedImpulse += lambda;

        if (data.Clamp == 1)
        {
            data.AccumulatedImpulse = MathR.Min(data.AccumulatedImpulse, (Real)0.0);
        }
        else if (data.Clamp == 2)
        {
            data.AccumulatedImpulse = MathR.Max(data.AccumulatedImpulse, (Real)0.0);
        }

        lambda = data.AccumulatedImpulse - origAcc;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity += JVector.Multiply(jacobian[0] * lambda, body1.InverseMassVector);
            body1.AngularVelocity += JVector.Transform(jacobian[1] * lambda, body1.InverseInertiaWorld);
        }

        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(jacobian[2] * lambda, body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(jacobian[3] * lambda, body2.InverseInertiaWorld);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        ref SliderData data = ref Data;
        JVector first = Body1.Position + JVector.Transform(data.LocalAnchor1, Body1.Orientation);
        JVector second = Body2.Position + JVector.Transform(data.LocalAnchor2, Body2.Orientation);
        JVector normal = JVector.Transform(data.LocalAxis, Body1.Orientation);
        JVector reference = JVector.Transform(MathHelper.CreateOrthonormal(data.LocalAxis), Body1.Orientation);
        Real minimum = data.Min, maximum = data.Max;
        if (minimum >= maximum)
        {
            Real offset = minimum * (Real)0.5 + maximum * (Real)0.5;
            if (Real.IsFinite(offset))
                drawer.DrawPlane(first + normal * offset, normal, reference, settings.GuideSize, settings.UpperLimitColor);
        }
        else
        {
            if (Real.IsFinite(minimum))
                drawer.DrawPlane(first + normal * minimum, normal, reference, settings.GuideSize, settings.LowerLimitColor);
            if (Real.IsFinite(maximum))
                drawer.DrawPlane(first + normal * maximum, normal, reference, settings.GuideSize, settings.UpperLimitColor);
            if (!Real.IsFinite(minimum) && !Real.IsFinite(maximum))
                drawer.DrawPlane(first, normal, reference, settings.GuideSize, settings.ReferenceColor);
        }
        drawer.DrawArrow(first, normal * (settings.GuideSize * (Real)0.4), settings.ReferenceColor, settings);
        if (Body2 != Body2.World.NullBody) drawer.DrawSegment(Body2.Position, second, settings.CurrentColor);
        Real distance = JVector.Dot(second - first, normal);
        Real target = minimum >= maximum ? minimum * (Real)0.5 + maximum * (Real)0.5
            : Real.Clamp(distance, minimum, maximum);
        if (Real.IsFinite(target) && target != distance)
            drawer.DrawSegment(second, second + normal * (target - distance), settings.CurrentColor);
        drawer.DrawPoint(first, settings.PointRadius, settings.ReferenceColor);
        drawer.DrawPoint(second, settings.PointRadius, settings.CurrentColor);
    }
}
