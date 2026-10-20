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
/// 将一个刚体参考系中的固定点约束到另一个刚体参考系中固定的直线上。该约束移除两个
/// 平动自由度；若启用限位则移除三个。
/// </summary>
public unsafe class PointOnLine : Constraint<PointOnLine.PointOnLineData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct PointOnLineData
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
        public Real LimitBias;
        public Real Softness;
        public Real LimitSoftness;

        public JMatrix EffectiveMass;
        public JVector BilateralMass;
        public Real CouplingX;
        public Real CouplingY;
        public JVector AccumulatedImpulse;
        public JVector Bias;

        public Real Min;
        public Real Max;

        public ushort Clamp;

    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationPointOnLine, &IteratePointOnLine);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = JVector.Zero;

    /// <inheritdoc cref="Initialize(JVector, JVector, JVector, LinearLimit)"/>
    public void Initialize(JVector axis, JVector anchor1, JVector anchor2)
    {
        Initialize(axis, anchor1, anchor2, LinearLimit.Fixed);
    }

    /// <summary>
    /// 使用世界空间参数初始化约束。
    /// </summary>
    /// <param name="axis">世界空间中的直线轴，固定于刚体 1 的参考系。</param>
    /// <param name="anchor1">刚体 1 上的锚点，在世界空间中定义直线原点。</param>
    /// <param name="anchor2">刚体 2 上的锚点，在世界空间中约束到该直线。</param>
    /// <param name="limit">沿轴的距离限位。</param>
    /// <remarks>
    /// 根据当前刚体位姿计算局部锚点和轴。
    /// 相等或反向的距离限位会将点固定在其中点。
    /// 默认值：<see cref="Bias"/> = <see cref="Constraint.DefaultLinearBias"/>，<see cref="Softness"/> = <see cref="Constraint.DefaultLinearSoftness"/>，
    /// <see cref="LimitSoftness"/> = <see cref="Constraint.DefaultLinearLimitSoftness"/>，<see cref="LimitBias"/> = <see cref="Constraint.DefaultLinearLimitBias"/>。
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

        ref PointOnLineData data = ref Data;
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
        data.LimitSoftness = Constraint.DefaultLinearLimitSoftness;
        data.LimitBias = Constraint.DefaultLinearLimitBias;

        (data.Min, data.Max) = limit;
    }

    /// <summary>
    /// 获取锚点沿轴方向距离直线原点的当前距离。
    /// </summary>
    public Real Distance
    {
        get
        {
            ref PointOnLineData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;
            ref RigidBodyData body2 = ref data.Body2.Data;

            JVector.Transform(data.LocalAnchor1, body1.Orientation, out JVector r1);
            JVector.Transform(data.LocalAnchor2, body2.Orientation, out JVector r2);

            JVector.Add(body1.Position, r1, out JVector p1);
            JVector.Add(body2.Position, r2, out JVector p2);

            JVector u = p2 - p1;

            JVector.Transform(data.LocalAxis, body1.Orientation, out JVector aw);

            return JVector.Dot(u, aw);
        }
    }

    [SkipLocalsInit]
    public static void PrepareForIterationPointOnLine(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, PointOnLineData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Transform(data.LocalAxis, body1.Orientation, out JVector aw);

        JVector n1 = MathHelper.CreateOrthonormal(data.LocalAxis);
        JVector.Transform(n1, body1.Orientation, out n1);

        JVector n2 = aw % n1;

        JVector.Transform(data.LocalAnchor1, body1.Orientation, out JVector r1);
        JVector.Transform(data.LocalAnchor2, body2.Orientation, out JVector r2);

        JVector.Add(body1.Position, r1, out JVector p1);
        JVector.Add(body2.Position, r2, out JVector p2);

        data.Clamp = 0;

        JVector u = p2 - p1;

        Span<JVector> jacobian = stackalloc JVector[12];

        jacobian[0] = -n1;
        jacobian[1] = -((r1 + u) % n1);
        jacobian[2] = n1;
        jacobian[3] = r2 % n1;

        jacobian[4] = -n2;
        jacobian[5] = -((r1 + u) % n2);
        jacobian[6] = n2;
        jacobian[7] = r2 % n2;

        jacobian[8] = -aw;
        jacobian[9] = -((r1 + u) % aw);
        jacobian[10] = aw;
        jacobian[11] = r2 % aw;

        var error = new JVector(JVector.Dot(u, n1), JVector.Dot(u, n2), JVector.Dot(u, aw));

        data.EffectiveMass = JMatrix.Identity;

        data.EffectiveMass.M11 = JVector.Multiply(jacobian[0], body1.InverseMassVector) * jacobian[0] +
                                 JVector.Multiply(jacobian[2], body2.InverseMassVector) * jacobian[2] +
                                 JVector.Transform(jacobian[1], body1.InverseInertiaWorld) * jacobian[1] +
                                 JVector.Transform(jacobian[3], body2.InverseInertiaWorld) * jacobian[3];

        data.EffectiveMass.M22 = JVector.Multiply(jacobian[4], body1.InverseMassVector) * jacobian[4] +
                                 JVector.Multiply(jacobian[6], body2.InverseMassVector) * jacobian[6] +
                                 JVector.Transform(jacobian[5], body1.InverseInertiaWorld) * jacobian[5] +
                                 JVector.Transform(jacobian[7], body2.InverseInertiaWorld) * jacobian[7];

        data.EffectiveMass.M12 = JVector.Transform(jacobian[1], body1.InverseInertiaWorld) * jacobian[5] +
                                 JVector.Transform(jacobian[3], body2.InverseInertiaWorld) * jacobian[7];

        data.EffectiveMass.M21 = JVector.Transform(jacobian[5], body1.InverseInertiaWorld) * jacobian[1] +
                                 JVector.Transform(jacobian[7], body2.InverseInertiaWorld) * jacobian[3];

        Real coupling12 = JVector.Multiply(jacobian[0], body1.InverseMassVector) * jacobian[4] +
                          JVector.Multiply(jacobian[2], body2.InverseMassVector) * jacobian[6];
        data.EffectiveMass.M12 += coupling12;
        data.EffectiveMass.M21 += coupling12;

        if (data.Min >= data.Max)
        {
            error.Z -= (Real)0.5 * data.Min + (Real)0.5 * data.Max;
            data.Clamp = 3;
        }
        else if (error.Z >= data.Max)
        {
            error.Z -= data.Max;
            data.Clamp = 1;
        }
        else if (error.Z <= data.Min)
        {
            error.Z -= data.Min;
            data.Clamp = 2;
        }
        else
        {
            data.AccumulatedImpulse.Z = 0;
        }

        if (data.Clamp != 0)
        {
            data.EffectiveMass.M33 = JVector.Multiply(jacobian[8], body1.InverseMassVector) * jacobian[8] +
                                     JVector.Multiply(jacobian[10], body2.InverseMassVector) * jacobian[10] +
                                     JVector.Transform(jacobian[9], body1.InverseInertiaWorld) * jacobian[9] +
                                     JVector.Transform(jacobian[11], body2.InverseInertiaWorld) * jacobian[11];

            data.EffectiveMass.M13 = JVector.Transform(jacobian[1], body1.InverseInertiaWorld) * jacobian[9] +
                                     JVector.Transform(jacobian[3], body2.InverseInertiaWorld) * jacobian[11];

            data.EffectiveMass.M31 = JVector.Transform(jacobian[9], body1.InverseInertiaWorld) * jacobian[1] +
                                     JVector.Transform(jacobian[11], body2.InverseInertiaWorld) * jacobian[3];

            data.EffectiveMass.M23 = JVector.Transform(jacobian[5], body1.InverseInertiaWorld) * jacobian[9] +
                                     JVector.Transform(jacobian[7], body2.InverseInertiaWorld) * jacobian[11];

            data.EffectiveMass.M32 = JVector.Transform(jacobian[9], body1.InverseInertiaWorld) * jacobian[5] +
                                     JVector.Transform(jacobian[11], body2.InverseInertiaWorld) * jacobian[7];

            Real coupling13 = JVector.Multiply(jacobian[0], body1.InverseMassVector) * jacobian[8] +
                              JVector.Multiply(jacobian[2], body2.InverseMassVector) * jacobian[10];
            Real coupling23 = JVector.Multiply(jacobian[4], body1.InverseMassVector) * jacobian[8] +
                              JVector.Multiply(jacobian[6], body2.InverseMassVector) * jacobian[10];
            data.EffectiveMass.M13 += coupling13;
            data.EffectiveMass.M31 += coupling13;
            data.EffectiveMass.M23 += coupling23;
            data.EffectiveMass.M32 += coupling23;
        }

        data.EffectiveMass.M11 += data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass.M22 += data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass.M33 += data.LimitSoftness * timeStep.InverseSubstepDt;

        if (data.Clamp == 1 || data.Clamp == 2)
        {
            data.BilateralMass = MathHelper.InverseBilateralBlock(data.EffectiveMass);
            data.CouplingX = data.EffectiveMass.M13;
            data.CouplingY = data.EffectiveMass.M23;
        }

        // 未启用的限位不得设置用于识别可响应行的尺度。
        if (data.Clamp == 0) data.EffectiveMass.M33 = 0;
        data.EffectiveMass = MathHelper.InverseSymmetric(data.EffectiveMass);

        data.Bias = error * timeStep.InverseStepDt;
        data.Bias.X *= data.BiasFactor;
        data.Bias.Y *= data.BiasFactor;
        data.Bias.Z *= data.LimitBias;

        // 限位发生变化时，不得用符号相反的冲量进行热启动。
        if (data.Clamp == 1) data.AccumulatedImpulse.Z = MathR.Min(0, data.AccumulatedImpulse.Z);
        else if (data.Clamp == 2) data.AccumulatedImpulse.Z = MathR.Max(0, data.AccumulatedImpulse.Z);

        JVector acc = data.AccumulatedImpulse;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity += JVector.Multiply(jacobian[0] * acc.X + jacobian[4] * acc.Y + jacobian[8] * acc.Z, body1.InverseMassVector);
            body1.AngularVelocity += JVector.Transform(jacobian[1] * acc.X + jacobian[5] * acc.Y + jacobian[9] * acc.Z, body1.InverseInertiaWorld);
        }

        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(jacobian[2] * acc.X + jacobian[6] * acc.Y + jacobian[10] * acc.Z, body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(jacobian[3] * acc.X + jacobian[7] * acc.Y + jacobian[11] * acc.Z, body2.InverseInertiaWorld);
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
    public JVector Impulse => Data.AccumulatedImpulse;

    /// <summary>
    /// 获取或设置距离限位启用时所应用的柔软度（柔顺性）。
    /// </summary>
    /// <value>
    /// 默认值为 0.0001。较高的值允许更大的限位违反量，但可提升稳定性。
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
    /// 获取或设置用于距离限位修正的偏置因子。
    /// </summary>
    /// <value>
    /// 默认值为 0.2。较高的值修正限位违反更快。
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

    [SkipLocalsInit]
    public static void IteratePointOnLine(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, PointOnLineData>(ref constraint);
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        JVector.Transform(data.LocalAxis, body1.Orientation, out JVector aw);

        JVector n1 = MathHelper.CreateOrthonormal(data.LocalAxis);
        JVector.Transform(n1, body1.Orientation, out n1);

        JVector n2 = aw % n1;

        JVector.Transform(data.LocalAnchor1, body1.Orientation, out JVector r1);
        JVector.Transform(data.LocalAnchor2, body2.Orientation, out JVector r2);

        JVector.Add(body1.Position, r1, out JVector p1);
        JVector.Add(body2.Position, r2, out JVector p2);

        JVector u = p2 - p1;

        Span<JVector> jacobian = stackalloc JVector[12];

        jacobian[0] = -n1;
        jacobian[1] = -((r1 + u) % n1);
        jacobian[2] = n1;
        jacobian[3] = r2 % n1;

        jacobian[4] = -n2;
        jacobian[5] = -((r1 + u) % n2);
        jacobian[6] = n2;
        jacobian[7] = r2 % n2;

        jacobian[8] = -aw;
        jacobian[9] = -((r1 + u) % aw);
        jacobian[10] = aw;
        jacobian[11] = r2 % aw;

        JVector jv;
        jv.X = jacobian[0] * body1.Velocity + jacobian[1] * body1.AngularVelocity + jacobian[2] * body2.Velocity +
               jacobian[3] * body2.AngularVelocity;

        jv.Y = jacobian[4] * body1.Velocity + jacobian[5] * body1.AngularVelocity + jacobian[6] * body2.Velocity +
               jacobian[7] * body2.AngularVelocity;

        jv.Z = jacobian[8] * body1.Velocity + jacobian[9] * body1.AngularVelocity + jacobian[10] * body2.Velocity +
               jacobian[11] * body2.AngularVelocity;

        JVector softnessVector = data.AccumulatedImpulse * timeStep.InverseSubstepDt;
        softnessVector.X *= data.Softness;
        softnessVector.Y *= data.Softness;
        softnessVector.Z *= data.LimitSoftness;

        JVector residual = jv + data.Bias + softnessVector;
        JVector lambda = -(Real)1.0 * JVector.Transform(residual, data.EffectiveMass);

        JVector origAcc = data.AccumulatedImpulse;

        data.AccumulatedImpulse += lambda;

        if ((data.Clamp == 1 && data.AccumulatedImpulse.Z > 0) ||
            (data.Clamp == 2 && data.AccumulatedImpulse.Z < 0))
        {
            // 将限位冲量固定在其边界上，并重新求解双边行。
            // 它们的完整块求解假设被丢弃的限位冲量已被施加。
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
            data.AccumulatedImpulse.Z = (Real)0.0;
            origAcc.Z = (Real)0.0;
        }

        lambda = data.AccumulatedImpulse - origAcc;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity += JVector.Multiply(jacobian[0] * lambda.X + jacobian[4] * lambda.Y + jacobian[8] * lambda.Z, body1.InverseMassVector);
            body1.AngularVelocity += JVector.Transform(jacobian[1] * lambda.X + jacobian[5] * lambda.Y + jacobian[9] * lambda.Z, body1.InverseInertiaWorld);
        }

        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(jacobian[2] * lambda.X + jacobian[6] * lambda.Y + jacobian[10] * lambda.Z, body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(jacobian[3] * lambda.X + jacobian[7] * lambda.Y + jacobian[11] * lambda.Z, body2.InverseInertiaWorld);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        ref PointOnLineData data = ref Data;
        JVector first = Body1.Position + JVector.Transform(data.LocalAnchor1, Body1.Orientation);
        JVector second = Body2.Position + JVector.Transform(data.LocalAnchor2, Body2.Orientation);
        JVector axis = JVector.Transform(data.LocalAxis, Body1.Orientation);
        Real distance = JVector.Dot(second - first, axis);
        JVector projected = first + axis * distance;
        Real minimum = data.Min, maximum = data.Max;
        if (minimum >= maximum) minimum = maximum = minimum * (Real)0.5 + maximum * (Real)0.5;
        Real start = Real.IsFinite(minimum) ? minimum : MathR.Min(-settings.GuideSize, distance - settings.GuideSize);
        Real end = Real.IsFinite(maximum) ? maximum : MathR.Max(settings.GuideSize, distance + settings.GuideSize);
        if (!Real.IsFinite(minimum) && Real.IsFinite(maximum)) start = MathR.Min(start, maximum - settings.GuideSize);
        if (!Real.IsFinite(maximum) && Real.IsFinite(minimum)) end = MathR.Max(end, minimum + settings.GuideSize);
        if (Real.IsFinite(start) && Real.IsFinite(end))
        {
            drawer.DrawSegment(first + axis * start, first + axis * end, settings.ReferenceColor);
            if (Real.IsFinite(minimum))
                drawer.DrawStop(first + axis * minimum, axis, settings.GuideSize * (Real)0.15,
                    minimum == maximum ? settings.UpperLimitColor : settings.LowerLimitColor);
            else drawer.DrawArrow(first + axis * start, -axis * (settings.GuideSize * (Real)0.25),
                settings.ReferenceColor, settings);
            if (Real.IsFinite(maximum) && minimum != maximum)
                drawer.DrawStop(first + axis * maximum, axis, settings.GuideSize * (Real)0.15, settings.UpperLimitColor);
            else if (!Real.IsFinite(maximum))
                drawer.DrawArrow(first + axis * end, axis * (settings.GuideSize * (Real)0.25),
                    settings.ReferenceColor, settings);
        }
        if (Body2 != Body2.World.NullBody) drawer.DrawSegment(Body2.Position, second, settings.CurrentColor);
        drawer.DrawSegment(second, projected, settings.CurrentColor);
        drawer.DrawPoint(first, settings.PointRadius, settings.ReferenceColor);
        drawer.DrawPoint(second, settings.PointRadius, settings.CurrentColor);
    }
}
