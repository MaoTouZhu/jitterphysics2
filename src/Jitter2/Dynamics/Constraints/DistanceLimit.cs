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
/// 约束一个刚体参考系中的固定点与另一个刚体参考系中的固定点之间的距离。
/// 固定限位会移除一个平动自由度；范围限位在到达边界时生效。若距离为零，
/// 请使用 <see cref="BallSocket"/> 约束。
/// </summary>
public unsafe class DistanceLimit : Constraint<DistanceLimit.DistanceLimitData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct DistanceLimitData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAnchor1;
        public JVector LocalAnchor2;

        public Real BiasFactor;
        public Real Softness;
        public Real Distance;

        public Real LimitMin;
        public Real LimitMax;

        public Real EffectiveMass;
        public Real AccumulatedImpulse;
        public Real Bias;

        public MemoryHelper.MemBlock12Real J0;

        public short Clamp;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationDistanceLimit, &IterateDistanceLimit);


    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = (Real)0.0;

    /// <summary>
    /// 使用锚点之间的固定距离初始化约束。
    /// </summary>
    /// <param name="anchor1">第一个刚体上的锚点，位于世界空间。</param>
    /// <param name="anchor2">第二个刚体上的锚点，位于世界空间。</param>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="anchor1"/> 或 <paramref name="anchor2"/> 包含非有限值时抛出。
    /// </exception>
    public void Initialize(JVector anchor1, JVector anchor2)
    {
        Initialize(anchor1, anchor2, LinearLimit.Fixed);
    }

    /// <summary>
    /// 使用锚点之间的距离限位初始化约束。
    /// </summary>
    /// <param name="anchor1">第一个刚体上的锚点，位于世界空间。</param>
    /// <param name="anchor2">第二个刚体上的锚点，位于世界空间。</param>
    /// <param name="limit">相对于锚点初始距离所允许的偏移量。
    /// 例如，限位 (-1, 1) 允许距离在任一方向上最多变化一个单位。</param>
    /// <remarks>
    /// 根据当前姿态计算局部锚点和初始距离。
    /// 相等或反向的限位会将距离偏移固定在其中点。
    /// 默认值：<see cref="Softness"/> = <see cref="Constraint.DefaultLinearSoftness"/>，<see cref="Bias"/> = <see cref="Constraint.DefaultLinearBias"/>。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="anchor1"/> 或 <paramref name="anchor2"/> 包含非有限值，
    /// 或 <paramref name="limit"/> 中的任一值为 NaN 时抛出。
    /// </exception>
    public void Initialize(JVector anchor1, JVector anchor2, LinearLimit limit)
    {
        VerifyNotZero();
        ArgumentCheck.Finite(anchor1, nameof(anchor1));
        ArgumentCheck.Finite(anchor2, nameof(anchor2));
        ArgumentCheck.NotNaN(limit.From, nameof(limit.From));
        ArgumentCheck.NotNaN(limit.To, nameof(limit.To));

        ref DistanceLimitData data = ref Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Subtract(anchor1, body1.Position, out data.LocalAnchor1);
        JVector.Subtract(anchor2, body2.Position, out data.LocalAnchor2);

        JVector.ConjugatedTransform(data.LocalAnchor1, body1.Orientation, out data.LocalAnchor1);
        JVector.ConjugatedTransform(data.LocalAnchor2, body2.Orientation, out data.LocalAnchor2);

        data.Softness = Constraint.DefaultLinearSoftness;
        data.BiasFactor = Constraint.DefaultLinearBias;
        data.Distance = (anchor2 - anchor1).Length();

        (data.LimitMin, data.LimitMax) = limit;
    }

    /// <summary>
    /// 获取或设置第一个刚体上的锚点，位于世界空间。该锚点
    /// 固定在第一个刚体的局部参考系中。
    /// </summary>
    [ReferenceFrame(ReferenceFrame.World)]
    public JVector Anchor1
    {
        set
        {
            DebugCheck.IsFinite(value, nameof(value));

            ref DistanceLimitData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;
            JVector.Subtract(value, body1.Position, out data.LocalAnchor1);
            JVector.ConjugatedTransform(data.LocalAnchor1, body1.Orientation, out data.LocalAnchor1);
        }
        get
        {
            ref DistanceLimitData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;
            JVector.Transform(data.LocalAnchor1, body1.Orientation, out JVector result);
            JVector.Add(result, body1.Position, out result);
            return result;
        }
    }

    /// <summary>
    /// 获取或设置第二个刚体上的锚点，位于世界空间。该锚点
    /// 固定在第二个刚体的局部参考系中。
    /// </summary>
    [ReferenceFrame(ReferenceFrame.World)]
    public JVector Anchor2
    {
        set
        {
            DebugCheck.IsFinite(value, nameof(value));

            ref DistanceLimitData data = ref Data;
            ref RigidBodyData body2 = ref data.Body2.Data;
            JVector.Subtract(value, body2.Position, out data.LocalAnchor2);
            JVector.ConjugatedTransform(data.LocalAnchor2, body2.Orientation, out data.LocalAnchor2);
        }
        get
        {
            ref DistanceLimitData data = ref Data;
            ref RigidBodyData body2 = ref data.Body2.Data;
            JVector.Transform(data.LocalAnchor2, body2.Orientation, out JVector result);
            JVector.Add(result, body2.Position, out result);
            return result;
        }
    }

    /// <summary>
    /// 获取或设置锚点之间的目标距离。
    /// </summary>
    /// <value>初始化时根据锚点的初始间距设置。</value>
    public Real TargetDistance
    {
        set
        {
            DebugCheck.IsNonNegative(value, nameof(value));

            ref DistanceLimitData data = ref Data;
            data.Distance = value;
        }
        get => Data.Distance;
    }

    /// <summary>
    /// 获取锚点之间在世界空间中的当前距离。
    /// </summary>
    public Real Distance
    {
        get
        {
            ref DistanceLimitData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;
            ref RigidBodyData body2 = ref data.Body2.Data;

            JVector.Transform(data.LocalAnchor1, body1.Orientation, out JVector r1);
            JVector.Transform(data.LocalAnchor2, body2.Orientation, out JVector r2);

            JVector.Add(body1.Position, r1, out JVector p1);
            JVector.Add(body2.Position, r2, out JVector p2);

            JVector.Subtract(p2, p1, out JVector dp);

            return dp.Length();
        }
    }

    public static void PrepareForIterationDistanceLimit(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, DistanceLimitData>(ref constraint);
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Transform(data.LocalAnchor1, body1.Orientation, out JVector r1);
        JVector.Transform(data.LocalAnchor2, body2.Orientation, out JVector r2);

        JVector.Add(body1.Position, r1, out JVector p1);
        JVector.Add(body2.Position, r2, out JVector p2);

        JVector.Subtract(p2, p1, out JVector dp);

        Real distance = dp.Length();
        Real error = distance - data.Distance;

        data.Clamp = 0;

        if (data.LimitMin >= data.LimitMax)
        {
            error -= (Real)0.5 * data.LimitMin + (Real)0.5 * data.LimitMax;
            data.Clamp = 3;
        }
        else if (error >= data.LimitMax)
        {
            data.Clamp = 1;
            error -= data.LimitMax;
        }
        else if (error <= data.LimitMin)
        {
            data.Clamp = 2;
            error -= data.LimitMin;
        }
        else
        {
            data.AccumulatedImpulse = (Real)0.0;
            return;
        }

        // 锚点完全重合时，距离约束没有方向。
        // 任何非零间距都需要单位雅可比，包括极小的距离。
        if (distance == 0)
        {
            data.Clamp = 0;
            data.AccumulatedImpulse = 0;
            return;
        }

        JVector n = dp * ((Real)1.0 / distance);

        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 4);

        jacobian[0] = -(Real)1.0 * n;
        jacobian[1] = -(Real)1.0 * (r1 % n);
        jacobian[2] = (Real)1.0 * n;
        jacobian[3] = r2 % n;

        data.EffectiveMass = JVector.Multiply(jacobian[0], body1.InverseMassVector) * jacobian[0] +
                             JVector.Multiply(jacobian[2], body2.InverseMassVector) * jacobian[2] +
                             JVector.Transform(jacobian[1], body1.InverseInertiaWorld) * jacobian[1] +
                             JVector.Transform(jacobian[3], body2.InverseInertiaWorld) * jacobian[3];

        data.EffectiveMass += data.Softness * timeStep.InverseSubstepDt;

        data.EffectiveMass = data.EffectiveMass > 0 ? (Real)1.0 / data.EffectiveMass : 0;
        if (data.EffectiveMass == 0) data.AccumulatedImpulse = 0;

        data.Bias = error * data.BiasFactor * timeStep.InverseStepDt;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity += JVector.Multiply(data.AccumulatedImpulse * jacobian[0], body1.InverseMassVector);
            body1.AngularVelocity += JVector.Transform(data.AccumulatedImpulse * jacobian[1], body1.InverseInertiaWorld);
        }

        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(data.AccumulatedImpulse * jacobian[2], body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(data.AccumulatedImpulse * jacobian[3], body2.InverseInertiaWorld);
        }
    }

    /// <summary>
    /// 获取或设置约束的柔软度（柔量）。
    /// </summary>
    /// <value>
    /// 默认为 <see cref="Constraint.DefaultLinearSoftness"/>。值越大允许的距离误差越大，但稳定性越好。
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
    /// 获取或设置偏置因子，用于控制距离误差的修正强度。
    /// </summary>
    /// <value>
    /// 默认为 0.2。值越大误差修正越快，但可能引起不稳定。
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
    /// 获取上一个已求解子步累积的求解器冲量。
    /// </summary>
    public Real Impulse => Data.AccumulatedImpulse;

    public static void IterateDistanceLimit(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, DistanceLimitData>(ref constraint);
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        if (data.Clamp == 0) return;

        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 4);

        Real jv =
            body1.Velocity * jacobian[0] +
            body1.AngularVelocity * jacobian[1] +
            body2.Velocity * jacobian[2] +
            body2.AngularVelocity * jacobian[3];

        Real softnessScalar = data.AccumulatedImpulse * data.Softness * timeStep.InverseSubstepDt;

        Real lambda = -data.EffectiveMass * (jv + data.Bias + softnessScalar);

        Real oldAccumulated = data.AccumulatedImpulse;

        data.AccumulatedImpulse += lambda;

        if (data.Clamp == 1)
        {
            data.AccumulatedImpulse = MathR.Min(data.AccumulatedImpulse, (Real)0.0);
        }
        else if (data.Clamp == 2)
        {
            data.AccumulatedImpulse = MathR.Max(data.AccumulatedImpulse, (Real)0.0);
        }

        lambda = data.AccumulatedImpulse - oldAccumulated;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.Velocity += JVector.Multiply(lambda * jacobian[0], body1.InverseMassVector);
            body1.AngularVelocity += JVector.Transform(lambda * jacobian[1], body1.InverseInertiaWorld);
        }

        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.Velocity += JVector.Multiply(lambda * jacobian[2], body2.InverseMassVector);
            body2.AngularVelocity += JVector.Transform(lambda * jacobian[3], body2.InverseInertiaWorld);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        ref DistanceLimitData data = ref Data;
        JVector first = Anchor1, second = Anchor2;
        if (Body1 != Body1.World.NullBody) drawer.DrawSegment(Body1.Position, first, settings.ReferenceColor);
        if (Body2 != Body2.World.NullBody) drawer.DrawSegment(Body2.Position, second, settings.CurrentColor);
        drawer.DrawSegment(first, second, settings.CurrentColor);
        drawer.DrawPoint(first, settings.PointRadius, settings.ReferenceColor);
        drawer.DrawPoint(second, settings.PointRadius, settings.CurrentColor);
        if (data.LimitMin >= data.LimitMax)
        {
            Real radius = data.Distance + data.LimitMin * (Real)0.5 + data.LimitMax * (Real)0.5;
            if (Real.IsFinite(radius) && radius >= 0)
                drawer.DrawSphere(first, radius, settings.UpperLimitColor, settings.Segments);
        }
        else
        {
            Real minimum = data.Distance + data.LimitMin, maximum = data.Distance + data.LimitMax;
            if (Real.IsFinite(minimum) && minimum > 0)
                drawer.DrawSphere(first, minimum, settings.LowerLimitColor, settings.Segments);
            if (Real.IsFinite(maximum) && maximum >= 0)
                drawer.DrawSphere(first, maximum, settings.UpperLimitColor, settings.Segments);
        }
    }
}
