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
/// 限制两个刚体之间的相对倾斜，激活时移除一个角度自由度。
/// </summary>
/// <remarks>
/// 固定倾角必须严格位于零与 pi 之间。使用 <see cref="HingeAngle"/> 配合
/// <see cref="AngularLimit.Full"/>，可在保持轴对齐的同时允许绕轴旋转。
/// </remarks>
public unsafe class ConeLimit : Constraint<ConeLimit.ConeLimitData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct ConeLimitData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAxis1, LocalAxis2;

        public Real BiasFactor;
        public Real Softness;

        public Real EffectiveMass;
        public Real AccumulatedImpulse;
        public Real Bias;

        public Real MinAngle;
        public Real MaxAngle;

        public short Clamp;

        public MemoryHelper.MemBlock6Real J0;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationConeLimit, &IterateConeLimit);


    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = 0;

    /// <summary>
    /// 使用两个世界空间轴和一个角度范围初始化锥形限位。
    /// </summary>
    /// <param name="axisBody1">刚体 1 在世界空间中的参考轴。</param>
    /// <param name="axisBody2">刚体 2 在世界空间中的参考轴。</param>
    /// <param name="limit">允许的最小与最大倾角。</param>
    /// <remarks>
    /// 每个轴都存储为对应刚体上的局部轴。该约束测量
    /// 这些轴之间的夹角，并将其限制在给定范围内。
    /// 默认值：<see cref="Softness"/> = <see cref="Constraint.DefaultAngularSoftness"/>，<see cref="Bias"/> = <see cref="Constraint.DefaultAngularBias"/>。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="axisBody1"/> 或 <paramref name="axisBody2"/> 为零或包含非有限值时抛出。
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="limit"/> 超出范围 [0, pi]，或其上限小于
    /// 下限，或该限位将倾角固定为零或 pi 时抛出。
    /// </exception>
    public void Initialize(JVector axisBody1, JVector axisBody2, AngularLimit limit)
    {
        VerifyNotZero();
        ArgumentCheck.NonZero(axisBody1, nameof(axisBody1));
        ArgumentCheck.NonZero(axisBody2, nameof(axisBody2));
        ValidateLimit(limit, nameof(limit));

        ref ConeLimitData data = ref Data;
        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.NormalizeInPlace(ref axisBody1);
        JVector.NormalizeInPlace(ref axisBody2);

        JVector.ConjugatedTransform(axisBody1, body1.Orientation, out data.LocalAxis1);
        JVector.ConjugatedTransform(axisBody2, body2.Orientation, out data.LocalAxis2);

        data.Softness = Constraint.DefaultAngularSoftness;
        data.BiasFactor = Constraint.DefaultAngularBias;

        Real lower = (Real)limit.From;
        Real upper = (Real)limit.To;

        data.MinAngle = lower;
        data.MaxAngle = upper;
    }

    /// <summary>
    /// 使用一个世界空间轴和一个角度范围初始化锥形限位。
    /// </summary>
    /// <param name="axis">用于初始姿态的世界空间参考轴。</param>
    /// <param name="limit">允许的最小与最大倾角。</param>
    /// <remarks>
    /// 将该轴存储为每个刚体上的局部轴。该约束测量
    /// 这些轴之间的夹角，并将其限制在给定范围内。
    /// 默认值：<see cref="Softness"/> = <see cref="Constraint.DefaultAngularSoftness"/>，<see cref="Bias"/> = <see cref="Constraint.DefaultAngularBias"/>。
    /// </remarks>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="axis"/> 为零或包含非有限值时抛出。
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="limit"/> 超出范围 [0, pi]，或其上限小于
    /// 下限，或该限位将倾角固定为零或 pi 时抛出。
    /// </exception>
    public void Initialize(JVector axis, AngularLimit limit)
    {
        if (limit.From > (JAngle)0.0)
        {
            Logger.Warning(
                "{0}.{1}(): The lower limit is greater than 0, but this overload initializes both body axes " +
                "from the same world-space axis (rest angle = 0). Use the two-axis overload " +
                "if you need a non-zero minimum angle.",
                nameof(ConeLimit),
                nameof(Initialize));
        }

        // 两个刚体使用同一根轴 → 静止角为零。
        Initialize(axis, axis, limit);
    }

    /// <summary>
    /// 获取两个刚体轴之间的当前夹角。
    /// </summary>
    public JAngle Angle
    {
        get
        {
            ref ConeLimitData data = ref Data;

            ref RigidBodyData body1 = ref data.Body1.Data;
            ref RigidBodyData body2 = ref data.Body2.Data;

            JVector.Transform(data.LocalAxis1, body1.Orientation, out JVector a1);
            JVector.Transform(data.LocalAxis2, body2.Orientation, out JVector a2);

            return (JAngle)StableMath.Atan2(JVector.Cross(a1, a2).Length(), JVector.Dot(a1, a2));
        }
    }

    /// <summary>
    /// 获取或设置刚体 1 在世界空间中的参考轴。
    /// </summary>
    /// <exception cref="ArgumentException">
    /// 当所赋值为零或包含非有限值时抛出。
    /// </exception>
    public JVector AxisBody1
    {
        get
        {
            ref ConeLimitData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;

            JVector.Transform(data.LocalAxis1, body1.Orientation, out JVector axis);
            return axis;
        }
        set
        {
            ArgumentCheck.NonZero(value, nameof(value));

            ref ConeLimitData data = ref Data;
            ref RigidBodyData body1 = ref data.Body1.Data;

            JVector normalized = value;
            JVector.NormalizeInPlace(ref normalized);

            JVector.ConjugatedTransform(normalized, body1.Orientation, out data.LocalAxis1);
        }
    }

    /// <summary>
    /// 获取或设置刚体 2 在世界空间中的参考轴。
    /// </summary>
    /// <exception cref="ArgumentException">
    /// 当所赋值为零或包含非有限值时抛出。
    /// </exception>
    public JVector AxisBody2
    {
        get
        {
            ref ConeLimitData data = ref Data;
            ref RigidBodyData body2 = ref data.Body2.Data;

            JVector.Transform(data.LocalAxis2, body2.Orientation, out JVector axis);
            return axis;
        }
        set
        {
            ArgumentCheck.NonZero(value, nameof(value));

            ref ConeLimitData data = ref Data;
            ref RigidBodyData body2 = ref data.Body2.Data;

            JVector normalized = value;
            JVector.NormalizeInPlace(ref normalized);

            JVector.ConjugatedTransform(normalized, body2.Orientation, out data.LocalAxis2);
        }
    }

    /// <summary>
    /// 获取或设置锥形限位的角度限位。
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当所赋限位超出范围 [0, pi]，或其上限小于
    /// 下限，或该限位将倾角固定为零或 pi 时抛出。
    /// </exception>
    public AngularLimit Limit
    {
        get
        {
            ref ConeLimitData data = ref Data;
            return new AngularLimit((JAngle)data.MinAngle, (JAngle)data.MaxAngle);
        }
        set
        {
            ValidateLimit(value, nameof(value));

            ref ConeLimitData data = ref Data;
            data.MinAngle = (Real)value.From;
            data.MaxAngle = (Real)value.To;
        }
    }

    private static void ValidateLimit(in AngularLimit limit, string parameterName)
    {
        ArgumentCheck.InRange((Real)limit.From, (Real)0.0, MathR.PI, nameof(limit.From));
        ArgumentCheck.InRange((Real)limit.To, (Real)limit.From, MathR.PI, nameof(limit.To));
        if (limit.From == limit.To && (limit.From == (JAngle)0 || limit.From == (JAngle)MathR.PI))
        {
            throw new ArgumentOutOfRangeException(parameterName,
                "A fixed tilt of zero or pi is not supported. Use HingeAngle with AngularLimit.Full instead.");
        }
    }

    public static void PrepareForIterationConeLimit(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, ConeLimitData>(ref constraint);

        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector.Transform(data.LocalAxis1, body1.Orientation, out JVector a1);
        JVector.Transform(data.LocalAxis2, body2.Orientation, out JVector a2);

        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 2);

        JVector previousJacobian = jacobian[0];

        jacobian[0] = JVector.Cross(a2, a1);
        jacobian[1] = JVector.Cross(a1, a2);

        data.Clamp = 0;

        Real error = JVector.Dot(a1, a2);
        Real lower = -data.MinAngle, upper = -data.MaxAngle;

        // 全程使用角度坐标，使柔软度和缓存的冲量保持
        // 相同的单位。残差使用其负值以保留限位顺序；
        // 下面的雅可比和取负的偏置使用正角度。
        Real sineSquared = jacobian[0].LengthSquared();
        Real sine = MathR.Sqrt(sineSquared);
        if (sine > 0)
        {
            jacobian[0] *= (Real)1.0 / sine;
        }
        else
        {
            // 在端点处角度存在尖点。围绕一个响应良好的垂直轴
            // 选取单侧导数。
            JVector axis = MathHelper.CreateOrthonormal(a1);
            JVector other = a1 % axis;
            JSymmetricMatrix inverseInertia = body1.InverseInertiaWorld + body2.InverseInertiaWorld;
            jacobian[0] = JVector.Transform(axis, inverseInertia) * axis >=
                          JVector.Transform(other, inverseInertia) * other ? axis : other;
        }

        jacobian[1] = -jacobian[0];
        if (previousJacobian * jacobian[0] < 0) data.AccumulatedImpulse = 0;
        error = -StableMath.Atan2(sine, error);

        if (lower == upper)
        {
            data.Clamp = 3;
            error -= upper;
        }
        else if (error <= upper && data.MaxAngle < MathR.PI)
        {
            data.Clamp = 1;
            error -= upper;
        }
        else if (error >= lower && data.MinAngle > 0)
        {
            data.Clamp = 2;
            error -= lower;
        }
        else
        {
            data.AccumulatedImpulse = (Real)0.0;
            return;
        }

        if (data.Clamp == 1) data.AccumulatedImpulse = MathR.Min(data.AccumulatedImpulse, 0);
        else if (data.Clamp == 2) data.AccumulatedImpulse = MathR.Max(data.AccumulatedImpulse, 0);

        data.EffectiveMass = JVector.Transform(jacobian[0], body1.InverseInertiaWorld) * jacobian[0] +
                             JVector.Transform(jacobian[1], body2.InverseInertiaWorld) * jacobian[1];

        data.EffectiveMass += data.Softness * timeStep.InverseSubstepDt;

        data.EffectiveMass = data.EffectiveMass > 0 ? (Real)1.0 / data.EffectiveMass : 0;
        if (data.EffectiveMass == 0) data.AccumulatedImpulse = 0;

        data.Bias = -error * data.BiasFactor * timeStep.InverseStepDt;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.AngularVelocity +=
                JVector.Transform(data.AccumulatedImpulse * jacobian[0], body1.InverseInertiaWorld);
        }

        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.AngularVelocity +=
                JVector.Transform(data.AccumulatedImpulse * jacobian[1], body2.InverseInertiaWorld);
        }
    }

    /// <summary>
    /// 获取或设置约束的柔软度（柔量）。
    /// </summary>
    /// <value>
    /// 默认为 0.001。值越大允许的角度误差越大，但稳定性越好。
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
    /// 获取或设置偏置因子，用于控制角度误差的修正强度。
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

    public static void IterateConeLimit(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, ConeLimitData>(ref constraint);
        ref RigidBodyData body1 = ref constraint.Body1.Data;
        ref RigidBodyData body2 = ref constraint.Body2.Data;

        if (data.Clamp == 0) return;

        var jacobian = new Span<JVector>(Unsafe.AsPointer(ref data.J0), 2);

        Real jv =
            body1.AngularVelocity * jacobian[0] +
            body2.AngularVelocity * jacobian[1];

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
            body1.AngularVelocity += JVector.Transform(lambda * jacobian[0], body1.InverseInertiaWorld);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.AngularVelocity += JVector.Transform(lambda * jacobian[1], body2.InverseInertiaWorld);
        }
    }

    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();
        JVector origin = GetDebugDrawOrigin(settings);
        JVector axis = AxisBody1;
        // 在刚体 1 的局部坐标系中选择参考方向，使线框几何随其一起旋转。
        JVector reference = JVector.Transform(MathHelper.CreateOrthonormal(Data.LocalAxis1), Body1.Orientation);
        AngularLimit limit = Limit;
        drawer.DrawArrow(origin, axis * settings.GuideSize, settings.ReferenceColor, settings);
        drawer.DrawArrow(origin, AxisBody2 * settings.GuideSize, settings.CurrentColor, settings);
        if (limit.From.Radian == 0 && limit.To.Radian == MathR.PI)
            drawer.DrawSphere(origin, settings.GuideSize, settings.ReferenceColor, settings.Segments);
        else
            drawer.DrawCone(origin, axis, reference, limit.To, limit.To, settings.GuideSize,
                settings.UpperLimitColor, settings.Segments);
        if (limit.From.Radian > 0 && limit.From != limit.To)
            drawer.DrawCone(origin, axis, reference, limit.From, limit.From, settings.GuideSize,
                settings.LowerLimitColor, settings.Segments, settings.Segments / 16);
    }
}
