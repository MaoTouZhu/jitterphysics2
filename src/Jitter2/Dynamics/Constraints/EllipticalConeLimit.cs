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
/// 将相对摆角限制在两个椭圆锥之间的区域，且不约束扭转。
/// </summary>
/// <remarks>
/// 刚体 1 拥有锥体坐标系；刚体 2 拥有被约束的轴。边界是摆角坐标中的椭圆
/// （最短倾斜角，沿两个坐标系方向分解）。
/// 上下边界相等时，轴被约束在锥面上。两个方向的取值范围相同则退化为圆锥。
/// 该约束不限制平移。
/// </remarks>
public unsafe class EllipticalConeLimit : Constraint<EllipticalConeLimit.EllipticalConeLimitData>
{
    [StructLayout(LayoutKind.Sequential)]
    public struct EllipticalConeLimitData
    {
        internal int _internal;
        internal uint DispatchId;
        internal ulong ConstraintId;

        public JHandle<RigidBodyData> Body1;
        public JHandle<RigidBodyData> Body2;

        public JVector LocalAxis1, LocalAxis2, LocalReference1;

        public Real MinAngle1;
        public Real MaxAngle1;
        public Real MinAngle2;
        public Real MaxAngle2;

        public Real Softness;
        public Real BiasFactor;

        public Real EffectiveMass;
        public Real AccumulatedImpulse;
        public Real Bias;

        public JVector Jacobian;

        public short Clamp;
    }

    private static readonly uint RegisteredDispatchId =
        RegisterFullConstraint(&PrepareForIterationEllipticalConeLimit, &IterateEllipticalConeLimit);

    protected override void Create()
    {
        DispatchId = RegisteredDispatchId;
        base.Create();
    }

    /// <inheritdoc />
    public override void ResetWarmStart() => Data.AccumulatedImpulse = 0;

    /// <summary>
    /// 使用同一世界空间方向初始化两个刚体的轴。
    /// </summary>
    /// <remarks>
    /// 参考方向定义刚体 1 上的第一个椭圆半轴。其沿锥轴的分量会被舍弃。
    /// 第二个方向为轴与参考方向的叉积。
    /// 内边界为正值表示初始对齐姿态位于允许区域之外。
    /// 若要以非零摆角初始化姿态，请参见双轴重载。
    /// </remarks>
    public void Initialize(JVector axis, JVector referenceDirection,
        AngularLimit limitAlongReference, AngularLimit limitAlongPerpendicular)
    {
        Initialize(axis, axis, referenceDirection, limitAlongReference, limitAlongPerpendicular);
    }

    /// <summary>
    /// 使用世界空间轴以及刚体 1 上的世界空间参考方向初始化约束。
    /// </summary>
    /// <param name="axisBody1">锥体的中心轴，附着于刚体 1。</param>
    /// <param name="axisBody2">附着于刚体 2、其摆角受约束的轴。</param>
    /// <param name="referenceDirectionBody1">第一个椭圆方向，附着于刚体 1。</param>
    /// <param name="limitAlongReference">沿参考方向的内、外角半轴。</param>
    /// <param name="limitAlongPerpendicular">沿垂直方向的内、外角半轴。</param>
    /// <exception cref="ArgumentException">某个方向无效，或参考方向与锥轴平行。</exception>
    /// <exception cref="ArgumentOutOfRangeException">限值不满足 <see cref="SetLimits"/> 的要求。</exception>
    public void Initialize(JVector axisBody1, JVector axisBody2, JVector referenceDirectionBody1,
        AngularLimit limitAlongReference, AngularLimit limitAlongPerpendicular)
    {
        VerifyNotZero();
        ArgumentCheck.NonZero(axisBody1, nameof(axisBody1));
        ArgumentCheck.NonZero(axisBody2, nameof(axisBody2));
        ArgumentCheck.NonZero(referenceDirectionBody1, nameof(referenceDirectionBody1));
        ValidateLimits(limitAlongReference, limitAlongPerpendicular);

        JVector.NormalizeInPlace(ref axisBody1);
        JVector.NormalizeInPlace(ref axisBody2);
        JVector.NormalizeInPlace(ref referenceDirectionBody1);

        referenceDirectionBody1 -= axisBody1 * JVector.Dot(axisBody1, referenceDirectionBody1);

        if (referenceDirectionBody1.LengthSquared() < (Real)1e-12)
        {
            throw new ArgumentException("The reference direction must not be parallel to the cone axis.",
                nameof(referenceDirectionBody1));
        }

        JVector.NormalizeInPlace(ref referenceDirectionBody1);

        ref var data = ref Data;

        JVector.ConjugatedTransform(axisBody1, data.Body1.Data.Orientation, out data.LocalAxis1);
        JVector.ConjugatedTransform(axisBody2, data.Body2.Data.Orientation, out data.LocalAxis2);
        JVector.ConjugatedTransform(referenceDirectionBody1, data.Body1.Data.Orientation, out data.LocalReference1);

        data.Softness = DefaultAngularSoftness;
        data.BiasFactor = DefaultAngularBias;
        data.Jacobian = JVector.Zero;

        SetLimits(limitAlongReference, limitAlongPerpendicular);
    }

    /// <summary>
    /// 获取刚体 1 在世界空间中的中心锥轴。
    /// </summary>
    public JVector AxisBody1 => JVector.Transform(Data.LocalAxis1, Data.Body1.Data.Orientation);

    /// <summary>
    /// 获取刚体 2 在世界空间中被约束的轴。
    /// </summary>
    public JVector AxisBody2 => JVector.Transform(Data.LocalAxis2, Data.Body2.Data.Orientation);

    /// <summary>
    /// 获取世界空间中的第一个椭圆方向，垂直于 <see cref="AxisBody1"/>。
    /// </summary>
    public JVector ReferenceDirectionBody1 => JVector.Transform(Data.LocalReference1, Data.Body1.Data.Orientation);

    /// <summary>
    /// 获取沿参考方向的内、外角半轴。
    /// </summary>
    public AngularLimit LimitAlongReference => new((JAngle)Data.MinAngle1, (JAngle)Data.MaxAngle1);

    /// <summary>
    /// 获取沿垂直方向的内、外角半轴。
    /// </summary>
    public AngularLimit LimitAlongPerpendicular => new((JAngle)Data.MinAngle2, (JAngle)Data.MaxAngle2);

    /// <summary>
    /// 同时设置两个角度范围，并清除缓存的冲量。
    /// </summary>
    /// <remarks>
    /// 每个范围必须满足 0 &lt;= From &lt;= To &lt; pi，且 To 严格为正。
    /// 下界必须同时为零（无内边界）或同时为正。
    /// 两个范围的上界相等时，定义固定的椭圆锥面。
    /// 不支持固定的零倾斜或 pi 倾斜；请改用带 AngularLimit.Full 的 HingeAngle。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">范围违反了上述要求。</exception>
    public void SetLimits(AngularLimit limitAlongReference, AngularLimit limitAlongPerpendicular)
    {
        ValidateLimits(limitAlongReference, limitAlongPerpendicular);

        ref var data = ref Data;

        data.MinAngle1 = (Real)limitAlongReference.From;
        data.MaxAngle1 = (Real)limitAlongReference.To;
        data.MinAngle2 = (Real)limitAlongPerpendicular.From;
        data.MaxAngle2 = (Real)limitAlongPerpendicular.To;

        data.AccumulatedImpulse = 0;
        data.Clamp = 0;
    }

    private static void ValidateLimits(AngularLimit first, AngularLimit second)
    {
        ValidateRange(first, nameof(first));
        ValidateRange(second, nameof(second));

        if ((first.From == (JAngle)0) != (second.From == (JAngle)0))
        {
            throw new ArgumentOutOfRangeException(nameof(second),
                "The inner semiaxes must either both be zero or both be positive.");
        }
    }

    private static void ValidateRange(AngularLimit limit, string parameterName)
    {
        Real min = (Real)limit.From;
        Real max = (Real)limit.To;

        if (!Real.IsFinite(min) || !Real.IsFinite(max) || min < 0 || max <= 0 || max >= MathR.PI || min > max)
        {
            throw new ArgumentOutOfRangeException(parameterName,
                "Expected 0 <= From <= To < pi and To > 0. Use HingeAngle with AngularLimit.Full for fixed zero or pi tilt.");
        }
    }

    /// <summary>
    /// 获取沿参考方向和垂直方向的有符号摆角坐标。
    /// 它们的欧几里得长度为倾斜角。当两轴恰好相反时，摆角方向不确定。
    /// </summary>
    public void GetSwing(out JAngle alongReference, out JAngle alongPerpendicular)
    {
        JVector axis = AxisBody1;
        JVector reference = ReferenceDirectionBody1;
        JVector other = axis % reference;
        JVector target = AxisBody2;

        Real x = target * reference;
        Real y = target * other;
        Real sine = MathR.Sqrt(x * x + y * y);
        Real angle = StableMath.Atan2(sine, target * axis);
        Real factor = sine > 0 ? angle / sine : 0;

        alongReference = (JAngle)(sine > 0 ? x * factor : angle);
        alongPerpendicular = (JAngle)(y * factor);
    }

    /// <summary>
    /// 获取或设置柔软度（柔顺度）。默认值为 0.001。
    /// </summary>
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
    /// 获取或设置角度误差修正因子。默认值为 0.2。
    /// </summary>
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
    public Real Impulse => Data.AccumulatedImpulse;

    // 摆角坐标中椭圆的半径及对数方位角导数。
    private static Real EllipseRadius(Real a, Real b, Real cosine, Real sine, out Real derivative)
    {
        Real scale = MathR.Min(a, b);
        Real u = scale / a;
        Real v = scale / b;

        Real squared = cosine * cosine * u * u + sine * sine * v * v;
        derivative = cosine * sine * (v * v - u * u) / squared;

        return scale / MathR.Sqrt(squared);
    }

    public static void PrepareForIterationEllipticalConeLimit(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, EllipticalConeLimitData>(ref constraint);

        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        JVector axis = JVector.Transform(data.LocalAxis1, body1.Orientation);
        JVector reference = JVector.Transform(data.LocalReference1, body1.Orientation);
        JVector other = axis % reference;
        JVector target = JVector.Transform(data.LocalAxis2, body2.Orientation);

        Real x = target * reference;
        Real y = target * other;
        Real z = target * axis;
        Real sine = MathR.Sqrt(x * x + y * y);
        Real angle = StableMath.Atan2(sine, z);

        Real cosineAzimuth, sineAzimuth;

        if (sine > 0)
        {
            cosineAzimuth = x / sine;
            sineAzimuth = y / sine;
        }
        else
        {
            // 对数在两极处没有唯一方向。选择一个具有响应性角度轴的主方向，
            // 包括某个刚体的旋转受到限制的情形。
            JSymmetricMatrix inertia = body1.InverseInertiaWorld + body2.InverseInertiaWorld;
            bool useReference = JVector.Transform(other, inertia) * other >= JVector.Transform(reference, inertia) * reference;
            cosineAzimuth = useReference ? 1 : 0;
            sineAzimuth = useReference ? 0 : 1;
        }

        Real outer = EllipseRadius(data.MaxAngle1, data.MaxAngle2, cosineAzimuth, sineAzimuth, out Real derivative);
        bool fixedCone = data.MinAngle1 == data.MaxAngle1 && data.MinAngle2 == data.MaxAngle2;
        Real boundary = outer;
        short clamp = 0;

        if (fixedCone)
        {
            clamp = 3;
        }
        else if (angle >= outer)
        {
            clamp = 1;
        }
        else if (data.MinAngle1 > 0)
        {
            Real inner = EllipseRadius(data.MinAngle1, data.MinAngle2, cosineAzimuth, sineAzimuth, out Real innerDerivative);

            if (angle <= inner)
            {
                clamp = 2;
                boundary = inner;
                derivative = innerDerivative;
            }
        }

        short previousClamp = data.Clamp;
        data.Clamp = clamp;

        if (clamp == 0)
        {
            data.AccumulatedImpulse = 0;
            return;
        }

        JVector tiltDirection = cosineAzimuth * reference + sineAzimuth * other;
        JVector azimuthDirection = axis % tiltDirection;
        JVector angleDirection = z * tiltDirection - sine * axis;

        // 对 sqrt((swing1/a)^2 + (swing2/b)^2) - 1 求导，然后将该行缩放
        // 为单位角度雅可比。这包含了椭圆方位角的变化；
        // 若只使用 cross(target, axis)，会错误地将限位变成圆形。
        Real azimuthFactor = sine > 0 ? angle / sine * derivative : 0;

        // 先缩放再求长度：在对跖点附近，对数的方位角导数可能大到
        // 使向量长度平方溢出。
        Real scale = MathR.Max((Real)1, MathR.Abs(azimuthFactor));
        JVector jacobian = -azimuthDirection * ((Real)1 / scale) + (azimuthFactor / scale) * angleDirection;
        Real inverseLength = (Real)1 / jacobian.Length();

        jacobian *= inverseLength;
        inverseLength /= scale;

        if (previousClamp != clamp || data.Jacobian * jacobian < 0)
        {
            data.AccumulatedImpulse = 0;
        }

        data.Jacobian = jacobian;

        if (clamp == 1)
        {
            data.AccumulatedImpulse = MathR.Min(data.AccumulatedImpulse, 0);
        }
        else if (clamp == 2)
        {
            data.AccumulatedImpulse = MathR.Max(data.AccumulatedImpulse, 0);
        }

        Real mass = JVector.Transform(jacobian, body1.InverseInertiaWorld) * jacobian +
                    JVector.Transform(jacobian, body2.InverseInertiaWorld) * jacobian +
                    data.Softness * timeStep.InverseSubstepDt;
        data.EffectiveMass = mass > 0 ? (Real)1 / mass : 0;

        if (data.EffectiveMass == 0)
        {
            data.AccumulatedImpulse = 0;
        }

        data.Bias = (angle - boundary) * inverseLength * data.BiasFactor * timeStep.InverseStepDt;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.AngularVelocity += JVector.Transform(data.AccumulatedImpulse * jacobian, body1.InverseInertiaWorld);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.AngularVelocity -= JVector.Transform(data.AccumulatedImpulse * jacobian, body2.InverseInertiaWorld);
        }
    }

    public static void IterateEllipticalConeLimit(ref ConstraintData constraint, in TimeStep timeStep)
    {
        ref var data = ref Unsafe.As<ConstraintData, EllipticalConeLimitData>(ref constraint);

        if (data.Clamp == 0) return;

        ref RigidBodyData body1 = ref data.Body1.Data;
        ref RigidBodyData body2 = ref data.Body2.Data;

        Real jv = (body1.AngularVelocity - body2.AngularVelocity) * data.Jacobian;
        Real lambda = -data.EffectiveMass * (jv + data.Bias +
            data.AccumulatedImpulse * data.Softness * timeStep.InverseSubstepDt);

        Real old = data.AccumulatedImpulse;
        data.AccumulatedImpulse += lambda;

        if (data.Clamp == 1)
        {
            data.AccumulatedImpulse = MathR.Min(data.AccumulatedImpulse, 0);
        }
        else if (data.Clamp == 2)
        {
            data.AccumulatedImpulse = MathR.Max(data.AccumulatedImpulse, 0);
        }

        lambda = data.AccumulatedImpulse - old;

        if (body1.MotionType == MotionType.Dynamic)
        {
            body1.AngularVelocity += JVector.Transform(lambda * data.Jacobian, body1.InverseInertiaWorld);
        }
        if (body2.MotionType == MotionType.Dynamic)
        {
            body2.AngularVelocity -= JVector.Transform(lambda * data.Jacobian, body2.InverseInertiaWorld);
        }
    }

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer) => DebugDraw(drawer, DebugDrawSettings.Default);

    /// <inheritdoc />
    public override void DebugDraw(IDebugDrawer drawer, in DebugDrawSettings settings)
    {
        ArgumentNullException.ThrowIfNull(drawer);
        VerifyNotZero();

        JVector origin = GetDebugDrawOrigin(settings);
        JVector axis = AxisBody1, reference = ReferenceDirectionBody1;

        drawer.DrawArrow(origin, axis * settings.GuideSize, settings.ReferenceColor, settings);
        drawer.DrawArrow(origin, AxisBody2 * settings.GuideSize, settings.CurrentColor, settings);

        drawer.DrawCone(origin, axis, reference, (JAngle)Data.MaxAngle1, (JAngle)Data.MaxAngle2,
            settings.GuideSize, settings.UpperLimitColor, settings.Segments);

        if (Data.MinAngle1 > 0 && (Data.MinAngle1 != Data.MaxAngle1 || Data.MinAngle2 != Data.MaxAngle2))
        {
            drawer.DrawCone(origin, axis, reference, (JAngle)Data.MinAngle1, (JAngle)Data.MinAngle2,
                settings.GuideSize, settings.LowerLimitColor, settings.Segments, settings.Segments / 16);
        }
    }
}
