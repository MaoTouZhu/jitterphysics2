using System;
using Jitter2;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Constraints;
using Jitter2.LinearMath;

namespace JitterDemo;

/*
 --- 1. 基本思路 ---
 我们使用标准的 DistanceLimit 约束来模拟齿轮连接。
 该约束就像一个“虚拟齿”，在接触点处把两个刚体钉在一起。

 关键在于，我们挂接进物理循环（OnPreStep），以便
 每一帧都更新该约束。由于齿轮在旋转，表面上
 相互接触的具体点不断变化。通过每帧把 DistanceLimit 的锚点重置到
 当前世界接触点，我们既允许齿轮自由旋转，
 又在接触处维持速度关系。

 --- 2. 改进：误差修正 ---
 如果我们严格把锚点放在当前接触点，齿轮最终会
 逐渐失步（“打滑”）。这是因为物理求解器的
 迭代次数有限，无法每一帧都完美求解
 速度约束，从而累积位置误差。

 为解决这个问题，本演示实现了一个主动误差修正系统：
 - 我们跟踪两个齿轮的精确总旋转量（计入完整的 360 度圈数）。
 - 我们计算“角度误差”（它们偏离理想传动比的程度）。
 - 我们依据该误差沿切线方向略微移动 BallSocket 锚点。

 这会迫使物理求解器把齿轮推回完美相位，
 修正任何漂移。
 */

public class GearCoupling
{
    // DistanceLimit 似乎能带来更稳定的系统。
    // BallSocketConstraint 会过度约束系统（Hinge、Hinge、BallSocket）。
    public DistanceLimit DistanceLimit { get; private set; }

    // 使用简单的 HingeJoint 将齿轮固定到世界（NullBody）
    public HingeJoint HingeJoint1 { get; private set; }
    public HingeJoint HingeJoint2 { get; private set; }

    public RigidBody Body1 { get; }
    public RigidBody Body2 { get; }

    private readonly JQuaternion initialOrientation1;
    private readonly JQuaternion initialOrientation2;

    public JVector ContactPoint { get; }

    private readonly JVector localAxis1;
    private readonly JVector localAxis2;

    private readonly World world;

    private float prevAngle1, prevAngle2;

    public float GearRatio { get; private set; }

    // 完整旋转的累加器
    public int Rotations1 { get; private set; }
    public int Rotations2 { get; private set; }

    public GearCoupling(World world, RigidBody body1, RigidBody body2,
        JVector rotationAxis1, JVector rotationAxis2, JVector contactPoint)
    {
        JVector.NormalizeInPlace(ref rotationAxis1);
        JVector.NormalizeInPlace(ref rotationAxis2);

        this.Body1 = body1;
        this.Body2 = body2;
        this.world = world;
        this.ContactPoint = contactPoint;

        // 1. 存储局部轴
        // 我们必须把轴存储为相对于每个刚体初始朝向的形式
        localAxis1 = JVector.ConjugatedTransform(rotationAxis1, body1.Orientation);
        localAxis2 = JVector.ConjugatedTransform(rotationAxis2, body2.Orientation);

        // 2. 设置世界铰链（将齿轮固定在原位）
        HingeJoint1 = new HingeJoint(world, world.NullBody, body1, body1.Position, rotationAxis1, AngularLimit.Full);
        HingeJoint2 = new HingeJoint(world, world.NullBody, body2, body2.Position, rotationAxis2, AngularLimit.Full);

        // 让铰链保持刚性，这样齿轮就不会晃动
        HingeJoint1.HingeAngle.Bias = 0.3f;
        HingeJoint2.HingeAngle.Bias = 0.3f;
        HingeJoint1.HingeAngle.Softness = 0.0f;
        HingeJoint2.HingeAngle.Softness = 0.0f;

        // 3. 设置虚拟齿
        DistanceLimit = world.CreateConstraint<DistanceLimit>(body1, body2);
        DistanceLimit.Initialize(contactPoint, contactPoint, LinearLimit.Fixed);

        DistanceLimit.Bias = 0.1f;
        DistanceLimit.Softness = 0.0000f;

        initialOrientation1 = body1.Orientation;
        initialOrientation2 = body2.Orientation;

        // 根据到接触点的半径计算传动比
        // Ratio = r1 / r2
        float r1 = (body1.Position - contactPoint).Length();
        float r2 = (body2.Position - contactPoint).Length();
        this.GearRatio = r1 / r2;

        // 4. 立即初始化角度，以避免“生成时自旋”
        // （为每个齿轮传入正确的局部轴！）
        this.prevAngle1 = GetTwistAngle(body1, initialOrientation1, localAxis1);
        this.prevAngle2 = GetTwistAngle(body2, initialOrientation2, localAxis2);

        world.PreSubStep += OnPreStep;
    }

    /// <summary>
    /// 计算相对于初始朝向的有符号扭转角。
    /// </summary>
    private static float GetTwistAngle(RigidBody body, JQuaternion initialOrientation, JVector localAxis)
    {
        JQuaternion q = JQuaternion.MultiplyConjugate(body.Orientation, initialOrientation);
        JVector ax = JVector.Transform(localAxis, body.Orientation);

        float y = JVector.Dot(q.Vector, ax);
        float x = q.Scalar;

        // 双重覆盖检查
        if (x < 0)
        {
            x = -x;
            y = -y;
        }

        return 2.0f * MathF.Atan2(y, x);
    }

    public float GearAngle1 => GetTwistAngle(Body1, initialOrientation1, localAxis1);
    public float GearAngle2 => GetTwistAngle(Body2, initialOrientation2, localAxis2);

    public float TrackDeltaAngle()
    {
        float angle1 = GearAngle1;
        float angle2 = GearAngle2;

        float deltaAngle1 = angle1 - prevAngle1;
        float deltaAngle2 = angle2 - prevAngle2;

        // 相位展开
        if (MathF.Abs(deltaAngle1) > MathF.PI)
            Rotations1 -= MathF.Sign(deltaAngle1);

        if (MathF.Abs(deltaAngle2) > MathF.PI)
            Rotations2 -= MathF.Sign(deltaAngle2);

        prevAngle1 = angle1;
        prevAngle2 = angle2;

        float totRot1 = Rotations1 * MathF.PI * 2.0f + angle1;
        float totRot2 = Rotations2 * MathF.PI * 2.0f + angle2;

        // 约束：Theta2 - Ratio * Theta1 = 0
        // 返回误差（delta）
        return totRot2 + GearRatio * totRot1; // 注意：如果齿轮反向旋转，通常用 '+'！
    }

    private void OnPreStep(TimeStep timeStep)
    {
        // 1. 计算误差
        float error = TrackDeltaAngle();

        // 可选：钳制误差，以防物理严重异常时发生爆炸
        error = Math.Clamp(error, -0.1f, 0.1f);

        // 2. 计算切线方向
        // 切线是轮齿在接触点处运动的方向。
        // 我们用车轴与半径的叉积来近似它。

        JVector ax1 = JVector.Transform(localAxis1, Body1.Orientation);
        JVector r = ContactPoint - Body1.Position;

        // 接触点处的切线方向
        JVector tangent = JVector.Normalize(JVector.Cross(ax1, r));

        // 3. 移动锚点（“虚拟齿”）
        // 我们沿切线方向移动锚点，以补偿角度误差。
        // 系数 '0.5' 把修正量分摊到两个刚体上。
        // 'r2'（半径）充当力臂转换器（角度 -> 弧长）。
        float radius2 = (Body2.Position - ContactPoint).Length();
        JVector offset = tangent * (error * radius2 * 0.5f);

        DistanceLimit.Anchor1 = ContactPoint + offset;
        DistanceLimit.Anchor2 = ContactPoint - offset;
    }

    public void Remove()
    {
        world.PreSubStep -= OnPreStep;
    }
}
