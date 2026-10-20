/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Constraints;

/// <summary>
/// 使用 <see cref="TwistAngle"/> 和 <see cref="BallSocket"/> 创建一个万向节，
/// 并可选地附带 <see cref="AngularMotor"/> 或 <see cref="AngularServo"/>。
/// </summary>
public class UniversalJoint : Joint
{
    public RigidBody Body1 { get; private set; }
    public RigidBody Body2 { get; private set; }

    public TwistAngle TwistAngle { get; }
    public BallSocket BallSocket { get; }
    public AngularMotor? Motor { get; }
    public AngularServo? Servo { get; }

    /// <summary>
    /// 初始化一个新的万向节。
    /// </summary>
    /// <param name="drive">可选的驱动。马达和伺服的力矩默认为零。</param>
    /// <exception cref="ArgumentOutOfRangeException">驱动选择无效。</exception>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="center"/> 包含非有限值时，当任一旋转轴为零或包含非有限值时，
    /// 当任一刚体不属于 <paramref name="world"/> 时，或当两个刚体引用相同时抛出。
    /// </exception>
    public UniversalJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector rotateAxis1, JVector rotateAxis2,
        JointDrive drive = JointDrive.None)
    {
        Body1 = body1;
        Body2 = body2;

        ArgumentCheck.Finite(center, nameof(center));
        ArgumentCheck.NonZero(rotateAxis1, nameof(rotateAxis1));
        ArgumentCheck.NonZero(rotateAxis2, nameof(rotateAxis2));
        if (drive is not (JointDrive.None or JointDrive.Motor or JointDrive.Servo))
            throw new ArgumentOutOfRangeException(nameof(drive));

        JVector.NormalizeInPlace(ref rotateAxis1);
        JVector.NormalizeInPlace(ref rotateAxis2);

        TwistAngle = world.CreateConstraint<TwistAngle>(body1, body2);
        TwistAngle.Initialize(rotateAxis1, rotateAxis2);
        Register(TwistAngle);

        BallSocket = world.CreateConstraint<BallSocket>(body1, body2);
        BallSocket.Initialize(center);
        Register(BallSocket);

        if (drive == JointDrive.Motor)
        {
            Motor = world.CreateConstraint<AngularMotor>(body1, body2);
            Motor.Initialize(rotateAxis1, rotateAxis2);
            Register(Motor);
        }
        else if (drive == JointDrive.Servo)
        {
            Servo = world.CreateConstraint<AngularServo>(body1, body2);
            Servo.Initialize();
            Register(Servo);
        }
    }

    /// <inheritdoc cref="UniversalJoint(World, RigidBody, RigidBody, JVector, JVector, JVector, JointDrive)"/>
    [Obsolete("Use the overload accepting JointDrive instead of hasMotor.")]
    public UniversalJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector rotateAxis1, JVector rotateAxis2,
        bool hasMotor) :
        this(world, body1, body2, center, rotateAxis1, rotateAxis2, hasMotor ? JointDrive.Motor : JointDrive.None)
    {
    }
}
