/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Constraints;

/// <summary>
/// 使用 <see cref="HingeAngle"/> 和 <see cref="BallSocket"/> 构造一个铰链，
/// 并可选地附带 <see cref="AngularMotor"/> 或 <see cref="TwistServo"/>。
/// </summary>
public class HingeJoint : Joint
{
    public RigidBody Body1 { get; private set; }
    public RigidBody Body2 { get; private set; }

    public HingeAngle HingeAngle { get; }
    public BallSocket BallSocket { get; }
    public AngularMotor? Motor { get; }
    public TwistServo? Servo { get; }

    /// <inheritdoc cref="HingeJoint(World, RigidBody, RigidBody, JVector, JVector, AngularLimit, JointDrive)"/>
    public HingeJoint(World world, RigidBody body1, RigidBody body2, JVector hingeCenter, JVector hingeAxis,
        JointDrive drive = JointDrive.None) :
        this(world, body1, body2, hingeCenter, hingeAxis, AngularLimit.Full, drive)
    {
    }

    /// <summary>
    /// 初始化一个新的铰链关节。
    /// </summary>
    /// <param name="drive">可选的驱动。马达和伺服的力矩默认为零。</param>
    /// <exception cref="ArgumentOutOfRangeException">驱动选择无效。</exception>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="hingeCenter"/> 包含非有限值时，当 <paramref name="hingeAxis"/> 为零或包含非有限值时，
    /// 当任一刚体不属于 <paramref name="world"/> 时，或当两个刚体引用相同时抛出。
    /// </exception>
    public HingeJoint(World world, RigidBody body1, RigidBody body2, JVector hingeCenter, JVector hingeAxis,
        AngularLimit angle, JointDrive drive = JointDrive.None)
    {
        Body1 = body1;
        Body2 = body2;

        ArgumentCheck.Finite(hingeCenter, nameof(hingeCenter));
        ArgumentCheck.NonZero(hingeAxis, nameof(hingeAxis));
        ArgumentCheck.Finite(angle.From, nameof(angle));
        ArgumentCheck.Finite(angle.To, nameof(angle));
        if (drive is not (JointDrive.None or JointDrive.Motor or JointDrive.Servo))
            throw new ArgumentOutOfRangeException(nameof(drive));

        JVector.NormalizeInPlace(ref hingeAxis);

        HingeAngle = world.CreateConstraint<HingeAngle>(body1, body2);
        HingeAngle.Initialize(hingeAxis, angle);
        Register(HingeAngle);

        BallSocket = world.CreateConstraint<BallSocket>(body1, body2);
        BallSocket.Initialize(hingeCenter);
        Register(BallSocket);

        if (drive == JointDrive.Motor)
        {
            Motor = world.CreateConstraint<AngularMotor>(body1, body2);
            Motor.Initialize(hingeAxis);
            Register(Motor);
        }
        else if (drive == JointDrive.Servo)
        {
            Servo = world.CreateConstraint<TwistServo>(body1, body2);
            Servo.Initialize(hingeAxis);
            Register(Servo);
        }
    }

    /// <inheritdoc cref="HingeJoint(World, RigidBody, RigidBody, JVector, JVector, AngularLimit, JointDrive)"/>
    [Obsolete("Use the overload accepting JointDrive instead of hasMotor.")]
    public HingeJoint(World world, RigidBody body1, RigidBody body2, JVector hingeCenter, JVector hingeAxis,
        AngularLimit angle, bool hasMotor) :
        this(world, body1, body2, hingeCenter, hingeAxis, angle, hasMotor ? JointDrive.Motor : JointDrive.None)
    {
    }

    /// <inheritdoc cref="HingeJoint(World, RigidBody, RigidBody, JVector, JVector, AngularLimit, JointDrive)"/>
    [Obsolete("Use the overload accepting JointDrive instead of hasMotor.")]
    public HingeJoint(World world, RigidBody body1, RigidBody body2, JVector hingeCenter, JVector hingeAxis,
        bool hasMotor) :
        this(world, body1, body2, hingeCenter, hingeAxis, AngularLimit.Full,
            hasMotor ? JointDrive.Motor : JointDrive.None)
    {
    }
}
