/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Constraints;

/// <summary>
/// 使用一个 <see cref="PointOnLine"/> 约束、一个 <see cref="FixedAngle"/> 或 <see cref="HingeAngle"/> 约束，
/// 以及一个可选的 <see cref="LinearMotor"/> 或 <see cref="LinearAxisServo"/> 驱动来构造移动关节。
/// </summary>
public class PrismaticJoint : Joint
{
    public RigidBody Body1 { get; private set; }
    public RigidBody Body2 { get; private set; }

    public PointOnLine Slider { get; }

    public FixedAngle? FixedAngle { get; }
    public HingeAngle? HingeAngle { get; }
    public LinearMotor? Motor { get; }
    public LinearAxisServo? Servo { get; }

    /// <inheritdoc cref="PrismaticJoint(World, RigidBody, RigidBody, JVector, JVector, LinearLimit, JointDrive, bool)"/>
    public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector axis) :
        this(world, body1, body2, center, axis, LinearLimit.Full, JointDrive.None)
    {
    }

    /// <inheritdoc cref="PrismaticJoint(World, RigidBody, RigidBody, JVector, JVector, LinearLimit, JointDrive, bool)"/>
    public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector axis,
        bool pinned) : this(world, body1, body2, center, axis, LinearLimit.Full, JointDrive.None, pinned)
    {
    }

    /// <inheritdoc cref="PrismaticJoint(World, RigidBody, RigidBody, JVector, JVector, LinearLimit, JointDrive, bool)"/>
    public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector axis,
        LinearLimit limit) : this(world, body1, body2, center, axis, limit, JointDrive.None)
    {
    }

    /// <inheritdoc cref="PrismaticJoint(World, RigidBody, RigidBody, JVector, JVector, LinearLimit, JointDrive, bool)"/>
    public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector axis,
        LinearLimit limit, bool pinned) : this(world, body1, body2, center, axis, limit, JointDrive.None, pinned)
    {
    }

    /// <inheritdoc cref="PrismaticJoint(World, RigidBody, RigidBody, JVector, JVector, LinearLimit, JointDrive, bool)"/>
    public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector axis,
        JointDrive drive, bool pinned = true) : this(world, body1, body2, center, axis, LinearLimit.Full, drive, pinned)
    {
    }

    /// <inheritdoc cref="PrismaticJoint(World, RigidBody, RigidBody, JVector, JVector, LinearLimit, JointDrive, bool)"/>
    [Obsolete("Use the overload accepting JointDrive instead of hasMotor.")]
    public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector axis,
        bool pinned = true, bool hasMotor = false) :
        this(world, body1, body2, center, axis, LinearLimit.Full,
            hasMotor ? JointDrive.Motor : JointDrive.None, pinned)
    {
    }

    /// <inheritdoc cref="PrismaticJoint(World, RigidBody, RigidBody, JVector, JVector, LinearLimit, JointDrive, bool)"/>
    [Obsolete("Use the overload accepting JointDrive instead of hasMotor.")]
    public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector axis, LinearLimit limit,
        bool pinned = true, bool hasMotor = false) :
        this(world, body1, body2, center, axis, limit,
            hasMotor ? JointDrive.Motor : JointDrive.None, pinned)
    {
    }

    /// <summary>
    /// 初始化一个新的移动关节。
    /// </summary>
    /// <param name="world">包含两个刚体的世界。</param>
    /// <param name="body1">第一个刚体。</param>
    /// <param name="body2">第二个刚体。</param>
    /// <param name="center">世界空间中的关节中心。</param>
    /// <param name="axis">世界空间中的滑动轴。</param>
    /// <param name="limit">沿滑动轴允许的位移。</param>
    /// <param name="pinned">若为 true，则用 <see cref="FixedAngle"/> 锁定相对旋转；否则用 <see cref="HingeAngle"/> 约束它。</param>
    /// <param name="drive">可选的驱动。马达和伺服的作用力默认为零。</param>
    /// <exception cref="ArgumentOutOfRangeException">驱动选择无效。</exception>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="center"/> 包含非有限值时，当 <paramref name="axis"/> 为零或包含非有限值时，
    /// 当任一限位值为 NaN 时，当任一刚体不属于 <paramref name="world"/> 时，或当两个刚体引用相同时抛出。
    /// </exception>
    public PrismaticJoint(World world, RigidBody body1, RigidBody body2, JVector center, JVector axis, LinearLimit limit,
        JointDrive drive, bool pinned = true)
    {
        Body1 = body1;
        Body2 = body2;

        ArgumentCheck.Finite(center, nameof(center));
        ArgumentCheck.NonZero(axis, nameof(axis));
        ArgumentCheck.NotNaN(limit.From, nameof(limit.From));
        ArgumentCheck.NotNaN(limit.To, nameof(limit.To));
        if (drive is not (JointDrive.None or JointDrive.Motor or JointDrive.Servo))
            throw new ArgumentOutOfRangeException(nameof(drive));

        JVector.NormalizeInPlace(ref axis);

        Slider = world.CreateConstraint<PointOnLine>(body1, body2);
        Slider.Initialize(axis, center, center, limit);
        Register(Slider);

        if (pinned)
        {
            FixedAngle = world.CreateConstraint<FixedAngle>(body1, body2);
            FixedAngle.Initialize();
            Register(FixedAngle);
        }
        else
        {
            HingeAngle = world.CreateConstraint<HingeAngle>(body1, body2);
            HingeAngle.Initialize(axis, AngularLimit.Full);
            Register(HingeAngle);
        }

        if (drive == JointDrive.Motor)
        {
            Motor = world.CreateConstraint<LinearMotor>(body1, body2);
            Motor.Initialize(axis, axis);
            Register(Motor);
        }
        else if (drive == JointDrive.Servo)
        {
            Servo = world.CreateConstraint<LinearAxisServo>(body1, body2);
            Servo.Initialize(axis, center, center);
            Register(Servo);
        }
    }
}
