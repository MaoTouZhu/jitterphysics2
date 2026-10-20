/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Constraints;

/// <summary>
/// 使用 <see cref="Constraints.FixedAngle"/> 约束锁定朝向、使用 <see cref="BallSocket"/> 约束
/// 锁定位置，在两个刚体之间创建刚性焊接关节。这会有效消除相连刚体之间的
/// 所有相对运动。
/// </summary>
public class WeldJoint : Joint
{
    public RigidBody Body1 { get; private set; }
    public RigidBody Body2 { get; private set; }

    public FixedAngle FixedAngle { get; }
    public BallSocket BallSocket { get; }

    /// <summary>
    /// 初始化新的焊接关节。
    /// </summary>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="center"/> 包含非有限值、任一刚体不属于 <paramref name="world"/>，
    /// 或两个刚体引用相同时抛出。
    /// </exception>
    public WeldJoint(World world, RigidBody body1, RigidBody body2, JVector center)
    {
        Body1 = body1;
        Body2 = body2;

        FixedAngle = world.CreateConstraint<FixedAngle>(body1, body2);
        FixedAngle.Initialize();
        Register(FixedAngle);

        BallSocket = world.CreateConstraint<BallSocket>(body1, body2);
        BallSocket.Initialize(center);
        Register(BallSocket);
    }
}
