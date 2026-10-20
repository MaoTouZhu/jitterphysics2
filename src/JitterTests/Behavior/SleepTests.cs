namespace JitterTests.Behavior;

public class SleepTests
{
    [TestCase]
    public void StaticBodySleepingAndWake()
    {
        var world = new World();

        // 创建一个静态刚体
        var body = world.CreateRigidBody();
        body.AddShape(new BoxShape(1));
        body.MotionType = MotionType.Static;
        body.DeactivationTime = TimeSpan.FromSeconds(1);

        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);
        Assert.That(!body.IsActive, "Static body should always be inactive.");

        body.SetActivationState(true);
        Assert.That(!body.IsActive, "Static body should always be inactive.");
        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);
        Assert.That(!body.IsActive, "Static body should always be inactive.");

        body.Position = new JVector(0, 10, 0);
        Assert.That(!body.IsActive, "Static body should always be inactive.");
        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);
        Assert.That(!body.IsActive, "Static body should always be inactive.");

        var dynamicBody = world.CreateRigidBody();
        dynamicBody.AddShape(new BoxShape(1));
        dynamicBody.Position = new JVector(0, 12, 0);
        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);
        Assert.That(!body.IsActive, "Static body should always be inactive.");

        world.Dispose();
    }

    [TestCase]
    public void DeactivationTime()
    {
        var world = new World();
        world.Gravity = JVector.Zero;

        var body = world.CreateRigidBody();
        body.DeactivationTime = TimeSpan.FromSeconds(3);

        // 刚体刚创建时总是处于活动状态。
        Assert.That(body.IsActive);

        // 将模拟推进 1 秒——此时仍应处于活动状态，
        // 因为停用超时（3 秒）尚未结束。
        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);
        Assert.That(body.IsActive);

        // 再过 3 秒（共 4 秒），刚体应停用，
        // 因为它没有移动，且其超时为 3 秒。
        Helper.AdvanceWorld(world, 3, (Real)(1.0 / 100.0), false);
        Assert.That(!body.IsActive);

        // 将停用超时改为 5 秒，并尝试
        // 手动重新激活刚体。激活请求
        // 只会在下一个模拟步之后生效。
        body.DeactivationTime = TimeSpan.FromSeconds(5);
        body.SetActivationState(true);

        // 在下一步之前，刚体保持停用状态。
        Assert.That(!body.IsActive);

        // 将世界步进一次——现在激活标志被处理，
        // 刚体应重新变为活动状态。
        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);
        Assert.That(body.IsActive);

        // 推进 3 秒——仍在 5 秒超时之内，因此保持活动。
        Helper.AdvanceWorld(world, 3, (Real)(1.0 / 100.0), false);
        Assert.That(body.IsActive);

        // 再推进 2 秒（共 6 秒）——超过超时时间，
        // 因此刚体应再次停用。
        Helper.AdvanceWorld(world, 2, (Real)(1.0 / 100.0), false);
        Assert.That(!body.IsActive);

        world.Dispose();
    }

    [TestCase]
    public void KinematicTriggerWake()
    {
        var world = new World();
        world.Gravity = JVector.Zero;

        // 静态（无限质量）刚体，充当移动平台。
        var platform = world.CreateRigidBody();
        platform.AddShape(new BoxShape(2));
        platform.MotionType = MotionType.Kinematic;
        platform.Position = new JVector(0, 0, 0);
        platform.DeactivationTime = TimeSpan.FromSeconds(1);

        // 略微静止在平台上方的动态刚体。
        var box = world.CreateRigidBody();
        box.AddShape(new BoxShape(1));
        box.Position = new JVector(0, 2, 0);
        box.DeactivationTime = TimeSpan.FromSeconds(1);

        // 让动态刚体稳定下来并进入休眠。
        Helper.AdvanceWorld(world, 3, (Real)(1.0 / 100.0), false);
        Assert.That(!box.IsActive, "Box should be inactive after settling.");

        // 给静态刚体一个向上的速度——模拟移动平台。
        platform.Velocity = new JVector(0, 1, 0);

        // 推进一个步，以处理运动并生成新的接触。
        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);

        // 休眠的盒子应因与移动平台接触而醒来。
        Assert.That(box.IsActive, "Box should wake up after being touched by moving static body.");

        // 可选地验证平台即使为静态（无限质量）也仍能移动。
        Assert.That(platform.Position.Y > 0, "Platform should have moved due to integration.");

        world.Dispose();
    }


    [TestCase]
    public void StackedSleeping()
    {
        var world = new World();

        List<RigidBody> bodies = new();

        for (int i = 0; i < 3; i++)
        {
            var body = world.CreateRigidBody();
            body.AddShape(new BoxShape(1));
            body.Position = new JVector(0, i, 0);
            body.DeactivationTime = TimeSpan.FromSeconds(1);
            bodies.Add(body);
        }

        bodies[0].MotionType = MotionType.Static;
        Helper.AdvanceWorld(world, 3, (Real)(1.0 / 100.0), false);

        // 确保所有刚体都处于停用状态
        foreach (var body in bodies)
        {
            Assert.That(!body.IsActive, $"Body at {body.Position} should be inactive.");
        }

        // 请求重新激活中间的刚体
        bodies[1].SetActivationState(true);

        // 在下一步中处理激活
        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);

        // 现在中间和顶部的刚体都应处于活动状态
        Assert.That(bodies[1].IsActive, "Middle body should be reactivated.");
        Assert.That(bodies[2].IsActive, "Top body should be reactivated due to contact propagation.");

        // 推进世界，让它们再次进入休眠
        Helper.AdvanceWorld(world, 2, (Real)(1.0 / 100.0), false);

        // 验证两个动态刚体再次变为停用状态
        Assert.That(!bodies[1].IsActive, "Middle body should be inactive again.");
        Assert.That(!bodies[2].IsActive, "Top body should be inactive again.");

        world.Dispose();
    }

    [TestCase]
    public void StackedWakeOnRemoval()
    {
        var world = new World();

        List<RigidBody> bodies = new();

        // 创建一个由三个盒子组成的竖直堆叠
        for (int i = 0; i < 3; i++)
        {
            var body = world.CreateRigidBody();
            body.AddShape(new BoxShape(1));
            body.Position = new JVector(0, i, 0);
            body.DeactivationTime = TimeSpan.FromSeconds(1);
            bodies.Add(body);
        }

        // 底部的盒子为静态（充当底板）
        bodies[0].MotionType = MotionType.Static;

        // 让它们稳定下来并进入休眠
        Helper.AdvanceWorld(world, 3, (Real)(1.0 / 100.0), false);

        // 确保所有非静态刚体都处于停用状态
        foreach (var body in bodies)
        {
            Assert.That(!body.IsActive, $"Body at {body.Position} should be inactive.");
        }

        // 移除中间的刚体——这会断开底部与顶部之间的接触
        world.Remove(bodies[1]);

        // 移除之后，岛应在下一步中更新
        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);

        // 顶部刚体现在应处于活动状态，因为其支撑接触已消失
        Assert.That(bodies[2].IsActive, "Top body should wake up after its support is removed.");

        world.Dispose();
    }

    [TestCase]
    public void StaticBaseMovementWakesDynamic()
    {
        var world = new World();

        // 底部刚体：静态（无限质量）底座
        var bottom = world.CreateRigidBody();
        bottom.AddShape(new BoxShape(2));
        bottom.Position = new JVector(0, 0, 0);
        bottom.MotionType = MotionType.Static;
        bottom.DeactivationTime = TimeSpan.FromSeconds(1);

        // 顶部刚体：静止在静态底座上的动态盒子
        var top = world.CreateRigidBody();
        top.AddShape(new BoxShape(1));
        top.Position = new JVector(0, 2, 0);
        top.DeactivationTime = TimeSpan.FromSeconds(1);

        // 让它们稳定下来——两者都应进入休眠
        Helper.AdvanceWorld(world, 3, (Real)(1.0 / 100.0), false);
        Assert.That(!bottom.IsActive, "Bottom (static) body should be inactive after settling.");
        Assert.That(!top.IsActive, "Top body should be inactive after settling.");

        // 将静态刚体略微向下移动
        bottom.Position -= new JVector(0, 0.5f, 0);

        // 处理一步，使接触系统更新
        Helper.AdvanceWorld(world, 1, (Real)(1.0 / 100.0), false);

        // 顶部刚体现在应醒来，因为其接触受到了扰动
        Assert.That(top.IsActive, "Top body should wake up after its static support moves.");

        world.Dispose();
    }

    [TestCase]
    public void ForceSleepIsland_DeactivatesWholeIslandImmediately()
    {
        using var world = new World();
        world.Gravity = JVector.Zero;

        var body1 = world.CreateRigidBody();
        body1.AddShape(new SphereShape(1));
        body1.Velocity = new JVector(1, 0, 0);

        var body2 = world.CreateRigidBody();
        body2.AddShape(new SphereShape(1));
        body2.Position = new JVector(2, 0, 0);
        body2.Velocity = new JVector(-1, 0, 0);

        var constraint = world.CreateConstraint<BallSocket>(body1, body2);
        constraint.Initialize(JVector.Zero);

        Island island = body1.Island;
        Assert.That(body2.Island, Is.SameAs(island));
        Assert.That(world.Islands.IsActive(island), Is.True);

        int deactivated = 0;
        bool callbackSawInactiveIsland = false;

        world.IslandDeactivated += deactivatedIsland =>
        {
            deactivated++;
            callbackSawInactiveIsland = !world.Islands.IsActive(deactivatedIsland);
        };

        world.ForceSleepIsland(island);

        Assert.That(deactivated, Is.EqualTo(1));
        Assert.That(callbackSawInactiveIsland, Is.True);
        Assert.That(world.Islands.IsActive(island), Is.False);
        Assert.That(world.RigidBodies.IsActive(body1), Is.False);
        Assert.That(world.RigidBodies.IsActive(body2), Is.False);
        Assert.That(body1.IsActive, Is.False);
        Assert.That(body2.IsActive, Is.False);
        Assert.That(body1.Velocity, Is.EqualTo(JVector.Zero));
        Assert.That(body2.Velocity, Is.EqualTo(JVector.Zero));
    }

    [Test]
    public void MovingKinematicBodyStaysAwake(
        [Values] bool angular,
        [Values(0.2, 1e-7, 1e-25)] double speed,
        [Values(0.0, 0.5)] double deactivationSeconds)
    {
        using var world = new World { Gravity = JVector.Zero };
        var body = world.CreateRigidBody();
        body.AddShape(new BoxShape(1));
        body.MotionType = MotionType.Kinematic;
        body.DeactivationTime = TimeSpan.FromSeconds(deactivationSeconds);

        var velocity = angular ? new JVector(0, (Real)speed, 0) : new JVector((Real)speed, 0, 0);
        if (angular) body.AngularVelocity = velocity;
        else body.Velocity = velocity;

        Helper.AdvanceWorld(world, 3, (Real)0.01, false);

        Assert.That(body.IsActive, Is.True);
        Assert.That(angular ? body.AngularVelocity : body.Velocity, Is.EqualTo(velocity));
        Real displacement;
        if (angular)
        {
            JVector forward = JVector.Transform(JVector.UnitX, body.Orientation);
            displacement = MathR.Atan2(-forward.Z, forward.X);
        }
        else
        {
            displacement = body.Position.X;
        }

        Assert.That(displacement, Is.EqualTo((Real)(3 * speed)).Within((Real)(0.05 * speed)));
    }

    [Test]
    public void NonzeroVelocityWakesSleepingKinematicBody(
        [Values] bool angular,
        [Values(1e-7, 1e-9, 1e-25)] double speed)
    {
        using var world = new World { Gravity = JVector.Zero };
        var body = world.CreateRigidBody();
        body.AddShape(new BoxShape(1));
        body.MotionType = MotionType.Kinematic;
        body.DeactivationTime = TimeSpan.FromSeconds(0.1);
        Helper.AdvanceWorld(world, 1, (Real)0.01, false);
        Assert.That(body.IsActive, Is.False);

        var velocity = new JVector((Real)speed, 0, 0);
        if (angular) body.AngularVelocity = velocity;
        else body.Velocity = velocity;

        world.Step((Real)0.01, false);
        Assert.That(body.IsActive, Is.True);
        Helper.AdvanceWorld(world, 1, (Real)0.01, false);
        Assert.That(body.IsActive, Is.True);
        Assert.That(angular ? body.AngularVelocity : body.Velocity, Is.EqualTo(velocity));
    }

    [Test]
    public void KinematicBodySleepsAfterMotionStops([Values] bool angular)
    {
        using var world = new World { Gravity = JVector.Zero };
        var body = world.CreateRigidBody();
        body.AddShape(new BoxShape(1));
        body.MotionType = MotionType.Kinematic;
        body.DeactivationTime = TimeSpan.FromSeconds(0.1);

        if (angular) body.AngularVelocity = new JVector(0, (Real)0.2, 0);
        else body.Velocity = new JVector((Real)0.2, 0, 0);
        Helper.AdvanceWorld(world, 1, (Real)0.01, false);
        Assert.That(body.IsActive, Is.True);

        if (angular) body.AngularVelocity = JVector.Zero;
        else body.Velocity = JVector.Zero;
        Helper.AdvanceWorld(world, 1, (Real)0.01, false);
        Assert.That(body.IsActive, Is.False);
    }

    [Test]
    public void SwitchingToKinematicPreservesMotionAndStaysAwake()
    {
        using var world = new World { Gravity = JVector.Zero };
        var body = world.CreateRigidBody();
        body.AddShape(new BoxShape(1));
        body.DeactivationTime = TimeSpan.FromSeconds(0.1);
        body.Velocity = new JVector((Real)0.2, 0, 0);
        body.AngularVelocity = new JVector(0, (Real)0.2, 0);
        world.Step((Real)0.01, false);
        JVector velocity = body.Velocity;
        JVector angularVelocity = body.AngularVelocity;

        body.MotionType = MotionType.Kinematic;
        Helper.AdvanceWorld(world, 1, (Real)0.01, false);

        Assert.That(body.IsActive, Is.True);
        Assert.That(body.Velocity, Is.EqualTo(velocity));
        Assert.That(body.AngularVelocity, Is.EqualTo(angularVelocity));
    }

    [Test]
    public void LockedKinematicMotionDoesNotWakeBody()
    {
        using var world = new World { Gravity = JVector.Zero };
        var body = world.CreateRigidBody();
        body.AddShape(new BoxShape(1));
        body.MotionType = MotionType.Kinematic;
        body.AllowedMotion = MotionAxes.None;
        body.DeactivationTime = TimeSpan.FromSeconds(0.1);
        Helper.AdvanceWorld(world, 1, (Real)0.01, false);
        Assert.That(body.IsActive, Is.False);

        body.Velocity = new JVector((Real)0.2, 0, 0);
        body.AngularVelocity = new JVector(0, (Real)0.2, 0);
        world.Step((Real)0.01, false);

        Assert.That(body.Velocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.AngularVelocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.IsActive, Is.False);
    }

    [Test]
    public void SlowKinematicPlatformCarriesItsRider()
    {
        using var world = new World();
        var platform = world.CreateRigidBody();
        platform.AddShape(new BoxShape(4, 1, 4));
        platform.MotionType = MotionType.Kinematic;
        platform.DeactivationTime = TimeSpan.FromSeconds(0.5);

        var rider = world.CreateRigidBody();
        rider.AddShape(new BoxShape(1));
        rider.Position = new JVector(0, 1, 0);
        rider.DeactivationTime = TimeSpan.FromSeconds(0.5);
        Helper.AdvanceWorld(world, 1, (Real)0.01, false);
        Real gap = rider.Position.Y - platform.Position.Y;

        platform.Velocity = new JVector(0, (Real)0.1, 0);
        Helper.AdvanceWorld(world, 4, (Real)0.01, false);

        Assert.That(platform.Position.Y, Is.EqualTo((Real)0.4).Within((Real)0.02));
        Assert.That(rider.Position.Y - platform.Position.Y, Is.EqualTo(gap).Within((Real)0.02));
    }

    [TestCase]
    public void ForceSleepIsland_ZeroesKinematicVelocityImmediately()
    {
        using var world = new World();

        var body = world.CreateRigidBody();
        body.AddShape(new BoxShape(1));
        body.MotionType = MotionType.Kinematic;
        body.Velocity = new JVector(1, 2, 3);
        body.AngularVelocity = new JVector(4, 5, 6);
        body.Force = new JVector(7, 8, 9);
        body.Torque = new JVector(10, 11, 12);

        world.ForceSleepIsland(body.Island);

        Assert.That(body.IsActive, Is.False);
        Assert.That(world.Islands.IsActive(body.Island), Is.False);
        Assert.That(body.Velocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.AngularVelocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.Force, Is.EqualTo(JVector.Zero));
        Assert.That(body.Torque, Is.EqualTo(JVector.Zero));
    }

    [TestCase]
    public void ForceSleepIsland_WithForeignIsland_Throws()
    {
        using var world = new World();
        using var otherWorld = new World();

        var foreignBody = otherWorld.CreateRigidBody();

        Assert.Throws<ArgumentException>(() => world.ForceSleepIsland(foreignBody.Island));
    }

    [TestCase]
    public void IslandDeactivatedEventIsRaisedAfterIslandSleeps()
    {
        var world = new World();
        world.Gravity = JVector.Zero;

        var body = world.CreateRigidBody();
        body.AddShape(new BoxShape(1));
        body.DeactivationTime = TimeSpan.FromSeconds(1);

        int deactivated = 0;
        bool callbackSawBody = false;
        bool callbackSawInactiveBody = false;
        bool callbackSawInactivePartition = false;

        world.IslandDeactivated += island =>
        {
            deactivated++;
            callbackSawBody = island.Bodies.Contains(body);
            callbackSawInactiveBody = !body.IsActive;
            callbackSawInactivePartition = !world.Islands.IsActive(island);
        };

        Helper.AdvanceWorld(world, 2, (Real)(1.0 / 100.0), false);

        Assert.That(deactivated, Is.EqualTo(1));
        Assert.That(callbackSawBody, Is.True);
        Assert.That(callbackSawInactiveBody, Is.True);
        Assert.That(callbackSawInactivePartition, Is.True);

        world.Dispose();
    }

    [TestCase]
    public void IslandActivatedEventIsRaisedAfterSleepingIslandWakes()
    {
        var world = new World();
        world.Gravity = JVector.Zero;

        var body = world.CreateRigidBody();
        body.AddShape(new BoxShape(1));
        body.DeactivationTime = TimeSpan.FromSeconds(1);

        Helper.AdvanceWorld(world, 2, (Real)(1.0 / 100.0), false);
        Assert.That(body.IsActive, Is.False);

        int activated = 0;
        bool callbackSawBody = false;
        bool callbackSawActiveBody = false;
        bool callbackSawActivePartition = false;

        world.IslandActivated += island =>
        {
            activated++;
            callbackSawBody = island.Bodies.Contains(body);
            callbackSawActiveBody = body.IsActive;
            callbackSawActivePartition = world.Islands.IsActive(island);
        };

        body.SetActivationState(true);
        world.Step((Real)(1.0 / 100.0), false);

        Assert.That(activated, Is.EqualTo(1));
        Assert.That(callbackSawBody, Is.True);
        Assert.That(callbackSawActiveBody, Is.True);
        Assert.That(callbackSawActivePartition, Is.True);

        world.Dispose();
    }

}
