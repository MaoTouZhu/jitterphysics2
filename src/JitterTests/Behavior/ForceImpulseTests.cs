namespace JitterTests.Behavior;

/// <summary>
/// 测试 AddForce 与 ApplyImpulse 的行为。
/// </summary>
public class ForceImpulseTests
{
    // -------------------------------------------------------------------------
    // ApplyImpulse — 立即改变速度，无需步进
    // -------------------------------------------------------------------------

    [TestCase]
    public void ApplyImpulse_ChangesVelocityImmediately()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.ApplyImpulse(new JVector(1, 0, 0));
        Assert.That(body.Velocity.X, Is.GreaterThan(0));
        world.Dispose();
    }

    [TestCase]
    public void ApplyImpulse_ScaledByInverseMass()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        var impulse = new JVector(10, 0, 0);
        body.ApplyImpulse(impulse);
        var expectedVelocityX = impulse.X / body.Mass;
        Assert.That(body.Velocity.X, Is.EqualTo(expectedVelocityX).Within(1e-4f));
        world.Dispose();
    }

    [TestCase]
    public void ApplyImpulse_WithPosition_AlsoChangesAngularVelocity()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        // 在偏离中心处施加冲量以产生角速度
        body.ApplyImpulse(new JVector(0, 1, 0), new JVector(1, 0, 0));
        Assert.That(body.AngularVelocity.LengthSquared(), Is.GreaterThan(0));
        world.Dispose();
    }

    [TestCase]
    public void ApplyImpulse_OnKinematic_HasNoEffect()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.MotionType = MotionType.Kinematic;
        body.ApplyImpulse(new JVector(1, 0, 0));
        Assert.That(body.Velocity, Is.EqualTo(JVector.Zero));
        world.Dispose();
    }

    [TestCase]
    public void ApplyImpulse_OnStatic_HasNoEffect()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.MotionType = MotionType.Static;
        body.ApplyImpulse(new JVector(1, 0, 0));
        // 静态刚体的 InverseMassVector 为零；速度保持为零
        Assert.That(body.Velocity, Is.EqualTo(JVector.Zero));
        world.Dispose();
    }

    [TestCase]
    public void ApplyImpulse_ZeroImpulse_HasNoEffect()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.ApplyImpulse(JVector.Zero);
        Assert.That(body.Velocity, Is.EqualTo(JVector.Zero));
        world.Dispose();
    }

    // -------------------------------------------------------------------------
    // AddForce — 累积并在下一步中应用
    // -------------------------------------------------------------------------

    [TestCase]
    public void AddForce_AccumulatesInForceProperty()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        var force = new JVector(10, 0, 0);
        body.AddForce(force);
        Assert.That(body.Force.X, Is.EqualTo(force.X).Within(1e-5f));
        world.Dispose();
    }

    [TestCase]
    public void AddForce_Accumulates_MultipleCalls()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.AddForce(new JVector(1, 0, 0));
        body.AddForce(new JVector(2, 0, 0));
        Assert.That(body.Force.X, Is.EqualTo(3).Within(1e-5f));
        world.Dispose();
    }

    [TestCase]
    public void AddForce_IsResetAfterStep()
    {
        var world = new World();
        world.AllowDeactivation = false;
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.AddForce(new JVector(100, 0, 0));
        world.Step(1f / 60f, false);
        Assert.That(body.Force, Is.EqualTo(JVector.Zero));
        world.Dispose();
    }

    [TestCase]
    public void AddForce_ChangesVelocityAfterStep()
    {
        // 在第 N 步之前施加的力会在第 N 步结束时由 UpdateBodies 转换为 DeltaVelocity，
        // 并在第 N+1 步开始时应用到 Velocity。
        // 因此需要两步才能观察到速度变化。
        var world = new World();
        world.AllowDeactivation = false;
        world.Gravity = JVector.Zero;
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        world.Step(1f / 60f, false);   // 预热：刚体处于活动状态，尚未施加力
        body.AddForce(new JVector(100, 0, 0));
        world.Step(1f / 60f, false);   // 由 Force 计算 DeltaVelocity
        world.Step(1f / 60f, false);   // 将 DeltaVelocity 应用到 Velocity
        Assert.That(body.Velocity.X, Is.GreaterThan(0));
        world.Dispose();
    }

    [TestCase]
    public void MotionTypeChange_ClearsQueuedForceAndTorque()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));

        body.AddForce(new JVector(0, 10, 0), new JVector(1, 0, 0));

        Assert.That(body.Force.LengthSquared(), Is.GreaterThan((Real)0.0));
        Assert.That(body.Torque.LengthSquared(), Is.GreaterThan((Real)0.0));

        body.MotionType = MotionType.Kinematic;

        Assert.That(body.Force, Is.EqualTo(JVector.Zero));
        Assert.That(body.Torque, Is.EqualTo(JVector.Zero));
        Assert.That(body.Data.DeltaVelocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.Data.DeltaAngularVelocity, Is.EqualTo(JVector.Zero));

        world.Dispose();
    }

    [TestCase]
    public void MotionTypeChange_ClearsForceDerivedVelocityDelta()
    {
        var world = new World
        {
            AllowDeactivation = false,
            Gravity = JVector.Zero
        };

        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.AddForce(new JVector(0, 100, 0), new JVector(1, 0, 0));

        world.Step((Real)1.0 / (Real)60.0, false);

        Assert.That(body.Velocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.AngularVelocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.Data.DeltaVelocity.LengthSquared(), Is.GreaterThan((Real)0.0));
        Assert.That(body.Data.DeltaAngularVelocity.LengthSquared(), Is.GreaterThan((Real)0.0));

        body.MotionType = MotionType.Kinematic;

        Assert.That(body.Data.DeltaVelocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.Data.DeltaAngularVelocity, Is.EqualTo(JVector.Zero));

        body.MotionType = MotionType.Dynamic;
        world.Step((Real)1.0 / (Real)60.0, false);

        Assert.That(body.Velocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.AngularVelocity, Is.EqualTo(JVector.Zero));

        world.Dispose();
    }

    [TestCase]
    public void AddForce_OnKinematic_HasNoEffect()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.MotionType = MotionType.Kinematic;
        body.AddForce(new JVector(100, 0, 0));
        Assert.That(body.Force, Is.EqualTo(JVector.Zero));
        world.Dispose();
    }

    [TestCase]
    public void AddForce_OnStatic_HasNoEffect()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.MotionType = MotionType.Static;
        body.AddForce(new JVector(100, 0, 0));
        Assert.That(body.Force, Is.EqualTo(JVector.Zero));
        world.Dispose();
    }

    [TestCase]
    public void AddForce_WithPosition_AccumulatesTorque()
    {
        var world = new World();
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        // 在偏离中心处施加力应产生力矩
        body.AddForce(new JVector(0, 10, 0), new JVector(1, 0, 0));
        Assert.That(body.Torque.LengthSquared(), Is.GreaterThan(0));
        world.Dispose();
    }

    [TestCase]
    public void AddForce_SleepingBody_WakeupFalse_HasNoEffect()
    {
        var world = new World();
        world.Gravity = JVector.Zero;
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.DeactivationTime = TimeSpan.FromSeconds(1);

        Helper.AdvanceWorld(world, 2, 1f / 100f, false);
        Assert.That(body.IsActive, Is.False);

        body.AddForce(new JVector(10, 0, 0), wakeup: false);

        Assert.That(body.Force, Is.EqualTo(JVector.Zero));
        Assert.That(body.IsActive, Is.False);
        world.Dispose();
    }

    [TestCase]
    public void AddForce_SleepingBody_WakeupTrue_QueuesForceAndReactivatesNextStep()
    {
        var world = new World();
        world.Gravity = JVector.Zero;
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.DeactivationTime = TimeSpan.FromSeconds(1);

        Helper.AdvanceWorld(world, 2, 1f / 100f, false);
        Assert.That(body.IsActive, Is.False);

        body.AddForce(new JVector(10, 0, 0), wakeup: true);

        Assert.That(body.Force.X, Is.EqualTo(10).Within(1e-6f));
        Assert.That(body.IsActive, Is.False);

        world.Step(1f / 100f, false);
        Assert.That(body.IsActive, Is.True);
        world.Dispose();
    }

    [TestCase]
    public void ApplyImpulse_SleepingBody_WakeupFalse_HasNoEffect()
    {
        var world = new World();
        world.Gravity = JVector.Zero;
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.DeactivationTime = TimeSpan.FromSeconds(1);

        Helper.AdvanceWorld(world, 2, 1f / 100f, false);
        Assert.That(body.IsActive, Is.False);

        body.ApplyImpulse(new JVector(10, 0, 0), wakeup: false);

        Assert.That(body.Velocity, Is.EqualTo(JVector.Zero));
        Assert.That(body.IsActive, Is.False);
        world.Dispose();
    }

    [TestCase]
    public void ApplyImpulse_SleepingBody_WakeupTrue_ChangesVelocityAndReactivatesNextStep()
    {
        var world = new World();
        world.Gravity = JVector.Zero;
        var body = world.CreateRigidBody();
        body.AddShape(new SphereShape(1));
        body.DeactivationTime = TimeSpan.FromSeconds(1);

        Helper.AdvanceWorld(world, 2, 1f / 100f, false);
        Assert.That(body.IsActive, Is.False);

        body.ApplyImpulse(new JVector(10, 0, 0), wakeup: true);

        Assert.That(body.Velocity.X, Is.GreaterThan(0));
        Assert.That(body.IsActive, Is.False);

        world.Step(1f / 100f, false);
        Assert.That(body.IsActive, Is.True);
        world.Dispose();
    }
}
