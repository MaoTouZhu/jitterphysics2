namespace JitterTests.Constraints;

[TestFixture]
public class ConstraintBlockMassTests
{
#if USE_DOUBLE_PRECISION
    private const Real Tolerance = 1e-10;
#else
    private const Real Tolerance = 1e-4f;
#endif

    [Test]
    public void BallSocket_SingularResponseStopsAnchorMotionAndPreservesFreeRotation(
        [Values] bool irrelevantLocks, [Values] bool obliqueAnchor)
    {
        using World world = new();
        RigidBody body = world.CreateRigidBody();
        body.SetMassInertia(JSymmetricMatrix.Identity, 0, true);
        if (irrelevantLocks) body.AllowedMotion = MotionAxes.Angular;

        JVector anchor = obliqueAnchor ? JVector.Normalize(new JVector(1, 2, 3)) : JVector.UnitX;
        var joint = world.CreateConstraint<BallSocket>(world.NullBody, body);
        joint.Initialize(anchor);
        joint.Softness = joint.Bias = 0;
        JVector initial = new(3, 2, 1);
        body.AngularVelocity = initial;

        BallSocket.PrepareForIterationBallSocket(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
        BallSocket.IterateBallSocket(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));

        // 没有平移响应时，只有绕锚点方向的旋转是自由的。
        JVector expected = anchor * (anchor * initial);
        Assert.That(JVector.MaxAbs(body.AngularVelocity - expected), Is.LessThan(Tolerance));
        Assert.That((body.AngularVelocity % anchor).Length(), Is.LessThan(Tolerance));
        Assert.That(body.AngularVelocity.LengthSquared(), Is.LessThanOrEqualTo(initial.LengthSquared()));
        Assert.That(body.Velocity, Is.EqualTo(JVector.Zero));
    }

    [Test]
    public void FixedAngle_HalfTurnStillStopsResponsiveAngularMotion(
        [Values(MotionAxes.All, MotionAxes.Angular, MotionAxes.Linear | MotionAxes.AngularY | MotionAxes.AngularZ)]
        MotionAxes axes)
    {
        using World world = new();
        RigidBody body = world.CreateRigidBody();
        body.AllowedMotion = axes;
        var joint = world.CreateConstraint<FixedAngle>(world.NullBody, body);
        joint.Initialize();
        joint.Softness = joint.Bias = 0;
        body.Orientation = new JQuaternion(1, 0, 0, 0);
        body.AngularVelocity = JVector.UnitY;

        FixedAngle.PrepareForIterationFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
        FixedAngle.IterateFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));

        // 即使 X 的四元数-向量行奇异，Y 旋转仍有响应。
        Assert.That(JVector.MaxAbs(body.AngularVelocity), Is.LessThan(Tolerance));
        Assert.That(Real.IsFinite(joint.Impulse.LengthSquared()), Is.True);
    }

    [Test]
    public void BallSocket_SingularResponsePreservesIndependentWeakRow([Values] bool irrelevantLocks)
    {
        using World world = new();
        RigidBody body = world.CreateRigidBody();
        Real weakResponse = Precision.IsDoublePrecision ? (Real)1e-15 : (Real)1e-9;
        body.SetMassInertia(JSymmetricMatrix.CreateScale(0, weakResponse, 1), 0, true);
        if (irrelevantLocks) body.AllowedMotion = MotionAxes.Angular;
        var joint = world.CreateConstraint<BallSocket>(world.NullBody, body);
        joint.Initialize(JVector.UnitX);
        joint.Softness = joint.Bias = 0;
        body.AngularVelocity = new JVector(0, 2, 3);

        BallSocket.PrepareForIterationBallSocket(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
        BallSocket.IterateBallSocket(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));

        // 两个横向行都可独立求解，尽管它们的响应量级不同。
        Assert.That(JVector.MaxAbs(body.AngularVelocity), Is.LessThan(Tolerance));
        Assert.That(Real.IsFinite(JVector.MaxAbs(joint.Impulse)), Is.True);
    }

    [Test]
    public void FixedAngle_SingularInertiaStopsResponsiveAxesWithoutLockFlags([Values] bool irrelevantLocks)
    {
        using World world = new();
        RigidBody body = world.CreateRigidBody();
        body.SetMassInertia(JSymmetricMatrix.CreateScale(0, 1, 1), 0, true);
        if (irrelevantLocks) body.AllowedMotion = MotionAxes.Angular;
        var joint = world.CreateConstraint<FixedAngle>(world.NullBody, body);
        joint.Initialize();
        joint.Softness = joint.Bias = 0;
        body.AngularVelocity = new JVector(3, 2, 1);

        FixedAngle.PrepareForIterationFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
        FixedAngle.IterateFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));

        // 绕 X 的无限惯性无法响应，但不得禁用 Y/Z 行。
        Assert.That(JVector.MaxAbs(body.AngularVelocity - new JVector(3, 0, 0)), Is.LessThan(Tolerance));
        Assert.That(Real.IsFinite(JVector.MaxAbs(joint.Impulse)), Is.True);
    }

    [Test]
    public void BallSocket_UniformMassScaleDoesNotDisableTranslation(
        [Values] bool largeInverseMass, [Values] bool irrelevantLocks)
    {
        using World world = new();
        RigidBody body = world.CreateRigidBody();
        Real inverseMass = ExtremeScale(largeInverseMass);
        body.SetMassInertia(JSymmetricMatrix.Zero, inverseMass, true);
        if (irrelevantLocks) body.AllowedMotion = MotionAxes.Linear;
        var joint = world.CreateConstraint<BallSocket>(world.NullBody, body);
        joint.Initialize(JVector.Zero);
        joint.Softness = joint.Bias = 0;
        body.Velocity = new JVector(1, -2, 3);

        BallSocket.PrepareForIterationBallSocket(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
        BallSocket.IterateBallSocket(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));

        Assert.That(JVector.MaxAbs(body.Velocity), Is.LessThan(Tolerance));
        // 再乘回逆质量可避免在检查极小质量冲量时发生溢出。
        Assert.That(Real.IsFinite(JVector.MaxAbs(joint.Impulse)), Is.True);
        Assert.That(JVector.MaxAbs(joint.Impulse * inverseMass + new JVector(1, -2, 3)),
            Is.LessThan(Tolerance));
    }

    [Test]
    public void FixedAngle_UniformInertiaScaleDoesNotDisableAngularResponse(
        [Values] bool largeInverseInertia, [Values] bool angularLock)
    {
        using World world = new();
        RigidBody body = world.CreateRigidBody();
        Real inverseInertia = ExtremeScale(largeInverseInertia);
        body.SetMassInertia(JSymmetricMatrix.Identity * inverseInertia, 0, true);
        if (angularLock) body.AllowedMotion = MotionAxes.AngularY | MotionAxes.AngularZ;
        var joint = world.CreateConstraint<FixedAngle>(world.NullBody, body);
        joint.Initialize();
        joint.Softness = joint.Bias = 0;
        body.AngularVelocity = new JVector(0, -2, 3);

        FixedAngle.PrepareForIterationFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
        FixedAngle.IterateFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));

        Assert.That(JVector.MaxAbs(body.AngularVelocity), Is.LessThan(Tolerance));
        Assert.That(Real.IsFinite(JVector.MaxAbs(joint.Impulse)), Is.True);
    }

    [Test]
    public void FixedAngle_MixedExtremeInertiaScalesKeepEveryResponsiveRow()
    {
        using World world = new();
        RigidBody body = world.CreateRigidBody();
        Real strong = Precision.IsDoublePrecision ? (Real)1e200 : (Real)1e20;
        Real weak = Precision.IsDoublePrecision ? (Real)1e-200 : (Real)1e-20;
        body.SetMassInertia(JSymmetricMatrix.CreateScale(strong, strong, weak), 0, true);
        var joint = world.CreateConstraint<FixedAngle>(world.NullBody, body);
        joint.Initialize();
        joint.Softness = joint.Bias = 0;
        body.AngularVelocity = new JVector(1, -2, 3);

        FixedAngle.PrepareForIterationFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
        FixedAngle.IterateFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));

        // 即使未缩放逆矩阵的余子式溢出，行列式仍是有限的。
        Assert.That(Real.IsFinite(JVector.MaxAbs(body.AngularVelocity)), Is.True);
        Assert.That(JVector.MaxAbs(body.AngularVelocity), Is.LessThan(Tolerance));
        Assert.That(Real.IsFinite(JVector.MaxAbs(joint.Impulse)), Is.True);
    }

    [Test]
    public void UnrepresentableInverseResponseDoesNotCorruptVelocity([Values] bool angular)
    {
        using World world = new();
        RigidBody body = world.CreateRigidBody();
        JVector initial = new(1, -2, 3);
        if (angular)
        {
            body.SetMassInertia(JSymmetricMatrix.Identity * Real.Epsilon, 0, true);
            var joint = world.CreateConstraint<FixedAngle>(world.NullBody, body);
            joint.Initialize();
            joint.Softness = joint.Bias = 0;
            body.AngularVelocity = initial;
            FixedAngle.PrepareForIterationFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
            FixedAngle.IterateFixedAngle(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
            Assert.That(body.AngularVelocity, Is.EqualTo(initial));
            Assert.That(joint.Impulse, Is.EqualTo(JVector.Zero));
        }
        else
        {
            body.SetMassInertia(JSymmetricMatrix.Zero, Real.Epsilon, true);
            var joint = world.CreateConstraint<BallSocket>(world.NullBody, body);
            joint.Initialize(JVector.Zero);
            joint.Softness = joint.Bias = 0;
            body.Velocity = initial;
            BallSocket.PrepareForIterationBallSocket(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
            BallSocket.IterateBallSocket(ref joint.Handle.Data, new TimeStep((Real)1.0 / 100));
            Assert.That(body.Velocity, Is.EqualTo(initial));
            Assert.That(joint.Impulse, Is.EqualTo(JVector.Zero));
        }
    }

    private static Real ExtremeScale(bool large) => Precision.IsDoublePrecision
        ? (large ? (Real)1e110 : (Real)1e-110)
        : (large ? (Real)1e20 : (Real)1e-13);
}
