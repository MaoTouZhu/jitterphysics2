// NOTE: 射线投射汽车演示是从优秀的 JigLib 的载具示例复制并稍作修改的版本。
//       许可证如下。

/*
Copyright (c) 2007 Danny Chapman
http://www.rowlhouse.co.uk

This software is provided 'as-is', without any express or implied
warranty. In no event will the authors be held liable for any damages
arising from the use of this software.

Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it
freely, subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not
claim that you wrote the original software. If you use this software
in a product, an acknowledgment in the product documentation would be
appreciated but is not required.

2. Altered source versions must be plainly marked as such, and must not be
misrepresented as being the original software.

3. This notice may not be removed or altered from any source
distribution.
*/

using System;
using Jitter2;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;
using JitterDemo.Renderer;

namespace JitterDemo;

/// <summary>
/// 一个为刚体施加驱动力矩的车轮。
/// 可用于创建载具。
/// </summary>
public class Wheel
{
    private readonly World world;

    private readonly RigidBody car;

    private float displacement, upSpeed, lastDisplacement;
    private bool onFloor;
    private float driveTorque;

    private float angVel;

    /// 用于估算摩擦力
    private float angVelForGrip;

    private float torque;

    private readonly DynamicTree.RayCastFilterPre rayCast;

    /// <summary>
    /// 设置或获取车轮当前的转向角，
    /// 以度为单位。
    /// </summary>
    public float SteerAngle { get; set; }

    /// <summary>
    /// 获取车轮当前的旋转角度，以度为单位。
    /// </summary>
    public float WheelRotation { get; private set; }

    /// <summary>
    /// 悬架弹簧的阻尼系数。
    /// </summary>
    public float Damping { get; set; }

    /// <summary>
    /// 悬架弹簧。
    /// </summary>
    public float Spring { get; set; }

    /// <summary>
    /// 车轮的惯性。
    /// </summary>
    public float Inertia { get; set; }

    /// <summary>
    /// 车轮的半径。
    /// </summary>
    public float Radius { get; set; }

    /// <summary>
    /// 汽车在侧向的摩擦力。
    /// </summary>
    public float SideFriction { get; set; }

    /// <summary>
    /// 汽车在前进方向的摩擦力。
    /// </summary>
    public float ForwardFriction { get; set; }

    /// <summary>
    /// 悬架弹簧的长度。
    /// </summary>
    public float WheelTravel { get; set; }

    /// <summary>
    /// 如果设为 true，车轮会锁死。
    /// </summary>
    public bool Locked { get; set; }

    /// <summary>
    /// 车轮可能达到的最高速度。
    /// </summary>
    public float MaximumAngularVelocity { get; set; }

    /// <summary>
    /// 用于此车轮的射线数量。
    /// </summary>
    public int NumberOfRays { get; set; }

    /// <summary>
    /// 车轮在刚体空间中的位置。
    /// </summary>
    public JVector Position { get; set; }

    public float AngularVelocity => angVel;

    public readonly JVector Up = JVector.UnitY;

    /// <summary>
    /// 创建 Wheel 类的新实例。
    /// </summary>
    /// <param name="world">世界。</param>
    /// <param name="car">施加车轮力的 RigidBody。</param>
    /// <param name="position">车轮在刚体上的位置（刚体空间中）。</param>
    /// <param name="radius">车轮半径。</param>
    public Wheel(World world, RigidBody car, JVector position, float radius)
    {
        this.world = world;
        this.car = car;
        Position = position;

        rayCast = RayCastCallback;

        // 设置一些默认值。
        SideFriction = 3.0f;
        ForwardFriction = 5.0f;
        Radius = radius;
        Inertia = 1.0f;
        WheelTravel = 0.2f;
        MaximumAngularVelocity = 200;
        NumberOfRays = 1;
    }

    /// <summary>
    /// 获取车轮在世界空间中的位置。
    /// </summary>
    /// <returns>车轮在世界空间中的位置。</returns>
    public JVector GetWheelCenter()
    {
        return Position + JVector.Transform(Up, car.Orientation) * displacement;
    }

    /// <summary>
    /// 添加驱动力矩。
    /// </summary>
    /// <param name="torque">施加到此车轮的力矩大小。</param>
    public void AddTorque(float torque)
    {
        driveTorque += torque;
    }

    public void PostStep(float timeStep)
    {
        if (timeStep <= 0.0f) return;

        float origAngVel = angVel;
        upSpeed = (displacement - lastDisplacement) / timeStep;

        if (Locked)
        {
            angVel = 0;
            torque = 0;
        }
        else
        {
            angVel += torque * timeStep / Inertia;
            torque = 0;

            if (!onFloor) driveTorque *= 0.1f;

            // 防止摩擦力使方向反转 - todo 通过限
            // 制力矩来更好地实现
            if ((origAngVel > angVelForGrip && angVel < angVelForGrip) ||
                (origAngVel < angVelForGrip && angVel > angVelForGrip))
                angVel = angVelForGrip;

            angVel += driveTorque * timeStep / Inertia;
            driveTorque = 0;

            float maxAngVel = MaximumAngularVelocity;
            angVel = Math.Clamp(angVel, -maxAngVel, maxAngVel);

            WheelRotation += timeStep * angVel;
        }
    }

    public void PreStep(float timeStep)
    {
        JVector force = JVector.Zero;
        lastDisplacement = displacement;
        displacement = 0.0f;

        float vel = car.Velocity.Length();

        JVector worldPos = car.Position + JVector.Transform(Position, car.Orientation);
        JVector worldAxis = JVector.Transform(Up, car.Orientation);

        JVector forward = JVector.Transform(-JVector.UnitZ, car.Orientation);
        JVector wheelFwd = JVector.Transform(forward, JMatrix.CreateRotationMatrix(worldAxis, SteerAngle));

        JVector wheelLeft = JVector.Cross(worldAxis, wheelFwd);
        JVector.NormalizeInPlace(ref wheelLeft);

        JVector wheelUp = JVector.Cross(wheelFwd, wheelLeft);

        float rayLen = 2.0f * Radius + WheelTravel;

        JVector wheelRayEnd = worldPos - Radius * worldAxis;
        JVector wheelRayOrigin = wheelRayEnd + rayLen * worldAxis;
        JVector wheelRayDelta = wheelRayEnd - wheelRayOrigin;

        float deltaFwd = 2.0f * Radius / (NumberOfRays + 1);
        float deltaFwdStart = deltaFwd;

        onFloor = false;

        JVector groundNormal = JVector.Zero;
        JVector groundPos = JVector.Zero;
        float deepestFrac = float.MaxValue;
        RigidBody worldBody = null!;

        for (int i = 0; i < NumberOfRays; i++)
        {
            float distFwd = deltaFwdStart + i * deltaFwd - Radius;
            float zOffset = Radius * (1.0f - (float)Math.Cos(Math.PI / 2.0f * (distFwd / Radius)));

            JVector newOrigin = wheelRayOrigin + distFwd * wheelFwd + zOffset * wheelUp;

            bool result = world.DynamicTree.RayCast(newOrigin, wheelRayDelta,
                rayCast, null, out IDynamicTreeProxy? shape, out JVector normal, out float frac);

            if (result && frac <= 1.0f)
            {
                // 由于我们过滤掉了其他射线检测，形状必然是 RigidBodyShape
                RigidBody body = (shape as RigidBodyShape)!.RigidBody;

                if (frac < deepestFrac)
                {
                    deepestFrac = frac;
                    groundPos = newOrigin + frac * wheelRayDelta;
                    worldBody = body;
                    groundNormal = normal;
                }

                onFloor = true;
            }
        }

        if (!onFloor) return;

        if (groundNormal.LengthSquared() > 0.0f) JVector.NormalizeInPlace(ref groundNormal);

        displacement = rayLen * (1.0f - deepestFrac);
        displacement = Math.Clamp(displacement, 0.0f, WheelTravel);

        float displacementForceMag = displacement * Spring;

        // 当悬架与地面平行时减小力
        displacementForceMag *= JVector.Dot(groundNormal, worldAxis);

        // 施加阻尼
        float dampingForceMag = upSpeed * Damping;

        float totalForceMag = displacementForceMag + dampingForceMag;

        if (totalForceMag < 0.0f) totalForceMag = 0.0f;

        JVector extraForce = totalForceMag * worldAxis;

        force += extraForce;

        // 侧滑摩擦力和驱动力。计算车轮和地面相对的坐标系
        JVector groundUp = groundNormal;
        JVector groundLeft = JVector.Cross(groundNormal, wheelFwd);
        if (groundLeft.LengthSquared() > 0.0f) JVector.NormalizeInPlace(ref groundLeft);

        JVector groundFwd = JVector.Cross(groundLeft, groundUp);

        JVector wheelPointVel = car.Velocity +
                                JVector.Cross(car.AngularVelocity, JVector.Transform(Position, car.Orientation));

        // rimVel=(wxr)*v
        JVector rimVel = angVel * JVector.Cross(wheelLeft, groundPos - worldPos);
        wheelPointVel += rimVel;

        if (worldBody == null) throw new Exception("car: world body is null.");

        JVector worldVel = worldBody.Velocity +
                           JVector.Cross(worldBody.AngularVelocity, groundPos - worldBody.Position);

        wheelPointVel -= worldVel;

        // 侧向力
        float noslipVel = 0.2f;
        float slipVel = 0.4f;
        float slipFactor = 0.7f;

        float smallVel = 3.0f;
        float friction = SideFriction;

        float sideVel = JVector.Dot(wheelPointVel, groundLeft);

        if (sideVel > slipVel || sideVel < -slipVel)
        {
            friction *= slipFactor;
        }
        else if (sideVel > noslipVel || sideVel < -noslipVel)
        {
            friction *= 1.0f - (1.0f - slipFactor) * (Math.Abs(sideVel) - noslipVel) / (slipVel - noslipVel);
        }

        if (sideVel < 0.0f)
            friction *= -1.0f;

        if (Math.Abs(sideVel) < smallVel)
            friction *= Math.Abs(sideVel) / smallVel;

        float sideForce = -friction * totalForceMag;

        extraForce = sideForce * groundLeft;
        force += extraForce;

        // 前/后方向力
        friction = ForwardFriction;
        float fwdVel = JVector.Dot(wheelPointVel, groundFwd);

        if (fwdVel > slipVel || fwdVel < -slipVel)
        {
            friction *= slipFactor;
        }
        else if (fwdVel > noslipVel || fwdVel < -noslipVel)
        {
            friction *= 1.0f - (1.0f - slipFactor) * (Math.Abs(fwdVel) - noslipVel) / (slipVel - noslipVel);
        }

        if (fwdVel < 0.0f)
            friction *= -1.0f;

        if (Math.Abs(fwdVel) < smallVel)
            friction *= Math.Abs(fwdVel) / smallVel;

        float fwdForce = -friction * totalForceMag;

        extraForce = fwdForce * groundFwd;
        force += extraForce;

        // 前向力也会使车轮旋转
        JVector wheelCentreVel = car.Velocity +
                                 JVector.Cross(car.AngularVelocity, JVector.Transform(Position, car.Orientation));

        angVelForGrip = JVector.Dot(wheelCentreVel, groundFwd) / Radius;
        torque += -fwdForce * Radius;

        // 向汽车施加力
        car.AddForce(force, groundPos);

        RenderWindow.Instance.DebugRenderer.PushPoint(DebugRenderer.Color.White, Conversion.FromJitter(groundPos), 0.2f);

        // 向世界施加力
        if (worldBody.MotionType == MotionType.Dynamic)
        {
            const float maxOtherBodyAcc = 500.0f;
            float maxOtherBodyForce = maxOtherBodyAcc * worldBody.Mass;

            if (force.LengthSquared() > (maxOtherBodyForce * maxOtherBodyForce))
                force *= maxOtherBodyForce / force.Length();

            worldBody.SetActivationState(true);

            worldBody.AddForce(force * -1, groundPos);
        }
    }

    private bool RayCastCallback(IDynamicTreeProxy shape)
    {
        if (shape is not RigidBodyShape rbs) return false;
        return rbs.RigidBody != car;
    }
}
