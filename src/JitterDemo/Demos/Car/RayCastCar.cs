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
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace JitterDemo;

/// <summary>
/// 创建带有 4 个车轮的 Jitter 默认汽车。要创建自定义汽车，
/// 请使用 Wheel 类并将其添加到一个刚体上。
/// </summary>
public class RayCastCar
{
    // 默认汽车有 4 个车轮
    private readonly World world;

    public RigidBody Body { get; }

    private float destSteering;
    private float destAccelerate;
    private float steering;
    private float accelerate;

    /// <summary>
    /// 两个前轮的最大转向角，以度为单位。
    /// </summary>
    public float SteerAngle { get; set; }

    /// <summary>
    /// 汽车加速时施加的最大力矩。
    /// </summary>
    public float DriveTorque { get; set; }

    /// <summary>
    /// 调低/调高汽车的加速度。
    /// </summary>
    public float AccelerationRate { get; set; }

    /// <summary>
    /// 调低/调高汽车的转向速率。
    /// </summary>
    public float SteerRate { get; set; }

    // 不要把阻尼调到完美，保留一些弹性。
    private const float dampingFrac = 0.8f;
    private const float springFrac = 0.45f;

    /// <summary>
    /// 初始化 DefaultCar 类的新实例。
    /// </summary>
    /// <param name="world">汽车所在的世界。</param>
    /// <param name="shape">汽车的形状。推荐使用盒形。</param>
    public RayCastCar(World world)
    {
        this.world = world;

        // 设置一些默认值
        AccelerationRate = 10f;
        SteerAngle = (float)JAngle.FromDegree(40.0f);
        DriveTorque = 340.0f;
        SteerRate = 5.0f;

        Body = world.CreateRigidBody();

        TransformedShape tfs1 = new(new BoxShape(3.1f, 1.4f, 8f), new JVector(0, 0.4f, 0));
        TransformedShape tfs2 = new(new BoxShape(2.4f, 0.8f, 5f), new JVector(0.0f, 1.5f, 1.1f));

        Body.AddShape(tfs1);
        Body.AddShape(tfs2);

        float mass = 100.0f;
        JVector sides = new JVector(3.1f, 1.0f, 8.0f);

        float Ixx = (1.0f / 12.0f) * mass * (sides.Y * sides.Y + sides.Z * sides.Z);
        float Iyy = (1.0f / 12.0f) * mass * (sides.X * sides.X + sides.Z * sides.Z);
        float Izz = (1.0f / 12.0f) * mass * (sides.X * sides.X + sides.Y * sides.Y);

        JSymmetricMatrix inertia = JSymmetricMatrix.CreateScale(Ixx, Iyy, Izz);

        Body.Position = new JVector(0, 0.5f, -4);
        Body.SetMassInertia(inertia, mass);

        Body.Damping = (0.0001f, 0.0001f);

        // 创建默认车轮
        Wheels[0] = new Wheel(world, Body, new JVector(-1.3f, -0.1f, -2.5f), 0.60f);
        Wheels[1] = new Wheel(world, Body, new JVector(+1.3f, -0.1f, -2.5f), 0.60f);
        Wheels[2] = new Wheel(world, Body, new JVector(-1.3f, -0.1f, +2.4f), 0.60f);
        Wheels[3] = new Wheel(world, Body, new JVector(+1.3f, -0.1f, +2.4f), 0.60f);

        AdjustWheelValues();
    }

    /// <summary>
    /// 根据汽车质量、车轮半径和重力，重新计算所有车轮的
    /// 惯性、阻尼和弹簧。在修改车轮数据后应调用此方法。
    /// </summary>
    public void AdjustWheelValues()
    {
        float mass = Body.Mass / 4.0f;
        float wheelMass = Body.Mass * 0.03f;

        foreach (Wheel w in Wheels)
        {
            w.Inertia = 0.5f * (w.Radius * w.Radius) * wheelMass;
            w.Spring = mass * world.Gravity.Length() / (w.WheelTravel * springFrac);
            w.Damping = 2.0f * (float)Math.Sqrt(w.Spring * Body.Mass) * 0.25f * dampingFrac;
        }
    }

    /// <summary>
    /// 访问车轮。
    /// </summary>
    public Wheel[] Wheels { get; } = new Wheel[4];

    /// <summary>
    /// 设置汽车的控制输入值。
    /// </summary>
    /// <param name="accelerate">
    /// 取值在 -1 到 1 之间（其他值会被钳制）。通过设置 <see cref="DriveTorque" /> 来调整
    /// 汽车的最大速度。通过设置 <see cref="AccelerationRate" /> 来调整最大加速度。
    /// </param>
    /// <param name="steer">
    /// 取值在 -1 到 1 之间（其他值会被钳制）。通过设置 <see cref="SteerAngle" /> 来调整
    /// 最大转向角。转向变化的速度通过 <see cref="SteerRate" /> 调整。
    /// </param>
    public void SetInput(float accelerate, float steer)
    {
        destAccelerate = accelerate;
        destSteering = steer;
    }

    public void Step(float timestep)
    {
        foreach (Wheel w in Wheels) w.PreStep(timestep);

        float deltaAccelerate = timestep * AccelerationRate;

        float deltaSteering = timestep * SteerRate;

        float dAccelerate = destAccelerate - accelerate;
        dAccelerate = Math.Clamp(dAccelerate, -deltaAccelerate, deltaAccelerate);

        float dSteering = destSteering - steering;
        dSteering = Math.Clamp(dSteering, -deltaSteering, deltaSteering);

        accelerate += dAccelerate;
        steering += dSteering;

        float maxTorque = DriveTorque * 0.5f;

        foreach (Wheel w in Wheels)
        {
            w.AddTorque(maxTorque * accelerate);

            if (destAccelerate == 0.0f && w.AngularVelocity < 0.8f)
            {
                // 如果汽车足够慢且 destAccelerate 为零，
                // 则沿角速度的反方向施加力矩，
                // 使汽车完全停止。
                w.AddTorque(-w.AngularVelocity);
            }
        }

        float alpha = SteerAngle * steering;

        Wheels[0].SteerAngle = alpha;
        Wheels[1].SteerAngle = alpha;

        foreach (Wheel w in Wheels)
        {
            w.PostStep(timestep);
        }
    }
}
