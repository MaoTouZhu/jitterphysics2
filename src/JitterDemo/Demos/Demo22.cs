using System;
using System.Collections.Generic;
using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;
using JitterDemo.Renderer;

namespace JitterDemo;

public class Demo22 : IDemo, ICleanDemo, IDrawUpdate
{
    public string Name => "Conveyor Belt";
    public string Description => "Kinematic planks forming a conveyor belt that transports rigid bodies.";

    private Playground pg = null!;
    private World world = null!;

    // 我们需要手动为物理步追踪时间，
    // 以确保传送带与求解器完全同步移动。
    private double physicsTime = 0.0;

    private struct BeltPlank
    {
        public RigidBody Body;
        public float DistanceOffset;
    }

    private readonly List<BeltPlank> planks = new();

    private static class Curve
    {
        public const float Speed = 2f;
        public const float StraightLength = 12.0f;
        public const float Radius = 6.0f;

        public static float TotalLength => (StraightLength * 2.0f) + (MathF.PI * Radius * 2.0f);

        public static void GetState(float distance, out JVector pos, out JVector vel, out float angVelY)
        {
            distance = distance % TotalLength;
            if (distance < 0) distance += TotalLength;

            float curveLen = MathF.PI * Radius;

            // 1. 底部直线段
            if (distance < StraightLength)
            {
                float t = distance;
                pos = new JVector(-StraightLength * 0.5f + t, 4.0f, -Radius);
                vel = new JVector(Speed, 0, 0);
                angVelY = 0;
            }
            // 2. 右侧转弯
            else if (distance < StraightLength + curveLen)
            {
                float t = distance - StraightLength;
                float angle = -MathF.PI * 0.5f + (t / Radius);
                pos = new JVector(StraightLength * 0.5f + MathF.Cos(angle) * Radius, 4.0f, MathF.Sin(angle) * Radius);
                vel = new JVector(-MathF.Sin(angle), 0, MathF.Cos(angle)) * Speed;
                angVelY = Speed / Radius;
            }
            // 3. 顶部直线段
            else if (distance < (StraightLength * 2.0f) + curveLen)
            {
                float t = distance - (StraightLength + curveLen);
                pos = new JVector(StraightLength * 0.5f - t, 4.0f, Radius);
                vel = new JVector(-Speed, 0, 0);
                angVelY = 0;
            }
            // 4. 左侧转弯
            else
            {
                float t = distance - ((StraightLength * 2.0f) + curveLen);
                float angle = MathF.PI * 0.5f + (t / Radius);
                pos = new JVector(-StraightLength * 0.5f + MathF.Cos(angle) * Radius, 4.0f, MathF.Sin(angle) * Radius);
                vel = new JVector(-MathF.Sin(angle), 0, MathF.Cos(angle)) * Speed;
                angVelY = Speed / Radius;
            }
        }
    }

    public void Build(Playground pg, World world)
    {
        this.pg = pg;
        this.world = world;
        pg.AddFloor();
        planks.Clear();
        physicsTime = 0;

        // 订阅物理步事件
        world.PreSubStep += OnPreStep;

        float plankWidth = 0.6f;
        int plankCount = (int)(Curve.TotalLength / plankWidth);
        float distStep = Curve.TotalLength / plankCount;

        for (int i = 0; i < plankCount; i++)
        {
            var body = world.CreateRigidBody();
            body.AddShape(new BoxShape(1.8f, 0.1f, 0.55f));
            body.MotionType = MotionType.Kinematic;
            body.Friction = 1.0f;

            float dist = i * distStep;
            Curve.GetState(dist, out JVector pos, out JVector vel, out float w);

            body.Position = pos;

            // 对齐初始朝向
            JVector fwd = JVector.Normalize(vel);
            JVector up = JVector.UnitY;
            JVector right = JVector.Normalize(JVector.Cross(up, fwd));
            up = JVector.Cross(fwd, right);
            JMatrix ori = JMatrix.FromColumns(right, up, fwd);
            body.Orientation = JQuaternion.CreateFromMatrix(ori);

            planks.Add(new BeltPlank { Body = body, DistanceOffset = dist });
        }

        // 碎片
        for (int i = 0; i < 8; i++)
        {
            var box = world.CreateRigidBody();
            box.AddShape(new BoxShape(1));
            box.Position = new JVector(-5 + i * 1.5f, 6, -Curve.Radius);
        }

        var floor = world.CreateRigidBody();
        floor.AddShape(new BoxShape(100, 1, 100));
        floor.Position = new JVector(0, -5, 0);
        floor.MotionType = MotionType.Static;
    }

    // 由 Jitter 在每次物理子步之前自动调用
    private void OnPreStep(TimeStep timeStep)
    {
        physicsTime += timeStep.SubstepDt;
        float globalDist = (float)physicsTime * Curve.Speed;

        foreach (var plank in planks)
        {
            float d = globalDist + plank.DistanceOffset;

            Curve.GetState(d, out JVector targetPos, out JVector targetVel, out float targetAngVelY);

            // 运动控制逻辑
            //
            // 我们希望平台沿路径 p(t) 移动并保持相应朝向。直接设置位置/朝向
            // 会导致瞬移，从而无法进行正常的物理模拟。
            //
            // 为此，我们采用一种控制方案，将前馈（目标速度）
            // 与比例控制器（位置误差修正）结合起来，驱动刚体速度 v(t)
            // 趋近目标：
            //
            // v(t) = alpha * (k(t) - x(t))
            //
            // 其中 x(t) 是当前位置，alpha 是增益常数。
            // 通过选取 k(t) = p(t) + 1/alpha * p'(t)，该微分方程的解为：
            //
            // x(t) = C * exp(-alpha * t) + p(t)
            //
            // 这意味着刚体的位置 x(t) 会以指数方式收敛到目标路径 p(t)。
            //
            // 我们对线速度（见下文）和角速度应用同样的逻辑。
            plank.Body.Velocity = targetVel + (targetPos - plank.Body.Position) * 10.0f;

            JVector currentForward = plank.Body.Orientation.GetBasisZ();
            JVector targetForward = JVector.Normalize(targetVel);

            // 计算角度误差的正弦值（叉积的 Y 分量）
            float angleError = (currentForward.Z * targetForward.X - currentForward.X * targetForward.Z);

            float correction = angleError * 20.0f;
            plank.Body.AngularVelocity = new JVector(0, targetAngVelY + correction, 0);
        }
    }

    public void DrawUpdate()
    {
        // 此处只保留绘制逻辑
        const int stepMax = 200;
        float totalLen = Curve.TotalLength;

        for (int step = 0; step < stepMax; step++)
        {
            float d1 = totalLen / stepMax * step;
            float d2 = totalLen / stepMax * (step + 1);
            Curve.GetState(d1, out JVector p1, out _, out _);
            Curve.GetState(d2, out JVector p2, out _, out _);
            pg.DebugRenderer.PushLine(DebugRenderer.Color.Green, Conversion.FromJitter(p1), Conversion.FromJitter(p2));
        }
    }

    // 重要：切换演示时要取消订阅，以避免内存泄漏，
    // 或让残留逻辑在下一个演示中继续运行。
    public void CleanUp()
    {
        if (world != null)
        {
            world.PreSubStep -= OnPreStep;
        }
    }
}
