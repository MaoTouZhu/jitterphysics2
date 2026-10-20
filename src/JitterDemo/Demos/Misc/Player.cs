using System;
using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.Dynamics.Constraints;
using Jitter2.LinearMath;

namespace JitterDemo;

// 展示实现角色控制器的一种方式。
public class Player
{
    public RigidBody Body { get; }
    public AngularMotor AngularMovement { get; }

    private readonly World world;

    public Player(World world, JVector position)
    {
        Body = world.CreateRigidBody();
        Body.AddShape(new CapsuleShape());
        Body.Position = position;

        // 禁用速度阻尼
        Body.Damping = (0, 0);

        this.world = world;

        // 禁用自动休眠
        Body.DeactivationTime = TimeSpan.MaxValue;

        // 添加两条手臂——以便在视觉上分辨玩家的朝向
        var arm1 = new TransformedShape(new BoxShape(0.2f, 0.8f, 0.2f), new JVector(+0.5f, 0.3f, 0));
        var arm2 = new TransformedShape(new BoxShape(0.2f, 0.8f, 0.2f), new JVector(-0.5f, 0.3f, 0));

        // 添加形状时不重新计算质量和惯性，我们沿用此前添加的胶囊体
        // 形状的质量和惯性。
        Body.AddShape(arm1, MassInertiaUpdateMode.Preserve);
        Body.AddShape(arm2, MassInertiaUpdateMode.Preserve);

        // 让胶囊体保持直立，但能够旋转 360 度。
        var ur = world.CreateConstraint<HingeAngle>(Body, world.NullBody);
        ur.Initialize(JVector.UnitY, AngularLimit.Full);

        // 添加一些摩擦
        Body.Friction = 0.8f;

        // 用于转向的角度马达。
        AngularMovement = world.CreateConstraint<AngularMotor>(Body, world.NullBody);
        AngularMovement.Initialize(JVector.UnitY, JVector.UnitY);
        AngularMovement.MaximumTorque = 1000;
    }

    public void SetAngularInput(float rotate)
    {
        AngularMovement.TargetVelocity = rotate;
    }

    private bool CanJump(out RigidBody? floor, out JVector hitPoint)
    {
        foreach (var contact in Body.Contacts)
        {
            // 遍历胶囊体（玩家）的所有接触

            var cd = contact.Handle.Data;
            int numContacts = 0;
            hitPoint = JVector.Zero;

            // 一个接触最多可包含四个接触点，
            // 看看上一步使用了其中哪些。
            uint mask = cd.UsageMask >> 4;

            bool isBody1 = contact.Body1 == Body;
            floor = isBody1 ? contact.Body2 : contact.Body1;

            if ((mask & ContactData.MaskContact0) != 0) { hitPoint += isBody1 ? cd.Contact0.RelativePosition1 : cd.Contact0.RelativePosition2; numContacts++; }
            if ((mask & ContactData.MaskContact1) != 0) { hitPoint += isBody1 ? cd.Contact1.RelativePosition1 : cd.Contact1.RelativePosition2; numContacts++; }
            if ((mask & ContactData.MaskContact2) != 0) { hitPoint += isBody1 ? cd.Contact2.RelativePosition1 : cd.Contact2.RelativePosition2; numContacts++; }
            if ((mask & ContactData.MaskContact3) != 0) { hitPoint += isBody1 ? cd.Contact3.RelativePosition1 : cd.Contact3.RelativePosition2; numContacts++; }

            if (numContacts == 0) continue;

            // 将结果除以接触点数量，得到接触的“中心”
            hitPoint *= (1.0f / numContacts);

            // 检查命中点是否位于玩家底部
            if (hitPoint.Y <= -0.8f) return true;
        }

        hitPoint = JVector.Zero;
        floor = null;

        return false;
    }

    public void Jump()
    {
        if (CanJump(out RigidBody? floorBody, out JVector hitPoint))
        {
            float newYVel = 5.0f;

            if (floorBody != null)
            {
                newYVel += floorBody.Velocity.Y;
            }

            float deltaVel = Body.Velocity.Y - newYVel;

            Body.Velocity = new JVector(Body.Velocity.X, newYVel, Body.Velocity.Z);

            if (floorBody != null && floorBody.MotionType == MotionType.Dynamic)
            {
                float force = Body.Mass * deltaVel * 100.0f;
                floorBody.SetActivationState(true);
                floorBody.AddForce(JVector.UnitY * force, floorBody.Position + hitPoint);
            }
        }
    }

    public void SetLinearInput(JVector deltaMove)
    {
        if (!CanJump(out var floor, out JVector hitpoint))
        {
            return;
        }

        deltaMove *= 3.0f;

        float deltaMoveLen = deltaMove.Length();

        JVector bodyVel = Body.Velocity;
        bodyVel.Y = 0;

        float bodyVelLen = bodyVel.Length();

        if (deltaMoveLen > 0.01f)
        {
            if (bodyVelLen < 5f)
            {
                var force = JVector.Transform(deltaMove, Body.Orientation) * 10.0f;

                Body.AddForce(force);

                // 遵循牛顿定律（就这一次），并沿相反方向
                // 施加大小相等的力。
                floor!.AddForce(-force, Body.Position + hitpoint);
            }
        }

    }
}
