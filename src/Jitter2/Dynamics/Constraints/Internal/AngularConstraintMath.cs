/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.LinearMath;

namespace Jitter2.Dynamics.Constraints;

internal static class AngularConstraintMath
{
    internal static Real ShortestAngle(Real angle, Real halfPeriod)
    {
        if (angle >= -halfPeriod && angle <= halfPeriod) return angle;
        Real period = (Real)2 * halfPeriod;
        angle %= period;
        if (angle > halfPeriod) angle -= period;
        else if (angle < -halfPeriod) angle += period;
        return angle;
    }

    // 四元数对数：最短相对旋转向量的一半。
    // 与四元数的向量部分不同，其导数在 pi 处仍然非奇异。
    internal static JVector RotationLog(in JQuaternion quaternion)
    {
        JQuaternion q = quaternion;
        if (q.W < 0) q *= -1;
        Real sine = q.Vector.Length();
        return sine > 0 ? q.Vector * (StableMath.Atan2(sine, q.W) / sine) : JVector.Zero;
    }

    internal static Real CalculateTwist(in JQuaternion left, in JQuaternion right, in JVector axis,
        out JVector jacobian, out short hemisphere)
    {
        JMatrix m = (-(Real)(1.0 / 2.0)) * QMatrix.ProjectMultiplyLeftRight(left, right);
        JQuaternion q = left * right;
        jacobian = JVector.TransposedTransform(axis, m);
        Real projection = JVector.Dot(axis, new JVector(q.X, q.Y, q.Z));
        hemisphere = q.W < 0 ? (short)-1 : (short)1;
        if (q.W < 0)
        {
            q *= -1;
            projection *= -(Real)1;
            jacobian *= -1;
        }

        // 全程使用投影角度，因此柔软度和冲量具有角度单位。
        Real derivativeSquared = jacobian.LengthSquared();
        if (derivativeSquared > 0)
        {
            jacobian *= (Real)1 / MathR.Sqrt(derivativeSquared);
        }
        else
        {
            // 在旋转半圈处，使用沿扭转方向的单侧导数。
            jacobian = -JVector.Transform(axis, right);
        }

        JVector perpendicular = q.Vector - axis * projection;
        Real cosine = MathR.Sqrt(q.W * q.W + perpendicular.LengthSquared());
        return (Real)2 * StableMath.Atan2(projection, cosine);
    }

    internal static JMatrix CalculateJacobian(in JQuaternion left, in JQuaternion right,
        out JVector error, out short hemisphere)
    {
        JQuaternion q = left * right;
        hemisphere = q.W < 0 ? (short)-1 : (short)1;
        if (q.W < 0) q *= -1;

        Real sine = q.Vector.Length();
        JMatrix derivative;
        if (sine > 0)
        {
            Real halfAngle = StableMath.Atan2(sine, q.W);
            JVector axis = q.Vector * ((Real)1 / sine);
            error = axis * halfAngle;

            // SO(3) 的左雅可比逆，用半角表示。
            // halfAngle*cot(halfAngle) 在零和 pi/2 处均为有限值。
            Real diagonal = halfAngle * q.W / sine;
            derivative = diagonal * JMatrix.Identity +
                ((Real)1 - diagonal) * JSymmetricMatrix.Outer(axis).ToMatrix() -
                JMatrix.CreateCrossProduct(error);
        }
        else
        {
            error = JVector.Zero;
            derivative = JMatrix.Identity;
        }

        // qdot = -.5 * (left * angularVelocityDifference * conjugate(left)) * q。
        return -(Real)0.5 * derivative * JMatrix.CreateFromQuaternion(left);
    }
}
