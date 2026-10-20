/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace Jitter2.SoftBodies;

/// <summary>
/// 表示软体模拟中的三角形形状。
/// </summary>
public sealed class SoftBodyTriangle : SoftBodyShape
{
    private readonly RigidBody v1;
    private readonly RigidBody v2;
    private readonly RigidBody v3;

    /// <summary>
    /// 获取三角形的第一个顶点（刚体）。
    /// </summary>
    public RigidBody Vertex1 => v1;

    /// <summary>
    /// 获取三角形的第二个顶点（刚体）。
    /// </summary>
    public RigidBody Vertex2 => v2;

    /// <summary>
    /// 获取三角形的第三个顶点（刚体）。
    /// </summary>
    public RigidBody Vertex3 => v3;

    private Real halfThickness = (Real)0.05;

    /// <summary>
    /// 获取或设置三角形的厚度。
    /// </summary>
    /// <exception cref="System.ArgumentOutOfRangeException">当厚度为负数或非有限值时抛出。</exception>
    public Real Thickness
    {
        get => halfThickness * (Real)2.0;
        set
        {
            halfThickness = ArgumentCheck.NonNegative(value, nameof(Thickness)) * (Real)0.5;
            UpdateWorldBoundingBox();
        }
    }

    /// <summary>
    /// 初始化 <see cref="SoftBodyTriangle"/> 类的新实例。
    /// </summary>
    /// <param name="body">此形状所属的软体。</param>
    /// <param name="v1">第一个顶点。</param>
    /// <param name="v2">第二个顶点。</param>
    /// <param name="v3">第三个顶点。</param>
    public SoftBodyTriangle(SoftBody body, RigidBody v1, RigidBody v2, RigidBody v3)
    {
        this.v1 = v1;
        this.v2 = v2;
        this.v3 = v3;

        SoftBody = body;
        UpdateWorldBoundingBox();
    }

    /// <inheritdoc/>
    public override JVector Velocity => (Real)(1.0 / 3.0) * (v1.Data.Velocity + v2.Data.Velocity + v3.Data.Velocity);

    /// <inheritdoc/>
    public override RigidBody GetClosest(in JVector pos)
    {
        Real distance1Squared = (pos - v1.Position).LengthSquared();
        Real distance2Squared = (pos - v2.Position).LengthSquared();
        Real distance3Squared = (pos - v3.Position).LengthSquared();

        return (distance1Squared < distance2Squared && distance1Squared < distance3Squared) ? v1 :
            (distance2Squared < distance3Squared) ? v2 : v3;
    }

    /// <inheritdoc/>
    public override void UpdateWorldBoundingBox(Real dt = (Real)0.0)
    {
        Real extraMargin = MathR.Max(halfThickness, (Real)0.01);

        JBoundingBox box = JBoundingBox.SmallBox;

        JBoundingBox.AddPointInPlace(ref box, Vertex1.Position);
        JBoundingBox.AddPointInPlace(ref box, Vertex2.Position);
        JBoundingBox.AddPointInPlace(ref box, Vertex3.Position);

        // 防止包围盒退化
        JVector extra = new(extraMargin);
        box.Min -= extra;
        box.Max += extra;

        WorldBoundingBox = box;
    }

    /// <inheritdoc/>
    public override void SupportMap(in JVector direction, out JVector result)
    {
        JVector a = v1.Position;
        JVector b = v2.Position;
        JVector c = v3.Position;

        Real maxDot = JVector.Dot(a, direction);
        Real dot = JVector.Dot(b, direction);

        result = a;

        if (dot > maxDot)
        {
            maxDot = dot;
            result = b;
        }

        dot = JVector.Dot(c, direction);

        if (dot > maxDot)
        {
            result = c;
        }

        result += JVector.Normalize(direction) * halfThickness;
    }

    /// <inheritdoc/>
    public override void GetCenter(out JVector point)
    {
        point = ((Real)(1.0 / 3.0)) * (Vertex1.Position + Vertex2.Position + Vertex3.Position);
    }
}
