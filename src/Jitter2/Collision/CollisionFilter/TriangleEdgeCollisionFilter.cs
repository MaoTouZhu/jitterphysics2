/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

// #define DEBUG_EDGEFILTER

using System.Runtime.CompilerServices;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 过滤 <see cref="TriangleShape"/> 几何体的内部边碰撞。在共享边处调整碰撞
/// 法线以匹配相邻三角形，或在法线无法解析时丢弃该碰撞。需要三角形邻接信息；
/// 边界边（无邻接三角形）保持不变。与相对三角形法线夹角在
/// <see cref="AngleThreshold"/> 以内的背面法线会被丢弃。
/// </summary>
public class TriangleEdgeCollisionFilter : INarrowPhaseFilter
{
    private enum EdgeContactEvaluation
    {
        Keep,
        Discard,
        Edge
    }

    /// <summary>
    /// 获取或设置边碰撞检测的距离阈值，单位为世界单位。
    /// </summary>
    /// <remarks>
    /// 值越大，对边的过滤越激进，但可能错误地影响
    /// 三角形边界附近的合法碰撞。
    /// </remarks>
    /// <value>默认值为 0.01 个世界单位。</value>
    public Real EdgeThreshold { get; set; } = (Real)0.01;

    private Real cosAngle = (Real)0.999;

    /// <summary>
    /// 获取或设置动态三角形刚体是否接受边过滤。静态和运动学
    /// 三角形几何体始终会被过滤。默认为 <see langword="false"/>。
    /// </summary>
    public bool FilterDynamicBodies { get; set; }

    /// <summary>
    /// 获取或设置保留接触所需的投影碰撞法线最小长度。
    /// </summary>
    /// <remarks>
    /// 当碰撞法线投影到由三角形法线及其相邻三角形法线构成的平面上时，
    /// 投影非常短意味着碰撞正沿边折痕本身发生，此时将丢弃该碰撞。
    /// 值越小，允许通过的边碰撞越多；值越大，过滤越激进。
    /// </remarks>
    /// <value>默认值为 0.5。</value>
    public Real ProjectionThreshold { get; set; } = (Real)0.5;

    /// <summary>
    /// 获取或设置用于两个目的的角度阈值：判断两个三角形法线
    /// 何时被视为共面（使用更简单的吸附逻辑），以及丢弃法线在此角度内
    /// 与三角形法线反平行的背面碰撞。
    /// </summary>
    /// <value>默认值约为 2.5 度。</value>
    public JAngle AngleThreshold
    {
        get => JAngle.FromRadian(StableMath.Acos(cosAngle));
        set => cosAngle = StableMath.Cos(value.Radian);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void ProjectPointOnPlane(ref JVector point, in JVector a, in JVector b, in JVector c)
    {
        var ab = b - a;
        var ac = c - a;
        var normal = JVector.Cross(ab, ac);
        JVector.NormalizeInPlace(ref normal);

        var ap = point - a;
        var distance = JVector.Dot(ap, normal);
        point -= distance * normal;
    }

    private EdgeContactEvaluation EvaluateTriangleContact(TriangleShape triangleShape, bool triangleIsShapeB,
        in JVector contactPoint, in JVector normal, out JVector triangleNormal, out JVector neighborNormal)
    {
        ref readonly var triangle = ref triangleShape.Mesh.Indices[triangleShape.Index];

        triangleNormal = triangle.Normal;
        triangleNormal = JVector.Transform(triangleNormal, triangleShape.RigidBody.Data.Orientation);
        neighborNormal = default;

        if (triangleIsShapeB) JVector.NegateInPlace(ref triangleNormal);

        // 使三角形可从一侧穿透。
        if (JVector.Dot(normal, triangleNormal) < -cosAngle) return EdgeContactEvaluation.Discard;

        triangleShape.GetWorldVertices(out JVector a, out JVector b, out JVector c);

        JVector projectedContactPoint = contactPoint;
        ProjectPointOnPlane(ref projectedContactPoint, a, b, c);

        var n = b - a;
        var pma = projectedContactPoint - a;
        var d0 = (pma - JVector.Dot(pma, n) * n * ((Real)1.0 / n.LengthSquared())).LengthSquared();

        n = c - a;
        pma = projectedContactPoint - a;
        var d1 = (pma - JVector.Dot(pma, n) * n * ((Real)1.0 / n.LengthSquared())).LengthSquared();

        n = c - b;
        pma = projectedContactPoint - b;
        var d2 = (pma - JVector.Dot(pma, n) * n * ((Real)1.0 / n.LengthSquared())).LengthSquared();

        if (MathR.Min(MathR.Min(d0, d1), d2) > EdgeThreshold * EdgeThreshold)
        {
            return EdgeContactEvaluation.Keep;
        }

        if (d0 < d1 && d0 < d2)
        {
            if (triangle.NeighborC == -1) return EdgeContactEvaluation.Keep;
            neighborNormal = triangleShape.Mesh.Indices[triangle.NeighborC].Normal;
        }
        else if (d1 < d2)
        {
            if (triangle.NeighborB == -1) return EdgeContactEvaluation.Keep;
            neighborNormal = triangleShape.Mesh.Indices[triangle.NeighborB].Normal;
        }
        else
        {
            if (triangle.NeighborA == -1) return EdgeContactEvaluation.Keep;
            neighborNormal = triangleShape.Mesh.Indices[triangle.NeighborA].Normal;
        }

        neighborNormal = JVector.Transform(neighborNormal, triangleShape.RigidBody.Data.Orientation);

        if (triangleIsShapeB) JVector.NegateInPlace(ref neighborNormal);

        return EdgeContactEvaluation.Edge;
    }

    /// <inheritdoc />
    public bool Filter(RigidBodyShape shapeA, RigidBodyShape shapeB,
        ref JVector pointA, ref JVector pointB, ref JVector normal, ref Real penetration)
    {
        TriangleShape? ts1 = shapeA as TriangleShape;
        TriangleShape? ts2 = shapeB as TriangleShape;

        bool c1 = ts1 != null;
        bool c2 = ts2 != null;

        // 两个形状都是三角形或都不是 -> 直接返回
        if (c1 == c2) return true;

        TriangleShape triangleShape;

        if (c1)
        {
            triangleShape = ts1!;
        }
        else
        {
            triangleShape = ts2!;
        }

        if (triangleShape.RigidBody.MotionType == MotionType.Dynamic && !FilterDynamicBodies) return true;

        EdgeContactEvaluation evaluation = EvaluateTriangleContact(triangleShape, c2,
            c1 ? pointA : pointB, normal, out JVector triangleNormal, out JVector neighborNormal);

        if (evaluation == EdgeContactEvaluation.Discard) return false;
        if (evaluation == EdgeContactEvaluation.Keep) return true;

        ref var b1Data = ref shapeA.RigidBody.Data;
        ref var b2Data = ref shapeB.RigidBody.Data;

        bool isSpeculative = penetration < (Real)0.0;

        if (!isSpeculative)
        {
            // 再次检查碰撞（针对非推测接触），
            // 但将 EPA 阈值参数设为零。这是必要的，因为
            // MPR 对平面形状（如三角形）并不精确。

            bool result = NarrowPhase.MprEpa(shapeA, shapeB,
                b1Data.Orientation, b2Data.Orientation,
                b1Data.Position, b2Data.Position,
                out pointA, out pointB, out normal, out penetration,
                epaThreshold: (Real)0.0);

            if (!result)
            {
                // MPR 细化失败；拒绝该接触。
                return false;
            }

            evaluation = EvaluateTriangleContact(triangleShape, c2,
                c1 ? pointA : pointB, normal, out triangleNormal, out neighborNormal);

            if (evaluation == EdgeContactEvaluation.Discard) return false;
            if (evaluation == EdgeContactEvaluation.Keep) return true;
        }

        JVector midPoint = (Real)0.5 * (pointA + pointB);

        // 现在是有趣的部分。
        //
        // 我们在一条边附近发生了碰撞，其中
        //
        // triangleNormal -> 碰撞发生处的三角形法线
        // neighborNormal -> 相邻三角形的法线
        // normal         -> 碰撞法线
        if (JVector.Dot(triangleNormal, neighborNormal) > cosAngle)
        {
            // triangleNormal 与 neighborNormal 相同
            // ----------------------------------------------
            Real f5 = JVector.Dot(normal, neighborNormal);
            Real f6 = JVector.Dot(normal, triangleNormal);

            if (f5 > f6)
            {
#if DEBUG_EDGEFILTER
                Console.WriteLine($"case #1: adjusting; normal {normal} -> {neighborNormal}");
#endif

                if (!isSpeculative)
                {
                    penetration = 0;
                    pointA = pointB = midPoint;
                }

                normal = neighborNormal;
            }
            else
            {
#if DEBUG_EDGEFILTER
                Console.WriteLine($"case #1: adjusting; normal {normal} -> {triangleNormal}");
#endif

                if (!isSpeculative)
                {
                    penetration = 0;
                    pointA = pointB = midPoint;
                }

                normal = triangleNormal;
            }

            return true;
        }
        // neighborNormal 与 triangleNormal 不同
        // -----------------------------------------------

        // 第一步，将法线投影到由 triangleNormal 和 neighborNormal 给定的平面上，
        // 即移除沿叉积轴方向的分量
        JVector cross = neighborNormal % triangleNormal;
        Real crossLenSq = cross.LengthSquared();
        JVector proj = normal - (cross * normal / crossLenSq) * cross;

        if (proj.LengthSquared() < ProjectionThreshold * ProjectionThreshold)
        {
#if DEBUG_EDGEFILTER
            Console.WriteLine($"case #3: discarding");

#endif
            // 无法投影到该平面上，丢弃。
            return false;
        }

        // 第二步，判断“proj”是否位于 neighborNormal 与 triangleNormal 之间
        //
        //    /    neighborNormal
        //   /
        //  /
        //  -----  proj
        // \
        //  \
        //   \     triangleNormal
        Real f1 = proj % neighborNormal * cross;
        Real f2 = proj % triangleNormal * cross;

        bool between = f1 <= (Real)0.0 && f2 >= (Real)0.0;

        if (!between)
        {
            // 不在两者之间，吸附法线
            Real f3 = JVector.Dot(normal, neighborNormal);
            Real f4 = JVector.Dot(normal, triangleNormal);

            if (f3 > f4)
            {
#if DEBUG_EDGEFILTER
                Console.WriteLine($"case #2: adjusting; normal {normal} -> {neighborNormal}");

#endif
                if (!isSpeculative)
                {
                    penetration = 0;
                    pointA = pointB = midPoint;
                }

                normal = neighborNormal;
            }
            else
            {
#if DEBUG_EDGEFILTER
                Console.WriteLine($"case #2: adjusting; normal {normal} -> {triangleNormal}");
#endif
                if (!isSpeculative)
                {
                    penetration = 0;
                    pointA = pointB = midPoint;
                }

                normal = triangleNormal;
            }
        }

        return true;
    }
}
