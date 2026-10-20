/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using Jitter2.Collision.Shapes;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

/// <summary>
/// 表示两个凸形状之间的接触流形。
/// </summary>
/// <remarks>
/// 该流形由围绕碰撞法线采样的支撑点构建，
/// 随后缩减为适合求解器的小型接触点集。
/// </remarks>
public unsafe struct CollisionManifold
{
    // 我们用六个扰动后的支撑方向对每个形状采样，但在面向求解器的缩减步骤之前，
    // 采样多边形最多可有十二个顶点相交。
    private const int MaxSupportPoints = 6;
    private const int MaxManifoldPoints = 12;
    private const int SolverContactLimit = 4;
    private const Real DuplicatePointDistanceSq = (Real)0.001;

    // 布局：先是形状 A 上的 MaxManifoldPoints 个点，随后是形状 B 上与之匹配的点。
    private fixed Real manifoldData[MaxManifoldPoints * 2 * 3];

    // 为每个形状收集的采样支撑点数量。
    private int leftCount;
    private int rightCount;
    // 写入 manifoldData 的实际接触点对数量。
    private int manifoldCount;

    private const Real Sqrt3Over2 = (Real)0.86602540378;
    // 用于围绕接触法线采样支撑点的小扰动。
    private const Real Perturbation = (Real)0.01;

    // 法线周围的单位六边形。这些偏移量生成六个支撑方向。
    private static readonly Real[] hexagonVertices = [(Real)1.0, (Real)0.0, (Real)0.5, Sqrt3Over2, -(Real)0.5, Sqrt3Over2,
        -(Real)1.0, (Real)0.0, -(Real)0.5, -Sqrt3Over2, (Real)0.5, -Sqrt3Over2];

    /// <summary>
    /// 获取形状 A 上的接触点。
    /// </summary>
    public Span<JVector> ManifoldA => MemoryMarshal.CreateSpan(ref Unsafe.As<Real, JVector>(ref manifoldData[0]), manifoldCount);

    /// <summary>
    /// 获取形状 B 上的接触点。
    /// </summary>
    public Span<JVector> ManifoldB => MemoryMarshal.CreateSpan(ref Unsafe.As<Real, JVector>(ref manifoldData[MaxManifoldPoints * 3]), manifoldCount);

    /// <summary>
    /// 获取流形中的接触点数量。
    /// </summary>
    public readonly int Count => manifoldCount;

    // 为形状 A 添加一个支撑采样点，除非它与已保留的点实际上重合。
    private void PushLeft(Span<JVector> left, in JVector v)
    {
        if (leftCount > 0)
        {
            if ((left[0] - v).LengthSquared() < DuplicatePointDistanceSq) return;
        }

        if (leftCount > 1)
        {
            if ((left[leftCount - 1] - v).LengthSquared() < DuplicatePointDistanceSq) return;
        }

        left[leftCount++] = v;
    }

    // 为形状 B 添加一个支撑采样点，除非它与已保留的点实际上重合。
    private void PushRight(Span<JVector> right, in JVector v)
    {
        if (rightCount > 0)
        {
            if ((right[0] - v).LengthSquared() < DuplicatePointDistanceSq) return;
        }

        if (rightCount > 1)
        {
            if ((right[rightCount - 1] - v).LengthSquared() < DuplicatePointDistanceSq) return;
        }

        right[rightCount++] = v;
    }

    // 确定采样多边形相对于接触法线的环绕方向符号。
    // 如果面积接近零，说明采样退化成了线段或点。
    private static bool TryGetPolygonSign(ReadOnlySpan<JVector> polygon, int count, in JVector normal, out Real sign)
    {
        const Real epsilon = (Real)1e-6;

        Real area = (Real)0.0;
        JVector previous = polygon[count - 1];

        for (int i = 0; i < count; i++)
        {
            JVector current = polygon[i];
            area += JVector.Dot(previous % current, normal);
            previous = current;
        }

        if (MathR.Abs(area) <= epsilon)
        {
            sign = (Real)0.0;
            return false;
        }

        sign = area < (Real)0.0 ? -(Real)1.0 : (Real)1.0;
        return true;
    }

    // 标准的凸多边形内点测试，借助接触法线直接在三维中计算。
    private static bool ContainsPoint(ReadOnlySpan<JVector> polygon, int count, in JVector point,
        in JVector normal, Real sign)
    {
        const Real epsilon = (Real)1e-3;

        JVector previous = polygon[count - 1];

        for (int i = 0; i < count; i++)
        {
            JVector current = polygon[i];
            Real side = sign * JVector.Dot((current - previous) % (point - previous), normal);

            if (side < -epsilon) return false;

            previous = current;
        }

        return true;
    }

    // 用凸多边形的各个半空间裁剪线段。
    // 当某个采样特征塌缩为直线时，这就是多边形/线段的回退方案。
    private static int ClipSegmentAgainstPolygon(in JVector start, in JVector end,
        ReadOnlySpan<JVector> polygon, int polygonCount, in JVector normal, Real sign, Span<JVector> clipped)
    {
        const Real epsilon = (Real)1e-3;

        Real enter = (Real)0.0;
        Real exit = (Real)1.0;
        JVector delta = end - start;

        for (int i = 0; i < polygonCount; i++)
        {
            JVector edgeStart = polygon[i];
            JVector edgeEnd = polygon[(i + 1) % polygonCount];

            Real startSide = sign * JVector.Dot((edgeEnd - edgeStart) % (start - edgeStart), normal);
            Real endSide = sign * JVector.Dot((edgeEnd - edgeStart) % (end - edgeStart), normal);

            bool startInside = startSide >= -epsilon;
            bool endInside = endSide >= -epsilon;

            if (!startInside && !endInside) return 0;
            if (startInside && endInside) continue;

            Real denominator = startSide - endSide;
            if (MathR.Abs(denominator) <= epsilon) return 0;

            Real t = Math.Clamp(startSide / denominator, (Real)0.0, (Real)1.0);

            if (!startInside)
            {
                enter = MathR.Max(enter, t);
            }
            else
            {
                exit = MathR.Min(exit, t);
            }

            if (exit < enter) return 0;
        }

        clipped[0] = start + enter * delta;

        if ((start + exit * delta - clipped[0]).LengthSquared() < epsilon)
        {
            return 1;
        }

        clipped[1] = start + exit * delta;
        return 2;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    // 沿接触法线投影后的带符号标量二维叉积。
    private static Real CrossAlongNormal(in JVector left, in JVector right, in JVector normal)
    {
        return JVector.Dot(left % right, normal);
    }

    // 在接触平面内求两条线段的交点。
    // 同时处理常规相交情形和平行共线重叠情形。
    private static int IntersectSegments(in JVector startA, in JVector endA,
        in JVector startB, in JVector endB, in JVector normal,
        Span<JVector> clippedA, Span<JVector> clippedB)
    {
        const Real epsilon = (Real)1e-3;

        JVector deltaA = endA - startA;
        JVector deltaB = endB - startB;
        JVector offset = startB - startA;

        Real cross = CrossAlongNormal(deltaA, deltaB, normal);

        if (MathR.Abs(cross) <= epsilon)
        {
            if (MathR.Abs(CrossAlongNormal(offset, deltaA, normal)) > epsilon) return 0;

            Real lengthASquared = deltaA.LengthSquared();
            Real lengthBSquared = deltaB.LengthSquared();

            if (lengthASquared <= epsilon || lengthBSquared <= epsilon) return 0;

            Real parameterB0 = JVector.Dot(startB - startA, deltaA) / lengthASquared;
            Real parameterB1 = JVector.Dot(endB - startA, deltaA) / lengthASquared;

            Real enter = MathR.Max((Real)0.0, MathR.Min(parameterB0, parameterB1));
            Real exit = MathR.Min((Real)1.0, MathR.Max(parameterB0, parameterB1));

            if (exit < enter - epsilon) return 0;

            clippedA[0] = startA + enter * deltaA;

            Real parameterA0 = Math.Clamp(JVector.Dot(clippedA[0] - startB, deltaB) / lengthBSquared, (Real)0.0, (Real)1.0);
            clippedB[0] = startB + parameterA0 * deltaB;

            clippedA[1] = startA + exit * deltaA;

            if ((clippedA[1] - clippedA[0]).LengthSquared() <= epsilon) return 1;

            Real parameterA1 = Math.Clamp(JVector.Dot(clippedA[1] - startB, deltaB) / lengthBSquared, (Real)0.0, (Real)1.0);
            clippedB[1] = startB + parameterA1 * deltaB;

            return 2;
        }

        Real t = CrossAlongNormal(offset, deltaB, normal) / cross;
        Real u = CrossAlongNormal(offset, deltaA, normal) / cross;

        if (t < -epsilon || t > (Real)1.0 + epsilon ||
            u < -epsilon || u > (Real)1.0 + epsilon)
        {
            return 0;
        }

        t = Math.Clamp(t, (Real)0.0, (Real)1.0);
        u = Math.Clamp(u, (Real)0.0, (Real)1.0);

        clippedA[0] = startA + t * deltaA;
        clippedB[0] = startB + u * deltaB;

        return 1;
    }

    // 避免因收集路径重叠而重复插入同一个接触点。
    private static bool IsDuplicatePoint(ReadOnlySpan<JVector> manifold, int count, in JVector point)
    {
        for (int i = 0; i < count; i++)
        {
            if ((manifold[i] - point).LengthSquared() < DuplicatePointDistanceSq) return true;
        }

        return false;
    }

    // 登记一个几何锚点来自形状 A 的接触点。
    // 形状 B 上的匹配点通过沿碰撞法线滑动来重建。
    private static void AddPointOnA(Span<JVector> manifoldA, Span<JVector> manifoldB, ref int manifoldCount,
        in JVector point, in JVector pB, in JVector normal)
    {
        if (manifoldCount == MaxManifoldPoints) return;

        Real diff = JVector.Dot(point - pB, normal);
        if (diff < (Real)0.0) return;
        if (IsDuplicatePoint(manifoldA, manifoldCount, point)) return;

        manifoldA[manifoldCount] = point;
        manifoldB[manifoldCount++] = point - diff * normal;
    }

    // 登记一个几何锚点来自形状 B 的接触点。
    private static void AddPointOnB(Span<JVector> manifoldA, Span<JVector> manifoldB, ref int manifoldCount,
        in JVector point, in JVector pA, in JVector normal)
    {
        if (manifoldCount == MaxManifoldPoints) return;

        Real diff = JVector.Dot(point - pA, normal);
        if (diff > (Real)0.0) return;
        if (IsDuplicatePoint(manifoldB, manifoldCount, point)) return;

        manifoldB[manifoldCount] = point;
        manifoldA[manifoldCount++] = point - diff * normal;
    }

    // 登记一个两侧均来自显式求交计算的接触点。
    private static void AddPointPair(Span<JVector> manifoldA, Span<JVector> manifoldB, ref int manifoldCount,
        in JVector pointA, in JVector pointB, in JVector normal)
    {
        if (manifoldCount == MaxManifoldPoints) return;
        if (JVector.Dot(pointA - pointB, normal) < (Real)0.0) return;
        if (IsDuplicatePoint(manifoldA, manifoldCount, pointA)) return;
        if (IsDuplicatePoint(manifoldB, manifoldCount, pointB)) return;

        manifoldA[manifoldCount] = pointA;
        manifoldB[manifoldCount++] = pointB;
    }

    // 求解器只需要四个接触点。我们围绕质心对点排序，
    // 并保留四个均匀分布的采样点，使保留的集合覆盖接触区域。
    private static void ReduceManifold(Span<JVector> manifoldA, Span<JVector> manifoldB, ref int manifoldCount, in JVector normal)
    {
        if (manifoldCount <= SolverContactLimit) return;

        JVector centroid = JVector.Zero;

        for (int i = 0; i < manifoldCount; i++)
        {
            centroid += manifoldA[i];
        }

        centroid *= (Real)(1.0 / manifoldCount);

        JVector tangent1 = MathHelper.CreateOrthonormal(normal);
        JVector tangent2 = normal % tangent1;

        Span<double> angles = stackalloc double[MaxManifoldPoints];
        Span<int> order = stackalloc int[MaxManifoldPoints];

        for (int i = 0; i < manifoldCount; i++)
        {
            JVector delta = manifoldA[i] - centroid;
            angles[i] = StableMath.Atan2(JVector.Dot(delta, tangent2), JVector.Dot(delta, tangent1));
            order[i] = i;
        }

        for (int i = 1; i < manifoldCount; i++)
        {
            int current = order[i];
            double currentAngle = angles[current];
            int j = i - 1;

            while (j >= 0 && angles[order[j]] > currentAngle)
            {
                order[j + 1] = order[j];
                j--;
            }

            order[j + 1] = current;
        }

        Span<JVector> reducedA = stackalloc JVector[SolverContactLimit];
        Span<JVector> reducedB = stackalloc JVector[SolverContactLimit];

        for (int i = 0; i < SolverContactLimit; i++)
        {
            int selected = order[((2 * i + 1) * manifoldCount) / (2 * SolverContactLimit)];
            reducedA[i] = manifoldA[selected];
            reducedB[i] = manifoldB[selected];
        }

        reducedA.CopyTo(manifoldA);
        reducedB.CopyTo(manifoldB);
        manifoldCount = SolverContactLimit;
    }

    /// <summary>
    /// 在给定两个形状的变换和初始接触点的情况下，构建两者之间的接触流形。
    /// </summary>
    /// <typeparam name="Ta">支撑形状 A 的类型。</typeparam>
    /// <typeparam name="Tb">支撑形状 B 的类型。</typeparam>
    /// <param name="shapeA">第一个形状。</param>
    /// <param name="shapeB">第二个形状。</param>
    /// <param name="quaternionA">形状 A 的朝向。</param>
    /// <param name="quaternionB">形状 B 的朝向。</param>
    /// <param name="positionA">形状 A 的位置。</param>
    /// <param name="positionB">形状 B 的位置。</param>
    /// <param name="pA">形状 A 上的初始接触点。</param>
    /// <param name="pB">形状 B 上的初始接触点。</param>
    /// <param name="normal">碰撞法线（从 B 指向 A）。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    [SkipLocalsInit]
    public void BuildManifold<Ta, Tb>(Ta shapeA, Tb shapeB, in JQuaternion quaternionA, in JQuaternion quaternionB,
        in JVector positionA, in JVector positionB, in JVector pA, in JVector pB, in JVector normal)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // 重置
        leftCount = 0;
        rightCount = 0;
        manifoldCount = 0;

        // 围绕碰撞法线构建局部切向坐标系。
        JVector crossVector1 = MathHelper.CreateOrthonormal(normal);
        JVector crossVector2 = normal % crossVector1;

        // 在平移了 -positionA 的坐标系中运算，使中间量保持在原点附近。
        // 裁剪辅助函数中的绝对 epsilon 按形状尺寸缩放，而不受世界坐标偏移影响。
        JVector relPosB = positionB - positionA;
        JVector pAloc = pA - positionA;
        JVector pBloc = pB - positionA;

        Span<JVector> left = stackalloc JVector[MaxSupportPoints];
        Span<JVector> right = stackalloc JVector[MaxSupportPoints];

        // 按六边形模式围绕法线为两个形状采样支撑点。
        // 这近似得到当前有效的接触特征，而无需完整的裁剪流程。
        for (int e = 0; e < MaxSupportPoints; e++)
        {
            JVector ptNormal = normal + hexagonVertices[2 * e + 0] * Perturbation * crossVector1 +
                               hexagonVertices[2 * e + 1] * Perturbation * crossVector2;

            JVector.ConjugatedTransform(ptNormal, quaternionA, out JVector tmp);
            shapeA.SupportMap(tmp, out JVector np1);
            JVector.Transform(np1, quaternionA, out np1);
            PushLeft(left, np1);

            JVector.NegateInPlace(ref ptNormal);

            JVector.ConjugatedTransform(ptNormal, quaternionB, out tmp);
            shapeB.SupportMap(tmp, out JVector np2);
            JVector.Transform(np2, quaternionB, out np2);
            JVector.Add(np2, relPosB, out np2);
            PushRight(right, np2);
        }

        Span<JVector> mA = MemoryMarshal.CreateSpan(ref Unsafe.As<Real, JVector>(ref manifoldData[0]), MaxManifoldPoints);
        Span<JVector> mB = MemoryMarshal.CreateSpan(ref Unsafe.As<Real, JVector>(ref manifoldData[MaxManifoldPoints * 3]), MaxManifoldPoints);

        // 如果某个采样环的带符号面积非零，就把它当作多边形；否则它实际上
        // 是一条直线或一个点，由下面专门的回退情形处理。
        Real leftSign = (Real)0.0;
        Real rightSign = (Real)0.0;
        bool leftPolygon = leftCount > 2 && TryGetPolygonSign(left, leftCount, normal, out leftSign);
        bool rightPolygon = rightCount > 2 && TryGetPolygonSign(right, rightCount, normal, out rightSign);

        // 形状 A 提供多边形的多边形/点与多边形/线段情形。
        if (leftPolygon)
        {
            for (int e = 0; e < rightCount; e++)
            {
                JVector p = right[e];
                if (!ContainsPoint(left, leftCount, p, normal, leftSign)) continue;

                AddPointOnB(mA, mB, ref manifoldCount, p, pAloc, normal);
                if (manifoldCount == MaxManifoldPoints) goto Finalize;
            }

            if (rightCount == 2)
            {
                Span<JVector> clipped = stackalloc JVector[2];
                int clippedCount = ClipSegmentAgainstPolygon(right[0], right[1], left, leftCount, normal, leftSign, clipped);

                for (int i = 0; i < clippedCount; i++)
                {
                    AddPointOnB(mA, mB, ref manifoldCount, clipped[i], pAloc, normal);
                    if (manifoldCount == MaxManifoldPoints) goto Finalize;
                }
            }
        }

        // 角色互换的相同逻辑，这样我们也能选取形状 A 提供的锚点。
        if (rightPolygon)
        {
            for (int e = 0; e < leftCount; e++)
            {
                JVector p = left[e];
                if (!ContainsPoint(right, rightCount, p, normal, rightSign)) continue;

                AddPointOnA(mA, mB, ref manifoldCount, p, pBloc, normal);
                if (manifoldCount == MaxManifoldPoints) goto Finalize;
            }

            if (leftCount == 2)
            {
                Span<JVector> clipped = stackalloc JVector[2];
                int clippedCount = ClipSegmentAgainstPolygon(left[0], left[1], right, rightCount, normal, rightSign, clipped);

                for (int i = 0; i < clippedCount; i++)
                {
                    AddPointOnA(mA, mB, ref manifoldCount, clipped[i], pBloc, normal);
                    if (manifoldCount == MaxManifoldPoints) goto Finalize;
                }
            }
        }

        // 纯线段/线段流形。这里需要它，因为两侧都没有生成多边形。
        if (leftCount == 2 && rightCount == 2)
        {
            Span<JVector> clippedA = stackalloc JVector[2];
            Span<JVector> clippedB = stackalloc JVector[2];
            int clippedCount = IntersectSegments(left[0], left[1], right[0], right[1], normal, clippedA, clippedB);

            for (int i = 0; i < clippedCount; i++)
            {
                AddPointPair(mA, mB, ref manifoldCount, clippedA[i], clippedB[i], normal);
                if (manifoldCount == MaxManifoldPoints) goto Finalize;
            }
        }

        // 多边形/多边形情形：除了内部顶点外，还要收集显式的边交点。
        if (leftPolygon && rightPolygon)
        {
            Span<JVector> clippedA = stackalloc JVector[2];
            Span<JVector> clippedB = stackalloc JVector[2];

            for (int i = 0; i < leftCount; i++)
            {
                JVector startA = left[i];
                JVector endA = left[(i + 1) % leftCount];

                for (int j = 0; j < rightCount; j++)
                {
                    JVector startB = right[j];
                    JVector endB = right[(j + 1) % rightCount];

                    int clippedCount = IntersectSegments(startA, endA, startB, endB, normal, clippedA, clippedB);

                    for (int k = 0; k < clippedCount; k++)
                    {
                        AddPointPair(mA, mB, ref manifoldCount, clippedA[k], clippedB[k], normal);
                        if (manifoldCount == MaxManifoldPoints) goto Finalize;
                    }
                }
            }
        }

        // 围绕法线采样支撑点可能会留下一个偏离原始接触点的单点，尤其在曲面上。
        // 这个偏移是流形算法产生的伪影，可能让静止的球体开始滚动。如果
        // 我们最多只找到一个点，就改用原始的最深接触点对。
        if (manifoldCount <= 1)
        {
            mA[0] = pAloc;
            mB[0] = pBloc;
            manifoldCount = 1;
        }
        else if (manifoldCount < SolverContactLimit)
        {
            // 支撑点采样也可能使小块接触区发生偏移。若有空闲的求解器槽位，
            // 就把原始点对保留在最后，同时不丢弃任何采样得到的接触点。
            mA[manifoldCount] = pAloc;
            mB[manifoldCount] = pBloc;
            manifoldCount++;
        }

    Finalize:
        // 从原始流形候选点最终缩减为面向求解器的接触点集。
        ReduceManifold(mA, mB, ref manifoldCount, normal);

        // 将保留下来的接触点抬回世界空间。
        for (int i = 0; i < manifoldCount; i++)
        {
            mA[i] += positionA;
            mB[i] += positionA;
        }
    } // BuildManifold

    /// <summary>
    /// 使用两个刚体形状当前变换，构建它们之间的接触流形。
    /// </summary>
    /// <typeparam name="Ta">形状 A 的类型。</typeparam>
    /// <typeparam name="Tb">形状 B 的类型。</typeparam>
    /// <param name="shapeA">第一个刚体形状。</param>
    /// <param name="shapeB">第二个刚体形状。</param>
    /// <param name="pA">形状 A 上的初始接触点。</param>
    /// <param name="pB">形状 B 上的初始接触点。</param>
    /// <param name="normal">碰撞法线（从 B 指向 A）。</param>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    [SkipLocalsInit]
    public void BuildManifold<Ta, Tb>(Ta shapeA, Tb shapeB,
        in JVector pA, in JVector pB, in JVector normal) where Ta : RigidBodyShape where Tb : RigidBodyShape
    {
        BuildManifold(shapeA, shapeB, shapeA.RigidBody.Orientation, shapeB.RigidBody.Orientation,
            shapeA.RigidBody.Position, shapeB.RigidBody.Position, pA, pB, normal);
    }
}
