/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Jitter2.LinearMath;
using Vertex = Jitter2.Collision.MinkowskiDifference.Vertex;

namespace Jitter2.Collision;

/// <summary>
/// 为通过支撑函数定义的凸形状提供碰撞检测算法。
/// </summary>
/// <remarks>
/// <para>
/// 实现了用于距离查询的 GJK（Gilbert-Johnson-Keerthi）、用于快速重叠检测的 MPR（Minkowski Portal Refinement），
/// 以及用于穿透深度的 EPA（Expanding Polytope Algorithm）。
/// </para>
/// <para>
/// 与线程本地的求解器状态配合使用时，所有方法都是线程安全的。
/// </para>
/// </remarks>
public static class NarrowPhase
{
    private const Real NumericEpsilon = (Real)1e-16;

    // 约定：基于 GJK 和 MPR 的查询只报告几何上的是/否结果，不会暴露
    // 算法失败状态。EPA 是唯一可能无法收敛的步骤。若 EPA 失败，MPR 精化会回退
    // 到 MPR 的结果，而 'NarrowPhase.Collision' 将 false 记录为 EPA/收敛失败。
    private struct MprEpaSolver
    {
        private ConvexPolytope convexPolytope;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private bool SolveMprEpa<Ta, Tb>(in Ta supportA, in Tb supportB, in JQuaternion orientationB, in JVector positionB,
            ref JVector point1, ref JVector point2, ref JVector normal, ref Real penetration)
            where Ta : ISupportMappable where Tb : ISupportMappable
        {
            const Real collideEpsilon = (Real)1e-5;
            const int maxIter = 85;

            if (!convexPolytope.InitTetrahedron()) return false;

            int iter = 0;

            Unsafe.SkipInit(out ConvexPolytope.Triangle ctri);

            while (++iter < maxIter)
            {
                ctri = convexPolytope.GetClosestTriangle();

                JVector searchDir = ctri.ClosestToOrigin;
                Real searchDirSq = ctri.ClosestToOriginSq;

                if (!convexPolytope.OriginEnclosed) JVector.NegateInPlace(ref searchDir);
                if (ctri.ClosestToOriginSq < NumericEpsilon)
                {
                    searchDir = ctri.Normal;
                    searchDirSq = ctri.NormalSq;
                }

                MinkowskiDifference.Support(supportA, supportB, orientationB, positionB,
                    searchDir, out Vertex vertex);

                // 与 Collision 中对应的代码进行比较。
                Real deltaDist = JVector.Dot(ctri.ClosestToOrigin - vertex.V, searchDir);

                if (deltaDist * deltaDist <= collideEpsilon * collideEpsilon * searchDirSq)
                {
                    goto converged;
                }

                var expansion = convexPolytope.AddVertexDetailed(vertex);
                if (expansion == ConvexPolytope.AddVertexResult.NoHorizon) goto converged;
                if (expansion != ConvexPolytope.AddVertexResult.Added) return false;
            }

            Logger.Warning("{0}: EPA, Could not converge within {1} iterations.", nameof(NarrowPhase), maxIter);

            return false;

        converged:

            if (!convexPolytope.OriginEnclosed) return false;
            convexPolytope.CalculatePoints(ctri, out point1, out point2);

            normal = ctri.Normal * ((Real)1.0 / MathR.Sqrt(ctri.NormalSq));
            penetration = MathR.Sqrt(ctri.ClosestToOriginSq);

            return true;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
        public bool SolveMpr<Ta, Tb>(in Ta supportA, in Tb supportB, in JQuaternion orientationB,
            in JVector positionB, Real epaThreshold,
            out JVector pointA, out JVector pointB, out JVector normal, out Real penetration)
            where Ta : ISupportMappable where Tb : ISupportMappable
        {
            /*
            This method is adapted from XenoCollide.
            Modified from the original version.

            XenoCollide is available under the zlib license:

            XenoCollide Collision Detection and Physics Library
            Copyright (c) 2007-2014 Gary Snethen http://xenocollide.com

            This software is provided 'as-is', without any express or implied warranty.
            In no event will the authors be held liable for any damages arising
            from the use of this software.
            Permission is granted to anyone to use this software for any purpose,
            including commercial applications, and to alter it and redistribute it freely,
            subject to the following restrictions:

            1. The origin of this software must not be misrepresented; you must
            not claim that you wrote the original software. If you use this
            software in a product, an acknowledgment in the product documentation
            would be appreciated but is not required.
            2. Altered source versions must be plainly marked as such, and must
            not be misrepresented as being the original software.
            3. This notice may not be removed or altered from any source distribution.
            */
            const Real collideEpsilon = (Real)1e-5;
            const int maxIter = 34;

            Unsafe.SkipInit(out Vertex v0);
            Unsafe.SkipInit(out Vertex v1);
            Unsafe.SkipInit(out Vertex v2);
            Unsafe.SkipInit(out Vertex v3);
            Unsafe.SkipInit(out Vertex v4);

            Unsafe.SkipInit(out JVector temp1);
            Unsafe.SkipInit(out JVector temp2);
            Unsafe.SkipInit(out JVector temp3);

            penetration = (Real)0.0;

            MinkowskiDifference.GetCenter(supportA, supportB, orientationB, positionB, out v0);

            if (Math.Abs(v0.V.X) < NumericEpsilon &&
                Math.Abs(v0.V.Y) < NumericEpsilon &&
                Math.Abs(v0.V.Z) < NumericEpsilon)
            {
                // 任意方向都可以
                v0.V.X = (Real)1e-05;
            }

            JVector.Negate(v0.V, out normal);

            MinkowskiDifference.Support(supportA, supportB, orientationB, positionB, normal, out v1);

            pointA = v1.A;
            pointB = v1.B;

            if (JVector.Dot(v1.V, normal) <= (Real)0.0) return false;
            JVector.Cross(v1.V, v0.V, out normal);

            const Real sphericalEpsilon = (Real)1e-12;
            if (normal.LengthSquared() < sphericalEpsilon)
            {
                // 原点、v0 和 v1 构成一条直线。最可能的情况是
                // 两个球体发生碰撞。
                JVector.Subtract(v1.V, v0.V, out normal);
                JVector.NormalizeInPlace(ref normal);

                JVector.Subtract(v1.A, v1.B, out temp1);
                penetration = JVector.Dot(temp1, normal);

                return true;
            }

            MinkowskiDifference.Support(supportA, supportB, orientationB, positionB, normal, out v2);

            if (JVector.Dot(v2.V, normal) <= (Real)0.0) return false;

            // 判断原点位于平面 (v1.V, v0.V, v2.V) 的 + 侧还是 - 侧。
            JVector.Subtract(v1.V, v0.V, out temp1);
            JVector.Subtract(v2.V, v0.V, out temp2);
            JVector.Cross(temp1, temp2, out normal);

            Real dist = JVector.Dot(normal, v0.V);

            // 如果原点位于平面的 - 侧，则反转平面的方向
            if (dist > (Real)0.0)
            {
                JVector.Swap(ref v1.V, ref v2.V);
                JVector.Swap(ref v1.A, ref v2.A);
                JVector.Swap(ref v1.B, ref v2.B);
                JVector.Negate(normal, out normal);
            }

            int phase2 = 0;
            int phase1 = 0;
            bool hit = false;

            // 阶段一：确定一个门户（portal）
            while (true)
            {
                if (phase1 > maxIter) return false;

                phase1++;

                MinkowskiDifference.Support(supportA, supportB, orientationB, positionB, normal, out v3);

                if (JVector.Dot(v3.V, normal) <= (Real)0.0)
                {
                    return false;
                }

                // 如果原点位于 (v1.V,v0.V,v3.V) 之外，则剔除 v2.V 并继续循环
                JVector.Cross(v1.V, v3.V, out temp1);
                if (JVector.Dot(temp1, v0.V) < (Real)0.0)
                {
                    v2 = v3;
                    JVector.Subtract(v1.V, v0.V, out temp1);
                    JVector.Subtract(v3.V, v0.V, out temp2);
                    JVector.Cross(temp1, temp2, out normal);
                    continue;
                }

                // 如果原点位于 (v3.V,v0.V,v2.V) 之外，则剔除 v1.V 并继续循环
                JVector.Cross(v3.V, v2.V, out temp1);
                if (JVector.Dot(temp1, v0.V) < (Real)0.0)
                {
                    v1 = v3;
                    JVector.Subtract(v3.V, v0.V, out temp1);
                    JVector.Subtract(v2.V, v0.V, out temp2);
                    JVector.Cross(temp1, temp2, out normal);
                    continue;
                }

                break;
            }

            // 阶段二：精化门
            // 现在我们位于一个楔形内部……
            while (true)
            {
                phase2++;

                // 计算楔形面的法线
                JVector.Subtract(v2.V, v1.V, out temp1);
                JVector.Subtract(v3.V, v1.V, out temp2);
                JVector.Cross(temp1, temp2, out normal);

                // normal.Normalize();
                Real normalSq = normal.LengthSquared();

                // 这会发生吗？？？能否更干净地处理？
                if (normalSq < NumericEpsilon)
                {
                    // 原为：return true;
                    // 最好还是不要返回碰撞
                    Debug.Assert(false, "MPR: This should not happen.");
                    return false;
                }

                if (!hit)
                {
                    // 计算原点到楔形面的距离
                    Real d = JVector.Dot(normal, v1.V);
                    // 如果原点位于楔形内部，则说明命中
                    hit = d >= 0;
                }

                MinkowskiDifference.Support(supportA, supportB, orientationB, positionB, normal, out v4);

                JVector.Subtract(v4.V, v3.V, out temp3);
                Real delta = JVector.Dot(temp3, normal);
                penetration = JVector.Dot(v4.V, normal);

                // 如果边界足够薄、原点位于新发现顶点的支撑平面之外，或者已达到最大迭代次数，
                // 那么就可以终止
                if (delta * delta <= collideEpsilon * collideEpsilon * normalSq || penetration <= (Real)0.0 ||
                    phase2 > maxIter)
                {
                    if (hit)
                    {
                        Real invnormal = (Real)1.0 / MathR.Sqrt(normalSq);

                        penetration *= invnormal;

                        if (penetration > epaThreshold)
                        {
                            convexPolytope.InitHeap();
                            convexPolytope.GetVertex(0) = v0;
                            convexPolytope.GetVertex(1) = v1;
                            convexPolytope.GetVertex(2) = v2;
                            convexPolytope.GetVertex(3) = v3;

                            // 如果 EPA 失败，它不会设置任何结果数据。我们继续使用 MPR 的数据。
                            if (SolveMprEpa(supportA, supportB, orientationB, positionB,
                                    ref pointA, ref pointB, ref normal, ref penetration)) return true;
                        }

                        normal *= invnormal;

                        // 计算原点的重心坐标
                        JVector.Cross(v1.V, temp1, out temp3);
                        Real gamma = JVector.Dot(temp3, normal) * invnormal;
                        JVector.Cross(temp2, v1.V, out temp3);
                        Real beta = JVector.Dot(temp3, normal) * invnormal;
                        Real alpha = (Real)1.0 - gamma - beta;

                        pointA = alpha * v1.A + beta * v2.A + gamma * v3.A;
                        pointB = alpha * v1.B + beta * v2.B + gamma * v3.B;
                    }

                    return hit;
                }

                // 计算四面体的分割面 (v4.V, v0.V, v3.V)
                JVector.Cross(v4.V, v0.V, out temp1);
                Real dot = JVector.Dot(temp1, v1.V);

                if (dot >= (Real)0.0)
                {
                    dot = JVector.Dot(temp1, v2.V);

                    if (dot >= (Real)0.0)
                    {
                        v1 = v4; // 位于 d1 内 & 位于 d2 内 -> 剔除 v1.V
                    }
                    else
                    {
                        v3 = v4; // 位于 d1 内 & 位于 d2 外 -> 剔除 v3.V
                    }
                }
                else
                {
                    dot = JVector.Dot(temp1, v3.V);

                    if (dot >= (Real)0.0)
                    {
                        v2 = v4; // 位于 d1 外 & 位于 d3 内 -> 剔除 v2.V
                    }
                    else
                    {
                        v1 = v4; // 位于 d1 外 & 位于 d3 外 -> 剔除 v1.V
                    }
                }
            }
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public bool Collision<Ta, Tb>(in Ta supportA, in Tb supportB, in JQuaternion orientationB, in JVector positionB,
            out JVector point1, out JVector point2, out JVector normal, out Real penetration)
            where Ta : ISupportMappable where Tb : ISupportMappable
        {
            const Real collideEpsilon = (Real)1e-4;
            const int maxIter = 85;

            MinkowskiDifference.GetCenter(supportA, supportB, orientationB, positionB, out Vertex centerVertex);
            JVector center = centerVertex.V;

            convexPolytope.InitHeap();
            if (!convexPolytope.InitTetrahedron(center))
            {
                point1 = point2 = normal = JVector.Zero;
                penetration = 0;
                return false;
            }

            int iter = 0;

            Unsafe.SkipInit(out ConvexPolytope.Triangle ctri);

            while (++iter < maxIter)
            {
                ctri = convexPolytope.GetClosestTriangle();

                JVector searchDir = ctri.ClosestToOrigin;
                Real searchDirSq = ctri.ClosestToOriginSq;

                if (!convexPolytope.OriginEnclosed) JVector.NegateInPlace(ref searchDir);

                if (ctri.ClosestToOriginSq < NumericEpsilon)
                {
                    searchDir = ctri.Normal;
                    searchDirSq = ctri.NormalSq;
                }

                MinkowskiDifference.Support(supportA, supportB, orientationB, positionB, searchDir, out Vertex vertex);

                // 我们能否通过添加新顶点来进一步“扩展”凸包？
                //
                // v = Vertices[vPointer]（支撑点）
                // c = Triangles[Head].ClosestToOrigin
                // s = searchDir
                //
                // abs(dot(c - v, s)) / len(s) < e <=> [dot(c - v, s)]^2 = e*e*s^2
                Real deltaDist = JVector.Dot(ctri.ClosestToOrigin - vertex.V, searchDir);

                if (deltaDist * deltaDist <= collideEpsilon * collideEpsilon * searchDirSq)
                {
                    goto converged;
                }

                var expansion = convexPolytope.AddVertexDetailed(vertex);
                if (expansion == ConvexPolytope.AddVertexResult.NoHorizon) goto converged;
                if (expansion != ConvexPolytope.AddVertexResult.Added)
                {
                    point1 = point2 = normal = JVector.Zero;
                    penetration = 0;
                    return false;
                }
            }

            point1 = point2 = normal = JVector.Zero;
            penetration = (Real)0.0;

            Logger.Warning("{0}: EPA, Could not converge within {1} iterations.", nameof(NarrowPhase), maxIter);

            return false;

        converged:

            convexPolytope.CalculatePoints(ctri, out point1, out point2);

            penetration = MathR.Sqrt(ctri.ClosestToOriginSq);
            if (!convexPolytope.OriginEnclosed) penetration *= -(Real)1.0;

            if (MathR.Abs(penetration) > NumericEpsilon) normal = ctri.ClosestToOrigin * ((Real)1.0 / penetration);
            else normal = ctri.Normal * ((Real)1.0 / MathR.Sqrt(ctri.NormalSq));

            return true;
        }
    }

    // ------------------------------------------------------------------------------------------------------------
    [ThreadStatic] private static MprEpaSolver _solver;

    /// <summary>
    /// 检查点是否位于形状内部。
    /// </summary>
    /// <param name="support">表示该形状的支撑映射。</param>
    /// <param name="point">要检查的点。</param>
    /// <returns>如果点包含在形状内则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool PointTest<Ta>(in Ta support, in JVector point) where Ta : ISupportMappable
    {
        const Real collideEpsilon = (Real)1e-4;
        const int maxIter = 34;

        JVector x = point;

        support.GetCenter(out var center);
        JVector v = x - center;

        Unsafe.SkipInit(out SimplexSolver simplexSolver);
        simplexSolver.Reset();

        int iter = maxIter;

        Real distSq = v.LengthSquared();

        while (distSq > collideEpsilon * collideEpsilon && iter-- != 0)
        {
            support.SupportMap(v, out JVector p);
            JVector.Subtract(x, p, out JVector w);

            Real vw = JVector.Dot(v, w);

            if (vw >= (Real)0.0)
            {
                return false;
            }

            if (!simplexSolver.AddVertex(w, out v))
            {
                goto converged;
            }

            distSq = v.LengthSquared();
        }

    converged:

        return true;
    }

    /// <summary>
    /// 检查点是否位于形状内部。
    /// </summary>
    /// <param name="support">表示该形状的支撑映射。</param>
    /// <param name="orientation">形状的朝向。</param>
    /// <param name="position">形状的位置。</param>
    /// <param name="point">要检查的点。</param>
    /// <returns>如果点包含在形状内则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool PointTest<Ta>(in Ta support, in JMatrix orientation,
        in JVector position, in JVector point) where Ta : ISupportMappable
    {
        JVector transformedOrigin = JVector.TransposedTransform(point - position, orientation);
        return PointTest(support, transformedOrigin);
    }

    /// <summary>
    /// 对形状执行一次射线投射。
    /// </summary>
    /// <param name="support">形状的支撑函数。</param>
    /// <param name="orientation">形状在世界空间中的朝向。</param>
    /// <param name="position">形状在世界空间中的位置。</param>
    /// <param name="origin">射线的起点。</param>
    /// <param name="direction">射线的方向；无需归一化。</param>
    /// <param name="lambda">指定射线的命中点，计算方式为 'origin + lambda * direction'。
    /// 如果起点位于形状内部则为零；如果射线未命中则为 <see cref="Real.PositiveInfinity"/>。</param>
    /// <param name="normal">
    /// 垂直于表面、指向外部的归一化法线向量。如果射线未命中或
    /// 射线起点位于形状内部则为零。
    /// </param>
    /// <returns>如果射线与形状相交则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool RayCast<Ta>(in Ta support, in JQuaternion orientation,
        in JVector position, in JVector origin, in JVector direction, out Real lambda, out JVector normal)
        where Ta : ISupportMappable
    {
        // 将射线旋转到 bodyA 的参考系中……
        JVector transformedDir = JVector.ConjugatedTransform(direction, orientation);
        JVector transformedOrigin = JVector.ConjugatedTransform(origin - position, orientation);

        bool result = RayCast(support, transformedOrigin, transformedDir, out lambda, out normal);

        // ……再旋转回来。
        JVector.Transform(normal, orientation, out normal);

        return result;
    }

    /// <summary>
    /// 对形状执行一次射线投射。
    /// </summary>
    /// <param name="support">形状的支撑函数。</param>
    /// <param name="origin">射线的起点。</param>
    /// <param name="direction">射线的方向；无需归一化。</param>
    /// <param name="lambda">指定射线的命中点，计算方式为 'origin + lambda * direction'。
    /// 如果起点位于形状内部则为零；如果射线未命中则为 <see cref="Real.PositiveInfinity"/>。</param>
    /// <param name="normal">
    /// 垂直于表面、指向外部的归一化法线向量。如果射线未命中或
    /// 射线起点位于形状内部则为零。
    /// </param>
    /// <returns>如果射线与形状相交则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool RayCast<Ta>(in Ta support, in JVector origin, in JVector direction,
        out Real lambda, out JVector normal) where Ta : ISupportMappable
    {
        const Real collideEpsilon = (Real)1e-4;
        const int maxIter = 34;

        normal = JVector.Zero;
        lambda = (Real)0.0;

        JVector r = direction;
        JVector x = origin;

        support.GetCenter(out var center);
        JVector v = x - center;

        Unsafe.SkipInit(out SimplexSolver simplexSolver);
        simplexSolver.Reset();

        int iter = maxIter;

        Real distSq = v.LengthSquared();

        while (distSq > collideEpsilon * collideEpsilon && iter-- != 0)
        {
            support.SupportMap(v, out JVector p);

            JVector.Subtract(x, p, out JVector w);

            Real vdotW = JVector.Dot(v, w);

            if (vdotW > (Real)0.0)
            {
                Real vdotR = JVector.Dot(v, r);

                if (vdotR >= -NumericEpsilon)
                {
                    lambda = Real.PositiveInfinity;
                    return false;
                }

                lambda -= vdotW / vdotR;

                JVector.Multiply(r, lambda, out x);
                JVector.Add(origin, x, out x);
                JVector.Subtract(x, p, out w);
                normal = v;
            }

            if (!simplexSolver.AddVertex(w, out v))
            {
                goto converged;
            }

            distSq = v.LengthSquared();
        }

    converged:

        normal = JVector.NormalizeSafe(in normal, NumericEpsilon);
        return true;
    }

    /// <summary>
    /// 判断两个凸形状是否重叠，并为重叠和分离两种情况提供详细信息。
    /// 它假定支撑形状 A 位于原点位置且未旋转。
    /// 在内部，该方法使用 Expanding Polytope Algorithm（EPA）来收集碰撞信息。
    /// </summary>
    /// <param name="supportA">形状 A 的支撑函数。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationB">形状 B 的朝向。</param>
    /// <param name="positionB">形状 B 的位置。</param>
    /// <param name="pointA">
    /// 重叠情况下：形状 A 位于形状 B 内部的最深点；分离情况下：
    /// 形状 A 上距形状 B 最近的点。
    /// </param>
    /// <param name="pointB">
    /// 重叠情况下：形状 B 位于形状 A 内部的最深点；分离情况下：
    /// 形状 B 上距形状 A 最近的点。
    /// </param>
    /// <param name="normal">
    /// 归一化的碰撞法线。它与 pointA - pointB 的点积给出有符号穿透深度：
    /// 重叠时为正，分离时为负。当两点重合时该值仍然有定义。
    /// </param>
    /// <param name="penetration">有符号穿透深度：重叠时为正，分离时为负。</param>
    /// <returns>
    /// 如果算法成功完成则返回 true，否则返回 false。若算法收敛失败，
    /// 碰撞信息会恢复为该类型的默认值。
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Collision<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationB, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real penetration)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // ……执行碰撞检测……
        bool success = _solver.Collision(supportA, supportB, orientationB, positionB,
            out pointA, out pointB, out normal, out penetration);

        return success;
    }

    /// <summary>
    /// 判断两个凸形状是否重叠，并为重叠和分离两种情况提供详细信息。
    /// 在内部，该方法使用 Expanding Polytope Algorithm（EPA）来收集碰撞信息。
    /// </summary>
    /// <param name="supportA">形状 A 的支撑函数。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationA">形状 A 在世界空间中的朝向。</param>
    /// <param name="orientationB">形状 B 在世界空间中的朝向。</param>
    /// <param name="positionA">形状 A 在世界空间中的位置。</param>
    /// <param name="positionB">形状 B 在世界空间中的位置。</param>
    /// <param name="pointA">
    /// 重叠情况下：形状 A 位于形状 B 内部的最深点；分离情况下：
    /// 形状 A 上距形状 B 最近的点。
    /// </param>
    /// <param name="pointB">
    /// 重叠情况下：形状 B 位于形状 A 内部的最深点；分离情况下：
    /// 形状 B 上距形状 A 最近的点。
    /// </param>
    /// <param name="normal">
    /// 归一化的碰撞法线。它与 pointA - pointB 的点积给出有符号穿透深度：
    /// 重叠时为正，分离时为负。当两点重合时该值仍然有定义。
    /// </param>
    /// <param name="penetration">有符号穿透深度：重叠时为正，分离时为负。</param>
    /// <returns>
    /// 如果算法成功完成则返回 true，否则返回 false。若算法收敛失败，
    /// 碰撞信息会恢复为该类型的默认值。
    /// </returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Collision<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real penetration)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // 旋转到 bodyA 的参考系中……
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out JQuaternion orientation);
        JVector.Subtract(positionB, positionA, out JVector position);
        JVector.ConjugatedTransform(position, orientationA, out position);

        // ……执行碰撞检测……
        bool success = _solver.Collision(supportA, supportB, orientation, position,
            out pointA, out pointB, out normal, out penetration);

        // ……再旋转回来。多次调用支撑函数时，这有望节省一些矩阵向量乘法。
        JVector.Transform(pointA, orientationA, out pointA);
        JVector.Add(pointA, positionA, out pointA);
        JVector.Transform(pointB, orientationA, out pointB);
        JVector.Add(pointB, positionA, out pointB);
        JVector.Transform(normal, orientationA, out normal);

        return success;
    }

    /// <summary>
    /// 为不重叠的形状提供距离和最近点。它
    /// 假定支撑形状 A 位于原点位置且未旋转。
    /// </summary>
    /// <param name="supportA">形状 A 的支撑函数。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationB">形状 B 在世界空间中的朝向。</param>
    /// <param name="positionB">形状 B 在世界空间中的位置。</param>
    /// <param name="pointA">形状 A 上的最近点。重叠情况下无定义。</param>
    /// <param name="pointB">形状 B 上的最近点。重叠情况下无定义。</param>
    /// <param name="normal">从形状 A 指向形状 B 的单位方向。重叠情况下无定义。</param>
    /// <param name="distance">分离形状之间的距离。若形状重叠则为零。</param>
    /// <returns>如果形状不重叠且可以提供距离信息，
    /// 则返回 true。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Distance<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationB, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real distance)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // ……执行重叠检测……
        const Real collideEpsilon = (Real)1e-4;
        const int maxIter = 34;

        Unsafe.SkipInit(out SimplexSolverAB simplexSolver);
        simplexSolver.Reset();

        int iter = maxIter;

        MinkowskiDifference.GetCenter(supportA, supportB, orientationB, positionB, out Vertex center);

        JVector v = center.V;
        Real distSq = v.LengthSquared();

        while (iter-- != 0)
        {
            if (distSq < collideEpsilon * collideEpsilon) goto ret_false;

            MinkowskiDifference.Support(supportA, supportB, orientationB, positionB, -v, out var w);

            Real deltaDist = JVector.Dot(v - w.V, v);
            if (deltaDist * deltaDist < collideEpsilon * collideEpsilon * distSq)
            {
                break;
            }

            if (!simplexSolver.AddVertex(w, out v)) goto ret_false;

            distSq = v.LengthSquared();
        }

        distance = MathR.Sqrt(distSq);
        normal = v * (-(Real)1.0 / distance);
        simplexSolver.GetClosest(out pointA, out pointB);
        return true;

    ret_false:

        distance = (Real)0.0;
        normal = JVector.Zero;
        simplexSolver.GetClosest(out pointA, out pointB);
        return false;
    }

    /// <summary>
    /// 为不重叠的形状提供距离和最近点。
    /// </summary>
    /// <param name="supportA">形状 A 的支撑函数。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationA">形状 A 在世界空间中的朝向。</param>
    /// <param name="orientationB">形状 B 在世界空间中的朝向。</param>
    /// <param name="positionA">形状 A 在世界空间中的位置。</param>
    /// <param name="positionB">形状 B 在世界空间中的位置。</param>
    /// <param name="pointA">形状 A 上的最近点。重叠情况下无定义。</param>
    /// <param name="pointB">形状 B 上的最近点。重叠情况下无定义。</param>
    /// <param name="normal">从形状 A 指向形状 B 的单位方向。重叠情况下无定义。</param>
    /// <param name="distance">分离形状之间的距离。若形状重叠则为零。</param>
    /// <returns>如果形状不重叠且可以提供距离信息，
    /// 则返回 true。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Distance<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real distance)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // 旋转到 bodyA 的参考系中……
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out JQuaternion orientation);
        JVector.Subtract(positionB, positionA, out JVector position);
        JVector.ConjugatedTransform(position, orientationA, out position);

        // ……执行距离检测……
        bool result = Distance(supportA, supportB, orientation, position, out pointA, out pointB, out normal, out distance);

        // ……再旋转回来。当支撑函数被多次调用时，这种方法可能节省一些矩阵向量乘法。
        JVector.Transform(pointA, orientationA, out pointA);
        JVector.Add(pointA, positionA, out pointA);
        JVector.Transform(pointB, orientationA, out pointB);
        JVector.Add(pointB, positionA, out pointB);
        JVector.Transform(normal, orientationA, out normal);

        return result;
    }

    /// <summary>
    /// 执行重叠检测。它假定支撑形状 A 位于
    /// 原点位置且未旋转。
    /// </summary>
    /// <param name="supportA">形状 A 的支撑函数。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationB">形状 B 在世界空间中的朝向。</param>
    /// <param name="positionB">形状 B 在世界空间中的位置。</param>
    /// <returns>如果形状重叠则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Overlap<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationB, in JVector positionB) where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // ……执行重叠检测……
        const Real collideEpsilon = (Real)1e-4;
        const int maxIter = 34;

        Unsafe.SkipInit(out SimplexSolverAB simplexSolver);
        simplexSolver.Reset();

        int iter = maxIter;

        MinkowskiDifference.GetCenter(supportA, supportB, orientationB, positionB, out Vertex center);

        JVector v = center.V;
        Real distSq = v.LengthSquared();

        while (distSq > collideEpsilon * collideEpsilon && iter-- != 0)
        {
            MinkowskiDifference.Support(supportA, supportB, orientationB, positionB, -v, out var w);
            Real vw = JVector.Dot(v, w.V);
            if (vw >= (Real)0.0)
                return false;
            if (!simplexSolver.AddVertex(w, out v)) return true;
            distSq = v.LengthSquared();
        }

        return true;
    }

    /// <summary>
    /// 执行重叠检测。
    /// </summary>
    /// <param name="supportA">形状 A 的支撑函数。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationA">形状 A 在世界空间中的朝向。</param>
    /// <param name="orientationB">形状 B 在世界空间中的朝向。</param>
    /// <param name="positionA">形状 A 在世界空间中的位置。</param>
    /// <param name="positionB">形状 B 在世界空间中的位置。</param>
    /// <returns>如果形状重叠则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Overlap<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // 旋转到 bodyA 的参考系中……
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out JQuaternion orientation);
        JVector.Subtract(positionB, positionA, out JVector position);
        JVector.ConjugatedTransform(position, orientationA, out position);

        // ……执行重叠检测……
        return Overlap(supportA, supportB, orientation, position);
    }

    /// <summary>
    /// 如果 MPR 报告的穿透深度大于此值，我们就不信任 MPR 已找到全局最小值，
    /// 而是执行一次 EPA 运行。
    /// </summary>
    private const Real EpaPenetrationThreshold = (Real)0.02;

    /// <summary>
    /// 检测两个凸形状是否重叠，并为重叠的形状提供详细的碰撞信息。
    /// 在内部，该方法利用 Minkowski Portal Refinement（MPR）来获取碰撞信息。
    /// 尽管 MPR 并不精确，但它给出穿透深度的严格上界。如果该上界超过
    /// 预定义的阈值，则使用 Expanding Polytope Algorithm（EPA）进一步精化结果。
    /// </summary>
    /// <param name="supportA">形状 A 的支撑函数。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationA">形状 A 在世界空间中的朝向。</param>
    /// <param name="orientationB">形状 B 在世界空间中的朝向。</param>
    /// <param name="positionA">形状 A 在世界空间中的位置。</param>
    /// <param name="positionB">形状 B 在世界空间中的位置。</param>
    /// <param name="pointA">形状 A 位于形状 B 内部的最深点。</param>
    /// <param name="pointB">形状 B 位于形状 A 内部的最深点。</param>
    /// <param name="normal">
    /// 从 pointB 指向 pointA 的归一化碰撞法线。即使 pointA 与 pointB 重合，该法线仍然有定义，
    /// 它表示形状必须以最小距离（由穿透深度决定）分离、以避免重叠的方向。
    /// </param>
    /// <param name="penetration">穿透深度。</param>
    /// <param name="epaThreshold">穿透深度阈值，超过该值时使用 EPA 精化 MPR 结果。</param>
    /// <returns>如果形状重叠（碰撞）则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool MprEpa<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real penetration,
        Real epaThreshold = EpaPenetrationThreshold)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // 旋转到 bodyA 的参考系中……
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out JQuaternion orientation);
        JVector.Subtract(positionB, positionA, out JVector position);
        JVector.ConjugatedTransform(position, orientationA, out position);

        // ……执行碰撞检测……
        bool res = _solver.SolveMpr(supportA, supportB, orientation, position,
            epaThreshold, out pointA, out pointB, out normal, out penetration);

        // ……再旋转回来。当支撑函数被多次调用时，这种方法可能节省一些矩阵向量乘法。
        JVector.Transform(pointA, orientationA, out pointA);
        JVector.Add(pointA, positionA, out pointA);
        JVector.Transform(pointB, orientationA, out pointB);
        JVector.Add(pointB, positionA, out pointB);
        JVector.Transform(normal, orientationA, out normal);

        return res;
    }

    /// <summary>
    /// 检测两个凸形状是否重叠，并为重叠的形状提供详细的碰撞信息。
    /// 它假定支撑形状 A 位于原点位置且未旋转。
    /// 在内部，该方法利用 Minkowski Portal Refinement（MPR）来获取碰撞信息。
    /// 尽管 MPR 并不精确，但它给出穿透深度的严格上界。如果该上界
    /// 超过预定义的阈值，则使用 Expanding Polytope Algorithm（EPA）进一步精化结果。
    /// </summary>
    /// <param name="supportA">形状 A 的支撑函数。</param>
    /// <param name="supportB">形状 B 的支撑函数。</param>
    /// <param name="orientationB">形状 B 的朝向。</param>
    /// <param name="positionB">形状 B 的位置。</param>
    /// <param name="pointA">形状 A 位于形状 B 内部的最深点。</param>
    /// <param name="pointB">形状 B 位于形状 A 内部的最深点。</param>
    /// <param name="normal">
    /// 从 pointB 指向 pointA 的归一化碰撞法线。即使 pointA 与 pointB 重合，该法线仍然有定义，
    /// 它表示形状必须以最小距离（由穿透深度决定）分离、以避免重叠的方向。
    /// </param>
    /// <param name="penetration">穿透深度。</param>
    /// <param name="epaThreshold">穿透深度阈值，超过该值时使用 EPA 精化 MPR 结果。</param>
    /// <returns>如果形状重叠（碰撞）则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool MprEpa<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationB, in JVector positionB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real penetration,
        Real epaThreshold = EpaPenetrationThreshold)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        return _solver.SolveMpr(supportA, supportB, orientationB, positionB, epaThreshold, out pointA, out pointB, out normal, out penetration);
    }

    /// <summary>
    /// 计算两个形状在世界空间中的撞击时间（TOI）和碰撞点，已知其线速度
    /// <paramref name="sweepA"/> 和 <paramref name="sweepB"/> 以及角速度
    /// <paramref name="sweepAngularA"/> 和 <paramref name="sweepAngularB"/>。
    /// </summary>
    /// <param name="supportA">形状 A。</param>
    /// <param name="supportB">形状 B。</param>
    /// <param name="orientationA">形状 A 在世界空间中的朝向。</param>
    /// <param name="orientationB">形状 B 在世界空间中的朝向。</param>
    /// <param name="positionA">形状 A 在世界空间中的位置。</param>
    /// <param name="positionB">形状 B 在世界空间中的位置。</param>
    /// <param name="sweepA">形状 A 的线速度。</param>
    /// <param name="sweepB">形状 B 的线速度。</param>
    /// <param name="sweepAngularA">形状 A 的角速度。</param>
    /// <param name="sweepAngularB">形状 B 的角速度。</param>
    /// <param name="extentA">形状 A 的包围球半径。用于限定角位移的贡献。</param>
    /// <param name="extentB">形状 B 的包围球半径。用于限定角位移的贡献。</param>
    /// <param name="pointA">扫掠起点处形状 A 在世界空间中的碰撞点。</param>
    /// <param name="pointB">扫掠起点处形状 B 在世界空间中的碰撞点。</param>
    /// <param name="normal">
    /// 撞击时刻在世界空间中的归一化碰撞法线（由 A 指向 B）。如果形状已经重叠
    /// 或未命中则为零。
    /// </param>
    /// <param name="lambda">撞击时间。如果未检测到命中则为 <see cref="Real.PositiveInfinity"/>。如果形状重叠则为零。</param>
    /// <returns>如果形状命中或已经重叠则返回 true，否则返回 false。</returns>
    /// <remarks>
    /// 使用保守推进（conservative advancement）进行连续碰撞检测。由于线性运动近似
    /// 和距离梯度估计的局限，在某些边界情况下可能无法收敛到正确的 TOI
    /// 和碰撞点。
    /// </remarks>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Sweep<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB,
        in JVector sweepA, in JVector sweepB,
        in JVector sweepAngularA, in JVector sweepAngularB,
        in Real extentA, in Real extentB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        const Real collideEpsilon = (Real)1e-4;
        const int maxIter = 64;

        Real maxAngularSpeed = extentA * sweepAngularA.Length() + extentB * sweepAngularB.Length();
        Real combinedRadius = extentA + extentB;

        JVector posA = positionA;
        JVector posB = positionB;

        JQuaternion oriA = orientationA;
        JQuaternion oriB = orientationB;

        lambda = 0;

        int iter = 0;

        JQuaternion sweepAngularDeltaA;
        JQuaternion sweepAngularDeltaB;

        Distance(supportA, supportB, oriA, oriB, posA, posB, out pointA, out pointB, out normal, out var distance);

        if (distance < collideEpsilon)
        {
            // 在时间 0 时我们已经重叠（或几乎重叠）。
            // 这种情况下 Sweep 函数应返回 true；normal 和 lambda 被设为零。
            return true;
        }

        while (true)
        {
            Real sweepLinearProj = JVector.Dot(normal, sweepA - sweepB);
            Real sweepLen = sweepLinearProj + maxAngularSpeed;

            if (sweepLen < NumericEpsilon || (sweepLinearProj < 0 && distance > combinedRadius))
            {
                normal = JVector.Zero;
                lambda = Real.PositiveInfinity;
                return false;
            }

            Real lambdaStep = distance / sweepLen;

            lambda += lambdaStep;

            Debug.Assert(!Real.IsNaN(lambda));

            sweepAngularDeltaA = MathHelper.RotationQuaternion(sweepAngularA, lambda);
            sweepAngularDeltaB = MathHelper.RotationQuaternion(sweepAngularB, lambda);

            oriA = sweepAngularDeltaA * orientationA;
            oriB = sweepAngularDeltaB * orientationB;

            posA = positionA + sweepA * lambda;
            posB = positionB + sweepB * lambda;

            if (iter++ > maxIter) break;

            bool hasDistance = Distance(supportA, supportB, oriA, oriB, posA, posB, out pointA, out pointB, out JVector newNormal, out distance);

            // 这里我们处境有点棘手。
            // 如果推进后的形状略有重叠（Distance 返回 false；当单纯形求解器包含原点，
            // 或单纯形上的最近点离原点足够近时都会发生），
            // 我们拥有有效的 posA 和 posB 信息，但法线没有良好定义。因此保留旧法线。
            if (hasDistance) normal = newNormal;

            if (distance < collideEpsilon)
                break;
        }

        // 在时间 lambda 处找到了世界空间中的命中点。变换回时间 0。
        var linearTransformationA = sweepA * lambda;
        var linearTransformationB = sweepB * lambda;

        var deltaA = pointA - posA;
        var deltaB = pointB - posB;

        pointA -= linearTransformationA + (deltaA - JVector.ConjugatedTransform(deltaA, sweepAngularDeltaA));
        pointB -= linearTransformationB + (deltaB - JVector.ConjugatedTransform(deltaB, sweepAngularDeltaB));

        return true;
    }

    /// <summary>
    /// 计算两个形状在世界空间中的撞击时间和碰撞点，已知其线速度
    /// <paramref name="sweepA"/> 和 <paramref name="sweepB"/>。
    /// </summary>
    /// <param name="supportA">形状 A。</param>
    /// <param name="supportB">形状 B。</param>
    /// <param name="orientationA">形状 A 在世界空间中的朝向。</param>
    /// <param name="orientationB">形状 B 在世界空间中的朝向。</param>
    /// <param name="positionA">形状 A 在世界空间中的位置。</param>
    /// <param name="positionB">形状 B 在世界空间中的位置。</param>
    /// <param name="sweepA">形状 A 的线速度。</param>
    /// <param name="sweepB">形状 B 的线速度。</param>
    /// <param name="pointA">扫掠起点处形状 A 在世界空间中的碰撞点。</param>
    /// <param name="pointB">扫掠起点处形状 B 在世界空间中的碰撞点。</param>
    /// <param name="normal">世界空间中的归一化碰撞法线（由 A 指向 B）。如果形状已经重叠或未命中则为零。</param>
    /// <param name="lambda">撞击时间。如果未检测到命中则为 <see cref="Real.PositiveInfinity"/>，如果形状重叠则为零。</param>
    /// <returns>如果形状将会命中或已经重叠则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Sweep<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationA, in JQuaternion orientationB,
        in JVector positionA, in JVector positionB,
        in JVector sweepA, in JVector sweepB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // 旋转到 bodyA 的参考系中……
        JQuaternion.ConjugateMultiply(orientationA, orientationB, out JQuaternion orientation);
        JVector.Subtract(positionB, positionA, out JVector position);
        JVector.ConjugatedTransform(position, orientationA, out position);

        // 我们还要变换相对速度
        JVector sweep = sweepB - sweepA;
        JVector.ConjugatedTransform(sweep, orientationA, out sweep);

        // ……执行 TOI 计算
        bool res = Sweep(supportA, supportB, orientation, position, sweep,
            out pointA, out pointB, out normal, out lambda);

        if (!res) return false;

        // ……再旋转回来。当支撑函数被多次调用时，这种方法可能节省一些矩阵向量乘法。
        JVector.Transform(pointA, orientationA, out pointA);
        JVector.Add(pointA, positionA, out pointA);
        JVector.Transform(pointB, orientationA, out pointB);
        JVector.Add(pointB, positionA, out pointB);
        JVector.Transform(normal, orientationA, out normal);

        // 从相对速度变换回来

        // 碰撞将发生在世界空间中的以下位置：
        //      pointA += lambda * sweepA;
        //      pointB += lambda * sweepA; // sweepA 不是笔误

        pointB += lambda * (sweepA - sweepB);

        return true;
    }

    /// <summary>
    /// 执行扫掠检测，其中形状 A 固定在原点，朝向为单位朝向，速度为零。
    /// </summary>
    /// <param name="supportA">形状 A（固定在原点，单位朝向）。</param>
    /// <param name="supportB">形状 B。</param>
    /// <param name="orientationB">形状 B 在世界空间中的朝向。</param>
    /// <param name="positionB">形状 B 在世界空间中的位置。</param>
    /// <param name="sweepB">形状 B 的线速度。</param>
    /// <param name="pointA">扫掠起点处形状 A 在世界空间中的碰撞点。</param>
    /// <param name="pointB">扫掠起点处形状 B 在世界空间中的碰撞点。</param>
    /// <param name="normal">世界空间中的归一化碰撞法线（由 A 指向 B）。如果形状已经重叠或未命中则为零。</param>
    /// <param name="lambda">撞击时间。如果未检测到命中则为 <see cref="Real.PositiveInfinity"/>，如果形状重叠则为零。</param>
    /// <returns>如果形状命中或已经重叠则返回 true，否则返回 false。</returns>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static bool Sweep<Ta, Tb>(in Ta supportA, in Tb supportB,
        in JQuaternion orientationB, in JVector positionB, in JVector sweepB,
        out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
        where Ta : ISupportMappable where Tb : ISupportMappable
    {
        // ……执行 TOI 计算
        const Real collideEpsilon = (Real)1e-4;
        const int maxIter = 34;

        Unsafe.SkipInit(out SimplexSolverAB simplexSolver);
        simplexSolver.Reset();

        MinkowskiDifference.GetCenter(supportA, supportB, orientationB, positionB, out var center);

        JVector posB = positionB;

        lambda = (Real)0.0;

        pointA = pointB = JVector.Zero;

        JVector r = sweepB;
        JVector v = -center.V;

        normal = JVector.Zero;

        int iter = maxIter;

        Real distSq = v.LengthSquared();

        while ((distSq > collideEpsilon * collideEpsilon) && (iter-- != 0))
        {
            MinkowskiDifference.Support(supportA, supportB, orientationB, posB, v, out Vertex vertex);
            var w = vertex.V;

            Real vDotW = -JVector.Dot(v, w);

            if (vDotW > (Real)0.0)
            {
                Real vDotR = JVector.Dot(v, r);

                if (vDotR >= -(Real)1e-12)
                {
                    lambda = Real.PositiveInfinity;
                    return false;
                }

                lambda -= vDotW / vDotR;

                posB = positionB + lambda * r;
                normal = v;
            }

            if (!simplexSolver.AddVertex(vertex, out v))
            {
                goto converged;
            }

            JVector.NegateInPlace(ref v);

            distSq = v.LengthSquared();
        }

    converged:

        simplexSolver.GetClosest(out pointA, out pointB);
        normal = JVector.NormalizeSafe(in normal, NumericEpsilon);

        return true;
    }
}
