/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Jitter2.LinearMath;
using Jitter2.Collision.Shapes;

namespace Jitter2.Collision;

public partial class DynamicTree
{
    /// <summary>
    /// 表示对动态树进行扫掠的结果。
    /// </summary>
    public struct SweepCastResult
    {
        /// <summary>被击中的代理。</summary>
        public IDynamicTreeProxy? Entity;

        /// <summary>扫掠起点处，移动查询形状在世界空间中的碰撞点。当形状已经重叠时未定义。</summary>
        public JVector PointA;

        /// <summary>扫掠起点处，被击中物体在世界空间中的碰撞点。当形状已经重叠时未定义。</summary>
        public JVector PointB;

        /// <summary>
        /// 世界空间中的碰撞法线；若形状已经重叠则为 <see cref="JVector.Zero"/>。
        /// 不要用它来判断是否发生命中。
        /// </summary>
        public JVector Normal;

        /// <summary>撞击时间，以扫掠方向向量为单位。若形状已经重叠则为零。</summary>
        public Real Lambda;
    }

    /// <summary>
    /// 在精确形状扫掠测试之后对扫掠结果进行过滤的委托。
    /// </summary>
    /// <param name="result">要评估的扫掠结果。</param>
    /// <returns><c>false</c> 表示过滤掉该命中；<c>true</c> 表示保留。</returns>
    public delegate bool SweepCastFilterPost(SweepCastResult result);

    /// <summary>
    /// 在精确形状扫掠测试之前对扫掠候选进行过滤的委托。
    /// </summary>
    /// <param name="proxy">要评估的代理。</param>
    /// <returns><c>false</c> 表示跳过该代理；<c>true</c> 表示对其进行测试。</returns>
    public delegate bool SweepCastFilterPre(IDynamicTreeProxy proxy);

    /// <summary>
    /// <see cref="SweepCast{T}(in T, in JQuaternion, in JVector, in JVector, SweepCastFilterPre?, SweepCastFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/> 的便捷重载，
    /// 对 <see cref="SupportPrimitives.Sphere"/> 进行扫掠。
    /// </summary>
    /// <param name="radius">球体半径。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="pointA">世界空间中查询形状上的接触点。</param>
    /// <param name="pointB">世界空间中所找到代理上的接触点。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">第一个被接受的命中处的扫掠参数；初始重叠时为零。</param>
    public bool SweepCastSphere(Real radius, in JVector position, in JVector direction,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda) =>
        SweepCast(SupportPrimitives.CreateSphere(radius), JQuaternion.Identity, position, direction, pre,
            post, out proxy, out pointA, out pointB, out normal, out lambda);

    /// <summary>
    /// <see cref="SweepCast{T}(in T, in JQuaternion, in JVector, in JVector, Real, SweepCastFilterPre?, SweepCastFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/> 的便捷重载，
    /// 对 <see cref="SupportPrimitives.Sphere"/> 进行扫掠。
    /// </summary>
    /// <param name="radius">球体半径。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向。</param>
    /// <param name="maxLambda">要考虑的最大扫掠参数。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="pointA">世界空间中查询形状上的接触点。</param>
    /// <param name="pointB">世界空间中所找到代理上的接触点。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">第一个被接受的命中处的扫掠参数；初始重叠时为零。</param>
    public bool SweepCastSphere(Real radius, in JVector position, in JVector direction, Real maxLambda,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda) =>
        SweepCast(SupportPrimitives.CreateSphere(radius), JQuaternion.Identity, position, direction, maxLambda, pre,
            post, out proxy, out pointA, out pointB, out normal, out lambda);

    /// <summary>
    /// <see cref="SweepCast{T}(in T, in JQuaternion, in JVector, in JVector, SweepCastFilterPre?, SweepCastFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/> 的便捷重载，
    /// 对 <see cref="SupportPrimitives.Box"/> 进行扫掠。
    /// </summary>
    /// <param name="halfExtents">盒子的半尺寸。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="pointA">世界空间中查询形状上的接触点。</param>
    /// <param name="pointB">世界空间中所找到代理上的接触点。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">第一个被接受的命中处的扫掠参数；初始重叠时为零。</param>
    public bool SweepCastBox(in JVector halfExtents, in JQuaternion orientation, in JVector position, in JVector direction,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda) =>
        SweepCast(SupportPrimitives.CreateBox(halfExtents), orientation, position, direction, pre,
            post, out proxy, out pointA, out pointB, out normal, out lambda);

    /// <summary>
    /// <see cref="SweepCast{T}(in T, in JQuaternion, in JVector, in JVector, Real, SweepCastFilterPre?, SweepCastFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/> 的便捷重载，
    /// 对 <see cref="SupportPrimitives.Box"/> 进行扫掠。
    /// </summary>
    /// <param name="halfExtents">盒子的半尺寸。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向。</param>
    /// <param name="maxLambda">要考虑的最大扫掠参数。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="pointA">世界空间中查询形状上的接触点。</param>
    /// <param name="pointB">世界空间中所找到代理上的接触点。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">第一个被接受的命中处的扫掠参数；初始重叠时为零。</param>
    public bool SweepCastBox(in JVector halfExtents, in JQuaternion orientation, in JVector position, in JVector direction, Real maxLambda,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda) =>
        SweepCast(SupportPrimitives.CreateBox(halfExtents), orientation, position, direction, maxLambda, pre,
            post, out proxy, out pointA, out pointB, out normal, out lambda);

    /// <summary>
    /// <see cref="SweepCast{T}(in T, in JQuaternion, in JVector, in JVector, SweepCastFilterPre?, SweepCastFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/> 的便捷重载，
    /// 对 <see cref="SupportPrimitives.Capsule"/> 进行扫掠。
    /// </summary>
    /// <param name="radius">胶囊半径。</param>
    /// <param name="halfLength">胶囊圆柱段长度的一半。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="pointA">世界空间中查询形状上的接触点。</param>
    /// <param name="pointB">世界空间中所找到代理上的接触点。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">第一个被接受的命中处的扫掠参数；初始重叠时为零。</param>
    public bool SweepCastCapsule(Real radius, Real halfLength, in JQuaternion orientation, in JVector position, in JVector direction,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda) =>
        SweepCast(SupportPrimitives.CreateCapsule(radius, halfLength), orientation, position, direction, pre,
            post, out proxy, out pointA, out pointB, out normal, out lambda);

    /// <summary>
    /// <see cref="SweepCast{T}(in T, in JQuaternion, in JVector, in JVector, Real, SweepCastFilterPre?, SweepCastFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/> 的便捷重载，
    /// 对 <see cref="SupportPrimitives.Capsule"/> 进行扫掠。
    /// </summary>
    /// <param name="radius">胶囊半径。</param>
    /// <param name="halfLength">胶囊圆柱段长度的一半。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向。</param>
    /// <param name="maxLambda">要考虑的最大扫掠参数。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="pointA">世界空间中查询形状上的接触点。</param>
    /// <param name="pointB">世界空间中所找到代理上的接触点。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">第一个被接受的命中处的扫掠参数；初始重叠时为零。</param>
    public bool SweepCastCapsule(Real radius, Real halfLength, in JQuaternion orientation, in JVector position, in JVector direction, Real maxLambda,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda) =>
        SweepCast(SupportPrimitives.CreateCapsule(radius, halfLength), orientation, position, direction, maxLambda, pre,
            post, out proxy, out pointA, out pointB, out normal, out lambda);

    /// <summary>
    /// <see cref="SweepCast{T}(in T, in JQuaternion, in JVector, in JVector, SweepCastFilterPre?, SweepCastFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/> 的便捷重载，
    /// 对 <see cref="SupportPrimitives.Cylinder"/> 进行扫掠。
    /// </summary>
    /// <param name="radius">圆柱半径。</param>
    /// <param name="halfHeight">圆柱高度的一半。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="pointA">世界空间中查询形状上的接触点。</param>
    /// <param name="pointB">世界空间中所找到代理上的接触点。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">第一个被接受的命中处的扫掠参数；初始重叠时为零。</param>
    public bool SweepCastCylinder(Real radius, Real halfHeight, in JQuaternion orientation, in JVector position, in JVector direction,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda) =>
        SweepCast(SupportPrimitives.CreateCylinder(radius, halfHeight), orientation, position, direction, pre,
            post, out proxy, out pointA, out pointB, out normal, out lambda);

    /// <summary>
    /// <see cref="SweepCast{T}(in T, in JQuaternion, in JVector, in JVector, Real, SweepCastFilterPre?, SweepCastFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/> 的便捷重载，
    /// 对 <see cref="SupportPrimitives.Cylinder"/> 进行扫掠。
    /// </summary>
    /// <param name="radius">圆柱半径。</param>
    /// <param name="halfHeight">圆柱高度的一半。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向。</param>
    /// <param name="maxLambda">要考虑的最大扫掠参数。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="pointA">世界空间中查询形状上的接触点。</param>
    /// <param name="pointB">世界空间中所找到代理上的接触点。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">第一个被接受的命中处的扫掠参数；初始重叠时为零。</param>
    public bool SweepCastCylinder(Real radius, Real halfHeight, in JQuaternion orientation, in JVector position, in JVector direction, Real maxLambda,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda) =>
        SweepCast(SupportPrimitives.CreateCylinder(radius, halfHeight), orientation, position, direction, maxLambda, pre,
            post, out proxy, out pointA, out pointB, out normal, out lambda);

    private struct SweepQuery(in JBoundingBox box, in JQuaternion orientation, in JVector position, in JVector direction)
    {
        public readonly JBoundingBox Box = box;
        public readonly JQuaternion Orientation = orientation;
        public readonly JVector Position = position;
        public readonly JVector Direction = direction;

        public SweepCastFilterPost? FilterPost;
        public SweepCastFilterPre? FilterPre;

        public Real Lambda;
    }

    /// <summary>
    /// 用支撑映射的查询形状对树中所有 <see cref="ISweepTestable"/> 代理进行扫掠，
    /// 并返回最近的精确命中。
    /// </summary>
    /// <param name="support">查询形状。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">扫掠开始时查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向向量。</param>
    /// <param name="pre">可选的前置过滤器，可在精确扫掠测试之前跳过候选代理。</param>
    /// <param name="post">可选的后置过滤器，可拒绝精确结果并继续搜索。</param>
    /// <param name="proxy">最近的被接受命中代理；若未找到命中则为 <c>null</c>。</param>
    /// <param name="pointA">扫掠起点处查询形状在世界空间中的碰撞点。当形状已经重叠时未定义。</param>
    /// <param name="pointB">扫掠起点处被击中代理在世界空间中的碰撞点。当形状已经重叠时未定义。</param>
    /// <param name="normal">世界空间中的碰撞法线；当形状已经重叠时为 <see cref="JVector.Zero"/>。</param>
    /// <param name="lambda">撞击时间，以 <paramref name="direction"/> 为单位。当形状已经重叠时为零。</param>
    /// <returns>
    /// <c>true</c> 表示找到被接受的命中。这也包括重叠的情况，此时 <paramref name="lambda"/>
    /// 为零。<c>false</c> 表示未找到被接受的命中，此时 <paramref name="proxy"/> 为 <c>null</c>。
    /// </returns>
    public bool SweepCast<T>(in T support, in JQuaternion orientation, in JVector position, in JVector direction,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
        where T : ISupportMappable
    {
        ShapeHelper.CalculateBoundingBox(support, orientation, position, out JBoundingBox box);

        SweepQuery sweep = new(box, orientation, position, direction)
        {
            FilterPre = pre,
            FilterPost = post,
            Lambda = Real.MaxValue
        };

        bool hit = QuerySweep(support, sweep, out var result);
        proxy = result.Entity;
        pointA = result.PointA;
        pointB = result.PointB;
        normal = result.Normal;
        lambda = result.Lambda;
        return hit;
    }

    /// <summary>
    /// <see cref="SweepCast{T}(in T, in JQuaternion, in JVector, in JVector, SweepCastFilterPre?, SweepCastFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/> 的有界变体，
    /// 将扫掠限制在 <paramref name="maxLambda"/> 内。
    /// </summary>
    /// <param name="support">查询形状。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="direction">世界空间中的扫掠方向。</param>
    /// <param name="maxLambda">沿 <paramref name="direction"/> 要考虑的最大扫掠参数。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="pointA">世界空间中查询形状上的接触点。</param>
    /// <param name="pointB">世界空间中所找到代理上的接触点。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">第一个被接受的命中处的扫掠参数；初始重叠时为零。</param>
    public bool SweepCast<T>(in T support, in JQuaternion orientation, in JVector position, in JVector direction, Real maxLambda,
        SweepCastFilterPre? pre, SweepCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real lambda)
        where T : ISupportMappable
    {
        ShapeHelper.CalculateBoundingBox(support, orientation, position, out JBoundingBox box);

        SweepQuery sweep = new(box, orientation, position, direction)
        {
            FilterPre = pre,
            FilterPost = post,
            Lambda = maxLambda
        };

        bool hit = QuerySweep(support, sweep, out var result);
        proxy = result.Entity;
        pointA = result.PointA;
        pointB = result.PointB;
        normal = result.Normal;
        lambda = result.Lambda;
        return hit;
    }

    private static bool SweepBox(in JBoundingBox movingBox, in JVector translation, in TreeBox targetBox, out Real enter)
    {
        JVector extents = (movingBox.Max - movingBox.Min) * (Real)0.5;
        JVector center = (movingBox.Max + movingBox.Min) * (Real)0.5;

        JBoundingBox expanded = new(targetBox.Min - extents, targetBox.Max + extents);
        return expanded.RayIntersect(center, translation, out enter);
    }

    private bool QuerySweep<T>(in T support, in SweepQuery sweep, out SweepCastResult result)
        where T : ISupportMappable
    {
        result = new SweepCastResult
        {
            Lambda = sweep.Lambda
        };

        if (root == NullNode)
        {
            return false;
        }

        Stack<int> stack = QueryStack;
        int baseCount = stack.Count;
        try
        {
            stack.Push(root);

            while (stack.Count > baseCount)
            {
                int index = stack.Pop();
                ref Node node = ref nodes[index];

                if (node.IsLeaf)
                {
                    if (node.Proxy is not ISweepTestable sweepCastable) continue;
                    if (sweep.FilterPre != null && !sweep.FilterPre(node.Proxy!)) continue;

                    Unsafe.SkipInit(out SweepCastResult res);
                    bool hit = sweepCastable.Sweep(support,
                        sweep.Orientation, sweep.Position, sweep.Direction,
                        out res.PointA, out res.PointB, out res.Normal, out res.Lambda);
                    res.Entity = node.Proxy;

                    if (!hit || res.Lambda > result.Lambda) continue;
                    if (sweep.FilterPost != null && !sweep.FilterPost(res)) continue;

                    result = res;
                    continue;
                }

                ref Node leftNode = ref nodes[node.Left];
                ref Node rightNode = ref nodes[node.Right];

                bool leftHit = SweepBox(sweep.Box, sweep.Direction, leftNode.ExpandedBox, out Real leftEnter);
                bool rightHit = SweepBox(sweep.Box, sweep.Direction, rightNode.ExpandedBox, out Real rightEnter);

                if (leftEnter > result.Lambda) leftHit = false;
                if (rightEnter > result.Lambda) rightHit = false;

                if (leftHit && rightHit)
                {
                    if (leftEnter < rightEnter)
                    {
                        stack.Push(node.Right);
                        stack.Push(node.Left);
                    }
                    else
                    {
                        stack.Push(node.Left);
                        stack.Push(node.Right);
                    }
                }
                else
                {
                    if (leftHit) stack.Push(node.Left);
                    if (rightHit) stack.Push(node.Right);
                }
            }

            return result.Entity != null;
        }
        finally
        {
            PopTo(stack, baseCount);
        }
    }

}
