/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

public partial class DynamicTree
{
    /// <summary>
    /// 表示对动态树进行射线投射的结果。
    /// </summary>
    public struct RayCastResult
    {
        /// <summary>被击中的代理。</summary>
        public IDynamicTreeProxy? Entity;

        /// <summary>命中处的射线参数：<c>hitPoint = origin + Lambda * direction</c>。</summary>
        public Real Lambda;

        /// <summary>
        /// 命中点处的表面法线；当射线原点位于被击中的形状内部时为 <see cref="JVector.Zero"/>。
        /// 不要用它来判断是否发生命中。
        /// </summary>
        public JVector Normal;
    }

    /// <summary>
    /// 在形状相交测试之后对射线投射结果进行过滤的委托。
    /// </summary>
    /// <param name="result">要评估的射线投射结果。</param>
    /// <returns><c>false</c> 表示过滤掉该命中；<c>true</c> 表示保留。</returns>
    public delegate bool RayCastFilterPost(RayCastResult result);

    /// <summary>
    /// 在形状相交测试之前对射线投射候选进行过滤的委托。
    /// </summary>
    /// <param name="proxy">要评估的代理。</param>
    /// <returns><c>false</c> 表示跳过该代理；<c>true</c> 表示对其进行测试。</returns>
    public delegate bool RayCastFilterPre(IDynamicTreeProxy proxy);

    private struct Ray(in JVector origin, in JVector direction)
    {
        public readonly JVector Origin = origin;
        public readonly JVector Direction = direction;

        public RayCastFilterPost? FilterPost = null;
        public RayCastFilterPre? FilterPre = null;

        public Real Lambda = Real.MaxValue;
    }

    /// <summary>
    /// 对世界进行射线投射。
    /// </summary>
    /// <param name="origin">射线的原点。</param>
    /// <param name="direction">射线的方向。不必归一化。</param>
    /// <param name="pre">可选的前置过滤器，可跳过候选形状。</param>
    /// <param name="post">可选的后置过滤器，可跳过候选结果。</param>
    /// <param name="proxy">被击中的形状。</param>
    /// <param name="normal">
    /// <param name="lambda">从原点到命中点的距离，以射线方向为单位。若原点位于被击中的形状内部则为零。</param>
    /// 命中点处的表面法线。若射线未命中，或射线原点位于被击中的形状内部，则为 <see cref="JVector.Zero"/>。
    /// 请使用返回值判断是否发生命中；不要依赖该值非零来判断是否命中。
    /// </param>
    /// <returns>若射线命中则返回 true，否则返回 false。</returns>
    public bool RayCast(JVector origin, JVector direction, RayCastFilterPre? pre, RayCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector normal, out Real lambda)
    {
        Ray ray = new(origin, direction)
        {
            FilterPre = pre,
            FilterPost = post
        };
        bool hit = QueryRay(ray, out var result);
        proxy = result.Entity;
        normal = result.Normal;
        lambda = result.Lambda;
        return hit;
    }

    /// <inheritdoc cref="RayCast(JVector, JVector, RayCastFilterPre?, RayCastFilterPost?, out IDynamicTreeProxy?, out JVector, out Real)"/>
    /// <param name="origin">世界空间中的射线原点。</param>
    /// <param name="direction">射线方向；无需归一化。</param>
    /// <param name="maxLambda">命中 lambda 的不含该值的上界；等于该值的命中会被排除。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理；若未找到则为 null。</param>
    /// <param name="normal">世界空间中的结果法线。</param>
    /// <param name="lambda">结果的射线或扫掠参数。</param>
    public bool RayCast(JVector origin, JVector direction, Real maxLambda, RayCastFilterPre? pre, RayCastFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector normal, out Real lambda)
    {
        Ray ray = new(origin, direction)
        {
            FilterPre = pre,
            FilterPost = post,
            Lambda = maxLambda
        };
        bool hit = QueryRay(ray, out var result);
        proxy = result.Entity;
        normal = result.Normal;
        lambda = result.Lambda;
        return hit;
    }

    private bool QueryRay(in Ray ray, out RayCastResult result)
    {
        result = new RayCastResult();

        if (root == NullNode)
        {
            return false;
        }

        Stack<int> stack = QueryStack;
        int baseCount = stack.Count;
        try
        {
            stack.Push(root);

            bool globalHit = false;

            result.Lambda = ray.Lambda;

            while (stack.Count > baseCount)
            {
                int index = stack.Pop();

                ref Node node = ref nodes[index];

                if (node.IsLeaf)
                {
                    IDynamicTreeProxy proxy = node.Proxy!;
                    if (proxy is not IRayCastable rayCastable) continue;

                    if (ray.FilterPre != null && !ray.FilterPre(proxy)) continue;

                    Unsafe.SkipInit(out RayCastResult res);
                    bool hit = rayCastable.RayCast(ray.Origin, ray.Direction, out res.Normal, out res.Lambda);
                    res.Entity = proxy;

                    // 并列时保留已接受的最近命中；maxLambda 是不含该值的上界。
                    if (hit && res.Lambda < result.Lambda)
                    {
                        if (ray.FilterPost != null && !ray.FilterPost(res)) continue;
                        result = res;
                        globalHit = true;
                    }

                    continue;
                }

                ref Node lNode = ref nodes[node.Left];
                ref Node rNode = ref nodes[node.Right];

                bool lRes = lNode.ExpandedBox.RayIntersect(ray.Origin, ray.Direction, out Real lEnter);
                bool rRes = rNode.ExpandedBox.RayIntersect(ray.Origin, ray.Direction, out Real rEnter);

                if (lEnter > result.Lambda) lRes = false;
                if (rEnter > result.Lambda) rRes = false;

                if (lRes && rRes)
                {
                    if (lEnter < rEnter)
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
                    if (lRes) stack.Push(node.Left);
                    if (rRes) stack.Push(node.Right);
                }
            }

            return globalHit;
        }
        finally
        {
            PopTo(stack, baseCount);
        }
    }
}
