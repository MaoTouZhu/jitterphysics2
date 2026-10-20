/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System.Collections.Generic;
using System.Runtime.CompilerServices;
using Jitter2.Collision.Shapes;
using Jitter2.LinearMath;

namespace Jitter2.Collision;

public partial class DynamicTree
{
    /// <summary>
    /// 表示对动态树进行距离查询的结果。
    /// </summary>
    public struct FindNearestResult
    {
        /// <summary>找到的最近代理。</summary>
        public IDynamicTreeProxy? Entity;

        /// <summary>查询形状在世界空间中的最近点。形状重叠时无定义。</summary>
        public JVector PointA;

        /// <summary>找到的代理在世界空间中的最近点。形状重叠时无定义。</summary>
        public JVector PointB;

        /// <summary>
        /// 从查询形状指向找到的代理的单位方向，形状重叠时为 <see cref="JVector.Zero"/>。
        /// 不要用它来判断是否找到了结果。
        /// </summary>
        public JVector Normal;

        /// <summary>查询形状与找到的代理之间的分离距离。形状重叠时为零。</summary>
        public Real Distance;
    }

    /// <summary>
    /// 用于在精确形状距离测试之后过滤距离查询结果的委托。
    /// </summary>
    /// <param name="result">要评估的距离结果。</param>
    /// <returns><c>false</c> 表示过滤掉此结果；<c>true</c> 表示保留。</returns>
    public delegate bool FindNearestFilterPost(FindNearestResult result);

    /// <summary>
    /// 用于在精确形状距离测试之前过滤距离查询候选的委托。
    /// </summary>
    /// <param name="proxy">要评估的代理。</param>
    /// <returns><c>false</c> 表示跳过此代理；<c>true</c> 表示对其进行测试。</returns>
    public delegate bool FindNearestFilterPre(IDynamicTreeProxy proxy);

    /// <summary>
    /// <see cref="FindNearest{T}(in T, in JQuaternion, in JVector, FindNearestFilterPre?, FindNearestFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/>
    /// 的便捷重载，使用 <see cref="SupportPrimitives.Point"/> 进行查询。
    /// </summary>
    public bool FindNearestPoint(in JVector position,
        FindNearestFilterPre? pre, FindNearestFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real distance) =>
        FindNearest(SupportPrimitives.CreatePoint(), JQuaternion.Identity, position, pre, post,
            out proxy, out pointA, out pointB, out normal, out distance);

    /// <summary>
    /// <see cref="FindNearest{T}(in T, in JQuaternion, in JVector, Real, FindNearestFilterPre?, FindNearestFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/>
    /// 的便捷重载，使用 <see cref="SupportPrimitives.Point"/> 进行查询。
    /// </summary>
    public bool FindNearestPoint(in JVector position, Real maxDistance,
        FindNearestFilterPre? pre, FindNearestFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real distance) =>
        FindNearest(SupportPrimitives.CreatePoint(), JQuaternion.Identity, position, maxDistance, pre, post,
            out proxy, out pointA, out pointB, out normal, out distance);

    /// <summary>
    /// <see cref="FindNearest{T}(in T, in JQuaternion, in JVector, FindNearestFilterPre?, FindNearestFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/>
    /// 的便捷重载，使用 <see cref="SupportPrimitives.Sphere"/> 进行查询。
    /// </summary>
    /// <param name="radius">球体半径。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理，若未找到则为 null。</param>
    /// <param name="pointA">查询形状在世界空间中的最近点。</param>
    /// <param name="pointB">找到的代理在世界空间中的最近点。</param>
    /// <param name="normal">指向找到的代理的单位方向，形状重叠时为零。</param>
    /// <param name="distance">到找到的代理的分离距离。</param>
    public bool FindNearestSphere(Real radius, in JVector position,
        FindNearestFilterPre? pre, FindNearestFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real distance) =>
        FindNearest(SupportPrimitives.CreateSphere(radius), JQuaternion.Identity, position, pre, post,
            out proxy, out pointA, out pointB, out normal, out distance);

    /// <summary>
    /// <see cref="FindNearest{T}(in T, in JQuaternion, in JVector, Real, FindNearestFilterPre?, FindNearestFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/>
    /// 的便捷重载，使用 <see cref="SupportPrimitives.Sphere"/> 进行查询。
    /// </summary>
    /// <param name="radius">球体半径。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="maxDistance">要考虑的最大分离距离。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理，若未找到则为 null。</param>
    /// <param name="pointA">查询形状在世界空间中的最近点。</param>
    /// <param name="pointB">找到的代理在世界空间中的最近点。</param>
    /// <param name="normal">指向找到的代理的单位方向，形状重叠时为零。</param>
    /// <param name="distance">到找到的代理的分离距离。</param>
    public bool FindNearestSphere(Real radius, in JVector position, Real maxDistance,
        FindNearestFilterPre? pre, FindNearestFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real distance) =>
        FindNearest(SupportPrimitives.CreateSphere(radius), JQuaternion.Identity, position, maxDistance, pre, post,
            out proxy, out pointA, out pointB, out normal, out distance);

    private struct DistanceQuery(in JBoundingBox box, in JQuaternion orientation, in JVector position)
    {
        public readonly JVector BoxExtents = (box.Max - box.Min) * (Real)0.5;
        public readonly JVector BoxCenter = (box.Max + box.Min) * (Real)0.5;
        public readonly JQuaternion Orientation = orientation;
        public readonly JVector Position = position;

        public FindNearestFilterPost? FilterPost;
        public FindNearestFilterPre? FilterPre;

        public Real Distance;
    }

    /// <summary>
    /// 在树中查找距离查询形状最近的 <see cref="IDistanceTestable"/> 代理，
    /// 并返回精确的最近点。
    /// </summary>
    /// <param name="support">查询形状。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="pre">可选的前置过滤器，可在精确距离测试前跳过候选代理。</param>
    /// <param name="post">可选的后置过滤器，可拒绝精确结果并继续搜索。</param>
    /// <param name="proxy">最近且被接受的代理，若未找到任何代理则为 <c>null</c>。</param>
    /// <param name="pointA">查询形状在世界空间中的最近点。形状重叠时无定义。</param>
    /// <param name="pointB">找到的代理在世界空间中的最近点。形状重叠时无定义。</param>
    /// <param name="normal">从查询形状指向找到的代理的单位方向，形状重叠时为 <see cref="JVector.Zero"/>。</param>
    /// <param name="distance">查询形状与找到的代理之间的分离距离。两者重叠时为零。</param>
    /// <returns>
    /// 若找到被接受的代理则返回 <c>true</c>。这包括重叠的情况，此时 <paramref name="distance"/>
    /// 为零。若未找到被接受的代理则返回 <c>false</c>，此时 <paramref name="proxy"/> 为 <c>null</c>。
    /// </returns>
    /// <remarks>
    /// 若要跳过重叠的代理并继续搜索分离的代理，请使用对 <c>distance == 0</c>
    /// 的结果返回 <c>false</c> 的 <paramref name="post"/> 过滤器。
    /// </remarks>
    public bool FindNearest<T>(in T support, in JQuaternion orientation, in JVector position,
        FindNearestFilterPre? pre, FindNearestFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real distance)
        where T : ISupportMappable
    {
        ShapeHelper.CalculateBoundingBox(support, orientation, position, out JBoundingBox box);

        DistanceQuery query = new(box, orientation, position)
        {
            FilterPre = pre,
            FilterPost = post,
            Distance = Real.MaxValue
        };

        bool hit = QueryDistance(support, query, out var result);
        proxy = result.Entity;
        pointA = result.PointA;
        pointB = result.PointB;
        normal = result.Normal;
        distance = result.Distance;
        return hit;
    }

    /// <summary>
    /// <see cref="FindNearest{T}(in T, in JQuaternion, in JVector, FindNearestFilterPre?, FindNearestFilterPost?, out IDynamicTreeProxy?, out JVector, out JVector, out JVector, out Real)"/>
    /// 的限定范围变体，将搜索限制在 <paramref name="maxDistance"/> 以内。
    /// </summary>
    /// <param name="support">查询形状。</param>
    /// <param name="orientation">查询形状在世界空间中的朝向。</param>
    /// <param name="position">查询形状在世界空间中的位置。</param>
    /// <param name="maxDistance">要考虑的最大分离距离。距离超过此值的代理将被忽略。</param>
    /// <param name="pre">在精确测试之前应用的可选过滤器。</param>
    /// <param name="post">在精确测试之后应用的可选过滤器。</param>
    /// <param name="proxy">被接受的代理，若未找到则为 null。</param>
    /// <param name="pointA">查询形状在世界空间中的最近点。</param>
    /// <param name="pointB">找到的代理在世界空间中的最近点。</param>
    /// <param name="normal">指向找到的代理的单位方向，形状重叠时为零。</param>
    /// <param name="distance">到找到的代理的分离距离。</param>
    public bool FindNearest<T>(in T support, in JQuaternion orientation, in JVector position, Real maxDistance,
        FindNearestFilterPre? pre, FindNearestFilterPost? post,
        out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out Real distance)
        where T : ISupportMappable
    {
        ShapeHelper.CalculateBoundingBox(support, orientation, position, out JBoundingBox box);

        DistanceQuery query = new(box, orientation, position)
        {
            FilterPre = pre,
            FilterPost = post,
            Distance = maxDistance
        };

        bool hit = QueryDistance(support, query, out var result);
        proxy = result.Entity;
        pointA = result.PointA;
        pointB = result.PointB;
        normal = result.Normal;
        distance = result.Distance;
        return hit;
    }

    // 返回查询 AABB 与树节点扩展 AABB 之间的最小距离。
    // 使用闵可夫斯基和：先将目标按查询的半尺寸扩展，再从查询中心测量
    // 点到 AABB 的距离。若 AABB 重叠则返回 0。
    private static Real MinDistBox(in JVector queryExtents, in JVector queryCenter, in TreeBox targetBox)
    {
        Real dx = MathR.Max(MathR.Max(targetBox.Min.X - queryExtents.X - queryCenter.X, queryCenter.X - targetBox.Max.X - queryExtents.X), (Real)0.0);
        Real dy = MathR.Max(MathR.Max(targetBox.Min.Y - queryExtents.Y - queryCenter.Y, queryCenter.Y - targetBox.Max.Y - queryExtents.Y), (Real)0.0);
        Real dz = MathR.Max(MathR.Max(targetBox.Min.Z - queryExtents.Z - queryCenter.Z, queryCenter.Z - targetBox.Max.Z - queryExtents.Z), (Real)0.0);

        return MathR.Sqrt(dx * dx + dy * dy + dz * dz);
    }

    private bool QueryDistance<T>(in T support, in DistanceQuery query, out FindNearestResult result)
        where T : ISupportMappable
    {
        result = new FindNearestResult
        {
            Distance = query.Distance
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
                    IDynamicTreeProxy proxy = node.Proxy!;
                    if (proxy is not IDistanceTestable distanceTestable) continue;
                    if (query.FilterPre != null && !query.FilterPre(proxy)) continue;

                    Unsafe.SkipInit(out FindNearestResult res);
                    bool separated = distanceTestable.Distance(support,
                        query.Orientation, query.Position,
                        out res.PointA, out res.PointB, out res.Normal, out res.Distance);
                    res.Entity = proxy;

                    if (!separated)
                    {
                        res.Distance = (Real)0.0;
                        res.Normal = JVector.Zero;
                        if (query.FilterPost != null && !query.FilterPost(res)) continue;
                        result = res;
                        return true;
                    }

                    if (res.Distance > result.Distance) continue;
                    if (query.FilterPost != null && !query.FilterPost(res)) continue;

                    result = res;
                    continue;
                }

                ref Node leftNode = ref nodes[node.Left];
                ref Node rightNode = ref nodes[node.Right];

                Real leftDist = MinDistBox(query.BoxExtents, query.BoxCenter, leftNode.ExpandedBox);
                Real rightDist = MinDistBox(query.BoxExtents, query.BoxCenter, rightNode.ExpandedBox);

                bool leftHit = leftDist <= result.Distance;
                bool rightHit = rightDist <= result.Distance;

                if (leftHit && rightHit)
                {
                    if (leftDist < rightDist)
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
