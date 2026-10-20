/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Runtime.CompilerServices;
using Jitter2.DataStructures;
using Jitter2.LinearMath;
using Jitter2.Parallelization;

namespace Jitter2.Collision;

/// <summary>
/// 表示用于宽相碰撞检测的动态 AABB 树。
/// </summary>
/// <remarks>
/// 使用带表面积启发式（SAH）的包围体层次结构进行插入。
/// 支持对移动对象进行增量更新以及有界后台优化。
/// </remarks>
public partial class DynamicTree
{
    private struct OverlapEnumerationParam
    {
        public Action<IDynamicTreeProxy, IDynamicTreeProxy> Action;
        public Parallel.Batch Batch;
    }

    private readonly PartitionedSet<IDynamicTreeProxy> proxies = [];

    private readonly SlimBag<IDynamicTreeProxy> movedProxies = [];

    /// <summary>
    /// 获取注册到此树的所有代理的只读视图。
    /// </summary>
    public ReadOnlyPartitionedSet<IDynamicTreeProxy> Proxies => new(proxies);

    internal void Trim()
    {
        proxies.Trim();
        movedProxies.Trim();
        tempList.TrimExcess();
        freeNodes.TrimExcess();
    }

    private PairHashSet potentialPairs = [];

    /// <summary>
    /// 表示空/无效节点索引的哨兵值。
    /// </summary>
    public const int NullNode = -1;

    /// <summary>
    /// 内部节点数组的初始容量。
    /// </summary>
    public const int InitialSize = 1024;

    /// <summary>
    /// 每次更新时扫描的潜在配对哈希集合比例，用于剪除无效条目。
    /// 值为 128 表示每次更新扫描哈希集合的 1/128。
    /// </summary>
    public const int PruningFraction = 128;

    /// <summary>
    /// 指定动态树结构中包围盒的扩展系数。扩展量计算为
    /// <see cref="IDynamicTreeProxy.Velocity"/> * ExpandFactor * (1 + alpha)，其中 alpha 是范围为 [0, 1) 的伪随机数。
    /// </summary>
    public const Real ExpandFactor = (Real)0.1;

    /// <summary>
    /// 指定包围盒一个恒定的少量额外扩展。
    /// </summary>
    public const Real ExpandEps = (Real)0.1;

    /// <summary>
    /// 表示 AABB 树中的节点。
    /// </summary>
    public struct Node
    {
        /// <summary>左子节点的索引；如果是叶节点，则为 <see cref="NullNode"/>。</summary>
        public int Left;

        /// <summary>右子节点的索引；如果是叶节点，则为 <see cref="NullNode"/>。</summary>
        public int Right;

        /// <summary>父节点的索引；如果是根节点，则为 <see cref="NullNode"/>。</summary>
        public int Parent;

        /// <summary>
        /// 该节点的扩展包围盒，用于宽相剔除。
        /// 对于叶节点，它是代理的包围盒按速度与边距扩展后的结果。
        /// 对于内部节点，它是其子节点盒的并集。
        /// </summary>
        public TreeBox ExpandedBox;

        /// <summary>
        /// 该子树包含的叶节点数量。叶节点的计数为一。
        /// </summary>
        public int LeafCount;

        /// <summary>
        /// 该子树的高度。叶节点的高度为零。
        /// </summary>
        public int Height;

        /// <summary>
        /// 与该节点关联的代理；对于内部节点为 <c>null</c>。
        /// </summary>
        public IDynamicTreeProxy? Proxy;

        /// <summary>
        /// 设置后，强制在下次 <see cref="DynamicTree.Update"/> 调用中更新该节点，
        /// 即使其包围盒未发生变化。
        /// </summary>
        public bool ForceUpdate;

        /// <summary>
        /// 如果这是叶节点（有关联的代理），则返回 <c>true</c>。
        /// </summary>
        public readonly bool IsLeaf => Proxy != null;
    }

    private Node[] nodes = new Node[InitialSize];

    /// <summary>
    /// 内部树节点的只读视图，用于调试或可视化。
    /// 不要在树操作之间缓存返回的 span。
    /// </summary>
    public ReadOnlySpan<Node> Nodes => nodes.AsSpan(0, nodePointer + 1);

    private readonly Stack<int> freeNodes = [];
    private int nodePointer = -1;
    private int root = NullNode;

    /// <summary>
    /// 获取根节点的索引；如果树为空，则为 <see cref="NullNode"/>。
    /// </summary>
    public int Root => root;

    private Func<IDynamicTreeProxy, IDynamicTreeProxy, bool> filter = null!;

    /// <summary>
    /// 获取或设置用于将代理对排除在碰撞检测之外的过滤器函数。
    /// </summary>
    /// <remarks>
    /// 该过滤器在重叠枚举期间调用。返回 <c>false</c> 即可排除某一配对。
    /// 在 Jitter 中，通常用于排除属于同一刚体的形状。
    /// 更改过滤器会为已注册的代理重建潜在配对。
    /// </remarks>
    /// <exception cref="ArgumentNullException">当值为 null 时抛出。</exception>
    public Func<IDynamicTreeProxy, IDynamicTreeProxy, bool> Filter
    {
        get => filter;
        set
        {
            ArgumentNullException.ThrowIfNull(value);

            if (proxies.Count == 0)
            {
                filter = value;
                return;
            }

            PairHashSet previousPairs = potentialPairs;
            var previousFilter = filter;
            var rebuiltPairs = new PairHashSet();

            filter = value;
            potentialPairs = rebuiltPairs;

            try
            {
                foreach (var proxy in proxies)
                {
                    OverlapCheckAdd(root, proxy.NodePtr);
                }
            }
            catch
            {
                filter = previousFilter;
                potentialPairs = previousPairs;
                throw;
            }
        }
    }

    private readonly Action<OverlapEnumerationParam> enumerateOverlaps;
    private readonly Action<Parallel.Batch> updateBoundingBoxes;
    private readonly Action<Parallel.Batch> scanForMovedProxies;
    private readonly Action<Parallel.Batch> scanForOverlaps;

    private readonly Random random = new(1234);
    private readonly Func<double> rndFunc;

    /// <summary>
    /// 初始化 <see cref="DynamicTree"/> 类的新实例。
    /// </summary>
    /// <param name="filter">
    /// 返回 <c>false</c> 以将代理对排除在碰撞检测之外的函数。
    /// 参见 <see cref="Filter"/>。
    /// </param>
    /// <exception cref="ArgumentNullException">
    /// 当 <paramref name="filter"/> 为 <see langword="null"/> 时抛出。
    /// </exception>
    public DynamicTree(Func<IDynamicTreeProxy, IDynamicTreeProxy, bool> filter)
    {
        enumerateOverlaps = EnumerateOverlapsCallback;
        updateBoundingBoxes = UpdateBoundingBoxesCallback;
        scanForMovedProxies = ScanForMovedProxies;
        scanForOverlaps = ScanForOverlapsCallback;

        rndFunc = () => random.NextDouble();

        Filter = filter;
    }

    /// <summary>
    /// 用于 <see cref="DebugTimings"/> 的性能分析分段，表示 <see cref="Update"/> 的各个阶段。
    /// </summary>
    public enum Timings
    {
        /// <summary>从潜在配对集合中移除失效配对所花费的时间。</summary>
        PruneInvalidPairs,

        /// <summary>更新代理包围盒所花费的时间。</summary>
        UpdateBoundingBoxes,

        /// <summary>扫描移出其扩展盒的代理所花费的时间。</summary>
        ScanMoved,

        /// <summary>将移动过的代理重新插入树中所花费的时间。</summary>
        UpdateProxies,

        /// <summary>扫描新的重叠配对所花费的时间。</summary>
        ScanOverlaps,

        /// <summary>增量树优化所花费的时间。</summary>
        Optimize,

        /// <summary>用于数组大小的哨兵值。不是真正的计时分段。</summary>
        Last
    }

    private readonly double[] debugTimings = new double[(int)Timings.Last];

    /// <summary>
    /// 包含上次调用 <see cref="Update"/> 各阶段的计时。
    /// 单位为毫秒。使用 <c>(int)Timings.XYZ</c> 索引。
    /// </summary>
    public ReadOnlySpan<double> DebugTimings => debugTimings;

    /// <summary>
    /// 获取上次调用 <see cref="Update"/> 期间更新的代理数量。
    /// </summary>
    public int UpdatedProxyCount => movedProxies.Count;

    /// <summary>
    /// 获取内部哈希集合中分配的总槽数，以及用于存储潜在重叠的条目数。
    /// </summary>
    public (int TotalSize, int Count) HashSetInfo => (potentialPairs.Slots.Length, potentialPairs.Count);

    private void EnumerateOverlapsCallback(OverlapEnumerationParam parameter)
    {
        var batch = parameter.Batch;

        for (int e = batch.Start; e < batch.End; e++)
        {
            var node = potentialPairs.Slots[e];
            if (node.ID == 0) continue;

            var proxyA = nodes[node.ID1].Proxy;
            var proxyB = nodes[node.ID2].Proxy;

            if (proxyA == null || proxyB == null) continue;
            if (!Filter(proxyA, proxyB)) continue;

            if (!JBoundingBox.Disjoint(proxyA.WorldBoundingBox, proxyB.WorldBoundingBox))
            {
                parameter.Action(proxyA, proxyB);
            }
        }
    }

    /// <summary>
    /// 枚举所有潜在碰撞配对，并对每个配对调用指定操作。
    /// </summary>
    /// <param name="action">对每个重叠配对调用的操作。</param>
    /// <param name="multiThread">如果为 <c>true</c>，则使用多线程进行枚举。</param>
    public void EnumerateOverlaps(Action<IDynamicTreeProxy, IDynamicTreeProxy> action, bool multiThread = false)
    {
        OverlapEnumerationParam overlapEnumerationParam;
        overlapEnumerationParam.Action = action;

        int slotsLength = potentialPairs.Slots.Length;

        if (multiThread)
        {
            var tpi = ThreadPool.Instance;

            // 通常，这是模拟步中第一个多线程阶段。
            // 线程可能仍在休眠，因此我们为每个线程分配多个任务，
            // 以在线程陆续唤醒时改善负载分布。
            const int taskMultiplier = 6;
            int taskCount = tpi.ThreadCount * taskMultiplier;

            for (int i = 0; i < taskCount; i++)
            {
                Parallel.GetBounds(slotsLength, taskCount, i, out int start, out int end);
                overlapEnumerationParam.Batch = new Parallel.Batch(start, end);
                tpi.AddTask(enumerateOverlaps, overlapEnumerationParam);
            }

            tpi.Execute();
        }
        else
        {
            overlapEnumerationParam.Batch = new Parallel.Batch(0, slotsLength);
            EnumerateOverlapsCallback(overlapEnumerationParam);
        }
    }

    /// <summary>
    /// 更新树中所有活动代理。
    /// </summary>
    /// <param name="multiThread">如果为 <c>true</c>，则使用多线程进行更新。</param>
    /// <param name="dt">时间步（秒），用于基于速度的包围盒扩展。</param>
    public void Update(bool multiThread, Real dt)
    {
        long time = Stopwatch.GetTimestamp();
        double invFrequency = 1.0d / Stopwatch.Frequency;

        void SetTime(Timings type)
        {
            long ctime = Stopwatch.GetTimestamp();
            double delta = (ctime - time) * 1000.0d;
            debugTimings[(int)type] = delta * invFrequency;
            time = ctime;
        }

        this.stepDt = dt;

        Tracer.ProfileBegin(TraceName.PruneInvalidPairs);
        PruneInvalidPairs();
        Tracer.ProfileEnd(TraceName.PruneInvalidPairs);
        SetTime(Timings.PruneInvalidPairs);

        Tracer.ProfileBegin(TraceName.UpdateBoundingBoxes);
        var batch = new Parallel.Batch(0, proxies.ActiveCount);
        if (multiThread) proxies.ParallelForBatch(256, updateBoundingBoxes);
        else UpdateBoundingBoxesCallback(batch);
        Tracer.ProfileEnd(TraceName.UpdateBoundingBoxes);
        SetTime(Timings.UpdateBoundingBoxes);

        Tracer.ProfileBegin(TraceName.ScanMoved);
        movedProxies.Clear();
        if (multiThread) proxies.ParallelForBatch(24, scanForMovedProxies);
        else ScanForMovedProxies(batch);
        Tracer.ProfileEnd(TraceName.ScanMoved);
        SetTime(Timings.ScanMoved);

        Tracer.ProfileBegin(TraceName.UpdateProxies);
        for (int i = 0; i < movedProxies.Count; i++)
        {
            InternalAddRemoveProxy(movedProxies[i]);
        }
        Tracer.ProfileEnd(TraceName.UpdateProxies);
        SetTime(Timings.UpdateProxies);

        Tracer.ProfileBegin(TraceName.OptimizeBroadPhase);
        if (EnableAutomaticOptimization) OptimizeIncrementally(AutomaticOptimizationBudget);
        Tracer.ProfileEnd(TraceName.OptimizeBroadPhase);
        SetTime(Timings.Optimize);

        Tracer.ProfileBegin(TraceName.ScanOverlaps);
        if (multiThread) movedProxies.ParallelForBatch(24, scanForOverlaps);
        else ScanForOverlapsCallback(new Parallel.Batch(0, movedProxies.Count));
        Tracer.ProfileEnd(TraceName.ScanOverlaps);
        SetTime(Timings.ScanOverlaps);

        // 确保我们不会在 SlimBag<T> 数据结构的内部数组中持有过多
        // 可能导致 GC 无法回收的悬空引用。但每次只释放一个，以避免额外开销。
        movedProxies.TrackAndNullOutOne();
    }

    private Real stepDt;

    private void UpdateBoundingBoxesCallback(Parallel.Batch batch)
    {
        for (int i = batch.Start; i < batch.End; i++)
        {
            var proxy = proxies[i];
            if (proxy is IUpdatableBoundingBox sh) sh.UpdateWorldBoundingBox(stepDt);
        }
    }

    private void ThrowIfUnregistered(IDynamicTreeProxy proxy)
    {
        ArgumentNullException.ThrowIfNull(proxy);
        if (!proxies.Contains(proxy))
            throw new InvalidOperationException("The proxy is not registered with this tree instance.");
    }

    /// <summary>
    /// 强制立即更新树中的单个代理。
    /// </summary>
    /// <typeparam name="T">代理类型。</typeparam>
    /// <param name="proxy">要更新的代理。</param>
    /// <exception cref="InvalidOperationException">如果该代理未注册到此树中，则抛出。</exception>
    public void Update<T>(T proxy) where T : class, IDynamicTreeProxy
        => Update(proxy, (Real)0.0);

    /// <summary>
    /// 使用指定的时间步强制立即更新树中的单个代理。
    /// </summary>
    /// <typeparam name="T">代理类型。</typeparam>
    /// <param name="proxy">要更新的代理。</param>
    /// <param name="dt">更新包围盒时传递给 <see cref="IUpdatableBoundingBox"/> 代理的时间步。</param>
    /// <exception cref="InvalidOperationException">如果该代理未注册到此树中，则抛出。</exception>
    public void Update<T>(T proxy, Real dt) where T : class, IDynamicTreeProxy
    {
        ThrowIfUnregistered(proxy);
        if (proxy is IUpdatableBoundingBox sh) sh.UpdateWorldBoundingBox(dt);
        OverlapCheckRemove(root, proxy.NodePtr);
        InternalRemoveProxy(proxy);
        InternalAddProxy(proxy);
        OverlapCheckAdd(root, proxy.NodePtr);
    }

    /// <summary>
    /// 向树中添加一个代理。
    /// </summary>
    /// <typeparam name="T">代理类型。</typeparam>
    /// <param name="proxy">要添加的代理。</param>
    /// <param name="active">如果为 <c>true</c>，则每次更新时跟踪该代理的移动。</param>
    /// <exception cref="ArgumentNullException">如果 <paramref name="proxy"/> 为 null，则抛出。</exception>
    /// <exception cref="ArgumentException">
    /// 当 <paramref name="proxy"/> 已注册到此树实例时抛出。
    /// </exception>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="proxy"/> 的包围盒过大，无法进行稳健的树优化时抛出。
    /// </exception>
    public void AddProxy<T>(T proxy, bool active = true) where T : class, IDynamicTreeProxy
    {
        ArgumentNullException.ThrowIfNull(proxy);
        if (proxies.Contains(proxy))
        {
            throw new ArgumentException(
                $"The proxy '{proxy}' has already been added to this tree instance.", nameof(proxy));
        }

        // 2^53（约 9e15）是双精度数值丢失整数精度的极限
        // （即 x + 1.0 == x）。超过此表面积后，表面积启发式（SAH）
        // 无法检测到细微变化，导致树的插入和优化性能下降。
        //
        // 注意：由于 TreeBox 使用 'double' 计算表面积，此断言
        // 在 Single（float）和 Double（double）精度构建中都是有效且必要的。
        double surfaceArea = proxy.WorldBoundingBox.GetSurfaceArea();
        if (surfaceArea > 9.007e15)
        {
            throw new ArgumentOutOfRangeException(
                nameof(proxy), surfaceArea,
                "The proxy's bounding box is too large for robust tree optimization.");
        }

        InternalAddProxy(proxy, balance: true);
        int node = proxy.NodePtr;

        try
        {
            OverlapCheckAdd(root, node);
            proxies.Add(proxy, active);
        }
        catch
        {
            // 过滤器可能在已添加部分配对后抛出异常。
            for (int i = 0; i < potentialPairs.Slots.Length; i++)
            {
                var pair = potentialPairs.Slots[i];
                if (pair.ID == 0 || (pair.ID1 != node && pair.ID2 != node)) continue;

                potentialPairs.Remove(i);
                i = -1; // 移除操作可能移动条目或调整表的大小。
            }

            if (proxies.Contains(proxy)) proxies.Remove(proxy);
            InternalRemoveProxy(proxy, balance: true);
            proxy.NodePtr = NullNode;
            throw;
        }
    }

    /// <summary>
    /// 检查指定代理当前是否处于活动状态。
    /// </summary>
    /// <typeparam name="T">代理类型。</typeparam>
    /// <param name="proxy">要检查的代理。</param>
    /// <returns>如果代理处于活动状态，则为 <c>true</c>；否则为 <c>false</c>。未注册的代理返回 false。</returns>
    public bool IsActive<T>(T proxy) where T : class, IDynamicTreeProxy
    {
        ArgumentNullException.ThrowIfNull(proxy);
        return proxies.Contains(proxy) && proxies.IsActive(proxy);
    }

    /// <summary>
    /// 将代理标记为活动状态，使其在更新期间被跟踪移动。
    /// </summary>
    /// <typeparam name="T">代理类型。</typeparam>
    /// <param name="proxy">要激活的代理。</param>
    /// <exception cref="InvalidOperationException">如果该代理未注册到此树中，则抛出。</exception>
    public void ActivateProxy<T>(T proxy) where T : class, IDynamicTreeProxy
    {
        ThrowIfUnregistered(proxy);
        if (proxies.MoveToActive(proxy))
        {
            nodes[proxy.NodePtr].ForceUpdate = true;
        }
    }

    /// <summary>
    /// 将代理标记为非活动状态，假定它不会移出其扩展包围盒。
    /// </summary>
    /// <typeparam name="T">代理类型。</typeparam>
    /// <param name="proxy">要停用的代理。</param>
    /// <exception cref="InvalidOperationException">如果该代理未注册到此树中，则抛出。</exception>
    public void DeactivateProxy<T>(T proxy) where T : class, IDynamicTreeProxy
    {
        ThrowIfUnregistered(proxy);
        proxies.MoveToInactive(proxy);
    }

    /// <summary>
    /// 从树中移除一个代理。
    /// </summary>
    /// <param name="proxy">要移除的代理。</param>
    /// <exception cref="InvalidOperationException">如果该代理不在此树中，则抛出。</exception>
    public void RemoveProxy(IDynamicTreeProxy proxy)
    {
        ThrowIfUnregistered(proxy);

        OverlapCheckRemove(root, proxy.NodePtr);
        InternalRemoveProxy(proxy, balance: true);
        proxy.NodePtr = NullNode;
        proxies.Remove(proxy);
    }

    /// <summary>
    /// 计算树的 SAH 代价（所有节点表面积之和）。
    /// </summary>
    /// <returns>总代价。值越小表示空间分组越好。</returns>
    public double CalculateCost()
    {
        if (root == NullNode) return 0.0d;
        return Cost(ref nodes[root]);
    }

    /// <summary>
    /// 枚举树中所有轴对齐包围盒。
    /// </summary>
    /// <param name="action">要对树中每个包围盒和节点高度执行的操作。</param>
    public void EnumerateTreeBoxes(Action<TreeBox, int> action)
    {
        if (root == -1) return;
        EnumerateTreeBoxes(ref nodes[root], action);
    }

    private void EnumerateTreeBoxes(ref Node node, Action<TreeBox, int> action, int depth = 1)
    {
        action(node.ExpandedBox, depth);
        if (node.IsLeaf) return;

        EnumerateTreeBoxes(ref nodes[node.Left], action, depth + 1);
        EnumerateTreeBoxes(ref nodes[node.Right], action, depth + 1);
    }

    private uint stepper;

    /// <summary>
    /// 当两个代理都处于非活动状态，或其扩展包围盒不再重叠时，
    /// 从内部记录中移除相应条目。
    /// 每次调用只搜索所有元素的一小部分，以减少开销。
    /// </summary>
    private void PruneInvalidPairs()
    {
        stepper += 1;

        for (int i = 0; i < potentialPairs.Slots.Length / PruningFraction; i++)
        {
            int t = (int)((i * PruningFraction + stepper) % potentialPairs.Slots.Length);

            var n = potentialPairs.Slots[t];
            if (n.ID == 0) continue;

            var proxyA = nodes[n.ID1].Proxy;
            var proxyB = nodes[n.ID2].Proxy;

            if (proxyA != null && proxyB != null &&
                !TreeBox.Disjoint(nodes[proxyA.NodePtr].ExpandedBox, nodes[proxyB.NodePtr].ExpandedBox) &&
                (proxies.IsActive(proxyA) || proxies.IsActive(proxyB)))
            {
                continue;
            }

            potentialPairs.Remove(t);
            i -= 1;
        }
    }

    [ThreadStatic] private static Stack<int>? _stack;

    private static Stack<int> QueryStack => _stack ??= new Stack<int>(256);

    private static void PopTo(Stack<int> stack, int count)
    {
        while (stack.Count > count) stack.Pop();
    }

    /// <summary>
    /// 查询树中世界包围盒与射线相交的代理，并将候选对象追加到接收器。
    /// </summary>
    /// <typeparam name="TSink">接收候选代理的接收器类型。</typeparam>
    /// <param name="hits">接收候选代理的接收器；它们的形状可能并不与射线相交。</param>
    /// <param name="rayOrigin">射线的起点。</param>
    /// <param name="rayDirection">射线的方向。</param>
    public void Query<TSink>(ref TSink hits, in JVector rayOrigin, in JVector rayDirection)
        where TSink : ISink<IDynamicTreeProxy>
    {
        if (root == NullNode) return;

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
                    if (node.Proxy!.WorldBoundingBox.RayIntersect(rayOrigin, rayDirection, out _))
                    {
                        hits.Add(node.Proxy);
                    }

                    continue;
                }

                int left = node.Left;
                int right = node.Right;

                if (nodes[left].ExpandedBox.RayIntersect(rayOrigin, rayDirection, out _))
                {
                    stack.Push(left);
                }

                if (nodes[right].ExpandedBox.RayIntersect(rayOrigin, rayDirection, out _))
                {
                    stack.Push(right);
                }
            }
        }
        finally
        {
            PopTo(stack, baseCount);
        }
    }

    /// <summary>
    /// 查询树中与轴对齐包围盒重叠的代理，并将所有命中追加到指定接收器。
    /// </summary>
    /// <typeparam name="TSink">接收所有重叠代理的接收器类型。</typeparam>
    /// <param name="hits">接收所有重叠代理的接收器。</param>
    /// <param name="box">要查询的包围盒。</param>
    public void Query<TSink>(ref TSink hits, in JBoundingBox box)
        where TSink : ISink<IDynamicTreeProxy>
    {
        if (root == NullNode) return;

        var sbox = new TreeBox(box);

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
                    if (!JBoundingBox.Disjoint(node.Proxy!.WorldBoundingBox, box))
                    {
                        hits.Add(node.Proxy);
                    }

                    continue;
                }

                int left = node.Left;
                int right = node.Right;

                if (!TreeBox.Disjoint(nodes[left].ExpandedBox, sbox))
                {
                    stack.Push(left);
                }

                if (!TreeBox.Disjoint(nodes[right].ExpandedBox, sbox))
                {
                    stack.Push(right);
                }
            }
        }
        finally
        {
            PopTo(stack, baseCount);
        }
    }

    /// <summary>
    /// 查询树中世界包围盒与射线相交的代理。
    /// </summary>
    /// <typeparam name="T">集合类型。</typeparam>
    /// <param name="hits">用于存储候选代理的集合；它们的形状可能并不与射线相交。</param>
    /// <param name="rayOrigin">射线的起点。</param>
    /// <param name="rayDirection">射线的方向。</param>
    public void Query<T>(T hits, in JVector rayOrigin, in JVector rayDirection)
        where T : class, ICollection<IDynamicTreeProxy>
    {
        var sink = new CollectionSink<IDynamicTreeProxy>(hits);
        Query(ref sink, in rayOrigin, in rayDirection);
    }

    /// <summary>
    /// 查询树中与轴对齐包围盒重叠的代理。
    /// </summary>
    /// <typeparam name="T">集合类型。</typeparam>
    /// <param name="hits">用于存储重叠代理的集合。</param>
    /// <param name="box">要查询的包围盒。</param>
    public void Query<T>(T hits, in JBoundingBox box)
        where T : class, ICollection<IDynamicTreeProxy>
    {
        var sink = new CollectionSink<IDynamicTreeProxy>(hits);
        Query(ref sink, in box);
    }

    readonly List<IDynamicTreeProxy> tempList = new();

    /// <summary>
    /// 尝试通过随机重新插入现有叶节点来改善树拓扑。
    /// </summary>
    /// <param name="sweeps">同步优化遍数。必须大于零。</param>
    /// <param name="chance">每个概率性扫描中重新插入每个代理的概率。范围：[0, 1]。</param>
    /// <param name="incremental">
    /// 如果为 <c>false</c>，则在第一遍中按随机顺序重新插入所有代理。如果
    /// 为 <c>true</c>，则每一遍根据 <paramref name="chance"/> 选择代理。
    /// 此选项不会在多次调用或模拟步之间分摊工作。
    /// </param>
    /// <remarks>
    /// 优化会保留叶节点索引、扩展包围盒、代理活动状态以及
    /// 潜在配对缓存。该操作是随机的，不保证每次调用都能降低
    /// <see cref="CalculateCost"/>。
    /// </remarks>
    /// <exception cref="ArgumentOutOfRangeException">
    /// 当 <paramref name="sweeps"/> 小于一，或 <paramref name="chance"/> 超出 [0, 1] 时抛出。
    /// </exception>
    public void Optimize(int sweeps = 100, Real chance = (Real)0.01, bool incremental = false)
    {
        Optimize(rndFunc, sweeps, chance, incremental);
    }

    /// <inheritdoc cref="Optimize(int, Real, bool)"/>
    /// <param name="getNextRandom">返回 [0, 1) 范围内随机值的函数。</param>
    /// <param name="sweeps">优化扫描遍数。</param>
    /// <param name="chance">每遍中选中每个叶节点的概率，从 0 到 1。</param>
    /// <param name="incremental">是否在优化期间增量更新树。</param>
    /// <exception cref="ArgumentNullException">当 <paramref name="getNextRandom"/> 为 <see langword="null"/> 时抛出。</exception>
    public void Optimize(Func<double> getNextRandom, int sweeps, Real chance, bool incremental)
    {
        ArgumentNullException.ThrowIfNull(getNextRandom);
        if (sweeps <= 0) throw new ArgumentOutOfRangeException(nameof(sweeps), "Sweeps must be greater than zero.");
        ArgumentCheck.InRange(chance, (Real)0.0, (Real)1.0, nameof(chance));

        for (int e = 0; e < sweeps; e++)
        {
            bool takeAll = (e == 0) && !incremental;

            try
            {
                for (int i = 0; i < proxies.Count; i++)
                {
                    if (!takeAll && getNextRandom() > chance) continue;

                    var proxy = proxies[i];
                    tempList.Add(proxy);
                    int leaf = proxy.NodePtr;
                    RemoveLeaf(leaf, balance: true);
                    nodes[leaf].Parent = NullNode;
                }

                // Fisher-Yates 洗牌
                int n = tempList.Count;

                for (int i = n - 1; i > 0; i--)
                {
                    double scaledValue = getNextRandom() * (i + 1);
                    int j = (int)scaledValue;
                    (tempList[i], tempList[j]) = (tempList[j], tempList[i]);
                }

            }
            finally
            {
                foreach (var proxy in tempList)
                {
                    InsertLeaf(proxy.NodePtr, root, balance: true);
                }

                tempList.Clear();
            }
        }

        if (!incremental)
        {
            // 在非增量模式下，第一遍会处理所有代理，可能导致
            // tempList 显著增长。由于我们不太可能再次需要该
            // 容量，因此裁剪多余部分以减少内存占用。
            tempList.TrimExcess();
        }
    }

    private int AllocateNode()
    {
        int result;

        if (freeNodes.Count > 0)
        {
            result = freeNodes.Pop();
            return result;
        }

        nodePointer += 1;
        if (nodePointer == nodes.Length)
        {
            Array.Resize(ref nodes, nodes.Length * 2);
            Logger.Information("{0}: Resized array of tree to {1} elements.", nameof(DynamicTree), nodes.Length);
        }

        return nodePointer;
    }

    private void FreeNode(int node)
    {
        nodes[node] = new Node
        {
            Left = NullNode,
            Right = NullNode,
            Parent = NullNode
        };
        freeNodes.Push(node);
    }

    private double Cost(ref Node node)
    {
        if (node.IsLeaf)
        {
            return node.ExpandedBox.GetSurfaceArea();
        }

        return node.ExpandedBox.GetSurfaceArea() + Cost(ref nodes[node.Left]) + Cost(ref nodes[node.Right]);
    }

    private void OverlapCheckAdd(int index, int node)
    {
        if (nodes[index].IsLeaf)
        {
            if (node == index) return;
            if (!Filter(nodes[node].Proxy!, nodes[index].Proxy!)) return;
            potentialPairs.ConcurrentAdd(new PairHashSet.Pair(index, node));
        }
        else
        {
            int child1 = nodes[index].Left;
            int child2 = nodes[index].Right;

            if (!TreeBox.Disjoint(nodes[child1].ExpandedBox, nodes[node].ExpandedBox))
                OverlapCheckAdd(child1, node);

            if (!TreeBox.Disjoint(nodes[child2].ExpandedBox, nodes[node].ExpandedBox))
                OverlapCheckAdd(child2, node);
        }
    }

    private void OverlapCheckRemove(int index, int node)
    {
        if (nodes[index].IsLeaf)
        {
            if (node == index) return;
            if (!Filter(nodes[node].Proxy!, nodes[index].Proxy!)) return;
            potentialPairs.Remove(new PairHashSet.Pair(index, node));
        }
        else
        {
            int child1 = nodes[index].Left;
            int child2 = nodes[index].Right;

            if (!TreeBox.Disjoint(nodes[child1].ExpandedBox, nodes[node].ExpandedBox))
                OverlapCheckRemove(child1, node);

            if (!TreeBox.Disjoint(nodes[child2].ExpandedBox, nodes[node].ExpandedBox))
                OverlapCheckRemove(child2, node);
        }
    }

    private void ScanForMovedProxies(Parallel.Batch batch)
    {
        for (int i = batch.Start; i < batch.End; i++)
        {
            var proxy = proxies[i];

            ref var node = ref nodes[proxy.NodePtr];

            if (node.ForceUpdate || !node.ExpandedBox.Contains(proxy.WorldBoundingBox))
            {
                node.ForceUpdate = false;
                movedProxies.ConcurrentAdd(proxy);
            }
        }
    }

    private void ScanForOverlapsCallback(Parallel.Batch batch)
    {
        for (int i = batch.Start; i < batch.End; i++)
        {
            OverlapCheckAdd(root, movedProxies[i].NodePtr);
        }
    }

    private static void ExpandBoundingBox(ref JBoundingBox box, in JVector direction)
    {
        if (direction.X < (Real)0.0) box.Min.X += direction.X;
        else box.Max.X += direction.X;

        if (direction.Y < (Real)0.0) box.Min.Y += direction.Y;
        else box.Max.Y += direction.Y;

        if (direction.Z < (Real)0.0) box.Min.Z += direction.Z;
        else box.Max.Z += direction.Z;

        box.Min -= new JVector(ExpandEps);
        box.Max += new JVector(ExpandEps);
    }

    private void InternalAddRemoveProxy(IDynamicTreeProxy proxy)
    {
        JBoundingBox box = proxy.WorldBoundingBox;

        // 我们保存被移除节点的父节点。稍后利用该信息重新插入该节点，
        // *使更新保持局部*。
        int parent = RemoveLeaf(proxy.NodePtr);

        int index = proxy.NodePtr;

        Real pseudoRandomExt = (Real)random.NextDouble();

        ExpandBoundingBox(ref box, proxy.Velocity * ExpandFactor * ((Real)1.0 + pseudoRandomExt));

        nodes[index].Left = NullNode;
        nodes[index].Right = NullNode;
        nodes[index].Proxy = proxy;
        nodes[index].LeafCount = 1;
        nodes[index].Height = 0;
        proxy.NodePtr = index;

        nodes[index].ExpandedBox = new TreeBox(box);

        // InsertLeaf 将 'where' 作为提示，即它仍会沿树向上遍历，
        // 直到新节点被完全包含。注意：从根节点搜索也能找到插入节点，
        // 因为搜索总是向下进入完全包含新节点的子节点。
        InsertLeaf(index, parent);
    }

    private void InternalAddProxy(IDynamicTreeProxy proxy, bool balance = false)
    {
        JBoundingBox box = proxy.WorldBoundingBox;

        int index = AllocateNode();

        nodes[index] = new Node
        {
            Left = NullNode,
            Right = NullNode,
            Parent = NullNode,
            Proxy = proxy,
            LeafCount = 1,
            Height = 0,
            ExpandedBox = new TreeBox(box)
        };
        proxy.NodePtr = index;

        InsertLeaf(index, root, balance);
    }

    private void InternalRemoveProxy(IDynamicTreeProxy proxy, bool balance = false)
    {
        Debug.Assert(nodes[proxy.NodePtr].IsLeaf);
        RemoveLeaf(proxy.NodePtr, balance);
        FreeNode(proxy.NodePtr);
    }

    private int RemoveLeaf(int node, bool balance = false)
    {
        if (node == root)
        {
            root = NullNode;
            return NullNode;
        }

        int parent = nodes[node].Parent;
        int grandParent = nodes[parent].Parent;

        int sibling = nodes[parent].Left == node ? nodes[parent].Right : nodes[parent].Left;

        if (grandParent == NullNode)
        {
            root = sibling;
            nodes[sibling].Parent = NullNode;
            FreeNode(parent);
            return root;
        }

        if (nodes[grandParent].Left == parent) nodes[grandParent].Left = sibling;
        else nodes[grandParent].Right = sibling;

        nodes[sibling].Parent = grandParent;
        FreeNode(parent);

        if (balance) RefitAndBalanceAncestors(grandParent);
        else RefitAncestors(grandParent);

        return grandParent;
    }

    private readonly PriorityQueue<int, double> priorityQueue = new();

    /// <summary>
    /// 使用优先队列驱动的表面积启发式（SAH）搜索，从 <paramref name="where"/> 开始，
    /// 为 <paramref name="node"/> 在 BVH 中找到确切的最佳插入点。
    /// 与更快的贪心版本 <see cref="FindBestGreedy"/> 不同，此方法保证
    /// 与暴力搜索相同的结果。
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    private int FindBest(int node, int where)
    {
        ref Node nb = ref nodes[node];

        double rootMergedArea = TreeBox.MergedSurface(nodes[where].ExpandedBox, nb.ExpandedBox);

        double bestCost = double.MaxValue;
        int currentBest = where;

        // value = 节点索引，priority = 该节点的完整代价
        priorityQueue.Enqueue(where, rootMergedArea);

        while (priorityQueue.TryDequeue(out int currentIndex, out double cost))
        {
            ref Node cn = ref nodes[currentIndex];

            // 重建该节点*之前*的继承代价：
            // cost = inhCostBefore + SA(merge(node, nb))
            double mergedHere = TreeBox.MergedSurface(cn.ExpandedBox, nb.ExpandedBox);
            double inhCostBeforeNode = cost - mergedHere;

            // 剪枝：即使下界也已比当前最优更差
            if (inhCostBeforeNode > bestCost)
                continue;

            // 该节点是一个候选插入位置
            if (cost < bestCost)
            {
                bestCost = cost;
                currentBest = currentIndex;
            }

            if (cn.IsLeaf) continue;

            double oldSurface = cn.ExpandedBox.GetSurfaceArea();

            // 该节点*之后*的继承代价
            double inhCostAfterNode = cost - oldSurface;

            // 向子节点扩展
            double leftMerged = TreeBox.MergedSurface(nodes[cn.Left].ExpandedBox, nb.ExpandedBox);
            double rightMerged = TreeBox.MergedSurface(nodes[cn.Right].ExpandedBox, nb.ExpandedBox);

            double leftCost = inhCostAfterNode + leftMerged;
            double rightCost = inhCostAfterNode + rightMerged;

            // 只存储索引与完整代价；inhCost 将在出队时重建
            priorityQueue.Enqueue(cn.Left, leftCost);
            priorityQueue.Enqueue(cn.Right, rightCost);
        }

        return currentBest;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    private int FindBestGreedy(int node, int where)
    {
        /*
        This method is adapted from Box2D.
        https://github.com/erincatto/box2d/blob/3a4f0da8374af61293a03021c9a0b3ebcfe67948/src/dynamic_tree.c#L187
        Modified from the original version.

        Box2D is available under the MIT license:

        MIT License

        Copyright (c) 2022 Erin Catto

        Permission is hereby granted, free of charge, to any person obtaining a copy
        of this software and associated documentation files (the "Software"), to deal
        in the Software without restriction, including without limitation the rights
        to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
        copies of the Software, and to permit persons to whom the Software is
        furnished to do so, subject to the following conditions:

        The above copyright notice and this permission notice shall be included in all
        copies or substantial portions of the Software.

        THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
        IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
        FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
        AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
        LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
        OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
        SOFTWARE.
        */

        ref TreeBox nodeTreeBox = ref nodes[node].ExpandedBox;

        double areaD = nodes[node].ExpandedBox.GetSurfaceArea();
        double areaBase = nodes[where].ExpandedBox.GetSurfaceArea();
        double directCost = TreeBox.MergedSurface(nodes[where].ExpandedBox, nodeTreeBox);
        double inheritedCost = 0.0d;

        int bestSibling = where;
        double bestCost = directCost;

        while (!nodes[where].IsLeaf)
        {
            int left = nodes[where].Left;
            int right = nodes[where].Right;

            double cost = directCost + inheritedCost;

            if (cost < bestCost)
            {
                bestSibling = where;
                bestCost = cost;
            }

            inheritedCost += directCost - areaBase;

            //
            // 下探到左子节点的代价
            //
            double lowerCostLeft = double.MaxValue;
            double directCostLeft = TreeBox.MergedSurface(nodes[left].ExpandedBox, nodeTreeBox);
            double areaLeft = 0.0d;

            if (nodes[left].IsLeaf)
            {
                // 左子节点是叶节点
                // 创建新节点并增大节点 P 面积的代价
                double costLeft = directCostLeft + inheritedCost;

                // 由于上面的 while 条件，需要在此处处理
                if (costLeft < bestCost)
                {
                    bestSibling = left;
                    bestCost = costLeft;
                }
            }
            else
            {
                // 左子节点是内部节点
                areaLeft = nodes[left].ExpandedBox.GetSurfaceArea();

                // 在左子节点下插入的下界代价。
                lowerCostLeft = inheritedCost + directCostLeft + double.Min(areaD - areaLeft, 0.0d);
            }

            //
            // 下探到右子节点的代价
            //
            double lowerCostRight = double.MaxValue;
            double directCostRight = TreeBox.MergedSurface(nodes[right].ExpandedBox, nodeTreeBox);
            double areaRight = 0.0d;

            if (nodes[right].IsLeaf)
            {
                // 右子节点是叶节点
                double costRight = directCostRight + inheritedCost;

                if (costRight < bestCost)
                {
                    bestSibling = right;
                    bestCost = costRight;
                }
            }
            else
            {
                // 右子节点是内部节点
                areaRight = nodes[right].ExpandedBox.GetSurfaceArea();
                lowerCostRight = inheritedCost + directCostRight + double.Min(areaD - areaRight, 0.0d);
            }

            // 如果两个子树都不能带来改进，则停止下探。
            if (bestCost <= lowerCostLeft && bestCost <= lowerCostRight)
                break;

            // 依据与新节点中心的接近程度来打破平局
            // ReSharper disable once CompareOfFloatsByEqualityOperator
            if (lowerCostLeft == lowerCostRight)
            {
                var center = nodeTreeBox.Center;
                lowerCostLeft = (nodes[left].ExpandedBox.Center - center).LengthSquared();
                lowerCostRight = (nodes[right].ExpandedBox.Center - center).LengthSquared();
            }

            // 下探到更优的那个子节点
            if (lowerCostLeft < lowerCostRight)
            {
                where = left;
                areaBase = areaLeft;
                directCost = directCostLeft;
            }
            else
            {
                where = right;
                areaBase = areaRight;
                directCost = directCostRight;
            }
        }

        return bestSibling;
    }


    [MethodImpl(MethodImplOptions.AggressiveInlining | MethodImplOptions.AggressiveOptimization)]
    private int FindBestHeuristic(int node, int where)
    {
        ref TreeBox nodeTreeBox = ref nodes[node].ExpandedBox;

        while (!nodes[where].IsLeaf)
        {
            int left = nodes[where].Left;
            int right = nodes[where].Right;

            double cost = 2.0d * nodes[where].ExpandedBox.GetSurfaceArea();

            double leftCost, rightCost;

            if (nodes[left].IsLeaf)
            {
                // 额外节点的代价
                leftCost = TreeBox.MergedSurface(nodes[left].ExpandedBox, nodeTreeBox);
            }
            else
            {
                // 下探的代价
                double oldArea = nodes[left].ExpandedBox.GetSurfaceArea();
                double newArea = TreeBox.MergedSurface(nodes[left].ExpandedBox, nodeTreeBox);
                leftCost = newArea - oldArea;
            }

            if (nodes[right].IsLeaf)
            {
                rightCost = TreeBox.MergedSurface(nodes[right].ExpandedBox, nodeTreeBox);
            }
            else
            {
                double oldArea = nodes[right].ExpandedBox.GetSurfaceArea();
                double newArea = TreeBox.MergedSurface(nodes[right].ExpandedBox, nodeTreeBox);
                rightCost = newArea - oldArea;
            }

            if (cost < leftCost && cost < rightCost) break;

            where = leftCost < rightCost ? left : right;
        }

        return where;
    }

    private void InsertLeaf(int node, int where, bool balance = false)
    {
        if (root == NullNode)
        {
            root = node;
            nodes[root].Parent = NullNode;
            return;
        }

        ref TreeBox nodeTreeBox = ref nodes[node].ExpandedBox;

        while (where != root)
        {
            if (TreeBox.Contains(nodes[where].ExpandedBox, nodeTreeBox))
            {
                break;
            }

            where = nodes[where].Parent;
        }

        // 搜索最佳兄弟节点
        int sibling = FindBestGreedy(node, where);

        // 创建新的父节点
        int oldParent = nodes[sibling].Parent;
        int newParent = AllocateNode();

        nodes[newParent] = new Node
        {
            Left = sibling,
            Right = node,
            Parent = oldParent,
            LeafCount = nodes[sibling].LeafCount + nodes[node].LeafCount,
            Height = 1 + int.Max(nodes[sibling].Height, nodes[node].Height)
        };

        if (oldParent != NullNode)
        {
            if (nodes[oldParent].Left == sibling) nodes[oldParent].Left = newParent;
            else nodes[oldParent].Right = newParent;

            nodes[sibling].Parent = newParent;
            nodes[node].Parent = newParent;
        }
        else
        {
            nodes[sibling].Parent = newParent;
            nodes[node].Parent = newParent;
            root = newParent;
        }

        if (balance) RefitAndBalanceAncestors(newParent);
        else RefitAncestors(newParent);
    }
}
