/*
 * Jitter2 Physics Library
 * (c) Thorben Linneweber and contributors
 * SPDX-License-Identifier: MIT
 */

using System;
using System.Diagnostics;
using System.Runtime.CompilerServices;

namespace Jitter2.Collision;

public partial class DynamicTree
{
    /// <summary>自动树维护重新插入的叶节点默认数量。</summary>
    public const int DefaultIncrementalOptimizationBudget = 1;

    /// <summary>
    /// 获取或设置 <see cref="Update(bool, Real)"/> 是否从根节点重新插入有限数量的
    /// 叶节点，并对受影响的路径应用不增加代价的旋转。默认启用。
    /// </summary>
    public bool EnableAutomaticOptimization { get; set; } = true;

    /// <summary>
    /// 获取或设置每次 <see cref="Update(bool, Real)"/> 重新插入的叶节点最大数量。
    /// </summary>
    /// <exception cref="ArgumentOutOfRangeException">当值为负数时抛出。</exception>
    public int AutomaticOptimizationBudget
    {
        get => automaticOptimizationBudget;
        set
        {
            if (value < 0)
            {
                throw new ArgumentOutOfRangeException(nameof(value),
                    "The automatic optimization budget cannot be negative.");
            }

            automaticOptimizationBudget = value;
        }
    }

    private int automaticOptimizationBudget = DefaultIncrementalOptimizationBudget;
    private int incrementalOptimizationCursor;

    private readonly struct RotationCandidate(
        int promoted, int moved, int maximumImbalance, int height,
        int totalHeightChange, double costChange)
    {
        public readonly int Promoted = promoted;
        public readonly int Moved = moved;
        public readonly int MaximumImbalance = maximumImbalance;
        public readonly int Height = height;
        public readonly int TotalHeightChange = totalHeightChange;
        public readonly double CostChange = costChange;
    }

    /// <summary>
    /// 从根节点重新插入至多 <paramref name="proxyBudget"/> 个叶节点。
    /// </summary>
    /// <param name="proxyBudget">要重新插入的叶节点最大数量。</param>
    /// <returns>实际重新插入的叶节点数量。</returns>
    /// <remarks>
    /// 持久的游标无需扫描代理集合即可选取叶节点。叶节点索引、
    /// 代理活动状态和潜在形状对缓存均保持不变。从根节点重新插入
    /// 会逐步消除拓扑历史，而受代价约束的旋转可以在不使每次普通树变更
    /// 都承担旋转开销的情况下修复高度失衡。
    /// </remarks>
    private int OptimizeIncrementally(
        int proxyBudget = DefaultIncrementalOptimizationBudget)
    {
        if (proxyBudget < 0)
        {
            throw new ArgumentOutOfRangeException(nameof(proxyBudget),
                "The proxy budget cannot be negative.");
        }

        int count = int.Min(proxyBudget, proxies.Count);

        for (int i = 0; i < count; i++)
        {
            if (incrementalOptimizationCursor >= proxies.Count)
            {
                incrementalOptimizationCursor = 0;
            }

            IDynamicTreeProxy proxy = proxies[incrementalOptimizationCursor++];
            int leaf = proxy.NodePtr;

            RemoveLeaf(leaf, balance: true);
            nodes[leaf].Parent = NullNode;
            InsertLeaf(leaf, root, balance: true);
        }

        return count;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private void RefitNode(int node)
    {
        Debug.Assert(!nodes[node].IsLeaf);

        int left = nodes[node].Left;
        int right = nodes[node].Right;

        TreeBox.CreateMerged(nodes[left].ExpandedBox, nodes[right].ExpandedBox,
            out nodes[node].ExpandedBox);
        nodes[node].LeafCount = nodes[left].LeafCount + nodes[right].LeafCount;
        nodes[node].Height = 1 + int.Max(nodes[left].Height, nodes[right].Height);
    }

    private void RefitAndBalanceAncestors(int node)
    {
        while (node != NullNode)
        {
            RefitNode(node);
            int subtreeRoot = BalanceNode(node);
            node = nodes[subtreeRoot].Parent;
        }
    }

    private void RefitAncestors(int node)
    {
        while (node != NullNode)
        {
            RefitNode(node);
            node = nodes[node].Parent;
        }
    }

    private int BalanceNode(int node)
    {
        if (nodes[node].IsLeaf) return node;
        return RestoreHeightBalanceWithoutIncreasingCost(node);
    }

    private int RestoreHeightBalanceWithoutIncreasingCost(int node)
    {
        int left = nodes[node].Left;
        int right = nodes[node].Right;
        int balance = nodes[right].Height - nodes[left].Height;

        if (balance is >= -1 and <= 1) return node;

        int promoted = balance > 1 ? right : left;
        Debug.Assert(!nodes[promoted].IsLeaf);

        RotationCandidate first = CreateRotationCandidate(
            node, promoted, nodes[promoted].Left);
        RotationCandidate second = CreateRotationCandidate(
            node, promoted, nodes[promoted].Right);

        bool firstAccepted = IsAcceptedHeightRotation(first);
        bool secondAccepted = IsAcceptedHeightRotation(second);

        if (!firstAccepted && !secondAccepted) return node;

        RotationCandidate selected = !firstAccepted ? second :
            !secondAccepted ? first : SelectBalancingRotation(first, second);
        int subtreeRoot = ApplyRotation(node, selected);

        // 新插入的叶节点可能与任意高的子树配对。
        // 在这种情况下，一次旋转会平衡新的根节点，但会使旧根节点保持倾斜。
        // 在再次检查新根节点之前，先在该子节点内部继续处理。当所有适用的旋转
        // 都会增加代价时，两种操作都可能留下高度失衡。
        RestoreHeightBalanceWithoutIncreasingCost(node);
        RefitNode(subtreeRoot);
        return RestoreHeightBalanceWithoutIncreasingCost(subtreeRoot);
    }

    private RotationCandidate CreateRotationCandidate(int node, int promoted, int moved)
    {
        int sibling = nodes[node].Left == promoted ? nodes[node].Right : nodes[node].Left;
        int stay = nodes[promoted].Left == moved ? nodes[promoted].Right : nodes[promoted].Left;

        int nodeHeight = 1 + int.Max(nodes[sibling].Height, nodes[moved].Height);
        int promotedHeight = 1 + int.Max(nodes[stay].Height, nodeHeight);
        int totalHeightChange = nodeHeight + promotedHeight -
            nodes[node].Height - nodes[promoted].Height;

        int nodeImbalance = int.Abs(nodes[sibling].Height - nodes[moved].Height);
        int promotedImbalance = int.Abs(nodes[stay].Height - nodeHeight);

        double newNodeArea = TreeBox.MergedSurface(
            nodes[sibling].ExpandedBox, nodes[moved].ExpandedBox);
        double costChange = newNodeArea - nodes[promoted].ExpandedBox.GetSurfaceArea();

        return new RotationCandidate(promoted, moved,
            int.Max(nodeImbalance, promotedImbalance), promotedHeight,
            totalHeightChange, costChange);
    }

    private static bool IsAcceptedHeightRotation(RotationCandidate candidate)
    {
        if (candidate.CostChange < 0.0d) return true;

        // 等代价旋转需要第二个严格递减的度量；否则两种
        // 等价的拓扑结构可能无限地来回旋转。
        return candidate.CostChange == 0.0d && candidate.TotalHeightChange < 0;
    }

    private static RotationCandidate SelectBalancingRotation(
        RotationCandidate first, RotationCandidate second)
    {
        bool firstBalanced = first.MaximumImbalance <= 1;
        bool secondBalanced = second.MaximumImbalance <= 1;

        if (firstBalanced != secondBalanced) return firstBalanced ? first : second;

        if (first.MaximumImbalance != second.MaximumImbalance)
        {
            return first.MaximumImbalance < second.MaximumImbalance ? first : second;
        }

        if (first.Height != second.Height) return first.Height < second.Height ? first : second;
        return first.CostChange <= second.CostChange ? first : second;
    }

    private int ApplyRotation(int node, RotationCandidate candidate)
    {
        int promoted = candidate.Promoted;
        int moved = candidate.Moved;
        int parent = nodes[node].Parent;
        bool promotedWasLeft = nodes[node].Left == promoted;
        bool movedWasLeft = nodes[promoted].Left == moved;

        if (parent == NullNode)
        {
            root = promoted;
        }
        else if (nodes[parent].Left == node)
        {
            nodes[parent].Left = promoted;
        }
        else
        {
            Debug.Assert(nodes[parent].Right == node);
            nodes[parent].Right = promoted;
        }

        nodes[promoted].Parent = parent;
        nodes[node].Parent = promoted;
        nodes[moved].Parent = node;

        if (promotedWasLeft) nodes[node].Left = moved;
        else nodes[node].Right = moved;

        if (movedWasLeft) nodes[promoted].Left = node;
        else nodes[promoted].Right = node;

        RefitNode(node);
        RefitNode(promoted);
        return promoted;
    }
}
