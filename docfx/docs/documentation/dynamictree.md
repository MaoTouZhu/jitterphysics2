# 动态树

动态树保存实现了 `IDynamicTreeProxy` 接口的实例。
它的主要任务是高效判断某个代理的轴对齐包围盒是否与世界中的任何其他代理的轴对齐包围盒重叠。
朴素实现需要 $\mathcal{O}\left(n\right)$ 次操作（与 $n-1$ 个实体逐一检查是否重叠）。
树结构将其加速到 $\mathcal{O}\left(\mathrm{log}(n)\right)$。
由于代理是动态的且可以移动，树必须持续更新。
为了降低更新频率，实体会被包裹在比其实际尺寸略大的包围盒中。
该包围盒扩展量由 `IDynamicTreeProxy` 接口的 `Velocity` 属性定义。

![动态树示意图](images/dynamictree.png)

## 添加代理

添加到刚体的所有形状（`body.AddShape`）都会自动注册到 `world.DynamicTree`。
`IDynamicTreeProxy` 的自定义实现可以通过 `tree.AddProxy` 添加到树中。
在这种情况下，必须实现并注册 `BroadPhaseFilter`（通过 `world.BroadPhaseFilter`）来处理与自定义代理的碰撞，否则会抛出 `InvalidCollisionTypeException`。

## 枚举重叠

树实现需要使用 `tree.Update` 更新。
对于世界类所拥有的动态树（`world.DynamicTree`），此操作是自动完成的。
内部会为实现了 `IUpdatableBoundingBox` 接口的活动代理调用 `UpdateWorldBoundingBox`，并更新重叠对内部记录。
重叠可以通过 `tree.EnumerateOverlaps` 查询。
要在修改某个形状的几何形状后刷新它，请调用 `world.DynamicTree.Update(shape)`。其他可能需要更新的状态见[修改已附加的形状](shapes.md#changing-an-attached-shape)。

## 查询树

动态树是一种宽相结构。
它的职责是快速缩小候选集合。
有些 API 止步于此并返回代理，而另一些则继续进行精确的窄相测试。

实际上查询分为两层：

- 仅宽相查询，例如 `Query(...)`，返回候选代理。
- 宽相 + 窄相查询，例如 `RayCast(...)`、`SweepCast(...)` 和 `FindNearest(...)`，会对受支持的代理继续进行精确测试。

### 仅宽相查询

可以查询与给定轴对齐盒重叠的所有树代理：

```cs
public void Query<T>(T hits, in JBoundingBox box) where T : class, ICollection<IDynamicTreeProxy>
public void Query<TSink>(ref TSink hits, in JBoundingBox box) where TSink : ISink<IDynamicTreeProxy>
```

以及所有与射线重叠的代理：

```cs
public void Query<T>(T hits, in JVector rayOrigin, in JVector rayDirection) where T : class, ICollection<IDynamicTreeProxy>
public void Query<TSink>(ref TSink hits, in JVector rayOrigin, in JVector rayDirection) where TSink : ISink<IDynamicTreeProxy>
```

自定义宽相查询可以很容易地实现。
下面是一个查询与单个点重叠的所有代理的实现：

```cs
var stack = new Stack<int>();
stack.Push(tree.Root);

ReadOnlySpan<DynamicTree.Node> nodes = tree.Nodes;

while (stack.TryPop(out int id))
{
    ref readonly DynamicTree.Node node = ref nodes[id];

    if (node.ExpandedBox.Contains(point))
    {
        if (node.IsLeaf)
        {
            Console.WriteLine($"{node.Proxy} contains {point}.");
        }
        else
        {
            stack.Push(node.Left);
            stack.Push(node.Right);
        }
    }
}
```

### 精确查询辅助方法

树还提供了将宽相剪枝与精确窄相测试相结合的辅助方法。
这些方法返回最近的精确命中结果，而不是原始候选代理。
所有精确查询辅助方法在成功时返回 `true`，未找到任何结果时返回 `false`。

#### 射线投射

所有实现 `IRayCastable` 的代理都可以进行射线投射，包括所有形状：

```cs
// 无界
public bool RayCast(JVector origin, JVector direction, RayCastFilterPre? pre, RayCastFilterPost? post,
    out IDynamicTreeProxy? proxy, out JVector normal, out float lambda)

// 有界：只考虑 lambda ≤ maxLambda 的命中
public bool RayCast(JVector origin, JVector direction, float maxLambda, RayCastFilterPre? pre, RayCastFilterPost? post,
    out IDynamicTreeProxy? proxy, out JVector normal, out float lambda)
```

前置过滤器和后置过滤器可用于在射线投射过程中丢弃命中。
射线从原点沿指定方向射出。
如果找到命中则返回 `true`，包括射线原点位于命中形状内部的情况（此时 `lambda` 为零，`normal` 为 `JVector.Zero`）。
碰撞点由 `hit = origin + lambda * direction` 给出。

#### 扫掠投射

所有实现 `ISweepTestable` 的代理都可以进行扫掠测试，包括所有形状：

```cs
// 无界
public bool SweepCast<T>(in T support, in JQuaternion orientation, in JVector position, in JVector direction,
    SweepCastFilterPre? pre, SweepCastFilterPost? post,
    out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out float lambda)
    where T : ISupportMappable

// 有界：只考虑 lambda ≤ maxLambda 的命中
public bool SweepCast<T>(in T support, in JQuaternion orientation, in JVector position, in JVector direction, float maxLambda,
    SweepCastFilterPre? pre, SweepCastFilterPost? post,
    out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out float lambda)
    where T : ISupportMappable
```

查询形状由一个 `ISupportMappable` 及其世界空间中的 `orientation` 和 `position` 定义。
`direction` 是扫掠方向向量；`lambda` 以 `direction` 为单位表示，因此查询形状在撞击时的中心位于 `position + lambda * direction`。

`pointA` 和 `pointB` 是扫掠原点处（尚未施加任何位移之前）的接触点。
由于扫掠投射的目标始终是静止的，`pointB` 就是世界空间中的接触点。
查询形状在撞击时的世界空间接触点为 `pointA + lambda * direction`。
`normal` 是世界空间中的碰撞法线。

如果找到命中则返回 `true`，包括形状已经重叠的情况（此时 `lambda` 为零，`normal` 为 `JVector.Zero`）。

**前置过滤器**（`SweepCastFilterPre`）在精确窄相测试之前调用，只接收候选代理。
可用它按标识廉价地跳过代理（例如忽略正在被扫掠的刚体）。
**后置过滤器**（`SweepCastFilterPost`）在精确测试之后调用，接收完整的 `SweepCastResult`（包括 `Lambda`、`Normal`、`PointA`、`PointB`）。
可用它根据几何形状拒绝命中，或收集多个结果而不是在第一个结果处停止。
任一过滤器返回 `false` 都会跳过该候选，而不会终止扫掠。

对于常见的内置查询形状，提供了便捷重载：

```cs
public bool SweepCastSphere(float radius, in JVector position, in JVector direction, ...)
public bool SweepCastBox(in JVector halfExtents, in JQuaternion orientation, in JVector position, in JVector direction, ...)
public bool SweepCastCapsule(float radius, float halfLength, in JQuaternion orientation, in JVector position, in JVector direction, ...)
public bool SweepCastCylinder(float radius, float halfHeight, in JQuaternion orientation, in JVector position, in JVector direction, ...)
```

每个都有一个带 `maxLambda` 参数的有界变体。

#### 查找最近查询

所有实现 `IDistanceTestable` 的代理都可以进行距离查询，包括所有形状：

```cs
// 无界：考虑树中所有 IDistanceTestable 代理
public bool FindNearest<T>(in T support, in JQuaternion orientation, in JVector position,
    FindNearestFilterPre? pre, FindNearestFilterPost? post,
    out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out float distance)
    where T : ISupportMappable

// 有界：只考虑距离小于 maxDistance 的代理
public bool FindNearest<T>(in T support, in JQuaternion orientation, in JVector position, float maxDistance,
    FindNearestFilterPre? pre, FindNearestFilterPost? post,
    out IDynamicTreeProxy? proxy, out JVector pointA, out JVector pointB, out JVector normal, out float distance)
    where T : ISupportMappable
```

查询形状由一个 `ISupportMappable` 及其世界空间中的 `orientation` 和 `position` 定义。
`pointA` 和 `pointB` 分别是查询形状上和找到的代理上的最近点。
`normal` 是从查询形状指向所找到代理的单位方向。

如果找到代理则返回 `true`，包括查询形状已经与某个代理重叠的情况（此时 `distance` 为零，`normal` 为 `JVector.Zero`）。
仅当范围内没有找到任何代理时返回 `false`。

**前置过滤器**（`FindNearestFilterPre`）在精确窄相测试之前调用，只接收候选代理。
**后置过滤器**（`FindNearestFilterPost`）在精确测试之后调用，接收完整的 `FindNearestResult`。
对于重叠结果，后置过滤器收到的 `distance = 0` 且 `normal = Zero`。
任一过滤器返回 `false` 都会跳过该候选并继续搜索。

若要跳过重叠的代理并寻找最近的*分离*代理，可在后置过滤器中拒绝重叠结果：

```cs
world.DynamicTree.FindNearestSphere(radius, position,
    pre: null,
    post: result => result.Distance > 0,
    out proxy, out pointA, out pointB, out normal, out distance);
```

对于常见的内置查询形状，提供了便捷重载：

```cs
public bool FindNearestPoint(in JVector position, ...)
public bool FindNearestSphere(float radius, in JVector position, ...)
```

每个都有一个带 `maxDistance` 参数的有界变体。

### 重叠检查示例

重叠检查可以这样组合：先用包围盒查询树，然后对返回的候选执行精确的窄相重叠测试：

```cs
// 返回所有与位于 'center'、半径为 'radius' 的球体重叠的 RigidBodyShape。
static IEnumerable<RigidBodyShape> OverlapSphere(World world, JVector center, float radius)
{
    var queryBox = new JBoundingBox(
        center - new JVector(radius),
        center + new JVector(radius));

    var candidates = new List<IDynamicTreeProxy>();
    world.DynamicTree.Query(candidates, queryBox);

    var querySphere = SupportPrimitives.CreateSphere(radius);

    foreach (var proxy in candidates)
    {
        if (proxy is not RigidBodyShape shape) continue;

        ref var data = ref shape.RigidBody.Data;

        bool hit = NarrowPhase.Overlap(
            querySphere,
            shape,
            JQuaternion.Identity,
            data.Orientation,
            center,
            data.Position);

        if (hit) yield return shape;
    }
}
```

这种模式通常足以完成重叠检查，并可避免引入额外的宽相 API。
