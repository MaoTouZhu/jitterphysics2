# 碰撞过滤器

碰撞过滤器有三种类型：`world.DynamicTree.Filter`、`world.BroadPhaseFilter` 和 `world.NarrowPhaseFilter`。

## 动态树过滤器

`world.DynamicTree.Filter`

```cs
public Func<IDynamicTreeProxy, IDynamicTreeProxy, bool> Filter { get; set; }
```

是在 `world.Step` 期间应用的最早的过滤器，默认设置为 `World.DefaultDynamicTreeFilter`：

```cs
public static bool DefaultDynamicTreeFilter(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB)
{
    if (proxyA is RigidBodyShape rbsA && proxyB is RigidBodyShape rbsB)
    {
        return rbsA.RigidBody != rbsB.RigidBody;
    }

    return true;
}
```

该过滤器会过滤掉属于同一刚体的形状之间的碰撞。
动态树会忽略这些碰撞，因此不会创建任何潜在配对。

对于软体，通常会使用另一种碰撞过滤器（定义在 `SoftBodies.DynamicTreeCollisionFilter.Filter` 中），它还会过滤掉属于同一软体的形状之间的碰撞。

## 宽相过滤器

默认情况下 `world.BroadPhaseFilter`

```cs
public IBroadPhaseFilter? BroadPhaseFilter { get; set; }
```

为 `null`。它用于过滤掉通过了宽相碰撞检测的碰撞——也就是说，在 `DynamicTree` 已将碰撞添加到 `PotentialPair` 哈希集之后。

当向 `world.DynamicTree` 添加自定义碰撞代理时，这可能很有用。
由于世界只知道如何处理 `RigidBodyShape` 之间的碰撞，必须由过滤器来处理检测到的碰撞（实现自定义碰撞响应代码并过滤掉该碰撞），以免抛出 `InvalidCollisionTypeException`。
软体实现正是基于这类过滤器（参见 `SoftBodies.BroadPhaseCollisionFilter`）。

### 示例：碰撞组

碰撞组可以通过宽相过滤器实现。
在这个示例中有两个队伍：蓝队和红队。
下面是一个忽略不同颜色队伍成员之间所有碰撞的过滤器：

```cs
public class TeamFilter : IBroadPhaseFilter
{
    public class TeamMember { }
    
    public static TeamMember TeamRed = new();
    public static TeamMember TeamBlue = new();
    
    public bool Filter(IDynamicTreeProxy proxyA, IDynamicTreeProxy proxyB)
    {
        if (proxyA is not RigidBodyShape shapeA || proxyB is not RigidBodyShape shapeB)
        {
            return true;
        }

        if (shapeA.RigidBody.Tag is not TeamMember || shapeB.RigidBody.Tag is not TeamMember)
        {
            // 如果至少有一个刚体不属于任何队伍，则按常规处理碰撞
            return true;
        }

        // 红队与蓝队之间不发生碰撞。
        return shapeA.RigidBody.Tag == shapeB.RigidBody.Tag;
    }
}
```

然后可以实例化 `TeamFilter` 类并将其赋值给 `world.BroadPhaseFilter`：

```cs
world.BroadPhaseFilter = new TeamFilter();
...
bodyA.Tag = TeamFilter.TeamBlue;
bodyB.Tag = TeamFilter.TeamRed;
bodyC.Tag = TeamFilter.TeamRed;
```

## 窄相过滤器

`world.NarrowPhaseFilter`

```cs
public INarrowPhaseFilter? NarrowPhaseFilter { get; set; }
```

的运作方式类似。
不过，此回调在窄相碰撞检测之后调用，这意味着在此阶段可以获取详细的碰撞信息（如法线、穿透深度和碰撞点）。
过滤器可以排除碰撞，也可以修改碰撞信息。

默认的窄相碰撞过滤器赋值给 `TriangleEdgeCollisionFilter` 的一个实例，它会过滤掉 `TriangleShape` 上所谓的“内部边”。
当刚体在构成静态几何的相连三角形边缘上滑动时，这些内部边通常会导致碰撞伪影。
这个问题也被称为“幽灵碰撞”。

静态和运动学三角形刚体会接受边缘过滤。对于动态三角形刚体，
默认情况下会禁用边缘过滤，以避免使运动中的网格失稳。可通过 `FilterDynamicBodies` 显式启用：

```cs
world.NarrowPhaseFilter = new TriangleEdgeCollisionFilter { FilterDynamicBodies = true };
```
