# 仲裁器

仲裁器管理两个刚体之间的接触信息。
当两个形状开始重叠时会创建一个仲裁器，当它们分离或其中一个相关刚体从世界中移除时，该仲裁器会被移除。
每个仲裁器最多可保存四个缓存的接触点（见 `ContactData`）。

## 仲裁器与形状

仲裁器是在*形状*之间创建的，而不是在刚体之间。
由于一个刚体可以附加多个形状，一对刚体可能产生多个仲裁器——每一对重叠形状对应一个。

例如，考虑两个刚体，刚体 A 有两个盒体形状，刚体 B 有一个球体形状。
如果两个盒体都与球体重叠，则会创建两个独立的仲裁器：每个盒体-球体对对应一个。

> [!NOTE]
> **每个形状对一个仲裁器**
> 即使接触流形随时间变化，只要两个形状保持邻近，仲裁器就保持不变。
> 仲裁器内的接触点会被缓存，并跨帧更新。

## ArbiterKey

每个仲裁器由一个 `ArbiterKey` 标识，它是一个由两个 `ulong` 标识符组成的有序对。
对于引擎创建的仲裁器，这些标识符是所涉及两个形状的 `ShapeId` 值。
顺序是规范的：ID 较小的形状始终排在前面。

```cs
public readonly struct ArbiterKey(ulong key1, ulong key2) : IEquatable<ArbiterKey>
{
    public readonly ulong Key1 = key1;
    public readonly ulong Key2 = key2;
}
```

`Key1` 和 `Key2` 的顺序对相等性比较有影响。

## 访问接触数据

`Arbiter` 类公开一个类型为 `JHandle<ContactData>` 的 `Handle` 属性。
通过 `arbiter.Handle.Data`，可以读取和修改底层接触信息：

```cs
ref ContactData data = ref arbiter.Handle.Data;
```

`ContactData` 结构最多包含四个接触槽位。
`UsageMask` 位域指示哪些槽位处于活动状态。
它还存储接触对的合成 `Friction` 和 `Restitution` 系数（取自两个刚体值中的最大值）。

> [!CAUTION]
> **生命周期**
> 接触数据仅在仲裁器注册于世界期间有效。
> 不要缓存对 `ContactData` 的引用。
> 在仲裁器被移除后访问它会导致未定义行为。

## 碰撞事件

`RigidBody` 类提供两个事件，用于提供关联的仲裁器：

```cs
body.BeginCollide += (Arbiter arb) =>
{
    // 当创建涉及此刚体的新仲裁器时调用。
};

body.EndCollide += (Arbiter arb) =>
{
    // 当涉及此刚体的仲裁器被移除时调用。
};
```

这些事件可用于检查或修改接触属性。
例如，覆盖特定接触的摩擦：

```cs
body.BeginCollide += (Arbiter arb) =>
{
    arb.Handle.Data.Friction = 0.0f;
};
```

由于仲裁器是按形状对划分的，具有多个形状的刚体可能会针对同一个对方刚体多次触发 `BeginCollide`——每个形状对触发一次。

## 自定义仲裁器

仲裁器并不局限于引擎内置的碰撞检测。
可以使用 `world.GetOrCreateArbiter` 和 `world.RegisterContact` 创建和管理自定义仲裁器。

### 创建仲裁器

`GetOrCreateArbiter` 接受两个 `ulong` 标识符和两个刚体。
如果给定 ID 对的仲裁器已存在，则返回它；否则会创建一个新的：

```cs
world.GetOrCreateArbiter(id0, id1, body1, body2, out Arbiter arbiter);
```

标识符不必是形状 ID——它们可以是任何唯一标识该接触对的 `ulong` 值。
这允许自定义碰撞系统为特定于应用的用途创建仲裁器。

> [!WARNING]
> **ID 顺序**
> 两个标识符的顺序有影响。
> `ArbiterKey(1, 2)` 和 `ArbiterKey(2, 1)` 被视为不同的键。
> 创建自定义仲裁器时，请保持顺序一致。

### 注册接触

一旦仲裁器存在，就可以向其中注册接触：

```cs
world.RegisterContact(arbiter, point1, point2, normal);
```

所有向量都必须位于世界空间，且法向必须是单位向量。
`removeFlags` 参数可用于将某些速度分量从求解器中排除（例如，`ContactData.SolveMode.Angular` 表示仅角响应或仅线性响应）。

或者，可以直接使用 ID 注册接触，如有需要也会创建仲裁器：

```cs
world.RegisterContact(id0, id1, body1, body2, point1, point2, normal);
```

### 接触缓存

一个仲裁器最多保存四个接触点。
当注册新接触时，仲裁器决定如何纳入它：

1. **有空槽位**：如果活动接触少于四个，则将新点与现有接触进行比较。如果它与某个现有接触足够接近（在断裂阈值内），则替换该接触并继承其累积冲量以进行热启动。否则，它作为新接触插入空槽位。
2. **所有槽位已满**：当四个槽位都被占用时，算法会评估应替换哪个现有接触。对于四个候选中的每一个，它计算由新点和其余三个接触构成的四边形的面积。移除后使该面积最大的候选会被替换。这种启发式方法使接触在接触区域上分布良好，从而产生最稳定的接触流形。

在位置积分过程中偏离超过距离阈值的接触会自动从流形中移除。

## 仲裁器生命周期

1. **创建**：当两个形状开始重叠时，引擎以形状 ID 为键调用 `GetOrCreateArbiter`。两个刚体上都会引发 `BeginCollide` 事件。
2. **更新**：每次 `world.Step` 时更新接触位置。分离超过阈值的接触会从仲裁器中移除。持续碰撞产生的新接触会被添加。
3. **移除**：当仲裁器中的所有接触都断开且形状不再重叠时，仲裁器被移除。两个刚体上都会引发 `EndCollide` 事件。仲裁器被返回内部池以供复用。
