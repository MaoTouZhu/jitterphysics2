# Jitter 世界

`World` 类包含物理模拟中的所有实体，并提供 `World.Step` 方法，用于将模拟推进单个时间步。

## World.Step

使用以下方法将世界推进单个时间步：

```cs
Step(float dt, bool multiThread = true)
```

### 时间步长

> [!NOTE]
> **Jitter2 中的单位**
> 单位制并未显式定义。
> 引擎针对尺寸为 1 [len_unit] 的物体进行了优化。
> 例如，碰撞系统使用的长度阈值约为 1e-04 [len_unit]。
> 它假定形状的质量属性采用 1 [mass_unit/len_unit³] 的单位密度。
> 因此，单位立方体的默认质量为 1 [mass_unit]。
> 重力默认值为 9.81 [len_unit/time_unit²]，这与公制单位（m/s²）下地球的重力加速度一致。
> 因此，在理解这些数值时使用公制单位（kg、m、s）是合理的。

时间步长越小，模拟越稳定。
不建议使用大于 $\mathrm{dt}=1/60\,\mathrm{s}$ 的时间步。
同时建议使用固定时间步。
典型的代码会累加时间增量，仅在固定的时间间隔调用 `world.Step`，如下例所示。

```cs
private float accumulatedTime = 0.0f;

public void FixedTimeStep(float dt, int maxSteps = 4)
{
    const float fixedStep = 1.0f / 100.0f;

    int steps = 0;
    accumulatedTime += dt;

    while (accumulatedTime >= fixedStep)
    {
        world.Step(fixedStep);
        accumulatedTime -= fixedStep;

        // 我们无法跟上真实时间，即模拟
        // 的运行速度慢于真实时间的流逝速度。
        if (++steps >= maxSteps) return;
    }
}
```

### 多线程

Jitter2 使用自有的线程池（`Parallelization.ThreadPool`）将任务分配到多个线程。
当 `world.Step` 以 `multiThread` 为 true 调用时，会使用该线程池。
默认情况下，会额外生成 `ThreadPool.ThreadCountSuggestion`$-1$ 个线程，其中建议值按如下方式计算：

```cs
public const float ThreadsPerProcessor = 0.9f;
public static int ThreadCountSuggestion => Math.Max((int)(Environment.ProcessorCount * ThreadsPerProcessor), 1);
```

线程池管理的工作线程数量可通过 `ChangeThreadCount(int numThreads)` 调整。
此处采用单例模式，如下所示：

```cs
ThreadPool.Instance.ChangeThreadCount(4);
```

这会将额外（相对于主线程）的工作线程数调整为 $4-1=3$。

`world.ThreadModel` 属性可用于在 `world.Step` 运行之后让线程池保持紧凑循环等待处理工作（`ThreadModelType.Persistent`），或在之后让出线程（`ThreadModelType.Regular`）。
推荐使用后者，以便将处理能力释放给其他代码，例如渲染。

#### 线程约定

每个 `World` 实例在任一时刻只有一个外部所有者。
不要从多个外部线程并发地对同一个世界调用 `Step`、`Stabilize`、`CreateRigidBody`、`CreateConstraint`、`Remove`、`Clear` 或其他会修改世界的 API。
同时应避免在一个线程步进或修改某个世界时，从另一个线程读取该世界的状态。

当每个世界都使用单线程步进时，可以从不同的宿主线程并发步进多个世界：

```cs
world.Step(dt, multiThread: false);
world.Stabilize(dt, iterations, multiThread: false);
```

当应用程序希望并行处理多个相互独立的世界时，这是推荐的配置。

> [!WARNING]
> `world.Step(dt, multiThread: true)` 和 `world.Stabilize(..., multiThread: true)` 使用 Jitter 的进程级工作线程池。
> 如果多个世界同时调用这些方法，Jitter 会通过进程级锁将这些调用串行化：一个多线程步进运行，其他则等待。
> 若要并行步进多个独立世界，请在自己的宿主线程上运行这些世界，并对每个世界调用 `world.Step(dt, multiThread: false)`。

由多个世界共享的回调和全局钩子必须是线程安全的。
当不同的世界被并发步进时，Jitter 可能在同一时刻从不同的外部线程调用共享回调。
这包括 `Logger.Listener`、宽相和窄相过滤器、碰撞回调、步进回调以及调试绘制器。

## 求解器模式

Jitter2 通过 <xref:Jitter2.SolveMode> 提供两种求解器策略：

```cs
world.SolveMode = SolveMode.Regular;       // 默认
world.SolveMode = SolveMode.Deterministic; // 可复现，较慢
```

`SolveMode.Regular` 是默认模式，针对吞吐量进行了优化。它使求解器保持快速，但接触和约束被处理的精确顺序可能发生变化。

`SolveMode.Deterministic` 是尽力而为的跨平台确定性路径。它以稳定的顺序处理每个模拟岛，并对接触和约束使用确定性的平局判定。岛仍然可以分布到多个线程，因此在确定性模式下 `world.Step(dt, multiThread: true)` 依然有效。

当可重复性比原始性能更重要时使用确定性模式，例如用于测试、回放、调试或确定性游戏模拟。它可能比 `SolveMode.Regular` 慢得多，因此通常应有意识地启用，而不是将其作为所有世界的默认设置。

相比之下，当在同一个 .NET 进程中以完全相同的方式构建世界时，`SolveMode.Regular` 配合 `multiThread: false` 看起来仍然可复现，但这一性质要弱得多，不应与跨平台确定性混为一谈。

对于确定性模式，并不要求整个世界都通过相同的历史构建。必须一致的是每个相互作用的岛内部的设置顺序：参与其中的刚体、形状和约束必须以相同顺序添加。精度仍然重要：float 构建和 double 构建都是确定性的，但它们不会产生相同的位模式。有关每种求解器配置所保证的内容以及该声明背后当前的 CI 覆盖情况，请参阅[通用](general.md#deterministic-simulation)。

## 求解器迭代次数

Jitter2 使用迭代求解器来求解接触和约束。
可以提高迭代次数以改善模拟质量（`world.SolverIterations`）。

```cs
world.SolverIterations = (solver: 6, relaxation: 4);
```

Jitter2 在速度层面求解物理接触（以及约束）（“求解器迭代”）。
Jitter2 还会向刚体添加速度，以解决刚体之间不真实的相互穿透。
这些额外的速度会给系统引入不需要的能量，可以通过在根据这些速度积分出新位置之后执行额外的松弛阶段来去除。
松弛阶段的迭代次数（“松弛迭代”）也在此处指定。
求解接触和约束的运行时间随迭代次数线性增长。

## 子步数

时间步可以被划分为更小的步，由 `world.SubstepCount` 定义。
这些更小的时间步的求解方式与常规完整步类似；但碰撞信息不会更新。
每个子步都使用 `world.SolverIterations` 中指定的求解器迭代次数进行求解。
例如

```cs
world.SubstepCount = 4;
world.SolverIterations = (solver: 2, relaxation: 1);
```

对每次 `world.Step` 调用共执行 $12$ 次求解器迭代。
其运行时间比执行 $12$ 次迭代的单个常规步更慢，但这种方法能增强模拟的稳定性。
子步非常适合提升约束的整体质量、稳定大型物体堆叠，以及更精确地模拟大质量比（例如重物压在轻物上）。

## 接触流形持久化

默认情况下，Jitter2 会在帧之间缓存接触点及其累积冲量（`world.PersistentContactManifold = true`）。这允许求解器从上一个解热启动，并让流形在多个步中逐渐增长，从而改善静止接触的稳定性。

设置 `world.PersistentContactManifold = false` 会在每帧结束时丢弃所有接触数据，因此每个接触都被视为全新的。这会移除帧间接触记忆，代价是求解器收敛速度下降。

单个刚体可以在不改变全局设置的情况下丢弃其缓存的接触：

```cs
body.ClearContactCache();  // 丢弃该刚体缓存的流形
```

这在传送等不连续的用户驱动变换之后很有用。当 `SolveMode.Deterministic` 处于活动状态时，设置刚体的 `Position` 或 `Orientation` 属性会自动调用 `ClearContactCache()`。

## 热启动重置

迭代求解器每帧都会从上一帧的累积冲量进行热启动。在恢复快照或进行任何其他不连续的状态变更之后，这份缓存状态可能已经过时。每个约束都公开 `ResetWarmStart()` 方法，它会在不移除约束的情况下清除其累积冲量：

```cs
foreach (var constraint in body.Constraints)
{
    constraint.ResetWarmStart();
}
```

随后可以调用 `World.Stabilize`，在恢复常规模拟之前重新求解恢复后的接触和约束。`Stabilize` 在设置了 `SolveMode.Deterministic` 时会遵循该设置。

## 辅助接触

Jitter2 采用一种称为“辅助接触”的技术，针对两个形状的平坦表面相互接触这一常见情况生成额外的接触。
这些额外的接触在一帧内计算完成，以“单次遍历”生成完整的接触流形，从而避免增量式构建碰撞流形时常见的抖动。
`world.EnableAuxiliaryContactPoints` 属性可用于启用或禁用辅助接触点生成。

## 刚体

所有注册到世界中的刚体都可以通过以下方式访问：

```cs
world.RigidBodies
```

其中 `RigidBodies` 的类型为 `ReadOnlyPartitionedSet<RigidBody>`。
刚体没有特定顺序，并且在调用 `world.Step` 期间可能被重新排序。

## 原始数据

`RigidBody`、`Arbiter` 和 `(Small)Constraint` 是驻留在托管堆上的常规 C# 类。
不过，这些对象与其非托管对应物相关联：`RigidBodyData`、`ContactData` 和 `(Small)ConstraintData`，可通过以下方式访问：

```cs
world.RawData
```

Jitter2 会重新安置原生结构，使活动对象存储在连续内存中，从而便于迭代求解器高效访问。

> [!CAUTION]
> **原始内存访问**
> 在使用 Jitter2 的标准功能时，通常不需要访问原始内存。
> 尽管读取对象的原始数据通常是安全的，但修改数据可能破坏引擎的内部状态。

> [!WARNING]
> **访问已移除的实体**
> `RigidBody`、`Arbiter` 和 `Constraint` 的实例会将部分数据存储在非托管内存中，这些内存在实体从世界中移除（`world.Remove`）后会自动释放。
> 在访问保留的 `RigidBody` 或 `Constraint` 引用之前，请检查其 `IsValid` 属性。
> 如果 `IsValid` 为 `false`，其非托管数据已不可用，不得再访问由数据支撑的属性与方法。
> 仲裁器引用在其碰撞事件生命周期结束后不得再使用。

## 移除实体

刚体、约束和形状都可以从世界中移除：

```cs
world.Remove(body);        // 移除刚体及其形状
world.Remove(constraint);  // 移除一个约束
world.Clear();             // 移除所有实体
```

移除刚体也会移除其形状、接触以及附加的约束。因此，即使没有直接调用 `world.Remove(constraint)`，约束也可能失效。
当相连的两个刚体都不再是动态时，改变刚体的运动类型也可能移除约束。
在此类操作之间保留约束引用时，请使用 `constraint.IsValid`。详情请参阅[约束生命周期](constraints.md#constraint-lifetime)。

## NullBody

世界提供了一个特殊的静态刚体 `world.NullBody`，它固定在世界中。
可用于创建将刚体固定于世界空间的约束（参见[约束](constraints.md)）。

## 停用

停用系统可以通过 `world.AllowDeactivation` 全局禁用。
将其设置为 `false` 可阻止刚体被停用，但不会唤醒已经休眠的刚体。

Jitter2 停用的是完整的模拟岛，而非孤立地停用单个刚体。只要岛中至少有一个非静态刚体在运动、被显式唤醒，或与新的活动接触或约束数据相连，该岛就保持活动。一旦岛中所有非静态刚体都在足够长的时间内保持低于各自的停用阈值，该岛就会被移入非活动分区。

非活动岛的运行时代价极小：其刚体、接触、约束和宽相代理都会被活动模拟跳过。当用户代码唤醒某个刚体，或某个活动刚体与休眠岛产生接触时，这些岛可以再次被激活。如果活动岛与非活动岛通过接触或约束连通，Jitter2 会将合并后的岛标记为需要更新，并在求解前将其非静态刚体移回活动分区。

静态刚体是特殊情况。它们通常处于非活动状态，不构成常规的模拟岛连接，也不会自行维持岛的唤醒状态。移动静态刚体仍可唤醒受影响的非静态刚体，以便在下一步更新接触。
