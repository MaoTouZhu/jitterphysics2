# 概述

本节介绍基本的设计决策和配置选项。

## 碰撞检测理念

Jitter2 采用统一的碰撞检测方式，这与许多其他物理引擎不同。不同于为特定形状对（球-球、盒-盒、胶囊-胶囊等）实现专用算法的引擎，Jitter2 使用隐式形状统一处理所有碰撞检测。每个形状都通过支撑函数表示，碰撞通过 MPR（Minkowski Portal Refinement，闵可夫斯基门户细化）求解，深度穿透时回退到 EPA（Expanding Polytope Algorithm，扩展多面体算法）。这种设计简化了代码库，并使添加自定义形状变得简单直接——任何提供支撑映射的形状都能自动与所有其他形状配合工作。

传统物理引擎使用三阶段碰撞管线：宽相（空间划分）、中相（针对复杂网格的层次包围体）和窄相（精确形状相交）。Jitter2 完全消除了中相。Jitter2 不为复杂几何构建内部加速结构，而是依赖[碰撞过滤器](filters.md)来处理大规模环境。这使得用户自定义的空间划分策略成为可能——高度图、精细三角网格，甚至无限体素世界，都可以通过在过滤器回调中按需生成碰撞几何来实现。

## 精度

Jitter2 同时支持单精度（`float`）和双精度（`double`）浮点运算，在编译时选择。要以双精度构建，可以取消 [Precision.cs](https://github.com/notgiven688/jitterphysics2/blob/main/src/Jitter2/Precision.cs) 中的 `#define USE_DOUBLE_PRECISION` 注释，或使用命令行选项：

```bash
dotnet build -c Release -p:DoublePrecision=true
```

可以在运行时通过 `Precision.IsDoublePrecision` 检查当前生效的精度模式。在单精度下，`JVector.X` 是 `float`；在双精度下，它是 `double`。

## 构建配置与校验

对象创建和配置会校验那些会使引擎处于无效状态的值，例如形状尺寸、质量、约束轴和无效限位。这些检查在 Debug 和 Release 构建中都会抛出常规的 .NET 异常。

Debug 构建会在运行时状态修改路径上增加额外的健全性检查，例如非有限的位置、速度、力以及未归一化的朝向。这些检查在 `DEBUG` 符号下编译，并从 Release 构建中剔除。

NuGet 包是 Release 构建。从 NuGet 使用 Jitter2 时，只存在 Release 可见的契约检查。要获得额外仅 Debug 的健全性检查，请以 Debug 配置引用或从源码构建 Jitter2。

## 确定性模拟

Jitter2 通过 <xref:Jitter2.SolveMode> 提供可选的跨平台确定性求解器模式：

```cs
world.SolveMode = SolveMode.Deterministic;
```

此模式旨在使用相同 Jitter2 版本、精度模式和步进输入时，在不同平台上获得可复现的模拟。它适用于自动化测试、回放系统、调试和锁步式模拟。

重要的要求并不是整个世界完全一致。关键在于每个相互作用的岛都以相同的顺序组装：参与该岛的刚体、形状和约束必须以相同顺序添加。如果同一个岛以相同顺序创建并接收相同的输入，即使世界中不相关的部分以不同方式构建，它也会以相同方式演化。

确定性路径让接触和约束保持稳定顺序，并使用内部稳定的三角函数辅助方法（`StableMath`），因此关键数学运算不依赖平台特定的 `Math`/`MathF` 行为。

| 配置 | 保证的内容 | 必须一致的内容 | 不保证的内容 |
| --- | --- | --- | --- |
| `SolveMode.Deterministic` | 跨平台可复现的模拟结果，与线程模式和内部 SIMD/标量执行路径无关 | 相同的 Jitter2 版本、相同的精度模式、相同的步进序列/固定时间步、在每个相互作用的岛内添加参与刚体、形状和约束的顺序相同 | float 与 double 相互匹配、不同引擎版本之间的结果 |
| `SolveMode.Regular` 且 `multiThread: false` | 仅在狭义上可复现：完全相同的世界构建在同一 .NET 进程内可以再次产生相同结果 | 完全相同的构建路径、完全相同的添加顺序、相同的步进序列、相同的 .NET 进程/运行时运行 | 跨平台确定性、通过不同历史重建相同的最终场景、不同进程启动之间的结果匹配 |
| `SolveMode.Regular` 且 `multiThread: true` | 无确定性保证 | 无 | 可复现的顺序或跨平台位级一致性 |

> [!WARNING]
> 据我们目前所知，该功能在上述情形下是跨平台确定性的，但这一说法基于当前实现和测试覆盖。撰写本文时，CI 在 `ubuntu-latest`、`windows-latest` 和 `macos-latest` 上运行确定性哈希测试，这些目前分别对应 GitHub 托管运行器上的 x64 Linux、x64 Windows 和 arm64 macOS。

`SolveMode.Deterministic` 可能明显慢于 `SolveMode.Regular`，后者仍是默认且推荐的选项，适用于常规游戏玩法或交互式沙盒场景。

## 坐标系

Jitter2 使用右手坐标系。
引擎本身与坐标系无关，不强制任何特定的轴约定。

唯一假定特定朝向的默认值是 `World.Gravity`，它初始化为 `(0, -9.81, 0)`，将 Y 轴视为向上。
这可以更改：

```cs
world.Gravity = new JVector(0, 0, -9.81f);  // Z 轴向上的约定
```

## 追踪与日志

Jitter2 提供了用于日志记录和性能分析的机制。

### 日志

`Logger` 类提供了简单的日志记录接口，包含三个严重级别：`Information`、`Warning` 和 `Error`。
要接收日志消息，请注册监听器：

```cs
Logger.Listener = (level, message) =>
{
    Console.WriteLine($"[{level}] {message}");
};
```

引擎会为 EPA 收敛问题或内存分配回退等事件记录警告。

### 性能追踪

使用 `PROFILE` 符号编译时，Jitter2 会记录每个模拟阶段（宽相、窄相、求解器迭代等）的详细计时信息。
跟踪数据存储在每线程缓冲区中，以尽量减少开销。

要导出记录的数据：

```cs
Tracer.WriteToFile("trace.json");
```

输出文件使用 Chrome Trace Event 格式，可在以下位置可视化：

- `chrome://tracing`（粘贴到 Chrome 地址栏中）
- [Perfetto UI](https://ui.perfetto.dev/)

未定义 `PROFILE` 时，所有追踪调用都会通过 `[Conditional]` 特性被编译器完全剥离，因此运行时开销为零。

## 自定义数学类型

Jitter2 定义了自己的数学类型（`JVector`、`JMatrix`、`JSymmetricMatrix`、`JQuaternion`、`JBoundingBox`），而不是使用 `System.Numerics`。这使得精度可以在 `float` 和 `double` 之间全局切换而无需修改代码，通过 `[StructLayout(LayoutKind.Explicit)]` 显式控制内存布局，并避免依赖外部数学库的行为。

显式的字段偏移保证了可预测的内存布局，从而实现与其他库类型之间的零拷贝转换：

```cs
// 转换为任何布局兼容的类型
MyVector3 myVec = jitterVector.UnsafeAs<MyVector3>();

// 从任何布局兼容的类型转换
JVector jitterVector = JVector.UnsafeFrom(myVec);
```

为方便起见，还提供了与 `System.Numerics` 类型之间的隐式转换：

```cs
System.Numerics.Vector3 sysVec = jitterVector;  // 隐式转换
JVector jitterVec = sysVec;                      // 隐式转换
```

这些转换涉及复制，并且从 double 转换为 float 时可能丢失精度。

### 向量与矩阵约定

Jitter2 将向量视为**列向量**。
向量 $v$ 通过 $M \cdot v$（后乘）被矩阵 $M$ 变换：

```cs
JVector result = JVector.Transform(v, M);  // 计算 M * v
```

这是数学、物理和大多数图形 API（例如 OpenGL、Vulkan/GLSL）中使用的标准约定。
`System.Numerics` 使用相反的约定：向量是**行向量**，变换写作 $v \cdot M$（前乘）。

因此，在 Jitter2 与 `System.Numerics` 之间转换变换矩阵时，必须对矩阵进行转置。
Jitter2 的 `JMatrix` 在内存中还以列主序存储元素（即 `M11`、`M21`、`M31` 是连续的），而 `System.Numerics.Matrix4x4` 使用行主序存储。
