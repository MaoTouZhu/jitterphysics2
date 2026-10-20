# 场景对比基准测试

构建两个 Jitter 版本，以无头方式模拟相同的场景，并生成一份 PDF，其中包含概览、来源信息、每个场景两页内容，以及最后的条形评分卡。Python 负责编排实验并生成报告；一个小型 C# 可执行文件调用实际的引擎。整个过程不涉及渲染器或 BenchmarkDotNet 依赖。

## 运行

需要 Git、.NET 10 SDK，以及 Python 3.12 或更高版本。在仓库根目录下执行：

```sh
python3 -m venv other/SceneBenchmark/.venv
other/SceneBenchmark/.venv/bin/python -m pip install -r other/SceneBenchmark/requirements.txt
other/SceneBenchmark/.venv/bin/python other/SceneBenchmark/run.py compare 2.9.0 local
```

在 Windows 上，请使用 `.venv\Scripts\python.exe`。引用可以是标签、提交哈希，或 `HEAD^` 之类的 Git 提交表达式。`local` 包含当前库中已跟踪的修改和未跟踪的源文件。两个引用都可以是 Git 修订，任意一侧都可以是 `local`。修订必须存在于本地仓库中。

默认值：两个场景、每个场景模拟 20 秒、时间步 0.01 秒、三次全新进程重复、共一个线程、两秒预热、`(8, 4)` 求解器/松弛迭代、一个子步，以及单精度。所有场景中都禁用休眠。下面是一个同时包含单线程与并行结果的示例：

```sh
other/SceneBenchmark/.venv/bin/python other/SceneBenchmark/run.py compare 6520fddb local \
  --threads 1 16 --repetitions 3 --output other/SceneBenchmark/results/tag-vs-local
```

`--threads` 统计调用线程。多个配置会在同一份 PDF 中分别生成场景页和最终评分卡。请在空闲主机上运行。默认的单线程完整实验可能需要数分钟；模拟时长并不意味着实时执行。如需快速集成检查：

```sh
other/SceneBenchmark/.venv/bin/python other/SceneBenchmark/run.py compare HEAD local \
  --seconds 0.1 --warmup 0.1 --repetitions 1 --threads 1 16 \
  --output other/SceneBenchmark/results/quick-check
```

其他选项：`--scenes colosseum ragdolls`、`--dt`、`--warmup`、`--iterations`、`--relaxations`、`--substeps` 和 `--trial-timeout`（每个进程的实际耗时秒数，默认 1,800）。时长必须是时间步的整数倍，且 `dt` 不能超过 1/60 秒。非空输出目录绝不会被复用。

## 场景

* **Colosseum：** Demo28 的六层结构，以及 21 个同心双层墙/平台环，位于同一个 200 单位的地面上。几何体是该演示的冻结副本，包含其原始的 BepuPhysics 署名和 Apache 2.0 声明。
* **带布娃娃的旋转立方体：** 500 个十刚体布娃娃，位于 50 单位的空心运动学立方体中，墙壁厚度一个单位，每步角速度为 `(0.14, 0.02, 0.03)`，按 10 x 5 x 10 的网格分布。每个布娃娃有 20 个约束，总计 10,000 个。形状、关节限位、柔软度以及八个相邻刚体碰撞排除均来自 `Common.BuildRagdoll`。每十步采样一次 RMS 和最大球窝锚点分离距离，包括铰链锚点（共 4,500 个锚点）。

两个库都针对完全相同的场景源代码编译。不会从所选修订中加载任何演示源代码，因此修改旧演示不会改变基准测试的输入。较旧的修订必须支持测试框架的公共 API（`World.DebugTimings`、动态树分析、`RawData`、运动类型和布娃娃约束）。不兼容的修订会生成构建日志并失败；运行器不会修补任何实现，也不会回退到其他引用。

## 测量与报告

每次试验都是一个全新的 .NET 进程。它会预热场景的一份丢弃副本，然后重建原始布局，并测量完整的 0-20 秒轨迹。仅用外部秒表对 `World.Step` 计时。场景搭建、容器速度赋值、预分配的观测数据、关节诊断以及序列化都在该秒表之外。托管分配计数器覆盖所有线程上的测量循环；诊断不会逐帧分配。

PDF 包含：

* 首页结果，指明更快的版本（A 或 B）、其加速比，以及平均步时间降低的百分比。结果相同时标记为平局。
* 平均步和 Solve 曲线随模拟时间的变化、窄相/宽相曲线，以及活跃接触流形数量，并带有重复范围的阴影。
* 所有命名的 `World.Timings` 和 `DynamicTree.Timings` 桶及其数值平均值，再加上 World 计时桶之外的耗时（步边界开销）。树计时嵌套在 BroadPhase 内部，不是额外开销。某个版本中缺失的桶标记为 `n/a`，而不是当作零处理。
* P95/P99 步延迟、进程均值范围、托管分配、GC 回收次数、引擎缓冲区内存，以及宽相候选数量。
* 引擎内存曲线或布娃娃关节质量曲线，以及有限状态检查。
* 最终的绝对平均步/Solve 条形图，带有重复范围误差棒和相对变化，以及整个区间累计的实际耗时步时间。

`Solve` 桶包含积分以及接触/约束求解；它并不单独区分关节迭代成本。活跃接触统计的是流形，而不是单个接触点。引擎内存是 `RawData` 暴露的非托管刚体、接触和约束缓冲区分配量，而不是进程总内存。原始试验还会保留进程峰值工作集（包括搭建和预热）以及最终的托管堆大小。

曲线使用 0.25 秒的分箱均值。每一帧都保留在原始 JSON 中。汇总统计让每个进程权重相同；P95/P99 按进程计算后再取平均。误差棒/阴影是观测到的范围，而不是置信区间。总体加速比是基线/候选平均步比值的几何平均，每个场景/线程配置权重相同。首页加速比将较快的版本与较慢的版本进行比较；JSON 中的 `geometricMeanSpeedup` 为兼容性保留基线/候选比值。

求解器可能改变轨迹、接触工作量和关节误差。PDF 比较的是由此得到的模拟结果，并不声称碰撞查询完全相同，也不声称相关帧具有统计显著性。重复实验在场景内交替 A/B 顺序、轮换场景顺序，并反转线程顺序。它们串行执行。两个构建以相同方式禁用分层编译和服务器 GC；所有生效的 `DOTNET_` 和 `COMPlus_` 环境变量都会被记录。

## 输出与来源

默认目标目录是 `other/SceneBenchmark/results/<timestamp>/`：

```text
comparison.pdf       # 唯一可分享的报告
summary.json         # 聚合指标
manifest.json        # 源/构建哈希、设置、机器、调度、状态
build-baseline.log
build-candidate.log
raw/*.json           # 帧计时、工作量、内存、诊断
raw/*.log            # 进程输出和失败详情
```

运行器只解析一次 Git 引用，并且只归档库源代码。`local` 复制时不包含 `bin/obj`。它在构建前对整个源快照做哈希，并记录构建出的程序集哈希、精确提交、本地库状态以及公共测试框架哈希。命令退出时会删除临时构建；检出状态绝不会被切换，也不会使用其构建输出。试验失败会将清单标记为失败并保留日志/数据；不会发布不完整的 PDF。

无需重新构建或模拟，即可从保存的数据重建报告：

```sh
other/SceneBenchmark/.venv/bin/python other/SceneBenchmark/run.py report \
  other/SceneBenchmark/results/tag-vs-local
```

添加 `--output /path/to/report.pdf` 可选择其他 PDF 路径。报告生成会校验已完成的试验、设置、帧数、有限观测值以及初始场景拓扑是否一致。从旧运行重新生成的报告仅包含 colosseum 和 ragdolls，并基于这些场景重新计算聚合结果。原始试验和清单会保留。JSON schema 版本当前为 1。

使用以下命令运行小型编排/统计回归检查：

```sh
other/SceneBenchmark/.venv/bin/python -m unittest discover -s other/SceneBenchmark -p 'test_*.py'
```
