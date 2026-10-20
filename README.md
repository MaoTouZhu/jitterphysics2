#  <img src="./media/logo/jitterstringsmallsmall.png" alt="截图" width="240"/> Jitter Physics 2

[![GitHub Actions Workflow Status](https://img.shields.io/github/actions/workflow/status/notgiven688/jitterphysics2/jitter-tests.yml?label=JitterTests)](https://github.com/notgiven688/jitterphysics2/actions/workflows/jitter-tests.yml)
[![Nuget](https://img.shields.io/nuget/v/Jitter2?color=yellow)](https://www.nuget.org/packages/Jitter2/)
[![Discord](https://img.shields.io/discord/1213790465225138197?logo=discord&logoColor=lightgray&label=discord&color=blue)](https://discord.gg/7jr3f4edmV)

Jitter Physics 2 是一个用 C# 编写的快速、无依赖的物理引擎，支持刚体与软体动力学。它可运行于 .NET 支持的各平台，是 [Jitter Physics](https://github.com/notgiven688/jitterphysics) 的继任者。

📦 从 NuGet 安装 [Jitter2](https://www.nuget.org/packages/Jitter2)，或安装 [Jitter2.Double](https://www.nuget.org/packages/Jitter2.Double) 以获得双精度。参见[变更日志](https://jitterphysics.com/docs/changelog)。

▶️ 在 **[jitterphysics.com](https://jitterphysics.com/docs/introduction.html)** 体验交互式演示并浏览文档。

---

<img src="./media/screenshots/jitter_screenshot0.png" alt="截图" width="400"/> <img src="./media/screenshots/jitter_screenshot1.png" alt="截图" width="400"/>

<img src="./media/screenshots/jitter_screenshot2.png" alt="截图" width="400"/> <img src="./media/screenshots/jitter_screenshot4.png" alt="截图" width="400"/>

## 快速开始

将 Jitter 添加到你的 .NET 项目：

```sh
dotnet add package Jitter2
```

创建一个世界、添加一个盒子，并用固定时间步模拟一秒：

```csharp
using System;
using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.LinearMath;

using var world = new World();
world.Gravity = new JVector(0, -9.81f, 0);

var box = world.CreateRigidBody();
box.AddShape(new BoxShape(1, 1, 1));
box.Position = new JVector(0, 5, 0);

for (int i = 0; i < 60; i++)
{
    world.Step(1.0f / 60.0f);
}

Console.WriteLine(box.Position);
```

如需包含渲染的完整示例，请跟随[落箱教程](https://jitterphysics.com/docs/tutorials/boxes/project-setup.html)。

## 运行演示

`src` 目录包含四个项目：

| 项目             | 说明                                                       |
|------------------|------------------------------------------------------------|
| Jitter2          | 物理库。                                                   |
| JitterDemo       | 交互式 OpenGL 演示。                                       |
| JitterBenchmark  | BenchmarkDotNet 基准测试。                                 |
| JitterTests      | NUnit 测试。                                               |

运行演示场景：

安装 [.NET 10 SDK](https://dotnet.microsoft.com/download/dotnet/10.0)，然后运行：

```sh
git clone https://github.com/notgiven688/jitterphysics2.git
cd jitterphysics2/src/JitterDemo
dotnet run -c Release
```

JitterDemo 使用 [GLFW](https://www.glfw.org/) 访问 OpenGL 并管理窗口，使用 [VellumUI](https://www.nuget.org/packages/VellumUI/) 实现演示 UI 叠加层。该项目以预编译形式包含 GLFW 原生二进制文件。

## 特性

- 刚体与软体动力学。
- 单精度或双精度，在编译时选择。
- 可选的跨平台确定性求解器，用于可复现的模拟。
- 基于冲量的求解器，采用半隐式欧拉积分器与子步。
- 仅线性的推测接触，减少快速平移导致的隧穿。
- 约束与马达，柔软度可配置。
- 停用以降低非活动刚体的开销。
- 三角网格，支持内部边过滤。
- 使用 EPA 辅助的 MPR 与一次性接触流形的凸体碰撞检测。
- 复合形状与自定义支撑映射，以及内置的盒子、球体、胶囊体、圆柱体、圆锥体、凸包、点云和三角形。

## 文档

浏览[文档与交互式演示](https://jitterphysics.com/docs/introduction.html)，获取教程、API 细节和示例。

## 致谢

衷心感谢 Erin Catto、Dirk Gregorius、Erwin Coumans、Gino van den Bergen、Daniel Chappuis、Marijn Tamis、Danny Chapman、Gary Snethen 和 Christer Ericson 通过论坛帖子、演讲、代码、论文和书籍分享知识。

还要特别感谢前身项目 JigLibX 与 Jitter 的贡献者。

## 参与贡献 👋

欢迎各种形式的贡献！请随意 fork 本项目并创建拉取请求。
