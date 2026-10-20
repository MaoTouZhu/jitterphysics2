# 项目搭建

本项目将使用 Raylib 和 Jitter 实现一个箱子掉落到地面的简单场景。

## 环境要求

安装 [.NET 10 SDK](https://dotnet.microsoft.com/download/dotnet/10.0)。

执行以下命令，确认 dotnet 已正确配置：

```sh
dotnet --version
```

## 创建新的控制台应用并添加 Jitter 与 Raylib

首先，创建一个名为“BoxDrop”的新目录并进入该目录：

```sh
mkdir BoxDrop && cd BoxDrop
```

接着，在该目录中创建一个新的控制台应用，并添加 Raylib-cs 和 Jitter2：

```sh
dotnet new console
dotnet add package Raylib-cs --version 6.1.1
dotnet add package Jitter2
```

至此环境搭建完成。如果现在执行以下命令：

```sh
dotnet run
```

控制台应输出：“Hello, World!”。
