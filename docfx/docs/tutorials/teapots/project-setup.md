# 项目搭建

在上一节中，我们创建了一个下落的箱子的模拟。Jitter 内置了若干默认形状，例如胶囊体、圆柱体和球体。
这些形状可以被变换和/或组合，它们已足以表示许多类型的可碰撞实体。

在本节中，我们将向模拟中添加一个自定义凸形状——具体来说，就是著名的*犹他茶壶*。我们将从其视觉表示出发构造这个形状：加载 `teapot.obj` 文件，并使用其顶点创建凸形状。


### 环境要求

安装 [.NET 10 SDK](https://dotnet.microsoft.com/download/dotnet/10.0)。

执行以下命令，确认 dotnet 已正确配置：

```sh
dotnet --version
```

### 创建新的控制台应用并添加 Jitter 与 Raylib

首先，创建一个名为 "TeaDrop" 的新目录，并进入该目录：

```sh
mkdir TeaDrop && cd TeaDrop
```

下载并解压 [teapot.obj](https://github.com/notgiven688/jitterphysics2/raw/refs/heads/main/src/JitterDemo/assets/teapot.obj.zip) 模型。

```sh
wget https://github.com/notgiven688/jitterphysics2/raw/refs/heads/main/src/JitterDemo/assets/teapot.obj.zip
unzip teapot.obj.zip
```

接下来，在该目录中创建一个新的控制台应用，并添加 Raylib-cs 与 Jitter2：

```sh
dotnet new console
dotnet add package Raylib-cs --version 6.1.1
dotnet add package Jitter2
```

在 `TeaDrop.csproj` 中添加以下代码，以允许不安全代码，并将 teapot.obj 自动复制到输出目录：

```xml
  <PropertyGroup>
    <AllowUnsafeBlocks>true</AllowUnsafeBlocks>
  </PropertyGroup>

  <ItemGroup>
    <None Update="teapot.obj">
      <CopyToOutputDirectory>Always</CopyToOutputDirectory>
    </None>
  </ItemGroup>
```

至此，搭建已完成。如果你现在执行以下命令：

```sh
dotnet run
```

你的控制台应显示："Hello, World!"。
