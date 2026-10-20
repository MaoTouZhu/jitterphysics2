# 凸包采样

茶壶是一个凹形状，我们将用它的凸包来近似（[Wikipedia](https://en.wikipedia.org/wiki/Convex_hull)）。
第一步，我们将减少用于构造凸包的顶点数量。

> [!NOTE]
> **采样**
>
> 我们当然可以使用完整的顶点集合，但这会带来两个主要缺点：
>
> 1. 集合中可能包含位于凸包内部的顶点。虽然它们不影响模拟结果，但在每一帧的碰撞检测中仍会被短暂考虑。
>
> 2. 我们通常不需要精确的凸包。省略少量顶点往往对精度影响很小，却能显著提升性能。
>
> 减少顶点数量有助于在不牺牲碰撞保真度的前提下加快模拟速度。

Jitter 提供了内置方法来简化顶点集合：`ShapeHelper.SampleHull`。
该函数在单位球面上均匀采样方向，并返回这些方向上最远的顶点。根据定义，这些顶点位于原始形状的凸包上。

将 `Program.cs` 的内容替换为以下代码：

```cs
using System.Numerics;
using Jitter2.Collision.Shapes;
using Jitter2.LinearMath;
using Raylib_cs;
using static Raylib_cs.Raylib;

// 设置抗锯齿提示
SetConfigFlags(ConfigFlags.Msaa4xHint);

// 初始化一个 1200x800 像素的窗口并设置标题
InitWindow(1200, 800, "TeaDrop example");

// 从文件加载茶壶模型
Model teapotModel = LoadModel("teapot.obj");

// 加载网格顶点
if (teapotModel.MeshCount == 0)
    throw new Exception("Model could not be loaded!");

Mesh mesh;
unsafe { mesh = teapotModel.Meshes[0]; }

var allVertices = mesh.VerticesAs<JVector>();
var vertices = ShapeHelper.SampleHull(allVertices, 4);

// 仅用于可视化——
// 模拟中并不需要构造显式的凸包！
var debugHull = ShapeHelper.Tessellate(vertices, 4);

// 创建相机
Camera3D camera = new ()
{
    Position = new Vector3(-5.0f, 2.0f, 3.0f),
    Target = new Vector3(0.0f, 1.0f, 0.0f),
    Up = new Vector3(0.0f, 1.0f, 0.0f),
    FovY = 45.0f,
    Projection = CameraProjection.Perspective
};

// 目标 100 fps
SetTargetFPS(100);

// 简单的渲染循环
while (!WindowShouldClose())
{
    BeginDrawing();
    ClearBackground(Color.Blue);

    // 让场景旋转
    UpdateCamera(ref camera, CameraMode.Orbital);

    BeginMode3D(camera);

    // 绘制茶壶模型
    DrawModelWires(teapotModel, Vector3.Zero, 1.0f, Color.White);

    // 绘制凸包的采样顶点
    foreach (var vertex in vertices)
    {
        DrawSphere(vertex, 0.02f, Color.Red);
    }

    // 用于调试：绘制显式凸包
    foreach (var vertex in debugHull)
    {
        DrawLine3D(vertex.V0, vertex.V1, Color.Green);
        DrawLine3D(vertex.V1, vertex.V2, Color.Green);
        DrawLine3D(vertex.V2, vertex.V0, Color.Green);
    }

    EndMode3D();
    DrawText($"{GetFPS()} fps", 10, 10, 20, Color.White);

    EndDrawing();
}

CloseWindow();
```

![平面](images/teapot.png)

茶壶的线框绘制为白色，绿色线框显示凸包，采样得到的凸包顶点绘制为红点。
