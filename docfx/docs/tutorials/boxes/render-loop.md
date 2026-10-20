# 渲染循环

我们首先要做的是熟悉一下 Raylib_cs。将 `Program.cs` 的内容替换为以下代码：

```cs
using System.Numerics;
using Raylib_cs;
using static Raylib_cs.Raylib;

static Texture2D GenCheckedTexture(int size, int checks, Color colorA, Color colorB)
{
    Image imageMag = GenImageChecked(size, size, checks, checks, colorA, colorB);
    Texture2D textureMag = LoadTextureFromImage(imageMag);
    UnloadImage(imageMag);
    return textureMag;
}

// 设置抗锯齿提示
SetConfigFlags(ConfigFlags.Msaa4xHint);

// 初始化一个 1200x800 像素的窗口并设置标题
InitWindow(1200, 800, "BoxDrop example");

// 动态创建一个平面模型
Texture2D texture = GenCheckedTexture(10, 1,  Color.LightGray, Color.Gray);
Model planeModel = LoadModelFromMesh(GenMeshPlane(10, 10, 10, 10));
SetMaterialTexture(ref planeModel, 0, MaterialMapIndex.Diffuse, ref texture);

// 创建相机
Camera3D camera = new ()
{
    Position = new Vector3(-20.0f, 8.0f, 10.0f),
    Target = new Vector3(0.0f, 4.0f, 0.0f),
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

    BeginMode3D(camera);

    DrawModel(planeModel, Vector3.Zero, 1.0f, Color.White);

    EndMode3D();
    DrawText($"{GetFPS()} fps", 10, 10, 20, Color.White);

    EndDrawing();
}

CloseWindow();
```

运行程序后，应该会显示一个平面：

![平面](images/raylibplane.png)

下一章我们将添加一些经过物理模拟的盒子。
