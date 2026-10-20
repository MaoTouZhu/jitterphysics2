# 你好，世界

### 创建 `PointCloudShape`

现在我们可以根据采样得到的顶点创建 `PointCloudShape`：

```cs
// 在茶壶凸包上找出若干点。
var vertices = ShapeHelper.SampleHull(allVertices, subdivisions: 3);

// 用这些点创建 PointCloudShape。
var pointCloudShape = new PointCloudShape(vertices);
```

不过，这里我们需要稍微小心一点。
如果直接把这个形状添加到刚体上，刚体的行为可能不符合直觉。
这是因为刚体的质心始终位于其局部坐标系的 `(0, 0, 0)` 处。

如果你在模型编辑器中打开 `teapot.obj`，会注意到模型并不是以原点为中心的。
要修正这一点，我们可以在模型编辑器中手动将模型居中——或者更方便地，使用 `PointCloudShape` 的 `Shift` 属性让质心与原点对齐：

```cs
// 在茶壶凸包上找出若干点。
var vertices = ShapeHelper.SampleHull(allVertices, subdivisions: 3);

// 用这些点创建 PointCloudShape。
var pointCloudShape = new PointCloudShape(vertices);

// 平移形状，使其质心位于原点。
pointCloudShape.GetCenter(out JVector centerOfMass);
pointCloudShape.Shift = -centerOfMass;

// pointCloudShape.GetCenter(out centerOfMass); // 现在返回 (0, 0, 0)

// 最后，为茶壶创建刚体
var rigidBody = world.CreateRigidBody();
rigidBody.AddShape(pointCloudShape);
```

> [!WARNING]
> 渲染模型时也必须考虑应用到形状上的平移，以确保模型在视觉上与模拟保持一致。

#### 创建同一形状的多个实例

在 Jitter 中，不能把同一个形状实例添加到多个刚体上。
要创建形状的额外实例，请使用 `PointCloudShape` 的 `Clone()` 方法。

该方法会创建一个新的形状对象，它共享底层数据结构，从而节省内存和计算时间：

```cs
var shapeInstance1 = new PointCloudShape(vertices);
var shapeInstance2 = shapeInstance1.Clone(); // 可以安全地用于第二个刚体
```

当许多刚体共享同一几何体时（例如模拟中多个相同的道具或角色），这种做法尤其有用。

### 整合起来


```cs
using System.Numerics;
using Raylib_cs;
using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;
using static Raylib_cs.Raylib;

static Matrix4x4 GetRayLibTransformMatrix(RigidBody body)
{
    JMatrix ori = JMatrix.CreateFromQuaternion(body.Orientation);
    JVector pos = body.Position;

    return new Matrix4x4(ori.M11, ori.M12, ori.M13, pos.X,
                         ori.M21, ori.M22, ori.M23, pos.Y,
                         ori.M31, ori.M32, ori.M33, pos.Z,
                         0, 0, 0, 1.0f);
}

static Texture2D GenCheckedTexture(int size, int checks, Color colorA, Color colorB)
{
    Image imageMag = GenImageChecked(size, size, checks, checks, colorA, colorB);
    Texture2D textureMag = LoadTextureFromImage(imageMag);
    UnloadImage(imageMag);
    return textureMag;
}

const int numberOfTeapots = 12;

// 设置抗锯齿提示
SetConfigFlags(ConfigFlags.Msaa4xHint);

// 初始化一个 1200x800 像素的窗口并设置标题
InitWindow(1200, 800, "TeaDrop example");

// 动态创建一个平面模型
Texture2D texture = GenCheckedTexture(10, 1,  Color.LightGray, Color.Gray);
Model planeModel = LoadModelFromMesh(GenMeshPlane(20, 20, 10, 10));
SetMaterialTexture(ref planeModel, 0, MaterialMapIndex.Diffuse, ref texture);

// 从文件加载茶壶模型
Model teapotModel = LoadModel("teapot.obj");

// 加载网格顶点
if (teapotModel.MeshCount == 0)
    throw new Exception("Model could not be loaded!");

Mesh teapotMesh;
unsafe { teapotMesh = teapotModel.Meshes[0]; }
var allVertices = teapotMesh.VerticesAs<JVector>();

// 在凸包上采样顶点
var vertices = ShapeHelper.SampleHull(allVertices, 4);

// 用精简后的顶点创建 PointCloudShape
var pointCloudShape = new PointCloudShape(vertices);

// 平移形状，使质心位于原点
pointCloudShape.GetCenter(out JVector center);
pointCloudShape.Shift = -center;

// 这里需要转置，因为 Raylib 和 System.Numerics
// 使用了不同的约定
Matrix4x4 shift = Matrix4x4.CreateTranslation(-center);
shift = Matrix4x4.Transpose(shift);

texture = GenCheckedTexture(16, 2,  Color.White, Color.Magenta);
Material teapotMat = LoadMaterialDefault();
SetMaterialTexture(ref teapotMat, MaterialMapIndex.Diffuse, texture);

// 初始化 Jitter 物理世界
World world = new ();
world.SubstepCount = 4;

// 添加一个表示平面的刚体
RigidBody planeBody = world.CreateRigidBody();
planeBody.AddShape(new BoxShape(20));
planeBody.Position = new JVector(0, -10, 0);
planeBody.MotionType = MotionType.Static;

// 添加 numberOfTeapots 个茶壶
for(int i = 0; i < numberOfTeapots; i++)
{
    RigidBody body = world.CreateRigidBody();
    body.AddShape(pointCloudShape.Clone());
    body.Position = new JVector(0, i * 4 + 0.5f, 0);
}

// 创建相机
Camera3D camera = new ()
{
    Position = new Vector3(-40.0f, 16.0f, 20.0f),
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

    world.Step(1.0f / 100.0f, true);

    foreach(var body in world.RigidBodies)
    {
        if (body == planeBody || body == world.NullBody) continue; // 不要绘制这个
        DrawMesh(teapotMesh, teapotMat, GetRayLibTransformMatrix(body) * shift);
    }

    EndMode3D();
    DrawText($"{GetFPS()} fps", 10, 10, 20, Color.White);

    EndDrawing();
}

CloseWindow();
```

![平面](images/teapots.gif)
