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

const int NumberOfBoxes = 12;

// 设置抗锯齿提示
SetConfigFlags(ConfigFlags.Msaa4xHint);

// 初始化一个 1200x800 像素、带标题的窗口
InitWindow(1200, 800, "BoxDrop example");

// 动态创建平面模型
Texture2D texture = GenCheckedTexture(10, 1, Color.LightGray, Color.Gray);
Model planeModel = LoadModelFromMesh(GenMeshPlane(10, 10, 10, 10));
SetMaterialTexture(ref planeModel, 0, MaterialMapIndex.Diffuse, ref texture);

// 动态创建盒子模型
texture = GenCheckedTexture(2, 1, Color.White, Color.Magenta);
Mesh boxMesh = GenMeshCube(1, 1, 1);
Material boxMat = LoadMaterialDefault();
SetMaterialTexture(ref boxMat, MaterialMapIndex.Diffuse, texture);

// 初始化 Jitter 物理世界
World world = new();
world.SubstepCount = 4;

// 添加一个代表平面的刚体
RigidBody planeBody = world.CreateRigidBody();
planeBody.AddShape(new BoxShape(10));
planeBody.Position = new JVector(0, -5, 0);
planeBody.MotionType = MotionType.Static;

// 添加 NumberOfBoxes 个立方体
for (int i = 0; i < NumberOfBoxes; i++)
{
    RigidBody body = world.CreateRigidBody();
    body.AddShape(new BoxShape(1));
    body.Position = new JVector(0, i * 2 + 0.5f, 0);
}

// 创建相机
Camera3D camera = new()
{
    Position = new Vector3(-20.0f, 8.0f, 10.0f),
    Target = new Vector3(0.0f, 4.0f, 0.0f),
    Up = new Vector3(0.0f, 1.0f, 0.0f),
    FovY = 45.0f,
    Projection = CameraProjection.Perspective
};

// 目标帧率 100 fps
SetTargetFPS(100);

// 简单的渲染循环
while (!WindowShouldClose())
{
    BeginDrawing();
    ClearBackground(Color.Blue);

    BeginMode3D(camera);

    DrawModel(planeModel, Vector3.Zero, 1.0f, Color.White);

    world.Step(1.0f / 100.0f, true);

    foreach (var body in world.RigidBodies)
    {
        if (body == planeBody || body == world.NullBody) continue; // 不要绘制该刚体
        DrawMesh(boxMesh, boxMat, GetRayLibTransformMatrix(body));
    }

    EndMode3D();
    DrawText($"{GetFPS()} fps", 10, 10, 20, Color.White);

    EndDrawing();
}

CloseWindow();
