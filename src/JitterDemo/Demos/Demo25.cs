using System;
using System.Collections.Generic;
using Jitter2;
using Jitter2.Collision;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;
using JitterDemo.Renderer;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo;

// 在本示例中，我们创建一个高度图并将其用于碰撞检测。
// 注意只有渲染代码需要内存，高度图本身可以程序化生成，
// 并且可以是无限大的。
// 为了在进行射线投射时节省资源，我们使用有限长度的射线（线段）。

public static class Heightmap
{
    public const int Width = 100;
    public const int Height = 100;
    public const float Amplitude = 2f;

    public static float GetHeight(int x, int z)
    {
        // GetHeight 也可以用数组实现，请确保遵守数组边界
        if (x < 0 || x >= Width || z < 0 || z >= Height)
            throw new ArgumentOutOfRangeException();

        return MathF.Sin(x * 0.1f) * MathF.Cos(z * 0.1f) * Amplitude;
    }

    public static JBoundingBox GetBoundingBox()
    {
        JVector min = new JVector(0, -Amplitude, 0);
        JVector max = new JVector(Width - 1, Amplitude, Height - 1);
        return new JBoundingBox(min, max);
    }
}

public class HeightmapTester(JBoundingBox box) : IDynamicTreeProxy, IRayCastable
{
    public int SetIndex { get; set; } = -1;
    public int NodePtr { get; set; }

    public JVector Velocity => JVector.Zero;
    public JBoundingBox WorldBoundingBox { get; } = box;

    public bool RayCast(in JVector origin, in JVector direction, out JVector normal, out float lambda)
    {
        const float maxDistance = 100.0f;

        float dirX = direction.X;
        float dirZ = direction.Z;

        float len2 = dirX * dirX + dirZ * dirZ;
        float ilen = 1.0f / MathF.Sqrt(len2);

        dirX *= ilen;
        dirZ *= ilen;

        int x = (int)Math.Floor(origin.X);
        int z = (int)Math.Floor(origin.Z);

        int stepX = dirX > 0 ? 1 : -1;
        int stepZ = dirZ > 0 ? 1 : -1;

        float nextX = dirX > 0 ? (x + 1) - origin.X : origin.X - x;
        float nextZ = dirZ > 0 ? (z + 1) - origin.Z : origin.Z - z;

        float tMaxX = dirX != 0 ? nextX / Math.Abs(dirX) : float.PositiveInfinity;
        float tMaxZ = dirZ != 0 ? nextZ / Math.Abs(dirZ) : float.PositiveInfinity;

        float tDeltaX = direction.X != 0 ? 1f / Math.Abs(dirX) : float.PositiveInfinity;
        float tDeltaZ = direction.Z != 0 ? 1f / Math.Abs(dirZ) : float.PositiveInfinity;

        float t = 0f;

        while (t <= maxDistance)
        {
            // 检查是否越界
            if (x < 0 || x + 1 >= Heightmap.Width || z < 0 || z + 1 >= Heightmap.Height)
                goto continue_walk;

            // 检查这个四边形！
            var a = new JVector(x + 0, Heightmap.GetHeight(x + 0, z + 0), z + 0);
            var b = new JVector(x + 1, Heightmap.GetHeight(x + 1, z + 0), z + 0);
            var c = new JVector(x + 1, Heightmap.GetHeight(x + 1, z + 1), z + 1);
            var d = new JVector(x + 0, Heightmap.GetHeight(x + 0, z + 1), z + 1);

            //  a ----- b
            //  | \     |
            //  |  \    |
            //  |   \   |
            //  |    \  |
            //  d ----- c

            JTriangle tri0 = new JTriangle(a, c, b);
            JTriangle tri1 = new JTriangle(a, d, c);

            tri0.RayIntersect(origin, direction, JTriangle.CullMode.BackFacing, out JVector normal0, out float lambda0);
            tri1.RayIntersect(origin, direction, JTriangle.CullMode.BackFacing, out JVector normal1, out float lambda1);

            if (lambda0 < float.MaxValue || lambda1 < float.MaxValue)
            {
                if (lambda0 <= lambda1)
                {
                    normal = normal0;
                    lambda = lambda0;
                }
                else
                {
                    normal = normal1;
                    lambda = lambda1;
                }

                return true;
            }

            continue_walk:

            if (tMaxX < tMaxZ)
            {
                x += stepX;
                t = tMaxX;
                tMaxX += tDeltaX;
            }
            else
            {
                z += stepZ;
                t = tMaxZ;
                tMaxZ += tDeltaZ;
            }
        }

        normal = JVector.Zero; lambda = 0.0f;
        return false;
    }
}

public class HeightmapDetection : IBroadPhaseFilter
{
    private readonly World world;
    private readonly HeightmapTester shape;
    private readonly ulong minIndex;

    public HeightmapDetection(World world, HeightmapTester shape)
    {
        this.shape = shape;
        this.world = world;

        (minIndex, _) = World.RequestId(Heightmap.Width * Heightmap.Height * 2);
    }

    public bool Filter(IDynamicTreeProxy shapeA, IDynamicTreeProxy shapeB)
    {
        if (shapeA != shape && shapeB != shape) return true;

        var collider = shapeA == shape ? shapeB : shapeA;

        if (collider is not RigidBodyShape rbs || rbs.RigidBody.MotionType != MotionType.Dynamic
                                               || !rbs.RigidBody.IsActive) return false;

        ref RigidBodyData body = ref rbs.RigidBody!.Data;

        var min = collider.WorldBoundingBox.Min;
        var max = collider.WorldBoundingBox.Max;

        int minX = Math.Max(0, (int)min.X);
        int minZ = Math.Max(0, (int)min.Z);
        int maxX = Math.Min(Heightmap.Width - 1, (int)max.X + 1);
        int maxZ = Math.Min(Heightmap.Height - 1, (int)max.Z + 1);

        for (int x = minX; x < maxX; x++)
        {
            for (int z = minZ; z < maxZ; z++)
            {
                // 四边形中的第一个三角形

                ulong index = 2 * (ulong)(x * Heightmap.Width + z);

                CollisionTriangle triangle;

                triangle.A = new JVector(x + 0, Heightmap.GetHeight(x + 0, z + 0), z + 0);
                triangle.B = new JVector(x + 1, Heightmap.GetHeight(x + 1, z + 1), z + 1);
                triangle.C = new JVector(x + 1, Heightmap.GetHeight(x + 1, z + 0), z + 0);

                JVector normal = JVector.Normalize((triangle.B - triangle.A) % (triangle.C - triangle.A));

                bool hit = NarrowPhase.MprEpa(triangle, rbs, body.Orientation, body.Position,
                    out JVector pointA, out JVector pointB, out _, out float penetration);

                if (hit)
                {
                    world.RegisterContact(rbs.ShapeId, minIndex + index, world.NullBody, rbs.RigidBody,
                        pointA, pointB, normal);
                }

                // 四边形中的第二个三角形

                index += 1;
                triangle.A = new JVector(x + 0, Heightmap.GetHeight(x + 0, z + 0), z + 0);
                triangle.B = new JVector(x + 0, Heightmap.GetHeight(x + 0, z + 1), z + 1);
                triangle.C = new JVector(x + 1, Heightmap.GetHeight(x + 1, z + 1), z + 1);

                normal = JVector.Normalize((triangle.B - triangle.A) % (triangle.C - triangle.A));

                hit = NarrowPhase.MprEpa(triangle, rbs, body.Orientation, body.Position,
                    out pointA, out pointB, out _, out penetration);

                if (hit)
                {
                    world.RegisterContact(rbs.ShapeId, minIndex + index, world.NullBody, rbs.RigidBody,
                        pointA, pointB, normal);
                }
            }
        }

        return false;
    }
}

public class Demo25 : IDemo, IDrawUpdate
{
    public string Name => "Heightmap (Custom Collision)";
    public string Description => "Procedural heightmap with custom per-triangle collision and raycasting.";

    private MutableMeshDrawable terrainRenderer = null!;

    // 仅用于渲染……
    TriangleVertexIndex[] GetIndices(int width, int height)
    {
        var indices = new List<TriangleVertexIndex>();

        for (int j = 0; j < height - 1; j++)
        {
            for (int i = 0; i < width - 1; i++)
            {
                int a = j * width + i;
                int b = j * width + (i + 1);
                int c = (j + 1) * width + i;
                int d = (j + 1) * width + (i + 1);

                indices.Add(new TriangleVertexIndex(b, a, c)); // 三角形 1
                indices.Add(new TriangleVertexIndex(d, b, c)); // 三角形 2
            }
        }
        return indices.ToArray();
    }

    // 仅用于渲染……
    void FillVertices(Vertex[] vertices, int width, int height)
    {
        for (int j = 0; j < height; j++)
        {
            for (int i = 0; i < width; i++)
            {
                int index = j * width + i;

                vertices[index].Position = new Vector3(i, Heightmap.GetHeight(i, j), j);
                vertices[index].Texture = new Vector2(i * 0.5f, j * 0.5f);
                // 法线会在 terrainRenderer.VerticesChanged() 中自动计算。
            }
        }
    }

    public void Build(Playground pg, World world)
    {
        var tester = new HeightmapTester(Heightmap.GetBoundingBox());

        world.BroadPhaseFilter = new HeightmapDetection(world, tester);
        world.DynamicTree.AddProxy(tester, false);

        // 仅用于渲染……
        terrainRenderer = RenderWindow.Instance.GetDrawable<MutableMeshDrawable>();
        terrainRenderer.SetTriangles(GetIndices(Heightmap.Width, Heightmap.Height));
        terrainRenderer.Material = ClothMaterial.Create();
        FillVertices(terrainRenderer.Mesh.Vertices, Heightmap.Width, Heightmap.Height);
        terrainRenderer.RefreshGeometry();
    }

    public void DrawUpdate()
    {
        terrainRenderer.Push(Matrix4.Identity);
    }
}
