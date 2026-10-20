using System.Collections.Generic;
using System.Linq;
using Jitter2;
using Jitter2.Collision.Shapes;
using Jitter2.Dynamics;
using Jitter2.LinearMath;
using JitterDemo.Renderer;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo;

public class Teapot : TriangleMeshDrawable
{
    public Teapot() : base("teapot.obj.zip", 1f) { }
}

public class Demo24 : IDemo, IDrawUpdate
{
    public string Name => "Convex PointCloudShape";
    public string Description => "Convex hull collision shape created from sampled teapot vertices.";

    private Teapot teapot = null!;
    private List<RigidBody> teapotBodies = null!;
    private Matrix4 shift;

    public void Build(Playground pg, World world)
    {
        pg.AddFloor();

        teapotBodies = new List<RigidBody>();

        teapot = RenderWindow.Instance.GetDrawable<Teapot>();

        var vertices = teapot.Mesh.Vertices.Select(v
            => new JVector(v.Position.X, v.Position.Y, v.Position.Z)).Distinct().ToList();

        // 在茶壶的凸包上找出少量点。
        var reducedVertices = ShapeHelper.SampleHull(vertices, subdivisions: 3);

        // 用这些点创建 PointCloudShape。也可以使用茶壶的全部顶点，
        // 但那样会更慢，因为其中还包含位于茶壶凸包内部的顶点。
        PointCloudShape pcs = new PointCloudShape(reducedVertices);

        // Jitter 要求质心位于原点！

        // 获取质心并平移凸包，使新的
        // 质心位于原点。
        pcs.GetCenter(out JVector ctr);
        pcs.Shift = -ctr;
        // pcs.GetCenter(out JVector ctr); <- 此时这里会返回 (0,0,0)

        // 同时平移茶壶的可视表示
        shift = MatrixHelper.CreateTranslation(-ctr.X, -ctr.Y, -ctr.Z);

        for (int i = 0; i < 16; i++)
        {
            RigidBody body = world.CreateRigidBody();
            body.Position = new JVector(0, 10 + i * 3, 0);
            body.AddShape(pcs.Clone());
            teapotBodies.Add(body);
        }
    }

    public void DrawUpdate()
    {
        foreach (var body in teapotBodies)
        {
            var color = ColorGenerator.GetColor(body.GetHashCode());
            if (!body.IsActive) color += new Vector3(0.2f, 0.2f, 0.2f);
            teapot.Push(Conversion.FromJitter(body) * shift, color);
        }
    }
}
