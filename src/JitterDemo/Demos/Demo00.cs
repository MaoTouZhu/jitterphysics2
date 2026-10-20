using Jitter2;
using Jitter2.LinearMath;
using JitterDemo.Renderer;

namespace JitterDemo;

public class DecomposedTeapot : TriangleMeshDrawable
{
    public DecomposedTeapot() : base("teapot_hull.obj", 0.03f)
    {
        // 为 OBJ 中的每个凸包分配各自的调色板条目，使每个茶壶的各个部件
        // 在黑色实例颜色下也能清晰区分。
        Groups = new MaterialSlot[Mesh.Groups.Length];
        for (int i = 0; i < Groups.Length; i++)
        {
            var m = Material.Default;
            m.Tint = ColorGenerator.GetColor(i * (i << 6));
            m.VertexColorWeight = 0f;
            Groups[i] = new MaterialSlot(i, m);
        }
    }
}

public class Demo00 : IDemo, IDrawUpdate
{
    private ConvexDecomposition<DecomposedTeapot> teapotDecomp = null!;

    public string Name => "Convex Decomposition";
    public string Description => "Convex-decomposed teapot models with compound convex hull collision.";

    public void Build(Playground pg, World world)
    {
        pg.AddFloor();

        teapotDecomp = new ConvexDecomposition<DecomposedTeapot>(world);
        teapotDecomp.Load();

        for (int i = 0; i < 6; i++)
        {
            teapotDecomp.Spawn(new JVector(0, 10 + i * 3, -14));
            teapotDecomp.Spawn(new JVector(0, 10 + i * 3, -6));
            teapotDecomp.Spawn(new JVector(5, 10 + i * 3, -14));
            teapotDecomp.Spawn(new JVector(5, 10 + i * 3, -6));
        }

        world.SolverIterations = (8, 4);
    }

    public void DrawUpdate()
    {
        teapotDecomp.PushMatrices();
    }
}
