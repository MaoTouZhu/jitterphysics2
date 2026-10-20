using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer;

/// 包装从 OBJ 文件（普通或压缩）加载的网格的可绘制对象。
public class TriangleMeshDrawable : InstancedDrawable
{
    public TriangleMeshDrawable(string filename, float scale = 1f, bool reverseWinding = false)
        : base(Mesh.LoadObj(filename, scale, reverseWinding))
    {
    }

    public TriangleMeshDrawable(Mesh mesh) : base(mesh) { }
}

/// 顶点数据可在运行时修改的可绘制对象（用于软体、高度图）。
/// 调用一次 <see cref="SetTriangles"/> 建立索引缓冲，之后每帧修改
/// <see cref="Mesh"/>.Vertices 并调用 <see cref="RefreshGeometry"/>。
public class MutableMeshDrawable : InstancedDrawable
{
    public MutableMeshDrawable() : base(new Mesh(new Vertex[4], new TriangleVertexIndex[0]))
    {
        // 先用占位数据初始化以便构造成功；SetTriangles 会提供真实数据。
        TwoSided = true;
        ShadowsDoubleSided = true;
    }

    public void SetTriangles(TriangleVertexIndex[] triangles)
    {
        Mesh.Indices = triangles;
        // 与 CreateBuffers 中原因相同：元素缓冲的绑定属于 VAO 状态。
        Vao.Bind();
        Ibo.Upload<TriangleVertexIndex>(triangles);

        uint largest = 0;
        for (int i = 0; i < triangles.Length; i++)
        {
            if (triangles[i].T1 > largest) largest = triangles[i].T1;
            if (triangles[i].T2 > largest) largest = triangles[i].T2;
            if (triangles[i].T3 > largest) largest = triangles[i].T3;
        }

        Mesh.Vertices = new Vertex[largest + 1];
    }

    /// 根据三角形重新计算逐顶点法线并上传顶点数据。
    public void RefreshGeometry()
    {
        var verts = Mesh.Vertices;
        var inds = Mesh.Indices;

        for (int i = 0; i < verts.Length; i++) verts[i].Normal = Vector3.Zero;

        for (int i = 0; i < inds.Length; i++)
        {
            ref var v1 = ref verts[inds[i].T1];
            ref var v2 = ref verts[inds[i].T2];
            ref var v3 = ref verts[inds[i].T3];

            Vector3 n = Vector3.Cross(v2.Position - v1.Position, v3.Position - v1.Position);
            v1.Normal += n;
            v2.Normal += n;
            v3.Normal += n;
        }

        Vbo.Stream<Vertex>(verts, verts.Length);
    }
}
