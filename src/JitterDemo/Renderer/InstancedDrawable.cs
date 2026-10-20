using System;
using System.Runtime.InteropServices;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer;

[StructLayout(LayoutKind.Sequential)]
internal struct Instance
{
    public Matrix4 Transform;
    public Vector3 Color;
}

/// 逐组材质覆盖（用于演示汽车等多材质网格）。
public struct MaterialSlot
{
    public int GroupIndex;
    public Material Material;

    public MaterialSlot(int groupIndex, in Material material)
    {
        GroupIndex = groupIndex;
        Material = material;
    }
}

/// <summary>
/// 持有单个网格的 GPU 资源，以及可增长的逐实例变换 + 颜色缓冲。
/// 调用方每帧使用 <see cref="Push"/>；引擎会一次性上传并绘制所有实例，
/// 阴影绘制阶段和光照绘制阶段都是如此。
/// </summary>
public class InstancedDrawable
{
    public Mesh Mesh { get; protected set; }

    public Material Material = Material.Default;

    /// 可选的逐组材质覆盖。为 null 时，<see cref="Material"/>
    /// 应用于所有三角形。设置后，未列出的组回退到
    /// <see cref="Material"/>。
    public MaterialSlot[]? Groups;

    /// 绘制网格的双面（两次绘制调用，第二次翻转法线）。
    public bool TwoSided;

    /// 在阴影绘制阶段禁用背面剔除（对薄几何体很有用）。
    public bool ShadowsDoubleSided;

    private Instance[] instances = new Instance[64];
    private int count;
    public int InstanceCount => count;

    protected Vao Vao = null!;
    protected GpuBuffer Vbo = null!;
    protected GpuBuffer Ibo = null!;
    protected GpuBuffer InstanceVbo = null!;

    public InstancedDrawable(Mesh mesh)
    {
        Mesh = mesh;
        CreateBuffers();
    }

    protected InstancedDrawable()
    {
        Mesh = null!;
    }

    protected void CreateBuffers()
    {
        Vao = new Vao();
        // GL_ELEMENT_ARRAY_BUFFER 的绑定属于 VAO 状态：先绑定我们的 VAO，
        // 这样下面的索引上传会把 EBO 关联到这个 VAO，而不是
        // 悄悄破坏之前恰好绑定的那个 VAO。
        Vao.Bind();

        Vbo = GpuBuffer.Vertex();
        Vbo.Upload<Vertex>(Mesh.Vertices);

        Ibo = GpuBuffer.Index();
        Ibo.Upload<TriangleVertexIndex>(Mesh.Indices);

        InstanceVbo = GpuBuffer.Vertex();

        int sof = sizeof(float);
        // 顶点流：位置(3) + 法线(3) + uv(2) = 8 个 float
        Vao.Attrib(0, Vbo, 3, AttribType.Float, 8 * sof, 0 * sof);
        Vao.Attrib(1, Vbo, 3, AttribType.Float, 8 * sof, 3 * sof);
        Vao.Attrib(2, Vbo, 2, AttribType.Float, 8 * sof, 6 * sof);

        // 实例流：mat4（4 个属性，位置 3..6）+ 颜色（1 个属性，位置 7）
        int instStride = 19 * sof;
        Vao.Attrib(3, InstanceVbo, 4, AttribType.Float, instStride, 0 * sof, divisor: 1);
        Vao.Attrib(4, InstanceVbo, 4, AttribType.Float, instStride, 4 * sof, divisor: 1);
        Vao.Attrib(5, InstanceVbo, 4, AttribType.Float, instStride, 8 * sof, divisor: 1);
        Vao.Attrib(6, InstanceVbo, 4, AttribType.Float, instStride, 12 * sof, divisor: 1);
        Vao.Attrib(7, InstanceVbo, 3, AttribType.Float, instStride, 16 * sof, divisor: 1);

        Vao.AttachIndexBuffer(Ibo);
    }

    public void Push(in Matrix4 transform)
    {
        Push(transform, new Vector3(1, 1, 1));
    }

    public void Push(in Matrix4 transform, in Vector3 color)
    {
        if (count == instances.Length)
        {
            Array.Resize(ref instances, instances.Length * 2);
        }

        ref Instance slot = ref instances[count++];
        slot.Transform = transform;
        slot.Color = color;
    }

    public void Clear() => count = 0;

    /// 引擎在每帧所有绘制阶段之前调用一次。
    public void UploadInstances()
    {
        if (count == 0) return;
        InstanceVbo.Stream<Instance>(instances, count);
    }

    /// 由阴影绘制阶段调用。默认实现绘制整个网格。
    public virtual void DrawShadow()
    {
        if (count == 0) return;
        Vao.Bind();

        if (ShadowsDoubleSided) GLDevice.Disable(Capability.CullFace);
        else GLDevice.CullFace(CullMode.Front);

        GLDevice.DrawElementsInstanced(DrawMode.Triangles, Mesh.Indices.Length * 3,
            IndexType.UnsignedInt, 0, count);

        if (ShadowsDoubleSided) GLDevice.Enable(Capability.CullFace);
        GLDevice.CullFace(CullMode.Back);
    }

    /// 由光照绘制阶段调用。默认实现绑定材质并
    /// 绘制整个网格。可重写以在网格上变化材质。
    public virtual void DrawLit(Shader shader)
    {
        if (count == 0) return;
        Vao.Bind();

        if (Groups is { Length: > 0 })
        {
            DrawLitGrouped(shader);
        }
        else
        {
            DrawAll(shader, Material);
        }

        if (TwoSided)
        {
            Material flipped = Material;
            flipped.FlipNormals = !flipped.FlipNormals;
            GLDevice.CullFace(CullMode.Front);
            DrawAll(shader, flipped);
            GLDevice.CullFace(CullMode.Back);
        }
    }

    private void DrawAll(Shader shader, in Material material)
    {
        ApplyMaterial(shader, material);
        GLDevice.DrawElementsInstanced(DrawMode.Triangles, Mesh.Indices.Length * 3,
            IndexType.UnsignedInt, 0, count);
    }

    private void DrawLitGrouped(Shader shader)
    {
        int sof = sizeof(float);
        var groups = Groups!;

        // 第 1 遍：用默认材质绘制所有未被组覆盖的索引范围。
        // 这里假设覆盖槽按 GroupIndex 排序（考虑到 OBJ 加载器产生的组是
        // 连续的，这实际上就等于“按声明顺序”）。
        ApplyMaterial(shader, Material);
        int cursor = 0;
        for (int i = 0; i < groups.Length; i++)
        {
            var g = Mesh.Groups[groups[i].GroupIndex];
            if (g.FromInclusive > cursor) DrawRange(cursor, g.FromInclusive);
            cursor = g.ToExclusive;
        }
        if (cursor < Mesh.Indices.Length) DrawRange(cursor, Mesh.Indices.Length);

        // 第 2 遍：绘制每个覆盖组。把这些推迟到最后，可确保
        // 半透明槽（例如汽车的玻璃座舱）在不透明几何体之上混合，
        // 而不是被不透明几何体的深度剔除。
        for (int i = 0; i < groups.Length; i++)
        {
            ApplyMaterial(shader, groups[i].Material);
            var g = Mesh.Groups[groups[i].GroupIndex];
            DrawRange(g.FromInclusive, g.ToExclusive);
        }

        void DrawRange(int from, int to)
        {
            if (to <= from) return;
            GLDevice.DrawElementsInstanced(DrawMode.Triangles,
                (to - from) * 3, IndexType.UnsignedInt, from * sof * 3, count);
        }
    }

    protected void ApplyMaterial(Shader shader, in Material material)
    {
        shader.Set("uMat.tint", material.Tint);
        shader.Set("uMat.specular", material.Specular);
        shader.Set("uMat.shininess", material.Shininess);
        shader.Set("uMat.alpha", material.Alpha);
        shader.Set("uMat.vertexColorWeight", material.VertexColorWeight);
        shader.Set("uMat.textureWeight", material.TextureWeight);
        shader.Set("uMat.normalFlip", material.FlipNormals ? -1f : 1f);

        // 始终向纹理单元 3 绑定某个纹理，这样无论之前绑定了什么，
        // 片元着色器的漫反射采样都是确定性的。
        (material.Texture ?? Texture2D.Empty()).Bind(3);
    }
}
