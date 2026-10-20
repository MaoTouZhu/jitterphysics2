using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer;

/// 一个屏幕空间纹理四边形，适合用于叠加显示阴影贴图或 UI 预览。
public sealed class TexturedQuad
{
    private readonly Vao vao;
    private readonly Shader shader;

    public Texture2D Texture { get; set; } = null!;
    public Vector2 Position { get; set; }
    public Vector2 Size { get; set; }

    public TexturedQuad(int width = 200, int height = 200)
    {
        Size = new Vector2(width, height);

        vao = new Vao();
        // 预先绑定 VAO，这样下面上传索引缓冲区时会把它的
        // EBO 附加到“这个”VAO 上，而不是覆盖此前绑定的那个。
        vao.Bind();

        var vbo = GpuBuffer.Vertex();
        var ebo = GpuBuffer.Index();

        Vector2[] vertices =
        {
            new(0, 0), new(1, 0), new(1, 1), new(0, 1)
        };
        TriangleVertexIndex[] indices =
        {
            new(1, 0, 2), new(2, 0, 3)
        };

        vbo.Upload<Vector2>(vertices);
        ebo.Upload<TriangleVertexIndex>(indices);

        vao.Attrib(0, vbo, 2, AttribType.Float, 2 * sizeof(float), 0);
        vao.AttachIndexBuffer(ebo);

        shader = new Shader(Vs, Fs);
        shader.Use();
        shader.Set("uTexture", 0);
    }

    public void Draw(int framebufferWidth, int framebufferHeight)
    {
        Texture.Bind(0);

        vao.Bind();
        shader.Use();

        Matrix4 m = MatrixHelper.CreateOrthographicOffCenter(0, framebufferWidth, framebufferHeight, 0, 1f, -1f);
        shader.Set("uProjection", m);
        shader.Set("uOffset", Position);
        shader.Set("uSize", Size);

        GLDevice.Enable(Capability.Blend);
        GLDevice.Disable(Capability.DepthTest);
        GLDevice.DrawElements(DrawMode.Triangles, 6, IndexType.UnsignedInt, 0);
        GLDevice.Disable(Capability.Blend);
        GLDevice.Enable(Capability.DepthTest);
    }

    private const string Vs = @"
#version 330 core
layout(location = 0) in vec2 aPos;

uniform vec2 uOffset;
uniform vec2 uSize;
uniform mat4 uProjection;

out vec2 vUV;

void main()
{
    gl_Position = uProjection * vec4(aPos * uSize + uOffset, 0.0, 1.0);
    vUV = aPos;
}
";

    private const string Fs = @"
#version 330 core
uniform sampler2D uTexture;
in vec2 vUV;
out vec4 FragColor;

void main() { FragColor = texture(uTexture, vUV); }
";
}
