using System;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer;

/// <summary>
/// 级联阴影贴图，级联数量和阴影贴图分辨率均可配置。
/// 通过把正交投影拟合到相机视锥切片来计算每级的光照矩阵，把每级渲染到
/// 深度纹理，并绑定好全部资源以供光照通道使用。
/// </summary>
public sealed class ShadowCaster
{
    public const int CascadeCount = 3;
    public const int ShadowMapSize = 4096;

    public Texture2D[] DepthMaps { get; } = new Texture2D[CascadeCount];

    private readonly Framebuffer[] depthFramebuffers = new Framebuffer[CascadeCount];
    private readonly Matrix4[] lightMatrices = new Matrix4[CascadeCount];

    /// 每个级联在线性视图空间中的范围 [near, far]，按级联顺序排列。
    public float[] CascadeSplits { get; set; } = { 20f, 60f };

    public Vector3 LightDirection { get; set; } = Vector3.Normalize(new Vector3(1, 2, 1));

    private readonly Shader shadowShader;

    public ShadowCaster()
    {
        for (int i = 0; i < CascadeCount; i++)
        {
            var tex = new Texture2D();
            tex.Allocate(TextureFormat.Depth, ShadowMapSize, ShadowMapSize, TexelType.Float);
            tex.SetMinMagFilter(TextureFilter.Nearest, TextureFilter.Nearest);
            tex.SetWrap(TextureWrap.ClampToBorder);
            tex.SetBorderColor(new Vector4(1, 1, 1, 1));

            var fb = new Framebuffer();
            fb.AttachDepthTexture(tex);

            DepthMaps[i] = tex;
            depthFramebuffers[i] = fb;
        }

        Framebuffer.Default.Bind();

        shadowShader = new Shader(ShadowVs, ShadowFs);
    }

    public ReadOnlySpan<Matrix4> LightMatrices => lightMatrices;

    /// <summary>
    /// 渲染阴影级联。在绑定帧缓冲并设置光照矩阵 uniform 之后，对每个级联
    /// 调用一次传入的 <paramref name="drawCallback"/>。
    /// </summary>
    public void Render(Camera camera, int viewportW, int viewportH, Action<Shader> drawCallback)
    {
        GLDevice.Viewport(0, 0, ShadowMapSize, ShadowMapSize);
        shadowShader.Use();

        ComputeLightMatrices(camera, viewportW, viewportH);

        for (int i = 0; i < CascadeCount; i++)
        {
            depthFramebuffers[i].Bind();
            GLDevice.Clear(ClearFlags.Depth);

            shadowShader.Set("uLightMatrix", lightMatrices[i]);
            drawCallback(shadowShader);
        }

        Framebuffer.Default.Bind();

        // 将深度纹理保留在 [0..CascadeCount-1] 号纹理单元上，这样光照通道
        // 可以直接从中采样。
        for (uint i = 0; i < CascadeCount; i++) DepthMaps[i].Bind(i);
    }

    /// 把级联矩阵、分割距离和太阳方向上传到指定的光照着色器。
    public void BindToLitShader(Shader lit)
    {
        for (int i = 0; i < CascadeCount; i++)
        {
            lit.Set($"uLightMatrices[{i}]", lightMatrices[i]);
        }

        float[] splits = new float[CascadeCount];
        for (int i = 0; i < CascadeCount; i++)
        {
            splits[i] = i < CascadeSplits.Length ? CascadeSplits[i] : float.MaxValue;
        }
        lit.Set("uCascadeSplits", splits);
        lit.Set("uSunDir", LightDirection);
    }

    private void ComputeLightMatrices(Camera camera, int viewportW, int viewportH)
    {
        float aspect = (float)viewportW / viewportH;

        for (int i = 0; i < CascadeCount; i++)
        {
            float near = i == 0 ? camera.NearPlane : CascadeSplits[i - 1];
            float far = i < CascadeSplits.Length ? CascadeSplits[i] : camera.FarPlane;
            lightMatrices[i] = FitLightMatrixToFrustum(camera, aspect, near, far);
        }
    }

    private Matrix4 FitLightMatrixToFrustum(Camera camera, float aspect, float near, float far)
    {
        Matrix4 proj = MatrixHelper.CreatePerspectiveFieldOfView(camera.FieldOfView, aspect, near, far);
        Span<Vector4> corners = stackalloc Vector4[8];
        FrustumCorners(corners, proj, camera.ViewMatrix);

        Vector3 center = Vector3.Zero;
        for (int i = 0; i < 8; i++) center += corners[i].XYZ;
        center *= 1f / 8f;

        Matrix4 lightView = MatrixHelper.CreateLookAt(center + LightDirection, center, Vector3.UnitY);

        Vector3 min = new(float.MaxValue, float.MaxValue, float.MaxValue);
        Vector3 max = new(float.MinValue, float.MinValue, float.MinValue);
        for (int i = 0; i < 8; i++)
        {
            Vector4 t = lightView * corners[i];
            min.X = Math.Min(min.X, t.X); max.X = Math.Max(max.X, t.X);
            min.Y = Math.Min(min.Y, t.Y); max.Y = Math.Max(max.Y, t.Y);
            min.Z = Math.Min(min.Z, t.Z); max.Z = Math.Max(max.Z, t.Z);
        }

        // 将近平面和远平面向外扩展，使视锥之外的投射者仍能出现在阴影中。
        const float zMult = 3f;
        min.Z = min.Z < 0 ? min.Z * zMult : min.Z / zMult;
        max.Z = max.Z < 0 ? max.Z / zMult : max.Z * zMult;

        Matrix4 lightProj = MatrixHelper.CreateOrthographicOffCenter(min.X, max.X, min.Y, max.Y, min.Z, max.Z);
        return lightProj * lightView;
    }

    private static void FrustumCorners(Span<Vector4> corners, in Matrix4 proj, in Matrix4 view)
    {
        if (!Matrix4.Invert(proj * view, out Matrix4 inv))
            throw new InvalidOperationException("Could not invert projection * view matrix.");

        for (int x = 0; x < 2; x++)
        for (int y = 0; y < 2; y++)
        for (int z = 0; z < 2; z++)
        {
            Vector4 p = new(2f * x - 1f, 2f * y - 1f, 2f * z - 1f, 1f);
            p = inv * p;
            corners[4 * x + 2 * y + z] = p * (1f / p.W);
        }
    }

    private const string ShadowVs = @"
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 3) in mat4 aInstanceModel;

uniform mat4 uLightMatrix;

void main()
{
    gl_Position = uLightMatrix * aInstanceModel * vec4(aPos, 1.0);
}
";

    private const string ShadowFs = @"
#version 330 core
void main() { }
";
}
