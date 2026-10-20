using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer;

/// 描述光照着色器如何对表面进行着色。每个字段彼此独立：
/// 最终片元颜色根据两个权重标量混合顶点颜色、无纹理的色调以及可选纹理。
public struct Material
{
    /// 当 VertexColorWeight &lt; 1 时叠加的固定颜色。
    public Vector3 Tint;

    /// 镜面反射颜色。
    public Vector3 Specular;

    /// 镜面反射指数。
    public float Shininess;

    /// 应用于最终颜色的透明度乘数。
    public float Alpha;

    /// Ambient = VertexColorWeight * vertexInstanceColor + (1 - VertexColorWeight) * Tint。
    /// 0 = 仅使用 Tint，1 = 仅使用逐实例颜色。
    public float VertexColorWeight;

    /// Diffuse = (1 - TextureWeight) * flatGrey + TextureWeight * sampleTexture。
    /// 0 = 无纹理，1 = 完全使用纹理。
    public float TextureWeight;

    /// 若为 true，则在光照前翻转顶点法线（用于双面绘制）。
    public bool FlipNormals;

    /// 当 TextureWeight &gt; 0 时进行采样。仅当 TextureWeight = 0 时可以为 null。
    public Texture2D? Texture;

    public static Material Default => new()
    {
        Tint = Vector3.Zero,
        Specular = new Vector3(0.1f, 0.1f, 0.1f),
        Shininess = 128f,
        Alpha = 1f,
        VertexColorWeight = 1f,
        TextureWeight = 0f,
        FlipNormals = false,
        Texture = null
    };
}
