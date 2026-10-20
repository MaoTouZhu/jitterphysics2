using System;
using System.Collections.Generic;
using Vellum;
using Vellum.Rendering;
using JitterDemo.Renderer.OpenGL;

namespace JitterDemo.Renderer;

/// 引擎的场景级宿主。持有阴影投射器、光照着色器、调试
/// 渲染器、GUI 后端、天空盒、相机，以及实例化可绘制对象的注册表。
public class RenderWindow : GLFWWindow
{
    public static RenderWindow Instance { get; private set; } = null!;

    public Camera Camera { get; set; }
    public ShadowCaster Shadow { get; private set; } = null!;
    public DebugRenderer DebugRenderer { get; private set; } = null!;
    public GuiRenderer GuiRenderer { get; private set; } = null!;

    public bool ShowShadowDebug { get; set; }

    private Skybox skybox = null!;
    private Shader litShader = null!;
    private TexturedQuad? shadowDebug;

    private readonly Dictionary<Type, InstancedDrawable> drawables = new();

    private double lastTime;

    public RenderWindow()
    {
        Instance = this;
        Camera = new FreeCamera();
    }

    /// 首次请求某个可绘制对象时注册它，并按运行时类型缓存。
    public T GetDrawable<T>() where T : InstancedDrawable, new()
    {
        if (!drawables.TryGetValue(typeof(T), out var d))
        {
            d = new T();
            drawables.Add(typeof(T), d);
        }
        return (T)d;
    }

    /// 注册一个由外部构造的可绘制对象。当构造需要参数时很有用。
    public T RegisterDrawable<T>(T drawable) where T : InstancedDrawable
    {
        drawables[typeof(T)] = drawable;
        return drawable;
    }

    public IReadOnlyCollection<InstancedDrawable> Drawables => drawables.Values;

    public Ui Gui { get; private set; } = null!;
    public bool WantsCaptureMouse { get; protected set; }
    public bool WantsCaptureKeyboard { get; protected set; }

    public override void Load()
    {
        GuiRenderer = new GuiRenderer();
        Gui = new Ui(GuiRenderer)
        {
            DefaultFontSize = 18f,
            FontStack = UiFont.Merge(
                UiFont.Source(UiFonts.DefaultSans),
                UiFont.Source(MaterialSymbols.Font, offsetY: 2f)),
            Lcd = true,
            Platform = new UiPlatform(this)
        };
        Gui.Theme.SurfaceBg = Color.Transparent;
        
        Shadow = new ShadowCaster();
        DebugRenderer = new DebugRenderer();

        skybox = new Skybox();
        skybox.Load();

        litShader = LitShader.Create();

        var fb = FramebufferSize;
        DebugRenderer.Load();

        VerticalSync = true;

        Camera.Position = new Vector3(0, 4, 8);
        Camera.Update(Keyboard, Mouse, Width > 0 ? (float)Width / Math.Max(1, Height) : 1f);

        lastTime = Time;
    }

    protected virtual void DrawCustomOverlay(int logicalWidth, int logicalHeight,
        int framebufferWidth, int framebufferHeight)
    {
    }

    public override void Draw()
    {
        float dt = (float)(Time - lastTime);
        lastTime = Time;

        // --- 渲染设置 -------------------------------------------------
        GLDevice.Enable(Capability.DepthTest);
        GLDevice.Enable(Capability.Blend);
        GLDevice.Blend(BlendFunc.SrcAlpha, BlendFunc.OneMinusSrcAlpha);

        GLDevice.ClearColor(73f / 255f, 76f / 255f, 92f / 255f, 1f);
        GLDevice.Clear(ClearFlags.ColorAndDepth);

        skybox.Draw(Camera);

        // --- 一次性上传所有可绘制对象的实例缓冲区 --------------------
        foreach (var d in drawables.Values) d.UploadInstances();

        // --- 阴影通道 --------------------------------------------------
        Shadow.Render(Camera, Width, Math.Max(1, Height), _ =>
        {
            foreach (var d in drawables.Values) d.DrawShadow();
        });

        // --- 光照通道 -----------------------------------------------------
        (int fbw, int fbh) = FramebufferSize;
        GLDevice.Viewport(0, 0, fbw, fbh);

        litShader.Use();
        litShader.Set("uProjection", Camera.ProjectionMatrix);
        litShader.Set("uView", Camera.ViewMatrix);
        litShader.Set("uViewPos", Camera.Position);
        Shadow.BindToLitShader(litShader);

        foreach (var d in drawables.Values) d.DrawLit(litShader);

        // --- 调试线条 --------------------------------------------------
        DebugRenderer.Draw(Camera);

        if (ShowShadowDebug)
        {
            shadowDebug ??= new TexturedQuad { Texture = Shadow.DepthMaps[0], Size = new Vector2(200, 200) };
            shadowDebug.Position = new Vector2(10, 10);
            shadowDebug.Draw(fbw, fbh);
        }

        if (Keyboard.IsKeyDown(Keyboard.Key.Escape)) Close();

        GuiRenderer.SetFramebufferSize(fbw, fbh);
        DrawCustomOverlay(Width, Math.Max(1, Height), fbw, fbh);

        WantsCaptureKeyboard = Gui.WantsCaptureKeyboard;
        WantsCaptureMouse = Gui.WantsCaptureMouse;

        // --- 使用完毕后清除每帧的实例数据 ------------
        foreach (var d in drawables.Values) d.Clear();

        // --- 相机更新 ------------------------------------------------
        Camera.IgnoreKeyboardInput = WantsCaptureKeyboard;
        Camera.IgnoreMouseInput = WantsCaptureMouse;
        float aspect = Width > 0 ? (float)Width / Math.Max(1, Height) : 1f;
        Camera.Update(Keyboard, Mouse, aspect);
    }
}
