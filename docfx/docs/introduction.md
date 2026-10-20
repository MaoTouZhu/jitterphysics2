---
_disableBreadcrumb: true
_disableContribution: true
---

<div class="jp-hero">
  <p class="jp-hero-eyebrow">快速 &middot; 轻量 &middot; 纯 C# &middot; MIT 许可证</p>
  <h1 class="jp-hero-title">Jitter Physics 2</h1>
  <p class="jp-hero-sub">
    面向 .NET 的基于冲量的 3D 刚体物理引擎 &mdash;<br>
    零原生依赖，凡能运行 .NET 之处皆可运行。
  </p>
  <div class="jp-hero-ctas">
    <a href="~/docs/tutorials/boxes/project-setup.md" class="jp-btn-primary">开始使用</a>
    <a href="https://github.com/notgiven688/jitterphysics2" class="jp-btn-ghost">在 GitHub 上查看</a>
  </div>
  <div class="jp-badges">
    <img src="https://img.shields.io/nuget/v/Jitter2?style=flat-square&label=NuGet&color=5ba8f5" alt="NuGet 版本">
    <img src="https://img.shields.io/github/stars/notgiven688/jitterphysics2?style=flat-square&label=Stars&color=5ba8f5" alt="GitHub 星标数">
    <img src="https://img.shields.io/badge/.NET-8%2B-blueviolet?style=flat-square" alt=".NET 8+">
    <img src="https://img.shields.io/github/license/notgiven688/jitterphysics2?style=flat-square&color=22c55e" alt="MIT 许可证">
  </div>
</div>

<div class="jp-features">
  <div class="jp-feature-card">
    <div class="jp-feature-icon">&#9889;</div>
    <div class="jp-feature-title">零依赖</div>
    <div class="jp-feature-body">纯 C# 实现，无原生 DLL，无 P/Invoke。可在任何支持 .NET 8 或更高版本的平台上运行。</div>
  </div>
  <div class="jp-feature-card">
    <div class="jp-feature-icon">&#128296;</div>
    <div class="jp-feature-title">基于冲量的动力学</div>
    <div class="jp-feature-body">半隐式欧拉积分器配合子步，在交互式帧率下实现稳定、高刚度的模拟。</div>
  </div>
  <div class="jp-feature-card">
    <div class="jp-feature-icon">&#129513;</div>
    <div class="jp-feature-title">确定性求解器</div>
    <div class="jp-feature-body">可选的跨平台确定性模式，用于可复现的模拟。</div>
  </div>
  <div class="jp-feature-card">
    <div class="jp-feature-icon">&#127919;</div>
    <div class="jp-feature-title">推测接触</div>
    <div class="jp-feature-body">仅线性方向的推测接触可减少快速平移造成的穿透（隧穿）。旋转引起的穿透不在覆盖范围内。</div>
  </div>
  <div class="jp-feature-card">
    <div class="jp-feature-icon">&#128260;</div>
    <div class="jp-feature-title">单精度或双精度</div>
    <div class="jp-feature-body">编译期精度切换 &mdash; 同一套 API 和源代码可配合 <code>float</code> 或 <code>double</code> 使用。</div>
  </div>
  <div class="jp-feature-card">
    <div class="jp-feature-icon">&#128085;</div>
    <div class="jp-feature-title">软体动力学</div>
    <div class="jp-feature-body">布料和可变形体可与刚体共存于同一个模拟世界。</div>
  </div>
</div>


<div class="demo-shell">
  <div class="demo-card" id="demo_card">
    <div class="demo-copy">
      <strong>交互式 Web 演示</strong>
      <span>直接在浏览器中运行物理引擎 &mdash; 无需安装。</span>
    </div>
    <button class="demo-button" id="load_demo_button" type="button">启动演示</button>
  </div>
  <div class="demo-frame-wrap" id="demo_frame_wrap" hidden>
    <iframe
      id="demo_frame"
      title="Jitter2 Web 演示"
      loading="lazy"
      allow="fullscreen"
      referrerpolicy="strict-origin-when-cross-origin"></iframe>
  </div>
</div>

<div class="jp-nav-cards">
  <a class="jp-nav-card" href="~/docs/tutorials/boxes/project-setup.md">
    <div class="jp-nav-card-icon">&#128218;</div>
    <div class="jp-nav-card-title">你好，世界教程</div>
    <div class="jp-nav-card-body">几分钟内搭建项目并模拟第一个下落的箱子。</div>
  </a>
  <a class="jp-nav-card" href="~/docs/tutorials/teapots/project-setup.md">
    <div class="jp-nav-card-icon">&#127861;</div>
    <div class="jp-nav-card-title">茶壶堆叠</div>
    <div class="jp-nav-card-body">加载凸包网格并模拟一座 Utah 茶壶塔。</div>
  </a>
  <a class="jp-nav-card" href="~/docs/documentation/world.md">
    <div class="jp-nav-card-icon">&#128196;</div>
    <div class="jp-nav-card-title">文档</div>
    <div class="jp-nav-card-body">深入了解刚体、形状、约束以及宽相树。</div>
  </a>
  <a class="jp-nav-card" href="~/api/Jitter2.yml">
    <div class="jp-nav-card-icon">&#128214;</div>
    <div class="jp-nav-card-title">API 参考</div>
    <div class="jp-nav-card-body">Jitter2 中每个公开类型的完整生成式参考。</div>
  </a>
</div>

<style>
/* ── 强调色与令牌 ───────────────────────────────────────── */
:root { --jp-accent: #5ba8f5; --jp-accent-hover: #7cbaf8; }

/* ── 主视觉区 ───────────────────────────────────────────── */
.jp-hero {
  padding: 2.5rem 0 2rem;
  text-align: center;
}
.jp-hero-eyebrow {
  font-size: 0.8rem;
  letter-spacing: 0.08em;
  text-transform: uppercase;
  color: var(--jp-accent);
  margin-bottom: 0.75rem;
}
.jp-hero-title {
  font-size: clamp(2rem, 5vw, 3rem);
  font-weight: 800;
  line-height: 1.1;
  margin: 0 0 1rem;
  color: var(--bs-body-color);
}
.jp-hero-sub {
  font-size: 1.1rem;
  color: var(--bs-secondary-color);
  line-height: 1.7;
  margin-bottom: 1.75rem;
}
.jp-hero-ctas {
  display: flex;
  gap: 0.75rem;
  justify-content: center;
  flex-wrap: wrap;
  margin-bottom: 1.25rem;
}
.jp-btn-primary, .jp-btn-ghost {
  display: inline-block;
  padding: 0.7rem 1.4rem;
  border-radius: 999px;
  font-weight: 700;
  font-size: 0.95rem;
  text-decoration: none;
  transition: background 0.15s, color 0.15s, border-color 0.15s;
}
.jp-btn-primary {
  background: var(--jp-accent);
  color: #08111c;
  border: 2px solid transparent;
}
.jp-btn-primary:hover { background: var(--jp-accent-hover); color: #08111c; text-decoration: none; }
.jp-btn-ghost {
  background: transparent;
  color: var(--bs-body-color);
  border: 2px solid var(--bs-border-color);
}
.jp-btn-ghost:hover {
  border-color: var(--jp-accent);
  color: var(--jp-accent);
  text-decoration: none;
}
.jp-badges {
  display: flex;
  gap: 0.4rem;
  justify-content: center;
  flex-wrap: wrap;
}
.jp-badges img { height: 20px; border-radius: 4px; }

/* ── 特性网格 ───────────────────────────────────────────── */
.jp-features {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(210px, 1fr));
  gap: 1rem;
  margin: 2.5rem 0;
}
.jp-feature-card {
  background: var(--bs-tertiary-bg);
  border: 1px solid var(--bs-border-color);
  border-radius: 0.75rem;
  padding: 1.25rem 1.1rem 1.1rem;
  transition: border-color 0.15s, transform 0.15s;
}
.jp-feature-card:hover {
  border-color: var(--jp-accent);
  transform: translateY(-2px);
}
.jp-feature-icon {
  font-size: 1.5rem;
  margin-bottom: 0.6rem;
  line-height: 1;
}
.jp-feature-title {
  font-weight: 700;
  font-size: 0.95rem;
  margin-bottom: 0.4rem;
  color: var(--bs-body-color);
}
.jp-feature-body {
  font-size: 0.85rem;
  color: var(--bs-secondary-color);
  line-height: 1.5;
}

/* ── 演示外壳 ───────────────────────────────────────────── */
.demo-shell { margin: 0 0 2rem; }
.demo-card,
.demo-frame-wrap {
  width: 100%;
  aspect-ratio: 800 / 600;
  border-radius: 10px;
  overflow: hidden;
  background: linear-gradient(160deg, #15181d 0%, #1f2732 100%);
  border: 1px solid rgba(255,255,255,0.08);
}
html[data-bs-theme="light"] .demo-card,
html[data-bs-theme="light"] .demo-frame-wrap {
  background: linear-gradient(160deg, #eef3f8 0%, #dde8f2 100%);
  border-color: rgba(34,48,60,0.12);
}
.demo-card {
  display: flex;
  flex-direction: column;
  justify-content: center;
  align-items: center;
  gap: 1rem;
  padding: 1.5rem;
  text-align: center;
}
.demo-copy { display: flex; flex-direction: column; gap: 0.5rem; max-width: 34rem; align-items: center; }
.demo-copy strong { font-size: 1.1rem; color: #fff; }
html[data-bs-theme="light"] .demo-copy strong { color: #22303c; }
.demo-copy span { color: #b6c2d0; line-height: 1.5; }
html[data-bs-theme="light"] .demo-copy span { color: #5f6f80; }
.demo-button {
  appearance: none;
  border: 0;
  border-radius: 999px;
  padding: 0.75rem 1.4rem;
  background: var(--jp-accent);
  color: #08111c;
  font: inherit;
  font-weight: 700;
  cursor: pointer;
  transition: background 0.15s;
}
.demo-button:hover, .demo-button:focus-visible { background: var(--jp-accent-hover); }
html[data-bs-theme="light"] .demo-button { background: #22303c; color: #f4f8fb; }
html[data-bs-theme="light"] .demo-button:hover,
html[data-bs-theme="light"] .demo-button:focus-visible { background: #314253; }
.demo-frame-wrap iframe { display: block; width: 100%; height: 100%; border: 0; }

/* ── 导航卡片 ───────────────────────────────────────────── */
.jp-nav-cards {
  display: grid;
  grid-template-columns: repeat(auto-fill, minmax(180px, 1fr));
  gap: 1rem;
  margin: 2rem 0;
}
.jp-nav-card {
  background: var(--bs-tertiary-bg);
  border: 1px solid var(--bs-border-color);
  border-radius: 0.75rem;
  padding: 1.2rem 1rem;
  text-decoration: none;
  color: var(--bs-body-color);
  transition: border-color 0.15s, transform 0.15s;
  display: block;
}
.jp-nav-card:hover {
  border-color: var(--jp-accent);
  transform: translateY(-2px);
  text-decoration: none;
  color: var(--bs-body-color);
}
.jp-nav-card-icon { font-size: 1.5rem; margin-bottom: 0.5rem; }
.jp-nav-card-title { font-weight: 700; font-size: 0.9rem; margin-bottom: 0.3rem; }
.jp-nav-card-body { font-size: 0.82rem; color: var(--bs-secondary-color); line-height: 1.4; }

/* ── 响应式微调 ─────────────────────────────────────────── */
@media (max-width: 600px) {
  .jp-hero { padding: 1.5rem 0 1.25rem; }
  .demo-card { padding: 1.1rem; }
}
</style>

<script>
(() => {
  const button   = document.getElementById('load_demo_button');
  const card     = document.getElementById('demo_card');
  const frameWrap = document.getElementById('demo_frame_wrap');
  const frame    = document.getElementById('demo_frame');
  if (!button || !card || !frameWrap || !frame) return;

  const buildFrameUrl = () => {
    const theme = document.documentElement.getAttribute('data-bs-theme') === 'light'
      ? 'light' : 'dark';
    return `https://jitterphysics.com/AppBundle/index.html?embed=1&theme=${theme}`;
  };

  button.addEventListener('click', () => {
    if (!frame.src) frame.src = buildFrameUrl();
    card.hidden = true;
    frameWrap.hidden = false;
  }, { once: true });

  new MutationObserver(() => {
    if (frame.src) frame.src = buildFrameUrl();
  }).observe(document.documentElement, {
    attributes: true,
    attributeFilter: ['data-bs-theme']
  });
})();
</script>
