# AGENTS.md：Jitter2 中文本地化维护指南

本文件面向后续的 LLM / 自动化代理。目标：**跟随上游 `notgiven688/jitterphysics2` 的更新，把新增或变更的注释、文档翻译成中文，并安全地更新 `zh` 分支。**

开始前先读：

1. `i18n/README.md` —— 人工维护说明
2. `i18n/glossary.yml` —— 统一术语
3. `tools/i18n/i18nlib.py` —— 注释/文档切分规则
4. `i18n/tm/meta.json` —— 上一次提取 TM 时的基准提交

---

## 1. 分支模型（不要搞反）

| 分支 | 作用 | 是否手改 |
|---|---|---|
| `main` | 上游英文镜像 | 不改，只快进 |
| `l10n` | 术语表、TM、工具、CI | 在这里维护 |
| `zh` | 生成的中文分支，含工具快照 | **不要直接改译文** |

`zh` 是生成物。直接改 `zh` 的译文，下次同步会被覆盖。要改译文，改 `i18n/tm/*.jsonl`，或在同步工作树里改源文件后重新提取 TM。

标准链路：

```text
upstream/main ──套用 TM──► .tmp-zh-sync ──verify/build──► zh
                              ▲
                              │
                   i18n/tm + tools/i18n
```

---

## 2. 先判断任务类型

### A. 只修一条已有译文

不要直接改 `zh`。在 `l10n` 分支：

```bash
git switch l10n
# 编辑 i18n/tm/csharp.jsonl / md.jsonl / yaml.jsonl / files.jsonl 中对应记录的 translation
make i18n-test
make i18n-verify
git add i18n/tm
git commit -m "i18n: 修正 xxx 译文"
git push origin l10n
```

然后按第 4 节重新生成 `zh`。

### B. 上游有新提交（最常见）

按第 3 节完整流程走。

### C. 上游新增了文件类型或工具需要调整

除了第 3 节，还要更新：

- `extract_tm.py` / `apply_tm.py` 的类型分派
- `i18n/tm/` 下新增的 JSONL
- `sync.sh` 的复制列表（确保新资产进入 `zh`）
- `i18n/README.md` 和本文件

---

## 3. 上游更新标准流程

### 3.0 检查环境

```bash
cd /home/magnet/jitterphysics2
git status --short          # 必须干净
git remote -v               # 需要 origin 和 upstream
git fetch upstream --prune
git log --oneline -3 upstream/main
```

如果本地没有 `upstream`：

```bash
git remote add upstream https://github.com/notgiven688/jitterphysics2.git
```

### 3.1 第一次套用：生成待译清单

```bash
git switch l10n
./tools/i18n/sync.sh --skip-build
```

行为：

- 在 `.tmp-zh-sync` 创建 `upstream/main` 的工作树
- 套用 `i18n/tm/` 中已有的翻译
- 未命中的单元保留英文，写入 `i18n/review/pending.md` 和 `pending.json`
- 有 pending 时脚本退出码为 1，这是**正常现象**

查看清单：

```bash
sed -n '1,200p' i18n/review/pending.md
cat i18n/review/pending.json
```

字段含义：

- `file`：相对路径
- `line`：行号（参考值，翻译后可能移动）
- `type`：`csharp` / `md` / `md-code` / `yaml` / `file`
- `source`：英文原文（完整单元）

### 3.2 在工作树里翻译

**所有编辑都在 `.tmp-zh-sync/` 内进行，不要改当前仓库根目录的英文文件。**

```bash
# 推荐先看涉及的文件
python3 - <<'PY'
import json
from collections import Counter
d=json.load(open('i18n/review/pending.json'))
print(Counter(x['file'] for x in d))
PY
```

翻译规则（硬性）：

1. **C# 只改注释**：`//`、`///`、`/* */`。绝不改代码、标识符、字符串字面量、异常消息、特性、预处理指令。
2. 保留 `///`、`//`、`/* */` 标记和缩进；可以调整换行。
3. 保留 XML 标签与属性：`<summary>`、`<param name="...">`、`<returns>`、`<remarks>`、`<see cref="..."/>`、`<paramref name="..."/>`、`<inheritdoc/>` 等。只翻译标签之间的自然语言。
4. 保留反引号代码、类型名、成员名、URL、数字、公式。
5. Markdown：翻译正文、标题、列表、表格、链接文字、图片 alt；保留结构、代码围栏、行内代码、URL、HTML 标签/属性、YAML 键名。
6. Markdown 代码块：只翻译 C# 注释，代码逐字不变；没有注释的代码块保持原样。
7. YAML：只翻译 `name:` 的值和 `#` 注释，不改键名、`href`、缩进。
8. HTML（`type=file`）：MVP 是整文件匹配。上游改了 HTML 会整文件进入 pending；只改可见文案/注释，保留 JS 逻辑、CSS、`class`、`id`、URL。改完需要重新提取 TM。
9. 中文使用全角标点；技术术语按 `i18n/glossary.yml`。
10. **不翻译**：许可证正文、SPDX、API 名称、类型名、URL、公式、纯 `<inheritdoc cref="..."/>`。这些保持英文是允许的。
11. 不要为了“看起来全中文”去翻译字符串字面量或异常消息，那会改变程序行为。

典型编辑方式：用 `read` 读取 `.tmp-zh-sync/<file>`，用 `edit` 精确替换英文注释/段落。

### 3.3 中途校验

```bash
python3 tools/i18n/verify.py \
  --base-ref upstream/main \
  --localized-dir .tmp-zh-sync \
  --pending /tmp/nonexistent-pending.json \
  --strict-english
```

必须通过：

- C# 剥离注释后代码与上游逐字一致
- 注释块数量不变
- Markdown 块/代码围栏数量不变，行内代码和 URL 不变
- YAML 非翻译行不变
- HTML 标签序列不变

`--strict-english` 只输出警告，用来发现漏译；警告中的 `<inheritdoc>`、公式、许可证可以忽略。

### 3.4 提交草稿并重新提取 TM

这是关键步骤：TM 必须从“翻译后的工作树”重新提取，而不是手工拼 JSON。

```bash
git -C .tmp-zh-sync checkout -b zh-draft
git -C .tmp-zh-sync add -A
git -C .tmp-zh-sync commit -m "i18n: draft translations"

python3 tools/i18n/extract_tm.py \
  --base upstream/main \
  --localized zh-draft \
  --out i18n/tm

git worktree remove --force .tmp-zh-sync
git branch -D zh-draft
```

检查 `i18n/tm/meta.json` 的 `counts` 是否符合预期。随后：

```bash
git add i18n/tm i18n/glossary.yml
git commit -m "i18n: sync translations with upstream <short-sha>"
git push origin l10n
```

### 3.5 用新 TM 重新生成

```bash
./tools/i18n/sync.sh --skip-build
```

这次应该看到 **0 条待译**，并且脚本会：

- 创建新的 `.tmp-zh-sync`
- 套用新 TM
- 复制 `i18n/`、`tools/i18n/`、`AGENTS.md`、`Makefile`、`.gitignore`、CI workflow
- 运行 `verify.py`
- 生成 `zh-next` 提交

确认：

```bash
cat i18n/review/pending.json     # 应该是 []
git -C .tmp-zh-sync log --oneline -2
git diff --stat l10n zh-next -- i18n tools AGENTS.md Makefile .gitignore .github/workflows/i18n.yml
```

最后一条应该没有输出（工具快照完全一致）。

### 3.6 构建与测试

```bash
mkdir -p .tmp-build/home .tmp-build/nuget .tmp-build/run .tmp-build/tmp
export HOME="$PWD/.tmp-build/home"
export DOTNET_CLI_HOME="$PWD/.tmp-build/home"
export NUGET_PACKAGES="$PWD/.tmp-build/nuget"
export XDG_RUNTIME_DIR="$PWD/.tmp-build/run"
export TMPDIR="$PWD/.tmp-build/tmp"

( cd .tmp-zh-sync && dotnet build src/JitterTests/JitterTests.csproj -c Release --nologo )
( cd .tmp-zh-sync && dotnet test  src/JitterTests/JitterTests.csproj -c Release --no-build --nologo )

rm -rf .tmp-build
```

预期：0 warning / 0 error；测试数量随上游变化（2.12.0 是 1749 项）。

### 3.7 推送 zh

`zh` 是生成分支，使用 `--force-with-lease`，不要用裸 `--force`。

```bash
git fetch origin zh
git -C .tmp-zh-sync push --force-with-lease origin zh-next:zh
```

推送后更新本地：

```bash
git fetch origin
git branch -f zh origin/zh
```

### 3.8 清理

```bash
git worktree remove --force .tmp-zh-sync 2>/dev/null || true
git branch -D zh-next 2>/dev/null || true
rm -rf .tmp-zh-sync i18n/review
find . -name __pycache__ -type d -prune -exec rm -rf {} +
git status --short
```

最终应干净。

---

## 4. 只更新 TM 后重新生成 zh

如果第 3.4 节已经提交了 TM，或者只修正了译文：

```bash
git switch l10n
./tools/i18n/sync.sh --skip-build
# 检查 pending 为 0
git fetch origin zh
git -C .tmp-zh-sync push --force-with-lease origin zh-next:zh
git worktree remove --force .tmp-zh-sync
git branch -D zh-next
rm -rf i18n/review
git fetch origin && git branch -f zh origin/zh
```

如果希望一次完成“套用 + 校验 + 构建 + 推送”，在 TM 已完整时可以用：

```bash
./tools/i18n/sync.sh --push
```

有 pending 时不要用 `--push`，先按第 3 节翻译。

---

## 5. 验证清单

提交前逐项确认：

- [ ] `i18n/review/pending.json` 是 `[]`
- [ ] `make i18n-test` 通过（从 `main + TM` 重建结果与 `zh` 逐字节一致）
- [ ] `make i18n-verify` 通过
- [ ] `dotnet build` 0 warning / 0 error
- [ ] `dotnet test` 全部通过
- [ ] `git diff --stat l10n zh-next -- i18n tools AGENTS.md Makefile .gitignore .github/workflows/i18n.yml` 为空
- [ ] 工作树、临时分支、`i18n/review`、`__pycache__` 已清理
- [ ] `l10n` 和 `zh` 都已推送

---

## 6. 常见问题

### Q: `sync.sh` 退出码 1，是不是失败？

不是。只要 `i18n/review/pending.json` 非空，脚本就退出 1，提示你翻译。翻译并重新提取 TM 后重跑即可。

### Q: 可以直接改 `zh` 里的中文吗？

不建议。`zh` 会被下一次同步覆盖。紧急修复也要同步改 `l10n` 的 TM，并重新生成 `zh`。

### Q: 为什么 pending 里有 `<inheritdoc/>`？

新重载可能只带 `<inheritdoc cref="..."/>`，没有自然语言。保持原样即可；重新提取后它会被记录为 `translation == source`，不再 pending。

### Q: 为什么 Markdown 代码块整块 pending？

MVP 按整块匹配。上游改了代码块任意一行，整块进入 pending。翻译其中的注释、保持代码不变即可。后续可改成注释级匹配。

### Q: HTML pending 怎么办？

`type=file` 是整文件匹配。用 `read` 读 `.tmp-zh-sync/<file>`，翻译可见文本/注释，保持 JS/CSS/结构不变。改完重新提取 TM。

### Q: 同一句英文在不同位置译文不同怎么办？

TM 查找顺序是 `(file, type, index)` → `(file, hash)` → 全局唯一 hash。索引不匹配时会回退。若出现歧义，`apply_tm.py` 会把它留在 pending；此时在具体文件里翻译，重新提取即可。

### Q: 构建时 NuGet 报 `Read-only file system`？

沙箱环境把 `~/.nuget` 设为只读。按 3.6 节设置 `HOME`、`NUGET_PACKAGES`、`XDG_RUNTIME_DIR`、`TMPDIR` 到工作区内。

### Q: `git log i18n` 报歧义？

工具分支叫 `l10n`，不是 `i18n`；`i18n/` 是目录。用 `git log l10n`。

### Q: 文档部署 workflow 失败？

fork 上的 `Build and deploy documentation` 已被禁用（缺少 Pages 写权限，且 CNAME 属于上游）。不要为了消除邮件重新启用；如需 fork 文档站，另建专用 workflow，加 `permissions: contents: write`、去掉上游 CNAME、启用 Pages。

---

## 7. 关键文件速查

```text
AGENTS.md                     本文件
i18n/README.md                人工维护说明
i18n/glossary.yml             术语表
i18n/allowlist.txt            允许保留英文的片段
i18n/tm/csharp.jsonl          C# 注释 TM
i18n/tm/md.jsonl              Markdown 块 TM
i18n/tm/yaml.jsonl            YAML 行 TM
i18n/tm/files.jsonl           HTML 整文件 TM
i18n/tm/meta.json             上次提取的基准提交
i18n/review/pending.*         待译清单（不提交）
tools/i18n/extract_tm.py      提取 TM
tools/i18n/apply_tm.py        套用 TM
tools/i18n/verify.py          校验
tools/i18n/sync.sh            一键同步
tools/i18n/tests/             端到端测试
```

当前版本与提交会随上游变化，请以以下命令为准：

```bash
git log --oneline -1 main
git log --oneline -1 l10n
git log --oneline -1 zh
cat i18n/tm/meta.json
```

写作本文档时，上游为 `2.12.0`，`zh` 已通过 471 个文件校验和 1749 项测试。
