# Jitter2 中文本地化

这个目录是 `zh` 分支的“源代码”。译文保存在翻译记忆（TM）里，`zh` 由脚本从上游代码重新生成。

> 面向 LLM / 自动化代理的完整操作手册见根目录 [`AGENTS.md`](../AGENTS.md)。

## 分支模型

- `main`：镜像上游英文代码，不提交汉化改动
- `l10n`：术语表、TM、工具和 CI（本分支）
- `zh`：生成结果；由 `l10n` 分支的 `sync.sh` 生成，包含一份 `i18n/` 和 `tools/i18n/` 快照

`zh` 是生成物。**不要直接改 `zh` 里的译文**，否则下次同步会丢失。要改译文，请改 `i18n/tm/*.jsonl`，然后重新生成。

## 目录

```text
i18n/
  glossary.yml       术语表
  allowlist.txt      允许保留英文的片段
  tm/csharp.jsonl    C# 注释翻译记忆
  tm/md.jsonl        Markdown 块翻译记忆
  tm/yaml.jsonl      YAML 名称/注释翻译记忆
  tm/files.jsonl     HTML 整文件翻译记忆
  tm/meta.json       提取时的基准提交信息
  review/            最近一次同步的待译清单（不提交到 zh）
tools/i18n/
  i18nlib.py         解析与替换核心
  extract_tm.py      从 main + zh 提取 TM
  apply_tm.py        把 TM 套用到新上游
  verify.py          校验代码/文档结构
  sync.sh            拉上游并生成 zh-next
  tests/             端到端回归测试
```

## 常用命令

```bash
# 1. 修改译文后，重新提取 TM（以当前 zh 为准）
make i18n-extract

# 2. 校验当前 zh 与 main 的一致性
make i18n-verify

# 3. 跑端到端测试：从 main + TM 重建，结果必须与 zh 逐字节一致
make i18n-test

# 4. 同步上游：生成 zh-next，校验并构建/测试
make i18n-sync

# 5. 同步并直接推送（force-with-lease）
tools/i18n/sync.sh --push
```

`sync.sh` 需要本地有 `upstream` remote；没有会自动添加。生成目录 `.tmp-zh-sync` 会保留以便检查，成功后可手动删除。

## 翻译记忆格式

C# 注释记录：

```json
{"type":"csharp","file":"src/Jitter2/World.cs","index":42,
 "source":"/// <summary>Advances the simulation.</summary>",
 "translation":"/// <summary>推进模拟。</summary>",
 "hash":"..."}
```

- `source`：英文注释块，已去掉公共缩进
- `translation`：中文注释块，标记和 XML 标签保持不变
- `hash`：`source` 的 SHA-256，用于快速匹配
- `file` + `index`：精确重建时优先使用；上游插入/删除注释后回退到 `hash` 匹配

Markdown 记录使用 `md`（正文/标题/HTML 块/ front matter）或 `md-code`（代码围栏整块）。YAML 只记录 `name:` 值、`#` 注释和行内 `#` 注释。HTML 因为包含较多脚本逻辑，MVP 采用整文件匹配：文件未变则复用译文；上游改动则进入待译清单。

## 同步流程

1. `git fetch upstream`
2. 以 `upstream/main` 创建临时工作树
3. 套用 TM：完全匹配的单元自动替换；新增/修改的单元保留英文并写入 `i18n/review/pending.md`
4. 人工翻译 pending 中的条目，更新 `i18n/tm/`
5. `verify.py`：剥离注释后代码必须与上游逐字一致；Markdown 代码块、行内代码、URL、YAML 结构、HTML 标签序列必须不变
6. 构建 `Jitter2` 并运行 `JitterTests`
7. 生成 `zh-next` 提交；确认后 `--push`（`--force-with-lease`）

## 策略

- 只翻译注释和文档，不翻译代码、标识符、字符串字面量、异常消息、许可证文本、URL、API 名称和公式
- 许可证头、第三方许可证正文保持英文；`allowlist.txt` 记录这些例外
- 中文正文使用全角标点；XML 标签、`<see cref>`、`<paramref>`、反引号代码保持原样
- 测试注释、Demo 注释也维护翻译；如果将来想缩小范围，可以在 `extract_tm.py` 中按路径过滤
- `zh` 使用 `--force-with-lease` 推送；如果有其他人在 `zh` 上提交，先将其改动折算进 TM 再重新生成

## 已知限制（MVP）

- Markdown 代码围栏按整块匹配：上游改动了代码块里的任何一行，整个块会进入待译清单。后续可以把 C# 代码块拆成注释级单元。
- HTML 采用整文件匹配：上游改动 HTML 后需要人工处理。
- 尚未接入机器翻译；新条目先保留英文，避免未经校对的内容进入 `zh`。
- 定时检查上游的 GitHub Actions 任务暂未启用；当前通过 `sync.sh` 手动触发。
