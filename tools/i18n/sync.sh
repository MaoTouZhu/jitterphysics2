#!/usr/bin/env bash
# 从上游生成 zh-next：套用翻译记忆 -> 校验 -> 构建/测试 -> 可选推送。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
cd "$ROOT"

UPSTREAM_URL="${UPSTREAM_URL:-https://github.com/notgiven688/jitterphysics2.git}"
WORKTREE="$ROOT/.tmp-zh-sync"
PUSH=0
SKIP_BUILD=0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --push) PUSH=1; shift ;;
    --skip-build) SKIP_BUILD=1; shift ;;
    --upstream-url) UPSTREAM_URL="$2"; shift 2 ;;
    -h|--help)
      cat <<'EOF'
用法：tools/i18n/sync.sh [--push] [--skip-build] [--upstream-url URL]

  --push          校验与构建通过后，force-with-lease 推送到 origin/zh
  --skip-build    跳过 dotnet build / dotnet test
  --upstream-url  上游仓库地址（默认 notgiven688/jitterphysics2）
EOF
      exit 0
      ;;
    *) echo "未知参数：$1" >&2; exit 2 ;;
  esac
done

if ! git remote get-url upstream >/dev/null 2>&1; then
  echo "添加 upstream remote: $UPSTREAM_URL"
  git remote add upstream "$UPSTREAM_URL"
fi

git fetch upstream --prune
BASE="upstream/main"
BASE_SHA="$(git rev-parse "$BASE")"
BASE_SHORT="$(git rev-parse --short "$BASE")"

echo "上游基点：$BASE_SHORT"

if git worktree list --porcelain | grep -q "^worktree $WORKTREE$"; then
  git worktree remove --force "$WORKTREE"
fi
rm -rf "$WORKTREE"
git worktree add --detach "$WORKTREE" "$BASE"

echo "套用翻译记忆..."
set +e
python3 "$ROOT/tools/i18n/apply_tm.py" \
  --source-dir "$WORKTREE" \
  --tm "$ROOT/i18n/tm" \
  --pending-md "$ROOT/i18n/review/pending.md" \
  --pending-json "$ROOT/i18n/review/pending.json"
APPLY_RC=$?
set -e

if [[ $APPLY_RC -ne 0 ]]; then
  echo
  echo "存在待翻译项。请编辑 i18n/tm/ 或 i18n/review/pending.md，然后重跑。"
  echo "生成目录保留在：$WORKTREE"
  exit 1
fi

# 把工具与翻译资产带进生成的分支，方便在 zh 上直接维护。
# 必须在 apply 之后复制，避免 i18n.yml 被当成上游待译文件。
cp -a "$ROOT/i18n" "$WORKTREE/i18n"
rm -rf "$WORKTREE/i18n/review"
mkdir -p "$WORKTREE/tools"
cp -a "$ROOT/tools/i18n" "$WORKTREE/tools/i18n"
find "$WORKTREE/tools" -name __pycache__ -type d -prune -exec rm -rf {} +
if [[ -f "$ROOT/.gitignore" ]]; then cp -a "$ROOT/.gitignore" "$WORKTREE/.gitignore"; fi
if [[ -f "$ROOT/Makefile" ]]; then cp -a "$ROOT/Makefile" "$WORKTREE/Makefile"; fi
if [[ -f "$ROOT/.github/workflows/i18n.yml" ]]; then
  mkdir -p "$WORKTREE/.github/workflows"
  cp -a "$ROOT/.github/workflows/i18n.yml" "$WORKTREE/.github/workflows/i18n.yml"
fi

echo "校验..."
python3 "$ROOT/tools/i18n/verify.py" \
  --base-ref "$BASE" \
  --localized-dir "$WORKTREE" \
  --pending "$ROOT/i18n/review/pending.json"

if [[ $SKIP_BUILD -eq 0 ]]; then
  echo "构建与测试..."
  ( cd "$WORKTREE" && dotnet build src/Jitter2/Jitter2.csproj -f net10.0 -c Release --nologo )
  ( cd "$WORKTREE" && dotnet test src/JitterTests/JitterTests.csproj -c Release --nologo )
fi

git -C "$WORKTREE" checkout -B zh-next
git -C "$WORKTREE" add -A
if git -C "$WORKTREE" diff --cached --quiet; then
  echo "没有产生新的翻译改动，zh 已是最新。"
  exit 0
fi
git -C "$WORKTREE" commit -m "i18n: sync with upstream $BASE_SHORT"

echo
echo "已生成 zh-next：$WORKTREE"
echo "检查：git -C \"$WORKTREE\" show --stat HEAD"

if [[ $PUSH -eq 1 ]]; then
  git fetch origin zh
  git -C "$WORKTREE" push --force-with-lease origin zh-next:zh
  echo "已推送到 origin/zh"
fi
