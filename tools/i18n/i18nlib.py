"""Jitter2 中文本地化工具的核心解析与替换逻辑。

设计原则：
- 只替换注释/文档块，代码与字符串字面量原样保留；
- 所有文本在内部统一使用 LF，读写时保留原文件的 BOM 与换行风格；
- C# 注释按“连续注释块”切分，包含行尾注释；
- Markdown 按空行分隔的块切分，代码围栏、front matter、style/script 单独识别。
"""

from __future__ import annotations

import hashlib
import json
import re
import subprocess
from pathlib import Path
from typing import Any, Iterable

REPO_ROOT = Path(__file__).resolve().parents[2]
CSHARP_LANGS = {"cs", "csharp", "c#"}
MD_TRANSLATABLE_KINDS = {"prose", "code"}


# ---------------------------------------------------------------------------
# 基础工具
# ---------------------------------------------------------------------------

def sha256_text(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


def run_git(*args: str, cwd: Path | None = None) -> bytes:
    return subprocess.check_output(["git", *args], cwd=cwd or REPO_ROOT)


def git_show(ref: str, path: str) -> str:
    return run_git("show", f"{ref}:{path}").decode("utf-8", errors="replace")


def git_ls_tree(ref: str) -> list[str]:
    out = run_git("ls-tree", "-r", "--name-only", ref).decode("utf-8")
    return [line for line in out.splitlines() if line]


def read_text_file(path: Path) -> tuple[str, bool, str]:
    """返回 (文本, 是否有 BOM, 换行风格)。文本统一为 LF。"""
    data = path.read_bytes()
    bom = data.startswith(b"\xef\xbb\xbf")
    if bom:
        data = data[3:]
    text = data.decode("utf-8", errors="replace")
    crlf = "\r\n" in text
    text = text.replace("\r\n", "\n").replace("\r", "\n")
    return text, bom, ("\r\n" if crlf else "\n")


def write_text_file(path: Path, text: str, bom: bool, newline: str) -> None:
    if newline == "\r\n":
        text = text.replace("\n", "\r\n")
    data = text.encode("utf-8")
    if bom:
        data = b"\xef\xbb\xbf" + data
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def load_jsonl(path: Path) -> list[dict[str, Any]]:
    if not path.exists():
        return []
    records: list[dict[str, Any]] = []
    with path.open(encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def write_jsonl(path: Path, records: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="\n") as fh:
        for record in records:
            fh.write(json.dumps(record, ensure_ascii=False, sort_keys=True))
            fh.write("\n")


# ---------------------------------------------------------------------------
# C# 注释解析
# ---------------------------------------------------------------------------

def _deindent(text: str, indent: str) -> str:
    out: list[str] = []
    for line in text.split("\n"):
        if line.startswith(indent):
            out.append(line[len(indent):])
        elif line.strip() == "":
            out.append("")
        else:
            out.append(line)
    return "\n".join(out)


def _advance_line_start(text: str, line_start: int, pos: int) -> int:
    k = text.rfind("\n", line_start, pos)
    return line_start if k == -1 else k + 1


def scan_cs_comments(text: str) -> list[dict[str, Any]]:
    """提取 C# 注释块。

    每个单元包含：
    - start/end: 替换区间（全行注释包含缩进；行尾注释从 // 开始）
    - marker: 注释标记起始位置
    - indent: 全行注释的缩进
    - text: 去掉公共缩进后的注释文本，保留注释标记
    """
    units: list[dict[str, Any]] = []
    i = 0
    n = len(text)
    line_start = 0

    def advance_to(pos: int) -> None:
        nonlocal line_start
        line_start = _advance_line_start(text, line_start, pos)

    while i < n:
        c = text[i]

        if text.startswith("//", i):
            j = text.find("\n", i)
            if j == -1:
                j = n
            end = j
            if end > i and text[end - 1] == "\r":
                end -= 1
            prefix = text[line_start:i]
            full = prefix.strip() == ""
            units.append({
                "marker": i,
                "start": line_start if full else i,
                "end": end,
                "kind": "line",
                "full": full,
                "indent": prefix if full else "",
            })
            i = j
            continue

        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            j = n if j == -1 else j + 2
            prefix = text[line_start:i]
            full = prefix.strip() == ""
            units.append({
                "marker": i,
                "start": line_start if full else i,
                "end": j,
                "kind": "block",
                "full": full,
                "indent": prefix if full else "",
            })
            advance_to(j)
            i = j
            continue

        # 字符串字面量：跳过其中的 // 与 /* */
        if text.startswith('$@"', i) or text.startswith('@$"', i):
            i += 3
            while i < n:
                if text[i] == '"':
                    if i + 1 < n and text[i + 1] == '"':
                        i += 2
                        continue
                    i += 1
                    break
                i += 1
            advance_to(i)
            continue

        if text.startswith('@"', i):
            i += 2
            while i < n:
                if text[i] == '"':
                    if i + 1 < n and text[i + 1] == '"':
                        i += 2
                        continue
                    i += 1
                    break
                i += 1
            advance_to(i)
            continue

        if text.startswith('"""', i) or text.startswith('$"""', i):
            prefix_len = 1 if text.startswith('$"""', i) else 0
            q = 0
            while i + prefix_len + q < n and text[i + prefix_len + q] == '"':
                q += 1
            start = i + prefix_len + q
            close = '"' * q
            j = text.find(close, start)
            i = n if j == -1 else j + q
            advance_to(i)
            continue

        if c == '"':
            i += 1
            while i < n:
                if text[i] == "\\":
                    i += 2
                    continue
                if text[i] == '"':
                    i += 1
                    break
                i += 1
            advance_to(i)
            continue

        if c == "'":
            i += 1
            while i < n:
                if text[i] == "\\":
                    i += 2
                    continue
                if text[i] == "'":
                    i += 1
                    break
                i += 1
            advance_to(i)
            continue

        if c == "\n":
            line_start = i + 1
        i += 1

    # 合并连续的全行 // 注释
    merged: list[dict[str, Any]] = []
    k = 0
    while k < len(units):
        unit = units[k]
        if unit["kind"] == "line" and unit["full"]:
            group = [unit]
            end = unit["end"]
            k += 1
            while k < len(units):
                nxt = units[k]
                if (
                    nxt["kind"] == "line"
                    and nxt["full"]
                    and re.fullmatch(r"\n[ \t]*", text[end:nxt["marker"]])
                ):
                    group.append(nxt)
                    end = nxt["end"]
                    k += 1
                    continue
                break
            indent = group[0]["indent"]
            raw = text[group[0]["marker"]:end]
            merged.append({
                "start": group[0]["start"],
                "end": end,
                "kind": "group",
                "indent": indent,
                "text": _deindent(raw, indent),
            })
        else:
            raw = text[unit["marker"]:unit["end"]]
            if unit["full"]:
                raw = _deindent(raw, unit["indent"])
            merged.append({
                "start": unit["start"],
                "end": unit["end"],
                "kind": unit["kind"],
                "indent": unit["indent"],
                "text": raw,
            })
            k += 1
    return merged


def render_cs_translation(unit: dict[str, Any], translation: str) -> str:
    indent = unit["indent"]
    lines = translation.split("\n")
    return "\n".join((indent + line if line.strip() else "") for line in lines)


def apply_replacements(text: str, replacements: list[tuple[int, int, str]]) -> str:
    """按区间替换文本；replacements 为 (start, end, replacement)。"""
    parts: list[str] = []
    pos = 0
    for start, end, replacement in sorted(replacements):
        if start < pos:
            raise ValueError("overlapping replacements")
        parts.append(text[pos:start])
        parts.append(replacement)
        pos = end
    parts.append(text[pos:])
    return "".join(parts)


def strip_cs_comments(text: str) -> str:
    """移除 C# 注释，保留代码与字符串字面量内容。"""
    out: list[str] = []
    i = 0
    n = len(text)
    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j == -1 else j
            continue
        if text.startswith("/*", i):
            j = text.find("*/", i + 2)
            i = n if j == -1 else j + 2
            continue
        if text.startswith('$@"', i) or text.startswith('@$"', i):
            out.append(text[i:i + 3])
            i += 3
            while i < n:
                if text[i] == '"':
                    if i + 1 < n and text[i + 1] == '"':
                        out.append('""')
                        i += 2
                        continue
                    out.append('"')
                    i += 1
                    break
                out.append(text[i])
                i += 1
            continue
        if text.startswith('@"', i):
            out.append('@"')
            i += 2
            while i < n:
                if text[i] == '"':
                    if i + 1 < n and text[i + 1] == '"':
                        out.append('""')
                        i += 2
                        continue
                    out.append('"')
                    i += 1
                    break
                out.append(text[i])
                i += 1
            continue
        if text.startswith('"""', i) or text.startswith('$"""', i):
            prefix = '$"""' if text.startswith('$"""', i) else '"""'
            out.append(prefix)
            i += len(prefix)
            while i < n:
                if text[i] == '"':
                    q = 0
                    while i + q < n and text[i + q] == '"':
                        q += 1
                    out.append('"' * q)
                    i += q
                    if q >= 3:
                        break
                    continue
                out.append(text[i])
                i += 1
            continue
        if text[i] == '"':
            out.append('"')
            i += 1
            while i < n:
                if text[i] == "\\":
                    out.append(text[i:i + 2])
                    i += 2
                    continue
                out.append(text[i])
                if text[i] == '"':
                    i += 1
                    break
                i += 1
            continue
        if text[i] == "'":
            out.append("'")
            i += 1
            while i < n:
                if text[i] == "\\":
                    out.append(text[i:i + 2])
                    i += 2
                    continue
                out.append(text[i])
                if text[i] == "'":
                    i += 1
                    break
                i += 1
            continue
        out.append(text[i])
        i += 1
    return "".join(out)


def normalize_code(text: str) -> str:
    return re.sub(r"\s+", "", text)


# ---------------------------------------------------------------------------
# Markdown 解析
# ---------------------------------------------------------------------------

_FENCE_RE = re.compile(r"^(\s*)(`{3,}|~{3,})(.*)$")


def split_md_blocks(text: str) -> list[dict[str, Any]]:
    """按空行切分 Markdown，识别 front matter、代码围栏和 style/script。"""
    lines = text.splitlines(keepends=True)
    n = len(lines)
    offsets = [0]
    for line in lines:
        offsets.append(offsets[-1] + len(line))

    units: list[dict[str, Any]] = []
    i = 0
    while i < n:
        if lines[i].strip() == "":
            i += 1
            continue

        start = i
        s = lines[i].strip()

        if i == 0 and s == "---":
            i += 1
            while i < n and lines[i].strip() != "---":
                i += 1
            i = min(i + 1, n)
            kind, lang = "frontmatter", ""
        elif _FENCE_RE.match(lines[i]):
            m = _FENCE_RE.match(lines[i])
            fence_char = m.group(2)[0]
            fence_len = len(m.group(2))
            lang = m.group(3).strip().split()[0].lower() if m.group(3).strip() else ""
            i += 1
            close_re = re.compile(
                r"^\s*" + re.escape(fence_char) + r"{" + str(fence_len) + r",}\s*$"
            )
            while i < n:
                if close_re.match(lines[i]):
                    i += 1
                    break
                i += 1
            kind = "code"
        elif s.startswith("<style") or s.startswith("<script"):
            tag = "style" if s.startswith("<style") else "script"
            close = f"</{tag}>"
            i += 1
            while i < n:
                if close in lines[i]:
                    i += 1
                    break
                i += 1
            kind, lang = tag, ""
        else:
            i += 1
            while i < n:
                line = lines[i]
                if line.strip() == "":
                    break
                if _FENCE_RE.match(line):
                    break
                if line.strip().startswith(("<style", "<script")):
                    break
                i += 1
            kind, lang = "prose", ""

        start_off = offsets[start]
        end_off = offsets[i]
        has_nl = text[end_off - 1:end_off] == "\n"
        if has_nl:
            end_off -= 1
        units.append({
            "kind": kind,
            "lang": lang,
            "start": start_off,
            "end": end_off,
            "has_nl": has_nl,
            "text": text[start_off:end_off],
            "start_line": start + 1,
            "end_line": i,
        })
    return units


def apply_md_replacements(
    text: str,
    units: list[dict[str, Any]],
    translations: list[str | None],
    newline_flags: list[bool | None] | None = None,
) -> str:
    reps: list[tuple[int, int, str]] = []
    for index, (unit, translation) in enumerate(zip(units, translations)):
        if translation is None:
            continue
        has_nl = unit["has_nl"]
        if newline_flags is not None and newline_flags[index] is not None:
            has_nl = bool(newline_flags[index])
        repl = translation + ("\n" if has_nl else "")
        reps.append((unit["start"], unit["end"] + (1 if has_nl else 0), repl))
    return apply_replacements(text, reps)


# ---------------------------------------------------------------------------
# YAML / 脚本行级解析
# ---------------------------------------------------------------------------

_YAML_UNIT_RE = re.compile(r"^(\s*#|\s*(?:-\s*)?name:\s*)")


def split_yaml_units(text: str) -> list[dict[str, Any]]:
    lines = text.splitlines(keepends=True)
    offsets = [0]
    for line in lines:
        offsets.append(offsets[-1] + len(line))

    units: list[dict[str, Any]] = []
    for idx, line in enumerate(lines):
        raw = line[:-1] if line.endswith("\n") else line
        inline_comment = bool(re.search(r"\S\s+#", raw)) and not raw.lstrip().startswith("#")
        if _YAML_UNIT_RE.match(raw) or inline_comment:
            has_nl = line.endswith("\n")
            start = offsets[idx]
            end = offsets[idx + 1] - (1 if has_nl else 0)
            units.append({
                "line": idx + 1,
                "start": start,
                "end": end,
                "has_nl": has_nl,
                "text": raw,
            })
    return units


def apply_yaml_replacements(
    text: str,
    units: list[dict[str, Any]],
    translations: list[str | None],
    newline_flags: list[bool | None] | None = None,
) -> str:
    reps: list[tuple[int, int, str]] = []
    for index, (unit, translation) in enumerate(zip(units, translations)):
        if translation is None:
            continue
        has_nl = unit["has_nl"]
        if newline_flags is not None and newline_flags[index] is not None:
            has_nl = bool(newline_flags[index])
        repl = translation + ("\n" if has_nl else "")
        reps.append((unit["start"], unit["end"] + (1 if has_nl else 0), repl))
    return apply_replacements(text, reps)


# ---------------------------------------------------------------------------
# 翻译记忆
# ---------------------------------------------------------------------------

class TranslationMemory:
    def __init__(self, records: Iterable[dict[str, Any]]):
        self.by_hash: dict[str, list[dict[str, Any]]] = {}
        self.by_file_hash: dict[tuple[str, str], dict[str, Any]] = {}
        self.by_file_index: dict[tuple[str, str, int], dict[str, Any]] = {}
        for record in records:
            self.by_hash.setdefault(record["hash"], []).append(record)
            self.by_file_hash[(record["file"], record["hash"])] = record
            if "index" in record:
                self.by_file_index[(record["file"], record["type"], record["index"])] = record

    def find(self, file: str, unit_type: str, source: str, index: int | None = None) -> dict[str, Any] | None:
        h = sha256_text(source)
        if index is not None:
            exact = self.by_file_index.get((file, unit_type, index))
            if exact and exact["hash"] == h:
                return exact
        direct = self.by_file_hash.get((file, h))
        if direct and direct["type"] == unit_type:
            return direct
        candidates = [r for r in self.by_hash.get(h, []) if r["type"] == unit_type]
        translations = {r["translation"] for r in candidates}
        if len(translations) == 1:
            return candidates[0]
        return None

    def lookup(self, file: str, unit_type: str, source: str, index: int | None = None) -> str | None:
        record = self.find(file, unit_type, source, index)
        return None if record is None else record["translation"]

    def has_source(self, unit_type: str, source: str) -> bool:
        h = sha256_text(source)
        return any(r["type"] == unit_type for r in self.by_hash.get(h, []))
