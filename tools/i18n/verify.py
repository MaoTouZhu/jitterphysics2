#!/usr/bin/env python3
"""校验汉化结果：代码不变、文档结构不变、无遗漏待译项。"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections import Counter
from pathlib import Path

from i18nlib import (
    REPO_ROOT,
    git_ls_tree,
    git_show,
    normalize_code,
    read_text_file,
    scan_cs_comments,
    split_md_blocks,
    split_yaml_units,
    strip_cs_comments,
)

CJK_RE = re.compile(r"[\u4e00-\u9fff]")
WORD_RE = re.compile(r"[A-Za-z][A-Za-z'\-]{2,}")
TAG_RE = re.compile(r"</?([A-Za-z][\w:.-]*)")
INLINE_CODE_RE = re.compile(r"`([^`\n]+)`")
URL_RE = re.compile(r"https?://[^\s)\]>]+?")
COMMENT_LANGS = {"cs", "csharp", "c#"}
HASH_COMMENT_LANGS = {"text", "sh", "bash", "shell", "console", "yaml", "yml"}


class Report:
    def __init__(self) -> None:
        self.errors: list[str] = []
        self.warnings: list[str] = []

    def error(self, message: str) -> None:
        self.errors.append(message)

    def warn(self, message: str) -> None:
        self.warnings.append(message)


def load_allowlist(path: Path) -> list[str]:
    if not path.exists():
        return []
    return [
        line.strip()
        for line in path.read_text(encoding="utf-8").splitlines()
        if line.strip() and not line.strip().startswith("#")
    ]


def is_allowed(text: str, allowlist: list[str]) -> bool:
    low = text.lower()
    return any(item.lower() in low for item in allowlist)


def strip_bom(text: str) -> str:
    return text[1:] if text.startswith("\ufeff") else text


def localized_reader(ref: str | None, root: Path | None):
    if ref:
        return lambda path: git_show(ref, path)
    assert root is not None

    def read(path: str) -> str:
        return read_text_file(root / path)[0]

    return read


def check_cs(path: str, base: str, localized: str, report: Report) -> None:
    base_units = scan_cs_comments(base)
    loc_units = scan_cs_comments(localized)
    if len(base_units) != len(loc_units):
        report.error(f"{path}: 注释块数量变化 {len(base_units)} -> {len(loc_units)}")

    base_code = normalize_code(strip_cs_comments(base))
    loc_code = normalize_code(strip_cs_comments(localized))
    if base_code != loc_code:
        k = 0
        while k < min(len(base_code), len(loc_code)) and base_code[k] == loc_code[k]:
            k += 1
        report.error(
            f"{path}: 代码发生变化 at {k}: "
            f"base={base_code[k:k + 60]!r} localized={loc_code[k:k + 60]!r}"
        )

    base_tags = Counter(TAG_RE.findall(base))
    loc_tags = Counter(TAG_RE.findall(localized))
    if base_tags != loc_tags:
        report.error(f"{path}: XML 文档标签数量发生变化")


def strip_hash_comments(text: str) -> str:
    out = []
    for line in text.splitlines():
        if line.lstrip().startswith("#"):
            continue
        if " #" in line:
            line = line.split(" #", 1)[0]
        out.append(line.rstrip())
    return "\n".join(out)


def check_md(path: str, base: str, localized: str, report: Report) -> None:
    base_all = split_md_blocks(base)
    loc_all = split_md_blocks(localized)
    if len(base_all) != len(loc_all):
        report.error(f"{path}: Markdown 块数量变化 {len(base_all)} -> {len(loc_all)}")
    base_blocks = [u for u in base_all if u["kind"] == "code"]
    loc_blocks = [u for u in loc_all if u["kind"] == "code"]
    if len(base_blocks) != len(loc_blocks):
        report.error(f"{path}: 代码围栏数量变化 {len(base_blocks)} -> {len(loc_blocks)}")
    else:
        for index, (a, b) in enumerate(zip(base_blocks, loc_blocks)):
            if a["lang"] != b["lang"]:
                report.error(f"{path}: 代码围栏 {index} 语言变化 {a['lang']!r} -> {b['lang']!r}")
                continue
            if a["lang"] in COMMENT_LANGS:
                same = normalize_code(strip_cs_comments(a["text"])) == normalize_code(strip_cs_comments(b["text"]))
            elif a["lang"] in HASH_COMMENT_LANGS:
                same = strip_hash_comments(a["text"]).strip() == strip_hash_comments(b["text"]).strip()
            else:
                same = a["text"] == b["text"]
            if not same:
                report.error(f"{path}: 代码围栏 {index} ({a['lang']}) 代码发生变化")

    if sorted(INLINE_CODE_RE.findall(base)) != sorted(INLINE_CODE_RE.findall(localized)):
        report.error(f"{path}: 行内代码片段发生变化")
    if sorted(URL_RE.findall(base)) != sorted(URL_RE.findall(localized)):
        report.error(f"{path}: URL 发生变化")


def yaml_is_unit(raw: str) -> bool:
    if re.match(r"^\s*#", raw) or re.match(r"^\s*(?:-\s*)?name:\s*", raw):
        return True
    return bool(re.search(r"\S\s+#", raw)) and not raw.lstrip().startswith("#")


def check_yaml(path: str, base: str, localized: str, report: Report) -> None:
    base_lines = [line for line in base.splitlines() if not yaml_is_unit(line)]
    loc_lines = [line for line in localized.splitlines() if not yaml_is_unit(line)]
    if base_lines != loc_lines:
        report.error(f"{path}: YAML 非翻译行发生变化")


def check_html(path: str, base: str, localized: str, report: Report) -> None:
    if TAG_RE.findall(base) != TAG_RE.findall(localized):
        report.error(f"{path}: HTML 标签序列发生变化")


def check_english(path: str, text: str, allowlist: list[str], report: Report) -> None:
    suffix = Path(path).suffix.lower()
    if suffix == ".cs":
        for unit in scan_cs_comments(text):
            body = unit["text"]
            if CJK_RE.search(body) or len(WORD_RE.findall(body)) < 3:
                continue
            if is_allowed(body, allowlist):
                continue
            line = text.count("\n", 0, unit["start"]) + 1
            report.warn(f"{path}:{line}: 可能残留英文注释：{body.splitlines()[0][:100]}")
    elif suffix == ".md":
        for unit in split_md_blocks(text):
            if unit["kind"] != "prose":
                continue
            body = unit["text"]
            if CJK_RE.search(body) or len(WORD_RE.findall(body)) < 6:
                continue
            if is_allowed(body, allowlist):
                continue
            line = text.count("\n", 0, unit["start"]) + 1
            report.warn(f"{path}:{line}: 可能残留英文段落：{body.splitlines()[0][:100]}")


def main() -> None:
    parser = argparse.ArgumentParser(description="校验 Jitter2 中文本地化结果")
    parser.add_argument("--base-ref", default="main")
    parser.add_argument("--localized-ref", default=None)
    parser.add_argument("--localized-dir", default=".")
    parser.add_argument("--pending", default="i18n/review/pending.json")
    parser.add_argument("--allowlist", default="i18n/allowlist.txt")
    parser.add_argument("--strict-english", action="store_true")
    args = parser.parse_args()

    report = Report()
    root = (REPO_ROOT / args.localized_dir).resolve()
    reader = localized_reader(args.localized_ref, root)

    files = git_ls_tree(args.base_ref)
    for path in files:
        suffix = Path(path).suffix.lower()
        if suffix not in {".cs", ".md", ".yml", ".yaml", ".html", ".htm"}:
            continue
        try:
            base = strip_bom(git_show(args.base_ref, path))
            localized = strip_bom(reader(path))
        except FileNotFoundError:
            report.error(f"{path}: localized 中缺少文件")
            continue
        if suffix == ".cs":
            check_cs(path, base, localized, report)
        elif suffix == ".md":
            check_md(path, base, localized, report)
        elif suffix in {".yml", ".yaml"}:
            check_yaml(path, base, localized, report)
        elif suffix in {".html", ".htm"}:
            check_html(path, base, localized, report)

        if args.strict_english:
            check_english(path, localized, load_allowlist(REPO_ROOT / args.allowlist), report)

    pending_path = REPO_ROOT / args.pending
    if pending_path.exists():
        try:
            pending = json.loads(pending_path.read_text(encoding="utf-8"))
        except json.JSONDecodeError:
            pending = []
        if pending:
            report.error(f"仍有 {len(pending)} 条待翻译项：{pending_path}")

    for warning in report.warnings:
        print(f"警告：{warning}")
    for error in report.errors:
        print(f"错误：{error}")

    if report.errors:
        print(f"\n校验失败：{len(report.errors)} 个错误，{len(report.warnings)} 个警告")
        raise SystemExit(1)
    print(f"校验通过：{len(files)} 个文件，{len(report.warnings)} 个警告")


if __name__ == "__main__":
    main()
