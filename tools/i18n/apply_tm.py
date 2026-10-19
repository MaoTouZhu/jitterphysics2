#!/usr/bin/env python3
"""把翻译记忆套用到一棵英文源码树。

默认原地修改 --source-dir。未命中的注释/文档会保留英文并写入待译清单。
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from i18nlib import (
    REPO_ROOT,
    TranslationMemory,
    apply_md_replacements,
    apply_replacements,
    apply_yaml_replacements,
    load_jsonl,
    read_text_file,
    render_cs_translation,
    scan_cs_comments,
    split_md_blocks,
    split_yaml_units,
    write_text_file,
)

SKIP_DIRS = {".git", "bin", "obj", ".tmp-tm", "i18n", "tools"}
SKIP_FILES = {".github/workflows/i18n.yml"}
HTML_SUFFIXES = {".html", ".htm"}


def load_memory(tm_dir: Path) -> TranslationMemory:
    records = []
    for name in ("csharp.jsonl", "md.jsonl", "yaml.jsonl", "files.jsonl"):
        records.extend(load_jsonl(tm_dir / name))
    return TranslationMemory(records)


def line_of(text: str, offset: int) -> int:
    return text.count("\n", 0, offset) + 1


def compact(source: str, limit: int = 100) -> str:
    one = " ".join(source.split())
    return one if len(one) <= limit else one[:limit] + "…"


class Pending:
    def __init__(self) -> None:
        self.items: list[dict] = []

    def add(self, file: str, line: int, unit_type: str, source: str) -> None:
        self.items.append({
            "file": file,
            "line": line,
            "type": unit_type,
            "source": source,
        })

    def write(self, md_path: Path, json_path: Path, source_dir: Path) -> None:
        md_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.parent.mkdir(parents=True, exist_ok=True)
        json_path.write_text(
            json.dumps(self.items, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

        lines = [
            "# 待翻译清单",
            "",
            f"共 {len(self.items)} 条。请更新 `i18n/tm/` 后重新运行 `apply_tm.py`。",
            "",
        ]
        current = None
        for item in self.items:
            if item["file"] != current:
                current = item["file"]
                lines.extend(["", f"## `{current}`", ""])
            lines.append(
                f"- 第 {item['line']} 行 · `{item['type']}`：{compact(item['source'])}"
            )
        md_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def process_file(
    path: Path,
    rel: str,
    memory: TranslationMemory,
    pending: Pending,
    dry_run: bool,
) -> bool:
    suffix = path.suffix.lower()
    if suffix == ".cs":
        text, bom, newline = read_text_file(path)
        units = scan_cs_comments(text)
        replacements = []
        for index, unit in enumerate(units):
            translation = memory.lookup(rel, "csharp", unit["text"], index=index)
            if translation is None:
                pending.add(rel, line_of(text, unit["start"]), "csharp", unit["text"])
                continue
            replacements.append(
                (unit["start"], unit["end"], render_cs_translation(unit, translation))
            )
        if not replacements:
            return False
        new_text = apply_replacements(text, replacements)
        if new_text == text:
            return False
        if not dry_run:
            write_text_file(path, new_text, bom, newline)
        return True

    if suffix == ".md":
        text, bom, newline = read_text_file(path)
        units = split_md_blocks(text)
        translations: list[str | None] = []
        flags: list[bool | None] = []
        for index, unit in enumerate(units):
            unit_type = "md-code" if unit["kind"] == "code" else "md"
            record = memory.find(rel, unit_type, unit["text"], index=index)
            if record is None:
                pending.add(rel, unit["start_line"], unit_type, unit["text"])
                translations.append(None)
                flags.append(None)
            else:
                translations.append(record["translation"])
                flags.append(record.get("translation_has_nl", unit["has_nl"]))
        new_text = apply_md_replacements(text, units, translations, flags)
        if new_text == text:
            return False
        if not dry_run:
            write_text_file(path, new_text, bom, newline)
        return True

    if suffix in {".yml", ".yaml"}:
        text, bom, newline = read_text_file(path)
        units = split_yaml_units(text)
        translations: list[str | None] = []
        flags: list[bool | None] = []
        for index, unit in enumerate(units):
            record = memory.find(rel, "yaml", unit["text"], index=index)
            if record is None:
                pending.add(rel, unit["line"], "yaml", unit["text"])
                translations.append(None)
                flags.append(None)
            else:
                translations.append(record["translation"])
                flags.append(record.get("translation_has_nl", unit["has_nl"]))
        new_text = apply_yaml_replacements(text, units, translations, flags)
        if new_text == text:
            return False
        if not dry_run:
            write_text_file(path, new_text, bom, newline)
        return True

    if suffix in HTML_SUFFIXES:
        text, bom, newline = read_text_file(path)
        translation = memory.lookup(rel, "file", text)
        if translation is None:
            pending.add(rel, 1, "file", text)
            return False
        if translation == text:
            return False
        if not dry_run:
            write_text_file(path, translation, bom, newline)
        return True

    return False


def iter_source_files(root: Path):
    for path in sorted(root.rglob("*")):
        if not path.is_file():
            continue
        rel_parts = path.relative_to(root).parts
        if any(part in SKIP_DIRS for part in rel_parts):
            continue
        rel = path.relative_to(root).as_posix()
        if rel in SKIP_FILES:
            continue
        if path.suffix.lower() not in {".cs", ".md", ".yml", ".yaml", ".html", ".htm"}:
            continue
        yield path, rel


def main() -> None:
    parser = argparse.ArgumentParser(description="套用 Jitter2 翻译记忆")
    parser.add_argument("--source-dir", required=True, help="英文源码树目录")
    parser.add_argument("--tm", default="i18n/tm", help="翻译记忆目录")
    parser.add_argument("--pending-md", default="i18n/review/pending.md")
    parser.add_argument("--pending-json", default="i18n/review/pending.json")
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-pending", action="store_true", help="有待译项时仍返回 0")
    args = parser.parse_args()

    source_dir = Path(args.source_dir)
    tm_dir = REPO_ROOT / args.tm
    if not source_dir.is_dir():
        print(f"错误：源码目录不存在：{source_dir}", file=sys.stderr)
        raise SystemExit(1)
    if not tm_dir.is_dir():
        print(f"错误：TM 目录不存在：{tm_dir}", file=sys.stderr)
        raise SystemExit(1)

    memory = load_memory(tm_dir)
    pending = Pending()
    changed = 0
    for path, rel in iter_source_files(source_dir):
        if process_file(path, rel, memory, pending, args.dry_run):
            changed += 1

    if not args.dry_run:
        pending.write(
            REPO_ROOT / args.pending_md,
            REPO_ROOT / args.pending_json,
            source_dir,
        )

    print(f"已处理 {changed} 个文件，待翻译 {len(pending.items)} 条")
    if pending.items:
        print(f"待译清单：{REPO_ROOT / args.pending_md}")
        if not args.allow_pending:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
