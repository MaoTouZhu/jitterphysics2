#!/usr/bin/env python3
"""从 base（英文）与 localized（中文）两个 git ref 提取翻译记忆。"""

from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

from i18nlib import (
    REPO_ROOT,
    git_ls_tree,
    git_show,
    run_git,
    scan_cs_comments,
    sha256_text,
    split_md_blocks,
    split_yaml_units,
    write_jsonl,
)


def die(message: str) -> None:
    print(f"错误：{message}", file=sys.stderr)
    raise SystemExit(1)


def extract_cs(path: str, base: str, localized: str) -> list[dict]:
    source_units = scan_cs_comments(git_show(base, path))
    target_units = scan_cs_comments(git_show(localized, path))
    if len(source_units) != len(target_units):
        die(f"{path}: 注释块数量不一致 {len(source_units)} -> {len(target_units)}")
    records = []
    for index, (source, target) in enumerate(zip(source_units, target_units)):
        records.append({
            "type": "csharp",
            "file": path,
            "index": index,
            "source": source["text"],
            "translation": target["text"],
            "hash": sha256_text(source["text"]),
        })
    return records


def extract_md(path: str, base: str, localized: str) -> list[dict]:
    source_units = split_md_blocks(git_show(base, path))
    target_units = split_md_blocks(git_show(localized, path))
    if len(source_units) != len(target_units):
        die(f"{path}: Markdown 块数量不一致 {len(source_units)} -> {len(target_units)}")
    records = []
    for index, (source, target) in enumerate(zip(source_units, target_units)):
        kind = source["kind"]
        unit_type = "md-code" if kind == "code" else "md"
        records.append({
            "type": unit_type,
            "file": path,
            "index": index,
            "kind": kind,
            "lang": source["lang"],
            "source": source["text"],
            "translation": target["text"],
            "translation_has_nl": target["has_nl"],
            "hash": sha256_text(source["text"]),
        })
    return records


def extract_yaml(path: str, base: str, localized: str) -> list[dict]:
    source_units = split_yaml_units(git_show(base, path))
    target_units = split_yaml_units(git_show(localized, path))
    if len(source_units) != len(target_units):
        die(f"{path}: YAML 可译行数量不一致 {len(source_units)} -> {len(target_units)}")
    records = []
    for index, (source, target) in enumerate(zip(source_units, target_units)):
        records.append({
            "type": "yaml",
            "file": path,
            "index": index,
            "line": source["line"],
            "source": source["text"],
            "translation": target["text"],
            "translation_has_nl": target["has_nl"],
            "hash": sha256_text(source["text"]),
        })
    return records


def extract_file(path: str, base: str, localized: str) -> list[dict]:
    source = git_show(base, path)
    target = git_show(localized, path)
    return [{
        "type": "file",
        "file": path,
        "source": source,
        "translation": target,
        "hash": sha256_text(source),
    }]


def main() -> None:
    parser = argparse.ArgumentParser(description="提取 Jitter2 翻译记忆")
    parser.add_argument("--base", default="main", help="英文基准分支/提交")
    parser.add_argument("--localized", default="zh", help="中文分支/提交")
    parser.add_argument("--out", default="i18n/tm", help="输出目录")
    args = parser.parse_args()

    base = args.base
    localized = args.localized

    try:
        base_commit = run_git("rev-parse", base).decode().strip()
        localized_commit = run_git("rev-parse", localized).decode().strip()
    except Exception as exc:  # noqa: BLE001
        die(f"无法解析 git ref：{exc}")

    base_files = set(git_ls_tree(base))
    localized_files = set(git_ls_tree(localized))
    missing = sorted(base_files - localized_files)
    if missing:
        die(f"localized 分支缺少 {len(missing)} 个文件，例如：{missing[:5]}")

    buckets: dict[str, list[dict]] = {
        "csharp": [],
        "md": [],
        "yaml": [],
        "files": [],
    }
    counts: Counter[str] = Counter()

    for path in sorted(base_files):
        suffix = Path(path).suffix.lower()
        if suffix == ".cs":
            records = extract_cs(path, base, localized)
            buckets["csharp"].extend(records)
            counts["csharp"] += len(records)
            counts["csharp_files"] += 1
        elif suffix == ".md":
            records = extract_md(path, base, localized)
            buckets["md"].extend(records)
            counts["md"] += len(records)
            counts["md_files"] += 1
        elif suffix in {".yml", ".yaml"}:
            records = extract_yaml(path, base, localized)
            buckets["yaml"].extend(records)
            counts["yaml"] += len(records)
            counts["yaml_files"] += 1
        elif suffix in {".html", ".htm"}:
            records = extract_file(path, base, localized)
            buckets["files"].extend(records)
            counts["files"] += len(records)

    out_dir = (REPO_ROOT / args.out).resolve()
    write_jsonl(out_dir / "csharp.jsonl", buckets["csharp"])
    write_jsonl(out_dir / "md.jsonl", buckets["md"])
    write_jsonl(out_dir / "yaml.jsonl", buckets["yaml"])
    write_jsonl(out_dir / "files.jsonl", buckets["files"])

    meta = {
        "base_ref": base,
        "base_commit": base_commit,
        "localized_ref": localized,
        "localized_commit": localized_commit,
        "counts": dict(counts),
    }
    (out_dir / "meta.json").write_text(
        json.dumps(meta, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    print(f"已写入 {out_dir}")
    for key in ("csharp", "md", "yaml", "files"):
        print(f"  {key}: {counts[key]} 条")
    print(f"  base={base_commit[:12]} localized={localized_commit[:12]}")


if __name__ == "__main__":
    main()
