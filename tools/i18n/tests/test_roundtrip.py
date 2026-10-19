"""端到端测试：从 main + TM 重建出与 zh 完全一致的文件。"""

from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
TOOLS = ROOT / "tools" / "i18n"


def git(*args: str) -> str:
    return subprocess.check_output(["git", *args], cwd=ROOT, text=True)


def ref_exists(ref: str) -> bool:
    return subprocess.run(
        ["git", "rev-parse", "--verify", "--quiet", ref],
        cwd=ROOT,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    ).returncode == 0


def pick_ref(name: str) -> str | None:
    env = os.environ.get(f"I18N_{name.upper()}_REF")
    if env:
        return env
    for candidate in (name, f"origin/{name}"):
        if ref_exists(candidate):
            return candidate
    return None


class RoundTripTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.base_ref = pick_ref("main")
        cls.zh_ref = pick_ref("zh")
        if not cls.base_ref or not cls.zh_ref:
            raise unittest.SkipTest("需要 main/zh（或 origin/main、origin/zh）git ref")
        cls.tmp = tempfile.TemporaryDirectory(prefix="jitter2-i18n-")
        cls.tmpdir = Path(cls.tmp.name)
        cls.source = cls.tmpdir / "source"
        cls.tm = cls.tmpdir / "tm"
        cls.source.mkdir()
        archive = subprocess.Popen(
            ["git", "archive", cls.base_ref], cwd=ROOT, stdout=subprocess.PIPE
        )
        try:
            subprocess.check_call(["tar", "-x", "-C", str(cls.source)], stdin=archive.stdout)
        finally:
            if archive.stdout:
                archive.stdout.close()
        archive.wait()
        if archive.returncode != 0:
            raise RuntimeError("git archive 失败")

        subprocess.check_call(
            [sys.executable, str(TOOLS / "extract_tm.py"),
             "--base", cls.base_ref, "--localized", cls.zh_ref, "--out", str(cls.tm)],
            cwd=ROOT,
        )

    @classmethod
    def tearDownClass(cls) -> None:
        cls.tmp.cleanup()

    def test_rebuild_matches_zh(self) -> None:
        pending_md = self.tmpdir / "pending.md"
        pending_json = self.tmpdir / "pending.json"
        result = subprocess.run(
            [sys.executable, str(TOOLS / "apply_tm.py"),
             "--source-dir", str(self.source),
             "--tm", str(self.tm),
             "--pending-md", str(pending_md),
             "--pending-json", str(pending_json)],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertFalse(pending_json.exists() and pending_json.read_text().strip() not in ("", "[]"))

        mismatches = []
        for path in git("ls-tree", "-r", "--name-only", self.base_ref).splitlines():
            actual = (self.source / path).read_bytes()
            expected = subprocess.check_output(["git", "show", f"{self.zh_ref}:{path}"], cwd=ROOT)
            if actual != expected:
                mismatches.append(path)
        self.assertEqual(mismatches, [], f"重建结果与 zh 不一致：{mismatches[:10]}")

    def test_verify_rebuilt_tree(self) -> None:
        result = subprocess.run(
            [sys.executable, str(TOOLS / "verify.py"),
             "--base-ref", self.base_ref,
             "--localized-dir", str(self.source)],
            cwd=ROOT,
            capture_output=True,
            text=True,
        )
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)


if __name__ == "__main__":
    unittest.main()
