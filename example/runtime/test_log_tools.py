#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise log quotas, failed writes, rotation and portable evidence export."""

import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import time
import unittest

TOOL = Path(__file__).resolve().parents[2] / "scripts/humanoid_logs.py"


class LogToolsTest(unittest.TestCase):
    """All fixtures are temporary; no robot or production logs are used."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="humanoid_log_tools_")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        self.logs = self.root / "logs"
        self.logs.mkdir()
        self.config = self.root / "config.yaml"
        self.write_config(10)

    def write_config(self, limit):
        """Set a small test quota."""
        self.config.write_text(f"robot_base: {{name: testbot}}\nlogging:\n  directory: {self.logs}\n"
                               f"  storage: {{max_total_mb: {limit}, min_free_mb: 0}}\n", encoding="utf-8")

    def command(self, name, *arguments, data=None):
        """Run the installed-equivalent CLI, independent of the developer's env."""
        environment = dict(os.environ)
        environment.pop("HUMANOID_LOG_DIRECTORY", None)
        return subprocess.run(  # noqa: S603
            [sys.executable, str(TOOL), name, "--config", str(self.config), *map(str, arguments)],
            input=data, capture_output=True, env=environment, timeout=15, check=False)

    def session(self, name, size=0):
        """Create an old, owned session; arbitrary directories lack this marker."""
        path = self.logs / ("testbot_" + name)
        path.mkdir()
        (path / "metadata.json").write_text('{"robot":"testbot","pid":0}', encoding="ascii")
        (path / "events.log").write_bytes(b"x" * size)
        old = time.time() - 120
        os.utime(path, (old, old))
        return path

    def test_retention_protects_active_unknown_and_links(self):
        self.write_config(3)
        old = self.session("old", 2 * 1024 * 1024)
        active = self.session("active", 1024 * 1024)
        unknown = self.logs / "testbot_unknown"
        unknown.mkdir()
        (unknown / "keep.txt").write_text("unowned", encoding="ascii")
        (self.logs / "testbot_link").symlink_to(unknown, target_is_directory=True)
        with (active / ".active.lock").open("w", encoding="ascii") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            result = self.command("prepare")
            self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(old.exists())
        self.assertTrue((active / "events.log").exists())
        self.assertTrue((unknown / "keep.txt").exists())
        self.assertTrue((self.logs / "testbot_link").is_symlink())

    def test_full_directory_falls_back_to_bounded_ram(self):
        self.config.write_text(f"robot_base: {{name: testbot}}\nlogging:\n"
                               f"  directory: {self.config}/impossible\n", encoding="utf-8")
        result = self.command("prepare")
        self.assertEqual(result.returncode, 0, result.stderr)
        root, session = map(Path, result.stdout.decode().splitlines())
        self.addCleanup(shutil.rmtree, session)
        self.assertTrue(str(root).startswith("/dev/shm/humanoid-logs-"))
        self.assertTrue((session / "launcher.json").is_file())

    def test_sink_rotates_and_drains_failed_writes(self):
        session = self.session("launch")
        output = session / "stdout.log"
        result = self.command("sink", "--file", output, "--max-mb", 1, "--files", 2,
                              data=b"z" * (5 * 1024 * 1024))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertLessEqual(sum(p.stat().st_size for p in session.glob("stdout.log*")), 2 * 1024 * 1024)
        result = self.command("sink", "--file", "/dev/full", data=b"z" * (2 * 1024 * 1024))
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertIn("丢弃".encode(), result.stderr)

    def test_capture_request_has_shared_clock_and_bound(self):
        result = self.command("capture", "--seconds", 3, "--reason", "test capture")
        self.assertEqual(result.returncode, 0, result.stderr)
        request = json.loads((self.logs / ".testbot.capture.json").read_text(encoding="utf-8"))
        self.assertLess(abs(request["monotonic_s"] - time.monotonic()), 2)
        self.assertEqual(request["post_s"], 3)
        self.assertNotEqual(self.command("capture", "--seconds", 1000).returncode, 0)

    def test_export_excludes_fifos_links_and_verifies_hashes(self):
        session = self.session("export", 1024)
        os.mkfifo(session / "control.ready")
        (session / "outside").symlink_to("/etc/passwd")
        output = self.root / "evidence.tar.gz"
        result = self.command("export", "--output", output)
        self.assertEqual(result.returncode, 0, result.stderr)
        with tarfile.open(output) as archive:
            manifest = json.load(archive.extractfile("manifest.json"))
            self.assertEqual(len(manifest["files"]), 2)
            for entry in manifest["files"]:
                data = archive.extractfile(entry["path"]).read()
                self.assertEqual(hashlib.sha256(data).hexdigest(), entry["sha256"])
        self.assertNotEqual(self.command("export", "--output", self.logs / "recursive.tar.gz").returncode, 0)


if __name__ == "__main__":
    unittest.main()
