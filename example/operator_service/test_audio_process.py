#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Exercise the production player with fake children; never open an audio device."""

import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


def main():
    """Missing audio, crashed/hung players and normal shutdown stay non-fatal."""
    binary = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory(prefix="operator_audio_test_") as directory:
        root = Path(directory)
        (root / "clip.wav").touch()
        config = {"robot_base": {"robot_dir": str(root)}, "audio_prompts": {
            "enabled": True, "directory": ".", "max_play_seconds": 0.2,
            "player": [sys.executable, "-c", "raise SystemExit(23)"],
            "clips": {"service.ready": "clip.wav", "service.restarting": "clip.wav",
                      "missing": "missing.wav"}}}
        path = root / "config.yaml"

        def run(*arguments, result="success"):
            path.write_text(json.dumps(config), encoding="utf-8")  # JSON is a YAML subset.
            return subprocess.run(  # noqa: S603
                [binary, str(path), *arguments], capture_output=True, text=True,
                timeout=3, check=True, env=dict(os.environ, SERVICE_RESULT=result))

        assert "service.ready" in run("--once", "service.ready", "--dry-run").stdout
        assert "playback failed" in run("--once", "service.ready").stderr
        assert "missing WAV" in run("--once", "missing").stderr
        assert not run("--service-failure").stdout
        assert "service.restarting" in run("--service-failure", "--dry-run", result="exit-code").stdout
        config["audio_prompts"]["player"] = [sys.executable, "-c", "import time; time.sleep(30)"]
        started = time.monotonic()
        assert "timed out" in run("--once", "service.ready").stderr
        assert time.monotonic() - started < 2
        config["audio_prompts"]["enabled"] = False
        assert not run("--once", "service.ready").stdout
        print("audio process: dry run, failed/missing/hung player, failure-only restart and disabled passed")


if __name__ == "__main__":
    main()
