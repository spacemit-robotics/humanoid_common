#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Test archive retries, verified mirrors and offline cache without external services."""

from collections import Counter
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import io
import os
from pathlib import Path
import subprocess
import tarfile
import tempfile
import threading
import unittest

FETCH_MODULE = Path(__file__).resolve().parents[2] / "cmake/FetchThirdParty.cmake"


class FetchThirdPartyTest(unittest.TestCase):
    """Serve a tiny pinned archive from loopback only."""

    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="humanoid_fetch_test_")
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        archive = io.BytesIO()
        with tarfile.open(fileobj=archive, mode="w:gz") as stream:
            entry = tarfile.TarInfo("package/marker")
            entry.size = 2
            stream.addfile(entry, io.BytesIO(b"ok"))
        payload = archive.getvalue()
        self.digest = hashlib.sha256(payload).hexdigest()
        self.requests = Counter()
        requests = self.requests

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):  # noqa: N802
                requests[self.path] += 1
                if self.path == "/missing" or (self.path == "/retry" and requests[self.path] == 1):
                    self.send_error(503)
                    return
                content = b"invalid archive" if self.path == "/corrupt" else payload
                self.send_response(200)
                self.send_header("Content-Length", str(len(content)))
                self.end_headers()
                self.wfile.write(content)

            def log_message(self, *args):
                pass

        self.server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        worker = threading.Thread(target=self.server.serve_forever, daemon=True)
        worker.start()
        self.addCleanup(self.server.server_close)
        self.addCleanup(worker.join)
        self.addCleanup(self.server.shutdown)
        self.url = f"http://127.0.0.1:{self.server.server_port}"

    def fetch(self, endpoint, fallback=None, offline=False):
        """Invoke exactly the component-owned CMake helper in an isolated cache."""
        script = self.root / "fetch.cmake"
        mirror = f'ARCHIVE_FALLBACK_URLS "{self.url}/{fallback}"' if fallback else ""
        script.write_text(
            f'set(SROBOTIS_THIRDPARTY_FETCH_OFF {"ON" if offline else "OFF"})\n'
            f'include("{FETCH_MODULE}")\n'
            f'fetch_thirdparty(NAME fixture ARCHIVE_URL "{self.url}/{endpoint}" {mirror}\n'
            f' ARCHIVE_SHA256 "{self.digest}" ARCHIVE_SUBDIR package OUT_SOURCE_DIR result)\n'
            'file(READ "${result}/marker" marker)\n'
            'if(NOT marker STREQUAL "ok")\n message(FATAL_ERROR "bad payload")\nendif()\n',
            encoding="utf-8")
        environment = {**os.environ, "SROBOTIS_THIRDPARTY_CACHE": str(self.root / "cache")}
        return subprocess.run(  # noqa: S603
            ["cmake", "-P", str(script)], env=environment, capture_output=True,
            text=True, timeout=15, check=False)

    def test_transient_error_retried_then_offline_cache_used(self):
        result = self.fetch("retry")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.requests["/retry"], 2)
        result = self.fetch("missing", offline=True)
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.requests["/missing"], 0)

    def test_mirror_is_used_after_bounded_primary_retries(self):
        result = self.fetch("missing", fallback="valid")
        self.assertEqual(result.returncode, 0, result.stdout + result.stderr)
        self.assertEqual(self.requests["/missing"], 2)
        self.assertEqual(self.requests["/valid"], 1)

    def test_bad_hash_is_rejected_without_populating_cache(self):
        result = self.fetch("corrupt")
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("SHA256 mismatch", result.stderr)
        self.assertEqual(self.requests["/corrupt"], 2)
        self.assertFalse((self.root / "cache/fixture").exists())

    def test_offline_miss_does_not_contact_server(self):
        result = self.fetch("valid", offline=True)
        self.assertNotEqual(result.returncode, 0)
        self.assertIn("fetch is disabled", " ".join(result.stderr.split()))
        self.assertFalse(self.requests)


if __name__ == "__main__":
    unittest.main()
