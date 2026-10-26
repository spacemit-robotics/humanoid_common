#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Bounded launcher output, manual flight-recorder capture and diagnostic export."""

import argparse
from contextlib import contextmanager
import fcntl
import hashlib
import io
import json
import os
from pathlib import Path
import re
import shutil
import stat
import sys
import tarfile
import tempfile
import time

import yaml

MB = 1024 * 1024


@contextmanager
def lease(path):
    """Never wait for a live session or another retention worker."""
    descriptor = None
    locked = False
    try:
        descriptor = os.open(path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o660)
        if os.fstat(descriptor).st_uid == os.geteuid():
            os.fchmod(descriptor, 0o660)
        fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        locked = True
    except OSError:
        pass
    try:
        yield locked
    finally:
        if descriptor is not None:
            os.close(descriptor)


def regular_files(directory):
    """Do not traverse symlinks, devices or readiness FIFOs."""
    for base, directories, names in os.walk(directory, followlinks=False):
        directories[:] = [name for name in directories if not (Path(base) / name).is_symlink()]
        for name in names:
            path = Path(base) / name
            try:
                if stat.S_ISREG(path.lstat().st_mode):
                    yield path
            except FileNotFoundError:
                continue


def owned(path, robot):
    """Only automatically remove sessions carrying this robot's explicit metadata."""
    for name in ("metadata.json", "launcher.json"):
        marker = path / name
        if marker.is_symlink() or not marker.is_file():
            continue
        try:
            data = json.loads(marker.read_text(encoding="utf-8"))
            if data.get("robot") != robot:
                return False
            if not (path / ".active.lock").exists() and data.get("pid", 0) > 0:
                try:
                    os.kill(data["pid"], 0)
                    return False
                except ProcessLookupError:
                    pass
            return True
        except (OSError, ValueError, TypeError):
            return False
    return False


def prune(root, robot, limit, reserve):
    """Run outside realtime threads; lock leases before removing inactive sessions."""
    with lease(root / f".{robot}.retention.lock") as locked:
        if not locked:
            return False
        sessions = [p for p in root.iterdir() if p.name.startswith(robot + "_")
                    and p.is_dir() and not p.is_symlink()]
        sizes = {p: sum(f.stat().st_size for f in regular_files(p)) for p in sessions}
        used = sum(sizes.values())
        margin = min(limit // 8, 16 * MB)
        for path in sorted(sessions, key=lambda p: p.stat().st_mtime):
            if (not limit or used + margin < limit) and shutil.disk_usage(root).free >= reserve:
                break
            if not owned(path, robot) or time.time() - path.stat().st_mtime < 60:
                continue
            with lease(path / ".active.lock") as inactive:
                if inactive:
                    shutil.rmtree(path)
                    used -= sizes[path]
        return (not limit or used + margin < limit) and shutil.disk_usage(root).free >= reserve


def settings(config):
    """Resolve the same paths and bounds as the C++ logger."""
    config = Path(config).resolve()
    value = yaml.safe_load(config.read_text(encoding="utf-8"))
    logging = value.get("logging", {})
    robot = re.sub(r"[^a-zA-Z0-9_-]", "_", value.get("robot_base", {}).get("name", "robot"))
    root = Path(os.environ.get("HUMANOID_LOG_DIRECTORY", "") or
                config.parent / logging.get("directory", "log/humanoid")).resolve()
    storage = logging.get("storage", {})
    limit = max(0, int(storage.get("max_total_mb", 0))) * MB
    reserve = max(0, int(storage.get("min_free_mb", 0))) * MB
    if str(root).startswith("/dev/shm/"):
        limit, reserve = 128 * MB, 16 * MB
    return root, robot, limit, reserve


def prepare(args):
    """A full/unwritable disk cannot prevent the launcher from starting the robot."""
    root, robot, limit, reserve = settings(args.config)
    try:
        root.mkdir(parents=True, exist_ok=True)
        if not prune(root, robot, limit, reserve):
            raise OSError("log quota or free-space reserve reached")
        launch = Path(tempfile.mkdtemp(prefix=f"{robot}_launch_{time.strftime('%Y%m%d_%H%M%S')}_",
                                      dir=root))
    except OSError as error:
        root = Path(f"/dev/shm/humanoid-logs-{os.getuid()}")
        root.mkdir(mode=0o700, parents=True, exist_ok=True)
        prune(root, robot, 128 * MB, 16 * MB)
        launch = Path(tempfile.mkdtemp(prefix=f"{robot}_launch_", dir=root))
        print(f"日志磁盘不可用：{error}；临时写入 {root}，不影响控制启动。", file=sys.stderr)
    (launch / "launcher.json").write_text(json.dumps({
        "robot": robot, "pid": os.getppid(), "boot_id": boot_id(),
        "started_monotonic_s": time.monotonic(), "started_at": time.time(),
    }), encoding="utf-8")
    print(root)
    print(launch)


def boot_id():
    """Correlate monotonic timestamps only within the same kernel boot."""
    return Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()


def capture(args):
    """Request a bounded high-rate recording without touching robot commands."""
    root, robot, _, _ = settings(args.config)
    request = root / f".{robot}.capture.json"
    now = time.monotonic()
    value = {"id": f"{time.monotonic_ns()}_manual", "boot_id": boot_id(),
             "monotonic_s": now, "reason": args.reason[:512], "post_s": args.seconds}
    with lease(root / f".{robot}.capture.lock") as locked:
        if not locked:
            raise RuntimeError("另一个故障片段正在触发，请稍后重试")
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=root,
                                         prefix=".capture-", delete=False) as output:
            json.dump(value, output)
            temporary = Path(output.name)
        temporary.replace(request)
    print(f"已请求片段 {value['id']}；前段按 YAML 配置，后段 {args.seconds:g} 秒。")


def sink(args):
    """Drain stdout even on ENOSPC. Rotation/storage checks are not on robot threads."""
    root, robot, limit, reserve = settings(args.config)
    path = Path(args.file)
    os.nice(10)
    output = None
    used = 0
    check_at = 0.0
    since_check = 0
    writable = True
    suppressed = 0
    try:
        while True:
            block = sys.stdin.buffer.read1(65536)
            if not block:
                break
            now = time.monotonic()
            if now >= check_at or since_check >= MB:
                check_at = now + 5.0
                since_check = 0
                try:
                    writable = prune(root, robot, limit, reserve)
                except OSError:
                    writable = False
            if not writable:
                suppressed += len(block)
                continue
            try:
                if output is None:
                    output = path.open("ab", buffering=0)
                    used = path.stat().st_size
                if used + len(block) > args.max_mb * MB:
                    output.close()
                    output = None
                    for index in range(args.files - 1, 0, -1):
                        previous = path if index == 1 else Path(f"{path}.{index - 1}")
                        if previous.exists():
                            previous.replace(Path(f"{path}.{index}"))
                    if args.files == 1:
                        path.unlink(missing_ok=True)
                    output = path.open("wb", buffering=0)
                    used = 0
                output.write(block)
                used += len(block)
                since_check += len(block)
            except OSError:
                if output is not None:
                    output.close()
                output = None
                writable = False
                suppressed += len(block)
    finally:
        if output is not None:
            output.close()
        if suppressed:
            print(f"日志输出降级：{path.name} 丢弃 {suppressed} 字节；子进程未中断。", file=sys.stderr)


class DigestReader:
    """Hash exactly the bytes copied, including a size-bounded live file prefix."""

    def __init__(self, stream):
        self.stream = stream
        self.digest = hashlib.sha256()

    def read(self, size):
        """tarfile requests only the snapshotted length."""
        data = self.stream.read(size)
        self.digest.update(data)
        return data


def export(args):
    """Build a compressed evidence bundle; never block on FIFOs or follow links."""
    root, robot, _, _ = settings(args.config)
    destination = Path(args.output).resolve()
    if destination == root or root in destination.parents:
        raise ValueError("诊断包输出必须放在日志目录之外")
    files = []
    for session in root.iterdir():
        if session.name.startswith(robot + "_") and session.is_dir() and not session.is_symlink():
            files.extend(p for p in regular_files(session)
                         if not p.name.startswith(".") or p.name == ".complete")
    files.sort(key=lambda p: p.stat().st_mtime, reverse=True)
    manifest = {"robot": robot, "boot_id": boot_id(), "created_at": time.time(),
                "kernel": list(os.uname()), "files": [], "omitted": []}
    copied = 0
    destination.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(prefix=".diagnostics-", dir=destination.parent)
    os.close(descriptor)
    temporary = Path(temporary_name)
    try:
        with tarfile.open(temporary, "w:gz", compresslevel=3) as archive:
            for path in files:
                try:
                    descriptor = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
                except OSError:
                    manifest["omitted"].append(str(path.relative_to(root)))
                    continue
                with os.fdopen(descriptor, "rb") as source:
                    info = os.fstat(source.fileno())
                    name = str(path.relative_to(root))
                    if not stat.S_ISREG(info.st_mode) or copied + info.st_size > args.max_mb * MB:
                        manifest["omitted"].append(name)
                        continue
                    item = tarfile.TarInfo(name)
                    item.size, item.mtime, item.mode = info.st_size, info.st_mtime, 0o600
                    reader = DigestReader(source)
                    archive.addfile(item, reader)
                    copied += info.st_size
                    manifest["files"].append({"path": name, "size": info.st_size,
                                              "sha256": reader.digest.hexdigest()})
            encoded = json.dumps(manifest, indent=2).encode()
            item = tarfile.TarInfo("manifest.json")
            item.size = len(encoded)
            archive.addfile(item, io.BytesIO(encoded))
        temporary.replace(destination)
    finally:
        temporary.unlink(missing_ok=True)
    print(f"{destination}：{len(manifest['files'])} 个文件，原始 {copied / MB:.1f} MiB，"
          f"跳过 {len(manifest['omitted'])} 项；SHA256 清单在 manifest.json。")


def main():
    """No command in this utility enables motors or changes FSM state."""
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for name, function in (("prepare", prepare), ("capture", capture), ("sink", sink), ("export", export)):
        command = commands.add_parser(name)
        command.add_argument("--config", required=True)
        command.set_defaults(function=function)
        if name == "capture":
            command.add_argument("--seconds", type=float, default=10)
            command.add_argument("--reason", default="manual diagnostic capture")
        elif name == "sink":
            command.add_argument("--file", required=True)
            command.add_argument("--max-mb", type=int, default=4)
            command.add_argument("--files", type=int, default=3)
        elif name == "export":
            command.add_argument("--output", required=True)
            command.add_argument("--max-mb", type=int, default=4096)
    args = parser.parse_args()
    if hasattr(args, "seconds") and not 0 < args.seconds <= 120:
        parser.error("--seconds must be in (0, 120]")
    if hasattr(args, "max_mb") and args.max_mb < 1:
        parser.error("--max-mb must be positive")
    if hasattr(args, "files") and not 1 <= args.files <= 32:
        parser.error("--files must be in [1, 32]")
    try:
        args.function(args)
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, f"日志工具：{error}\n")


if __name__ == "__main__":
    main()
