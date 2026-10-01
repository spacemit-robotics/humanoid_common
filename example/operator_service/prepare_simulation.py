#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Create an isolated PC simulation config without changing the robot config."""

import argparse
import os
from pathlib import Path

import yaml


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--port", type=int, default=8765)
    args = parser.parse_args()
    if not 1 <= args.port <= 65535:
        parser.error("port must be 1..65535")
    source = args.config.resolve()
    config = yaml.safe_load(source.read_text(encoding="utf-8"))
    robot = config["robot_base"]
    robot["robot_dir"] = str((source.parent / robot["robot_dir"]).resolve())
    output = args.output.resolve()
    output.mkdir(mode=0o700, parents=True, exist_ok=True)
    if output.stat().st_uid != os.getuid() or output.stat().st_mode & 0o077:
        parser.error("output directory must be owned by this user with mode 0700")
    config["driver"] = {"backend": "mujoco"}
    config["simulation"]["mujoco"]["viewer"] = not args.headless
    config["transport"]["type"] = "shm"
    config["transport"].setdefault("shm", {})["prefix"] = f"{robot['name']}_operator_{os.getpid()}"
    logging = config.setdefault("logging", {})
    logging["directory"] = str(output / "logs")
    logging["artifacts"] = [str((source.parent / path).resolve()) for path in logging.get("artifacts", [])]
    config["operator_service"] = {
        "bind_address": "127.0.0.1", "port": args.port, "lease_ms": 1000,
        "connection_file": str(output / "connection.json"),
    }
    destination = output / "simulation.yaml"
    destination.write_text(yaml.safe_dump(config, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print(f"Configuration: {destination}")
    print(f"Connection: {output / 'connection.json'}")
    print(f"Backend: mujoco, viewer={not args.headless}; source file unchanged")


if __name__ == "__main__":
    main()
