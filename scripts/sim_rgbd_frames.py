#!/usr/bin/env python3
# Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
# SPDX-License-Identifier: Apache-2.0
"""Read paired MuJoCo camera frames and write 640x180 BGR24 to the video encoder."""
import argparse
import struct
import sys

import cv2
import numpy as np
import zmq


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("endpoint")
    args = parser.parse_args()
    cv2.setNumThreads(1)
    with zmq.Context() as context, context.socket(zmq.SUB) as sub:
        sub.setsockopt(zmq.LINGER, 0)
        sub.setsockopt(zmq.CONFLATE, 1)
        sub.setsockopt(zmq.SUBSCRIBE, b"")
        sub.connect(args.endpoint)
        while True:
            packet = sub.recv()
            if len(packet) < 4:
                continue
            start = 4 + struct.unpack_from("<I", packet)[0]
            color = start + 64 * 36 * 4
            if start > 4100 or len(packet) != color + 320 * 180 * 3:
                continue
            depth = np.frombuffer(packet, "<f4", 64 * 36, start).reshape(36, 64)
            rgb = np.frombuffer(packet, np.uint8, offset=color).reshape(180, 320, 3)
            valid = np.isfinite(depth) & (depth > 0)
            scale = np.rint(255 * (1 - np.clip(np.where(valid, depth, 2.5) / 2.5, 0, 1)))
            colored = cv2.applyColorMap(scale.astype(np.uint8), cv2.COLORMAP_JET)
            colored[~valid] = 0
            panel = cv2.resize(colored, (320, 180), interpolation=cv2.INTER_NEAREST)
            sys.stdout.buffer.write(np.hstack((rgb[::-1, :, ::-1], panel)).tobytes())


if __name__ == "__main__":
    try:
        main()
    except (BrokenPipeError, KeyboardInterrupt):
        pass
