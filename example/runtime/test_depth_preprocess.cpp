// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "depth_stream.h"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

int main() {
    std::vector<float> raw(perception::kPixels, 1.25f);
    const auto intact = raw;
    assert(perception::FillDepthHoles(raw) == raw);
    for (float d : perception::Preprocess(raw)) assert(std::abs(d - .5f) < 1e-6);

    // Small holes recover a flat surface, including the center of a 3x3 patch.
    for (int y = 22; y <= 24; ++y)
        for (int x = 30; x <= 32; ++x) raw[y * 64 + x] = 0;
    raw[0] = std::numeric_limits<float>::quiet_NaN();
    raw[63] = -1;
    const auto small = perception::FillDepthHoles(raw);
    assert(small == intact);
    assert(raw[23 * 64 + 31] == 0 && std::isnan(raw[0]));  // Raw input stays untouched.
    for (float d : perception::Preprocess(raw)) assert(std::abs(d - .5f) < 1e-6);

    // The center of a 5x5 void has no original donor within radius 2.
    raw = intact;
    for (int y = 20; y <= 24; ++y)
        for (int x = 30; x <= 34; ++x) raw[y * 64 + x] = 0;
    assert(perception::FillDepthHoles(raw)[22 * 64 + 32] == 2.5f);

    // Ties are deterministic; copied values come from actual surfaces rather
    // than averaging a foreground and background into an invented distance.
    raw.assign(perception::kPixels, 0);
    raw[0] = .8f;
    raw[4] = 1.4f;
    const auto sparse = perception::FillDepthHoles(raw);
    assert(sparse[2] == .8f);
    assert(sparse[64 + 3] == 1.4f);
    assert(sparse[2 * 64 + 2] == 2.5f);  // No recursive propagation from filled pixels.

    raw = intact;
    for (int y = 0; y < 36; ++y)
        for (int x = 32; x < 64; ++x) raw[y * 64 + x] = 2;
    raw[23 * 64 + 31] = raw[23 * 64 + 32] = 0;
    const auto edge = perception::FillDepthHoles(raw);
    assert(edge[23 * 64 + 31] == 1.25f && edge[23 * 64 + 32] == 2);
    for (size_t i = 0; i < raw.size(); ++i)
        if (raw[i] > 0) assert(edge[i] == raw[i]);

    raw.assign(perception::kPixels, std::numeric_limits<float>::infinity());
    for (float d : perception::FillDepthHoles(raw)) assert(d == 2.5f);
    for (float d : perception::Preprocess(raw)) assert(std::abs(d - 1) < 1e-6);
    std::cout << "PASS bounded nearest-neighbor hole filling and Hiking preprocessing\n";
}
