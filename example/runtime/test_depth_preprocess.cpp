/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_depth_preprocess.cpp
 * @brief Verify the Hiking depth crop, invalid-pixel repair and normalization.
 */
#include "depth_stream.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <vector>

int main() {
    std::vector<float> raw(perception::kPixels, 2.5f);
    for (int y = 18; y < 36; ++y)
        for (int x = 16; x < 48; ++x) raw[y * 64 + x] = 1.f;

    // The hole touches the crop boundary. Pixels above the crop must not contribute.
    for (int y = 18; y < 25; ++y)
        for (int x = 26; x < 33; ++x) raw[y * 64 + x] = (x % 2) ? .1f : 0.f;
    raw[21 * 64 + 28] = std::numeric_limits<float>::quiet_NaN();
    const auto repaired = perception::Preprocess(raw);
    assert(repaired.size() == perception::kFrame);
    for (float d : repaired) assert(std::abs(d - .4f) < 1e-5f);

    // No valid measurement inside the crop: do not invent a far surface.
    for (int y = 18; y < 36; ++y)
        for (int x = 16; x < 48; ++x) raw[y * 64 + x] = 0.f;
    for (float d : perception::Preprocess(raw)) assert(d == 0.f);
    return 0;
}
