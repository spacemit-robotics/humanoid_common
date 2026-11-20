// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "sim_depth.h"
#include "depth_stream.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <vector>

namespace {
perception::SimDepthConfig Clean() {
    perception::SimDepthConfig config;
    config.enabled = true;
    config.random_hole_probability = 0;
    config.patch_count = 0;
    config.edge_hole_probability = 0;
    return config;
}

void CheckRate() {
    perception::SimDepthRate rate(30);
    std::vector<int> ticks;
    for (int i = 0; i < 10000; ++i)
        if (rate.Due(i * .001)) ticks.push_back(i);
    assert(ticks.size() == 300);
    for (size_t i = 1; i < ticks.size(); ++i) {
        const int interval = ticks[i] - ticks[i - 1];
        assert(interval == 33 || interval == 34);
    }
    assert(rate.Due(20));
    assert(!rate.Due(20));  // Skip missed captures rather than creating a burst.
    assert(!rate.Due(20.001));
    assert(rate.Due(0));   // A simulation reset starts a fresh capture schedule.
    assert(!rate.Due(0));
}

void CheckMasks() {
    const std::vector<float> raw(perception::kPixels, 1);
    auto config = Clean();
    config.enabled = false;
    config.random_hole_probability = 1;
    auto frame = raw;
    perception::SimDepthDegrader disabled(config);
    disabled.Apply(frame);
    assert(frame == raw);

    config.enabled = true;
    perception::SimDepthDegrader all_holes(config);
    all_holes.Apply(frame);
    assert(std::count(frame.begin(), frame.end(), 0.f) == perception::kPixels);
    perception::Frame packet;
    packet.clock = "sim";
    packet.time = 1;
    packet.session = "synthetic-test";
    packet.pixels = frame;
    const auto bytes = perception::Encode(packet);
    const auto decoded = perception::Decode(bytes.data(), bytes.size());
    assert(decoded.time == 1 && decoded.clock == "sim" && decoded.pixels == frame);
    // An entirely missing image has no measured surface to inpaint from.
    const auto processed = perception::Preprocess(frame);
    for (float d : processed) assert(d == 0.f);

    config.random_hole_probability = .1;
    config.hold_frames = 3;
    perception::SimDepthDegrader persistent(config), same_seed(config);
    auto first = raw, same = raw;
    persistent.Apply(first);
    same_seed.Apply(same);
    assert(first == same && first != raw);
    for (int i = 1; i < 3; ++i) {
        frame = raw;
        persistent.Apply(frame);
        assert(frame == first);
    }
    frame = raw;
    persistent.Apply(frame);
    assert(frame != first);
    persistent.Reset();
    frame = raw;
    persistent.Apply(frame);
    assert(frame == first);

    config = Clean();
    config.patch_count = 1;
    config.patch_radius_px = 1;
    perception::SimDepthDegrader patches(config);
    frame = raw;
    patches.Apply(frame);
    int left = 64, right = -1, top = 36, bottom = -1, count = 0;
    for (int y = 0; y < 36; ++y)
        for (int x = 0; x < 64; ++x)
            if (frame[y * 64 + x] == 0) {
                left = std::min(left, x); right = std::max(right, x);
                top = std::min(top, y); bottom = std::max(bottom, y);
                ++count;
            }
    assert(count > 0 && count <= 9);
    assert(right - left <= 2 && bottom - top <= 2);
    assert(count == (right - left + 1) * (bottom - top + 1));
}

void CheckEdgesAndNoise() {
    auto config = Clean();
    config.edge_hole_probability = 1;
    for (int width = 1; width <= 2; ++width) {
        config.edge_width_px = width;
        perception::SimDepthDegrader edges(config);
        std::vector<float> frame(perception::kPixels, 1);
        for (int y = 0; y < 36; ++y)
            for (int x = 32; x < 64; ++x) frame[y * 64 + x] = 2;
        edges.Apply(frame);
        for (int y = 0; y < 36; ++y)
            for (int x = 0; x < 64; ++x) {
                const float expected = x >= 32 - width && x < 32 + width ? 0 : (x < 32 ? 1 : 2);
                assert(frame[y * 64 + x] == expected);
            }
    }
    config = Clean();
    config.noise_std_m = .01;
    perception::SimDepthDegrader noise(config);
    std::vector<float> frame(perception::kPixels, 1);
    frame[0] = 0; frame[1] = std::numeric_limits<float>::quiet_NaN(); frame[2] = -1;
    noise.Apply(frame);
    assert(frame[0] == 0 && frame[1] == 0 && frame[2] == 0);
    double squared_error = 0;
    for (size_t i = 3; i < frame.size(); ++i) {
        assert(std::isfinite(frame[i]) && frame[i] > 0);
        squared_error += std::pow(frame[i] - 1, 2);
    }
    const double rms = std::sqrt(squared_error / (frame.size() - 3));
    assert(rms > .008 && rms < .012);
}

void CheckValidation() {
    for (int i = 0; i < 11; ++i) {
        auto config = Clean();
        switch (i) {
            case 0: config.fps = 0; break;
            case 1: config.fps = std::numeric_limits<double>::quiet_NaN(); break;
            case 2: config.latency_s = -1; break;
            case 3: config.latency_s = 1.1; break;
            case 4: config.random_hole_probability = 1.1; break;
            case 5: config.edge_hole_probability = -1; break;
            case 6: config.patch_count = -1; break;
            case 7: config.patch_radius_px = 9; break;
            case 8: config.hold_frames = 0; break;
            case 9: config.edge_width_px = 0; break;
            case 10: config.noise_std_m = std::numeric_limits<double>::infinity(); break;
        }
        bool rejected = false;
        try { config.Validate(); } catch (const std::runtime_error &) { rejected = true; }
        assert(rejected);
    }
}
}  // namespace

int main() {
    CheckRate();
    CheckMasks();
    CheckEdgesAndNoise();
    CheckValidation();
    std::cout << "PASS simulation depth schedule, masks, reset, wire format and validation\n";
}
