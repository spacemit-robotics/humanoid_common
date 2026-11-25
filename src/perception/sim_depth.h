// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#ifndef SIM_DEPTH_H
#define SIM_DEPTH_H

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace perception {
struct SimDepthConfig {
    std::string preview_endpoint;
    double fps = 30;
    double latency_s = 0;
    bool enabled = false;
    uint32_t seed = 42;
    double random_hole_probability = .01;
    int patch_count = 2;
    int patch_radius_px = 1;
    double edge_threshold_m = .1;
    double edge_hole_probability = .25;
    int edge_width_px = 1;
    int hold_frames = 3;
    double noise_std_m = 0;
    void Validate() const;
};

// Absolute simulation-time deadlines avoid rounding 30 FPS down to 29.4 Hz
// when the physics step is 1 ms. Missed deadlines never cause a capture burst.
class SimDepthRate {
public:
    explicit SimDepthRate(double fps);
    bool Due(double time);
    void Reset();

private:
    double period_, next_ = -1, last_ = -1;
};

// Synthetic stress inputs at the published 64x36 resolution, before preprocessing.
// Missing depth remains zero; the control receiver decides how to handle it.
class SimDepthDegrader {
public:
    explicit SimDepthDegrader(const SimDepthConfig &config);
    void Apply(std::vector<float> &depth);
    void Reset();

private:
    SimDepthConfig config_;
    std::mt19937 rng_;
    uint64_t frames_ = 0;
    std::vector<float> random_, edge_random_;
    std::vector<unsigned char> patches_;
    void GenerateMasks();
};
}  // namespace perception
#endif  // SIM_DEPTH_H
