// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "sim_depth.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "depth_stream.h"

namespace perception {
void SimDepthConfig::Validate() const {
    const auto probability = [](double p) { return std::isfinite(p) && p >= 0 && p <= 1; };
    if (!std::isfinite(fps) || fps < 1 || fps > 120 ||
        !std::isfinite(latency_s) || latency_s < 0 || latency_s > 1 ||
        !probability(random_hole_probability) || !probability(edge_hole_probability) ||
        patch_count < 0 || patch_count > 128 || patch_radius_px < 0 || patch_radius_px > 8 ||
        !std::isfinite(edge_threshold_m) || edge_threshold_m <= 0 ||
        edge_width_px < 1 || edge_width_px > 4 || hold_frames < 1 || hold_frames > 300 ||
        !std::isfinite(noise_std_m) || noise_std_m < 0 || noise_std_m > 1)
        throw std::runtime_error("invalid simulation depth configuration");
}

SimDepthRate::SimDepthRate(double fps) : period_(1 / fps) {
    if (!std::isfinite(fps) || fps < 1 || fps > 120)
        throw std::runtime_error("simulation depth FPS must be in [1, 120]");
}

void SimDepthRate::Reset() { next_ = last_ = -1; }

bool SimDepthRate::Due(double time) {
    if (!std::isfinite(time) || time < 0)
        throw std::runtime_error("invalid simulation depth time");
    if (time < last_) Reset();
    last_ = time;
    if (next_ < 0) {
        next_ = time + period_;
        return true;
    }
    if (time + 1e-9 < next_) return false;
    if (time - next_ + 1e-9 >= period_) next_ = time + period_;
    else next_ += period_;
    return true;
}

SimDepthDegrader::SimDepthDegrader(const SimDepthConfig &config)
    : config_(config), rng_(config.seed), random_(kPixels), edge_random_(kPixels), patches_(kPixels) {
    config_.Validate();
}

void SimDepthDegrader::Reset() {
    rng_.seed(config_.seed);
    frames_ = 0;
}

void SimDepthDegrader::GenerateMasks() {
    std::uniform_real_distribution<float> uniform(0, 1);
    for (int i = 0; i < kPixels; ++i) {
        random_[i] = uniform(rng_);
        edge_random_[i] = uniform(rng_);
    }
    std::fill(patches_.begin(), patches_.end(), 0);
    std::uniform_int_distribution<int> column(0, kWidth - 1), row(0, kHeight - 1);
    const int radius = config_.patch_radius_px;
    for (int n = 0; n < config_.patch_count; ++n) {
        const int cx = column(rng_), cy = row(rng_);
        for (int y = std::max(0, cy - radius); y <= std::min(kHeight - 1, cy + radius); ++y)
            for (int x = std::max(0, cx - radius); x <= std::min(kWidth - 1, cx + radius); ++x)
                patches_[y * kWidth + x] = 1;
    }
}

void SimDepthDegrader::Apply(std::vector<float> &depth) {
    if (depth.size() != kPixels) throw std::runtime_error("simulation depth must be 64x36");
    if (!config_.enabled) return;
    if (frames_++ % config_.hold_frames == 0) GenerateMasks();
    // Compute discontinuities from the unmodified frame, so synthetic holes do
    // not themselves spawn new edges or spread uncontrollably across the image.
    std::vector<unsigned char> holes = patches_;
    const auto valid = [](float d) { return std::isfinite(d) && d > 0; };
    for (int y = 0; y < kHeight; ++y)
        for (int x = 0; x < kWidth; ++x) {
            const int index = y * kWidth + x;
            const float d = depth[index];
            if (!valid(d)) continue;
            bool edge = false;
            for (const int offset : {-1, 1, -kWidth, kWidth}) {
                if ((offset == -1 && x == 0) || (offset == 1 && x == kWidth - 1) ||
                    (offset == -kWidth && y == 0) || (offset == kWidth && y == kHeight - 1)) continue;
                const float neighbor = depth[index + offset];
                if (valid(neighbor) && std::abs(d - neighbor) > config_.edge_threshold_m) edge = true;
            }
            if (edge && edge_random_[index] < config_.edge_hole_probability) {
                const int radius = config_.edge_width_px - 1;
                for (int yy = std::max(0, y - radius); yy <= std::min(kHeight - 1, y + radius); ++yy)
                    for (int xx = std::max(0, x - radius); xx <= std::min(kWidth - 1, x + radius); ++xx)
                        holes[yy * kWidth + xx] = 1;
            }
        }
    std::normal_distribution<float> noise(0, config_.noise_std_m > 0 ? config_.noise_std_m : 1);
    for (int i = 0; i < kPixels; ++i) {
        if (!valid(depth[i]) || holes[i] || random_[i] < config_.random_hole_probability) depth[i] = 0;
        else if (config_.noise_std_m > 0) depth[i] = std::max(.001f, depth[i] + noise(rng_));
    }
}
}  // namespace perception
