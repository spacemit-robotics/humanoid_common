/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file depth_stream.h
 * @brief Hiking depth stream and encoder input interface.
 */

#ifndef DEPTH_STREAM_H
#define DEPTH_STREAM_H

#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <string>
#include <thread>
#include <vector>

namespace perception {
constexpr int kWidth = 64, kHeight = 36, kPixels = kWidth * kHeight;
constexpr int kFrame = 18 * 32, kStack = 8 * kFrame;
constexpr const char *kProfile = "g1-sdk-hiking-v1";
struct Frame {
    double time = 0;
    uint64_t sequence = 0;
    std::string session;
    std::string clock;
    std::vector<float> pixels;
};
// Hiking encoder input: bottom-center crop, inpaint, blur, normalize to [0, 1].
std::vector<float> Preprocess(const std::vector<float> &raw);
std::vector<unsigned char> Encode(const Frame &frame);
Frame Decode(const void *data, size_t size);
class Receiver {
    public:
    explicit Receiver(const std::string &endpoint, double timeout_s = .2);
    ~Receiver();
    // Returns only frames at or before the robot state time in the same clock domain.
    std::string Error(double time, const std::string &clock) const;
    std::vector<float> Stack(double time, const std::string &clock) const;
    nlohmann::json Status() const;

    private:
    struct Sample {
        Frame frame;
        std::vector<float> image;
    };
    void Loop(std::string endpoint);
    std::string ErrorLocked(double time, const std::string &clock) const;
    double timeout_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    std::deque<Sample> frames_;
    std::chrono::steady_clock::time_point received_{};
    std::string error_ = "depth not received";
    uint64_t rejected_ = 0;
};
} // namespace perception

#endif // DEPTH_STREAM_H
