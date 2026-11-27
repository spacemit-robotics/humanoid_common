/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file depth_stream.h
 * @brief Hiking depth stream and encoder input interface.
 */

#ifndef DEPTH_STREAM_H
#define DEPTH_STREAM_H

#include <array>
#include <array>
#include <atomic>
#include <chrono>
#include <deque>
#include <mutex>
#include <nlohmann/json.hpp>
#include <stdexcept>
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
struct DepthObservation {
    std::vector<float> stack;
    float frame_age_s = 0;
    float history_delay_s = 0;
};
class Receiver {
    public:
    explicit Receiver(const std::string &endpoint, double timeout_s = .2);
    ~Receiver();
    // Returns only frames at or before the robot state time in the same clock domain.
    std::string Error(double time, const std::string &clock) const;
    std::vector<float> Stack(double time, const std::string &clock) const;
    std::string PrimeHeldInput(double time, const std::string &clock);
    DepthObservation HeldStack(double time, const std::string &clock);
    nlohmann::json Status() const;

    private:
    using Clock = std::chrono::steady_clock;
    struct Sample {
        Frame frame;
        std::vector<float> image;
        Clock::time_point received_at;
    };
    using History = std::array<const Sample *, 8>;
    void Loop(std::string endpoint);
    // Caller holds mutex_; each pointer refers to a causal sample in the current session.
    std::string LatestError(double time, const std::string &clock, Clock::time_point now,
                            Clock::time_point &capture_at) const;
    std::string SelectHistory(double time, History &history) const;
    static void CopyStack(const History &history, std::vector<float> &stack);
    std::vector<float> held_;
    Clock::time_point last_frame_at_{};
    Clock::time_point last_history_at_{};
    double timeout_;
    std::atomic<bool> stop_{false};
    std::thread thread_;
    mutable std::mutex mutex_;
    std::deque<Sample> frames_;
    std::string error_ = "depth not received";
    uint64_t rejected_ = 0;
};
} // namespace perception

#endif // DEPTH_STREAM_H
