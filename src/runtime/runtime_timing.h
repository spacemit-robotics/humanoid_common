/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file runtime_timing.h
 * @brief Fixed-capacity runtime loop timing statistics
 */

#ifndef RUNTIME_TIMING_H
#define RUNTIME_TIMING_H

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace runtime_timing {

using Clock = std::chrono::steady_clock;

struct Summary {
    uint64_t samples = 0;
    double period_p95_ms = 0.0;
    double period_p99_ms = 0.0;
    double period_max_ms = 0.0;
    double lateness_p95_ms = 0.0;
    double lateness_p99_ms = 0.0;
    double lateness_max_ms = 0.0;
    uint64_t skipped_cycles = 0;
};

class Window {
public:
    explicit Window(Clock::time_point start) : window_start_(start) {}

    void Observe(Clock::time_point start, Clock::time_point deadline) {
        if (has_previous_start_) {
            Store(&period_histogram_, std::chrono::duration<double, std::milli>(
                start - previous_start_).count());
        }
        Store(&lateness_histogram_, std::max(0.0,
            std::chrono::duration<double, std::milli>(start - deadline).count()));
        previous_start_ = start;
        has_previous_start_ = true;
    }

    void AddSkippedCycles(uint64_t count) { skipped_cycles_ += count; }

    bool IsDue(Clock::time_point now, double window_s) const {
        return std::chrono::duration<double>(now - window_start_).count() >= window_s;
    }

    Summary Consume(Clock::time_point now) {
        Summary result;
        result.samples = period_count_;
        result.period_p95_ms = Quantile(period_histogram_, period_count_, 0.95);
        result.period_p99_ms = Quantile(period_histogram_, period_count_, 0.99);
        result.period_max_ms = period_max_ms_;
        result.lateness_p95_ms = Quantile(lateness_histogram_, lateness_count_, 0.95);
        result.lateness_p99_ms = Quantile(lateness_histogram_, lateness_count_, 0.99);
        result.lateness_max_ms = lateness_max_ms_;
        result.skipped_cycles = skipped_cycles_;
        period_histogram_.fill(0);
        lateness_histogram_.fill(0);
        period_count_ = 0;
        lateness_count_ = 0;
        period_max_ms_ = 0.0;
        lateness_max_ms_ = 0.0;
        skipped_cycles_ = 0;
        window_start_ = now;
        return result;
    }

private:
    // 0.05 ms buckets preserve useful 500 Hz timing resolution through 51 ms.
    static constexpr std::size_t kHistogramBuckets = 1024;
    static constexpr double kBucketWidthMs = 0.05;

    static void Store(std::array<uint32_t, kHistogramBuckets> *histogram,
            uint64_t *count, double *maximum, double value) {
        const double nonnegative = value > 0.0 ? value : 0.0;
        const std::size_t bucket = nonnegative == 0.0 ? 0 :
            std::min(static_cast<std::size_t>(std::ceil(
                nonnegative / kBucketWidthMs)), kHistogramBuckets - 1);
        ++(*histogram)[bucket];
        ++(*count);
        *maximum = std::max(*maximum, nonnegative);
    }

    void Store(std::array<uint32_t, kHistogramBuckets> *histogram, double value) {
        if (histogram == &period_histogram_) {
            Store(histogram, &period_count_, &period_max_ms_, value);
        } else {
            Store(histogram, &lateness_count_, &lateness_max_ms_, value);
        }
    }

    static double Quantile(
            const std::array<uint32_t, kHistogramBuckets> &histogram,
            uint64_t count, double quantile) {
        if (count == 0) return 0.0;
        const uint64_t target = static_cast<uint64_t>(
            std::ceil(quantile * static_cast<double>(count)));
        uint64_t cumulative = 0;
        for (std::size_t i = 0; i < histogram.size(); ++i) {
            cumulative += histogram[i];
            if (cumulative >= target)
                return static_cast<double>(i) * kBucketWidthMs;
        }
        return static_cast<double>(kHistogramBuckets - 1) * kBucketWidthMs;
    }

    std::array<uint32_t, kHistogramBuckets> period_histogram_{};
    std::array<uint32_t, kHistogramBuckets> lateness_histogram_{};
    uint64_t period_count_ = 0;
    uint64_t lateness_count_ = 0;
    double period_max_ms_ = 0.0;
    double lateness_max_ms_ = 0.0;
    uint64_t skipped_cycles_ = 0;
    Clock::time_point window_start_;
    Clock::time_point previous_start_{};
    bool has_previous_start_ = false;
};

/**
 * @brief Detects a sampled monotonic value that stops advancing.
 *
 * Packet receipt and device-state progress are different signals. Repeated or
 * regressing values deliberately do not refresh the progress timestamp.
 */
class ProgressWatchdog {
public:
    void Reset() {
        latest_value_ = 0.0;
        last_progress_ = {};
        has_value_ = false;
    }

    void Observe(double value, Clock::time_point received_at) {
        if (!std::isfinite(value)) return;
        if (!has_value_) {
            latest_value_ = value;
            last_progress_ = received_at;
            has_value_ = true;
            return;
        }
        if (value > latest_value_) {
            latest_value_ = value;
            last_progress_ = received_at;
        }
    }

    bool HasValue() const { return has_value_; }

    double AgeSeconds(Clock::time_point now) const {
        if (!has_value_) return 0.0;
        return std::max(0.0,
            std::chrono::duration<double>(now - last_progress_).count());
    }

    bool Expired(Clock::time_point now, double timeout_s) const {
        return has_value_ && AgeSeconds(now) > timeout_s;
    }

private:
    double latest_value_ = 0.0;
    Clock::time_point last_progress_{};
    bool has_value_ = false;
};

}  // namespace runtime_timing

#endif  // RUNTIME_TIMING_H
