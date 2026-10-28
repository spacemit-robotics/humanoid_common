/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file latest_sample_worker.h
 * @brief Bounded, non-waiting snapshot handoff for non-control telemetry work.
 */
#ifndef LATEST_SAMPLE_WORKER_H
#define LATEST_SAMPLE_WORKER_H

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>

#include "robot_base.h"

namespace runtime_logging {

template <typename Sample>
class LatestSampleWorker {
public:
    static_assert(std::is_trivially_copyable<Sample>::value,
        "snapshot must own plain values, with no driver references or heap allocation");
    explicit LatestSampleWorker(std::function<void(const Sample &)> consume)
        : consume_(std::move(consume)), worker_([this] { Run(); }) {}

    ~LatestSampleWorker() { Stop(); }
    LatestSampleWorker(const LatestSampleWorker &) = delete;
    LatestSampleWorker &operator=(const LatestSampleWorker &) = delete;

    bool Publish(const Sample &sample) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock() || stopping_ || failed_.load()) {
            ++dropped_;
            return false;
        }
        if (pending_) ++dropped_;
        sample_ = sample;
        pending_ = true;
        lock.unlock();
        ready_.notify_one();
        return true;
    }

    // Stop drains the last accepted snapshot; the owner outlives the consumer.
    void Stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        ready_.notify_one();
        if (worker_.joinable()) worker_.join();
    }
    uint64_t Dropped() const { return dropped_.load(); }
    bool Failed() const { return failed_.load(); }

private:
    void Run() {
        robot_base::ThreadLoop::Background("telemetry").Apply();
        for (;;) {
            Sample current;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this] { return pending_ || stopping_; });
                if (!pending_) return;
                current = sample_;
                pending_ = false;
            }
            try {
                consume_(current);
            } catch (...) {
                failed_ = true;
                ++dropped_;
                return;
            }
        }
    }

    std::function<void(const Sample &)> consume_;
    std::mutex mutex_;
    std::condition_variable ready_;
    Sample sample_{};
    bool pending_ = false;
    bool stopping_ = false;
    std::atomic<uint64_t> dropped_{0};
    std::atomic<bool> failed_{false};
    std::thread worker_;
};

}  // namespace runtime_logging

#endif  // LATEST_SAMPLE_WORKER_H
