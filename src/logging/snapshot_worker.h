/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file snapshot_worker.h
 * @brief Preallocated non-waiting handoff to a background telemetry formatter.
 */
#ifndef SNAPSHOT_WORKER_H
#define SNAPSHOT_WORKER_H

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>

#include "robot_base.h"
#include "runtime_logger.h"

namespace runtime_logging {

template <typename Sample, size_t Capacity = 32>
class SnapshotWorker {
public:
    static_assert(std::is_trivially_copyable<Sample>::value, "snapshot must own plain values");
    explicit SnapshotWorker(std::function<void(const Sample &)> consume)
        : samples_(std::make_unique<std::array<Sample, Capacity>>()),
            consume_(std::move(consume)), thread_([this] { Run(); }) {}
    ~SnapshotWorker() { Stop(); }
    SnapshotWorker(const SnapshotWorker &) = delete;
    SnapshotWorker &operator=(const SnapshotWorker &) = delete;

    bool Publish(const Sample &sample) {
        std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
        if (!lock.owns_lock() || stopped_ || size_ == Capacity) {
            ++dropped_;
            ReportTelemetryDrop();
            return false;
        }
        (*samples_)[(head_ + size_) % Capacity] = sample;
        ++size_;
        lock.unlock();
        ready_.notify_one();
        return true;
    }
    void Stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
        }
        ready_.notify_one();
        if (thread_.joinable()) thread_.join();
    }
    uint64_t Dropped() const { return dropped_.load(); }

private:
    void Run() {
        robot_base::ThreadLoop::Background("log_format").Apply();
        for (;;) {
            Sample sample;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                ready_.wait(lock, [this] { return stopped_ || size_ != 0; });
                if (size_ == 0) return;
                sample = (*samples_)[head_];
                head_ = (head_ + 1) % Capacity;
                --size_;
            }
            try { consume_(sample); } catch (...) { ++dropped_; ReportTelemetryDrop(); }
        }
    }
    std::unique_ptr<std::array<Sample, Capacity>> samples_;
    std::function<void(const Sample &)> consume_;
    std::mutex mutex_;
    std::condition_variable ready_;
    size_t head_ = 0, size_ = 0;
    bool stopped_ = false;
    std::atomic<uint64_t> dropped_{0};
    std::thread thread_;
};

}  // namespace runtime_logging
#endif  // SNAPSHOT_WORKER_H
