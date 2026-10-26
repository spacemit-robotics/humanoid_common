/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_bus_telemetry.cpp
 * @brief Snapshot handoff cost with the real diagnostic size and a blocked consumer.
 */
#include <time.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <stdexcept>
#include <vector>

#include "backends/whole_body_diagnostics.h"
#include "latest_sample_worker.h"

namespace {

struct BeforeBusSnapshot {
    whole_body_diagnostics_v2 feedback{};
    whole_body_motor_command_diagnostics_v2 command{};
    double wall_time = 0.0;
    bool render = false;
    bool record = false;
};

double ThreadCpuSeconds() {
    timespec time{};
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &time);
    return time.tv_sec + time.tv_nsec * 1e-9;
}

template <typename Sample>
void Measure(const char *name) {
    std::promise<void> entered, release;
    auto ready = entered.get_future();
    auto blocked = release.get_future();
    std::atomic<bool> first{true};
    runtime_logging::LatestSampleWorker<Sample> worker([&](const Sample &) {
        if (first.exchange(false)) {
            entered.set_value();
            blocked.wait();  // Simulate slow disk/CSV: producer must never wait here.
        }
    });
    Sample sample{};
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (!worker.Publish(sample) && std::chrono::steady_clock::now() < deadline)
        std::this_thread::yield();
    if (ready.wait_until(deadline) != std::future_status::ready) {
        release.set_value();
        throw std::runtime_error("telemetry worker did not start");
    }
    std::vector<double> times(10000);
    const double cpu_start = ThreadCpuSeconds();
    for (auto &time : times) {
        ++sample.wall_time;
        const auto start = std::chrono::steady_clock::now();
        (void)worker.Publish(sample);
        time = std::chrono::duration<double, std::micro>(
            std::chrono::steady_clock::now() - start).count();
    }
    const double cpu_us = (ThreadCpuSeconds() - cpu_start) * 1e6 / times.size();
    const auto dropped = worker.Dropped();
    release.set_value();
    worker.Stop();
    std::sort(times.begin(), times.end());
    std::cout << name << " bytes=" << sizeof(Sample) << " publish_cpu_us=" << cpu_us
        << " p50_us=" << times[times.size() / 2] << " p99_us=" << times[times.size() * 99 / 100]
        << " max_us=" << times.back() << " dropped=" << dropped << '\n';
    if (dropped < times.size() - 1 || worker.Failed())
        throw std::runtime_error("blocked consumer must drop snapshots without failing producer");
}

}  // namespace

int main() {
    try {
        Measure<BeforeBusSnapshot>("before_bus_counters");
        Measure<driver_runtime::WholeBodyDiagnosticsSnapshot>("with_bus_counters");
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
