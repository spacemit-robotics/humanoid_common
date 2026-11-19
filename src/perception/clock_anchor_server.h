// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#ifndef CLOCK_ANCHOR_SERVER_H
#define CLOCK_ANCHOR_SERVER_H

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace perception {

// Read-only time RPC. Observe never waits for the network thread.
class ClockAnchorServer {
public:
    ClockAnchorServer(const std::string &endpoint, double receive_delay_bound_s);
    ~ClockAnchorServer();
    void Observe(double robot_time, double receive_monotonic, bool fresh);
    void Invalidate();
    const std::string &Endpoint() const { return endpoint_; }

private:
    struct Anchor {
        double robot = 0;
        double receive = 0;
        bool valid = false;
        uint64_t epoch = 0;
    };
    std::string endpoint_;
    std::string identity_;
    double receive_bound_;
    std::atomic<bool> stop_{false};
    std::mutex mutex_;
    Anchor anchor_;
    std::thread thread_;
};

}  // namespace perception
#endif  // CLOCK_ANCHOR_SERVER_H
