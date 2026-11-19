// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "clock_anchor_server.h"

#include <unistd.h>
#include <zmq.h>

#include <chrono>
#include <cmath>
#include <future>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <utility>

#include "robot_base.h"
#include "runtime_logger.h"

namespace perception {
namespace {
double Mono() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

ClockAnchorServer::ClockAnchorServer(const std::string &endpoint, double receive_delay_bound_s)
    : endpoint_(endpoint),
        identity_(std::to_string(getpid()) + "-" + std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count())),
        receive_bound_(receive_delay_bound_s) {
    if (!std::isfinite(receive_bound_) || receive_bound_ <= 0 || receive_bound_ > .02)
        throw std::runtime_error("clock receive-delay bound must be in (0, 0.02] seconds");
    std::promise<void> ready;
    auto result = ready.get_future();
    thread_ = std::thread([this, ready = std::move(ready)]() mutable {
        void *context = nullptr, *socket = nullptr;
        bool started = false;
        try {
            robot_base::ThreadLoop::Background("clock_anchor").Apply();
            context = zmq_ctx_new();
            socket = context ? zmq_socket(context, ZMQ_REP) : nullptr;
            if (!socket) throw std::runtime_error("clock ZMQ initialization failed");
            int wait = 50, linger = 0, hwm = 4;
            int64_t maximum = 1024;
            zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));
            zmq_setsockopt(socket, ZMQ_RCVTIMEO, &wait, sizeof(wait));
            zmq_setsockopt(socket, ZMQ_SNDTIMEO, &wait, sizeof(wait));
            zmq_setsockopt(socket, ZMQ_RCVHWM, &hwm, sizeof(hwm));
            zmq_setsockopt(socket, ZMQ_SNDHWM, &hwm, sizeof(hwm));
            zmq_setsockopt(socket, ZMQ_MAXMSGSIZE, &maximum, sizeof(maximum));
            if (zmq_bind(socket, endpoint_.c_str()) != 0)
                throw std::runtime_error("clock endpoint bind failed: " + endpoint_);
            char actual[256];
            size_t size = sizeof(actual);
            if (zmq_getsockopt(socket, ZMQ_LAST_ENDPOINT, actual, &size) != 0)
                throw std::runtime_error("clock endpoint query failed");
            endpoint_ = actual;
            ready.set_value();
            started = true;
            while (!stop_) {
                char request[1024];
                const int length = zmq_recv(socket, request, sizeof(request), 0);
                const double t2 = Mono();
                if (length < 0) continue;
                auto query = length <= static_cast<int>(sizeof(request))
                    ? nlohmann::json::parse(request, request + length, nullptr, false)
                    : nlohmann::json{};
                Anchor anchor;
                {
                    std::lock_guard<std::mutex> lock(mutex_);
                    anchor = anchor_;
                }
                const bool valid_request = query.is_object() && query.contains("version") && query["version"] == 1 &&
                    query.contains("op") && query["op"] == "clock";
                const double age = t2 - anchor.receive;
                const bool valid = valid_request && anchor.valid && age >= 0 && age <= .2;
                nlohmann::json reply = {
                    {"version", 1}, {"valid", valid}, {"server_clock", "monotonic"},
                    {"t2_mono_s", t2}, {"anchor_robot_s", anchor.robot},
                    {"anchor_receive_mono_s", anchor.receive},
                    {"receive_delay_bound_s", receive_bound_},
                    {"session", identity_ + "-" + std::to_string(anchor.epoch)},
                    {"detail", valid ? "ready" : "invalid request or stale IMU anchor"}
                };
                reply["t3_mono_s"] = Mono();
                const auto bytes = reply.dump();
                if (zmq_send(socket, bytes.data(), bytes.size(), 0) < 0)
                    throw std::runtime_error("clock reply failed");
            }
        } catch (...) {
            if (!started) ready.set_exception(std::current_exception());
            else runtime_logging::Log(runtime_logging::Level::kError, "clock anchor service stopped", false);
        }
        if (socket) zmq_close(socket);
        if (context) zmq_ctx_term(context);
    });
    try {
        result.get();
    } catch (...) {
        stop_ = true;
        thread_.join();
        throw;
    }
}

ClockAnchorServer::~ClockAnchorServer() {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
}

void ClockAnchorServer::Observe(double robot_time, double receive_monotonic, bool fresh) {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (!lock.owns_lock()) return;
    const double age = Mono() - receive_monotonic;
    if (!fresh || !std::isfinite(robot_time) || robot_time < 0 ||
        !std::isfinite(receive_monotonic) || receive_monotonic <= 0 || age < 0 || age > .2) {
        anchor_.valid = false;
        return;
    }
    if (anchor_.receive > 0 && (robot_time < anchor_.robot || receive_monotonic < anchor_.receive))
        ++anchor_.epoch;
    anchor_.robot = robot_time;
    anchor_.receive = receive_monotonic;
    anchor_.valid = true;
}

void ClockAnchorServer::Invalidate() {
    std::unique_lock<std::mutex> lock(mutex_, std::try_to_lock);
    if (lock.owns_lock()) anchor_.valid = false;
}

}  // namespace perception
