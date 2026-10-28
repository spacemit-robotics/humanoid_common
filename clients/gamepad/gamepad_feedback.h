/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file gamepad_feedback.h
 * @brief Confirmed-request feedback and optional USB controller rumble worker
 */
#ifndef GAMEPAD_FEEDBACK_H
#define GAMEPAD_FEEDBACK_H
#include <fcntl.h>
#include <linux/hidraw.h>
#include <linux/input.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <array>
#include <chrono>
#include <condition_variable>
#include <cstdio>
#include <filesystem>  // NOLINT(build/c++17): SDK requires C++17.
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>
#include "operator_types.h"

namespace operator_gamepad {
struct FeedbackConfig {
    std::string device;
    int duration_ms = 120;
    int strength = 90;  // USB motor amplitude, 0..255.
};
class SuccessFeedback {
public:
    void Clear() { sequence_ = 0; }
    void Track(const std::string &operation, const std::string &target,
        const operator_service::Status &before, const operator_service::Reply &reply) {
        if (operation != "state" && operation != "policy") return;
        if (!reply.ok || reply.request.phase != "accepted" || !reply.request.sequence) return;
        Clear();
        if ((operation == "state" ? before.state : before.policy) == target) return;
        sequence_ = reply.request.sequence;
        operation_ = operation;
        target_ = target;
    }
    bool Update(const operator_service::Status &status) {
        if (!status.online) { Clear(); return false; }
        if (!sequence_ || status.request.sequence < sequence_) return false;
        if (status.request.sequence != sequence_ || status.request.operation != operation_) {
            Clear();
            return false;
        }
        if (status.request.phase == "accepted") return false;
        const bool success = status.request.phase == "completed" &&
            (operation_ == "state" ? status.state : status.policy) == target_;
        Clear();
        return success;
    }
private:
    uint64_t sequence_ = 0;
    std::string operation_, target_;
};

inline std::array<unsigned char, 32> Ds4RumbleReport(unsigned char strength) {
    // DualShock4 USB report 0x05: only enable the motor field, leaving LEDs/audio untouched.
    std::array<unsigned char, 32> report{};
    report[0] = 0x05;
    report[1] = 0x01;
    report[4] = strength;
    report[5] = strength;
    return report;
}
inline bool HasDs4Output(const unsigned char *data, size_t size) {
    struct Globals { unsigned id = 0, bits = 0, count = 0; } g;
    std::vector<Globals> stack;
    unsigned output_bits = 0;
    for (size_t i = 0; i < size;) {
        const auto tag = data[i++];
        if (tag == 0xfe) return false;
        const size_t length = (tag & 3) == 3 ? 4 : tag & 3;
        if (length > size - i) return false;
        unsigned value = 0;
        for (size_t j = 0; j < length; ++j) value |= unsigned(data[i++]) << (8 * j);
        switch (tag & 0xfc) {
        case 0x74: g.bits = value; break;
        case 0x84: g.id = value; break;
        case 0x94: g.count = value; break;
        case 0xa4: stack.push_back(g); break;
        case 0xb4:
            if (stack.empty()) return false;
            g = stack.back(); stack.pop_back(); break;
        case 0x90:
            if (g.id != 5) break;
            if (g.bits > 248 || g.count > 248 || g.bits * g.count > 248 - output_bits) return false;
            output_bits += g.bits * g.count;
            break;
        default: break;
        }
    }
    return output_bits == 248;
}

// A successful hidraw write acknowledges USB submission, not motor stopping.
// Send three independent zero reports so one dropped receiver report cannot latch rumble.
template <typename Send, typename Delay>
bool SendRumbleOff(Send send, Delay delay) {
    bool delivered = false;
    for (int attempt = 0; attempt < 3; ++attempt) {
        if (attempt) delay();
        delivered = send(0) || delivered;
    }
    return delivered;
}

struct RumbleSession {
    FeedbackConfig config;
    std::string input;
    int vendor = 0, product = 0;
    std::mutex mutex;
    std::condition_variable wake;
    bool stop = false, pulse = false;

    void Cancel() {
        std::lock_guard<std::mutex> lock(mutex);
        stop = true;
        pulse = false;
        wake.notify_one();
    }
};

class Rumble {
public:
    using Runner = std::function<void(const std::shared_ptr<RumbleSession> &)>;
    explicit Rumble(Runner run = RunSession) : state_(std::make_shared<WorkerState>(std::move(run))) {}
    Rumble(const Rumble &) = delete;
    Rumble &operator=(const Rumble &) = delete;
    ~Rumble() {
        Stop();
        std::unique_lock<std::mutex> lock(state_->mutex);
        state_->shutdown = true;
        state_->wake.notify_one();
        if (!worker_.joinable()) return;
        const bool finished = state_->wake.wait_for(lock, std::chrono::milliseconds(100),
            [&] { return state_->finished; });
        lock.unlock();
        if (finished) worker_.join();
        else worker_.detach();  // The worker owns its state and session, never this object.
    }
    void Start(const FeedbackConfig &config, const std::string &input, int vendor, int product) {
        Stop();
        if (config.device.empty()) return;
        auto session = std::make_shared<RumbleSession>();
        session->config = config;
        session->input = input;
        session->vendor = vendor;
        session->product = product;
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->pending = std::move(session);
        if (!worker_.joinable()) worker_ = std::thread(Work, state_);
        state_->wake.notify_one();
    }
    void Pulse() {
        std::lock_guard<std::mutex> lock(state_->mutex);
        if (!state_->active) return;
        std::lock_guard<std::mutex> session_lock(state_->active->mutex);
        if (state_->active->stop) return;
        state_->active->pulse = true;
        state_->active->wake.notify_one();
    }
    void Stop() {
        std::lock_guard<std::mutex> lock(state_->mutex);
        state_->pending.reset();
        if (state_->active) state_->active->Cancel();
    }

private:
    struct WorkerState {
        explicit WorkerState(Runner function) : run(std::move(function)) {}
        Runner run;
        std::mutex mutex;
        std::condition_variable wake;
        std::shared_ptr<RumbleSession> pending, active;
        bool shutdown = false, finished = false;
    };

    static void Work(const std::shared_ptr<WorkerState> &state) {
        std::unique_lock<std::mutex> lock(state->mutex);
        while (!state->shutdown) {
            state->wake.wait(lock, [&] { return state->shutdown || state->pending; });
            if (state->shutdown) break;
            state->active = std::move(state->pending);
            const auto session = state->active;
            lock.unlock();
            try {
                state->run(session);
            } catch (const std::exception &error) {
                std::fprintf(stderr, "[gamepad] vibration worker failed: %s\n", error.what());
            } catch (...) {
                std::fprintf(stderr, "[gamepad] vibration worker failed\n");
            }
            lock.lock();
            state->active.reset();
        }
        state->finished = true;
        state->wake.notify_one();
    }

    static void RunSession(const std::shared_ptr<RumbleSession> &session) {
        const auto &config = session->config;
        const auto &input = session->input;
        const int vendor = session->vendor, product = session->product;
        int fd = -1;
        bool verified = false;
        try {
            const auto raw_name = std::filesystem::canonical(config.device).filename();
            const auto hid = std::filesystem::canonical(
                std::filesystem::path("/sys/class/hidraw") / raw_name / "device");
            auto node = std::filesystem::canonical(std::filesystem::path("/sys/class/input") /
                std::filesystem::canonical(input).filename() / "device");
            while (node != hid && node != node.root_path()) node = node.parent_path();
            if (node != hid) throw std::runtime_error("rumble device is not the connected input device");
            fd = open(config.device.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
            hidraw_devinfo info{};
            int size = 0;
            hidraw_report_descriptor descriptor{};
            if (fd < 0 || ioctl(fd, HIDIOCGRAWINFO, &info) < 0 || info.bustype != BUS_USB ||
                static_cast<uint16_t>(info.vendor) != vendor || static_cast<uint16_t>(info.product) != product ||
                ioctl(fd, HIDIOCGRDESCSIZE, &size) < 0 || size <= 0 || size > HID_MAX_DESCRIPTOR_SIZE)
                throw std::runtime_error("rumble USB identity/access check failed");
            descriptor.size = size;
            if (ioctl(fd, HIDIOCGRDESC, &descriptor) < 0 || !HasDs4Output(descriptor.value, descriptor.size))
                throw std::runtime_error("rumble report 0x05/32 bytes is unavailable");
            verified = true;
            const auto send = [&](int strength) {
                if (strength) {
                    std::lock_guard<std::mutex> lock(session->mutex);
                    if (session->stop) return false;
                }
                const auto report = Ds4RumbleReport(static_cast<unsigned char>(strength));
                return write(fd, report.data(), report.size()) == static_cast<ssize_t>(report.size());
            };
            const auto off = [&] {
                return SendRumbleOff(send, [] {
                    std::this_thread::sleep_for(std::chrono::milliseconds(20));
                });
            };
            if (!off()) throw std::runtime_error("initial rumble reset failed");
            std::printf("[gamepad] rumble ready (USB report 0x05, %d ms)\n", config.duration_ms);
            std::unique_lock<std::mutex> lock(session->mutex);
            while (!session->stop) {
                // Receiver reconnects may retain a motor command even when
                // evdev remains open. An idle zero also clears that state.
                session->wake.wait_for(lock, std::chrono::seconds(1),
                    [&] { return session->stop || session->pulse; });
                if (session->stop) break;
                const bool pulse = session->pulse;
                session->pulse = false;
                lock.unlock();
                if (pulse) {
                    const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(config.duration_ms);
                    const bool sent = send(config.strength);
                    lock.lock();
                    if (sent) session->wake.wait_until(lock, deadline, [&] { return session->stop; });
                    // Coalesce requests while vibrating; never extend/replay a pulse.
                    session->pulse = false;
                    lock.unlock();
                    if (!sent) std::fprintf(stderr, "[gamepad] rumble start write failed; clearing motors\n");
                }
                if (!off()) std::fprintf(stderr, "[gamepad] rumble reset failed; retrying while idle\n");
                lock.lock();
            }
        } catch (const std::exception &e) {
            std::fprintf(stderr, "[gamepad] vibration unavailable: %s; input stays enabled\n", e.what());
        }
        if (fd >= 0) {
            const auto report = Ds4RumbleReport(0);
            if (verified && !SendRumbleOff([&](int) {
                    return write(fd, report.data(), report.size()) == static_cast<ssize_t>(report.size());
                }, [] { std::this_thread::sleep_for(std::chrono::milliseconds(20)); }))
                std::fprintf(stderr, "[gamepad] rumble stop unavailable\n");
            close(fd);
        }
    }
    std::shared_ptr<WorkerState> state_;
    std::thread worker_;
};
}  // namespace operator_gamepad
#endif  // GAMEPAD_FEEDBACK_H
