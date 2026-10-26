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
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
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

class Rumble {
public:
    ~Rumble() { Stop(); }
    void Start(const FeedbackConfig &config, const std::string &input, int vendor, int product) {
        Stop();
        if (config.device.empty()) return;
        stop_ = false;
        worker_ = std::thread([this, config, input, vendor, product] {
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
                std::unique_lock<std::mutex> lock(mutex_);
                while (!stop_) {
                    // Receiver reconnects may retain a motor command even when
                    // evdev remains open. An idle zero also clears that state.
                    wake_.wait_for(lock, std::chrono::seconds(1), [&] { return stop_ || pulse_; });
                    if (stop_) break;
                    const bool pulse = pulse_;
                    pulse_ = false;
                    lock.unlock();
                    if (pulse) {
                        const auto deadline = std::chrono::steady_clock::now() +
                            std::chrono::milliseconds(config.duration_ms);
                        const bool sent = send(config.strength);
                        lock.lock();
                        if (sent) wake_.wait_until(lock, deadline, [&] { return stop_; });
                        // Coalesce requests while vibrating; never extend/replay a pulse.
                        pulse_ = false;
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
        });
    }
    void Pulse() {
        std::lock_guard<std::mutex> lock(mutex_);
        pulse_ = true;
        wake_.notify_one();
    }
    void Stop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
            pulse_ = false;
            wake_.notify_one();
        }
        if (worker_.joinable()) worker_.join();
    }
private:
    std::mutex mutex_;
    std::condition_variable wake_;
    std::thread worker_;
    bool stop_ = true, pulse_ = false;
};
}  // namespace operator_gamepad
#endif  // GAMEPAD_FEEDBACK_H
