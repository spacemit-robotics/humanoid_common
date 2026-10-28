/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file evdev_device.h
 * @brief Read-only hotplug gamepad input; no HID writes or exclusive grab
 */
#ifndef EVDEV_DEVICE_H
#define EVDEV_DEVICE_H
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17): the SDK requires C++17.
#include <string>
#include <vector>
#include "gamepad_input.h"

namespace operator_gamepad {
class Device {
public:
    ~Device() { Close(); }
    Device() = default;
    Device(const Device &) = delete;
    Device &operator=(const Device &) = delete;
    bool Opened() const { return fd_ >= 0; }
    void Close() {
        if (fd_ >= 0) close(fd_);
        fd_ = -1;
    }
    void Open(const std::string &path, int vendor, int product, const std::string &layout, const Layout &input,
        double deadzone, Chords *chords) {
        Close();
        fd_ = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
        if (fd_ < 0) throw std::runtime_error("waiting for gamepad: " + std::string(std::strerror(errno)));
        try {
            input_id identity{};
            if (ioctl(fd_, EVIOCGID, &identity) < 0 ||
                identity.vendor != vendor || identity.product != product)
                throw std::runtime_error("gamepad identity does not match configuration");
            if (layout == "ds4_generic") {
                auto parent = std::filesystem::canonical(path).filename();
                auto node = std::filesystem::canonical(std::filesystem::path("/sys/class/input") / parent / "device");
                std::string driver;
                while (node != node.root_path()) {
                    if (std::filesystem::is_symlink(node / "driver")) {
                        driver = std::filesystem::canonical(node / "driver").filename();
                        break;
                    }
                    node = node.parent_path();
                }
                if (driver != "hid-generic")
                    throw std::runtime_error("ds4_generic requires the verified hid-generic key layout");
            }
            layout_ = input;
            deadzone_ = deadzone;
            Snapshot();
            chords->Reset(held_);
            dropped_ = false;
            report_pending_ = false;
        } catch (...) {
            Close();
            throw;
        }
    }
    std::vector<Binding> Read(Chords *chords, bool *resynced) {
        *resynced = false;
        pollfd item{fd_, POLLIN, 0};
        if (poll(&item, 1, 0) < 0 && errno != EINTR) throw std::runtime_error("gamepad poll failed");
        if (item.revents & (POLLHUP | POLLERR | POLLNVAL)) throw std::runtime_error("gamepad disconnected");
        std::vector<Binding> commands;
        input_event events[64];
        // Drain queued events before commanding, retaining the latest complete state.
        for (int batch = 0; batch < 32; ++batch) {
            const auto bytes = read(fd_, events, sizeof(events));
            if (bytes < 0 && errno == EINTR) continue;
            if (bytes < 0 && errno == EAGAIN) {
                if (!dropped_ && !report_pending_) {
                    const auto held_commands = chords->Poll();
                    commands.insert(commands.end(), held_commands.begin(), held_commands.end());
                }
                return commands;
            }
            if (bytes <= 0 || bytes % sizeof(input_event)) throw std::runtime_error("gamepad disconnected");
            for (size_t i = 0; i < static_cast<size_t>(bytes) / sizeof(input_event); ++i) {
                const auto &event = events[i];
                if (event.type != EV_SYN) report_pending_ = true;
                if (event.type == EV_SYN && event.code == SYN_DROPPED) {
                    dropped_ = true;
                    chords->Reset(held_);
                    velocity_ = {};
                    *resynced = true;
                    commands.clear();
                } else if (dropped_) {
                    if (event.type == EV_SYN && event.code == SYN_REPORT) {
                        Snapshot();
                        chords->Reset(held_);
                        dropped_ = false;
                        report_pending_ = false;
                        *resynced = true;
                    }
                } else if (event.type == EV_KEY && event.value != 2) {
                    const auto it = layout_.buttons.find(event.code);
                    if (it != layout_.buttons.end()) {
                        if (event.value) held_.insert(it->second);
                        else held_.erase(it->second);
                    }
                } else if (event.type == EV_ABS) {
                    if (axes_.count(event.code)) axes_[event.code].value = event.value;
                    if (event.code == ABS_HAT0X || event.code == ABS_HAT0Y) Hat(event.code, event.value);
                } else if (event.type == EV_SYN && event.code == SYN_REPORT) {
                    report_pending_ = false;
                    velocity_ = {-Axis(layout_.left_y), -Axis(layout_.left_x), -Axis(layout_.right_x)};
                    const auto edges = chords->Update(held_);
                    if (commands.size() < 8) commands.insert(commands.end(), edges.begin(), edges.end());
                }
            }
        }
        Snapshot();
        chords->Reset(held_);
        report_pending_ = false;
        *resynced = true;
        return {};  // Never replay commands from a backlogged input queue.
    }
    operator_service::Velocity Velocity() const { return velocity_; }
private:
    double Axis(int code) const {
        const auto &a = axes_.at(code);
        return Normalize(a.value, a.minimum, a.maximum, deadzone_);
    }
    void Hat(int code, int value) {
        const std::string negative = code == ABS_HAT0X ? "LEFT" : "UP";
        const std::string positive = code == ABS_HAT0X ? "RIGHT" : "DOWN";
        held_.erase(negative);
        held_.erase(positive);
        if (value < 0) held_.insert(negative);
        if (value > 0) held_.insert(positive);
    }
    void Snapshot() {
        unsigned char keys[(KEY_MAX + 8) / 8]{};
        if (ioctl(fd_, EVIOCGKEY(sizeof(keys)), keys) < 0) throw std::runtime_error("cannot read gamepad keys");
        held_.clear();
        for (const auto &[code, name] : layout_.buttons)
            if (keys[code / 8] & (1 << (code % 8))) held_.insert(name);
        for (const auto code : {layout_.left_x, layout_.left_y, layout_.right_x}) {
            input_absinfo axis{};
            if (ioctl(fd_, EVIOCGABS(code), &axis) < 0 || axis.minimum >= axis.maximum)
                throw std::runtime_error("gamepad axis unavailable");
            axes_[code] = axis;
        }
        for (const auto code : {ABS_HAT0X, ABS_HAT0Y}) {
            input_absinfo axis{};
            if (ioctl(fd_, EVIOCGABS(code), &axis) == 0) Hat(code, axis.value);
        }
        velocity_ = {-Axis(layout_.left_y), -Axis(layout_.left_x), -Axis(layout_.right_x)};
    }
    int fd_ = -1;
    bool dropped_ = false;
    bool report_pending_ = false;
    double deadzone_ = 0.06;
    Layout layout_;
    std::map<int, input_absinfo> axes_;
    std::set<std::string> held_;
    operator_service::Velocity velocity_;
};
}  // namespace operator_gamepad
#endif  // EVDEV_DEVICE_H
