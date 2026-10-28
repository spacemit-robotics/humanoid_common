/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file input_connection.h
 * @brief Read-only input-device presence observer for connection announcements
 */
#ifndef INPUT_CONNECTION_H
#define INPUT_CONNECTION_H
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <string>
#include <utility>

namespace operator_audio {
class InputConnection {
public:
    InputConnection(std::string path, int vendor, int product)
        : path_(std::move(path)), vendor_(vendor), product_(product) {}
    ~InputConnection() { Close(); }
    InputConnection(const InputConnection &) = delete;
    InputConnection &operator=(const InputConnection &) = delete;
    bool Connected() {
        if (path_.empty()) return false;
        if (fd_ < 0) {
            const auto now = std::chrono::steady_clock::now();
            if (now < next_open_) return false;
            next_open_ = now + std::chrono::seconds(1);
            fd_ = open(path_.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
            if (fd_ < 0) return false;
            input_id identity{};
            if (ioctl(fd_, EVIOCGID, &identity) < 0 ||
                identity.vendor != vendor_ || identity.product != product_) {
                Close();
                return false;
            }
        }
        // No input reads or grab: the operator client retains its own event queue.
        // No POLLIN interest: only device removal/error, not idle input, is a disconnect.
        pollfd device{fd_, 0, 0};
        const auto result = poll(&device, 1, 0);
        if ((result < 0 && errno != EINTR) ||
            (device.revents & (POLLHUP | POLLERR | POLLNVAL))) Close();
        return fd_ >= 0;
    }

private:
    void Close() {
        if (fd_ >= 0) close(fd_);
        fd_ = -1;
    }
    std::string path_;
    int vendor_, product_, fd_ = -1;
    std::chrono::steady_clock::time_point next_open_{};
};
} // namespace operator_audio
#endif // INPUT_CONNECTION_H
