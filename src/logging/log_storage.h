/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file log_storage.h
 * @brief Background-only disk budgeting and inactive session retention.
 */
#ifndef LOG_STORAGE_H
#define LOG_STORAGE_H

#include <chrono>
#include <cstdint>
#include <filesystem>  // NOLINT(build/c++17)
#include <string>

namespace runtime_logging {

class LogStorage {
public:
    LogStorage(const std::string &root, const std::string &session,
        const std::string &robot, int max_total_mb, int min_free_mb);
    ~LogStorage();
    LogStorage(const LogStorage &) = delete;
    LogStorage &operator=(const LogStorage &) = delete;
    bool Allow(size_t bytes, bool essential);
    void Refresh();
    uint64_t Suppressed() const { return suppressed_; }

private:
    std::filesystem::path root_, session_;
    std::string robot_;
    int lease_ = -1;
    uint64_t max_bytes_ = 0, min_free_ = 0;
    uint64_t used_ = 0, free_ = 0, written_ = 0, suppressed_ = 0;
    bool writable_ = true;
    std::chrono::steady_clock::time_point checked_{};
};

}  // namespace runtime_logging
#endif  // LOG_STORAGE_H
