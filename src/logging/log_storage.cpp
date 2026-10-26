/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file log_storage.cpp
 * @brief Bounded log storage with active-session leases.
 */
#include "log_storage.h"

#include <fcntl.h>
#include <signal.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <algorithm>
#include <fstream>
#include <stdexcept>
#include <system_error>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

namespace runtime_logging {
namespace {
constexpr uint64_t kMb = 1024 * 1024;

uint64_t DirectoryBytes(const std::filesystem::path &path) {
    uint64_t size = 0;
    for (const auto &entry : std::filesystem::recursive_directory_iterator(path)) {
        if (entry.is_regular_file() && !entry.is_symlink()) size += entry.file_size();
    }
    return size;
}

int Lock(const std::filesystem::path &path) {
    const int fd = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0660);
    if (fd >= 0) (void)fchmod(fd, 0660);
    if (fd >= 0 && flock(fd, LOCK_EX | LOCK_NB) == 0) return fd;
    if (fd >= 0) close(fd);
    return -1;
}

bool OwnedSession(const std::filesystem::path &path, const std::string &robot) {
    for (const auto *name : {"metadata.json", "launcher.json"}) {
        const auto file = path / name;
        if (!std::filesystem::is_regular_file(file) || std::filesystem::is_symlink(file))
            continue;
        try {
            const auto metadata = YAML::LoadFile(file.string());
            if (metadata["robot"].as<std::string>("") != robot) return false;
            // Old sessions predate leases. A live PID is conservatively retained.
            if (!std::filesystem::exists(path / ".active.lock")) {
                const int pid = metadata["pid"].as<int>(0);
                if (pid > 0 && kill(pid, 0) == 0) return false;
            }
            return true;
        } catch (...) { return false; }
    }
    return false;
}
}  // namespace

LogStorage::LogStorage(const std::string &root, const std::string &session,
        const std::string &robot, int max_total_mb, int min_free_mb)
    : root_(root), session_(session), robot_(robot),
        max_bytes_(static_cast<uint64_t>(max_total_mb) * kMb),
        min_free_(static_cast<uint64_t>(min_free_mb) * kMb) {
    lease_ = Lock(session_ / ".active.lock");
    if (lease_ < 0) throw std::runtime_error("cannot protect active log session");
}

LogStorage::~LogStorage() {
    if (lease_ >= 0) close(lease_);
}

void LogStorage::Refresh() {
    const auto now = std::chrono::steady_clock::now();
    if (now - checked_ < std::chrono::seconds(1) && written_ < kMb) return;
    checked_ = now;
    written_ = 0;
    if (max_bytes_ == 0 && min_free_ == 0) return;
    const int guard = Lock(root_ / ("." + robot_ + ".retention.lock"));
    if (guard < 0) return;
    try {
        std::vector<std::pair<std::filesystem::file_time_type, std::filesystem::path>> old;
        used_ = 0;
        for (const auto &entry : std::filesystem::directory_iterator(root_)) {
            if (!entry.is_directory() || entry.is_symlink() ||
                entry.path().filename().string().rfind(robot_ + "_", 0) != 0) continue;
            used_ += DirectoryBytes(entry.path());
            if (entry.path() != session_ && OwnedSession(entry.path(), robot_))
                old.emplace_back(entry.last_write_time(), entry.path());
        }
        // Completed captures can be retired without removing an active session.
        for (const auto &entry : std::filesystem::directory_iterator(session_)) {
            if (entry.is_directory() && !entry.is_symlink() &&
                entry.path().filename().string().rfind("incident_", 0) == 0 &&
                std::filesystem::is_regular_file(entry.path() / ".complete"))
                old.emplace_back(entry.last_write_time(), entry.path());
        }
        std::sort(old.begin(), old.end());
        free_ = std::filesystem::space(root_).available;
        const auto margin = max_bytes_ ? std::min(max_bytes_ / 8, 16 * kMb) : 0;
        const auto over = [&] {
            return (max_bytes_ && used_ + margin >= max_bytes_) || free_ < min_free_;
        };
        for (const auto &candidate : old) {
            if (!over()) break;
            const int active = Lock(candidate.second / ".active.lock");
            if (active < 0) continue;
            uint64_t size = 0;
            try { size = DirectoryBytes(candidate.second); }
            catch (...) { close(active); continue; }
            std::error_code error;
            std::filesystem::remove_all(candidate.second, error);
            close(active);
            if (!error) used_ -= std::min(used_, size);
            free_ = std::filesystem::space(root_).available;
        }
        writable_ = !over();
    } catch (...) { writable_ = false; }
    close(guard);
}

bool LogStorage::Allow(size_t bytes, bool essential) {
    Refresh();
    // Small events retain a bounded reserve while bulk telemetry is suspended.
    const bool allowed = (max_bytes_ == 0 && min_free_ == 0) ||
        (essential ? free_ > 1024 * 1024 && (!max_bytes_ || used_ + bytes < max_bytes_)
            : writable_ && free_ > min_free_ + bytes);
    if (!allowed) { ++suppressed_; return false; }
    written_ += bytes;
    used_ += bytes;
    free_ -= std::min<uint64_t>(free_, bytes);
    return true;
}
}  // namespace runtime_logging
