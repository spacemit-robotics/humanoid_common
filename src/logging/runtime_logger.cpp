/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file runtime_logger.cpp
 * @brief Internal asynchronous runtime logger implementation
 */

#include "runtime_logger.h"

#include <fcntl.h>
#include <spawn.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <cerrno>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include <deque>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "log_storage.h"

extern char **environ;

#ifndef HUMANOID_COMMON_GIT_COMMIT
#define HUMANOID_COMMON_GIT_COMMIT "unknown"
#endif

#ifndef HUMANOID_COMMON_GIT_DIRTY
#define HUMANOID_COMMON_GIT_DIRTY 0
#endif

namespace runtime_logging {
namespace {

std::atomic<uint64_t> formatter_drops{0};

class FileLock {
public:
    explicit FileLock(const std::filesystem::path &path) {
        fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0660);
        if (fd_ >= 0) (void)fchmod(fd_, 0660);
        if (fd_ >= 0 && flock(fd_, LOCK_EX | LOCK_NB) != 0) {
            close(fd_);
            fd_ = -1;
        }
    }
    ~FileLock() { if (fd_ >= 0) close(fd_); }
    bool Locked() const { return fd_ >= 0; }
private:
    int fd_ = -1;
};

struct QueueItem {
    std::string stream;
    std::string header;
    std::string line;
    bool csv = false;
    enum class Kind {
        kRecord,
        kArtifact,
    } kind = Kind::kRecord;
    std::string artifact_kind;
    std::string artifact_name;
    std::string artifact_path;
    bool emit_console = false;
    bool console_error = false;
    bool trigger = false;
    double monotonic_s = 0.0;
};

struct FileState {
    std::ofstream stream;
    std::chrono::steady_clock::time_point last_flush;
    uint64_t bytes = 0;
    std::string header;
    bool csv = false;
    bool opened_once = false;
    std::chrono::steady_clock::time_point retry_after{};
};

double MonotonicSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

std::string BootId() {
    std::ifstream input("/proc/sys/kernel/random/boot_id");
    std::string id;
    input >> id;
    return id;
}

std::string LevelName(Level level) {
    switch (level) {
        case Level::kDebug:
            return "DEBUG";
        case Level::kInfo:
            return "INFO";
        case Level::kWarning:
            return "WARN";
        case Level::kError:
            return "ERROR";
    }
    return "UNKNOWN";
}

Level ParseLevel(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    if (value == "debug") return Level::kDebug;
    if (value == "warning" || value == "warn") return Level::kWarning;
    if (value == "error") return Level::kError;
    return Level::kInfo;
}

double ReadFiniteClamped(const robot_base::YamlFile &yaml_file,
        const std::string &path, double fallback, double minimum, double maximum) {
    const double value = yaml_file.Read<double>(path).value_or(fallback);
    if (!std::isfinite(value)) {
        throw std::runtime_error(path + " must be finite");
    }
    return std::clamp(value, minimum, maximum);
}

std::string Timestamp(const char *format, bool milliseconds) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t value = std::chrono::system_clock::to_time_t(now);
    std::tm local{};
    localtime_r(&value, &local);
    std::ostringstream output;
    output << std::put_time(&local, format);
    if (milliseconds) {
        const auto fraction = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        output << "." << std::setw(3) << std::setfill('0') << fraction.count();
    }
    return output.str();
}

std::string Sanitize(std::string value) {
    for (char &c : value) {
        const bool valid = std::isalnum(static_cast<unsigned char>(c)) ||
            c == '_' || c == '-';
        if (!valid) c = '_';
    }
    return value.empty() ? "runtime" : value;
}

std::string JsonEscape(const std::string &value) {
    std::string output;
    output.reserve(value.size());
    for (char c : value) {
        if (c == '\\' || c == '"') output.push_back('\\');
        if (static_cast<unsigned char>(c) < 0x20) {
            constexpr char hex[] = "0123456789abcdef";
            output += "\\u00";
            output += hex[(static_cast<unsigned char>(c) >> 4) & 15];
            output += hex[static_cast<unsigned char>(c) & 15];
        } else {
            output.push_back(c);
        }
    }
    return output;
}

std::string CsvEscape(const std::string &value) {
    if (value.find_first_of(",\"\r\n") == std::string::npos) return value;
    std::string output = "\"";
    for (char c : value) {
        if (c == '"') output.push_back('"');
        output.push_back(c);
    }
    output.push_back('"');
    return output;
}

struct CsvShape {
    bool valid = false;
    size_t columns = 0;
    size_t records = 0;
};

CsvShape AnalyzeCsv(const std::string &value) {
    if (value.empty()) return {};

    CsvShape result;
    size_t columns = 1;
    bool in_quotes = false;
    bool record_has_data = false;
    const auto finish_record = [&]() {
        if (!record_has_data && columns == 1) return false;
        if (result.records == 0) {
            result.columns = columns;
        } else if (result.columns != columns) {
            return false;
        }
        ++result.records;
        columns = 1;
        record_has_data = false;
        return true;
    };

    for (size_t i = 0; i < value.size(); ++i) {
        const char character = value[i];
        if (character == '"') {
            record_has_data = true;
            if (in_quotes && i + 1 < value.size() && value[i + 1] == '"') {
                ++i;
            } else {
                in_quotes = !in_quotes;
            }
            continue;
        }
        if (!in_quotes && character == ',') {
            ++columns;
            record_has_data = true;
            continue;
        }
        if (!in_quotes && (character == '\n' || character == '\r')) {
            if (!finish_record()) return {};
            if (character == '\r' && i + 1 < value.size() && value[i + 1] == '\n') ++i;
            continue;
        }
        record_has_data = true;
    }
    if (in_quotes) return {};
    if (record_has_data || columns > 1) {
        if (!finish_record()) return {};
    }
    result.valid = result.records > 0;
    return result;
}

std::string CaptureCommand(const std::vector<std::string> &arguments) {
    if (arguments.empty()) return {};
    int output_pipe[2];
    if (pipe(output_pipe) != 0) return {};

    posix_spawn_file_actions_t actions;
    if (posix_spawn_file_actions_init(&actions) != 0) {
        close(output_pipe[0]);
        close(output_pipe[1]);
        return {};
    }
    posix_spawn_file_actions_adddup2(&actions, output_pipe[1], STDOUT_FILENO);
    posix_spawn_file_actions_addclose(&actions, output_pipe[0]);
    posix_spawn_file_actions_addclose(&actions, output_pipe[1]);
    posix_spawn_file_actions_addopen(
        &actions, STDERR_FILENO, "/dev/null", O_WRONLY, 0);

    std::vector<char *> argv;
    argv.reserve(arguments.size() + 1);
    for (const auto &argument : arguments)
        argv.push_back(const_cast<char *>(argument.c_str()));
    argv.push_back(nullptr);

    pid_t child = -1;
    const int spawn_result = posix_spawnp(&child, arguments[0].c_str(),
        &actions, nullptr, argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    close(output_pipe[1]);
    if (spawn_result != 0) {
        close(output_pipe[0]);
        return {};
    }

    std::string output;
    char buffer[512];
    for (;;) {
        const ssize_t count = read(output_pipe[0], buffer, sizeof(buffer));
        if (count > 0) {
            output.append(buffer, static_cast<size_t>(count));
            continue;
        }
        if (count < 0 && errno == EINTR) continue;
        break;
    }
    close(output_pipe[0]);

    int status = 0;
    while (waitpid(child, &status, 0) < 0 && errno == EINTR) {}
    if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) return {};
    while (!output.empty() &&
        (output.back() == '\n' || output.back() == '\r')) {
        output.pop_back();
    }
    return output;
}

std::string Sha256File(const std::string &path) {
    const std::string output = CaptureCommand({"sha256sum", "--", path});
    if (output.size() < 64) return {};
    const std::string digest = output.substr(0, 64);
    const bool valid = std::all_of(digest.begin(), digest.end(), [](unsigned char c) {
        return std::isxdigit(c) != 0;
    });
    return valid ? digest : std::string{};
}

struct GitInfo {
    std::string commit = "unknown";
    bool dirty = false;
};

GitInfo GitInfoForPath(const std::string &path) {
    std::error_code error;
    std::filesystem::path directory = std::filesystem::absolute(path, error);
    if (error) return {};
    if (!std::filesystem::is_directory(directory, error)) directory = directory.parent_path();
    const std::string root = CaptureCommand(
        {"git", "-C", directory.string(), "rev-parse", "--show-toplevel"});
    if (root.empty()) return {};

    GitInfo result;
    const std::string commit = CaptureCommand(
        {"git", "-C", root, "rev-parse", "HEAD"});
    if (!commit.empty()) result.commit = commit;
    result.dirty = !CaptureCommand(
        {"git", "-C", root, "status", "--porcelain"}).empty();
    return result;
}

gid_t RuntimeLogGroup() {
    if (geteuid() != 0) return getegid();

    const char *sudo_gid = std::getenv("SUDO_GID");
    if (sudo_gid == nullptr || *sudo_gid == '\0') return getegid();

    errno = 0;
    char *end = nullptr;
    const unsigned long value = std::strtoul(sudo_gid, &end, 10);
    if (errno != 0 || end == sudo_gid || *end != '\0' ||
        value > std::numeric_limits<gid_t>::max()) {
        return getegid();
    }
    return static_cast<gid_t>(value);
}

void PrepareDirectory(const std::filesystem::path &path) {
    std::filesystem::create_directories(path);

    struct stat info {};
    if (stat(path.c_str(), &info) != 0) {
        throw std::filesystem::filesystem_error(
            "failed to inspect log directory", path, std::error_code(errno,
                std::generic_category()));
    }

    if (geteuid() == 0 &&
        chown(path.c_str(), static_cast<uid_t>(-1), RuntimeLogGroup()) != 0) {
        throw std::filesystem::filesystem_error(
            "failed to set log directory group", path, std::error_code(errno,
                std::generic_category()));
    }

    if (geteuid() == 0 || info.st_uid == geteuid()) {
        std::filesystem::permissions(path,
            std::filesystem::perms::owner_all | std::filesystem::perms::group_all |
                std::filesystem::perms::set_gid,
            std::filesystem::perm_options::replace);
        return;
    }

    if (access(path.c_str(), W_OK | X_OK) != 0) {
        throw std::filesystem::filesystem_error(
            "log directory is not writable", path, std::error_code(errno,
                std::generic_category()));
    }
}

bool PrepareLogFile(const std::filesystem::path &path) {
    bool prepared = true;
    if (geteuid() == 0 &&
        chown(path.c_str(), static_cast<uid_t>(-1), RuntimeLogGroup()) != 0) {
        std::cerr << "[runtime_logger] failed to set log file group: "
            << path << ": " << std::strerror(errno) << "\n";
        prepared = false;
    }
    std::error_code error;
    std::filesystem::permissions(path,
        std::filesystem::perms::owner_read | std::filesystem::perms::owner_write |
            std::filesystem::perms::group_read | std::filesystem::perms::group_write,
        std::filesystem::perm_options::replace, error);
    if (error) {
        std::cerr << "[runtime_logger] failed to set log file permissions: "
            << path << ": " << error.message() << "\n";
        prepared = false;
    }
    return prepared;
}

uint64_t HashFile(const std::string &path) {
    std::ifstream input(path, std::ios::binary);
    uint64_t hash = 14695981039346656037ULL;
    char buffer[4096];
    while (input.read(buffer, sizeof(buffer)) || input.gcount() > 0) {
        for (std::streamsize i = 0; i < input.gcount(); ++i) {
            hash ^= static_cast<unsigned char>(buffer[i]);
            hash *= 1099511628211ULL;
        }
    }
    return hash;
}

class Logger {
public:
    static Logger &Instance() {
        static Logger logger;
        return logger;
    }

    void Initialize(const robot_base::YamlFile &yaml_file,
        const std::string &yaml_path, const std::string &component,
        bool console_allowed) {
        Shutdown();

        Config config;
        config.level = ParseLevel(
            yaml_file.Read<std::string>("logging.level").value_or("info"));
        config.console_enabled = console_allowed &&
            yaml_file.Read<bool>("logging.console.enabled").value_or(true);
        config.file_enabled =
            yaml_file.Read<bool>("logging.file.enabled").value_or(false);
        config.telemetry_enabled = config.file_enabled &&
            yaml_file.Read<bool>("logging.telemetry.enabled").value_or(false);
        config.driver_monitor_enabled = console_allowed &&
            yaml_file.Read<bool>("logging.driver_monitor.enabled").value_or(false);
        config.telemetry_rate_hz = ReadFiniteClamped(yaml_file,
            "logging.telemetry.rate_hz", 20.0, 0.1, 500.0);
        config.control_telemetry_rate_hz = ReadFiniteClamped(yaml_file,
            "logging.telemetry.control_rate_hz", config.telemetry_rate_hz,
            0.1, 500.0);
        config.hardware_telemetry_rate_hz = ReadFiniteClamped(yaml_file,
            "logging.telemetry.hardware_rate_hz", config.telemetry_rate_hz,
            0.1, 500.0);
        config.driver_monitor_rate_hz = ReadFiniteClamped(yaml_file,
            "logging.driver_monitor.rate_hz", 2.0, 0.1, 20.0);
        config.timing_window_s = ReadFiniteClamped(yaml_file,
            "logging.telemetry.timing_window_s", 5.0, 1.0, 15.0);
        config.queue_capacity = std::clamp(
            yaml_file.Read<int>("logging.queue_capacity").value_or(4096),
            64, 65536);
        config.max_file_size_mb = std::clamp(
            yaml_file.Read<int>("logging.file.max_size_mb").value_or(64),
            1, 4096);
        config.max_files = std::clamp(
            yaml_file.Read<int>("logging.file.max_files").value_or(4),
            1, 32);
        config.blackbox_enabled = yaml_file.Read<bool>("logging.blackbox.enabled").value_or(false);
        config.policy_trace_enabled = yaml_file.Read<bool>("logging.telemetry.policy_enabled")
            .value_or(config.level == Level::kDebug);
        config.idle_rate_hz = ReadFiniteClamped(yaml_file,
            "logging.telemetry.idle_rate_hz", 0.0, 0.0, 500.0);
        config.continuous_rate_hz = ReadFiniteClamped(yaml_file,
            "logging.telemetry.continuous_rate_hz", 2.0, 0.1, 500.0);
        config.pre_trigger_s = ReadFiniteClamped(yaml_file,
            "logging.blackbox.pre_trigger_s", 30.0, 0.0, 120.0);
        config.post_trigger_s = ReadFiniteClamped(yaml_file,
            "logging.blackbox.post_trigger_s", 10.0, 0.0, 120.0);
        config.trigger_cooldown_s = ReadFiniteClamped(yaml_file,
            "logging.blackbox.cooldown_s", 60.0, 0.0, 3600.0);
        config.blackbox_memory_mb = std::clamp(
            yaml_file.Read<int>("logging.blackbox.max_memory_mb").value_or(64), 1, 512);
        config.queue_memory_mb = std::clamp(
            yaml_file.Read<int>("logging.queue_memory_mb").value_or(16), 1, 128);
        config.max_total_size_mb = std::clamp(
            yaml_file.Read<int>("logging.storage.max_total_mb").value_or(0), 0, 1024 * 1024);
        config.min_free_space_mb = std::clamp(
            yaml_file.Read<int>("logging.storage.min_free_mb").value_or(0), 0, 1024 * 1024);

        const std::string configured_directory =
            yaml_file.Read<std::string>("logging.directory").value_or("log/humanoid");
        config.directory = yaml_file.ToAbsPath(configured_directory);
        if (const char *override_directory = std::getenv("HUMANOID_LOG_DIRECTORY")) {
            config.directory = override_directory;
            if (config.directory.rfind("/dev/shm/", 0) == 0) {
                config.max_total_size_mb = 128;
                config.min_free_space_mb = 16;
                config.max_file_size_mb = 4;
                config.max_files = 2;
            }
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            config_ = config;
            component_ = Sanitize(component);
            yaml_path_ = yaml_path;
            stopped_ = false;
            artifacts_stopped_ = false;
            initialized_ = true;
            dropped_.store(0);
            high_rate_.store(false);
            queue_bytes_ = 0;
            last_trigger_ = capture_started_ = -1.0e9;
            last_maintenance_ = last_health_ = 0.0;
            seen_capture_.clear();
            snapshot_pending_.store(false);
            total_dropped_ = 0;
            io_errors_ = 0;
            formatter_drops.store(0);
        }

        if (!config.file_enabled && !config.console_enabled) return;
        try {
            if (!config.file_enabled) {
                worker_ = std::thread(&Logger::Worker, this);
                return;
            }
            const std::string robot = Sanitize(
                yaml_file.Read<std::string>("robot_base.name").value_or("robot"));
            robot_ = robot;
            boot_id_ = BootId();
            const std::string session_name = robot + "_" + component_ + "_" +
                Timestamp("%Y%m%d_%H%M%S", false) + "_" + std::to_string(getpid());
            const std::filesystem::path base_directory(config.directory);
            const std::filesystem::path session = base_directory / session_name;
            PrepareDirectory(base_directory);
            PrepareDirectory(session);
            {
                std::lock_guard<std::mutex> lock(mutex_);
                session_directory_ = session.string();
            }
            WriteMetadata(robot);
            storage_ = std::make_unique<LogStorage>(config.directory, session.string(), robot,
                config.max_total_size_mb, config.min_free_space_mb);
            worker_ = std::thread(&Logger::Worker, this);
            artifact_worker_ = std::thread(&Logger::ArtifactWorker, this);
            WriteArtifact("config", "runtime_config", yaml_path_);
            const auto artifacts = yaml_file.Read<std::vector<std::string>>(
                "logging.artifacts").value_or(std::vector<std::string>{});
            for (const auto &artifact : artifacts) {
                const auto artifact_path = yaml_file.ToAbsPath(artifact);
                WriteArtifact("configured",
                    std::filesystem::path(artifact_path).filename().string(), artifact_path);
            }
        } catch (const std::exception &error) {
            std::cerr << "[runtime_logger] failed to open log directory: "
                << error.what() << "\n";
            {
                std::lock_guard<std::mutex> lock(mutex_);
                config_.file_enabled = false;
                config_.telemetry_enabled = false;
            }
            Shutdown();
        }
    }

    void Shutdown() {
        std::thread artifact_worker;
        std::thread worker;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!initialized_) return;
            artifacts_stopped_ = true;
            artifact_condition_.notify_all();
            artifact_worker = std::move(artifact_worker_);
        }
        if (artifact_worker.joinable()) artifact_worker.join();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopped_ = true;
            condition_.notify_all();
            worker = std::move(worker_);
        }
        if (worker.joinable()) worker.join();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            files_.clear();
            queue_.clear();
            ring_.clear();
            csv_headers_.clear();
            summary_times_.clear();
            ring_bytes_ = 0;
            queue_bytes_ = 0;
            capture_.clear();
            storage_.reset();
            artifact_queue_.clear();
            initialized_ = false;
            session_directory_.clear();
        }
    }

    void WriteLog(Level level, const std::string &message, bool emit_console) {
        Config config;
        std::string component;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            config = config_;
            component = component_;
        }
        if (level < config.level) return;
        const std::string line = Timestamp("%Y-%m-%d %H:%M:%S", true) +
            " [" + component + "] [" + LevelName(level) + "] " + message;
        QueueItem item{"events", "", line, false};
        item.emit_console = emit_console && config.console_enabled;
        item.console_error = level >= Level::kWarning;
        item.trigger = level >= Level::kError;
        if (config.file_enabled || item.emit_console) Enqueue(std::move(item));
    }

    void WriteCsv(const std::string &stream, const std::string &header,
        const std::string &row) {
        const Config config = GetConfig();
        if (!config.telemetry_enabled) return;
        Enqueue({Sanitize(stream), header, row, true});
    }

    void Trigger(const std::string &reason) {
        QueueItem item{"events", "", "snapshot requested: " + reason.substr(0, 512), false};
        item.trigger = true;
        Enqueue(std::move(item));
    }
    bool HighRate() const { return high_rate_.load(std::memory_order_relaxed); }

    void WriteArtifact(const std::string &kind, const std::string &name,
        const std::string &path) {
        const Config config = GetConfig();
        if (!config.file_enabled) return;
        QueueItem item;
        item.kind = QueueItem::Kind::kArtifact;
        item.artifact_kind = kind;
        item.artifact_name = name;
        item.artifact_path = path;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_ || !config_.file_enabled || artifacts_stopped_) return;
        if (artifact_queue_.size() >= static_cast<size_t>(config_.queue_capacity)) {
            ++dropped_;
            return;
        }
        artifact_queue_.push_back(std::move(item));
        artifact_condition_.notify_one();
    }

    Config GetConfig() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return config_;
    }

    std::string SessionDirectory() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return session_directory_;
    }

private:
    Logger() = default;
    ~Logger() { Shutdown(); }

    void Enqueue(QueueItem item) {
        item.monotonic_s = MonotonicSeconds();
        // CSV callers are background formatters. Waiting for this short queue
        // mutex cannot stall a realtime producer and avoids splitting paired rows.
        std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
        if (item.csv) lock.lock();
        else lock.try_lock();
        if (!lock.owns_lock()) {
            if (!item.trigger || !PreserveCritical(item)) ++dropped_;
            return;
        }
        if (!initialized_ || stopped_) return;
        const size_t bytes = ItemBytes(item);
        if (queue_.size() >= static_cast<size_t>(config_.queue_capacity) ||
            queue_bytes_ + bytes > static_cast<size_t>(config_.queue_memory_mb) * 1024 * 1024) {
            if (!item.trigger || !PreserveCritical(item)) ++dropped_;
            return;
        }
        queue_bytes_ += bytes;
        queue_.push_back(std::move(item));
        condition_.notify_one();
    }

    struct CriticalSlot {
        std::atomic<int> state{0};  // 0 free, 1 producer owns, 2 ready, 3 consumer owns.
        std::array<char, 4096> line{};
        double monotonic_s = 0;
        bool emit_console = false, console_error = false;
    };

    bool PreserveCritical(const QueueItem &item) {
        for (auto &slot : critical_) {
            int expected = 0;
            if (!slot.state.compare_exchange_strong(expected, 1, std::memory_order_acquire)) continue;
            const size_t length = std::min(item.line.size(), slot.line.size() - 1);
            std::copy_n(item.line.data(), length, slot.line.data());
            slot.line[length] = '\0';
            slot.monotonic_s = item.monotonic_s;
            slot.emit_console = item.emit_console;
            slot.console_error = item.console_error;
            slot.state.store(2, std::memory_order_release);
            condition_.notify_one();
            return true;
        }
        snapshot_pending_.store(true);
        return false;
    }

    void DrainCritical() {
        for (auto &slot : critical_) {
            int expected = 2;
            if (!slot.state.compare_exchange_strong(expected, 3, std::memory_order_acquire)) continue;
            QueueItem item{"events", "", slot.line.data(), false};
            item.monotonic_s = slot.monotonic_s;
            item.emit_console = slot.emit_console;
            item.console_error = slot.console_error;
            item.trigger = true;
            slot.state.store(0, std::memory_order_release);
            try { WriteRecord(item); } catch (...) { ++dropped_; snapshot_pending_.store(true); }
        }
    }

    void Worker() {
        robot_base::ThreadLoop::Background("log_writer").Apply();
        (void)setpriority(PRIO_PROCESS, 0, 10);
#ifdef SYS_ioprio_set
        (void)syscall(SYS_ioprio_set, 1, 0, (3 << 13));  // Idle I/O class, this thread only.
#endif
        for (;;) {
            QueueItem item;
            bool have_item = false;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait_for(lock, std::chrono::milliseconds(100),
                    [this]() { return stopped_ || !queue_.empty(); });
                if (queue_.empty() && stopped_) break;
                if (!queue_.empty()) {
                    queue_bytes_ -= ItemBytes(queue_.front());
                    item = std::move(queue_.front());
                    queue_.pop_front();
                    have_item = true;
                }
            }
            try { Maintain(); } catch (...) { ++dropped_; }
            if (!have_item) continue;
            try { WriteRecord(item); } catch (...) { ++dropped_; }
        }
        try {
            DrainCritical();
            FinishCapture();
            WriteHealth();
            for (auto &[name, file] : files_) file.stream.flush();
        } catch (...) { /* Logging failures must never terminate the robot process. */ }
    }

    void ArtifactWorker() {
        robot_base::ThreadLoop::Background("log_artifacts").Apply();
        for (;;) {
            QueueItem item;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                artifact_condition_.wait(lock, [this]() {
                    return artifacts_stopped_ || !artifact_queue_.empty();
                });
                if (artifact_queue_.empty() && artifacts_stopped_) break;
                item = std::move(artifact_queue_.front());
                artifact_queue_.pop_front();
            }
            try { Enqueue(BuildArtifactRecord(item)); } catch (...) { ++dropped_; }
        }
    }

    void WriteRecord(const QueueItem &item) {
        if (item.trigger && config_.file_enabled && config_.blackbox_enabled) {
            try { RequestCapture(item.line); } catch (...) { ++dropped_; }
        }
        if (item.emit_console) {
            std::ostream &output = item.console_error ? std::cerr : std::cout;
            // Launchers also consume this stream when stdout is a regular file.
            // Flush on the background writer, never on the real-time caller.
            output << item.line << std::endl;
        }
        if (!config_.file_enabled) return;
        if (item.csv) {
            std::string reason;
            if (!ValidateCsvItem(item, &reason)) {
                WriteItem({"events", "",
                    Timestamp("%Y-%m-%d %H:%M:%S", true) +
                        " [" + component_ + "] [ERROR] rejected CSV stream " +
                        item.stream + ": " + reason,
                    false});
                return;
            }
        }
        if (item.csv && config_.blackbox_enabled && item.stream != "artifacts" &&
            item.stream.find("_inventory") == std::string::npos &&
            item.stream.find("_timing") == std::string::npos) {
            RecordTelemetry(item);
        } else {
            WriteItem(item);
        }
    }

    static size_t ItemBytes(const QueueItem &item) {
        return sizeof(item) + item.stream.capacity() + item.header.capacity() + item.line.capacity();
    }

    void RecordTelemetry(const QueueItem &item) {
        csv_headers_[item.stream] = item.header;
        QueueItem cached{item.stream, "", item.line, true};
        cached.monotonic_s = item.monotonic_s;
        ring_bytes_ += ItemBytes(cached);
        ring_.push_back(std::move(cached));
        while (!ring_.empty() && (ring_bytes_ >
                static_cast<size_t>(config_.blackbox_memory_mb) * 1024 * 1024 ||
                item.monotonic_s - ring_.front().monotonic_s > config_.pre_trigger_s)) {
            ring_bytes_ -= ItemBytes(ring_.front());
            ring_.pop_front();
        }
        if (!capture_.empty()) WriteCaptureItem(item);
        // Rows from the same sampled frame (e.g. several buses) stay together.
        const double sample = std::strtod(item.line.c_str(), nullptr);
        auto &last = summary_times_[item.stream];
        if (sample == last || sample < last || sample - last >= 1.0 / config_.continuous_rate_hz) {
            last = sample;
            WriteItem(item);
        }
    }

    void RequestCapture(const std::string &reason) {
        const double now = MonotonicSeconds();
        if (now - last_trigger_ < config_.trigger_cooldown_s) return;
        FileLock guard(std::filesystem::path(config_.directory) / ("." + robot_ + ".capture.lock"));
        if (!guard.Locked()) { snapshot_pending_.store(true); return; }
        last_trigger_ = now;
        const std::string id = std::to_string(static_cast<uint64_t>(now * 1e9)) +
            "_" + std::to_string(getpid());
        const auto request = std::filesystem::path(config_.directory) / ("." + robot_ + ".capture.json");
        // All processes reuse the first trigger in a burst, so their files share one incident ID.
        if (!std::filesystem::is_symlink(request) && std::filesystem::is_regular_file(request) &&
            std::filesystem::file_size(request) <= 4096) {
            try {
                const auto value = YAML::LoadFile(request.string());
                const double age = now - value["monotonic_s"].as<double>();
                if (value["boot_id"].as<std::string>() == boot_id_ && age >= 0 &&
                    age < std::max(config_.post_trigger_s, config_.trigger_cooldown_s)) {
                    StartCapture(value["id"].as<std::string>(), value["reason"].as<std::string>(),
                        std::max(0.0, config_.post_trigger_s - age));
                    return;
                }
            } catch (...) { /* Replace a malformed request. */ }
        }
        const auto temporary = request.string() + "." + std::to_string(getpid()) + ".tmp";
        std::ofstream output(temporary);
        output << "{\"id\":\"" << id << "\",\"boot_id\":\"" << boot_id_
            << "\",\"monotonic_s\":" << std::setprecision(17) << now
            << ",\"reason\":\"" << JsonEscape(reason.substr(0, 512)) << "\"}";
        output.close();
        if (output) std::filesystem::rename(temporary, request);
        StartCapture(id, reason, config_.post_trigger_s);
    }

    void StartCapture(const std::string &id, const std::string &reason, double post_s) {
        if (id == seen_capture_) return;
        seen_capture_ = id;
        const double now = MonotonicSeconds();
        if (!capture_.empty()) return;  // Coalesce a fault storm into the current incident.
        if (now - capture_started_ < config_.trigger_cooldown_s) return;
        if (storage_ && !storage_->Allow(2048, false)) return;
        while (!ring_.empty() && now - ring_.front().monotonic_s > config_.pre_trigger_s) {
            ring_bytes_ -= ItemBytes(ring_.front());
            ring_.pop_front();
        }
        capture_started_ = now;
        capture_write_failed_ = false;
        capture_deadline_ = now + std::clamp(post_s, 0.0, 120.0);
        capture_ = "incident_" + Sanitize(id);
        std::error_code error;
        std::filesystem::create_directory(std::filesystem::path(session_directory_) / capture_, error);
        if (error) { capture_.clear(); ++dropped_; return; }
        std::ofstream metadata(std::filesystem::path(session_directory_) / capture_ / "trigger.json");
        metadata << "{\"id\":\"" << JsonEscape(id) << "\",\"reason\":\""
            << JsonEscape(reason) << "\",\"trigger_monotonic_s\":" << std::setprecision(17)
            << now << ",\"buffer_records\":" << ring_.size()
            << ",\"buffer_bytes\":" << ring_bytes_
            << ",\"retained_seconds\":" << (ring_.empty() ? 0.0 : now - ring_.front().monotonic_s)
            << "}";
        metadata.close();
        if (!metadata) { capture_write_failed_ = true; ++io_errors_; }
        high_rate_.store(true);
        for (const auto &item : ring_) WriteCaptureItem(item);
        // Save the pre-trigger evidence now, not after the post-trigger window.
        // This runs only on log_writer; realtime producers never wait for I/O.
        SyncCapture();
    }

    void WriteCaptureItem(const QueueItem &item) {
        QueueItem captured = item;
        captured.header = csv_headers_[item.stream];
        captured.stream = capture_ + "/" + item.stream;
        WriteItem(captured);
    }

    static bool SyncPath(const std::filesystem::path &path, bool directory = false) {
        const int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW |
            (directory ? O_DIRECTORY : 0));
        if (fd < 0) return false;
        int result;
        do { result = fsync(fd); } while (result < 0 && errno == EINTR);
        const int closed = close(fd);
        return result == 0 && closed == 0;
    }

    bool SyncCapture() {
        last_capture_sync_ = MonotonicSeconds();
        bool ok = true;
        for (auto &[name, file] : files_) {
            if (name.rfind(capture_ + "/", 0) != 0) continue;
            file.stream.flush();
            if (!file.stream.good()) { capture_write_failed_ = true; ok = false; }
        }
        const auto directory = std::filesystem::path(session_directory_) / capture_;
        std::error_code error;
        for (std::filesystem::directory_iterator it(directory, error), end; !error && it != end; it.increment(error)) {
            if (it->is_regular_file(error) && !SyncPath(it->path())) ok = false;
        }
        if (error || !SyncPath(directory, true) ||
            !SyncPath(session_directory_, true) || !SyncPath(config_.directory, true)) ok = false;
        if (!ok) ++io_errors_;
        return ok;
    }

    void FinishCapture() {
        if (capture_.empty()) return;
        const bool synced = SyncCapture() && !capture_write_failed_;
        for (auto it = files_.begin(); it != files_.end();) {
            if (it->first.rfind(capture_ + "/", 0) == 0) it = files_.erase(it);
            else ++it;
        }
        if (synced) {
            const auto directory = std::filesystem::path(session_directory_) / capture_;
            const auto pending = directory / ".complete.pending";
            std::ofstream complete(pending);
            complete << "durable=1\nsuppressed_records=" << (storage_ ? storage_->Suppressed() : 0) << '\n';
            complete << "dropped_records=" << total_dropped_ + dropped_.load()
                << "\nformatter_drops=" << formatter_drops.load() << '\n';
            complete.close();
            std::error_code error;
            if (complete && SyncPath(pending)) {
                std::filesystem::rename(pending, directory / ".complete", error);
                if (error || !SyncPath(directory, true)) {
                    ++io_errors_;
                    std::filesystem::remove(directory / ".complete", error);
                }
            } else { ++io_errors_; }
        }
        capture_.clear();
        high_rate_.store(false);
    }

    void Maintain() {
        DrainCritical();
        const double now = MonotonicSeconds();
        if (!capture_.empty() && now >= capture_deadline_) FinishCapture();
        if (now - last_maintenance_ < 0.25) return;
        last_maintenance_ = now;
        if (!capture_.empty() && now - last_capture_sync_ >= 1.0) SyncCapture();
        if (storage_) storage_->Refresh();
        if (config_.file_enabled && now - last_health_ >= 5.0) {
            last_health_ = now;
            WriteHealth();
        }
        if (!config_.file_enabled || !config_.blackbox_enabled) return;
        if (snapshot_pending_.exchange(false)) RequestCapture("fault trigger preserved after queue overflow");
        const auto request = std::filesystem::path(config_.directory) / ("." + robot_ + ".capture.json");
        std::error_code error;
        if (std::filesystem::is_symlink(request, error) ||
            !std::filesystem::is_regular_file(request, error) ||
            std::filesystem::file_size(request, error) > 4096) return;
        try {
            const auto value = YAML::LoadFile(request.string());
            const double time = value["monotonic_s"].as<double>();
            if (value["boot_id"].as<std::string>() != boot_id_ || now - time > 2.0 || time > now) return;
            StartCapture(value["id"].as<std::string>(), value["reason"].as<std::string>(),
                value["post_s"].as<double>(config_.post_trigger_s));
        } catch (...) { /* An invalid/stale request cannot affect the runtime. */ }
    }

    void WriteHealth() {
        total_dropped_ += dropped_.exchange(0);
        if (!config_.file_enabled || (storage_ && !storage_->Allow(512, true))) return;
        std::ofstream health(std::filesystem::path(session_directory_) / "logger_health.json");
        health << "{\"monotonic_s\":" << std::setprecision(17) << MonotonicSeconds()
            << ",\"queue_drops\":" << total_dropped_
            << ",\"formatter_drops\":" << formatter_drops.load()
            << ",\"io_errors\":" << io_errors_
            << ",\"storage_suppressed\":" << (storage_ ? storage_->Suppressed() : 0)
            << ",\"ring_bytes\":" << ring_bytes_ << ",\"ring_records\":" << ring_.size() << "}";
    }

    bool ValidateCsvItem(const QueueItem &item, std::string *reason) const {
        const CsvShape header = AnalyzeCsv(item.header);
        if (!header.valid || header.records != 1) {
            *reason = "header is not one valid CSV record";
            return false;
        }
        const CsvShape rows = AnalyzeCsv(item.line);
        if (!rows.valid) {
            *reason = "row data is not valid CSV";
            return false;
        }
        if (rows.columns != header.columns) {
            *reason = "header has " + std::to_string(header.columns) +
                " columns but row has " + std::to_string(rows.columns);
            return false;
        }
        const auto existing = files_.find(item.stream);
        if (existing != files_.end() && existing->second.header != item.header) {
            *reason = "header changed after the stream was opened";
            return false;
        }
        return true;
    }

    QueueItem BuildArtifactRecord(const QueueItem &item) const {
        std::error_code error;
        const auto absolute_path = std::filesystem::absolute(item.artifact_path, error);
        const std::string resolved_path = error ? item.artifact_path
            : absolute_path.lexically_normal().string();
        error.clear();
        const bool exists = std::filesystem::is_regular_file(resolved_path, error);
        error.clear();
        const uintmax_t size = exists ? std::filesystem::file_size(resolved_path, error) : 0;
        const std::string digest = exists && !error ? Sha256File(resolved_path) : std::string{};
        std::ostringstream row;
        row << CsvEscape(item.artifact_kind) << "," << CsvEscape(item.artifact_name) << ","
            << CsvEscape(resolved_path) << "," << exists << "," << size << ","
            << digest;
        return {"artifacts", "kind,name,path,exists,size_bytes,sha256",
            row.str(), true};
    }

    void WriteItem(const QueueItem &item) {
        if (storage_ && !storage_->Allow(item.line.size() + item.header.size() + 2, !item.csv))
            return;
        FileState &file = OpenFile(item);
        if (!file.stream.is_open()) {
            if (!capture_.empty() && item.stream.rfind(capture_ + "/", 0) == 0)
                capture_write_failed_ = true;
            return;
        }
        const std::string line = item.line + "\n";
        const uint64_t limit =
            static_cast<uint64_t>(config_.max_file_size_mb) * 1024ULL * 1024ULL;
        if (file.bytes + line.size() > limit) {
            Rotate(item.stream, &file);
        }
        if (!file.stream.is_open()) {
            if (!capture_.empty() && item.stream.rfind(capture_ + "/", 0) == 0)
                capture_write_failed_ = true;
            return;
        }
        file.stream << line;
        file.bytes += line.size();

        const auto now = std::chrono::steady_clock::now();
        if (!item.csv || now - file.last_flush >= std::chrono::seconds(1)) {
            file.stream.flush();
            file.last_flush = now;
        }
        if (!file.stream.good()) {
            ++io_errors_;
            if (!capture_.empty() && item.stream.rfind(capture_ + "/", 0) == 0)
                capture_write_failed_ = true;
            file.stream.close();
            file.retry_after = now + std::chrono::seconds(1);
        }
    }

    FileState &OpenFile(const QueueItem &item) {
        auto [iterator, inserted] = files_.try_emplace(item.stream);
        auto &file = iterator->second;
        if (inserted) {
            file.header = item.header;
            file.csv = item.csv;
        }
        if (!file.stream.is_open() && std::chrono::steady_clock::now() >= file.retry_after)
            Open(item.stream, &file, file.opened_once);
        return file;
    }

    std::filesystem::path FilePath(
        const std::string &stream, bool csv, int suffix = 0) const {
        std::string filename = stream + (csv ? ".csv" : ".log");
        if (suffix > 0) filename += "." + std::to_string(suffix);
        return std::filesystem::path(session_directory_) / filename;
    }

    void Open(const std::string &stream, FileState *file, bool append = false) {
        const auto path = FilePath(stream, file->csv);
        file->stream.clear();
        file->stream.open(path, std::ios::out | (append ? std::ios::app : std::ios::trunc));
        file->bytes = 0;
        file->last_flush = std::chrono::steady_clock::now();
        file->retry_after = file->last_flush + std::chrono::seconds(1);
        if (!file->stream.is_open()) { ++io_errors_; return; }
        file->opened_once = true;
        std::error_code size_error;
        if (append) file->bytes = std::filesystem::file_size(path, size_error);
        if (size_error) file->bytes = 0;
        if (file->stream.is_open() && !PrepareLogFile(path)) {
            file->stream.close();
            std::error_code error;
            if (!append) std::filesystem::remove(path, error);
            ++io_errors_;
            return;
        }
        if (file->stream.is_open() && file->bytes == 0 && file->csv && !file->header.empty()) {
            file->stream << file->header << "\n";
            file->bytes = file->header.size() + 1;
        }
    }

    void Rotate(const std::string &stream, FileState *file) {
        file->stream.close();
        std::error_code error;
        const auto oldest = FilePath(stream, file->csv, config_.max_files - 1);
        if (config_.max_files > 1) std::filesystem::remove(oldest, error);
        for (int i = config_.max_files - 2; i >= 1; --i) {
            const auto from = FilePath(stream, file->csv, i);
            const auto to = FilePath(stream, file->csv, i + 1);
            error.clear();
            if (std::filesystem::exists(from, error))
                std::filesystem::rename(from, to, error);
        }
        if (config_.max_files > 1) {
            const auto current = FilePath(stream, file->csv);
            const auto first = FilePath(stream, file->csv, 1);
            error.clear();
            std::filesystem::rename(current, first, error);
        }
        Open(stream, file);
    }

    void WriteMetadata(const std::string &robot) {
        const std::filesystem::path path =
            std::filesystem::path(session_directory_) / "metadata.json";
        std::ofstream output(path);
        if (!output.is_open()) {
            throw std::runtime_error("failed to create log metadata file");
        }
        if (!PrepareLogFile(path)) {
            output.close();
            std::error_code error;
            std::filesystem::remove(path, error);
            throw std::runtime_error("failed to secure log metadata file");
        }
        const std::string config_sha256 = Sha256File(yaml_path_);
        const GitInfo config_git = GitInfoForPath(yaml_path_);
        output << "{\n"
            << "  \"robot\": \"" << JsonEscape(robot) << "\",\n"
            << "  \"component\": \"" << JsonEscape(component_) << "\",\n"
            << "  \"pid\": " << getpid() << ",\n"
            << "  \"boot_id\": \"" << boot_id_ << "\",\n"
            << "  \"started_monotonic_s\": " << std::setprecision(17)
            << MonotonicSeconds() << ",\n"
            << "  \"started_at\": \"" << Timestamp("%Y-%m-%dT%H:%M:%S", false)
            << "\",\n"
            << "  \"config_path\": \"" << JsonEscape(yaml_path_) << "\",\n"
            << "  \"config_hash_fnv1a64\": \"0x" << std::hex
            << HashFile(yaml_path_) << std::dec << "\",\n"
            << "  \"config_sha256\": \"" << config_sha256 << "\",\n"
            << "  \"config_repository_commit\": \""
            << JsonEscape(config_git.commit) << "\",\n"
            << "  \"config_repository_dirty\": "
            << (config_git.dirty ? "true" : "false") << ",\n"
            << "  \"common_build_commit\": \"" << HUMANOID_COMMON_GIT_COMMIT
            << "\",\n"
            << "  \"common_build_dirty\": "
            << (HUMANOID_COMMON_GIT_DIRTY ? "true" : "false") << "\n"
            << "}\n";
    }

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    std::condition_variable artifact_condition_;
    std::deque<QueueItem> queue_;
    std::deque<QueueItem> artifact_queue_;
    std::map<std::string, FileState> files_;
    std::thread worker_;
    std::thread artifact_worker_;
    std::atomic<uint64_t> dropped_{0};
    std::atomic<bool> high_rate_{false};
    std::atomic<bool> snapshot_pending_{false};
    std::array<CriticalSlot, 8> critical_;
    uint64_t total_dropped_ = 0;
    uint64_t io_errors_ = 0;
    double last_health_ = 0.0;
    size_t queue_bytes_ = 0, ring_bytes_ = 0;
    std::deque<QueueItem> ring_;
    std::map<std::string, std::string> csv_headers_;
    std::map<std::string, double> summary_times_;
    std::unique_ptr<LogStorage> storage_;
    std::string robot_, boot_id_, capture_, seen_capture_;
    double capture_deadline_ = 0.0, capture_started_ = -3600.0;
    double last_capture_sync_ = 0.0;
    bool capture_write_failed_ = false;
    double last_trigger_ = -3600.0, last_maintenance_ = 0.0;
    Config config_;
    std::string component_ = "runtime";
    std::string yaml_path_;
    std::string session_directory_;
    bool initialized_ = false;
    bool stopped_ = true;
    bool artifacts_stopped_ = true;
};

}  // namespace

Session::Session(const robot_base::YamlFile &yaml_file,
    const std::string &yaml_path, const std::string &component,
    bool console_allowed) {
    Logger::Instance().Initialize(yaml_file, yaml_path, component, console_allowed);
    active_ = true;
}

Session::~Session() {
    if (active_) Logger::Instance().Shutdown();
}

void Log(Level level, const std::string &message, bool emit_console) {
    Logger::Instance().WriteLog(level, message, emit_console);
}

void TriggerSnapshot(const std::string &reason) { Logger::Instance().Trigger(reason); }
bool HighRateRequested() { return Logger::Instance().HighRate(); }
void ReportTelemetryDrop() { formatter_drops.fetch_add(1, std::memory_order_relaxed); }

void RecordCsv(
    const std::string &stream, const std::string &header, const std::string &row) {
    Logger::Instance().WriteCsv(stream, header, row);
}

void RecordArtifact(
    const std::string &kind, const std::string &name, const std::string &path) {
    Logger::Instance().WriteArtifact(kind, name, path);
}

Config GetConfig() { return Logger::Instance().GetConfig(); }

std::string GetSessionDirectory() {
    return Logger::Instance().SessionDirectory();
}

}  // namespace runtime_logging
