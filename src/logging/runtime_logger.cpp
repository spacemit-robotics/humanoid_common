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
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
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
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

extern char **environ;

#ifndef HUMANOID_COMMON_GIT_COMMIT
#define HUMANOID_COMMON_GIT_COMMIT "unknown"
#endif

#ifndef HUMANOID_COMMON_GIT_DIRTY
#define HUMANOID_COMMON_GIT_DIRTY 0
#endif

namespace runtime_logging {
namespace {

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
};

struct FileState {
    std::ofstream stream;
    std::chrono::steady_clock::time_point last_flush;
    uint64_t bytes = 0;
    std::string header;
    bool csv = false;
};

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
        if (c == '\n') {
            output += "\\n";
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

        const std::string configured_directory =
            yaml_file.Read<std::string>("logging.directory").value_or("log/humanoid");
        config.directory = yaml_file.ToAbsPath(configured_directory);

        {
            std::lock_guard<std::mutex> lock(mutex_);
            config_ = config;
            component_ = Sanitize(component);
            yaml_path_ = yaml_path;
            stopped_ = false;
            artifacts_stopped_ = false;
            initialized_ = true;
            dropped_.store(0);
        }

        if (!config.file_enabled) return;
        try {
            const std::string robot = Sanitize(
                yaml_file.Read<std::string>("robot_base.name").value_or("robot"));
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
        if (emit_console && config.console_enabled) {
            std::ostream &output =
                level >= Level::kWarning ? std::cerr : std::cout;
            output << line << "\n";
        }
        if (config.file_enabled) Enqueue({"events", "", line, false});
    }

    void WriteCsv(const std::string &stream, const std::string &header,
        const std::string &row) {
        const Config config = GetConfig();
        if (!config.telemetry_enabled) return;
        Enqueue({Sanitize(stream), header, row, true});
    }

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
        std::lock_guard<std::mutex> lock(mutex_);
        if (!initialized_ || !config_.file_enabled || stopped_) return;
        if (queue_.size() >= static_cast<size_t>(config_.queue_capacity)) {
            ++dropped_;
            return;
        }
        queue_.push_back(std::move(item));
        condition_.notify_one();
    }

    void Worker() {
        for (;;) {
            QueueItem item;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this]() { return stopped_ || !queue_.empty(); });
                if (queue_.empty() && stopped_) break;
                item = std::move(queue_.front());
                queue_.pop_front();
            }
            const uint64_t dropped = dropped_.exchange(0);
            if (dropped > 0) {
                WriteItem({"events", "",
                    Timestamp("%Y-%m-%d %H:%M:%S", true) +
                        " [" + component_ + "] [WARN] dropped " +
                        std::to_string(dropped) + " log records",
                    false});
            }
            WriteRecord(item);
        }
        const uint64_t dropped = dropped_.exchange(0);
        if (dropped > 0) {
            WriteItem({"events", "",
                Timestamp("%Y-%m-%d %H:%M:%S", true) +
                    " [" + component_ + "] [WARN] dropped " +
                    std::to_string(dropped) + " log records",
                false});
        }
        for (auto &[name, file] : files_) file.stream.flush();
    }

    void ArtifactWorker() {
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
            Enqueue(BuildArtifactRecord(item));
        }
    }

    void WriteRecord(const QueueItem &item) {
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
        WriteItem(item);
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
        FileState &file = OpenFile(item);
        if (!file.stream.is_open()) return;
        const std::string line = item.line + "\n";
        const uint64_t limit =
            static_cast<uint64_t>(config_.max_file_size_mb) * 1024ULL * 1024ULL;
        if (file.bytes + line.size() > limit) {
            Rotate(item.stream, &file);
        }
        file.stream << line;
        file.bytes += line.size();

        const auto now = std::chrono::steady_clock::now();
        if (!item.csv || now - file.last_flush >= std::chrono::seconds(1)) {
            file.stream.flush();
            file.last_flush = now;
        }
    }

    FileState &OpenFile(const QueueItem &item) {
        auto [iterator, inserted] = files_.try_emplace(item.stream);
        auto &file = iterator->second;
        if (inserted) {
            file.header = item.header;
            file.csv = item.csv;
            Open(item.stream, &file);
        }
        return file;
    }

    std::filesystem::path FilePath(
        const std::string &stream, bool csv, int suffix = 0) const {
        std::string filename = stream + (csv ? ".csv" : ".log");
        if (suffix > 0) filename += "." + std::to_string(suffix);
        return std::filesystem::path(session_directory_) / filename;
    }

    void Open(const std::string &stream, FileState *file) {
        const auto path = FilePath(stream, file->csv);
        file->stream.open(path, std::ios::out | std::ios::trunc);
        file->bytes = 0;
        file->last_flush = std::chrono::steady_clock::now();
        if (file->stream.is_open() && !PrepareLogFile(path)) {
            file->stream.close();
            std::error_code error;
            std::filesystem::remove(path, error);
            return;
        }
        if (file->stream.is_open() && file->csv && !file->header.empty()) {
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
