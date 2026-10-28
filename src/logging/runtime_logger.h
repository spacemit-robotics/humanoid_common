/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file runtime_logger.h
 * @brief Internal runtime event and telemetry logging
 */

#ifndef RUNTIME_LOGGER_H
#define RUNTIME_LOGGER_H

#include <string>

#include "robot_base.h"

namespace runtime_logging {

enum class Level {
    kDebug = 0,
    kInfo = 1,
    kWarning = 2,
    kError = 3,
};

struct Config {
    Level level = Level::kInfo;
    bool console_enabled = true;
    bool file_enabled = false;
    bool telemetry_enabled = false;
    bool driver_monitor_enabled = false;
    bool blackbox_enabled = false;
    bool policy_trace_enabled = false;
    double idle_rate_hz = 0.0;
    double continuous_rate_hz = 2.0;
    double pre_trigger_s = 30.0;
    double post_trigger_s = 10.0;
    double trigger_cooldown_s = 60.0;
    int blackbox_memory_mb = 64;
    int queue_memory_mb = 16;
    int max_total_size_mb = 0;
    int min_free_space_mb = 0;
    double telemetry_rate_hz = 20.0;
    double control_telemetry_rate_hz = 20.0;
    double hardware_telemetry_rate_hz = 20.0;
    double driver_monitor_rate_hz = 2.0;
    double timing_window_s = 5.0;
    int queue_capacity = 4096;
    int max_file_size_mb = 64;
    int max_files = 4;
    std::string directory;
};

class Session {
public:
    Session(const robot_base::YamlFile &yaml_file, const std::string &yaml_path,
        const std::string &component, bool console_allowed = true);
    ~Session();

    Session(const Session &) = delete;
    Session &operator=(const Session &) = delete;

private:
    bool active_ = false;
};

void Log(Level level, const std::string &message, bool emit_console = true);
// Background-only formatting path; realtime callers must use SnapshotWorker.
void RecordCsv(
    const std::string &stream, const std::string &header, const std::string &row);
void RecordArtifact(
    const std::string &kind, const std::string &name, const std::string &path);
Config GetConfig();
std::string GetSessionDirectory();
void TriggerSnapshot(const std::string &reason);
bool HighRateRequested();
// Nonblocking counter used by fixed-size telemetry producer queues.
void ReportTelemetryDrop();

}  // namespace runtime_logging

#endif  // RUNTIME_LOGGER_H
