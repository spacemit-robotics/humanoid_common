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
    double telemetry_rate_hz = 20.0;
    double driver_monitor_rate_hz = 2.0;
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
void RecordCsv(
    const std::string &stream, const std::string &header, const std::string &row);
Config GetConfig();
std::string GetSessionDirectory();

}  // namespace runtime_logging

#endif  // RUNTIME_LOGGER_H
