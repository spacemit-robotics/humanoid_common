/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_log_retention.cpp
 * @brief Verify logging.file.retention_days prunes only stale session directories
 */

#include <chrono>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>

#include "robot_base.h"
#include "runtime_logger.h"

namespace {

namespace fs = std::filesystem;

void Require(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

void Touch(const fs::path &path, fs::file_time_type time) {
    std::ofstream(path) << "x\n";
    fs::last_write_time(path, time);
}

fs::path MakeSession(const fs::path &root, const std::string &name,
    fs::file_time_type time) {
    const fs::path path = root / name;
    fs::create_directories(path);
    Touch(path / "events.log", time);
    fs::last_write_time(path, time);
    return path;
}

void WriteYaml(const fs::path &path, int retention_days) {
    std::ofstream yaml(path);
    yaml << "robot_base:\n"
            "  name: retention_test\n"
            "logging:\n"
            "  directory: logs\n"
            "  console: {enabled: false}\n"
            "  file: {enabled: true, retention_days: "
         << retention_days << "}\n";
}

bool RunSession(const fs::path &yaml_path) {
    const auto yaml = robot_base::YamlFile::Load(yaml_path.string());
    runtime_logging::Session logging(yaml, yaml_path.string(), "driver", false);
    // Retention runs asynchronously; give it time before shutdown joins it.
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    const auto session = runtime_logging::GetSessionDirectory();
    return !session.empty() && fs::exists(session);
}

}  // namespace

int main() {
    const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
    const fs::path root = fs::temp_directory_path() /
        ("log_retention_" + std::to_string(nonce));
    fs::create_directories(root / "logs");
    int status = 0;
    try {
        const auto now = fs::file_time_type::clock::now();
        const auto old_time = now - std::chrono::hours(24 * 5);
        const fs::path logs = root / "logs";

        const auto stale = MakeSession(logs, "retention_test_driver_old_1", old_time);
        const auto stale_but_active = MakeSession(
            logs, "retention_test_control_old_2", old_time);
        Touch(stale_but_active / "control_trace.csv", now);
        fs::last_write_time(stale_but_active, old_time);
        const auto fresh = MakeSession(logs, "retention_test_hmi_new", now);
        const auto foreign = MakeSession(logs, "other_robot_driver_old", old_time);
        Touch(logs / "retention_test_stray_file", old_time);

        const fs::path yaml_path = root / "robot.yaml";

        WriteYaml(yaml_path, 0);
        Require(RunSession(yaml_path), "session directory missing");
        Require(fs::exists(stale), "retention_days=0 must keep all sessions");

        WriteYaml(yaml_path, 3);
        Require(RunSession(yaml_path), "current session must never be removed");
        Require(!fs::exists(stale), "stale session was not removed");
        Require(fs::exists(stale_but_active),
            "session with recent file writes must be kept");
        Require(fs::exists(fresh), "fresh session must be kept");
        Require(fs::exists(foreign), "other robot's sessions must be kept");
        Require(fs::exists(logs / "retention_test_stray_file"),
            "plain files must not be removed");
        std::cout << "[PASS] log retention\n";
    } catch (const std::exception &error) {
        std::cerr << "[FAIL] " << error.what() << "\n";
        status = 1;
    }
    std::error_code ignored;
    fs::remove_all(root, ignored);
    return status;
}
