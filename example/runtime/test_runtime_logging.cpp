/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_runtime_logging.cpp
 * @brief Validate bounded recording and retention without robot hardware.
 */
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <yaml-cpp/yaml.h>

#include "log_storage.h"
#include "runtime_logger.h"
#include "snapshot_worker.h"

namespace {
namespace fs = std::filesystem;
void Require(bool value, const std::string &message) {
    if (!value) throw std::runtime_error(message);
}
size_t Lines(const fs::path &path) {
    std::ifstream input(path);
    std::string line;
    size_t count = 0;
    while (std::getline(input, line)) ++count;
    return count;
}

void RecordTest(const fs::path &root) {
    const auto config = root / "logging.yaml";
    {
        std::ofstream output(config);
        output << "robot_base: {name: logtest}\nlogging:\n"
            "  directory: logs\n  level: info\n  console: {enabled: false}\n"
            "  file: {enabled: true, max_size_mb: 1, max_files: 2}\n"
            "  queue_memory_mb: 1\n"
            "  telemetry: {enabled: true, policy_enabled: true, continuous_rate_hz: 2}\n"
            "  blackbox: {enabled: true, pre_trigger_s: 1, post_trigger_s: 0.2, "
            "cooldown_s: 0.5, max_memory_mb: 1}\n";
    }
    fs::path session;
    {
        const auto yaml = robot_base::YamlFile::Load(config.string());
        runtime_logging::Session logging(yaml, config.string(), "control", false);
        session = runtime_logging::GetSessionDirectory();
        Require(runtime_logging::GetConfig().policy_trace_enabled, "info must keep explicit policy tracing");
        for (int i = 0; i < 160; ++i) {
            runtime_logging::RecordCsv("control_trace", "wall_time_s,value", std::to_string(i * 0.01) + ",1");
            if (i == 125) {
                runtime_logging::Log(runtime_logging::Level::kError, "injected test fault", false);
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
                while (!runtime_logging::HighRateRequested() && std::chrono::steady_clock::now() < deadline)
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                Require(runtime_logging::HighRateRequested(), "post-trigger recording not active");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        Require(!runtime_logging::HighRateRequested(), "recording failed to expire");
        runtime_logging::RecordCsv("oversize", "value", std::string(2 * 1024 * 1024, '1'));
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    Require(Lines(session / "control_trace.csv") <= 6, "continuous log was not downsampled");
    int captures = 0;
    for (const auto &entry : fs::directory_iterator(session)) {
        if (!entry.is_directory() || entry.path().filename().string().rfind("incident_", 0) != 0) continue;
        ++captures;
        const size_t rows = Lines(entry.path() / "control_trace.csv");
        Require(rows > 60 && rows < 130, "pre/post full-rate samples were not retained");
        Require(fs::exists(entry.path() / ".complete"), "capture not finalized");
        std::ifstream marker(entry.path() / ".complete");
        std::string durable;
        std::getline(marker, durable);
        Require(durable == "durable=1", "capture completion has no durable-write confirmation");
    }
    Require(captures == 1, "fault storm was not coalesced");
    const auto health = YAML::LoadFile((session / "logger_health.json").string());
    Require(health["queue_drops"].as<int>() > 0, "oversized queue item was not accounted for");
    Require(health["ring_bytes"].as<int>() <= 1024 * 1024, "ring byte limit exceeded");
    // Invalid writable location must disable logging, not throw into robot startup.
    std::ofstream broken(root / "blocked.yaml");
    broken << "logging:\n  file: {enabled: true}\n  directory: logging.yaml/invalid\n";
    broken.close();
    const auto bad = robot_base::YamlFile::Load((root / "blocked.yaml").string());
    runtime_logging::Session logging(bad, (root / "blocked.yaml").string(), "driver", false);
    Require(!runtime_logging::GetConfig().file_enabled, "invalid path did not degrade logging");
}

void StorageTest(const fs::path &root) {
    const auto logs = root / "retention";
    const auto old = logs / "robot_old";
    const auto live = logs / "robot_live";
    const auto current = logs / "robot_current";
    for (const auto &path : {old, live, current}) {
        fs::create_directories(path);
        std::ofstream(path / "metadata.json") << "{\"robot\":\"robot\",\"pid\":0}";
    }
    std::ofstream(old / "data.csv") << std::string(2 * 1024 * 1024, 'x');
    std::ofstream(live / "data.csv") << std::string(2 * 1024 * 1024, 'x');
    const int lease = open((live / ".active.lock").c_str(), O_CREAT | O_RDWR, 0600);
    Require(lease >= 0 && flock(lease, LOCK_EX | LOCK_NB) == 0, "cannot lock test session");
    fs::create_directory_symlink(live, logs / "robot_link");
    {
        runtime_logging::LogStorage storage(logs.string(), current.string(), "robot", 1, 0);
        Require(!storage.Allow(100, false), "quota overflow should suppress bulk logging");
        Require(!fs::exists(old), "inactive session was not retired");
        Require(fs::exists(live / "data.csv"), "active session was removed");
        Require(fs::is_symlink(logs / "robot_link"), "symlink was traversed");
    }
    close(lease);
    runtime_logging::LogStorage space(logs.string(), current.string(), "robot", 0, 1024 * 1024);
    Require(!space.Allow(100, false), "free-space reserve not applied");
}

void EarlyCaptureTest(const fs::path &root, bool fail_sync) {
    const auto config = root / (fail_sync ? "sync-fail.yaml" : "early.yaml");
    std::ofstream(config) << "robot_base: {name: " << (fail_sync ? "early_fail" : "early_ok")
        << "}\nlogging:\n  directory: logs\n"
        "  console: {enabled: false}\n  file: {enabled: true}\n"
        "  telemetry: {enabled: true}\n"
        "  blackbox: {enabled: true, pre_trigger_s: 10, post_trigger_s: 30, cooldown_s: 1}\n";
    fs::path incident, session;
    {
        const auto yaml = robot_base::YamlFile::Load(config.string());
        runtime_logging::Session logging(yaml, config.string(), "driver", false);
        session = runtime_logging::GetSessionDirectory();
        for (int i = 0; i < 20; ++i)
            runtime_logging::RecordCsv("probe", "time,value", std::to_string(i) + ",7");
        runtime_logging::TriggerSnapshot("early persistence test");
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (std::chrono::steady_clock::now() < deadline) {
            for (const auto &entry : fs::directory_iterator(session)) {
                if (entry.is_directory() && entry.path().filename().string().rfind("incident_", 0) == 0)
                    incident = entry.path();
            }
            if (!incident.empty() && Lines(incident / "probe.csv") == 21) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        Require(!incident.empty() && Lines(incident / "probe.csv") == 21,
            "pre-trigger data was not saved before the 30-second recording window ended");
        Require(!fs::exists(incident / ".complete"), "active recording marked complete prematurely");
        if (fail_sync) {
            // Only damage this temporary test incident: O_NOFOLLOW must reject it.
            fs::remove(incident / "trigger.json");
            fs::create_symlink(config, incident / "trigger.json");
        }
    }
    Require(fs::exists(incident / ".complete") == !fail_sync,
        "completion marker did not reflect durable-write success/failure");
    if (fail_sync) {
        const auto health = YAML::LoadFile((session / "logger_health.json").string());
        Require(health["io_errors"].as<int>() > 0, "failed fsync was not reported");
    }
}

void ProducerTest() {
    std::atomic<bool> release{false};
    std::atomic<bool> entered{false};
    runtime_logging::SnapshotWorker<int, 2> worker([&](int) {
        entered.store(true);
        while (!release.load()) std::this_thread::sleep_for(std::chrono::milliseconds(1));
    });
    worker.Publish(1);
    while (!entered.load()) std::this_thread::yield();
    const auto started = std::chrono::steady_clock::now();
    for (int i = 0; i < 10000; ++i) worker.Publish(i);
    const auto elapsed = std::chrono::steady_clock::now() - started;
    release.store(true);
    worker.Stop();
    Require(elapsed < std::chrono::seconds(1) && worker.Dropped() > 0,
        "stalled consumer blocked producer");
    std::cout << "snapshot publish 10000 calls: "
        << std::chrono::duration<double, std::milli>(elapsed).count() << " ms\n";
}

void WriteRecoveryTest(const fs::path &root) {
    const auto config = root / "recovery.yaml";
    std::ofstream(config) << "robot_base: {name: recovery}\nlogging:\n  directory: logs\n"
        "  file: {enabled: true}\n  console: {enabled: false}\n  telemetry: {enabled: true}\n";
    fs::path session;
    {
        const auto yaml = robot_base::YamlFile::Load(config.string());
        runtime_logging::Session logging(yaml, config.string(), "driver", false);
        session = runtime_logging::GetSessionDirectory();
        // Temporarily replace only this test session path with a regular file.
        const auto saved = root / "saved-session";
        fs::rename(session, saved);
        std::ofstream(session) << "temporarily unavailable";
        runtime_logging::RecordCsv("recovered", "time,value", "1,1");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        fs::remove(session);
        fs::rename(saved, session);
        std::this_thread::sleep_for(std::chrono::milliseconds(1100));
        runtime_logging::RecordCsv("recovered", "time,value", "2,2");
    }
    Require(Lines(session / "recovered.csv") == 2, "logging did not resume after storage recovered");
    const auto health = YAML::LoadFile((session / "logger_health.json").string());
    Require(health["io_errors"].as<int>() > 0, "failed open was not accounted for");
}
}  // namespace

int main() {
    char directory[] = "/tmp/humanoid_logging_XXXXXX";
    const auto *created = mkdtemp(directory);
    if (!created) return 1;
    const fs::path root(created);
    try {
        ProducerTest();
        RecordTest(root);
        EarlyCaptureTest(root, false);
        EarlyCaptureTest(root, true);
        StorageTest(root);
        WriteRecoveryTest(root);
        fs::remove_all(root);
        std::cout << "PASS bounded logger, incident capture, inactive retention and I/O degradation\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << " (evidence: " << root << ")\n";
        return 1;
    }
}
