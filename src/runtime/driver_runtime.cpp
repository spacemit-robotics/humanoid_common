/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file driver_runtime.cpp
 * @brief Transport-facing runtime for selectable robot driver backends
 */

#include <csignal>
#include <exception>
#include <iostream>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "driver_backend.h"
#include "robot_base.h"
#include "runtime_logger.h"
#include "transport_executor.h"

namespace {

volatile std::sig_atomic_t g_running = 1;

void OnSignal(int) { g_running = 0; }

void PrintUsage(const char *program, std::ostream &output) {
    output << "Usage: " << program << " <config.yaml>\n";
    output << "Options:\n";
    output << "  <config.yaml>  Robot configuration file\n";
    output << "  -h, --help     Show this help\n";
}

}  // namespace

int main(int argc, char *argv[]) {
    if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        PrintUsage(argv[0], argc < 2 ? std::cerr : std::cout);
        return argc < 2 ? 1 : 0;
    }

    const std::string yaml_path = argv[1];
    std::unique_ptr<runtime_logging::Session> logging_session;
    try {
        const auto yaml_file = robot_base::YamlFile::Load(yaml_path);
        logging_session = std::make_unique<runtime_logging::Session>(
            yaml_file, yaml_path, "driver");
        robot_base::ThreadLoop::FromYaml(yaml_file, "driver_main").Apply();

        const auto backend_kind = driver_runtime::ParseBackendKind(yaml_file);
        if (!driver_runtime::BackendIsCompiled(backend_kind)) {
            throw std::runtime_error("selected driver backend is unavailable in this build");
        }
        auto backend = driver_runtime::CreateBackend(backend_kind, yaml_path);

        auto transport = transport::Create(yaml_path);
        if (!transport->Init(yaml_path, transport::Role::DRIVER))
            throw std::runtime_error("failed to initialize driver transport");

        std::signal(SIGINT, OnSignal);
        std::signal(SIGTERM, OnSignal);
        runtime_logging::Log(runtime_logging::Level::kInfo, "driver runtime started");

        const auto exchange =
            [&](const robot_base::RobotData &state) -> std::optional<robot_base::ControlCmd> {
            transport->SendState(state);
            robot_base::ControlCmd latest;
            robot_base::ControlCmd received;
            bool has_command = false;
            while (transport->RecvControl(received)) {
                latest = std::move(received);
                has_command = true;
            }
            if (!has_command) return std::nullopt;
            return latest;
        };

        const int result = backend->Run(exchange, []() { return g_running != 0; });
        runtime_logging::Log(runtime_logging::Level::kInfo, "driver runtime stopped");
        return result;
    } catch (const std::exception &error) {
        if (logging_session) {
            runtime_logging::Log(runtime_logging::Level::kError, error.what());
        } else {
            std::cerr << "[driver_runtime] " << error.what() << "\n";
        }
        return 1;
    }
}
