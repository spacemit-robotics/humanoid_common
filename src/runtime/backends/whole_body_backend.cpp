/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_backend.cpp
 * @brief Hardware whole-body adapter for the generic driver runtime
 */

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include "backends/whole_body_adapter.h"
#include "backends/whole_body_diagnostics.h"
#include "driver_backend.h"
#include "runtime_logger.h"
#include "whole_body.h"

namespace driver_runtime {
namespace {

double MonotonicTime() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

class WholeBodyBackend final : public DriverBackend {
public:
    explicit WholeBodyBackend(const std::string &yaml_path) {
        if (whole_body_create(yaml_path.c_str(), &device_) != WHOLE_BODY_OK)
            throw std::runtime_error("failed to create whole_body from YAML");
        if (whole_body_get_cycle_s(device_, &cycle_s_) != WHOLE_BODY_OK ||
            !std::isfinite(cycle_s_) || cycle_s_ <= 0.0) {
            whole_body_destroy(device_);
            device_ = nullptr;
            throw std::runtime_error("whole_body returned an invalid cycle period");
        }
    }

    ~WholeBodyBackend() override { whole_body_destroy(device_); }

    int Run(const ExchangeCallback &exchange, const ContinueCallback &should_continue) override {
        if (whole_body_init(device_) != WHOLE_BODY_OK) {
            runtime_logging::Log(runtime_logging::Level::kError,
                std::string("whole_body init failed: ") + whole_body_last_error(device_));
            return 1;
        }

        const auto logging_config = runtime_logging::GetConfig();
        const bool monitor_enabled =
            logging_config.driver_monitor_enabled && isatty(STDOUT_FILENO);
        const auto monitor_period =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(
                    1.0 / logging_config.driver_monitor_rate_hz));
        const auto telemetry_period =
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(1.0 / logging_config.telemetry_rate_hz));
        auto last_monitor = std::chrono::steady_clock::now() - monitor_period;
        auto last_telemetry = std::chrono::steady_clock::now() - telemetry_period;
        if (logging_config.driver_monitor_enabled && !monitor_enabled) {
            runtime_logging::Log(runtime_logging::Level::kInfo,
                "driver monitor disabled because stdout is not a terminal", false);
        }

        auto next_cycle = std::chrono::steady_clock::now();
        const auto cycle = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(cycle_s_));
        int previous_read_result = WHOLE_BODY_OK;
        int previous_command_result = WHOLE_BODY_OK;
        int previous_tick_result = WHOLE_BODY_OK;
        while (should_continue()) {
            whole_body_state hardware_state{};
            const int read_result = whole_body_read(device_, &hardware_state);
            if (read_result == WHOLE_BODY_OK) {
                if (previous_read_result != WHOLE_BODY_OK) {
                    runtime_logging::Log(
                        runtime_logging::Level::kInfo, "whole_body feedback recovered");
                }
                robot_base::RobotData state;
                if (!ConvertWholeBodyState(hardware_state, &state)) {
                    runtime_logging::Log(runtime_logging::Level::kError,
                        "rejected whole_body state dimensions");
                    (void)whole_body_set_mode(device_, WHOLE_BODY_MODE_SAFETY);
                } else {
                    const auto command = exchange(state);
                    if (command) {
                        whole_body_joint_command hardware_command{};
                        if (!ConvertControlCommand(
                                *command, hardware_state.num_dof, &hardware_command)) {
                            runtime_logging::Log(runtime_logging::Level::kError,
                                "rejected command dimensions");
                            (void)whole_body_set_mode(device_, WHOLE_BODY_MODE_SAFETY);
                        } else {
                            const int result = whole_body_write(device_, &hardware_command);
                            if (result != WHOLE_BODY_OK &&
                                result != WHOLE_BODY_ERR_READ_ONLY &&
                                result != previous_command_result) {
                                runtime_logging::Log(runtime_logging::Level::kError,
                                    std::string("whole_body command failed: ") +
                                        whole_body_last_error(device_));
                            } else if (result == WHOLE_BODY_OK &&
                                previous_command_result != WHOLE_BODY_OK) {
                                runtime_logging::Log(runtime_logging::Level::kInfo,
                                    "whole_body command path recovered");
                            }
                            previous_command_result = result;
                        }
                    }
                }
            } else if (read_result != previous_read_result) {
                runtime_logging::Log(runtime_logging::Level::kWarning,
                    std::string("whole_body feedback unavailable: ") +
                        whole_body_last_error(device_));
            }
            previous_read_result = read_result;

            const auto sample_time = std::chrono::steady_clock::now();
            const bool render_due =
                monitor_enabled && sample_time - last_monitor >= monitor_period;
            const bool telemetry_due = logging_config.telemetry_enabled &&
                sample_time - last_telemetry >= telemetry_period;
            if (render_due || telemetry_due) {
                whole_body_diagnostics diagnostics{};
                if (whole_body_get_diagnostics(device_, &diagnostics) == WHOLE_BODY_OK) {
                    if (render_due) {
                        RenderWholeBodyDiagnostics(diagnostics, cycle_s_);
                        last_monitor = sample_time;
                    }
                    if (telemetry_due) {
                        RecordWholeBodyDiagnostics(diagnostics);
                        last_telemetry = sample_time;
                    }
                }
            }

            const int tick_result = whole_body_tick(device_, MonotonicTime());
            if (tick_result != WHOLE_BODY_OK &&
                tick_result != previous_tick_result) {
                runtime_logging::Log(
                    tick_result == WHOLE_BODY_ERR_TIMEOUT
                        ? runtime_logging::Level::kWarning
                        : runtime_logging::Level::kError,
                    std::string("whole_body tick failed: ") +
                        whole_body_last_error(device_));
            } else if (tick_result == WHOLE_BODY_OK &&
                previous_tick_result != WHOLE_BODY_OK) {
                runtime_logging::Log(
                    runtime_logging::Level::kInfo, "whole_body tick recovered");
            }
            previous_tick_result = tick_result;
            if (tick_result != WHOLE_BODY_OK && tick_result != WHOLE_BODY_ERR_TIMEOUT)
                return 1;
            next_cycle += cycle;
            const auto now = std::chrono::steady_clock::now();
            if (next_cycle < now - cycle) next_cycle = now;
            std::this_thread::sleep_until(next_cycle);
        }
        return 0;
    }

private:
    whole_body_dev *device_ = nullptr;
    double cycle_s_ = 0.0;
};

}  // namespace

std::unique_ptr<DriverBackend> CreateWholeBodyBackend(const std::string &yaml_path) {
    return std::make_unique<WholeBodyBackend>(yaml_path);
}

}  // namespace driver_runtime
