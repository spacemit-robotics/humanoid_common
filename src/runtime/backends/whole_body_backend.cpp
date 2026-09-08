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
#include <cstdint>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "backends/whole_body_adapter.h"
#include "backends/whole_body_diagnostics.h"
#include "driver_backend.h"
#include "runtime_logger.h"
#include "runtime_timing.h"
#include "whole_body.h"

namespace driver_runtime {
namespace {

double MonotonicTime() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

double WallTime() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

enum class FaultContext {
    kInitialization,
    kRead,
    kCommand,
    kTick,
    kStateAdapter,
    kCommandAdapter,
};

void RecordDriverTiming(const runtime_timing::Summary &timing) {
    std::ostringstream row;
    row << std::fixed << std::setprecision(6) << WallTime() << ","
        << timing.samples << "," << timing.period_p95_ms << ","
        << timing.period_p99_ms << "," << timing.period_max_ms << ","
        << timing.lateness_p95_ms << "," << timing.lateness_p99_ms << ","
        << timing.lateness_max_ms << "," << timing.skipped_cycles;
    runtime_logging::RecordCsv("driver_timing",
        "wall_time_s,samples,period_p95_ms,period_p99_ms,period_max_ms,"
        "lateness_p95_ms,lateness_p99_ms,lateness_max_ms,skipped_cycles",
        row.str());
}

class WholeBodyBackend final : public DriverBackend {
public:
    explicit WholeBodyBackend(const std::string &yaml_path) {
        const auto yaml_file = robot_base::YamlFile::Load(yaml_path);
        state_template_ = robot_base::RobotData::FromYaml(yaml_path);
        const auto hardware_config = yaml_file.Read<std::string>("whole_body.config_file");
        if (hardware_config) {
            runtime_logging::RecordArtifact("hardware_config", "whole_body",
                yaml_file.ToAbsPath(*hardware_config));
        }
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

    int Run(const PublishStateCallback &publish_state,
            const ReceiveCommandCallback &receive_command,
            const ContinueCallback &should_continue) override {
        const int init_result = whole_body_init(device_);
        if (init_result != WHOLE_BODY_OK) {
            runtime_logging::Log(runtime_logging::Level::kError,
                std::string("whole_body init failed: ") + whole_body_last_error(device_));
            if (!PublishFault(publish_state,
                    UpdateFault(init_result, FaultContext::kInitialization, nullptr, true))) {
                return 1;
            }
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
                std::chrono::duration<double>(
                    1.0 / logging_config.hardware_telemetry_rate_hz));
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
        runtime_timing::Window timing_window(next_cycle);
        while (should_continue()) {
            const auto cycle_started = std::chrono::steady_clock::now();
            timing_window.Observe(cycle_started, next_cycle);
            whole_body_state hardware_state{};
            const int read_result = whole_body_read(device_, &hardware_state);
            bool feedback_fault_active = false;
            if (read_result == WHOLE_BODY_OK) {
                RecoverFault(FaultContext::kRead);
                if (previous_read_result != WHOLE_BODY_OK) {
                    runtime_logging::Log(
                        runtime_logging::Level::kInfo, "whole_body feedback recovered");
                }
                robot_base::RobotData state;
                if (!ConvertWholeBodyState(hardware_state, &state)) {
                    runtime_logging::Log(runtime_logging::Level::kError,
                        "rejected whole_body state dimensions");
                    (void)whole_body_set_mode(device_, WHOLE_BODY_MODE_SAFETY);
                    if (!PublishFault(publish_state,
                            UpdateFault(WHOLE_BODY_ERR_STATE,
                                FaultContext::kStateAdapter,
                                nullptr, true))) {
                        return 1;
                    }
                } else {
                    RecoverFault(FaultContext::kStateAdapter);
                    last_valid_state_ = state;
                    if (!PublishState(publish_state, state, current_fault_)) return 1;
                    const auto command = receive_command();
                    if (command) {
                        whole_body_joint_command hardware_command{};
                        if (!ConvertControlCommand(
                                *command, hardware_state.num_dof, &hardware_command)) {
                            runtime_logging::Log(runtime_logging::Level::kError,
                                "rejected command dimensions");
                            (void)whole_body_set_mode(device_, WHOLE_BODY_MODE_SAFETY);
                            if (!PublishFault(publish_state,
                                    UpdateFault(WHOLE_BODY_ERR_COMMAND,
                                        FaultContext::kCommandAdapter,
                                        nullptr, true))) {
                                return 1;
                            }
                        } else {
                            RecoverFault(FaultContext::kCommandAdapter);
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
                            if (result != WHOLE_BODY_OK &&
                                result != WHOLE_BODY_ERR_READ_ONLY) {
                                whole_body_diagnostics_v2 diagnostics{};
                                (void)whole_body_get_diagnostics_v2(device_, &diagnostics);
                                if (!PublishFault(publish_state,
                                        UpdateFault(result, FaultContext::kCommand,
                                            &diagnostics, true))) {
                                    return 1;
                                }
                            } else {
                                RecoverFault(FaultContext::kCommand);
                            }
                            previous_command_result = result;
                        }
                    }
                }
            } else {
                const std::string detail = whole_body_last_error(device_);
                if (read_result != previous_read_result) {
                    runtime_logging::Log(runtime_logging::Level::kWarning,
                        std::string("whole_body feedback unavailable: ") + detail);
                }
                if (detail.rfind("waiting for initial feedback", 0) != 0) {
                    feedback_fault_active = true;
                    whole_body_diagnostics_v2 diagnostics{};
                    (void)whole_body_get_diagnostics_v2(device_, &diagnostics);
                    if (!PublishFault(publish_state,
                            UpdateFault(read_result, FaultContext::kRead,
                                &diagnostics, true))) {
                        return 1;
                    }
                }
            }
            previous_read_result = read_result;

            const auto sample_time = std::chrono::steady_clock::now();
            const bool render_due =
                monitor_enabled && sample_time - last_monitor >= monitor_period;
            const bool telemetry_due = logging_config.telemetry_enabled &&
                sample_time - last_telemetry >= telemetry_period;
            if (render_due || telemetry_due) {
                whole_body_diagnostics_v2 diagnostics{};
                if (whole_body_get_diagnostics_v2(device_, &diagnostics) == WHOLE_BODY_OK) {
                    if (render_due) {
                        RenderWholeBodyDiagnostics(diagnostics, cycle_s_);
                        last_monitor = sample_time;
                    }
                    if (telemetry_due) {
                        whole_body_motor_command_diagnostics_v2 command_diagnostics{};
                        if (whole_body_get_motor_command_diagnostics_v2(
                                device_, &command_diagnostics) == WHOLE_BODY_OK) {
                            RecordWholeBodyDiagnostics(diagnostics, command_diagnostics);
                        }
                        last_telemetry = sample_time;
                    }
                }
            }

            const int tick_result = whole_body_tick(device_, MonotonicTime());
            const bool secondary_tick_timeout = feedback_fault_active &&
                tick_result == WHOLE_BODY_ERR_TIMEOUT;
            const int reported_tick_result = secondary_tick_timeout
                ? WHOLE_BODY_OK : tick_result;
            if (reported_tick_result != WHOLE_BODY_OK &&
                reported_tick_result != previous_tick_result) {
                runtime_logging::Log(
                    reported_tick_result == WHOLE_BODY_ERR_TIMEOUT
                        ? runtime_logging::Level::kWarning
                        : runtime_logging::Level::kError,
                    std::string("whole_body tick failed: ") +
                        whole_body_last_error(device_));
            } else if (reported_tick_result == WHOLE_BODY_OK &&
                previous_tick_result != WHOLE_BODY_OK) {
                runtime_logging::Log(
                    runtime_logging::Level::kInfo, "whole_body tick recovered");
            }
            if (reported_tick_result != WHOLE_BODY_OK) {
                whole_body_diagnostics_v2 diagnostics{};
                (void)whole_body_get_diagnostics_v2(device_, &diagnostics);
                const auto &fault = UpdateFault(
                    reported_tick_result, FaultContext::kTick, &diagnostics, true);
                if (!PublishFault(publish_state, fault)) return 1;
            } else if (!secondary_tick_timeout) {
                RecoverFault(FaultContext::kTick);
            }
            previous_tick_result = reported_tick_result;
            if (tick_result != WHOLE_BODY_OK && tick_result != WHOLE_BODY_ERR_TIMEOUT)
                return 1;

            next_cycle += cycle;
            const auto finished_at = std::chrono::steady_clock::now();
            if (next_cycle <= finished_at) {
                const auto behind = finished_at - next_cycle;
                const uint64_t skipped = 1 + static_cast<uint64_t>(behind / cycle);
                next_cycle += cycle * skipped;
                timing_window.AddSkippedCycles(skipped);
            }
            if (timing_window.IsDue(finished_at, logging_config.timing_window_s)) {
                const auto timing = timing_window.Consume(finished_at);
                if (logging_config.telemetry_enabled) RecordDriverTiming(timing);
            }
            std::this_thread::sleep_until(next_cycle);
        }
        return 0;
    }

private:
    robot_base::FaultSource ClassifyFaultSource(
        FaultContext context, const whole_body_diagnostics_v2 *diagnostics) const {
        if (context == FaultContext::kStateAdapter ||
            context == FaultContext::kCommandAdapter ||
            context == FaultContext::kCommand ||
            context == FaultContext::kTick) {
            return robot_base::FaultSource::CONTROL;
        }
        if (context != FaultContext::kRead || !diagnostics)
            return robot_base::FaultSource::WHOLE_BODY;

        bool motor_problem = false;
        for (uint32_t i = 0; i < diagnostics->motor_count; ++i) {
            const auto &motor = diagnostics->motors[i];
            if (!motor.feedback_received || !motor.feedback_fresh || motor.fatal_error != 0) {
                motor_problem = true;
                break;
            }
        }
        const bool imu_problem = !diagnostics->imu.feedback_received ||
            !diagnostics->imu.feedback_fresh;
        if (imu_problem && !motor_problem) return robot_base::FaultSource::IMU;
        if (motor_problem && !imu_problem) return robot_base::FaultSource::MOTOR;
        return robot_base::FaultSource::WHOLE_BODY;
    }

    robot_base::FaultCode ClassifyFaultCode(int result, FaultContext context) const {
        if (context == FaultContext::kTick)
            return result == WHOLE_BODY_ERR_TIMEOUT
                ? robot_base::FaultCode::COMMAND_TIMEOUT
                : robot_base::FaultCode::DEVICE_ERROR;
        if (context == FaultContext::kCommand)
            return result == WHOLE_BODY_ERR_TIMEOUT
                ? robot_base::FaultCode::COMMAND_TIMEOUT
                : (result == WHOLE_BODY_ERR_DEVICE
                    ? robot_base::FaultCode::DEVICE_ERROR
                    : robot_base::FaultCode::COMMAND_REJECTED);
        if (context == FaultContext::kStateAdapter ||
            context == FaultContext::kCommandAdapter) {
            return robot_base::FaultCode::INVALID_DATA;
        }
        if (context == FaultContext::kInitialization)
            return robot_base::FaultCode::DEVICE_ERROR;
        if (result == WHOLE_BODY_ERR_TIMEOUT)
            return robot_base::FaultCode::FEEDBACK_TIMEOUT;
        if (result == WHOLE_BODY_ERR_DEVICE)
            return robot_base::FaultCode::DEVICE_ERROR;
        return robot_base::FaultCode::INVALID_DATA;
    }

    const robot_base::FaultStatus &UpdateFault(int result, FaultContext context,
        const whole_body_diagnostics_v2 *diagnostics, bool active) {
        if (current_fault_.latched) {
            current_fault_.active = current_fault_.active || active;
            return current_fault_;
        }

        const std::string detail = whole_body_last_error(device_);
        auto source = ClassifyFaultSource(context, diagnostics);
        const auto code = ClassifyFaultCode(result, context);
        if ((context == FaultContext::kCommand || context == FaultContext::kTick) &&
            result == WHOLE_BODY_ERR_DEVICE) {
            source = robot_base::FaultSource::MOTOR;
        }
        current_fault_ = {};
        current_fault_.source = source;
        current_fault_.code = code;
        current_fault_.native_code = result;
        current_fault_.sequence = ++fault_sequence_;
        current_fault_.timestamp_s = MonotonicTime();
        current_fault_.detail = detail;
        current_fault_.active = active;
        current_fault_.latched = true;
        current_fault_context_ = context;
        return current_fault_;
    }

    void RecoverFault(FaultContext context) {
        if (!current_fault_context_ || *current_fault_context_ != context) return;
        current_fault_ = {};
        current_fault_context_.reset();
    }

    bool PublishState(const PublishStateCallback &publish_state,
            const robot_base::RobotData &state,
            const robot_base::FaultStatus &fault) const {
        if (publish_state(state, fault)) return true;
        runtime_logging::Log(runtime_logging::Level::kError,
            "driver state validation or transport send failed", false);
        return false;
    }

    bool PublishFault(const PublishStateCallback &publish_state,
        const robot_base::FaultStatus &fault) const {
        robot_base::RobotData state = last_valid_state_.value_or(state_template_);
        return PublishState(publish_state, state, fault);
    }

    whole_body_dev *device_ = nullptr;
    double cycle_s_ = 0.0;
    robot_base::RobotData state_template_;
    std::optional<robot_base::RobotData> last_valid_state_;
    robot_base::FaultStatus current_fault_;
    std::optional<FaultContext> current_fault_context_;
    uint64_t fault_sequence_ = 0;
};

}  // namespace

std::unique_ptr<DriverBackend> CreateWholeBodyBackend(const std::string &yaml_path) {
    return std::make_unique<WholeBodyBackend>(yaml_path);
}

}  // namespace driver_runtime
