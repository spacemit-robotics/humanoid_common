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
#include "latest_sample_worker.h"
#include "runtime_logger.h"
#include "runtime_timing.h"
#include "snapshot_worker.h"
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

using DiagnosticsSnapshot = WholeBodyDiagnosticsSnapshot;

struct DriverTimingSnapshot {
    runtime_timing::Summary timing;
    uint64_t dropped = 0;
    bool failed = false;
    double wall_time = 0;
};

void RecordDriverTiming(const runtime_timing::Summary &timing, uint64_t dropped, bool failed, double wall_time) {
    std::ostringstream row;
    row << std::fixed << std::setprecision(6) << wall_time << ","
        << timing.samples << "," << timing.period_p95_ms << ","
        << timing.period_p99_ms << "," << timing.period_max_ms << ","
        << timing.lateness_p95_ms << "," << timing.lateness_p99_ms << ","
        << timing.lateness_max_ms << "," << timing.skipped_cycles << ","
        << timing.read_mean_ms << "," << timing.read_max_ms << ","
        << timing.work_mean_ms << "," << timing.work_max_ms << "," << dropped << "," << failed;
    runtime_logging::RecordCsv("driver_timing",
        "wall_time_s,samples,period_p95_ms,period_p99_ms,period_max_ms,"
        "lateness_p95_ms,lateness_p99_ms,lateness_max_ms,skipped_cycles,"
        "read_mean_ms,read_max_ms,work_mean_ms,work_max_ms,telemetry_dropped,telemetry_failed",
        row.str());
}

class WholeBodyBackend final : public DriverBackend {
public:
    explicit WholeBodyBackend(const std::string &yaml_path) {
        const auto yaml_file = robot_base::YamlFile::Load(yaml_path);
        const bool fixed_base = yaml_file.Read<bool>("robot_base.fixed_base").value_or(false);
        state_template_ = robot_base::RobotData::FromYaml(yaml_path);
        const auto hardware_config = yaml_file.Read<std::string>("whole_body.config_file");
        if (hardware_config) {
            runtime_logging::RecordArtifact("hardware_config", "whole_body",
                yaml_file.ToAbsPath(*hardware_config));
        }
        if (whole_body_create(yaml_path.c_str(), &device_) != WHOLE_BODY_OK)
            throw std::runtime_error("failed to create whole_body from YAML");
        const int has_imu = whole_body_has_imu(device_);
        if (has_imu < 0 || (has_imu == 0 && !fixed_base)) {
            whole_body_destroy(device_);
            device_ = nullptr;
            throw std::runtime_error(has_imu < 0
                ? "failed to query whole_body IMU configuration"
                : "whole_body without IMU requires explicit robot_base.fixed_base: true");
        }
        imu_configured_ = has_imu != 0;
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
            return init_result == WHOLE_BODY_ERR_DEVICE ? kHardwareInitUnavailable : 1;
        }

        const auto logging_config = runtime_logging::GetConfig();
        const bool monitor_enabled =
            logging_config.driver_monitor_enabled && isatty(STDOUT_FILENO);
        runtime_logging::LatestSampleWorker<DiagnosticsSnapshot> diagnostics_worker(
            [this](const DiagnosticsSnapshot &sample) {
                if (sample.render) RenderWholeBodyDiagnostics(sample.feedback, cycle_s_, imu_configured_);
                if (sample.record) {
                    RecordWholeBodyDiagnostics(sample.feedback, sample.command, sample.wall_time, imu_configured_);
                    RecordTransportDiagnostics(sample.transport, sample.wall_time);
                }
            });
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
        bool telemetry_motors_enabled = false;
        runtime_logging::SnapshotWorker<DriverTimingSnapshot, 4> timing_worker(
            [](const DriverTimingSnapshot &sample) {
                RecordDriverTiming(sample.timing, sample.dropped, sample.failed, sample.wall_time);
            });
        runtime_timing::Window timing_window(next_cycle);
        robot_base::RobotData state = state_template_;
        auto wait_for_cycle = [&](const auto &cycle_started, const auto &read_finished) {
            next_cycle += cycle;
            const auto finished_at = std::chrono::steady_clock::now();
            timing_window.ObserveWork(
                std::chrono::duration<double, std::milli>(read_finished - cycle_started).count(),
                std::chrono::duration<double, std::milli>(finished_at - cycle_started).count());
            if (next_cycle <= finished_at) {
                const auto behind = finished_at - next_cycle;
                const uint64_t skipped = 1 + static_cast<uint64_t>(behind / cycle);
                next_cycle += cycle * skipped;
                timing_window.AddSkippedCycles(skipped);
            }
            if (timing_window.IsDue(finished_at, logging_config.timing_window_s)) {
                const auto timing = timing_window.Consume(finished_at);
                if (logging_config.telemetry_enabled)
                    timing_worker.Publish({timing, diagnostics_worker.Dropped(), diagnostics_worker.Failed(), WallTime()});
            }
            std::this_thread::sleep_until(next_cycle);
        };
        while (should_continue()) {
            const auto cycle_started = std::chrono::steady_clock::now();
            timing_window.Observe(cycle_started, next_cycle);
            // Consume old motion commands even while waiting for hardware.
            const auto command = receive_command();
            if (command) telemetry_motors_enabled = command->enable;
            bool hardware_cycle_failed = false;
            whole_body_state hardware_state{};
            const int read_result = whole_body_read(device_, &hardware_state);
            const auto read_finished = std::chrono::steady_clock::now();
            bool feedback_fault_active = false;
            if (read_result == WHOLE_BODY_OK || read_result == WHOLE_BODY_RETRY) {
                RecoverFault(FaultContext::kRead);
                if (read_result != previous_read_result) {
                    if (read_result == WHOLE_BODY_RETRY)
                        runtime_logging::TriggerSnapshot("temporary hardware feedback delay");
                    runtime_logging::Log(
                        read_result == WHOLE_BODY_RETRY
                            ? runtime_logging::Level::kWarning : runtime_logging::Level::kInfo,
                        read_result == WHOLE_BODY_RETRY
                            ? "temporary feedback delay; retaining current motor state"
                            : "whole_body feedback recovered");
                }
                if (!ConvertWholeBodyState(hardware_state, &state, imu_configured_)) {
                    hardware_cycle_failed = true;
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
                    const bool accepts_command = !recovery_active_ &&
                        (!await_power_off_ || (command && !command->enable &&
                            command->mode == robot_base::ControlMode::POWER_OFF));
                    if (command && accepts_command) {
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
                                if (result == WHOLE_BODY_RETRY)
                                    runtime_logging::TriggerSnapshot("temporary hardware command retry");
                                runtime_logging::Log(result == WHOLE_BODY_RETRY
                                        ? runtime_logging::Level::kWarning : runtime_logging::Level::kError,
                                    std::string(result == WHOLE_BODY_RETRY
                                        ? "whole_body command retry: " : "whole_body command failed: ") +
                                        whole_body_last_error(device_));
                            } else if (result == WHOLE_BODY_OK &&
                                previous_command_result != WHOLE_BODY_OK) {
                                runtime_logging::Log(runtime_logging::Level::kInfo,
                                    "whole_body command path recovered");
                            }
                            if (result < WHOLE_BODY_OK &&
                                result != WHOLE_BODY_ERR_READ_ONLY) {
                                hardware_cycle_failed = result == WHOLE_BODY_ERR_DEVICE;
                                whole_body_diagnostics_v2 diagnostics{};
                                (void)whole_body_get_diagnostics_v2(device_, &diagnostics);
                                if (!PublishFault(publish_state,
                                        UpdateFault(result, FaultContext::kCommand,
                                            &diagnostics, true))) {
                                    return 1;
                                }
                            } else if (result == WHOLE_BODY_OK) {
                                if (await_power_off_) {
                                    await_power_off_ = false;
                                    current_fault_ = {};
                                    current_fault_context_.reset();
                                }
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
                    hardware_cycle_failed = true;
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
            const bool idle_telemetry = !telemetry_motors_enabled &&
                logging_config.idle_rate_hz > 0.0 && !runtime_logging::HighRateRequested();
            const double sample_period = idle_telemetry ? 1.0 / logging_config.idle_rate_hz :
                std::chrono::duration<double>(telemetry_period).count();
            const bool telemetry_due = logging_config.telemetry_enabled &&
                std::chrono::duration<double>(sample_time - last_telemetry).count() >= sample_period;
            if (render_due || telemetry_due) {
                DiagnosticsSnapshot snapshot;
                snapshot.wall_time = WallTime();
                if (whole_body_get_diagnostics_v2(device_, &snapshot.feedback) == WHOLE_BODY_OK) {
                    if (render_due) {
                        snapshot.render = true;
                        last_monitor = sample_time;
                    }
                    if (telemetry_due) {
                        if (whole_body_get_motor_command_diagnostics_v2(
                                device_, &snapshot.command) == WHOLE_BODY_OK) {
                            (void)whole_body_get_transport_diagnostics(device_, &snapshot.transport);
                            snapshot.record = true;
                        }
                        last_telemetry = sample_time;
                    }
                    diagnostics_worker.Publish(snapshot);
                }
            }

            const int tick_result = whole_body_tick(device_, MonotonicTime());
            const bool secondary_tick_timeout = feedback_fault_active &&
                tick_result == WHOLE_BODY_ERR_TIMEOUT;
            const int reported_tick_result = secondary_tick_timeout
                ? WHOLE_BODY_OK : tick_result;
            if (reported_tick_result != WHOLE_BODY_OK &&
                reported_tick_result != previous_tick_result) {
                runtime_logging::TriggerSnapshot("hardware communication/tick fault");
                runtime_logging::Log(
                    reported_tick_result == WHOLE_BODY_ERR_TIMEOUT || reported_tick_result == WHOLE_BODY_RETRY
                        ? runtime_logging::Level::kWarning
                        : runtime_logging::Level::kError,
                    std::string(reported_tick_result == WHOLE_BODY_RETRY
                        ? "whole_body tick retry: " : "whole_body tick failed: ") +
                        whole_body_last_error(device_));
            } else if (reported_tick_result == WHOLE_BODY_OK &&
                previous_tick_result != WHOLE_BODY_OK) {
                runtime_logging::Log(
                    runtime_logging::Level::kInfo, "whole_body tick recovered");
            }
            if (reported_tick_result < WHOLE_BODY_OK) {
                whole_body_diagnostics_v2 diagnostics{};
                (void)whole_body_get_diagnostics_v2(device_, &diagnostics);
                const auto &fault = UpdateFault(
                    reported_tick_result, FaultContext::kTick, &diagnostics, true);
                if (!PublishFault(publish_state, fault)) return 1;
            } else if (reported_tick_result == WHOLE_BODY_OK && !secondary_tick_timeout) {
                RecoverFault(FaultContext::kTick);
            }
            previous_tick_result = reported_tick_result;
            if (tick_result < WHOLE_BODY_OK && tick_result != WHOLE_BODY_ERR_TIMEOUT &&
                tick_result != WHOLE_BODY_ERR_DEVICE)
                return 1;
            if (hardware_cycle_failed || tick_result == WHOLE_BODY_ERR_DEVICE) {
                if (!recovery_active_) {
                    // A disabled Tick() failure does not itself latch safety.
                    (void)whole_body_set_mode(device_, WHOLE_BODY_MODE_SAFETY);
                    runtime_logging::Log(runtime_logging::Level::kWarning,
                        "hardware unavailable; keeping operator access, retrying disabled state");
                }
                recovery_active_ = true;
                await_power_off_ = true;
                // Continue polling at the normal cycle, including while disabled.
            } else if (recovery_active_ && read_result == WHOLE_BODY_OK &&
                tick_result == WHOLE_BODY_OK) {
                recovery_active_ = false;
                current_fault_.active = false;
                runtime_logging::Log(runtime_logging::Level::kInfo,
                    "hardware communication recovered; motors remain disabled, "
                    "return to POWER_OFF and acknowledge the fault before enabling");
            }
            wait_for_cycle(cycle_started, read_finished);
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
        const bool imu_problem = imu_configured_ &&
            (!diagnostics->imu.feedback_received || !diagnostics->imu.feedback_fresh);
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
        if (recovery_active_ || await_power_off_) return;
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
    bool imu_configured_ = true;
    double cycle_s_ = 0.0;
    bool recovery_active_ = false;
    bool await_power_off_ = false;
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
