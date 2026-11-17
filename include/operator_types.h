/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file operator_types.h
 * @brief Public operator types independent of internal transport
 */
#ifndef OPERATOR_TYPES_H
#define OPERATOR_TYPES_H
#include <algorithm>
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>
namespace operator_service {
struct Velocity {
    double vx = 0;
    double vy = 0;
    double wz = 0;
};
struct Action {
    std::string key;
    std::string display_name;
};
struct Policy {
    std::string name;
    bool manual_reference = false;
    Velocity minimum;
    Velocity maximum;
    std::vector<Action> actions;
    bool client_smoothing = true;  // Legacy name: service ramps; false delegates smoothing to the policy.
    std::string switch_group;  // Explicitly compatible live-switch policies; empty disables it.
    std::string display_name;
    std::optional<Velocity> slow_limits;
    std::optional<Velocity> fast_limits;
    bool HasSpeedModes() const {
        if (!slow_limits || !fast_limits) return false;
        const auto different = [](double low, double high, double slow, double fast) {
            return std::min(std::max(0.0, -low), slow) != std::min(std::max(0.0, -low), fast) ||
                std::min(std::max(0.0, high), slow) != std::min(std::max(0.0, high), fast);
        };
        return different(minimum.vx, maximum.vx, slow_limits->vx, fast_limits->vx) ||
            different(minimum.vy, maximum.vy, slow_limits->vy, fast_limits->vy) ||
            different(minimum.wz, maximum.wz, slow_limits->wz, fast_limits->wz);
    }
};
struct Fault {
    bool active = false;
    bool latched = false;
    uint64_t sequence = 0;
    std::string source;
    std::string code;
    std::string detail;
};
struct RequestResult {
    uint64_t sequence = 0;
    std::string operation;
    std::string phase;
    std::string message;
};
struct Status {
    bool online = false;
    bool hmi_connected = false;
    bool zero_ready = false;
    std::string state = "POWER_OFF";
    std::string policy;
    double age_ms = 0;
    double rl_hz = 0;
    Velocity velocity;
    Fault fault;
    std::string owner;
    bool owns_control = false;
    double lease_remaining_ms = 0;
    std::string interaction_phase = "IDLE";
    std::string interaction_action;
    double interaction_progress = 0;
    uint64_t interaction_sequence = 0;
    bool interaction_accepted = false;
    // Client-selected speed preset, not the locomotion network's walk/run branch.
    std::string teleop_preset;
    uint64_t teleop_preset_sequence = 0;
    RequestResult request;
    bool trajectory_enabled = false;
    Velocity target_velocity;
    Velocity command_velocity;
};
struct Reply {
    bool ok = false;
    std::string code;
    std::string message;
    RequestResult request;
};
/** @brief Read-only joint and base-IMU feedback; no world position or odometry. */
struct RobotFeedback {
    bool has_base_imu = false, has_acceleration = false;
    bool available = false;
    bool fresh = false;
    uint64_t stream_id = 0;  // Changes when the driver stream restarts.
    uint64_t sequence = 0;
    double source_time_s = 0;  // Driver clock, not Unix time.
    double received_at_unix_s = 0;  // Driver receipt time, not hardware capture time.
    double age_ms = 0;  // Requires synchronized system clocks across hosts.
    double stale_after_ms = 0;
    std::string frame_id;  // Configured base frame; empty when unspecified.
    std::vector<std::string> joint_names;
    std::vector<double> joint_positions;  // rad
    std::vector<double> joint_velocities;  // rad/s
    std::vector<double> joint_efforts;  // Nm
    std::array<double, 4> orientation{1, 0, 0, 0};  // wxyz; expressed in the driver's base convention.
    std::array<double, 3> angular_velocity{};
    std::array<double, 3> acceleration{};
};
struct Connection {
    std::string endpoint;
    std::string token;
    std::string name = "terminal";
    int timeout_ms = 1500;
};
} // namespace operator_service
#endif // OPERATOR_TYPES_H
