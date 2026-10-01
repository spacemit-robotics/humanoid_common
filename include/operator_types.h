/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file operator_types.h
 * @brief Public operator types independent of internal transport
 */
#ifndef OPERATOR_TYPES_H
#define OPERATOR_TYPES_H
#include <cstdint>
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
    RequestResult request;
};
struct Reply {
    bool ok = false;
    std::string code;
    std::string message;
    RequestResult request;
};
struct Connection {
    std::string endpoint;
    std::string token;
    std::string name = "terminal";
    int timeout_ms = 1500;
};
} // namespace operator_service
#endif // OPERATOR_TYPES_H
