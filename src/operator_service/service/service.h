/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file service.h
 * @brief Operator requests and control leases; FSM remains in control
 */
#ifndef SERVICE_H
#define SERVICE_H
#include <cstdint>
#include <map>
#include <string>
#include <vector>

#include "protocol.h"
#include "robot_base.h"
namespace operator_service {
struct Config {
    std::string robot;
    bool terminal_only = true;
    std::string bind_address = "127.0.0.1";
    uint16_t port = 8765;
    std::string public_url;
    std::string token;
    std::string connection_file;
    std::string credential_file;
    std::string qr_file;
    std::string web_root;
    double heartbeat_hz = 20;
    double status_timeout = 0.6;
    double request_timeout = 15;
    int lease_ms = 1000;
    Velocity velocity_step{0.1, 0.1, 0.1};
    std::vector<Policy> policies;
    bool trajectory_enabled = false;
    std::vector<Action> actions;
};
Config LoadConfig(const std::string &path);
std::string DefaultConnectionFile(const std::string &robot);
void PreparePairing(Config *config);
std::string PairingSvg(const std::string &url);
std::string PairingUrl(const Config &config);
class Service {
public:
    explicit Service(Config config);
    uint64_t Open();
    void Disconnect(uint64_t session);
    Json Handle(uint64_t session, const Json &request);
    void Tick();
    void UpdateStatus(const robot_base::ControlStatus &status, const robot_base::FaultStatus &fault);
    Status Snapshot(uint64_t session = 0) const;
    Json Catalog() const;
    robot_base::Command Command() const;
    uint64_t AcknowledgeSequence() const { return acknowledge_sequence_; }
    bool Authenticated(uint64_t session) const;
    const Config &Configuration() const { return config_; }

private:
    struct Session {
        bool authenticated = false;
        std::string name;
        uint64_t last_id = 0;
    };
    Json ReplyTo(uint64_t id, bool ok, const std::string &code, const std::string &message,
        const Json &data = Json::object()) const;
    bool Online() const;
    void ClearInput(const std::string &reason);
    void Finish(const std::string &phase, const std::string &message);
    const Policy *ActivePolicy() const;
    Config config_;
    std::map<uint64_t, Session> sessions_;
    uint64_t next_session_ = 0;
    uint64_t owner_ = 0;
    double lease_until_ = 0;
    double velocity_until_ = 0;
    double status_at_ = -1;
    int status_samples_ = 0;
    double request_until_ = 0;
    double reference_until_ = 0;
    robot_base::ControlStatus status_;
    robot_base::FaultStatus fault_;
    robot_base::Command command_;
    uint64_t acknowledge_sequence_ = 0;
    RequestResult result_;
    std::string pending_target_;
    robot_base::ControlMode source_mode_ = robot_base::ControlMode::POWER_OFF;
};
} // namespace operator_service
#endif // SERVICE_H
