/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file service.h
 * @brief Operator requests and control leases; FSM remains in control
 */
#ifndef SERVICE_H
#define SERVICE_H
#include <cstdint>
#include <future>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include "protocol.h"
#include "robot_base.h"
namespace operator_service {
struct TeleopConfig {
    double deadzone = 0.06;
    Velocity acceleration{1.0, 1.0, 2.0};
    Velocity deceleration{2.0, 2.0, 4.0};
    std::optional<Velocity> slow_limits;
    std::optional<Velocity> fast_limits;
};
struct Config {
    std::string robot;
    bool terminal_only = true;
    std::string bind_address = "127.0.0.1";
    uint16_t port = 8765;
    std::string public_url;
    std::string token;
    std::string control_token;
    std::string admin_password_hash;
    std::string connection_file;
    std::string credential_file;
    std::string qr_file;
    std::string web_root;
    std::string preview_policy;
    uint16_t preview_port = 0;
    int voice_chat_port = 0;  // >0 时网页显示「语音对话」，链到同一主机这个端口的 http 入口
    double heartbeat_hz = 20;
    double status_timeout = 0.6;
    double request_timeout = 15;
    int lease_ms = 1000;
    Velocity velocity_step{0.1, 0.1, 0.1};
    TeleopConfig teleop;
    std::vector<Policy> policies;
    bool trajectory_enabled = false;
    std::vector<Action> actions;
    // enabled_in of policies and of their actions, keyed by policy name; the
    // standalone trajectory catalog uses the empty key. Clients only receive
    // the entries enabled in the environment reported by the driver.
    std::map<std::string, robot_base::EnvironmentSet> policy_environments;
    std::map<std::string, std::map<std::string, robot_base::EnvironmentSet>> action_environments;
    std::map<std::string, robot_base::EnvironmentSet> planner_environments;  // Keyed by planner name.
    std::vector<std::string> joint_names;
    std::string feedback_frame_id;
};
Config LoadConfig(const std::string &path);
std::string DefaultConnectionFile(const std::string &robot);
void PreparePairing(Config *config);
std::string PairingSvg(const std::string &url);
std::string PairingUrl(const Config &config);
class Service {
public:
    explicit Service(Config config);
    uint64_t Open(std::string preview_ticket = {});
    void Disconnect(uint64_t session);
    Json Handle(uint64_t session, const Json &request);
    void Tick();
    void UpdateFeedback(const robot_base::RobotFeedback &sample);
    void UpdateStatus(const robot_base::ControlStatus &status, const robot_base::FaultStatus &fault);
    Status Snapshot(uint64_t session = 0) const;
    Json StatusMessage(uint64_t session) const;
    RobotFeedback FeedbackSnapshot() const;
    bool FeedbackSubscribed(uint64_t session) const;
    Json Catalog() const;
    /** @brief Changes whenever Catalog() would return different entries. */
    uint64_t CatalogRevision() const { return catalog_revision_; }
    robot_base::Command Command() const;
    void MarkCommandSent() { command_sent_ = true; }
    uint64_t AcknowledgeSequence() const { return acknowledge_sequence_; }
    bool Authenticated(uint64_t session) const;
    bool PreviewAuthorized(const std::string &ticket) const;
    bool PreviewActive() const;
    const Config &Configuration() const { return config_; }

private:
    struct Session {
        bool authenticated = false;
        bool admin = false;
        bool feedback_subscribed = false;
        std::string preview_ticket;
        std::string login_error;
        std::string name;
        uint64_t last_id = 0;
    };
    Json ReplyTo(uint64_t id, bool ok, const std::string &code, const std::string &message,
        const Json &data = Json::object()) const;
    bool Online() const;
    void CancelPendingPlannerStart();
    void ClearInput(const std::string &reason, bool immediate = false);
    void ClearVelocity(bool immediate);
    void AdvanceVelocity(double now);
    Velocity TeleopTarget(const Policy &policy) const;
    bool SelectPreset(uint64_t session, const std::string &preset);
    void Finish(const std::string &phase, const std::string &message);
    const Policy *ActivePolicy() const;
    // Velocity range, presets and smoothing in effect: the running planner's, else the policy's.
    const Policy *CommandPolicy() const;
    const Planner *ActivePlanner(const std::string &name) const;
    bool PlannerEnabled(const std::string &planner) const;
    bool PolicyEnabled(const std::string &policy) const;
    bool ActionEnabled(const std::string &policy, const std::string &action) const;
    std::string EnvironmentMessage(const std::string &item) const;
    Config config_;
    std::map<std::string, Policy> planner_profiles_;  // Planner velocity settings, as a Policy.
    uint64_t catalog_revision_ = 0;
    std::map<uint64_t, Session> sessions_;
    uint64_t next_session_ = 0;
    uint64_t owner_ = 0;
    double lease_until_ = 0;
    double velocity_until_ = 0;
    double velocity_at_ = 0;
    Velocity velocity_input_;
    Velocity target_velocity_;
    bool normalized_input_ = false;
    double status_at_ = -1;
    int status_samples_ = 0;
    double request_until_ = 0;
    double reference_until_ = 0;
    robot_base::ControlStatus status_;
    robot_base::RobotFeedback feedback_;
    double feedback_at_ = 0;
    robot_base::FaultStatus fault_;
    robot_base::Command command_;
    uint64_t acknowledge_sequence_ = 0;
    uint64_t policy_sequence_ = 0;
    bool command_sent_ = false;
    std::string teleop_preset_ = "slow";
    uint64_t teleop_preset_sequence_ = 0;
    RequestResult result_;
    std::string pending_target_;
    std::string planner_preset_argument_;
    robot_base::ControlMode source_mode_ = robot_base::ControlMode::POWER_OFF;
    std::future<bool> login_result_;
    uint64_t login_session_ = 0;
    int login_attempts_ = 0;
    double login_window_until_ = 0;
};
} // namespace operator_service
#endif // SERVICE_H
