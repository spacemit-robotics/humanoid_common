/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file service.cpp
 * @brief Authenticated operation requests, leases and control result tracking
 */
#include "service.h"

#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem> // NOLINT(build/c++17): this module requires C++17.
#include <stdexcept>
#include <utility>

#include "policy_command_limits.h"
#include "qrcodegen.hpp"
#include "runtime_logger.h"

namespace operator_service {
namespace {
double Now() { return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
const char *StateName(robot_base::ControlMode mode) {
    using M = robot_base::ControlMode;
    switch (mode) {
    case M::POWER_OFF:
        return "POWER_OFF";
    case M::DAMP:
        return "DAMP";
    case M::HOME:
        return "HOME";
    case M::ZERO:
        return "ZERO";
    case M::RL:
        return "RL";
    case M::TRAJECTORY:
        return "TRAJECTORY";
    case M::SAFETY:
        return "SAFETY";
    }
    return "UNKNOWN";
}
const char *PhaseName(robot_base::InteractionStatus::Phase phase) {
    using P = robot_base::InteractionStatus::Phase;
    switch (phase) {
    case P::IDLE:
        return "IDLE";
    case P::BLEND_IN:
        return "BLEND_IN";
    case P::PLAYING:
        return "PLAYING";
    case P::HOLDING:
        return "HOLDING";
    case P::BLEND_OUT:
        return "BLEND_OUT";
    case P::FINISHED:
        return "FINISHED";
    case P::REJECTED:
        return "REJECTED";
    }
    return "UNKNOWN";
}
bool Busy(robot_base::InteractionStatus::Phase p) {
    using P = robot_base::InteractionStatus::Phase;
    return p == P::BLEND_IN || p == P::PLAYING || p == P::HOLDING || p == P::BLEND_OUT;
}
bool SupportsInteraction(robot_base::ControlMode mode) {
    return mode == robot_base::ControlMode::RL || mode == robot_base::ControlMode::TRAJECTORY;
}
std::vector<Action> LoadActions(const robot_base::YamlFile &yaml, const std::string &catalog) {
    std::vector<Action> actions;
    for (const auto &key :
        yaml.Read<std::vector<std::string>>(catalog + ".action_names").value_or(std::vector<std::string>{})) {
        actions.push_back({key, yaml.Read<std::string>(catalog + ".actions." + key + ".display_name").value_or(key)});
    }
    return actions;
}
bool TokenEqual(const std::string &a, const std::string &b) {
    if (a.size() != b.size() || a.empty()) return false;
    unsigned difference = 0;
    for (size_t i = 0; i < a.size(); ++i) difference |= a[i] ^ b[i];
    return difference == 0;
}
std::string StateRoot() {
    const char *state = std::getenv("XDG_STATE_HOME");
    const char *home = std::getenv("HOME");
    if (state && *state) return state;
    if (home && *home) return std::string(home) + "/.local/state";
    throw std::runtime_error("operator service requires HOME or XDG_STATE_HOME");
}
} // namespace

std::string DefaultConnectionFile(const std::string &robot) {
    return StateRoot() + "/humanoid-operator/" + robot + "/connection.json";
}

std::string PairingUrl(const Config &config) { return config.public_url + "/#token=" + config.token; }

std::string PairingSvg(const std::string &url) {
    const auto qr = qrcodegen::QrCode::encodeText(url.c_str(), qrcodegen::QrCode::Ecc::LOW);
    std::string svg = "<svg xmlns=\"http://www.w3.org/2000/svg\" viewBox=\"0 0 " + std::to_string(qr.getSize() + 8) +
        " " + std::to_string(qr.getSize() + 8) +
        "\"><rect width=\"100%\" height=\"100%\" "
        "fill=\"white\"/><path fill=\"black\" d=\"";
    for (int y = 0; y < qr.getSize(); ++y)
        for (int x = 0; x < qr.getSize(); ++x)
            if (qr.getModule(x, y)) svg += "M" + std::to_string(x + 4) + "," + std::to_string(y + 4) + "h1v1h-1z ";
    return svg + "\"/></svg>";
}

Config LoadConfig(const std::string &path) {
    const auto yaml = robot_base::YamlFile::Load(path);
    Config c;
    c.terminal_only = yaml.Read<std::string>("driver.backend").value_or("mujoco") != "whole_body";
    c.robot = yaml.Read<std::string>("robot_base.name").value_or("robot");
    if (c.robot.empty() || c.robot.size() > 64 ||
        c.robot.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-") !=
            std::string::npos)
        throw std::runtime_error("invalid robot name");
    c.bind_address = yaml.Read<std::string>("operator_service.bind_address").value_or("127.0.0.1");
    const int port = yaml.Read<int>("operator_service.port").value_or(8765);
    if (port < 1 || port > 65535) throw std::runtime_error("invalid operator port");
    c.port = static_cast<uint16_t>(port);
    c.public_url = yaml.Read<std::string>("operator_service.public_url").value_or("");
    c.connection_file =
        yaml.Read<std::string>("operator_service.connection_file").value_or(DefaultConnectionFile(c.robot));
    const auto pairing_dir = StateRoot() + "/humanoid-operator/" + c.robot;
    c.credential_file =
        yaml.Read<std::string>("operator_service.credential_file").value_or(pairing_dir + "/credential");
    c.qr_file = yaml.Read<std::string>("operator_service.qr_file").value_or(pairing_dir + "/access.svg");
    char executable[4096];
    const auto length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
    std::string installed;
    if (length > 0) {
        executable[length] = '\0';
        installed =
            (std::filesystem::path(executable).parent_path().parent_path() / "share/humanoid_common/operator_web")
                .string();
    }
    const auto configured_web = yaml.Read<std::string>("operator_service.web_root");
    c.web_root =
        configured_web.value_or(std::filesystem::exists(installed + "/index.html") ? installed : OPERATOR_WEB_ROOT);
    c.heartbeat_hz = yaml.Read<double>("hmi.command_heartbeat_hz").value_or(20);
    c.status_timeout = yaml.Read<double>("hmi.status_timeout").value_or(0.6);
    c.request_timeout = yaml.Read<double>("hmi.request_timeout").value_or(15);
    c.lease_ms = yaml.Read<int>("operator_service.lease_ms").value_or(1000);
    c.velocity_step = {std::abs(yaml.Read<double>("hmi.velocity.step_vx").value_or(0.1)),
        std::abs(yaml.Read<double>("hmi.velocity.step_vy").value_or(0.1)),
        std::abs(yaml.Read<double>("hmi.velocity.step_wz").value_or(0.1))};
    if (!std::isfinite(c.velocity_step.vx) || !std::isfinite(c.velocity_step.vy) ||
        !std::isfinite(c.velocity_step.wz)) throw std::runtime_error("hmi velocity steps must be finite");
    if (!std::isfinite(c.heartbeat_hz) || c.heartbeat_hz < 10 || c.heartbeat_hz > 100 ||
        !std::isfinite(c.status_timeout) || c.status_timeout < 0.1 || !std::isfinite(c.request_timeout) ||
        c.request_timeout < 0.5 || c.lease_ms < 300 || c.lease_ms > 5000)
        throw std::runtime_error("invalid operator timing");
    c.trajectory_enabled = yaml.Read<bool>("behavior_manager.trajectory.enabled").value_or(false);
    if (c.trajectory_enabled) {
        const auto file = yaml.Read<std::string>("behavior_manager.trajectory.catalog_file");
        const auto catalog = yaml.Read<std::string>("behavior_manager.trajectory.catalog")
            .value_or("behavior_manager.trajectory");
        c.actions = LoadActions(file ? robot_base::YamlFile::Load(yaml.ToAbsPath(*file)) : yaml, catalog);
        return c;
    }
    const auto names =
        yaml.Read<std::vector<std::string>>("rl_policy.onnx_infer.policy_names").value_or(std::vector<std::string>{});
    for (const auto &name : names) {
        Policy p;
        p.name = name;
        const std::string base = "rl_policy.onnx_infer.policies." + name;
        p.manual_reference =
            yaml.Read<std::string>(base + ".policy_adapter.start_mode")
                .value_or(yaml.Read<std::string>(base + ".tracker.start_mode").value_or("")) == "manual";
        const auto limits = runtime_config::LoadOnePolicyCommandLimits(yaml, name);
        p.minimum = {limits.min_vx, limits.min_vy, limits.min_wz};
        p.maximum = {limits.max_vx, limits.max_vy, limits.max_wz};
        const std::string adapter = base + ".policy_adapter";
        const auto type = yaml.Read<std::string>(adapter + ".type").value_or("");
        const auto configured = yaml.Read<std::string>(adapter + ".catalog");
        if (type == "joint_trajectory" || (type == "sonic" && configured && !configured->empty())) {
            const std::string catalog = configured.value_or(adapter);
            p.actions = LoadActions(yaml, catalog);
        }
        c.policies.push_back(p);
    }
    return c;
}

Service::Service(Config config) : config_(std::move(config)) {}
uint64_t Service::Open() {
    if (sessions_.size() >= 32) return 0;
    const auto id = ++next_session_;
    sessions_.emplace(id, Session{});
    return id;
}
bool Service::Authenticated(uint64_t session) const {
    const auto it = sessions_.find(session);
    return it != sessions_.end() && it->second.authenticated;
}
void Service::Disconnect(uint64_t session) {
    if (owner_ == session) ClearInput("operator disconnected");
    sessions_.erase(session);
}
bool Service::Online() const {
    // Two distinct transport updates prevent a retained SHM frame granting
    // readiness.
    return status_samples_ >= 2 && status_.hmi_connected && status_at_ >= 0 &&
        Now() - status_at_ <= config_.status_timeout;
}
const Policy *Service::ActivePolicy() const {
    const auto it = std::find_if(config_.policies.begin(), config_.policies.end(),
        [this](const Policy &p) { return p.name == status_.active_policy; });
    return it == config_.policies.end() ? nullptr : &*it;
}
void Service::Finish(const std::string &phase, const std::string &message) {
    result_.phase = phase;
    result_.message = message;
    command_.key = 0;
    command_.switch_policy.clear();
    acknowledge_sequence_ = 0;
    pending_target_.clear();
    runtime_logging::Log(runtime_logging::Level::kInfo,
        "operator request #" + std::to_string(result_.sequence) + " " + result_.operation + " " + phase + ": " +
            message);
}
void Service::ClearInput(const std::string &reason) {
    if (owner_) runtime_logging::Log(runtime_logging::Level::kInfo, "operator control released: " + reason);
    if (result_.phase == "accepted") Finish("cancelled", reason);
    owner_ = 0;
    lease_until_ = 0;
    velocity_until_ = 0;
    reference_until_ = 0;
    command_.key = 0;
    command_.switch_policy.clear();
    command_.vx = command_.vy = command_.wz = 0;
    acknowledge_sequence_ = 0;
    command_.interaction.sequence = std::max(command_.interaction.sequence, status_.interaction.sequence) + 1;
    command_.interaction.operation = robot_base::InteractionRequest::Operation::CANCEL;
    command_.interaction.action.clear();
}
void Service::Tick() {
    const auto now = Now();
    if (owner_ && (now >= lease_until_ || !Online())) ClearInput("control lease or feedback expired");
    if (now >= velocity_until_ || status_.mode != robot_base::ControlMode::RL)
        command_.vx = command_.vy = command_.wz = 0;
    if (reference_until_ > 0 && now >= reference_until_) {
        command_.key = 0;
        reference_until_ = 0;
    }
    if (result_.phase == "accepted" && now >= request_until_) Finish("expired", "control did not confirm request");
    if (command_.interaction.operation != robot_base::InteractionRequest::Operation::NONE &&
        (!SupportsInteraction(status_.mode) ||
            status_.interaction.sequence == command_.interaction.sequence)) {
        command_.interaction.operation = robot_base::InteractionRequest::Operation::NONE;
        command_.interaction.action.clear();
    }
}
void Service::UpdateStatus(const robot_base::ControlStatus &status, const robot_base::FaultStatus &fault) {
    const bool changed = status.mode != status_.mode || status.active_policy != status_.active_policy;
    status_ = status;
    fault_ = fault;
    status_at_ = Now();
    status_samples_ = std::min(2, status_samples_ + 1);
    if (changed) {
        command_.vx = command_.vy = command_.wz = 0;
        reference_until_ = 0;
        if (result_.operation != "state") command_.key = 0;
    }
    if (result_.phase != "accepted") return;
    if (result_.operation == "state") {
        if (pending_target_ == StateName(status.mode)) Finish("completed", "control entered requested state");
        else if (status.mode != source_mode_) Finish("rejected", "control entered a different state");
    } else if (result_.operation == "policy" && pending_target_ == status.active_policy) {
        Finish("completed", "control selected requested policy");
    } else if ((result_.operation == "interaction" || result_.operation == "cancel") &&
        command_.interaction.sequence == status.interaction.sequence) {
        Finish(status.interaction.request_accepted ? "completed" : "rejected", "control handled action request");
        command_.interaction.operation = robot_base::InteractionRequest::Operation::NONE;
        command_.interaction.action.clear();
    } else if (result_.operation == "ack" && !fault.latched) {
        Finish("completed", "control acknowledged fault");
    }
    if (result_.phase == "accepted" && fault.latched && result_.operation != "ack" && pending_target_ != "POWER_OFF" &&
        pending_target_ != "DAMP")
        Finish("rejected", fault.detail);
}
robot_base::Command Service::Command() const { return command_; }
Status Service::Snapshot(uint64_t session) const {
    Status s;
    s.online = Online();
    s.hmi_connected = status_.hmi_connected;
    s.zero_ready = status_.zero_ready;
    s.trajectory_enabled = config_.trajectory_enabled;
    s.state = StateName(status_.mode);
    s.policy = status_.active_policy;
    s.age_ms = status_at_ < 0 ? 0 : std::max(0.0, (Now() - status_at_) * 1000);
    s.rl_hz = status_.rl_frequency_hz;
    s.velocity = {status_.vx, status_.vy, status_.wz};
    s.fault = {fault_.active, fault_.latched, fault_.sequence, robot_base::FaultSourceName(fault_.source),
        robot_base::FaultCodeName(fault_.code), fault_.detail};
    const auto it = sessions_.find(owner_);
    if (it != sessions_.end()) s.owner = it->second.name + "#" + std::to_string(owner_);
    s.owns_control = owner_ != 0 && owner_ == session && Now() < lease_until_;
    s.lease_remaining_ms = owner_ ? std::max(0.0, (lease_until_ - Now()) * 1000) : 0;
    s.interaction_phase = PhaseName(status_.interaction.phase);
    s.interaction_action = status_.interaction.action;
    s.interaction_progress = status_.interaction.progress;
    s.request = result_;
    return s;
}
Json Service::Catalog() const {
    Json policies = Json::array();
    for (const auto &p : config_.policies) policies.push_back(Encode(p));
    Json actions = Json::array();
    for (const auto &a : config_.actions) actions.push_back({{"key", a.key}, {"display_name", a.display_name}});
    return {{"robot", config_.robot}, {"policies", policies}, {"lease_ms", config_.lease_ms},
        {"velocity_step", Encode(config_.velocity_step)}, {"trajectory_enabled", config_.trajectory_enabled},
        {"actions", actions}};
}
Json Service::ReplyTo(
    uint64_t id, bool ok, const std::string &code, const std::string &message, const Json &data) const {
    return {{"v", 1}, {"id", id}, {"ok", ok}, {"code", code}, {"message", message}, {"data", data}};
}

Json Service::Handle(uint64_t session, const Json &request) {
    Tick();
    uint64_t id = 0;
    const auto fail = [&](const std::string &code, const std::string &message) {
        return ReplyTo(id, false, code, message);
    };
    try {
        if (!request.is_object() || request.value("v", 0) != 1 || !request.contains("id") ||
            !request.at("id").is_number_unsigned())
            return fail("bad_request", "invalid version or id");
        id = request.at("id").get<uint64_t>();
        auto it = sessions_.find(session);
        if (it == sessions_.end() || id == 0 || id > 9007199254740991ULL || id <= it->second.last_id)
            return fail("bad_request", "invalid or replayed request id");
        it->second.last_id = id;
        const auto op = request.at("op").get<std::string>();
        const auto args = request.value("args", Json::object());
        if (!args.is_object()) return fail("bad_request", "args must be an object");
        if (op == "hello") {
            if (!TokenEqual(args.value("token", ""), config_.token)) return fail("unauthorized", "invalid token");
            const auto name = args.value("name", "client");
            if (name.empty() || name.size() > 48 ||
                std::any_of(name.begin(), name.end(), [](unsigned char c) { return c < 32 || c == 127; }))
                return fail("bad_request", "invalid client name");
            it->second.authenticated = true;
            it->second.name = name;
            return ReplyTo(
                id, true, "ok", "authenticated", {{"catalog", Catalog()}, {"status", Encode(Snapshot(session))}});
        }
        if (!it->second.authenticated) return fail("unauthorized", "authenticate first");
        if (op == "status") return ReplyTo(id, true, "ok", "", {{"status", Encode(Snapshot(session))}});
        if (op == "catalog") return ReplyTo(id, true, "ok", "", {{"catalog", Catalog()}});
        if (op == "pairing") {
            if (config_.terminal_only) return fail("unsupported", "simulation supports the local terminal only");
            const std::string url = PairingUrl(config_);
            return ReplyTo(id, true, "ok", "", {{"url", url}, {"qr_svg", PairingSvg(url)}});
        }
        if (op == "stop") {
            ClearInput("stopped by authenticated client");
            return ReplyTo(id, true, "ok", "velocity cleared; pending requests cancelled");
        }
        if (op == "release") {
            if (owner_ == session) ClearInput("control released");
            return ReplyTo(id, true, "ok", "released");
        }
        if (op == "acquire") {
            if (!Online()) return fail("not_ready", "waiting for fresh control status");
            if (owner_ && owner_ != session) return fail("busy", "another client owns control");
            owner_ = session;
            lease_until_ = Now() + config_.lease_ms / 1000.0;
            runtime_logging::Log(
                runtime_logging::Level::kInfo, "operator control acquired: session=" + std::to_string(session));
            return ReplyTo(id, true, "ok", "control acquired");
        }
        if (op == "renew") {
            if (owner_ != session) return fail("not_owner", "acquire control first");
            lease_until_ = Now() + config_.lease_ms / 1000.0;
            return ReplyTo(id, true, "ok", "control renewed");
        }
        if (!Online()) return fail("not_ready", "control feedback is unavailable");
        const auto target = args.value("state", "");
        const bool safety_request = op == "state" && (target == "DAMP" || target == "POWER_OFF");
        if (owner_ != session && !safety_request) return fail("not_owner", "acquire control first");
        if (safety_request) {
            const bool was_owner = owner_ == session;
            ClearInput("safety state requested");
            if (was_owner) {
                owner_ = session;
                lease_until_ = Now() + config_.lease_ms / 1000.0;
            }
        }
        if (op == "velocity") {
            if (config_.trajectory_enabled || status_.mode != robot_base::ControlMode::RL || fault_.latched)
                return fail("not_ready", "velocity is only available in healthy RL");
            const auto *policy = ActivePolicy();
            if (!policy) return fail("not_ready", "active policy has no catalog");
            const double vx = args.at("vx"), vy = args.at("vy"), wz = args.at("wz");
            const int ttl = args.value("valid_for_ms", 300);
            if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) || ttl < 50 || ttl > 1000)
                return fail("bad_request", "invalid velocity or validity period");
            command_.vx = std::clamp(vx, policy->minimum.vx, policy->maximum.vx);
            command_.vy = std::clamp(vy, policy->minimum.vy, policy->maximum.vy);
            command_.wz = std::clamp(wz, policy->minimum.wz, policy->maximum.wz);
            velocity_until_ = Now() + ttl / 1000.0;
            return ReplyTo(id, true, "ok", "velocity accepted",
                {{"velocity", Encode(Velocity{command_.vx, command_.vy, command_.wz})}});
        }
        if (result_.phase == "accepted") return fail("busy", "waiting for control to confirm previous request");
        if (fault_.latched && !safety_request && op != "ack") return fail("fault", "fault is latched");
        using M = robot_base::ControlMode;
        if (op == "state") {
            const auto mode = status_.mode;
            int key = 0;
            if (target == "POWER_OFF") key = -1;
            if (target == "DAMP" && mode != M::SAFETY) key = 1;
            if (target == "HOME" && mode == M::DAMP) key = 4;
            if (target == "ZERO" && mode == M::HOME) key = 2;
            const auto active_mode = config_.trajectory_enabled ? "TRAJECTORY" : "RL";
            if (target == active_mode && mode == M::ZERO && status_.zero_ready) key = 3;
            if (target == StateName(mode)) return ReplyTo(id, true, "ok", "already in requested state");
            if (!key) return fail("not_ready", "state prerequisite is not satisfied");
            command_.key = key;
            source_mode_ = mode;
            pending_target_ = target;
        } else if (op == "policy") {
            if (config_.trajectory_enabled) return fail("not_ready", "trajectory mode has no RL policy");
            if (status_.mode != M::POWER_OFF && status_.mode != M::DAMP)
                return fail("not_ready", "select policy in POWER_OFF or DAMP");
            const auto name = args.at("policy").get<std::string>();
            if (std::none_of(
                    config_.policies.begin(), config_.policies.end(), [&](const Policy &p) { return p.name == name; }))
                return fail("bad_request", "unknown policy");
            command_.switch_policy = name;
            pending_target_ = name;
        } else if (op == "interaction" || op == "cancel") {
            const auto active_mode = config_.trajectory_enabled ? M::TRAJECTORY : M::RL;
            if (status_.mode != active_mode) return fail("not_ready", "action requires the configured active mode");
            const auto *policy = ActivePolicy();
            const auto *actions = config_.trajectory_enabled ? &config_.actions : (policy ? &policy->actions : nullptr);
            const auto action = args.value("action", "");
            if (op == "interaction" &&
                (!actions || Busy(status_.interaction.phase) ||
                    std::none_of(actions->begin(), actions->end(),
                        [&](const Action &a) { return a.key == action; })))
                return fail("not_ready", "action is unavailable or another action is busy");
            command_.interaction.sequence = std::max(command_.interaction.sequence, status_.interaction.sequence) + 1;
            command_.interaction.operation = op == "interaction"
                ? robot_base::InteractionRequest::Operation::START
                : robot_base::InteractionRequest::Operation::CANCEL;
            command_.interaction.action = op == "interaction" ? action : "";
        } else if (op == "reference_start") {
            const auto *policy = ActivePolicy();
            if (config_.trajectory_enabled || status_.mode != M::RL || !policy || !policy->manual_reference)
                return fail("not_ready", "active policy is not a manual reference policy");
            command_.key = robot_base::kCommandStartReference;
            reference_until_ = Now() + 0.25;
        } else if (op == "ack") {
            if (status_.mode != M::POWER_OFF || !fault_.latched || fault_.active || fault_.sequence == 0)
                return fail("not_ready", "acknowledge inactive fault in POWER_OFF");
            acknowledge_sequence_ = fault_.sequence;
        } else {
            return fail("bad_request", "unknown operation");
        }
        result_ = {result_.sequence + 1, op, op == "reference_start" ? "sent" : "accepted", "request sent to control"};
        request_until_ = Now() + config_.request_timeout;
        runtime_logging::Log(runtime_logging::Level::kInfo,
            "operator request #" + std::to_string(result_.sequence) + " " + op + " session=" + std::to_string(session) +
                (pending_target_.empty() ? "" : " target=" + pending_target_));
        return ReplyTo(id, true, "accepted", "waiting for control result", {{"request", Encode(result_)}});
    } catch (const Json::exception &) {
        return fail("bad_request", "invalid message fields");
    }
}
} // namespace operator_service
