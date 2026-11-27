/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file service.cpp
 * @brief Authenticated operation requests, leases and control result tracking
 */
#include "service.h"

#include <openssl/crypto.h>
#include <openssl/evp.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem> // NOLINT(build/c++17): this module requires C++17.
#include <stdexcept>
#include <utility>

#include "planner_catalog.h"
#include "policy_availability.h"
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
const char *PhaseName(robot_base::PlannerStatus::Phase phase) {
    using P = robot_base::PlannerStatus::Phase;
    switch (phase) {
    case P::IDLE:
        return "IDLE";
    case P::LOADING:
        return "LOADING";
    case P::WARMING:
        return "WARMING";
    case P::READY:
        return "READY";
    case P::GENERATING:
        return "GENERATING";
    case P::RUNNING:
        return "RUNNING";
    case P::STOPPING:
        return "STOPPING";
    case P::FAILED:
        return "FAILED";
    }
    return "UNKNOWN";
}
// The tracker follows the planner or is preparing to.
bool Engaged(robot_base::PlannerStatus::Phase p) {
    using P = robot_base::PlannerStatus::Phase;
    return p == P::LOADING || p == P::WARMING || p == P::GENERATING || p == P::RUNNING || p == P::STOPPING;
}
bool Busy(robot_base::InteractionStatus::Phase p) {
    using P = robot_base::InteractionStatus::Phase;
    return p == P::BLEND_IN || p == P::PLAYING || p == P::HOLDING || p == P::BLEND_OUT;
}
bool SupportsInteraction(robot_base::ControlMode mode) {
    return mode == robot_base::ControlMode::RL || mode == robot_base::ControlMode::TRAJECTORY;
}
double Advance(double value, double target, double acceleration, double deceleration, double dt) {
    if (value * target < 0) {
        const auto braking = std::abs(value) / deceleration;
        if (dt <= braking) return value - std::copysign(deceleration * dt, value);
        dt -= braking;
        value = 0;
    }
    const auto rate = std::abs(target) > std::abs(value) ? acceleration : deceleration;
    return value + std::clamp(target - value, -rate * dt, rate * dt);
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
constexpr char kPasswordPrefix[] = "pbkdf2-sha256:100000:";
bool ValidPasswordHash(const std::string &value) {
    const size_t prefix = sizeof(kPasswordPrefix) - 1;
    if (value.size() != prefix + 32 + 1 + 64 || value.compare(0, prefix, kPasswordPrefix) != 0 ||
        value[prefix + 32] != ':') return false;
    return value.substr(prefix, 32).find_first_not_of("0123456789abcdef") == std::string::npos &&
        value.substr(prefix + 33).find_first_not_of("0123456789abcdef") == std::string::npos;
}
bool CheckPassword(const std::string &encoded, const std::string &password) {
    const size_t prefix = sizeof(kPasswordPrefix) - 1;
    const auto byte = [](const std::string &s, size_t i) {
        const auto digit = [](char c) { return c <= '9' ? c - '0' : c - 'a' + 10; };
        return static_cast<unsigned char>((digit(s[i]) << 4) | digit(s[i + 1]));
    };
    unsigned char salt[16], expected[32], actual[32];
    for (size_t i = 0; i < sizeof(salt); ++i) salt[i] = byte(encoded, prefix + i * 2);
    for (size_t i = 0; i < sizeof(expected); ++i) expected[i] = byte(encoded, prefix + 33 + i * 2);
    return PKCS5_PBKDF2_HMAC(password.data(), static_cast<int>(password.size()), salt, sizeof(salt),
        100000, EVP_sha256(), sizeof(actual), actual) == 1 &&
        CRYPTO_memcmp(actual, expected, sizeof(actual)) == 0;
}
std::string StateRoot() {
    const char *state = std::getenv("XDG_STATE_HOME");
    const char *home = std::getenv("HOME");
    if (state && *state) return state;
    if (home && *home) return std::string(home) + "/.local/state";
    throw std::runtime_error("operator service requires HOME or XDG_STATE_HOME");
}
std::optional<Velocity> ReadTeleopVector(const robot_base::YamlFile &yaml, const std::string &name, bool positive,
    const std::string &base = "operator_service.teleop") {
    const auto key = base + "." + name;
    const auto values = yaml.Read<std::vector<double>>(key);
    if (!values) return std::nullopt;
    if (values->size() != 3 || std::any_of(values->begin(), values->end(), [positive](double value) {
            return !std::isfinite(value) || (positive ? value <= 0 : value < 0);
        }))
        throw std::runtime_error(key + " must contain three finite " + (positive ? "positive" : "nonnegative") +
            " values [vx, vy, wz]");
    return Velocity{(*values)[0], (*values)[1], (*values)[2]};
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
    c.terminal_only = yaml.Read<bool>("operator_service.terminal_only").value_or(false);
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
    c.preview_policy = yaml.Read<std::string>("operator_service.camera_preview.policy").value_or("");
    if (!c.preview_policy.empty()) {
        const int preview_port = yaml.Read<int>("operator_service.camera_preview.port").value_or(0);
        if (preview_port <= 0 || preview_port > 65535)
            throw std::runtime_error("camera_preview requires a valid local signaling port");
        c.preview_port = static_cast<uint16_t>(preview_port);
    }
    c.voice_chat_port = yaml.Read<int>("operator_service.voice_chat_port").value_or(0);
    c.admin_password_hash = yaml.Read<std::string>("operator_service.admin_password_hash").value_or("");
    if (c.voice_chat_port < 0 || c.voice_chat_port > 65535) throw std::runtime_error("invalid voice chat port");
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
    c.joint_names = yaml.Read<std::vector<std::string>>("robot_base.joint_names")
        .value_or(std::vector<std::string>{});
    c.feedback_frame_id = yaml.Read<std::string>("operator_service.feedback.frame_id").value_or("");
    c.status_timeout = yaml.Read<double>("hmi.status_timeout").value_or(0.6);
    c.request_timeout = yaml.Read<double>("hmi.request_timeout").value_or(15);
    c.lease_ms = yaml.Read<int>("operator_service.lease_ms").value_or(1000);
    c.velocity_step = {std::abs(yaml.Read<double>("hmi.velocity.step_vx").value_or(0.1)),
        std::abs(yaml.Read<double>("hmi.velocity.step_vy").value_or(0.1)),
        std::abs(yaml.Read<double>("hmi.velocity.step_wz").value_or(0.1))};
    if (!std::isfinite(c.velocity_step.vx) || !std::isfinite(c.velocity_step.vy) ||
        !std::isfinite(c.velocity_step.wz)) throw std::runtime_error("hmi velocity steps must be finite");
    c.teleop.deadzone = yaml.Read<double>("operator_service.teleop.deadzone").value_or(c.teleop.deadzone);
    if (!std::isfinite(c.teleop.deadzone) || c.teleop.deadzone < 0 || c.teleop.deadzone >= 0.5)
        throw std::runtime_error("operator_service.teleop.deadzone must be in [0, 0.5)");
    c.teleop.acceleration = ReadTeleopVector(yaml, "acceleration", true).value_or(c.teleop.acceleration);
    c.teleop.deceleration = ReadTeleopVector(yaml, "deceleration", true).value_or(c.teleop.deceleration);
    c.teleop.slow_limits = ReadTeleopVector(yaml, "slow_limits", false);
    c.teleop.fast_limits = ReadTeleopVector(yaml, "fast_limits", false);
    if (c.teleop.slow_limits && c.teleop.fast_limits) {
        const auto &slow = *c.teleop.slow_limits;
        const auto &fast = *c.teleop.fast_limits;
        if (slow.vx > fast.vx || slow.vy > fast.vy || slow.wz > fast.wz)
            throw std::runtime_error("teleop slow_limits must not exceed fast_limits");
    }
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
        c.action_environments[""] = runtime_config::LoadTrajectoryActionEnvironments(yaml);
        return c;
    }
    const auto names =
        yaml.Read<std::vector<std::string>>("rl_policy.onnx_infer.policy_names").value_or(std::vector<std::string>{});
    for (const auto &name : names) {
        Policy p;
        p.name = name;
        const std::string base = "rl_policy.onnx_infer.policies." + name;
        p.display_name = yaml.Read<std::string>(base + ".display_name").value_or(name);
        p.client_smoothing = yaml.Read<bool>(base + ".command.smoothing").value_or(
            yaml.Read<bool>(base + ".command.client_smoothing").value_or(true));
        p.switch_group = yaml.Read<std::string>(base + ".switch_group").value_or("");
        p.manual_reference =
            yaml.Read<std::string>(base + ".policy_adapter.start_mode")
                .value_or(yaml.Read<std::string>(base + ".tracker.start_mode").value_or("")) == "manual";
        const auto limits = runtime_config::LoadOnePolicyCommandLimits(yaml, name);
        p.minimum = {limits.min_vx, limits.min_vy, limits.min_wz};
        p.maximum = {limits.max_vx, limits.max_vy, limits.max_wz};
        p.slow_limits = ReadTeleopVector(yaml, "slow_limits", false, base + ".command.teleop");
        p.fast_limits = ReadTeleopVector(yaml, "fast_limits", false, base + ".command.teleop");
        const auto slow = p.slow_limits ? p.slow_limits : c.teleop.slow_limits;
        const auto fast = p.fast_limits ? p.fast_limits : c.teleop.fast_limits;
        if (slow && fast && (slow->vx > fast->vx || slow->vy > fast->vy || slow->wz > fast->wz))
            throw std::runtime_error(base + ".command.teleop slow_limits must not exceed fast_limits");
        const auto catalog = runtime_config::PolicyActionCatalog(yaml, name);
        if (!catalog.empty()) p.actions = LoadActions(yaml, catalog);
        if (catalog == base + ".policy_actions") p.manual_reference = false;
        c.policy_environments[name] = runtime_config::LoadPolicyEnvironments(yaml, name);
        c.action_environments[name] = runtime_config::LoadActionEnvironments(yaml, catalog);
        for (const auto &entry : runtime_config::LoadPolicyPlanners(yaml, name)) {
            Planner planner;
            planner.name = entry.name;
            planner.display_name = entry.display_name;
            planner.category = entry.category;
            planner.type = entry.type;
            const auto presets = entry.base + ".command.presets";
            for (size_t i = 0; yaml.Contains(presets + "[" + std::to_string(i) + "]"); ++i) {
                const auto path = presets + "[" + std::to_string(i) + "]";
                PlannerPreset preset;
                preset.key = yaml.Read<std::string>(path + ".key").value_or("");
                if (preset.key.empty() || preset.key.find(':') != std::string::npos ||
                    std::any_of(planner.presets.begin(), planner.presets.end(),
                        [&](const PlannerPreset &p) { return p.key == preset.key; }))
                    throw std::runtime_error("invalid planner preset key: " + path);
                preset.display_name = yaml.Read<std::string>(path + ".display_name").value_or(preset.key);
                preset.heights_cm = yaml.Read<std::vector<int>>(path + ".heights_cm").value_or(std::vector<int>{});
                planner.presets.push_back(std::move(preset));
            }
            planner.minimum = {entry.limits.min_vx, entry.limits.min_vy, entry.limits.min_wz};
            planner.maximum = {entry.limits.max_vx, entry.limits.max_vy, entry.limits.max_wz};
            planner.client_smoothing = yaml.Read<bool>(entry.base + ".command.smoothing").value_or(true);
            planner.slow_limits = ReadTeleopVector(yaml, "slow_limits", false, entry.base + ".command.teleop");
            planner.fast_limits = ReadTeleopVector(yaml, "fast_limits", false, entry.base + ".command.teleop");
            const auto planner_slow = planner.slow_limits ? planner.slow_limits : c.teleop.slow_limits;
            const auto planner_fast = planner.fast_limits ? planner.fast_limits : c.teleop.fast_limits;
            if (planner_slow && planner_fast && (planner_slow->vx > planner_fast->vx ||
                    planner_slow->vy > planner_fast->vy || planner_slow->wz > planner_fast->wz))
                throw std::runtime_error(
                    entry.base + ".command.teleop slow_limits must not exceed fast_limits");
            c.planner_environments[entry.name] = entry.environments;
            p.planners.push_back(std::move(planner));
        }
        p.auto_planner = yaml.Read<std::string>(base + ".policy_adapter.auto_planner").value_or("");
        c.policies.push_back(p);
    }
    return c;
}

Service::Service(Config config) : config_(std::move(config)) {
    if (!config_.admin_password_hash.empty() && !ValidPasswordHash(config_.admin_password_hash))
        throw std::runtime_error("invalid operator admin_password_hash");
    if (!config_.admin_password_hash.empty() && TokenEqual(config_.control_token, config_.token))
        throw std::runtime_error("public and control tokens must differ when administrator access is enabled");
    // Presets default to the global teleop limits, else to 40% and 100% of the range.
    const auto fill_presets = [this](const Velocity &minimum, const Velocity &maximum,
            std::optional<Velocity> *slow, std::optional<Velocity> *fast) {
        const Velocity full{std::max(-minimum.vx, maximum.vx),
            std::max(-minimum.vy, maximum.vy), std::max(-minimum.wz, maximum.wz)};
        if (!*slow) *slow = config_.teleop.slow_limits.value_or(
            Velocity{full.vx * 0.4, full.vy * 0.4, full.wz * 0.4});
        if (!*fast) *fast = config_.teleop.fast_limits.value_or(full);
    };
    for (auto &policy : config_.policies) {
        fill_presets(policy.minimum, policy.maximum, &policy.slow_limits, &policy.fast_limits);
        for (auto &planner : policy.planners) {
            fill_presets(planner.minimum, planner.maximum, &planner.slow_limits, &planner.fast_limits);
            Policy profile;
            profile.name = planner.name;
            profile.display_name = planner.display_name;
            profile.minimum = planner.minimum;
            profile.maximum = planner.maximum;
            profile.client_smoothing = planner.client_smoothing;
            profile.slow_limits = planner.slow_limits;
            profile.fast_limits = planner.fast_limits;
            planner_profiles_[planner.name] = std::move(profile);
        }
    }
    velocity_at_ = Now();
}
uint64_t Service::Open(std::string preview_ticket) {
    if (sessions_.size() >= 32) return 0;
    const auto id = ++next_session_;
    sessions_[id].preview_ticket = std::move(preview_ticket);
    return id;
}
bool Service::Authenticated(uint64_t session) const {
    const auto it = sessions_.find(session);
    return it != sessions_.end() && it->second.authenticated;
}
void Service::Disconnect(uint64_t session) {
    if (owner_ == session) ClearInput("operator disconnected");
    if (login_session_ == session) login_session_ = 0;
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
const Policy *Service::CommandPolicy() const {
    if (status_.planner.phase == robot_base::PlannerStatus::Phase::RUNNING) {
        const auto profile = planner_profiles_.find(status_.planner.planner);
        if (profile != planner_profiles_.end()) return &profile->second;
    }
    return ActivePolicy();
}
const Planner *Service::ActivePlanner(const std::string &name) const {
    const auto *policy = ActivePolicy();
    if (!policy) return nullptr;
    const auto it = std::find_if(policy->planners.begin(), policy->planners.end(),
        [&](const Planner &p) { return p.name == name; });
    return it == policy->planners.end() ? nullptr : &*it;
}
bool Service::PlannerEnabled(const std::string &planner) const {
    const auto it = config_.planner_environments.find(planner);
    return it == config_.planner_environments.end() || it->second.Allows(status_.environment);
}
bool Service::PolicyEnabled(const std::string &policy) const {
    const auto it = config_.policy_environments.find(policy);
    return it == config_.policy_environments.end() || it->second.Allows(status_.environment);
}
bool Service::ActionEnabled(const std::string &policy, const std::string &action) const {
    const auto catalog = config_.action_environments.find(policy);
    if (catalog == config_.action_environments.end()) return true;
    const auto it = catalog->second.find(action);
    return it == catalog->second.end() || it->second.Allows(status_.environment);
}
std::string Service::EnvironmentMessage(const std::string &item) const {
    return item + " is not enabled in the " + robot_base::RuntimeEnvironmentName(status_.environment) +
        " environment";
}
void Service::Finish(const std::string &phase, const std::string &message) {
    result_.phase = phase;
    result_.message = message;
    command_.key = 0;
    command_.switch_policy.clear();
    command_.policy_request_sequence = 0;
    acknowledge_sequence_ = 0;
    pending_target_.clear();
    runtime_logging::Log(runtime_logging::Level::kInfo,
        "operator request #" + std::to_string(result_.sequence) + " " + result_.operation + " " + phase + ": " +
            message);
}
void Service::ClearVelocity(bool immediate) {
    velocity_input_ = {};
    target_velocity_ = {};
    normalized_input_ = false;
    velocity_until_ = 0;
    const auto *policy = CommandPolicy();
    if (immediate || !policy || !policy->client_smoothing) command_.vx = command_.vy = command_.wz = 0;
}
Velocity Service::TeleopTarget(const Policy &policy) const {
    const auto &caps = *(teleop_preset_ == "fast" ? policy.fast_limits : policy.slow_limits);
    const auto scale = [](double input, double low, double high, double cap) {
        return input * std::min(cap, input >= 0 ? std::max(0.0, high) : std::max(0.0, -low));
    };
    return {scale(velocity_input_.vx, policy.minimum.vx, policy.maximum.vx, caps.vx),
        scale(velocity_input_.vy, policy.minimum.vy, policy.maximum.vy, caps.vy),
        scale(velocity_input_.wz, policy.minimum.wz, policy.maximum.wz, caps.wz)};
}
void Service::AdvanceVelocity(double now) {
    const auto dt = std::clamp(now - velocity_at_, 0.0, 0.1);
    velocity_at_ = now;
    const auto *policy = CommandPolicy();
    if (!Online() || !policy || status_.mode != robot_base::ControlMode::RL || fault_.latched || fault_.active) {
        ClearVelocity(true);
        return;
    }
    if (now >= velocity_until_) ClearVelocity(false);
    target_velocity_ = normalized_input_ ? TeleopTarget(*policy) : velocity_input_;
    const auto &a = config_.teleop.acceleration;
    const auto &d = config_.teleop.deceleration;
    if (policy->client_smoothing) {
        command_.vx = Advance(command_.vx, target_velocity_.vx, a.vx, d.vx, dt);
        command_.vy = Advance(command_.vy, target_velocity_.vy, a.vy, d.vy, dt);
        command_.wz = Advance(command_.wz, target_velocity_.wz, a.wz, d.wz, dt);
    } else {
        command_.vx = target_velocity_.vx;
        command_.vy = target_velocity_.vy;
        command_.wz = target_velocity_.wz;
    }
}
bool Service::SelectPreset(uint64_t session, const std::string &preset) {
    if (owner_ != session || !Online() || (preset != "slow" && preset != "fast")) return false;
    const auto *policy = CommandPolicy();
    const auto selected = policy && !policy->HasSpeedModes() ? "slow" : preset;
    if (teleop_preset_ != selected) {
        teleop_preset_ = selected;
        ++teleop_preset_sequence_;
    }
    return true;
}
void Service::CancelPendingPlannerStart() {
    if (command_.planner.operation != robot_base::PlannerRequest::Operation::START) return;
    if (command_sent_) command_.planner.StopAfter(status_.planner.sequence);
    else command_.planner.ClearOperation();
}

void Service::ClearInput(const std::string &reason, bool immediate) {
    if (owner_) runtime_logging::Log(runtime_logging::Level::kInfo, "operator control released: " + reason);
    CancelPendingPlannerStart();
    if (result_.phase == "accepted") Finish("cancelled", reason);
    owner_ = 0;
    lease_until_ = 0;
    ClearVelocity(immediate);
    reference_until_ = 0;
    command_.key = 0;
    command_.switch_policy.clear();
    command_.policy_request_sequence = 0;
    acknowledge_sequence_ = 0;
    command_.interaction.sequence = std::max(command_.interaction.sequence, status_.interaction.sequence) + 1;
    command_.interaction.operation = robot_base::InteractionRequest::Operation::CANCEL;
    command_.interaction.action.clear();
    // A motion generated by an offline planner ends with the operator's control,
    // like a catalog action. A realtime planner keeps running on a zero command.
    const auto *planner = ActivePlanner(status_.planner.planner);
    if (planner && planner->category == "offline" && Engaged(status_.planner.phase) &&
        command_.planner.operation != robot_base::PlannerRequest::Operation::STOP) {
        command_.planner.StopAfter(status_.planner.sequence);
    }
}
void Service::Tick() {
    const auto now = Now();
    // Password derivation must not block the HMI command heartbeat.
    if (login_result_.valid() && login_result_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
        const bool valid = login_result_.get();
        const auto it = sessions_.find(login_session_);
        if (it != sessions_.end()) {
            it->second.admin = valid;
            it->second.login_error = valid ? "" : "incorrect_password";
        }
        login_session_ = 0;
    }
    if (owner_ && (now >= lease_until_ || !Online())) ClearInput("control lease or feedback expired");
    AdvanceVelocity(now);
    if (reference_until_ > 0 && now >= reference_until_) {
        command_.key = 0;
        reference_until_ = 0;
    }
    if (result_.phase == "accepted" && now >= request_until_) {
        CancelPendingPlannerStart();
        Finish("expired", "control did not confirm request");
    }
    if (command_.interaction.operation != robot_base::InteractionRequest::Operation::NONE &&
        (!SupportsInteraction(status_.mode) ||
            status_.interaction.sequence == command_.interaction.sequence)) {
        command_.interaction.operation = robot_base::InteractionRequest::Operation::NONE;
        command_.interaction.action.clear();
    }
    if (command_.planner.operation != robot_base::PlannerRequest::Operation::NONE &&
        (status_.mode != robot_base::ControlMode::RL ||
            status_.planner.sequence == command_.planner.sequence)) {
        command_.planner.operation = robot_base::PlannerRequest::Operation::NONE;
        command_.planner.planner.clear();
        command_.planner.argument.clear();
    }
}
void Service::UpdateStatus(const robot_base::ControlStatus &status, const robot_base::FaultStatus &fault) {
    // A planner starting or stopping changes who receives velocity commands and their range.
    using PlannerPhase = robot_base::PlannerStatus::Phase;
    const bool planner_runs = status.planner.phase == PlannerPhase::RUNNING;
    const bool changed = status.mode != status_.mode || status.active_policy != status_.active_policy ||
        planner_runs != (status_.planner.phase == PlannerPhase::RUNNING) ||
        (planner_runs && status.planner.planner != status_.planner.planner);
    if (status.mode != robot_base::ControlMode::RL || !Engaged(status.planner.phase) ||
        status.active_policy != status_.active_policy || status.planner.planner != status_.planner.planner)
        planner_preset_argument_.clear();
    if (status.planner.request_accepted && command_.planner.sequence == status.planner.sequence &&
        command_.planner.operation == robot_base::PlannerRequest::Operation::START &&
        status.planner.planner == command_.planner.planner) {
        const auto *planner = ActivePlanner(command_.planner.planner);
        if (planner && !planner->presets.empty()) planner_preset_argument_ = command_.planner.argument;
    }
    if (status.environment != status_.environment) ++catalog_revision_;
    status_ = status;
    auto &detail = status_.policy_switch.detail;
    waiting_for_input_ = detail.rfind(robot_base::kWaitingForPolicyInput, 0) == 0;
    if (waiting_for_input_) detail.erase(0, sizeof(robot_base::kWaitingForPolicyInput) - 1);
    fault_ = fault;
    status_at_ = Now();
    status_samples_ = std::min(2, status_samples_ + 1);
    if (changed) {
        ClearVelocity(true);
        if (teleop_preset_ != "slow") {
            teleop_preset_ = "slow";
            ++teleop_preset_sequence_;
        }
        reference_until_ = 0;
        if (result_.operation != "state") command_.key = 0;
    }
    if (result_.phase != "accepted") return;
    if (result_.operation == "state") {
        if (pending_target_ == StateName(status.mode) && (status.mode != source_mode_ || command_sent_))
            Finish("completed", "control entered requested state");
        else if (status.mode != source_mode_) Finish("rejected", "control entered a different state");
    } else if (result_.operation == "policy" &&
        status.policy_switch.sequence == command_.policy_request_sequence &&
        status.policy_switch.policy == pending_target_) {
        using Phase = robot_base::PolicySwitchStatus::Phase;
        if (status.policy_switch.phase == Phase::READY)
            Finish("completed", "control prepared requested policy");
        else if (status.policy_switch.phase == Phase::FAILED)
            Finish("failed", detail);
        else if (status.policy_switch.phase == Phase::REJECTED)
            Finish("rejected", detail);
        else if (status.policy_switch.phase == Phase::PREPARING &&
            !detail.empty())
            result_.message = detail;
    } else if ((result_.operation == "interaction" || result_.operation == "cancel") &&
        command_.interaction.sequence == status.interaction.sequence) {
        Finish(status.interaction.request_accepted ? "completed" : "rejected", "control handled action request");
        command_.interaction.operation = robot_base::InteractionRequest::Operation::NONE;
        command_.interaction.action.clear();
    } else if ((result_.operation == "planner" || result_.operation == "planner_stop") &&
        command_.planner.sequence == status.planner.sequence) {
        // Later failures of an accepted start are reported by the planner phase and detail.
        if (status.planner.request_accepted) Finish("completed", "control handled planner request");
        else Finish("rejected", status.planner.detail.empty() ? "control rejected planner request"
            : status.planner.detail);
        command_.planner.operation = robot_base::PlannerRequest::Operation::NONE;
        command_.planner.planner.clear();
        command_.planner.argument.clear();
    } else if (result_.operation == "ack" && !fault.latched) {
        Finish("completed", "control acknowledged fault");
    }
    if (result_.phase == "accepted" && fault.latched && result_.operation != "ack" && pending_target_ != "POWER_OFF" &&
        pending_target_ != "DAMP") {
        CancelPendingPlannerStart();
        Finish("rejected", fault.detail);
    }
}
robot_base::Command Service::Command() const { return command_; }
void Service::UpdateFeedback(const robot_base::RobotFeedback &sample) {
    if (sample.stream_id == 0) return;
    const double now = Now();
    const double unix_now = UnixTimeSeconds();
    const double transit_age = unix_now - sample.received_at_unix_s;
    const bool same_stream = sample.stream_id == feedback_.stream_id;
    if (same_stream) {
        const uint32_t advance = sample.packet_sequence - feedback_.packet_sequence;
        if (advance == 0 || advance >= (1U << 31)) return;
    } else {
        if (sample.stream_id / 1.0e6 > unix_now) return;
        // If the wall clock stepped back across a restart, wait for the old stream
        // to expire and require a current sample before accepting the lower ID.
        if (sample.stream_id < feedback_.stream_id &&
            (feedback_.stream_id / 1.0e6 <= unix_now || now - feedback_at_ <= config_.status_timeout ||
                !sample.available || !sample.fresh || transit_age < 0 ||
                std::max(sample.age_s, transit_age) > config_.status_timeout)) return;
    }
    const bool same_sample = same_stream && sample.available && feedback_.available &&
        sample.sequence == feedback_.sequence;
    const double previous_age = same_sample ? feedback_.age_s + std::max(0.0, now - feedback_at_) : 0.0;
    feedback_ = sample;
    if (sample.available)
        feedback_.age_s = std::max({sample.age_s, transit_age, previous_age});
    feedback_at_ = now;
}
RobotFeedback Service::FeedbackSnapshot() const {
    RobotFeedback result;
    result.frame_id = config_.feedback_frame_id;
    result.stale_after_ms = config_.status_timeout * 1000;
    const auto &sample = feedback_;
    result.stream_id = sample.stream_id;
    if (!sample.available || sample.num_dof <= 0 || sample.num_dof > static_cast<int>(sample.kMaxJoints) ||
        config_.joint_names.size() != static_cast<std::size_t>(sample.num_dof)) return result;
    result.available = true;
    result.sequence = sample.sequence;
    result.source_time_s = sample.source_time_s;
    result.received_at_unix_s = sample.received_at_unix_s;
    result.age_ms = (sample.age_s + std::max(0.0, Now() - feedback_at_)) * 1000;
    result.fresh = sample.fresh && Online() && result.age_ms <= result.stale_after_ms;
    result.joint_names = config_.joint_names;
    result.joint_positions.assign(sample.joint_pos.begin(), sample.joint_pos.begin() + sample.num_dof);
    result.joint_velocities.assign(sample.joint_vel.begin(), sample.joint_vel.begin() + sample.num_dof);
    if (fault_.active && fault_.code == robot_base::FaultCode::STATE_TIMEOUT) result.fresh = false;
    AccountForTransit(&result);
    if (sample.has_joint_effort)
        result.joint_efforts.assign(sample.joint_torque.begin(), sample.joint_torque.begin() + sample.num_dof);
    result.has_base_imu = sample.has_base_imu;
    result.has_acceleration = sample.has_acceleration;
    result.orientation = sample.orientation;
    result.angular_velocity = sample.angular_velocity;
    result.acceleration = sample.acceleration;
    return result;
}
bool Service::FeedbackSubscribed(uint64_t session) const {
    const auto it = sessions_.find(session);
    return it != sessions_.end() && it->second.authenticated && it->second.feedback_subscribed;
}
bool Service::PreviewAuthorized(const std::string &ticket) const {
    return std::any_of(sessions_.begin(), sessions_.end(), [&](const auto &item) {
        return item.second.authenticated && item.second.admin && TokenEqual(ticket, item.second.preview_ticket);
    });
}
bool Service::PreviewActive() const {
    return !config_.terminal_only && !config_.preview_policy.empty() && Online() && !fault_.latched &&
        status_.environment != robot_base::RuntimeEnvironment::UNKNOWN && status_.mode == robot_base::ControlMode::RL &&
        status_.active_policy == config_.preview_policy;
}
Status Service::Snapshot(uint64_t session) const {
    Status s;
    s.online = Online();
    s.waiting_for_input = waiting_for_input_;
    s.hmi_connected = status_.hmi_connected;
    s.zero_ready = status_.zero_ready;
    s.trajectory_enabled = config_.trajectory_enabled;
    s.environment = robot_base::RuntimeEnvironmentName(status_.environment);
    s.state = StateName(status_.mode);
    s.policy = status_.active_policy;
    s.age_ms = status_at_ < 0 ? 0 : std::max(0.0, (Now() - status_at_) * 1000);
    s.rl_hz = status_.rl_frequency_hz;
    s.velocity = {status_.vx, status_.vy, status_.wz};
    s.target_velocity = target_velocity_;
    s.command_velocity = {command_.vx, command_.vy, command_.wz};
    s.fault = {fault_.active, fault_.latched, fault_.sequence, robot_base::FaultSourceName(fault_.source),
        robot_base::FaultCodeName(fault_.code), fault_.detail};
    const auto it = sessions_.find(owner_);
    if (it != sessions_.end()) s.owner = it->second.name + "#" + std::to_string(owner_);
    s.owns_control = owner_ != 0 && owner_ == session && Now() < lease_until_;
    s.lease_remaining_ms = owner_ ? std::max(0.0, (lease_until_ - Now()) * 1000) : 0;
    s.interaction_phase = PhaseName(status_.interaction.phase);
    s.interaction_action = status_.interaction.action;
    s.interaction_progress = status_.interaction.progress;
    s.interaction_sequence = status_.interaction.sequence;
    s.interaction_accepted = status_.interaction.request_accepted;
    s.teleop_preset = teleop_preset_;
    s.teleop_preset_sequence = teleop_preset_sequence_;
    s.planner.name = status_.planner.planner;
    s.planner.phase = PhaseName(status_.planner.phase);
    s.planner.progress = status_.planner.progress;
    s.planner.sequence = status_.planner.sequence;
    s.planner.accepted = status_.planner.request_accepted;
    s.planner.plan_count = status_.planner.plan_count;
    s.planner.plan_rate_hz = status_.planner.plan_rate_hz;
    s.planner.last_plan_ms = status_.planner.last_plan_ms;
    s.planner.reference_remaining_s = status_.planner.reference_remaining_s;
    s.planner.detail = status_.planner.detail;
    s.planner.preset_argument = planner_preset_argument_;
    s.request = result_;
    return s;
}
Json Service::StatusMessage(uint64_t session) const {
    auto message = Encode(Snapshot(session));
    const auto viewer = sessions_.find(session);
    message["admin"] = viewer != sessions_.end() && viewer->second.admin;
    message["admin_login_pending"] = session != 0 && login_session_ == session;
    message["admin_login_error"] = viewer != sessions_.end() ? viewer->second.login_error : "";
    return message;
}
Json Service::Catalog() const {
    Json policies = Json::array();
    for (const auto &p : config_.policies) {
        if (!PolicyEnabled(p.name)) continue;
        Policy visible = p;
        visible.actions.erase(std::remove_if(visible.actions.begin(), visible.actions.end(),
            [&](const Action &a) { return !ActionEnabled(p.name, a.key); }), visible.actions.end());
        visible.planners.erase(std::remove_if(visible.planners.begin(), visible.planners.end(),
            [&](const Planner &planner) { return !PlannerEnabled(planner.name); }), visible.planners.end());
        if (!PlannerEnabled(visible.auto_planner)) visible.auto_planner.clear();
        policies.push_back(Encode(visible));
    }
    Json actions = Json::array();
    for (const auto &a : config_.actions) {
        if (ActionEnabled("", a.key)) actions.push_back({{"key", a.key}, {"display_name", a.display_name}});
    }
    Json teleop = {{"deadzone", config_.teleop.deadzone},
        {"acceleration", Encode(config_.teleop.acceleration)},
        {"deceleration", Encode(config_.teleop.deceleration)}};
    if (config_.teleop.slow_limits) teleop["slow_limits"] = Encode(*config_.teleop.slow_limits);
    if (config_.teleop.fast_limits) teleop["fast_limits"] = Encode(*config_.teleop.fast_limits);
    return {{"robot", config_.robot}, {"policies", policies}, {"lease_ms", config_.lease_ms},
        {"velocity_step", Encode(config_.velocity_step)}, {"trajectory_enabled", config_.trajectory_enabled},
        {"actions", actions}, {"teleop", teleop}, {"voice_chat_port", config_.voice_chat_port},
        {"admin_required", !config_.admin_password_hash.empty()},
        {"preview_policy", config_.preview_policy},
        {"environment", robot_base::RuntimeEnvironmentName(status_.environment)}};
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
            if (it->second.authenticated) return fail("bad_request", "already authenticated");
            const auto token = args.value("token", "");
            const bool control = TokenEqual(token, config_.control_token);
            if (!control && !TokenEqual(token, config_.token)) return fail("unauthorized", "invalid token");
            const auto name = args.value("name", "client");
            if (name.empty() || name.size() > 48 ||
                std::any_of(name.begin(), name.end(), [](unsigned char c) { return c < 32 || c == 127; }))
                return fail("bad_request", "invalid client name");
            it->second.authenticated = true;
            it->second.admin = control || config_.admin_password_hash.empty();
            it->second.name = name;
            Json data = {{"catalog", Catalog()}, {"status", StatusMessage(session)}};
            if (!it->second.preview_ticket.empty()) data["preview_ticket"] = it->second.preview_ticket;
            return ReplyTo(id, true, "ok", "authenticated", data);
        }
        if (!it->second.authenticated) return fail("unauthorized", "authenticate first");
        if (op == "subscribe_robot_feedback") {
            const bool enabled = args.value("enabled", true);
            if (enabled) {
                auto names = config_.joint_names;
                std::sort(names.begin(), names.end());
                if (names.empty() || names.front().empty() ||
                    names.size() > robot_base::RobotFeedback::kMaxJoints ||
                    std::adjacent_find(names.begin(), names.end()) != names.end())
                    return fail("unavailable", "robot_base.joint_names must contain unique joint names");
            }
            it->second.feedback_subscribed = enabled;
            return ReplyTo(id, true, "ok", "", {{"enabled", enabled}});
        }
        if (op == "client_event") {
            const auto event = args.value("event", "");
            auto detail = args.value("detail", "");
            const std::vector<std::string> allowed = {"direction_down", "manual_disconnect",
                "page_hidden", "pagehide", "protocol_error", "ui_error", "script_error",
                "promise_error", "velocity_rejected", "teleop_preset"};
            if (std::find(allowed.begin(), allowed.end(), event) == allowed.end() || detail.size() > 1024)
                return fail("bad_request", "invalid client event");
            if (event == "teleop_preset") {
                if (detail != "slow" && detail != "fast") return fail("bad_request", "unknown speed preset");
                if (owner_ != session || !Online()) return fail("not_owner", "acquire control first");
                SelectPreset(session, detail);
            }
            for (char &c : detail)
                if (static_cast<unsigned char>(c) < 32 || c == 127) c = ' ';
            runtime_logging::Log(runtime_logging::Level::kInfo,
                "operator client event: session=" + std::to_string(session) + " event=" + event +
                " detail=" + detail);
            return ReplyTo(id, true, "ok", "client event recorded");
        }
        if (op == "status") return ReplyTo(id, true, "ok", "", {{"status", StatusMessage(session)}});
        if (op == "catalog") return ReplyTo(id, true, "ok", "", {{"catalog", Catalog()}});
        if (op == "pairing") {
            if (config_.terminal_only) return fail("unsupported", "terminal-only mode has no pairing entry");
            const std::string url = PairingUrl(config_);
            return ReplyTo(id, true, "ok", "", {{"url", url}, {"qr_svg", PairingSvg(url)}});
        }
        if (op == "admin_login") {
            if (it->second.admin) return ReplyTo(id, true, "ok", "already authorized");
            if (login_result_.valid()) return fail("busy", "password verification is in progress");
            const auto password = args.value("password", "");
            if (password.empty() || password.size() > 128) return fail("bad_request", "invalid password length");
            const auto now = Now();
            if (now >= login_window_until_) {
                login_attempts_ = 0;
                login_window_until_ = now + 30;
            }
            if (login_attempts_ >= 5) return fail("rate_limited", "wait before retrying administrator login");
            ++login_attempts_;
            it->second.login_error.clear();
            login_result_ = std::async(std::launch::async, CheckPassword, config_.admin_password_hash, password);
            login_session_ = session;
            return ReplyTo(id, true, "pending", "verifying administrator password");
        }
        if (op == "admin_logout") {
            if (owner_ == session) ClearInput("administrator signed out");
            if (login_session_ == session) login_session_ = 0;
            it->second.admin = config_.admin_password_hash.empty();
            it->second.login_error.clear();
            return ReplyTo(id, true, "ok", "administrator signed out");
        }
        if (!it->second.admin) return fail("admin_required", "administrator login required");
        if (op == "stop") {
            ClearInput("stopped by authenticated client", true);
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
        const auto target = args.value("state", "");
        const bool safety_request = op == "state" && (target == "DAMP" || target == "POWER_OFF");
        if (!Online() && !(op == "state" && target == "POWER_OFF"))
            return fail("not_ready", "control feedback is unavailable");
        if (owner_ != session && !safety_request) return fail("not_owner", "acquire control first");
        if (safety_request) {
            const bool was_owner = owner_ == session;
            ClearInput("safety state requested", true);
            if (was_owner) {
                owner_ = session;
                lease_until_ = Now() + config_.lease_ms / 1000.0;
            }
        }
        if (op == "preset") {
            const auto preset = args.value("name", "");
            if (!SelectPreset(session, preset)) return fail("bad_request", "unknown speed preset");
            return ReplyTo(id, true, "ok", "speed preset selected");
        }
        if (op == "velocity" || op == "teleop") {
            if (config_.trajectory_enabled || status_.mode != robot_base::ControlMode::RL || fault_.latched || fault_.active)
                return fail("not_ready", "velocity is only available in healthy RL");
            const auto *policy = CommandPolicy();
            if (!policy) return fail("not_ready", "active policy has no catalog");
            const double vx = args.at("vx"), vy = args.at("vy"), wz = args.at("wz");
            const int ttl = args.value("valid_for_ms", 300);
            if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(wz) || ttl < 50 || ttl > 1000)
                return fail("bad_request", "invalid velocity or validity period");
            if (result_.phase == "accepted" && result_.operation == "policy")
                return ReplyTo(id, true, "ok", "stopping for policy switch");
            normalized_input_ = op == "teleop";
            velocity_input_ = normalized_input_
                ? Velocity{std::clamp(vx, -1.0, 1.0), std::clamp(vy, -1.0, 1.0), std::clamp(wz, -1.0, 1.0)}
                : Velocity{std::clamp(vx, policy->minimum.vx, policy->maximum.vx),
                    std::clamp(vy, policy->minimum.vy, policy->maximum.vy),
                    std::clamp(wz, policy->minimum.wz, policy->maximum.wz)};
            target_velocity_ = normalized_input_ ? TeleopTarget(*policy) : velocity_input_;
            if (!policy->client_smoothing) {
                command_.vx = target_velocity_.vx;
                command_.vy = target_velocity_.vy;
                command_.wz = target_velocity_.wz;
            }
            velocity_until_ = Now() + ttl / 1000.0;
            return ReplyTo(id, true, "ok", "velocity accepted",
                {{"target_velocity", Encode(target_velocity_)},
                    {"velocity", Encode(Velocity{command_.vx, command_.vy, command_.wz})}});
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
            if (target == StateName(mode) && target != "POWER_OFF")
                return ReplyTo(id, true, "ok", "already in requested state");
            if (!key) return fail("not_ready", "state prerequisite is not satisfied");
            if (key == 3 && !config_.trajectory_enabled && !PolicyEnabled(status_.active_policy))
                return fail("unavailable", EnvironmentMessage("policy " + status_.active_policy));
            command_.key = key;
            source_mode_ = mode;
            pending_target_ = target;
        } else if (op == "policy") {
            if (config_.trajectory_enabled) return fail("not_ready", "trajectory mode has no RL policy");
            const auto name = args.at("policy").get<std::string>();
            const auto selected = std::find_if(config_.policies.begin(), config_.policies.end(),
                [&](const Policy &p) { return p.name == name; });
            if (selected == config_.policies.end())
                return fail("bad_request", "unknown policy");
            if (!PolicyEnabled(name)) return fail("unavailable", EnvironmentMessage("policy " + name));
            const auto *active = ActivePolicy();
            const bool hot = status_.mode == M::RL && active && !active->switch_group.empty() &&
                active->switch_group == selected->switch_group;
            if (status_.mode != M::POWER_OFF && status_.mode != M::DAMP && !hot)
                return fail("not_ready", "select policy in POWER_OFF/DAMP or a compatible running group");
            ClearVelocity(false);
            command_.switch_policy = name;
            policy_sequence_ = std::max(policy_sequence_, status_.policy_switch.sequence) + 1;
            if (policy_sequence_ == 0) ++policy_sequence_;
            command_.policy_request_sequence = policy_sequence_;
            pending_target_ = name;
        } else if (op == "interaction" || op == "cancel") {
            const auto active_mode = config_.trajectory_enabled ? M::TRAJECTORY : M::RL;
            if (status_.mode != active_mode) return fail("not_ready", "action requires the configured active mode");
            const auto *policy = ActivePolicy();
            const auto *actions = config_.trajectory_enabled ? &config_.actions : (policy ? &policy->actions : nullptr);
            const auto action = args.value("action", "");
            if (op == "interaction" &&
                !ActionEnabled(config_.trajectory_enabled || !policy ? "" : policy->name, action))
                return fail("unavailable", EnvironmentMessage("action " + action));
            if (Engaged(status_.planner.phase))
                return fail("not_ready", "motion planner " + status_.planner.planner +
                    " holds the reference; stop it first");
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
        } else if (op == "planner" || op == "planner_stop") {
            using Operation = robot_base::PlannerRequest::Operation;
            if (config_.trajectory_enabled || status_.mode != M::RL)
                return fail("not_ready", "motion planners run inside the RL state");
            const bool start = op == "planner";
            const auto name = start ? args.at("name").get<std::string>() : std::string();
            auto argument = start ? args.value("argument", "") : std::string();
            if (start) {
                const auto *planner = ActivePlanner(name);
                if (!planner) return fail("bad_request", "unknown planner for the active policy");
                if (!PlannerEnabled(name)) return fail("unavailable", EnvironmentMessage("planner " + name));
                if (!planner->presets.empty()) {
                    if (argument.empty()) argument = planner->presets.front().DefaultArgument();
                    if (std::none_of(planner->presets.begin(), planner->presets.end(),
                            [&](const PlannerPreset &p) { return p.Accepts(argument); }))
                        return fail("bad_request", "unknown planner preset or height");
                }
                if (argument.size() > robot_base::kPlannerArgumentMaxLength ||
                    std::any_of(argument.begin(), argument.end(),
                        [](unsigned char c) { return c < 32 || c == 127; }))
                    return fail("bad_request", "planner argument is too long or has control characters");
                if (Busy(status_.interaction.phase))
                    return fail("not_ready", "an action is in progress; cancel it or wait for it to finish");
                if (Engaged(status_.planner.phase) &&
                    (status_.planner.planner != name || planner->category != "realtime"))
                    return fail("not_ready", "motion planner " + status_.planner.planner +
                        " is active; stop it first");
            } else if (!Engaged(status_.planner.phase) &&
                status_.planner.phase != robot_base::PlannerStatus::Phase::FAILED) {
                return ReplyTo(id, true, "ok", "no motion planner is active");
            }
            if (!start || status_.planner.phase != robot_base::PlannerStatus::Phase::RUNNING) ClearVelocity(false);
            command_.planner.sequence = std::max(command_.planner.sequence, status_.planner.sequence) + 1;
            if (command_.planner.sequence == 0) ++command_.planner.sequence;
            command_.planner.operation = start ? Operation::START : Operation::STOP;
            command_.planner.planner = name;
            command_.planner.argument = argument;
            pending_target_ = name;
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
        command_sent_ = false;
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
