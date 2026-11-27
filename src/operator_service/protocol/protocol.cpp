/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file protocol.cpp
 * @brief Operator message serialization
 */
#include "protocol.h"

#include <algorithm>
#include <chrono>
#include <utility>
namespace operator_service {
namespace {
Velocity ReadVelocity(const Json &v) { return {v.value("vx", 0.0), v.value("vy", 0.0), v.value("wz", 0.0)}; }
RequestResult ReadRequest(const Json &v) {
    return {v.value("sequence", uint64_t{0}), v.value("operation", ""), v.value("phase", ""), v.value("message", "")};
}
} // namespace
Json Encode(const Velocity &v) { return {{"vx", v.vx}, {"vy", v.vy}, {"wz", v.wz}}; }
// Planner entries carry the same velocity fields as policies, so clients
// configure their motion controls from either.
Json Encode(const Planner &p) {
    Json result = {{"name", p.name}, {"display_name", p.display_name.empty() ? p.name : p.display_name},
        {"category", p.category}, {"type", p.type}, {"minimum", Encode(p.minimum)},
        {"maximum", Encode(p.maximum)}, {"service_smoothing", p.client_smoothing}};
    for (const auto &preset : p.presets)
        result["presets"].push_back({{"key", preset.key}, {"display_name", preset.display_name},
            {"heights_cm", preset.heights_cm}});
    if (p.slow_limits) result["teleop"]["slow_limits"] = Encode(*p.slow_limits);
    if (p.fast_limits) result["teleop"]["fast_limits"] = Encode(*p.fast_limits);
    return result;
}
Json Encode(const Policy &p) {
    Json actions = Json::array();
    for (const auto &a : p.actions)
        actions.push_back({{"key", a.key}, {"display_name", a.display_name}});
    Json planners = Json::array();
    for (const auto &planner : p.planners) planners.push_back(Encode(planner));
    Json result = {{"name", p.name}, {"manual_reference", p.manual_reference}, {"minimum", Encode(p.minimum)},
        {"maximum", Encode(p.maximum)}, {"actions", actions}, {"client_smoothing", false},
        {"service_smoothing", p.client_smoothing},
        {"switch_group", p.switch_group}, {"display_name", p.display_name.empty() ? p.name : p.display_name},
        {"planners", planners}, {"auto_planner", p.auto_planner}};
    if (p.slow_limits) result["teleop"]["slow_limits"] = Encode(*p.slow_limits);
    if (p.fast_limits) result["teleop"]["fast_limits"] = Encode(*p.fast_limits);
    return result;
}
Json Encode(const RequestResult &r) {
    return {{"sequence", r.sequence}, {"operation", r.operation}, {"phase", r.phase}, {"message", r.message}};
}
Json Encode(const Status &s) {
    const auto &f = s.fault;
    return {{"online", s.online}, {"hmi_connected", s.hmi_connected}, {"zero_ready", s.zero_ready}, {"state", s.state},
        {"trajectory_enabled", s.trajectory_enabled}, {"environment", s.environment},
        {"policy", s.policy}, {"age_ms", s.age_ms}, {"rl_hz", s.rl_hz}, {"velocity", Encode(s.velocity)},
        {"waiting_for_input", s.waiting_for_input},
        {"target_velocity", Encode(s.target_velocity)}, {"command_velocity", Encode(s.command_velocity)},
        {"owner", s.owner}, {"owns_control", s.owns_control}, {"lease_remaining_ms", s.lease_remaining_ms},
        {"interaction",
            {{"phase", s.interaction_phase}, {"action", s.interaction_action}, {"progress", s.interaction_progress},
                {"sequence", s.interaction_sequence}, {"accepted", s.interaction_accepted}}},
        {"teleop_preset", {{"name", s.teleop_preset}, {"sequence", s.teleop_preset_sequence}}},
        {"planner",
            {{"name", s.planner.name}, {"phase", s.planner.phase}, {"progress", s.planner.progress},
                {"sequence", s.planner.sequence}, {"accepted", s.planner.accepted}, {"preset_argument", s.planner.preset_argument},
                {"plan_count", s.planner.plan_count}, {"plan_rate_hz", s.planner.plan_rate_hz},
                {"last_plan_ms", s.planner.last_plan_ms},
                {"reference_remaining_s", s.planner.reference_remaining_s}, {"detail", s.planner.detail}}},
        {"request", Encode(s.request)},
        {"fault",
            {{"active", f.active}, {"latched", f.latched}, {"sequence", f.sequence}, {"source", f.source},
                {"code", f.code}, {"detail", f.detail}}}};
}
Json Encode(const RobotFeedback &s) {
    return {{"available", s.available}, {"fresh", s.fresh}, {"sequence", s.sequence},
        {"stream_id", s.stream_id},
        {"source_time_s", s.source_time_s}, {"received_at_unix_s", s.received_at_unix_s},
        {"age_ms", s.age_ms}, {"stale_after_ms", s.stale_after_ms}, {"frame_id", s.frame_id},
        {"joint_names", s.joint_names}, {"joint_positions", s.joint_positions},
        {"joint_velocities", s.joint_velocities}, {"joint_efforts", s.joint_efforts},
        {"orientation", s.has_base_imu ? Json(s.orientation) : Json(nullptr)},
        {"angular_velocity", s.has_base_imu ? Json(s.angular_velocity) : Json(nullptr)},
        {"acceleration", s.has_acceleration ? Json(s.acceleration) : Json(nullptr)}};
}
double UnixTimeSeconds() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
void AccountForTransit(RobotFeedback *sample) {
    const double elapsed = (UnixTimeSeconds() - sample->received_at_unix_s) * 1000.0;
    if (elapsed < 0 || sample->received_at_unix_s <= 0) sample->fresh = false;
    sample->age_ms = std::max(sample->age_ms, elapsed);
    if (sample->age_ms > sample->stale_after_ms) sample->fresh = false;
}
RobotFeedback DecodeRobotFeedback(const Json &v) {
    RobotFeedback s;
    s.available = v.at("available");
    s.fresh = v.at("fresh");
    s.stream_id = v.at("stream_id");
    s.sequence = v.at("sequence");
    s.source_time_s = v.at("source_time_s");
    s.received_at_unix_s = v.at("received_at_unix_s");
    s.age_ms = v.at("age_ms");
    s.stale_after_ms = v.at("stale_after_ms");
    s.frame_id = v.at("frame_id");
    s.joint_names = v.at("joint_names").get<std::vector<std::string>>();
    s.joint_positions = v.at("joint_positions").get<std::vector<double>>();
    s.joint_velocities = v.at("joint_velocities").get<std::vector<double>>();
    s.joint_efforts = v.at("joint_efforts").get<std::vector<double>>();
    s.has_base_imu = !v.at("orientation").is_null() && !v.at("angular_velocity").is_null();
    if (s.has_base_imu) {
        s.orientation = v.at("orientation").get<std::array<double, 4>>();
        s.angular_velocity = v.at("angular_velocity").get<std::array<double, 3>>();
    }
    s.has_acceleration = !v.at("acceleration").is_null();
    if (s.has_acceleration) s.acceleration = v.at("acceleration").get<std::array<double, 3>>();
    return s;
}
Status DecodeStatus(const Json &v) {
    Status s;
    s.online = v.at("online");
    s.hmi_connected = v.at("hmi_connected");
    s.zero_ready = v.at("zero_ready");
    s.trajectory_enabled = v.value("trajectory_enabled", false);
    s.environment = v.value("environment", "unknown");
    s.state = v.at("state");
    s.policy = v.at("policy");
    s.waiting_for_input = v.value("waiting_for_input", false);
    s.age_ms = v.at("age_ms");
    s.rl_hz = v.at("rl_hz");
    s.velocity = ReadVelocity(v.at("velocity"));
    s.target_velocity = ReadVelocity(v.value("target_velocity", v.at("velocity")));
    s.command_velocity = ReadVelocity(v.value("command_velocity", v.at("velocity")));
    s.owner = v.at("owner");
    s.owns_control = v.at("owns_control");
    s.lease_remaining_ms = v.at("lease_remaining_ms");
    const auto &f = v.at("fault");
    s.fault = {f.at("active"), f.at("latched"), f.at("sequence"), f.at("source"), f.at("code"), f.at("detail")};
    const auto &i = v.at("interaction");
    s.interaction_phase = i.at("phase");
    s.interaction_action = i.at("action");
    s.interaction_progress = i.at("progress");
    s.interaction_sequence = i.value("sequence", uint64_t{0});
    s.interaction_accepted = i.value("accepted", false);
    const auto preset = v.value("teleop_preset", Json::object());
    s.teleop_preset = preset.value("name", "");
    s.teleop_preset_sequence = preset.value("sequence", uint64_t{0});
    const auto planner = v.value("planner", Json::object());
    s.planner.name = planner.value("name", "");
    s.planner.preset_argument = planner.value("preset_argument", "");
    s.planner.phase = planner.value("phase", "IDLE");
    s.planner.progress = planner.value("progress", 0.0);
    s.planner.sequence = planner.value("sequence", uint64_t{0});
    s.planner.accepted = planner.value("accepted", false);
    s.planner.plan_count = planner.value("plan_count", uint32_t{0});
    s.planner.plan_rate_hz = planner.value("plan_rate_hz", 0.0);
    s.planner.last_plan_ms = planner.value("last_plan_ms", 0.0);
    s.planner.reference_remaining_s = planner.value("reference_remaining_s", 0.0);
    s.planner.detail = planner.value("detail", "");
    s.request = ReadRequest(v.at("request"));
    return s;
}
Policy DecodePolicy(const Json &v) {
    Policy p;
    p.name = v.at("name");
    p.display_name = v.value("display_name", p.name);
    p.manual_reference = v.at("manual_reference");
    p.client_smoothing = v.value("service_smoothing", v.value("client_smoothing", true));
    p.switch_group = v.value("switch_group", "");
    p.minimum = ReadVelocity(v.at("minimum"));
    p.maximum = ReadVelocity(v.at("maximum"));
    const auto teleop = v.value("teleop", Json::object());
    if (teleop.contains("slow_limits")) p.slow_limits = ReadVelocity(teleop.at("slow_limits"));
    if (teleop.contains("fast_limits")) p.fast_limits = ReadVelocity(teleop.at("fast_limits"));
    for (const auto &a : v.at("actions"))
        p.actions.push_back({a.at("key"), a.at("display_name")});
    for (const auto &entry : v.value("planners", Json::array())) {
        Planner planner;
        planner.name = entry.at("name");
        planner.display_name = entry.value("display_name", planner.name);
        planner.category = entry.value("category", "realtime");
        planner.type = entry.value("type", "");
        planner.minimum = ReadVelocity(entry.value("minimum", Json::object()));
        planner.maximum = ReadVelocity(entry.value("maximum", Json::object()));
        planner.client_smoothing = entry.value("service_smoothing", true);
        const auto limits = entry.value("teleop", Json::object());
        if (limits.contains("slow_limits")) planner.slow_limits = ReadVelocity(limits.at("slow_limits"));
        if (limits.contains("fast_limits")) planner.fast_limits = ReadVelocity(limits.at("fast_limits"));
        for (const auto &preset : entry.value("presets", Json::array()))
            planner.presets.push_back({preset.at("key"), preset.at("display_name"),
                preset.value("heights_cm", std::vector<int>{})});
        p.planners.push_back(std::move(planner));
    }
    p.auto_planner = v.value("auto_planner", "");
    return p;
}
Reply DecodeReply(const Json &v) {
    Reply r;
    r.ok = v.at("ok");
    r.code = v.value("code", "");
    r.message = v.value("message", "");
    if (v.contains("data") && v.at("data").contains("request"))
        r.request = ReadRequest(v.at("data").at("request"));
    return r;
}
} // namespace operator_service
