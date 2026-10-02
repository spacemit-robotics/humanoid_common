/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file protocol.cpp
 * @brief Operator message serialization
 */
#include "protocol.h"
namespace operator_service {
namespace {
Velocity ReadVelocity(const Json &v) { return {v.value("vx", 0.0), v.value("vy", 0.0), v.value("wz", 0.0)}; }
RequestResult ReadRequest(const Json &v) {
    return {v.value("sequence", uint64_t{0}), v.value("operation", ""), v.value("phase", ""), v.value("message", "")};
}
} // namespace
Json Encode(const Velocity &v) { return {{"vx", v.vx}, {"vy", v.vy}, {"wz", v.wz}}; }
Json Encode(const Policy &p) {
    Json actions = Json::array();
    for (const auto &a : p.actions)
        actions.push_back({{"key", a.key}, {"display_name", a.display_name}});
    return {{"name", p.name}, {"manual_reference", p.manual_reference}, {"minimum", Encode(p.minimum)},
        {"maximum", Encode(p.maximum)}, {"actions", actions}};
}
Json Encode(const RequestResult &r) {
    return {{"sequence", r.sequence}, {"operation", r.operation}, {"phase", r.phase}, {"message", r.message}};
}
Json Encode(const Status &s) {
    const auto &f = s.fault;
    return {{"online", s.online}, {"hmi_connected", s.hmi_connected}, {"zero_ready", s.zero_ready}, {"state", s.state},
        {"trajectory_enabled", s.trajectory_enabled},
        {"policy", s.policy}, {"age_ms", s.age_ms}, {"rl_hz", s.rl_hz}, {"velocity", Encode(s.velocity)},
        {"owner", s.owner}, {"owns_control", s.owns_control}, {"lease_remaining_ms", s.lease_remaining_ms},
        {"interaction",
            {{"phase", s.interaction_phase}, {"action", s.interaction_action}, {"progress", s.interaction_progress}}},
        {"request", Encode(s.request)},
        {"fault",
            {{"active", f.active}, {"latched", f.latched}, {"sequence", f.sequence}, {"source", f.source},
                {"code", f.code}, {"detail", f.detail}}}};
}
Status DecodeStatus(const Json &v) {
    Status s;
    s.online = v.at("online");
    s.hmi_connected = v.at("hmi_connected");
    s.zero_ready = v.at("zero_ready");
    s.trajectory_enabled = v.value("trajectory_enabled", false);
    s.state = v.at("state");
    s.policy = v.at("policy");
    s.age_ms = v.at("age_ms");
    s.rl_hz = v.at("rl_hz");
    s.velocity = ReadVelocity(v.at("velocity"));
    s.owner = v.at("owner");
    s.owns_control = v.at("owns_control");
    s.lease_remaining_ms = v.at("lease_remaining_ms");
    const auto &f = v.at("fault");
    s.fault = {f.at("active"), f.at("latched"), f.at("sequence"), f.at("source"), f.at("code"), f.at("detail")};
    const auto &i = v.at("interaction");
    s.interaction_phase = i.at("phase");
    s.interaction_action = i.at("action");
    s.interaction_progress = i.at("progress");
    s.request = ReadRequest(v.at("request"));
    return s;
}
Policy DecodePolicy(const Json &v) {
    Policy p;
    p.name = v.at("name");
    p.manual_reference = v.at("manual_reference");
    p.minimum = ReadVelocity(v.at("minimum"));
    p.maximum = ReadVelocity(v.at("maximum"));
    for (const auto &a : v.at("actions"))
        p.actions.push_back({a.at("key"), a.at("display_name")});
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
