/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_operator_service.cpp
 * @brief Operator authentication, lease, freshness and request regression tests
 */
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem> // NOLINT(build/c++17): isolated configuration fixtures.
#include <fstream>
#include <stdexcept>
#include <thread>

#include "service.h"

namespace {
void TestRobotFeedback() {
    using namespace operator_service;  // NOLINT(build/namespaces)
    Config config;
    config.token = "feedback-test";
    config.joint_names = {"joint_a", "joint_b"};
    config.feedback_frame_id = "base_link";
    Service service(config);
    const auto observer = service.Open();
    uint64_t id = 0;
    const auto call = [&](const std::string &op, const Json &args = Json::object()) {
        return service.Handle(observer, {{"v", 1}, {"id", ++id}, {"op", op}, {"args", args}});
    };
    assert(!call("subscribe_robot_feedback").at("ok"));
    assert(call("hello", {{"token", config.token}}).at("ok"));
    assert(call("subscribe_robot_feedback").at("ok") && service.FeedbackSubscribed(observer));
    assert(!service.FeedbackSnapshot().available && service.Snapshot(observer).owner.empty());
    robot_base::ControlStatus status;
    status.hmi_connected = true;
    robot_base::RobotFeedback sample;
    sample.available = true;
    sample.fresh = true;
    sample.stream_id = static_cast<uint64_t>(UnixTimeSeconds() * 1.0e6) - 1000000;
    sample.packet_sequence = 102;
    sample.sequence = 11;
    sample.num_dof = 2;
    sample.source_time_s = 25;
    sample.has_joint_effort = sample.has_base_imu = sample.has_acceleration = true;
    sample.received_at_unix_s = std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    sample.joint_pos[1] = 0.5;
    sample.joint_vel[1] = 0.1;
    sample.joint_torque[1] = 0.2;
    sample.acceleration[2] = 9.81;
    service.UpdateFeedback(sample);
    service.UpdateStatus(status, {});
    service.UpdateStatus(status, {});
    const auto feedback = DecodeRobotFeedback(Encode(service.FeedbackSnapshot()));
    assert(feedback.available && feedback.fresh && feedback.sequence == 11 &&
        feedback.stream_id == sample.stream_id);
    assert(feedback.joint_names == config.joint_names && feedback.joint_positions.size() == 2);
    assert(feedback.joint_positions[1] == 0.5 && feedback.joint_velocities[1] == 0.1);
    assert(feedback.joint_efforts[1] == 0.2 && feedback.acceleration[2] == 9.81);
    assert(feedback.orientation[0] == 1 && feedback.frame_id == "base_link");
    assert(feedback.source_time_s == 25 && feedback.received_at_unix_s == sample.received_at_unix_s);
    assert(service.Snapshot(observer).owner.empty() && service.Command().key == 0);
    assert(!Encode(service.Snapshot()).contains("joint_positions"));
    auto late = sample;
    late.packet_sequence = 101;
    late.sequence = 10;
    late.joint_pos[1] = 2.5;
    service.UpdateFeedback(late);
    service.UpdateFeedback({});  // Invalid transport frames do not erase the last good sample.
    assert(service.FeedbackSnapshot().sequence == 11 && service.FeedbackSnapshot().joint_positions[1] == 0.5);
    sample.stream_id += 1000;
    sample.packet_sequence = 1;
    sample.sequence = 1;
    sample.source_time_s = 0.01;
    service.UpdateFeedback(sample);
    service.UpdateFeedback(late);
    assert(service.FeedbackSnapshot().stream_id == sample.stream_id && service.FeedbackSnapshot().sequence == 1);
    assert(service.FeedbackSnapshot().fresh);
    auto fault_sample = sample;
    fault_sample.packet_sequence++;
    fault_sample.fresh = false;
    service.UpdateFeedback(fault_sample);
    service.UpdateFeedback(sample);  // An old healthy publication cannot undo a new fault.
    assert(!service.FeedbackSnapshot().fresh && service.Snapshot(observer).online);
    sample.packet_sequence = fault_sample.packet_sequence + 1;
    sample.sequence++;
    sample.has_joint_effort = sample.has_base_imu = sample.has_acceleration = false;
    sample.received_at_unix_s -= config.status_timeout + 1;
    service.UpdateFeedback(sample);
    const auto delayed = service.FeedbackSnapshot();
    assert(!delayed.fresh && delayed.joint_efforts.empty() && !delayed.has_base_imu && !delayed.has_acceleration);
    assert(Encode(delayed).at("orientation").is_null() && Encode(delayed).at("acceleration").is_null());
    sample.received_at_unix_s = feedback.received_at_unix_s;
    sample.age_s = config.status_timeout + 1;
    ++sample.packet_sequence;
    service.UpdateFeedback(sample);
    assert(service.FeedbackSnapshot().available && !service.FeedbackSnapshot().fresh);
    sample.age_s = 0;
    ++sample.packet_sequence;
    service.UpdateFeedback(sample);
    assert(!service.FeedbackSnapshot().fresh);
    sample.num_dof = 3;
    ++sample.packet_sequence;
    service.UpdateFeedback(sample);
    assert(!service.FeedbackSnapshot().available);
    Service wrap(config);
    sample.num_dof = 2;
    sample.packet_sequence = UINT32_MAX;
    wrap.UpdateFeedback(sample);
    sample.packet_sequence = 0;
    ++sample.sequence;
    wrap.UpdateFeedback(sample);
    assert(wrap.FeedbackSnapshot().sequence == sample.sequence);
    assert(call("subscribe_robot_feedback", {{"enabled", false}}).at("ok"));
    assert(!service.FeedbackSubscribed(observer));
    service.Disconnect(observer);
    assert(!service.FeedbackSubscribed(observer));
    for (const auto &names : {std::vector<std::string>{}, std::vector<std::string>{"a", "a"}}) {
        config.joint_names = names;
        Service invalid(config);
        const auto session = invalid.Open();
        assert(invalid.Handle(session, {{"v", 1}, {"id", 1U}, {"op", "hello"},
            {"args", {{"token", config.token}}}}).at("ok"));
        assert(invalid.Handle(session, {{"v", 1}, {"id", 2U}, {"op", "subscribe_robot_feedback"},
            {"args", Json::object()}}).at("code") == "unavailable");
    }
}

void TestTeleop() {
    using namespace operator_service;  // NOLINT(build/namespaces)
    Config config;
    config.token = "teleop-test";
    config.status_timeout = 5;
    config.lease_ms = 5000;
    config.teleop.slow_limits = Velocity{0.6, 0.5, 1};
    config.teleop.fast_limits = Velocity{1.5, 0.5, 1};
    config.policies.push_back({"walk", false, {-0.3, -0.5, -1}, {1.5, 0.5, 1}, {}});
    Service service(config);
    robot_base::ControlStatus status;
    status.mode = robot_base::ControlMode::RL;
    status.active_policy = "walk";
    status.hmi_connected = true;
    service.UpdateStatus(status, {});
    service.UpdateStatus(status, {});
    auto session = service.Open();
    uint64_t id = 0;
    const auto call = [&](const std::string &op, const Json &args = Json::object()) {
        return service.Handle(session, {{"v", 1}, {"id", ++id}, {"op", op}, {"args", args}});
    };
    assert(call("hello", {{"token", config.token}}).at("ok"));
    assert(call("acquire").at("ok"));
    const auto input = [&](double vx, int ttl = 300) {
        return call("teleop", {{"vx", vx}, {"vy", 0}, {"wz", 0}, {"valid_for_ms", ttl}});
    };
    const auto tick = [&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        service.UpdateStatus(status, {});
        service.Tick();
    };
    assert(input(1).at("data").at("target_velocity").at("vx") == 0.6);
    tick();
    const auto first = service.Command().vx;
    assert(first > 0 && first <= 0.1);
    assert(call("preset", {{"name", "fast"}}).at("ok"));
    assert(input(1).at("data").at("target_velocity").at("vx") == 1.5);
    assert(input(-1).at("data").at("target_velocity").at("vx") == -0.3);
    tick();
    assert(service.Command().vx < first);  // Reversal brakes before accelerating.
    assert(input(1).at("ok"));
    for (int i = 0; i < 4; ++i) tick();
    const auto before_release = service.Command().vx;
    assert(before_release > 0);
    assert(call("release").at("ok"));
    assert(!service.Snapshot(session).owns_control);
    assert(service.Command().vx > 0 && service.Snapshot().target_velocity.vx == 0);
    for (int i = 0; i < 10; ++i) tick();
    assert(service.Command().vx == 0 && service.Snapshot().target_velocity.vx == 0);
    assert(call("acquire").at("ok"));
    assert(service.Command().vx == 0);  // New ownership cannot replay expired input.
    assert(input(1, 50).at("ok"));
    tick();
    assert(service.Command().vx > 0);
    for (int i = 0; i < 8; ++i) tick();
    assert(service.Command().vx == 0 && service.Snapshot(session).owns_control);
    assert(input(1).at("ok"));
    tick();
    assert(call("stop").at("ok") && service.Command().vx == 0);
    assert(call("acquire").at("ok"));
    assert(input(1).at("ok"));
    tick();
    robot_base::FaultStatus fault;
    fault.latched = true;
    service.UpdateStatus(status, fault);
    service.Tick();
    assert(service.Command().vx == 0);
    assert(!input(1).at("ok"));
    service.UpdateStatus(status, {});
    service.Tick();
    assert(service.Command().vx == 0);
    assert(DecodeStatus(Encode(service.Snapshot())).command_velocity.vx == 0);

    config.policies[0].client_smoothing = false;
    Service delegated(config);
    delegated.UpdateStatus(status, {});
    delegated.UpdateStatus(status, {});
    const auto owner = delegated.Open();
    id = 0;
    const auto send = [&](const std::string &op, Json args) {
        return delegated.Handle(owner, {{"v", 1}, {"id", ++id}, {"op", op}, {"args", args}});
    };
    assert(send("hello", {{"token", config.token}}).at("ok"));
    assert(send("acquire", Json::object()).at("ok"));
    assert(send("teleop", {{"vx", 1}, {"vy", 0}, {"wz", 0}}).at("ok"));
    assert(std::abs(delegated.Command().vx - 0.6) < 1e-6);
    assert(send("teleop", {{"vx", 0}, {"vy", 0}, {"wz", 0}}).at("ok"));
    assert(delegated.Command().vx == 0);
}

void TestAdministrator() {
    using namespace operator_service;  // NOLINT(build/namespaces)
    Config config;
    config.token = "public-pairing-test";
    config.control_token = "local-control-test";
    config.admin_password_hash = "pbkdf2-sha256:100000:00112233445566778899aabbccddeeff:"
        "ce640bcfa4ac6c9479894ecb9990fe597e6f6e4f6878e84ebbcce384bbb4f7b2";
    Service service(config);
    const auto browser = service.Open(), terminal = service.Open();
    uint64_t request_id = 0;
    const auto call = [&](uint64_t session, const std::string &op, const Json &args = Json::object()) {
        return service.Handle(session, {{"v", 1}, {"id", ++request_id}, {"op", op}, {"args", args}});
    };
    robot_base::ControlStatus status;
    status.hmi_connected = true;
    service.UpdateStatus(status, {});
    service.UpdateStatus(status, {});
    assert(call(browser, "hello", {{"token", config.token}}).at("ok"));
    assert(call(terminal, "hello", {{"token", config.control_token}}).at("ok"));
    assert(!service.StatusMessage(browser).at("admin") && service.StatusMessage(terminal).at("admin"));
    assert(service.Catalog().at("admin_required"));
    assert(service.Catalog().dump().find(config.admin_password_hash) == std::string::npos);
    assert(call(browser, "status").at("ok"));
    for (const auto *op : {"acquire", "stop", "cancel", "ack", "velocity", "interaction", "planner",
            "planner_stop"})
        assert(call(browser, op).at("code") == "admin_required");
    assert(call(browser, "state", {{"state", "POWER_OFF"}}).at("code") == "admin_required");
    assert(call(terminal, "acquire").at("ok"));
    const auto await_login = [&] {
        for (int i = 0; i < 1000 && service.StatusMessage(browser).at("admin_login_pending"); ++i) {
            // Password work runs independently while normal service requests continue.
            service.UpdateStatus(status, {});
            assert(call(terminal, "renew").at("ok"));
            service.Tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        assert(!service.StatusMessage(browser).at("admin_login_pending"));
    };
    assert(call(browser, "admin_login", {{"password", "wrong"}}).at("code") == "pending");
    await_login();
    assert(!service.StatusMessage(browser).at("admin"));
    assert(service.StatusMessage(browser).at("admin_login_error") == "incorrect_password");
    assert(call(browser, "admin_login", {{"password", "test-admin-only"}}).at("ok"));
    await_login();
    assert(service.StatusMessage(browser).at("admin") && !service.Snapshot(browser).owns_control);
    assert(call(terminal, "release").at("ok"));
    assert(call(browser, "acquire").at("ok"));
    assert(call(browser, "admin_logout").at("ok"));
    assert(!service.StatusMessage(browser).at("admin") && !service.Snapshot(browser).owns_control);
    assert(service.Command().key == 0);
    assert(call(browser, "acquire").at("code") == "admin_required");
    assert(call(terminal, "acquire").at("ok"));
    for (int i = 0; i < 3; ++i) {
        assert(call(browser, "admin_login", {{"password", "wrong"}}).at("ok"));
        await_login();
    }
    assert(call(browser, "admin_login", {{"password", "test-admin-only"}}).at("code") == "rate_limited");
    assert(DecodeStatus(service.StatusMessage(browser)).state == "POWER_OFF");
    config.admin_password_hash = "invalid";
    bool rejected = false;
    try { Service invalid(config); } catch (const std::exception &) { rejected = true; }
    assert(rejected);
}

void TestTrajectory() {
    using namespace operator_service;  // NOLINT(build/namespaces)
    char directory[] = "/tmp/operator-trajectory-XXXXXX";
    assert(mkdtemp(directory));
    const std::filesystem::path root(directory);
    const auto path = root / "runtime.yaml";
    std::ofstream(path) << "robot_base: {name: test}\n"
        "behavior_manager:\n  trajectory:\n    enabled: true\n"
        "    catalog_file: linglong.yaml\n    catalog: upper_body_actions\n";
    std::ofstream(root / "linglong.yaml") << "upper_body_actions:\n"
        "  action_names: [wave, bow]\n  actions:\n    wave: {display_name: Wave}\n";
    auto config = LoadConfig(path.string());
    assert(config.trajectory_enabled && config.policies.empty());
    assert(config.actions.size() == 2 && config.actions[0].key == "wave");
    assert(config.actions[0].display_name == "Wave" && config.actions[1].display_name == "bow");
    config.token = "trajectory-test";
    Service service(config);
    const auto session = service.Open();
    uint64_t id = 0;
    const auto call = [&](const std::string &op, const Json &args = Json::object()) {
        return service.Handle(session, {{"v", 1}, {"id", ++id}, {"op", op}, {"args", args}}).at("ok").get<bool>();
    };
    assert(call("hello", {{"token", config.token}}));
    assert(service.Catalog().at("trajectory_enabled").get<bool>());
    assert(service.Catalog().at("policies").empty());
    assert(service.Catalog().at("actions").size() == 2);
    robot_base::ControlStatus status;
    status.hmi_connected = true;
    service.UpdateStatus(status, {});
    service.UpdateStatus(status, {});
    assert(call("acquire"));
    assert(!call("policy", {{"policy", "wave"}}));
    assert(!call("interaction", {{"action", "wave"}}));
    assert(!call("state", {{"state", "TRAJECTORY"}}));
    status.mode = robot_base::ControlMode::ZERO;
    service.UpdateStatus(status, {});
    assert(!call("state", {{"state", "TRAJECTORY"}}));
    status.zero_ready = true;
    service.UpdateStatus(status, {});
    assert(!call("state", {{"state", "RL"}}));
    assert(call("state", {{"state", "TRAJECTORY"}}));
    assert(service.Command().key == 3);
    service.Tick();
    assert(service.Command().key == 3);
    status.mode = robot_base::ControlMode::TRAJECTORY;
    service.UpdateStatus(status, {});
    assert(service.Command().key == 0 && service.Snapshot().request.phase == "completed");
    const auto decoded = DecodeStatus(Encode(service.Snapshot()));
    assert(decoded.state == "TRAJECTORY" && decoded.trajectory_enabled && decoded.policy.empty());
    assert(decoded.rl_hz == 0 && decoded.velocity.vx == 0);
    auto legacy = Encode(Status{});
    legacy.erase("trajectory_enabled");
    assert(!DecodeStatus(legacy).trajectory_enabled);
    assert(!call("velocity", {{"vx", 1}, {"vy", 0}, {"wz", 0}}));
    assert(!call("reference_start"));
    assert(!call("interaction", {{"action", "unknown"}}));
    assert(call("interaction", {{"action", "wave"}}));
    service.Tick();
    assert(service.Command().interaction.operation == robot_base::InteractionRequest::Operation::START);
    status.interaction = {service.Command().interaction.sequence, true,
        robot_base::InteractionStatus::Phase::PLAYING, 0.3F, "wave"};
    service.UpdateStatus(status, {});
    assert(service.Snapshot().request.phase == "completed");
    assert(!call("interaction", {{"action", "bow"}}));
    assert(call("cancel"));
    service.Tick();
    assert(service.Command().interaction.operation == robot_base::InteractionRequest::Operation::CANCEL);
    status.interaction.sequence = service.Command().interaction.sequence;
    status.interaction.phase = robot_base::InteractionStatus::Phase::IDLE;
    service.UpdateStatus(status, {});
    assert(service.Command().interaction.operation == robot_base::InteractionRequest::Operation::NONE);
    assert(call("interaction", {{"action", "bow"}}));
    service.Disconnect(session);
    service.Tick();
    assert(service.Snapshot().request.phase == "cancelled");
    assert(service.Command().interaction.operation == robot_base::InteractionRequest::Operation::CANCEL);

    std::filesystem::remove(root / "linglong.yaml");
    bool rejected = false;
    try { (void)LoadConfig(path.string()); } catch (const std::exception &) { rejected = true; }
    assert(rejected);
    std::ofstream(path) << "behavior_manager:\n  trajectory:\n    enabled: true\n"
        "    action_names: [wave]\n    actions:\n      wave: {display_name: Wave}\n";
    const auto inline_config = LoadConfig(path.string());
    assert(inline_config.trajectory_enabled && inline_config.actions.size() == 1);
    assert(inline_config.actions[0].key == "wave" && inline_config.actions[0].display_name == "Wave");
    std::ofstream(path) << "behavior_manager: {trajectory: {enabled: false}}\n";
    assert(!LoadConfig(path.string()).trajectory_enabled);
    assert(LoadConfig(path.string()).voice_chat_port == 0);
    std::ofstream(path) << "operator_service: {voice_chat_port: 8442}\n";
    const auto voice_config = LoadConfig(path.string());
    assert(voice_config.voice_chat_port == 8442);
    assert(Service(voice_config).Catalog().at("voice_chat_port").get<int>() == 8442);
    std::ofstream(path) << "operator_service: {voice_chat_port: 70000}\n";
    rejected = false;
    try { (void)LoadConfig(path.string()); } catch (const std::exception &) { rejected = true; }
    assert(rejected);
    std::ofstream(path) << "rl_policy:\n  onnx_infer:\n"
        "    policy_names: [motions]\n    policies:\n"
        "      motions:\n        display_name: Motion mode\n        switch_group: whole_body\n"
        "        policy_actions:\n          idle_policy: idle\n          action_names: [wave, bow]\n"
        "          actions:\n            wave: {policy: wave_model, display_name: Wave}\n"
        "            bow: {policy: bow_model, display_name: Bow}\n"
        "      idle: {}\n      wave_model: {}\n      bow_model: {}\n";
    const auto modes = LoadConfig(path.string());
    assert(modes.policies.size() == 1 && modes.policies.front().name == "motions");
    assert(modes.policies.front().display_name == "Motion mode");
    assert(!modes.policies.front().manual_reference && modes.policies.front().actions.size() == 2);
    assert(modes.policies.front().actions[0].key == "wave" && modes.policies.front().actions[0].display_name == "Wave");
    assert(Service(modes).Catalog().at("policies").size() == 1);
    std::filesystem::remove_all(root);
}
// The catalog and the request checks follow the environment reported by the driver.
void TestEnvironment() {
    using namespace operator_service;  // NOLINT(build/namespaces)
    using Environment = robot_base::RuntimeEnvironment;
    const auto root = std::filesystem::temp_directory_path() /
        ("operator_environment_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    const auto path = root / "config.yaml";
    std::ofstream(path) << "rl_policy:\n  onnx_infer:\n"
        "    policy_names: [walk, show]\n    policies:\n"
        "      walk:\n        policy_adapter: {type: sonic, catalog: walk_actions}\n"
        "      show:\n        enabled_in: [sim]\n        switch_group: whole_body\n"
        "        policy_actions:\n          idle_policy: walk\n          action_names: [wave]\n"
        "          actions:\n            wave: {policy: wave_model}\n"
        "      wave_model: {}\n"
        "walk_actions:\n  action_names: [stand, flip]\n  actions:\n"
        "    stand: {display_name: Stand}\n    flip: {display_name: Flip, enabled_in: [sim]}\n";
    auto config = LoadConfig(path.string());
    std::filesystem::remove_all(root);
    assert(config.policies.size() == 2 && config.policies[0].actions.size() == 2);
    config.token = "environment-test";
    config.status_timeout = 5;
    config.lease_ms = 5000;
    Service service(config);
    // Until the driver reports its backend only unrestricted entries are listed.
    auto catalog = service.Catalog();
    assert(catalog.at("environment") == "unknown" && catalog.at("policies").size() == 1);
    assert(catalog.at("policies")[0].at("name") == "walk" && catalog.at("policies")[0].at("actions").size() == 1);
    robot_base::ControlStatus status;
    status.active_policy = "walk";
    status.hmi_connected = true;
    status.environment = Environment::REAL;
    const auto revision = service.CatalogRevision();
    service.UpdateStatus(status, {});
    service.UpdateStatus(status, {});
    assert(service.CatalogRevision() == revision + 1);
    catalog = service.Catalog();
    assert(catalog.at("environment") == "real" && catalog.at("policies").size() == 1);
    assert(catalog.at("policies")[0].at("actions").size() == 1 &&
        catalog.at("policies")[0].at("actions")[0].at("key") == "stand");
    assert(service.Snapshot().environment == "real");
    const auto session = service.Open();
    uint64_t id = 0;
    const auto call = [&](const std::string &op, const Json &args = Json::object()) {
        return service.Handle(session, {{"v", 1}, {"id", ++id}, {"op", op}, {"args", args}});
    };
    assert(call("hello", {{"token", config.token}}).at("data").at("catalog").at("policies").size() == 1);
    assert(call("acquire").at("ok"));
    auto reply = call("policy", {{"policy", "show"}});
    assert(!reply.at("ok") && reply.at("code") == "unavailable");
    status.mode = robot_base::ControlMode::RL;
    service.UpdateStatus(status, {});
    reply = call("interaction", {{"action", "flip"}});
    assert(!reply.at("ok") && reply.at("code") == "unavailable");
    assert(service.Command().interaction.operation == robot_base::InteractionRequest::Operation::NONE);
    status.environment = Environment::SIMULATION;
    service.UpdateStatus(status, {});
    assert(service.CatalogRevision() == revision + 2);
    catalog = service.Catalog();
    assert(catalog.at("environment") == "sim" && catalog.at("policies").size() == 2);
    assert(catalog.at("policies")[0].at("actions").size() == 2);
    assert(DecodeStatus(service.StatusMessage(session)).environment == "sim");
    assert(call("interaction", {{"action", "flip"}}).at("ok"));
    assert(service.Command().interaction.action == "flip");
    // A selected policy that the environment disables cannot enter RL.
    status.mode = robot_base::ControlMode::ZERO;
    status.zero_ready = true;
    status.active_policy = "show";
    status.environment = Environment::REAL;
    service.UpdateStatus(status, {});
    call("stop");  // Cancels the pending action request and releases control.
    assert(call("acquire").at("ok"));
    reply = call("state", {{"state", "RL"}});
    assert(!reply.at("ok") && reply.at("code") == "unavailable");
}
void TestPlanner() {
    using namespace operator_service;  // NOLINT(build/namespaces)
    using Environment = robot_base::RuntimeEnvironment;
    using Phase = robot_base::PlannerStatus::Phase;
    using Operation = robot_base::PlannerRequest::Operation;
    const auto root = std::filesystem::temp_directory_path() /
        ("operator_planner_" + std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    std::filesystem::create_directories(root);
    const auto path = root / "config.yaml";
    const std::string yaml = "rl_policy:\n  onnx_infer:\n"
        "    policy_names: [track, plain]\n    policies:\n"
        "      track:\n"
        "        command:\n          limits: {max_vx: 0.2, max_vy: 0.2, max_wz: 0.2}\n"
        "        policy_adapter:\n          type: sonic\n          catalog: track_actions\n"
        "          planners: [walker, writer, lab]\n          auto_planner: lab\n"
        "      plain: {}\n"
        "track_actions:\n  action_names: [wave]\n  actions:\n    wave: {display_name: Wave}\n"
        "motion_planners:\n"
        "  walker:\n    display_name: 行走规划\n    type: walker_model\n"
        "    command:\n      limits: {min_vx: -0.3, max_vx: 0.8, max_vy: 0.5, max_wz: 1.0}\n"
        "      teleop: {slow_limits: [0.4, 0.2, 0.5], fast_limits: [0.8, 0.5, 1.0]}\n"
        "      smoothing: false\n"
        "  writer: {type: text_to_motion, category: offline}\n"
        "  lab: {type: lab_model, enabled_in: [sim]}\n";
    const auto load = [&](const std::string &text) {
        std::ofstream(path) << text;
        return LoadConfig(path.string());
    };
    const auto rejected = [&](std::string text, const std::string &from, const std::string &to) {
        const auto at = text.find(from);
        assert(at != std::string::npos);
        text.replace(at, from.size(), to);
        try {
            load(text);
        } catch (const std::exception &) {
            return true;
        }
        return false;
    };
    const auto near = [](double value, double expected) { return std::abs(value - expected) < 1e-6; };
    assert(rejected(yaml, "[walker, writer, lab]", "[walker, ghost]"));   // Not registered.
    assert(rejected(yaml, "[walker, writer, lab]", "[walker, walker]"));  // Listed twice.
    assert(rejected(yaml, "[walker, writer, lab]", "walker"));            // Not a list.
    assert(rejected(yaml, "category: offline", "category: sometimes"));
    assert(rejected(yaml, "slow_limits: [0.4, 0.2, 0.5]", "slow_limits: [0.9, 0.2, 0.5]"));
    assert(rejected(yaml, "min_vx: -0.3, max_vx: 0.8, max_vy: 0.5, max_wz: 1.0", "max_vx: 0.8"));
    assert(rejected(yaml, "type: walker_model\n", ""));
    assert(rejected(yaml, "[walker, writer, lab]", "{walker: true}"));
    assert(rejected(yaml, "[walker, writer, lab]", "[walker, [writer]]"));
    auto config = load(yaml);
    std::filesystem::remove_all(root);
    assert(config.policies.size() == 2 && config.policies[1].planners.empty());
    assert(config.policies[0].planners.size() == 3 && config.policies[0].auto_planner == "lab");
    {
        const auto &walker = config.policies[0].planners[0];
        assert(walker.name == "walker" && walker.display_name == "行走规划");
        assert(walker.category == "realtime" && walker.type == "walker_model" && !walker.client_smoothing);
        assert(near(walker.minimum.vx, -0.3) && near(walker.maximum.vx, 0.8));
        assert(near(walker.minimum.vy, -0.5) && near(walker.maximum.wz, 1.0));
        assert(walker.slow_limits->vx == 0.4 && walker.fast_limits->wz == 1.0);
        const auto &writer = config.policies[0].planners[1];
        assert(writer.name == "writer" && writer.display_name == "writer" && writer.category == "offline");
        assert(writer.client_smoothing && !writer.slow_limits && writer.maximum.vx == 0);
    }
    config.token = "planner-test";
    config.status_timeout = 5;
    config.lease_ms = 5000;
    Service service(config);
    const auto planners = [&] { return service.Catalog().at("policies")[0].at("planners"); };
    const auto auto_planner = [&] {
        return service.Catalog().at("policies")[0].at("auto_planner").get<std::string>();
    };
    // Until the driver reports its backend the simulation-only planner is hidden.
    assert(planners().size() == 2 && auto_planner().empty());
    assert(service.Catalog().at("policies")[1].at("planners").empty());
    robot_base::ControlStatus status;
    status.active_policy = "plain";
    status.hmi_connected = true;
    status.environment = Environment::SIMULATION;
    const auto update = [&] { service.UpdateStatus(status, {}); };
    update();
    update();
    assert(planners().size() == 3 && auto_planner() == "lab");
    {
        const auto decoded = DecodePolicy(service.Catalog().at("policies")[0]);
        assert(decoded.planners.size() == 3 && decoded.auto_planner == "lab");
        const auto &walker = decoded.planners[0];
        assert(walker.name == "walker" && walker.display_name == "行走规划");
        assert(walker.category == "realtime" && walker.type == "walker_model" && !walker.client_smoothing);
        assert(near(walker.minimum.vx, -0.3) && near(walker.maximum.vx, 0.8) && walker.HasSpeedModes());
        assert(walker.slow_limits->vx == 0.4 && walker.fast_limits->vx == 0.8);
        assert(decoded.planners[1].category == "offline" && !decoded.planners[1].HasSpeedModes());
        assert(decoded.planners[2].name == "lab");
        // Catalogs of older services carry no planner fields.
        auto legacy = service.Catalog().at("policies")[0];
        legacy.erase("planners");
        legacy.erase("auto_planner");
        assert(DecodePolicy(legacy).planners.empty() && DecodePolicy(legacy).auto_planner.empty());
        Json status_message = Encode(service.Snapshot());
        status_message.erase("planner");
        assert(DecodeStatus(status_message).planner.phase == "IDLE" && !DecodeStatus(status_message).planner.Engaged());
    }
    const auto session = service.Open();
    uint64_t id = 0;
    const auto call = [&](const std::string &op, const Json &args = Json::object()) {
        return service.Handle(session, {{"v", 1}, {"id", ++id}, {"op", op}, {"args", args}});
    };
    const auto refused = [&](const std::string &code, const std::string &op, const Json &args = Json::object()) {
        const auto reply = call(op, args);
        return !reply.at("ok").get<bool>() && reply.at("code") == code;
    };
    const auto snapshot = [&] { return DecodeStatus(Encode(service.Snapshot(session))); };
    const auto idle = [&] { return service.Command().planner.operation == Operation::NONE &&
        service.Command().planner.planner.empty() && service.Command().planner.argument.empty(); };
    assert(call("hello", {{"token", config.token}}).at("ok"));
    assert(call("acquire").at("ok"));
    // Planners run inside RL and belong to the active policy.
    assert(refused("not_ready", "planner", {{"name", "walker"}}));
    assert(refused("not_ready", "planner_stop"));
    status.mode = robot_base::ControlMode::RL;
    update();
    assert(refused("bad_request", "planner", {{"name", "walker"}}));
    status.active_policy = "track";
    update();
    assert(refused("bad_request", "planner"));
    assert(refused("bad_request", "planner", {{"name", "ghost"}}));
    assert(refused("bad_request", "planner",
        {{"name", "writer"}, {"argument", std::string(robot_base::kPlannerArgumentMaxLength + 1, 'x')}}));
    assert(refused("bad_request", "planner", {{"name", "writer"}, {"argument", "two\nlines"}}));
    status.environment = Environment::REAL;
    update();
    assert(planners().size() == 2 && auto_planner().empty());
    assert(refused("unavailable", "planner", {{"name", "lab"}}));
    status.environment = Environment::SIMULATION;
    status.interaction.phase = robot_base::InteractionStatus::Phase::PLAYING;
    update();
    assert(refused("not_ready", "planner", {{"name", "walker"}}));  // An action is playing.
    status.interaction.phase = robot_base::InteractionStatus::Phase::IDLE;
    update();
    assert(idle() && service.Command().planner.sequence == 0);
    for (const bool sent : {false, true}) {
        for (const bool expired : {false, true}) {
            auto pending_config = config;
            pending_config.request_timeout = 0;
            Service pending(pending_config);
            pending.UpdateStatus(status, {});
            pending.UpdateStatus(status, {});
            const auto peer = pending.Open();
            const auto invoke = [&](uint64_t id, const std::string &op, const Json &args) {
                return pending.Handle(peer, {{"v", 1}, {"id", id}, {"op", op}, {"args", args}}).at("ok");
            };
            assert(invoke(1, "hello", {{"token", config.token}}));
            assert(invoke(2, "acquire", Json::object()));
            assert(invoke(3, "planner", {{"name", "writer"}}));
            const auto start = pending.Command().planner.sequence;
            if (sent) pending.MarkCommandSent();
            if (expired) pending.Tick();
            else pending.Disconnect(peer);
            const auto cancelled = pending.Command().planner;
            assert(cancelled.operation == (sent ? Operation::STOP : Operation::NONE));
            assert(cancelled.planner.empty() && cancelled.argument.empty());
            assert(!sent || cancelled.sequence > start);
            assert(pending.Snapshot().request.phase == (expired ? "expired" : "cancelled"));
        }
    }
    // Stopping when nothing runs is answered without asking control.
    auto reply = call("planner_stop");
    assert(reply.at("ok") && reply.at("code") == "ok" && idle());

    // Velocity follows the policy until the planner reports RUNNING.
    const auto teleop = [&](double vx, double vy = 0, double wz = 0) {
        return call("teleop", {{"vx", vx}, {"vy", vy}, {"wz", wz}}).at("data").at("target_velocity");
    };
    assert(near(teleop(1).at("vx"), 0.08));
    reply = call("planner", {{"name", "walker"}});
    assert(reply.at("ok") && reply.at("code") == "accepted");
    assert(reply.at("data").at("request").at("operation") == "planner");
    const auto start_sequence = service.Command().planner.sequence;
    assert(start_sequence == 1 && service.Command().planner.operation == Operation::START);
    assert(service.Command().planner.planner == "walker" && service.Command().planner.argument.empty());
    assert(snapshot().request.phase == "accepted" && snapshot().request.operation == "planner");
    assert(refused("busy", "planner_stop"));
    status.planner.sequence = start_sequence;
    status.planner.request_accepted = true;
    status.planner.phase = Phase::LOADING;
    status.planner.planner = "walker";
    update();
    assert(snapshot().request.phase == "completed" && idle());
    assert(snapshot().planner.name == "walker" && snapshot().planner.phase == "LOADING");
    assert(snapshot().planner.Engaged() && snapshot().planner.accepted &&
        snapshot().planner.sequence == start_sequence);
    assert(near(teleop(1).at("vx"), 0.08));
    // The planner holds the reference: actions wait, another planner cannot start.
    assert(refused("not_ready", "interaction", {{"action", "wave"}}));
    assert(refused("not_ready", "cancel"));
    assert(refused("not_ready", "planner", {{"name", "writer"}}));
    assert(service.Command().interaction.operation == robot_base::InteractionRequest::Operation::NONE);
    status.planner.phase = Phase::WARMING;
    update();
    assert(snapshot().planner.phase == "WARMING" && snapshot().planner.Engaged());
    assert(call("preset", {{"name", "fast"}}).at("ok") && snapshot().teleop_preset == "fast");
    const auto preset_sequence = snapshot().teleop_preset_sequence;

    // RUNNING: velocity commands use the planner's range, presets and smoothing.
    status.planner.phase = Phase::RUNNING;
    status.planner.plan_count = 3;
    status.planner.plan_rate_hz = 9.5F;
    status.planner.last_plan_ms = 104.0F;
    status.planner.reference_remaining_s = 1.25F;
    update();
    assert(service.Command().vx == 0);
    assert(snapshot().teleop_preset == "slow" && snapshot().teleop_preset_sequence == preset_sequence + 1);
    assert(snapshot().planner.phase == "RUNNING" && snapshot().planner.plan_count == 3);
    assert(snapshot().planner.plan_rate_hz == 9.5 && snapshot().planner.last_plan_ms == 104.0);
    assert(snapshot().planner.reference_remaining_s == 1.25);
    auto target = teleop(1, -1);
    assert(near(target.at("vx"), 0.4) && near(target.at("vy"), -0.2));
    assert(near(service.Command().vx, 0.4) && near(service.Command().vy, -0.2));  // Not ramped.
    assert(call("preset", {{"name", "fast"}}).at("ok"));
    assert(near(teleop(1).at("vx"), 0.8) && near(teleop(-1).at("vx"), -0.3));
    reply = call("velocity", {{"vx", 9}, {"vy", 0}, {"wz", -9}});
    assert(near(reply.at("data").at("target_velocity").at("vx"), 0.8));
    assert(near(reply.at("data").at("target_velocity").at("wz"), -1.0));
    assert(refused("not_ready", "interaction", {{"action", "wave"}}));
    // A realtime planner keeps running on a zero command when control is released.
    assert(call("release").at("ok"));
    assert(idle() && service.Command().vx == 0 && service.Command().wz == 0);
    assert(service.Command().interaction.operation == robot_base::InteractionRequest::Operation::CANCEL);
    update();
    assert(call("acquire").at("ok"));

    reply = call("planner_stop");
    assert(reply.at("ok") && reply.at("code") == "accepted");
    assert(service.Command().planner.operation == Operation::STOP &&
        service.Command().planner.sequence == start_sequence + 1 && service.Command().planner.planner.empty());
    status.planner.sequence = start_sequence + 1;
    status.planner.phase = Phase::STOPPING;
    update();
    assert(snapshot().request.phase == "completed" && snapshot().request.operation == "planner_stop" && idle());
    assert(snapshot().planner.Engaged() && refused("not_ready", "interaction", {{"action", "wave"}}));
    assert(near(teleop(1).at("vx"), 0.08));  // The policy's range again.
    status.planner.phase = Phase::READY;
    status.planner.plan_count = 0;
    update();
    assert(!snapshot().planner.Engaged());
    reply = call("planner_stop");
    assert(reply.at("ok") && reply.at("code") == "ok" && idle());
    // Actions are available again; a planner cannot start while one plays.
    assert(call("interaction", {{"action", "wave"}}).at("ok"));
    status.interaction.sequence = service.Command().interaction.sequence;
    status.interaction.request_accepted = true;
    status.interaction.phase = robot_base::InteractionStatus::Phase::PLAYING;
    update();
    assert(refused("not_ready", "planner", {{"name", "walker"}}));
    status.interaction.phase = robot_base::InteractionStatus::Phase::FINISHED;
    update();

    // Control rejects a start: the request reports control's reason.
    assert(call("planner", {{"name", "walker"}}).at("ok"));
    status.planner.sequence = service.Command().planner.sequence;
    status.planner.request_accepted = false;
    status.planner.detail = "planner thread is busy";
    update();
    assert(snapshot().request.phase == "rejected" && snapshot().request.message == "planner thread is busy" && idle());
    assert(!snapshot().planner.accepted && snapshot().planner.detail == "planner thread is busy");

    // A failed planner is not engaged; a stop request clears the failure.
    status.planner.phase = Phase::FAILED;
    status.planner.detail = "model file not found";
    update();
    assert(snapshot().planner.phase == "FAILED" && !snapshot().planner.Engaged());
    assert(snapshot().planner.detail == "model file not found");
    reply = call("planner_stop");
    assert(reply.at("code") == "accepted" && service.Command().planner.operation == Operation::STOP);
    status.planner = {};
    status.planner.sequence = service.Command().planner.sequence;
    status.planner.request_accepted = true;
    update();
    assert(snapshot().request.phase == "completed" && snapshot().planner.phase == "IDLE" && idle());

    // Offline planner: the argument travels with the request; the generated
    // motion ends when the operator's control ends.
    const std::string text(robot_base::kPlannerArgumentMaxLength, 'x');
    assert(call("planner", {{"name", "writer"}, {"argument", text}}).at("ok"));
    const auto generate_sequence = service.Command().planner.sequence;
    assert(service.Command().planner.operation == Operation::START &&
        service.Command().planner.planner == "writer" && service.Command().planner.argument == text);
    status.planner.sequence = generate_sequence;
    status.planner.phase = Phase::GENERATING;
    status.planner.planner = "writer";
    update();
    assert(snapshot().request.phase == "completed" && snapshot().planner.phase == "GENERATING" && idle());
    assert(refused("not_ready", "planner", {{"name", "writer"}, {"argument", "again"}}));
    assert(call("release").at("ok"));
    assert(service.Command().planner.operation == Operation::STOP &&
        service.Command().planner.sequence == generate_sequence + 1);
    // The pending stop is dropped once control leaves RL.
    status.mode = robot_base::ControlMode::DAMP;
    status.planner = {};
    update();
    service.Tick();
    assert(idle() && snapshot().planner.phase == "IDLE");
}
} // namespace

int main(int argc, char **argv) {
    TestRobotFeedback();
    TestTeleop();
    TestAdministrator();
    TestTrajectory();
    TestEnvironment();
    TestPlanner();
    using operator_service::Config;
    using operator_service::DecodeStatus;
    using operator_service::Encode;
    using operator_service::Json;
    using operator_service::Service;
    if (argc > 1) {
        const auto loaded = operator_service::LoadConfig(argv[1]);
        assert(loaded.teleop.deadzone == 0.08);
        assert(loaded.teleop.acceleration.vy == 0.7);
        assert(loaded.teleop.deceleration.wz == 2.5);
        assert(loaded.teleop.slow_limits->vx == 0.05);
        assert(loaded.teleop.fast_limits->vx == 0.1);
        const auto teleop = Service(loaded).Catalog().at("teleop");
        assert(teleop.at("slow_limits").at("vy") == 0.04);
        assert(teleop.at("fast_limits").at("wz") == 0.2);
        assert(!loaded.policies.at(0).client_smoothing);
        const auto policy = Service(loaded).Catalog().at("policies").at(0);
        assert(!policy.at("client_smoothing").get<bool>());
        assert(!operator_service::DecodePolicy(policy).client_smoothing);
        assert(operator_service::DecodePolicy(policy).display_name == policy.at("name"));
        auto labelled = policy;
        labelled["display_name"] = "走跑运控";
        assert(operator_service::DecodePolicy(labelled).display_name == "走跑运控");
        labelled.erase("display_name");
        assert(operator_service::DecodePolicy(labelled).display_name == policy.at("name"));
        auto legacy = policy;
        legacy.erase("client_smoothing");
        legacy.erase("service_smoothing");
        assert(operator_service::DecodePolicy(legacy).client_smoothing);
        auto presets = operator_service::DecodePolicy(policy);
        assert(presets.slow_limits->vx == 0.05 && presets.fast_limits->vx == 0.1);
        presets.slow_limits = operator_service::Velocity{0.7, 0.4, 1};
        presets.fast_limits = presets.slow_limits;
        const auto decoded = operator_service::DecodePolicy(Encode(presets));
        assert(decoded.slow_limits->vx == 0.7 && decoded.fast_limits->vy == 0.4);
    }
    Config config;
    config.terminal_only = false;
    config.robot = "test";
    config.token = "test-token";
    config.public_url = "http://127.0.0.1:8765";
    config.lease_ms = 80;
    config.status_timeout = 0.08;
    config.request_timeout = 0.08;
    config.policies.push_back({"test_policy", true, {-0.1, -0.2, -0.3}, {0.1, 0.2, 0.3}, {{"wave", "Wave"}}});
    config.policies.back().client_smoothing = false;
    {
        Service offline(config);
        const auto spectator = offline.Open();
        uint64_t id = 0;
        const auto request = [&](const std::string &op, const Json &args) {
            return offline.Handle(spectator, {{"v", 1}, {"id", ++id}, {"op", op}, {"args", args}});
        };
        assert(!request("state", {{"state", "POWER_OFF"}})["ok"].get<bool>());
        assert(request("hello", {{"token", config.token}, {"name", "spectator"}})["ok"].get<bool>());
        assert(!offline.Snapshot(spectator).online && !offline.Snapshot(spectator).owns_control);
        assert(!request("state", {{"state", "RL"}})["ok"].get<bool>());
        assert(request("state", {{"state", "POWER_OFF"}})["ok"].get<bool>());
        assert(offline.Command().key == -1 && !offline.Snapshot(spectator).owns_control);
    }
    Service service(config);
    assert(!service.Catalog().at("teleop").contains("slow_limits"));
    assert(!service.Catalog().at("policies").at(0).at("client_smoothing").get<bool>());
    const auto alice = service.Open(), bob = service.Open();
    uint64_t alice_id = 0, bob_id = 0;
    const auto call = [&](uint64_t session, uint64_t *id, const std::string &op, const Json &args = Json::object()) {
        return service.Handle(session, {{"v", 1}, {"id", ++*id}, {"op", op}, {"args", args}});
    };
    assert(!call(alice, &alice_id, "acquire")["ok"].get<bool>());
    assert(!call(alice, &alice_id, "client_event", {{"event", "direction_down"}})["ok"].get<bool>());
    assert(!call(alice, &alice_id, "hello", {{"token", "wrong"}})["ok"].get<bool>());
    assert(call(alice, &alice_id, "hello", {{"token", config.token}, {"name", "alice"}})["ok"].get<bool>());
    assert(call(bob, &bob_id, "hello", {{"token", config.token}, {"name", "bob"}})["ok"].get<bool>());
    robot_base::ControlStatus status;
    status.active_policy = "test_policy";
    status.hmi_connected = true;
    const auto update = [&] { service.UpdateStatus(status, {}); };
    update();
    assert(!call(alice, &alice_id, "acquire")["ok"].get<bool>());
    update();
    status.hmi_connected = false;
    update();
    assert(!service.Snapshot().online);
    assert(!call(alice, &alice_id, "acquire")["ok"].get<bool>());
    status.hmi_connected = true;
    update();
    assert(call(alice, &alice_id, "acquire")["ok"].get<bool>());
    assert(!call(bob, &bob_id, "acquire")["ok"].get<bool>());
    assert(!call(bob, &bob_id, "client_event",
        {{"event", "teleop_preset"}, {"detail", "fast"}})["ok"].get<bool>());
    assert(call(alice, &alice_id, "client_event",
        {{"event", "teleop_preset"}, {"detail", "fast"}})["ok"].get<bool>());
    const auto preset = DecodeStatus(Encode(service.Snapshot(alice)));
    assert(preset.teleop_preset == "fast" && preset.teleop_preset_sequence == 1);
    assert(!call(alice, &alice_id, "client_event",
        {{"event", "teleop_preset"}, {"detail", "other"}})["ok"].get<bool>());
    assert(call(alice, &alice_id, "state", {{"state", "POWER_OFF"}})["ok"].get<bool>());
    update();
    assert(service.Command().key == -1);
    assert(service.Snapshot(alice).request.phase == "accepted");
    service.MarkCommandSent();
    update();
    assert(service.Command().key == 0);
    assert(service.Snapshot(alice).request.phase == "completed");
    assert(call(alice, &alice_id, "state", {{"state", "DAMP"}})["ok"].get<bool>());
    assert(service.Command().key == 1);
    assert(service.Snapshot(alice).request.phase == "accepted");
    status.mode = robot_base::ControlMode::DAMP;
    update();
    assert(service.Snapshot(alice).request.phase == "completed");
    assert(service.Command().key == 0);
    assert(service.Snapshot(alice).owns_control);
    assert(!call(alice, &alice_id, "state", {{"state", "RL"}})["ok"].get<bool>());
    status.mode = robot_base::ControlMode::ZERO;
    status.zero_ready = false;
    update();
    assert(!call(alice, &alice_id, "state", {{"state", "RL"}})["ok"].get<bool>());
    status.zero_ready = true;
    update();
    assert(!call(alice, &alice_id, "state", {{"state", "TRAJECTORY"}})["ok"].get<bool>());
    assert(call(alice, &alice_id, "state", {{"state", "RL"}})["ok"].get<bool>());
    status.mode = robot_base::ControlMode::RL;
    update();
    assert(
        call(alice, &alice_id, "velocity", {{"vx", 9}, {"vy", -9}, {"wz", 0}, {"valid_for_ms", 50}})["ok"].get<bool>());
    assert(service.Command().vx > 0.09F && service.Command().vx < 0.11F);
    assert(service.Command().vy < -0.19F);
    assert(call(bob, &bob_id, "client_event", {{"event", "page_hidden"}})["ok"].get<bool>());
    assert(call(alice, &alice_id, "client_event",
        {{"event", "direction_down"}, {"detail", "w\n"}})["ok"].get<bool>());
    assert(!call(alice, &alice_id, "client_event", {{"event", "unrecognized"}})["ok"].get<bool>());
    assert(!call(alice, &alice_id, "client_event",
        {{"event", "ui_error"}, {"detail", std::string(1025, 'x')}})["ok"].get<bool>());
    assert(service.Command().vx > 0.09F && service.Command().vy < -0.19F);
    assert(service.Snapshot(alice).owns_control);
    std::this_thread::sleep_for(std::chrono::milliseconds(55));
    update();
    service.Tick();
    assert(service.Command().vx == 0);
    assert(call(alice, &alice_id, "renew")["ok"].get<bool>());
    assert(!call(alice, &alice_id, "interaction", {{"action", "unknown"}})["ok"].get<bool>());
    assert(call(alice, &alice_id, "interaction", {{"action", "wave"}})["ok"].get<bool>());
    status.interaction.sequence = service.Command().interaction.sequence;
    status.interaction.request_accepted = true;
    status.interaction.phase = robot_base::InteractionStatus::Phase::PLAYING;
    update();
    assert(service.Command().interaction.operation == robot_base::InteractionRequest::Operation::NONE);
    assert(service.Command().interaction.action.empty());
    const auto interaction = DecodeStatus(Encode(service.Snapshot(alice)));
    assert(interaction.interaction_accepted && interaction.interaction_sequence == status.interaction.sequence);
    assert(call(alice, &alice_id, "cancel")["ok"].get<bool>());
    service.Disconnect(alice);
    assert(service.Snapshot().request.phase == "cancelled");
    assert(service.Command().interaction.operation == robot_base::InteractionRequest::Operation::CANCEL);
    assert(service.Command().key == 0 && service.Command().vx == 0);
    assert(service.Snapshot().state == "RL");
    update();
    assert(call(bob, &bob_id, "acquire")["ok"].get<bool>());
    assert(call(bob, &bob_id, "stop")["ok"].get<bool>());
    assert(!service.Snapshot(bob).owns_control);
    assert(!call(bob, &bob_id, "velocity", {{"vx", 0}, {"vy", 0}, {"wz", 0}})["ok"].get<bool>());
    assert(call(bob, &bob_id, "pairing")["data"]["qr_svg"].get<std::string>().find("<svg") == 0);
    const auto replay = service.Handle(bob, {{"v", 1}, {"id", bob_id}, {"op", "acquire"}});
    assert(!replay["ok"].get<bool>());
    update();
    assert(call(bob, &bob_id, "acquire")["ok"].get<bool>());
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    service.Tick();
    assert(!service.Snapshot().online && service.Snapshot().owner.empty());
    assert(service.Command().key == 0 && service.Command().vx == 0);
    assert(DecodeStatus(Encode(service.Snapshot())).state == "RL");
    {
        Config live_config = config;
        live_config.policies[0].switch_group = "compatible";
        auto peer = live_config.policies[0];
        peer.name = "peer";
        live_config.policies.push_back(peer);
        peer.name = "incompatible";
        peer.switch_group.clear();
        live_config.policies.push_back(peer);
        Service live(live_config);
        const auto session = live.Open();
        uint64_t id = 0;
        const auto request = [&](const std::string &op, const Json &args) {
            return live.Handle(session, {{"v", 1}, {"id", ++id}, {"op", op}, {"args", args}});
        };
        assert(request("hello", {{"token", config.token}, {"name", "live"}})["ok"].get<bool>());
        robot_base::ControlStatus running;
        running.mode = robot_base::ControlMode::RL;
        running.active_policy = "test_policy";
        running.hmi_connected = true;
        live.UpdateStatus(running, {});
        live.UpdateStatus(running, {});
        assert(request("acquire", Json::object())["ok"].get<bool>());
        assert(!request("policy", {{"policy", "hidden"}})["ok"].get<bool>());
        assert(!request("policy", {{"policy", "incompatible"}})["ok"].get<bool>());
        assert(request("velocity", {{"vx", 0.1}, {"vy", 0.0}, {"wz", 0.0}})["ok"].get<bool>());
        running.interaction.phase = robot_base::InteractionStatus::Phase::PLAYING;
        live.UpdateStatus(running, {});
        assert(request("policy", {{"policy", "peer"}})["ok"].get<bool>());
        assert(live.Command().vx == 0 && live.Command().switch_policy == "peer");
        assert(operator_service::DecodePolicy(live.Catalog()["policies"][0]).switch_group == "compatible");
    }
    std::puts(
        "operator service tests passed: auth, stale cache, lease, readiness, limits, disconnect, stop, replay, QR, "
        "planners");
}
