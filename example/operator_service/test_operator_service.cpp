/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_operator_service.cpp
 * @brief Operator authentication, lease, freshness and request regression tests
 */
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem> // NOLINT(build/c++17): isolated configuration fixtures.
#include <fstream>
#include <stdexcept>
#include <thread>

#include "service.h"

namespace {
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
    for (const auto *op : {"acquire", "stop", "cancel", "ack", "velocity", "interaction"})
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
    std::filesystem::remove_all(root);
}
} // namespace

int main(int argc, char **argv) {
    TestAdministrator();
    TestTrajectory();
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
        assert(operator_service::DecodePolicy(legacy).client_smoothing);
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
    Service service(config);
    assert(!service.Catalog().at("teleop").contains("slow_limits"));
    assert(service.Catalog().at("policies").at(0).at("client_smoothing").get<bool>());
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
        assert(request("policy", {{"policy", "peer"}})["ok"].get<bool>());
        assert(live.Command().vx == 0 && live.Command().switch_policy == "peer");
        assert(operator_service::DecodePolicy(live.Catalog()["policies"][0]).switch_group == "compatible");
    }
    std::puts(
        "operator service tests passed: auth, stale cache, lease, readiness, limits, disconnect, stop, replay, QR");
}
