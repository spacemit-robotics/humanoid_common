/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_operator_service.cpp
 * @brief Operator authentication, lease, freshness and request regression tests
 */
#include <cassert>
#include <chrono>
#include <cstdio>
#include <thread>

#include "service.h"

int main() {
    using operator_service::Config;
    using operator_service::DecodeStatus;
    using operator_service::Encode;
    using operator_service::Json;
    using operator_service::Service;
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
    const auto alice = service.Open(), bob = service.Open();
    uint64_t alice_id = 0, bob_id = 0;
    const auto call = [&](uint64_t session, uint64_t *id, const std::string &op, const Json &args = Json::object()) {
        return service.Handle(session, {{"v", 1}, {"id", ++*id}, {"op", op}, {"args", args}});
    };
    assert(!call(alice, &alice_id, "acquire")["ok"].get<bool>());
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
    assert(call(alice, &alice_id, "state", {{"state", "RL"}})["ok"].get<bool>());
    status.mode = robot_base::ControlMode::RL;
    update();
    assert(
        call(alice, &alice_id, "velocity", {{"vx", 9}, {"vy", -9}, {"wz", 0}, {"valid_for_ms", 50}})["ok"].get<bool>());
    assert(service.Command().vx > 0.09F && service.Command().vx < 0.11F);
    assert(service.Command().vy < -0.19F);
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
    std::puts(
        "operator service tests passed: auth, stale cache, lease, readiness, limits, disconnect, stop, replay, QR");
}
