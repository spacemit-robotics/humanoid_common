/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_prompt_events.cpp
 * @brief Confirmed state/action cues, deduplication and observation-only reconnect
 */
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

#include "prompt_events.h"
#include "input_connection.h"

void TestFaultReminders() {
    using Events = operator_service::PromptEvents;
    using Ms = std::chrono::milliseconds;
    const auto start = Events::Clock::time_point{};
    Events events(15);
    operator_service::Status status;
    status.online = true;
    const auto expect = [&](int ms, std::vector<std::string> cues) {
        assert(events.Update(status, start + Ms(ms)) == cues);
    };
    expect(0, {});
    status.fault = {true, true, 1, "driver", "timeout", "fault"};
    expect(1000, {"fault"});
    expect(15999, {});
    expect(16000, {"fault"});
    expect(16001, {});
    expect(60000, {"fault"});  // A delayed observer emits one cue, not a backlog.
    expect(60000, {});
    status.fault.sequence = 2;
    expect(61000, {"fault"});  // A new fault is announced immediately.
    status.fault.active = false;
    expect(75999, {});
    expect(76000, {"fault"});  // Still latched until confirmed.
    status.fault.latched = false;
    expect(91000, {});
    expect(120000, {});
    status.fault.latched = true;
    expect(121000, {"fault"});
    status.online = false;
    expect(140000, {});
    events.Reset();
    status.online = true;
    expect(150000, {});  // Reconnect seeds the timer without replaying the initial cue.
    expect(164999, {});
    expect(165000, {"fault"});
    Events initial_fault(15);
    assert(initial_fault.Update(status, start).empty());
    assert(initial_fault.Update(status, start + Ms(15000)) == std::vector<std::string>{"fault"});
    Events disabled;
    assert(disabled.Update(status, start).empty());
    assert(disabled.Update(status, start + Ms(60000)).empty());
}

int main() {
    TestFaultReminders();
    operator_audio::InputConnection disabled("", 0, 0);
    assert(!disabled.Connected());
    operator_audio::InputConnection missing("/nonexistent/operator-gamepad-test", 0, 0);
    assert(!missing.Connected());
    assert(!missing.Connected());
    operator_audio::InputConnection not_input("/dev/null", 0, 0);
    assert(!not_input.Connected());  // A present non-evdev node cannot announce a controller.
    operator_service::GamepadEvents gamepad;
    assert(gamepad.Update(false).empty());  // Startup without a controller is silent.
    assert(gamepad.Update(true) == "gamepad.connected");
    assert(gamepad.Update(true).empty());  // Idle input or repeated status stays silent.
    assert(gamepad.Update(false) == "gamepad.disconnected");
    assert(gamepad.Update(false).empty());
    assert(gamepad.Update(true) == "gamepad.connected");
    operator_service::PromptEvents ownership;
    operator_service::Status owner_status;
    owner_status.online = true;
    const auto expect_owner = [&](std::vector<std::string> expected) {
        assert(ownership.Update(owner_status) == expected);
    };
    expect_owner({});
    owner_status.request = {1, "acquire", "rejected", "held by another client"};
    expect_owner({});  // A request is not proof of ownership.
    owner_status.owner = "web#1";
    expect_owner({"control.acquired"});
    expect_owner({});  // Heartbeats and renewals do not announce again.
    owner_status.owner.clear();
    expect_owner({"control.released"});  // Release, disconnect and lease expiry share this status.
    expect_owner({});
    owner_status.owner = "gamepad#2";
    expect_owner({"control.acquired"});
    owner_status.owner = "web#3";
    expect_owner({"control.acquired"});  // Direct handoff: announce only the new acquisition.
    owner_status.online = false;
    owner_status.owner.clear();
    expect_owner({});  // Missing feedback alone must not invent a release.
    owner_status.online = true;
    owner_status.owner = "web#3";
    expect_owner({});
    ownership.Reset();
    expect_owner({});  // Reconnecting an observer does not replay existing ownership.
    owner_status.owner.clear();
    expect_owner({"control.released"});
    operator_service::PromptEvents events;
    operator_service::Status status;
    const auto expect = [&](std::vector<std::string> expected) { assert(events.Update(status) == expected); };
    expect({});
    status.online = true;
    expect({});
    status.state = "DAMP";
    expect({"state.DAMP"});
    expect({});
    status.request = {1, "state", "accepted", "RL request not yet confirmed"};
    expect({});
    status.state = "RL";
    status.policy = "walk";
    expect({"state.RL", "policy.walk"});
    status.request = {2, "policy", "accepted", "dance still loading"};
    expect({});
    status.request.phase = "rejected";
    expect({});
    status.policy = "dance";
    expect({"policy.dance"});
    expect({});
    status.interaction_sequence = 1;
    status.interaction_phase = "REJECTED";
    expect({});
    status.interaction_sequence = 2;
    status.interaction_accepted = true;
    status.interaction_phase = "BLEND_IN";
    status.interaction_action = "shake_hand";
    expect({"action.shake_hand"});
    status.interaction_phase = "PLAYING";
    expect({});
    // A cancel acknowledgement also increments sequence; it is not a new action.
    // The controller may publish PLAYING once before the BLEND_OUT sample.
    status.interaction_sequence = 3;
    expect({});
    status.interaction_phase = "BLEND_OUT";
    expect({});
    status.interaction_phase = "IDLE";
    expect({});
    status.interaction_sequence = 4;
    status.interaction_phase = "PLAYING";
    expect({"action.shake_hand"});
    status.teleop_preset = "fast";
    status.teleop_preset_sequence = 1;
    expect({"preset.fast"});
    expect({});
    status.online = false;
    status.state = "POWER_OFF";
    expect({});
    status.online = true;
    status.fault = {true, true, 1, "driver", "timeout", "not an identified emergency stop"};
    expect({"fault"});
    expect({});
    events.Reset();
    expect({});
    status.fault = {};
    status.state = "ZERO";
    status.policy = "walk";
    expect({"state.ZERO"});
    status.policy = "dance";
    expect({});  // Selection outside RL is not a successful mode entry.
    status.state = "RL";
    expect({"state.RL", "policy.dance"});
    std::puts("audio events: confirmed status, ownership, actions, preset, fault, gamepad and reconnect passed");
}
