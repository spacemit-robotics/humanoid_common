/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_gamepad_input.cpp
 * @brief Input calibration, timed chords and policy-aware speed scaling
 */
#include "gamepad_input.h"
#include <cassert>
#include <cstdio>

void TestHeldChords() {
    using operator_gamepad::Binding;
    using operator_gamepad::Chords;
    using Ms = std::chrono::milliseconds;
    const auto start = Chords::Clock::time_point{};
    for (const auto &binding : std::vector<Binding>{
        {{"LB", "A"}, "state", "DAMP", 1000},
        {{"RB", "A"}, "policy", "walk", 1000},
        {{"RB", "UP"}, "action", "rhythm_1", 1000}}) {
        Chords chords({binding});
        const std::set<std::string> one{*binding.buttons.begin()};
        assert(chords.Update(one, start).empty());
        assert(chords.Update(binding.buttons, start + Ms(100)).empty());
        assert(chords.Poll(start + Ms(1099)).empty());
        // Identical reports do not restart timing; no report is needed at expiry.
        assert(chords.Update(binding.buttons, start + Ms(1099)).empty());
        auto commands = chords.Poll(start + Ms(1100));
        assert(commands.size() == 1 && commands[0].value == binding.value);
        assert(chords.Update(binding.buttons, start + Ms(2000)).empty());
        assert(chords.Poll(start + Ms(3000)).empty());
        // Releasing and pressing again starts a new full hold.
        assert(chords.Update(one, start + Ms(3100)).empty());
        assert(chords.Update(binding.buttons, start + Ms(3200)).empty());
        assert(chords.Poll(start + Ms(4199)).empty());
        assert(chords.Poll(start + Ms(4200)).size() == 1);
    }
    Chords chords({{{"LB", "Y"}, "state", "RL", 1000}});
    assert(chords.Update({"LB", "Y"}, start).empty());
    assert(chords.Update({"LB"}, start + Ms(999)).empty());
    assert(chords.Poll(start + Ms(1000)).empty());  // Early release cancels.
    assert(chords.Update({"LB", "Y"}, start + Ms(1100)).empty());
    assert(chords.Poll(start + Ms(2000)).empty());  // No accumulated hold time.
    assert(chords.Update({"LB", "Y", "A"}, start + Ms(2050)).empty());
    assert(chords.Poll(start + Ms(2100)).empty());
    assert(chords.Update({"LB", "Y"}, start + Ms(2200)).empty());
    assert(chords.Poll(start + Ms(4000)).empty());  // Third-key release cannot arm.
    chords.Reset({});
    assert(chords.Update({"LB", "Y"}, start + Ms(4100)).empty());
    chords.Reset({"LB", "Y"});  // Reconnect or dropped-input resync cancels hold.
    assert(chords.Update({"LB", "Y"}, start + Ms(4200)).empty());
    assert(chords.Poll(start + Ms(6000)).empty());
    chords.Reset({});
    assert(chords.Update({"LB", "Y"}, start + Ms(6100)).empty());
    // A queued release is processed before polling the elapsed timer.
    assert(chords.Update({}, start + Ms(8000)).empty());
    assert(chords.Poll(start + Ms(8000)).empty());
}

void TestSingleAndCombinedBindings() {
    using operator_gamepad::Chords;
    using Ms = std::chrono::milliseconds;
    const auto start = Chords::Clock::time_point{};
    Chords chords({
        {{"LB"}, "state", "HOME", 1000}, {{"RB"}, "state", "ZERO", 1000},
        {{"LB", "RB"}, "state", "RL", 1000}, {{"RT"}, "cancel", "", 1000},
        {{"Y"}, "preset", "toggle", 0, "walk", true},
        {{"Y"}, "preset", "toggle", 0, "obstacle", true},
        {{"RT", "Y"}, "action", "clap", 0, "walk", true},
        {{"RT", "Y"}, "action", "rhythm_4", 1000, "dance"},
        {{"CAPTURE"}, "control", "", 0, "", true}, {{"R3"}, "ack", "", 0, "", true}});
    chords.SetPolicy("walk");
    for (const auto &key : {"LB", "RB"}) {
        chords.Reset({});
        assert(chords.Update({key}, start).empty());
        assert(chords.Poll(start + Ms(999)).empty());
        const auto command = chords.Poll(start + Ms(1000));
        assert(command.size() == 1 && command[0].buttons == std::set<std::string>{key});
        assert(chords.Poll(start + Ms(2000)).empty());
    }
    for (const auto &first : {"LB", "RB"}) {
        chords.Reset({});
        assert(chords.Update({first}, start).empty());
        assert(chords.Update({"LB", "RB"}, start + Ms(700)).empty());
        assert(chords.Poll(start + Ms(1000)).empty());  // Cancel the individual key timer.
        assert(chords.Poll(start + Ms(1699)).empty());
        const auto command = chords.Poll(start + Ms(1700));
        assert(command.size() == 1 && command[0].value == "RL");
        assert(chords.Update({first}, start + Ms(2000)).empty());
        assert(chords.Poll(start + Ms(5000)).empty());  // No HOME/ZERO on chord release.
    }
    for (const auto &first : {"RT", "Y"}) {
        chords.Reset({});
        assert(chords.Update({first}, start).empty());
        assert(chords.Update({"RT", "Y"}, start + Ms(100)).empty());
        assert(chords.Poll(start + Ms(4000)).empty());  // Tap gesture is not RT hold/cancel.
        const auto commands = chords.Update({first}, start + Ms(4100));
        assert(commands.size() == 1 && commands[0].value == "clap");
        assert(chords.Poll(start + Ms(6000)).empty());
        assert(chords.Update({}, start + Ms(6100)).empty());  // No Y toggle or cancel.
    }
    for (const auto &key : {"Y", "CAPTURE", "R3"}) {
        chords.Reset({});
        assert(chords.Update({key}, start).empty());
        assert(chords.Poll(start + Ms(200)).empty());
        auto command = chords.Update({}, start + Ms(201));
        assert(command.size() == 1 && command[0].buttons == std::set<std::string>{key});
        assert(chords.Update({}, start + Ms(202)).empty());
    }
    chords.Reset({});
    assert(chords.Update({"RT"}, start).empty());
    assert(chords.Poll(start + Ms(999)).empty());
    assert(chords.Poll(start + Ms(1000)).at(0).operation == "cancel");
    assert(chords.Poll(start + Ms(4000)).empty());
    chords.Reset({});
    assert(chords.Update({"Y"}, start).empty());
    assert(chords.Update({"Y", "A"}, start + Ms(100)).empty());
    assert(chords.Update({"Y"}, start + Ms(200)).empty());
    assert(chords.Update({}, start + Ms(300)).empty());  // Cancel a contaminated tap.
    chords.Reset({});
    assert(chords.Update({"RT", "Y"}, start).empty());
    chords.SetPolicy("dance");
    assert(chords.Update({"RT", "Y"}, start + Ms(100)).empty());
    assert(chords.Poll(start + Ms(2000)).empty());  // Mode change cannot start an already-held chord.
    assert(chords.Update({}, start + Ms(2100)).empty());
    assert(chords.Update({"RT", "Y"}, start + Ms(2200)).empty());
    assert(chords.Poll(start + Ms(3199)).empty());
    assert(chords.Poll(start + Ms(3200)).at(0).value == "rhythm_4");
    assert(chords.Update({"RT"}, start + Ms(3300)).empty());
    assert(chords.Poll(start + Ms(6000)).empty());
    chords.Reset({});
    chords.SetPolicy("obstacle");
    assert(chords.Update({"Y"}, start).empty());
    const auto preset = chords.Update({}, start + Ms(100));
    assert(preset.size() == 1 && preset[0].operation == "preset");
    assert(chords.Update({}, start + Ms(101)).empty());
    for (const auto &policy : {"dance", ""}) {
        chords.Reset({});
        chords.SetPolicy(policy);
        assert(chords.Update({"Y"}, start).empty());
        assert(chords.Update({}, start + Ms(100)).empty());  // Only configured scopes toggle speed.
    }
    chords.Reset({});
    assert(chords.Update({"LB"}, start).empty());
    chords.SetPolicy("dance");
    assert(chords.Poll(start + Ms(1000)).at(0).value == "HOME");  // Global FSM keys keep working.
    chords.Reset({"CAPTURE"});
    assert(chords.Update({}, start).empty());  // Reconnect while held must not claim ownership.
}

int main() {
    assert(operator_gamepad::IsPowerOffRequest({{"BACK"}, "state", "POWER_OFF", 1000}));
    assert(!operator_gamepad::IsPowerOffRequest({{"START"}, "state", "DAMP", 1000}));
    assert(!operator_gamepad::IsPowerOffRequest({{"LB", "RB"}, "state", "RL", 1000}));
    assert(!operator_gamepad::IsPowerOffRequest({{"LT", "X"}, "policy", "dance", 1000}));
    TestHeldChords();
    TestSingleAndCombinedBindings();
    using operator_gamepad::MakeLayout;
    using operator_gamepad::Normalize;
    using operator_gamepad::Chords;
    using operator_gamepad::Scale;
    using operator_gamepad::Advance;
    using operator_gamepad::Neutral;
    const auto ds4 = MakeLayout("ds4_generic"), xbox = MakeLayout("xbox");
    assert(ds4.buttons.at(305) == "A" && ds4.buttons.at(304) == "X");
    assert(ds4.buttons.at(308) == "LB" && ds4.right_x == ABS_Z);
    assert(ds4.buttons.at(310) == "LT" && ds4.buttons.at(311) == "RT");
    assert(ds4.buttons.at(315) == "R3" && ds4.buttons.at(317) == "TOUCHPAD");
    assert(xbox.buttons.at(304) == "A" && xbox.right_x == ABS_RX);
    assert(Normalize(127, 0, 255, 0.06) == 0);
    assert(Normalize(128, 0, 255, 0.06) == 0);
    assert(Normalize(0, 0, 255, 0.06) == -1);
    assert(Normalize(255, 0, 255, 0.06) == 1);
    assert(Normalize(32767, -32768, 32767, 0.06) == 1);
    Chords chords({{{"LB", "A"}, "state", "DAMP"}, {{"RB", "Y"}, "policy", "dance"}});
    chords.Reset({"LB", "A"});
    assert(chords.Update({"LB", "A"}).empty());  // Held during connect does not fire.
    assert(chords.Update({}).empty());
    assert(chords.Update({"LB"}).empty());
    auto commands = chords.Update({"LB", "A"});
    assert(commands.size() == 1 && commands[0].value == "DAMP");
    assert(chords.Update({"LB", "A"}).empty());  // No repeats while held.
    assert(chords.Update({"LB", "A", "Y"}).empty());
    assert(chords.Update({"LB", "A"}).empty());  // Releasing third key is not a press.
    chords.Update({});
    assert(chords.Update({"Y"}).empty());
    assert(chords.Update({"RB", "Y"}).size() == 1);  // Either button may come first.
    chords.Reset({"RB", "Y"});  // SYN_DROPPED snapshot must not replay a chord.
    assert(chords.Update({"RB", "Y"}).empty());
    operator_service::Policy policy;
    policy.minimum = {-1, -0.3, -1};
    policy.maximum = {2, 0.4, 1};
    const auto v = Scale({1, -1, 0.5}, policy, {0.6, 0.6, 1.0});
    assert(v.vx == 0.6 && v.vy == -0.3 && v.wz == 0.5);
    assert(Scale({-1, 0, 0}, policy, {1.5, 0.6, 1}).vx == -1);
    policy.minimum.vx = -0.6;
    assert(Scale({1, 0, 0}, policy, {0.6, 0.6, 1}).vx == 0.6);
    assert(Scale({1, 0, 0}, policy, {1.5, 0.6, 1}).vx == 1.5);
    assert(Scale({-1, 0, 0}, policy, {0.6, 0.6, 1}).vx == -0.6);
    assert(Scale({-1, 0, 0}, policy, {1.5, 0.6, 1}).vx == -0.6);
    assert(Scale({-0.5, 0, 0}, policy, {1.5, 0.6, 1}).vx == -0.3);
    assert(Advance(0, 1, 1, 2.5, 0.02) == 0.02);
    assert(std::abs(Advance(1, 0, 1, 2.5, 0.02) - 0.95) < 1e-12);
    assert(Advance(0.1, -1, 1, 2, 0.1) < 0);
    policy.minimum = {};
    policy.maximum = {};
    assert(Neutral(Scale({1, 1, 1}, policy, {1, 1, 1})));  // Dance stays stationary.
    std::puts("gamepad input: layout, deadzone, chord edges/holds, reconnect, scaling and ramps passed");
}
