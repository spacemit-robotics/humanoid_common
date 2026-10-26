/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_gamepad_config.cpp
 * @brief Check tap/hold scopes, machine key names and optional real YAML replay
 */
#include "gamepad_config.h"
#include <cassert>
#include <cstdio>

void CheckBindings(const operator_gamepad::Config &config) {
    using operator_gamepad::Chords;
    using Ms = std::chrono::milliseconds;
    const auto start = Chords::Clock::time_point{};
    for (const auto &binding : config.bindings) {
        Chords chords(config.bindings);
        chords.SetPolicy(binding.policy);
        auto commands = chords.Update(binding.buttons, start);
        if (binding.hold_ms) {
            assert(commands.empty());
            assert(chords.Poll(start + Ms(binding.hold_ms - 1)).empty());
            commands = chords.Poll(start + Ms(binding.hold_ms));
        } else if (binding.on_release) {
            assert(commands.empty());
            commands = chords.Update({}, start + Ms(30));
        }
        assert(commands.size() == 1);
        assert(commands[0].operation == binding.operation && commands[0].value == binding.value);
        assert(chords.Poll(start + Ms(10000)).empty());
    }
}

int main(int argc, char **argv) {
    if (argc == 2) {
        const auto config = operator_gamepad::ReadConfig(YAML::LoadFile(argv[1]));
        CheckBindings(config);
        std::printf("YAML replay: %zu bindings passed (no device or robot access)\n", config.bindings.size());
        return 0;
    }
    const auto root = YAML::Load(R"(
operator_gamepad:
    device: /dev/null
    layout: ds4_generic
    vendor: 0x054c
    product: 0x09cc
    button_codes: {CAPTURE: 317}
    bindings:
        - {buttons: [CAPTURE], operation: control, trigger: tap}
        - {buttons: [BACK], operation: state, value: POWER_OFF, hold_ms: 1000}
        - {buttons: [Y], operation: preset, value: toggle, trigger: tap, policy: walk}
        - {buttons: [RT, A], operation: action, value: shake_hand, trigger: tap, policy: walk}
        - {buttons: [RT, A], operation: action, value: rhythm_1, hold_ms: 1000, policy: dance}
operator_service:
    teleop:
        deadzone: 0.06
        slow_limits: [0.6, 0.6, 1]
        fast_limits: [1.5, 0.6, 1]
        acceleration: [1, 1, 2]
        deceleration: [2.5, 0.5, 2.5]
rl_policy:
    onnx_infer:
        policy_names: [walk, dance]
)");
    const auto config = operator_gamepad::ReadConfig(root);
    assert(config.input.buttons.at(317) == "CAPTURE");
    CheckBindings(config);
    const auto rejects = [&](const YAML::Node &replacement, const std::string &field = "bindings") {
        auto bad = YAML::Clone(root);
        bad["operator_gamepad"][field] = replacement;
        bool rejected = false;
        try { operator_gamepad::ReadConfig(bad); } catch (const std::exception &) { rejected = true; }
        assert(rejected);
    };
    for (const auto &yaml : {
        "[{buttons: [], operation: ack}]",
        "[{buttons: [LB, LB], operation: ack}]",
        "[{buttons: [LB, RB, Y], operation: ack}]",
        "[{buttons: [M], operation: ack}]",
        "[{buttons: [LB], operation: state, value: TEST}]",
        "[{buttons: [LB], operation: state, value: HOME, hold_ms: -1}]",
        "[{buttons: [LB], operation: state, value: HOME, hold_ms: invalid}]",
        "[{buttons: [LB], operation: state, value: HOME, hold_ms: 1.5}]",
        "[{buttons: [Y], operation: preset, value: toggle, trigger: unknown}]",
        "[{buttons: [Y], operation: preset, value: toggle, trigger: tap, hold_ms: 1000}]",
        "[{buttons: [Y], operation: preset, value: toggle, policy: absent}]",
        "[{buttons: [Y], operation: preset, value: toggle, policy: [walk]}]",
        "[{buttons: [Y], operation: policy, value: absent}]",
        "[{buttons: [Y], operation: preset, value: turbo}]",
        "[{buttons: [Y], operation: ack}, {buttons: [Y], operation: ack, policy: walk}]",
        "[{buttons: [Y], operation: ack, policy: walk}, {buttons: [Y], operation: ack}]",
        "[{buttons: [Y], operation: ack, policy: walk}, {buttons: [Y], operation: ack, policy: walk}]"})
        rejects(YAML::Load(yaml));
    for (const auto &yaml : {"{CAPTURE: -1}", "{CAPTURE: 10000}", "{CAPTURE: bad}",
        "{CAPTURE: 317, OTHER: 317}", "{A: 317}", "{UP: 317}", "[317]"})
        rejects(YAML::Load(yaml), "button_codes");
    for (const auto &yaml : {
        "{enabled: true, backend: unknown, device: /dev/null}",
        "{enabled: true, backend: ds4_usb, device: /dev/null, strength: 256}",
        "{enabled: true, backend: ds4_usb, device: /dev/null, duration_ms: 0}"})
        rejects(YAML::Load(yaml), "rumble");
    std::puts("gamepad config: tap/hold replay, context overlap and invalid mappings passed");
}
