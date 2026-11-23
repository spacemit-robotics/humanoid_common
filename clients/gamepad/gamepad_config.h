/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file gamepad_config.h
 * @brief Machine-declared gamepad layout, policy scopes and binding validation
 */
#ifndef GAMEPAD_CONFIG_H
#define GAMEPAD_CONFIG_H
#include <yaml-cpp/yaml.h>
#include <string>
#include "gamepad_input.h"
#include "gamepad_feedback.h"

namespace operator_gamepad {
struct Config {
    std::string device, layout;
    Layout input;
    int vendor = 0, product = 0;
    double deadzone = 0.06;
    std::vector<Binding> bindings;
    FeedbackConfig feedback;
};
inline Config ReadConfig(const YAML::Node &root) {
    const auto pad = root["operator_gamepad"], teleop = root["operator_service"]["teleop"];
    Config c;
    c.device = pad["device"].as<std::string>();
    c.layout = pad["layout"].as<std::string>();
    c.input = MakeLayout(c.layout);
    c.vendor = pad["vendor"].as<int>();
    c.product = pad["product"].as<int>();
    if (const auto feedback = pad["rumble"]; feedback && feedback["enabled"].as<bool>(false)) {
        if (feedback["backend"].as<std::string>() != "ds4_usb" || c.layout != "ds4_generic")
            throw std::runtime_error("unsupported gamepad rumble backend/layout");
        c.feedback.device = feedback["device"].as<std::string>();
        c.feedback.duration_ms = feedback["duration_ms"].as<int>(120);
        c.feedback.strength = feedback["strength"].as<int>(90);
        if (c.feedback.device.empty() || c.feedback.duration_ms < 1 || c.feedback.duration_ms > 500 ||
            c.feedback.strength < 1 || c.feedback.strength > 255)
            throw std::runtime_error("invalid gamepad rumble configuration");
    }
    c.deadzone = teleop && teleop["deadzone"] ? teleop["deadzone"].as<double>() : c.deadzone;
    if (!std::isfinite(c.deadzone) || c.deadzone < 0 || c.deadzone >= 0.5)
        throw std::runtime_error("invalid gamepad deadzone");
    // Physical extra-button names belong to the machine YAML, not the generic layout.
    if (const auto codes = pad["button_codes"]) {
        if (!codes.IsMap()) throw std::runtime_error("button_codes must be a map");
        std::set<int> used;
        for (const auto &entry : codes) {
            const auto name = entry.first.as<std::string>();
            const auto code = entry.second.as<int>();
            if (name.empty() || code <= 0 || code > KEY_MAX || !used.insert(code).second ||
                std::set<std::string>{"UP", "DOWN", "LEFT", "RIGHT"}.count(name))
                throw std::runtime_error("invalid gamepad button code override");
            c.input.buttons[code] = name;
        }
    }
    std::set<std::string> names{"UP", "DOWN", "LEFT", "RIGHT"};
    for (const auto &[code, name] : c.input.buttons)
        if (!names.insert(name).second) throw std::runtime_error("duplicated gamepad button name");
    const std::set<std::string> operations{
        "control", "state", "policy", "preset", "action", "cancel", "ack", "reference", "planner", "planner_preset"};
    const auto policies = root["rl_policy"]["onnx_infer"]["policy_names"].as<std::vector<std::string>>();
    const auto known_policy = [&](const std::string &name) {
        return std::find(policies.begin(), policies.end(), name) != policies.end();
    };
    // A planner is started inside a tracker policy that lists it under policy_adapter.planners.
    const auto lists_planner = [&](const std::string &policy, const std::string &planner) {
        const auto entries = root["rl_policy"]["onnx_infer"]["policies"];
        if (!entries || !entries[policy] || !entries[policy]["policy_adapter"]) return false;
        const auto list = entries[policy]["policy_adapter"]["planners"];
        if (!list || !list.IsSequence()) return false;
        const auto names = list.as<std::vector<std::string>>();
        return std::find(names.begin(), names.end(), planner) != names.end();
    };
    for (const auto &node : pad["bindings"]) {
        const auto buttons = node["buttons"].as<std::vector<std::string>>();
        Binding b{{buttons.begin(), buttons.end()}, node["operation"].as<std::string>(),
            node["value"].as<std::string>(""), node["hold_ms"] ? node["hold_ms"].as<int>() : 0,
            node["policy"] ? node["policy"].as<std::string>() : ""};
        const auto trigger = node["trigger"] ? node["trigger"].as<std::string>() : "press";
        b.on_release = trigger == "tap";
        if (b.hold_ms < 0 || (trigger != "press" && trigger != "tap") || (b.on_release && b.hold_ms))
            throw std::runtime_error("invalid gamepad hold_ms/trigger");
        if (buttons.empty() || buttons.size() > 2 || b.buttons.size() != buttons.size() ||
            !operations.count(b.operation)) throw std::runtime_error("invalid gamepad binding");
        for (const auto &old : c.bindings)
            if (old.buttons == b.buttons && (old.policy.empty() || b.policy.empty() || old.policy == b.policy))
                throw std::runtime_error("overlapping gamepad bindings");
        for (const auto &name : b.buttons)
            if (!names.count(name)) throw std::runtime_error("unknown gamepad button: " + name);
        if (!b.policy.empty() && !known_policy(b.policy)) throw std::runtime_error("unknown binding policy scope");
        if (b.operation == "state" &&
            !std::set<std::string>{"POWER_OFF", "DAMP", "HOME", "ZERO", "RL"}.count(b.value))
            throw std::runtime_error("unknown FSM state");
        if (b.operation == "policy" && !known_policy(b.value))
            throw std::runtime_error("gamepad policy is not in selectable catalog");
        if (b.operation == "preset" && b.value != "slow" && b.value != "fast" && b.value != "toggle")
            throw std::runtime_error("unknown speed preset");
        if (b.operation == "planner_preset" &&
            !std::set<std::string>{"previous", "next", "higher", "lower"}.count(b.value))
            throw std::runtime_error("unknown planner preset adjustment");
        if (b.operation == "planner" && !(b.policy.empty()
                ? std::any_of(policies.begin(), policies.end(),
                    [&](const std::string &policy) { return lists_planner(policy, b.value); })
                : lists_planner(b.policy, b.value)))
            throw std::runtime_error("gamepad planner is not listed by the bound policy");
        c.bindings.push_back(std::move(b));
    }
    if (c.bindings.empty()) throw std::runtime_error("empty gamepad bindings");
    return c;
}
}  // namespace operator_gamepad
#endif  // GAMEPAD_CONFIG_H
