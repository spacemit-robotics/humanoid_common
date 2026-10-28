/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file gamepad_input.h
 * @brief Evdev snapshots and policy-scoped tap/hold operator bindings
 */
#ifndef GAMEPAD_INPUT_H
#define GAMEPAD_INPUT_H
#include <linux/input.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <string>
#include <vector>
#include "operator_types.h"

namespace operator_gamepad {
struct Binding {
    std::set<std::string> buttons;
    std::string operation;
    std::string value;
    int hold_ms = 0;
    std::string policy;
    bool on_release = false;
};
inline bool Applies(const Binding &binding, const std::string &policy) {
    return binding.policy.empty() || binding.policy == policy;
}
struct Layout {
    std::map<int, std::string> buttons;
    int left_x = ABS_X, left_y = ABS_Y, right_x = ABS_Z;
};
inline Layout MakeLayout(const std::string &name) {
    if (name == "ds4_generic")
        return {{{304, "X"}, {305, "A"}, {306, "B"}, {307, "Y"},
                {308, "LB"}, {309, "RB"}, {310, "LT"}, {311, "RT"},
                {312, "BACK"}, {313, "START"}, {314, "L3"}, {315, "R3"},
                {316, "GUIDE"}, {317, "TOUCHPAD"}}, ABS_X, ABS_Y, ABS_Z};
    if (name == "xbox")
        return {{{304, "A"}, {305, "B"}, {307, "X"}, {308, "Y"},
                {310, "LB"}, {311, "RB"}, {314, "BACK"}, {315, "START"},
                {316, "GUIDE"}, {317, "L3"}, {318, "R3"}}, ABS_X, ABS_Y, ABS_RX};
    throw std::runtime_error("gamepad layout must be ds4_generic or xbox");
}
inline double Normalize(int value, int minimum, int maximum, double deadzone) {
    if (minimum >= maximum) throw std::runtime_error("invalid gamepad axis range");
    const double v = std::clamp(2.0 * (value - minimum) / (maximum - minimum) - 1, -1.0, 1.0);
    return std::abs(v) <= deadzone ? 0 : std::copysign((std::abs(v) - deadzone) / (1 - deadzone), v);
}
class Chords {
public:
    using Clock = std::chrono::steady_clock;
    explicit Chords(std::vector<Binding> bindings)
        : bindings_(std::move(bindings)), pending_(bindings_.size()) {}
    void Reset(const std::set<std::string> &held) {
        previous_ = held;
        pending_ = bindings_.size();
    }
    void SetPolicy(const std::string &policy) {
        if (policy != policy_ && pending_ != bindings_.size() &&
            !bindings_[pending_].policy.empty()) pending_ = bindings_.size();
        policy_ = policy;
    }
    std::vector<Binding> Update(const std::set<std::string> &held,
        Clock::time_point now = Clock::now()) {
        std::vector<Binding> result;
        // Exact sets: adding a modifier cancels the pending single-key tap/hold.
        // Releasing a chord never arms the remaining single key.
        bool pressed = false;
        for (const auto &key : held) pressed |= !previous_.count(key);
        if (held != previous_ && !pressed && pending_ != bindings_.size() &&
            bindings_[pending_].on_release) result.push_back(bindings_[pending_]);
        if (held != previous_) pending_ = bindings_.size();
        if (pressed && !held.empty() && held.size() <= 2)
            for (size_t i = 0; i < bindings_.size(); ++i) {
                const auto &binding = bindings_[i];
                if (binding.buttons != held || !Applies(binding, policy_)) continue;
                if (binding.hold_ms == 0 && !binding.on_release) result.push_back(binding);
                else {
                    pending_ = i;
                    pressed_at_ = now;
                }
            }
        previous_ = held;
        return result;
    }
    // Poll after draining input, even if a stationary controller sends no events.
    std::vector<Binding> Poll(Clock::time_point now = Clock::now()) {
        if (pending_ == bindings_.size() || bindings_[pending_].on_release ||
            now - pressed_at_ < std::chrono::milliseconds(bindings_[pending_].hold_ms)) return {};
        const auto binding = bindings_[pending_];
        pending_ = bindings_.size();
        return {binding};
    }
private:
    std::vector<Binding> bindings_;
    std::set<std::string> previous_;
    std::string policy_;
    size_t pending_;
    Clock::time_point pressed_at_{};
};
inline double Advance(double value, double target, double accel, double decel, double dt) {
    if (value * target < 0) {
        const double braking = std::abs(value) / decel;
        if (dt <= braking) return value - std::copysign(decel * dt, value);
        dt -= braking;
        value = 0;
    }
    const double rate = std::abs(target) > std::abs(value) ? accel : decel;
    return value + std::clamp(target - value, -rate * dt, rate * dt);
}
inline operator_service::Velocity Scale(const operator_service::Velocity &input,
    const operator_service::Policy &policy, const std::array<double, 3> &caps) {
    const auto scale = [](double x, double low, double high, double cap) {
        return x * std::min(cap, x >= 0 ? std::max(0.0, high) : std::max(0.0, -low));
    };
    return {scale(input.vx, policy.minimum.vx, policy.maximum.vx, caps[0]),
            scale(input.vy, policy.minimum.vy, policy.maximum.vy, caps[1]),
            scale(input.wz, policy.minimum.wz, policy.maximum.wz, caps[2])};
}
inline bool Neutral(const operator_service::Velocity &value) {
    return value.vx == 0 && value.vy == 0 && value.wz == 0;
}
}  // namespace operator_gamepad
#endif  // GAMEPAD_INPUT_H
