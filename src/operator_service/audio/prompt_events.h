/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file prompt_events.h
 * @brief Select audio cues from confirmed robot status, without issuing commands
 */
#ifndef PROMPT_EVENTS_H
#define PROMPT_EVENTS_H
#include <chrono>
#include <string>
#include <vector>

#include "operator_types.h"

namespace operator_service {
class GamepadEvents {
public:
    std::string Update(bool connected) {
        if (connected == connected_) return {};
        connected_ = connected;
        return connected ? "gamepad.connected" : "gamepad.disconnected";
    }

private:
    bool connected_ = false;
};

class PromptEvents {
public:
    using Clock = std::chrono::steady_clock;
    explicit PromptEvents(double fault_repeat_interval_s = 0)
        : fault_repeat_interval_s_(fault_repeat_interval_s) {}
    // First fresh sample seeds the observer: do not replay an action in progress.
    std::vector<std::string> Update(const Status &status, Clock::time_point now = Clock::now()) {
        if (!status.online) return {};
        std::vector<std::string> cues;
        if (!initialized_) {
            initialized_ = true;
            last_fault_ = now;
        } else {
            // Observe the global owner, not this read-only client's owns_control.
            // A handoff announces the new owner once; renewals stay silent.
            if (status.owner != previous_.owner)
                cues.emplace_back(status.owner.empty() ? "control.released" : "control.acquired");
            const bool repeat_fault = fault_repeat_interval_s_ > 0 &&
                std::chrono::duration<double>(now - last_fault_).count() >= fault_repeat_interval_s_;
            if (status.fault.latched && (!previous_.fault.latched ||
                status.fault.sequence != previous_.fault.sequence || repeat_fault)) {
                cues.emplace_back("fault");
                last_fault_ = now;
            } else if (status.state != previous_.state)
                cues.push_back("state." + status.state);
            if (status.state == "RL" && !status.fault.latched &&
                (previous_.state != "RL" || status.policy != previous_.policy))
                cues.push_back("policy." + status.policy);
            if (status.teleop_preset_sequence != previous_.teleop_preset_sequence &&
                !status.teleop_preset.empty())
                cues.push_back("preset." + status.teleop_preset);
            const bool entering = status.interaction_phase == "BLEND_IN" || status.interaction_phase == "PLAYING";
            if (status.state == "RL" && !status.fault.latched && status.interaction_accepted && entering &&
                (status.interaction_action != previous_.interaction_action ||
                    (previous_.interaction_phase != "BLEND_IN" && previous_.interaction_phase != "PLAYING")))
                cues.push_back("action." + status.interaction_action);
        }
        previous_ = status;
        return cues;
    }
    void Reset() { initialized_ = false; }

private:
    bool initialized_ = false;
    double fault_repeat_interval_s_ = 0;
    Clock::time_point last_fault_{};
    Status previous_;
};
} // namespace operator_service
#endif // PROMPT_EVENTS_H
