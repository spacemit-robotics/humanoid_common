/**
 * SPDX-License-Identifier: Apache-2.0
 * Timed signed-direction requests, bounded FIFO and finite playback scheduling.
 */
#include "BT_action_planner.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>
#include <stdexcept>
#include <utility>

namespace behavior_manager {
namespace policy_adapter {
namespace {

bool ValidId(const std::string &id) {
    return !id.empty() && id.find_first_of("#\r\n\t") == std::string::npos;
}

// A unique suffix changes Common's existing argument mailbox value. It is not
// a reliable event transport: updates overwritten between ticks are still lost.
std::string ActionId(const std::string &text) {
    const auto marker = text.find('#');
    if (marker == std::string::npos) return text;
    const auto suffix = text.substr(marker + 1);
    if (suffix.empty() || !std::all_of(suffix.begin(), suffix.end(),
            [](unsigned char ch) { return ch >= '0' && ch <= '9'; }))
        throw std::invalid_argument("action request suffix must be decimal digits");
    return text.substr(0, marker);
}

}  // namespace

CodeDrivenActionPlanner::CodeDrivenActionPlanner(ActionPlannerConfig config)
    : config_(std::move(config)) {
    if (config_.modes.empty() || config_.initial_mode >= config_.modes.size() ||
        !std::isfinite(config_.threshold) || config_.threshold < 0 || config_.threshold >= 1 ||
        !std::isfinite(config_.period_s) || config_.period_s <= 0 ||
        config_.capacity == 0 || config_.capacity > 4096)
        throw std::invalid_argument("invalid action planner modes/timing/capacity");
    std::set<std::string> modes;
    std::set<std::string> mapped;
    for (const auto &mode : config_.modes) {
        if (!ValidId(mode.id) || mode.id == "next" || mode.id == "previous" ||
            mode.id.find_first_of(".[]") != std::string::npos ||
            !modes.insert(mode.id).second || !ValidId(mode.neutral))
            throw std::invalid_argument("invalid or duplicate mode/neutral ID");
        mapped.insert(mode.neutral);
        for (const auto &action : mode.directions) {
            if (!action.empty() && !ValidId(action))
                throw std::invalid_argument("invalid directional action ID");
            if (!action.empty()) mapped.insert(action);
        }
    }
    std::set<std::string> once;
    for (const auto &action : config_.once_actions)
        if (!ValidId(action) || !once.insert(action).second || mapped.count(action))
            throw std::invalid_argument("invalid, duplicate or ambiguous once action");
    Reset(0);
}

void CodeDrivenActionPlanner::Reset(double time_s) {
    if (!std::isfinite(time_s)) throw std::invalid_argument("invalid start time");
    mode_ = config_.initial_mode;
    pending_.clear();
    current_.clear();
    output_.reset();
    neutral_ = acknowledged_ = stopped_ = false;
    held_direction_ = -1;
    next_due_s_ = last_time_s_ = time_s;
    last_sequence_ = rejected_ = plan_count_ = 0;
}

bool CodeDrivenActionPlanner::Idle() const {
    return current_.empty() && pending_.empty() && held_direction_ < 0;
}

void CodeDrivenActionPlanner::Enqueue(const std::string &action, double count) {
    const std::size_t room = config_.capacity - pending_.size();
    const std::size_t accepted = count >= static_cast<double>(room)
        ? room : static_cast<std::size_t>(count);
    for (std::size_t i = 0; i < accepted; ++i) pending_.push_back(action);
    const double rejected = count - static_cast<double>(accepted);
    const auto remaining = std::numeric_limits<std::uint64_t>::max() - rejected_;
    rejected_ += rejected >= static_cast<double>(remaining) ? remaining
        : static_cast<std::uint64_t>(rejected);
}

void CodeDrivenActionPlanner::Argument(const std::string &argument) {
    if (argument.empty() || stopped_) return;
    if (argument == "stop") {
        stopped_ = true;
        pending_.clear();
        held_direction_ = -1;
        return;
    }
    if (argument.compare(0, 5, "mode:") == 0) {
        const std::string id = argument.substr(5);
        std::size_t next = mode_;
        if (id == "next") next = (mode_ + 1) % config_.modes.size();
        else if (id == "previous") next = (mode_ + config_.modes.size() - 1) % config_.modes.size();
        else {
            const auto found = std::find_if(config_.modes.begin(), config_.modes.end(),
                [&](const ActionPlannerMode &mode) { return mode.id == id; });
            if (found == config_.modes.end()) throw std::invalid_argument("unknown action mode");
            next = static_cast<std::size_t>(found - config_.modes.begin());
        }
        if (next != mode_) held_direction_ = -1;  // new mode earns a full interval
        mode_ = next;
        return;
    }
    if (argument.compare(0, 7, "action:") == 0) {
        const std::string action = ActionId(argument.substr(7));
        if (std::find(config_.once_actions.begin(), config_.once_actions.end(), action) ==
                config_.once_actions.end())
            throw std::invalid_argument("one-shot action not whitelisted");
        Enqueue(action);
        return;
    }
    throw std::invalid_argument("expected mode:<id>, action:<id>[#digits], or stop");
}

void CodeDrivenActionPlanner::Accumulate(const motion_planner::PlannerInput &input) {
    const std::array<float, 6> values{{input.command.vx, -input.command.vx,
        input.command.vy, -input.command.vy, input.command.wz, -input.command.wz}};
    const auto strongest = std::max_element(values.begin(), values.end());
    int selected = *strongest > config_.threshold ? static_cast<int>(strongest - values.begin()) : -1;
    if (selected >= 0 && config_.modes[mode_].directions[static_cast<std::size_t>(selected)].empty())
        selected = -1;  // a missing reference is never replaced with another motion
    if (selected < 0 || selected != held_direction_) {
        held_direction_ = selected;
        next_due_s_ = input.time_s + config_.period_s;
        if (next_due_s_ <= input.time_s)
            next_due_s_ = std::nextafter(input.time_s, std::numeric_limits<double>::infinity());
        return;
    }
    if (input.time_s < next_due_s_) return;
    const double elapsed = input.time_s - next_due_s_;
    const double periods = std::floor(elapsed / config_.period_s) + 1.0;
    Enqueue(config_.modes[mode_].directions[static_cast<std::size_t>(selected)], periods);
    // Advance from the previous deadline, not from a potentially late tick.
    next_due_s_ += periods * config_.period_s;
    // A period below the clock's precision must not retrigger at the same time.
    if (next_due_s_ <= input.time_s)
        next_due_s_ = std::nextafter(input.time_s, std::numeric_limits<double>::infinity());
}

void CodeDrivenActionPlanner::Publish(const std::string &action, bool neutral) {
    auto next = std::make_shared<motion_planner::PlannerOutput>();
    next->action = action;
    current_ = action;
    neutral_ = neutral;
    acknowledged_ = false;
    output_ = std::move(next);
    if (plan_count_ != std::numeric_limits<std::uint64_t>::max()) ++plan_count_;
}

void CodeDrivenActionPlanner::Update(const motion_planner::PlannerInput &input) {
    for (const float axis : {input.command.vx, input.command.vy, input.command.wz})
        if (!std::isfinite(axis) || axis < -1 || axis > 1)
            throw std::invalid_argument("selector requires normalized finite axes in [-1,1]");
    if (!std::isfinite(input.time_s) || input.time_s < last_time_s_ ||
        input.argument_sequence < last_sequence_ || !std::isfinite(input.playback.progress) ||
        input.playback.progress < 0 || input.playback.progress > 1)
        throw std::invalid_argument("invalid/regressed time, argument sequence or playback progress");
    last_time_s_ = input.time_s;
    if (input.argument_sequence != last_sequence_) {
        Argument(input.argument);
        last_sequence_ = input.argument_sequence;
    }
    if (!stopped_) Accumulate(input);

    bool completed_action = false;
    if (!current_.empty()) {
        if (input.playback.active && !input.playback.finished) acknowledged_ = true;
        if (!acknowledged_ || !input.playback.active || !input.playback.finished) return;
        completed_action = !neutral_;
        current_.clear();
        acknowledged_ = false;
    }
    if (!pending_.empty()) {
        const std::string action = std::move(pending_.front());
        pending_.pop_front();
        Publish(action, false);
    } else if (completed_action) {
        Publish(config_.modes[mode_].neutral, true);
    }
    // Keep the identical final neutral publication. PR38 Host ignores nullptr
    // after a publication; only its existing STOP lifecycle releases ownership.
}

}  // namespace policy_adapter
}  // namespace behavior_manager
