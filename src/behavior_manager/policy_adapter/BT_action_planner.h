/**
 * SPDX-License-Identifier: Apache-2.0
 * Robot-independent finite-action selector; no inference or reference loading.
 */
#ifndef BT_ACTION_PLANNER_H
#define BT_ACTION_PLANNER_H

#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "motion_planner/motion_planner.h"

namespace behavior_manager {
namespace policy_adapter {

struct ActionPlannerMode {
    std::string id;
    // forward, backward, left, right, turn_left, turn_right. Empty = unavailable.
    std::array<std::string, 6> directions;
    std::string neutral;
};

struct ActionPlannerConfig {
    std::vector<ActionPlannerMode> modes;
    std::size_t initial_mode = 0;
    std::vector<std::string> once_actions;
    float threshold = 0.5F;
    double period_s = 1.5;
    std::size_t capacity = 32;
};

// Owning-thread only. Common's existing PlannerInput is consumed without changes.
// Queue entries snapshot logical action IDs; assets belong to robot packages.
class CodeDrivenActionPlanner {
public:
    explicit CodeDrivenActionPlanner(ActionPlannerConfig config);
    void Reset(double time_s);
    void Update(const motion_planner::PlannerInput &input);
    std::shared_ptr<const motion_planner::PlannerOutput> Output() const { return output_; }
    bool Idle() const;
    std::size_t PendingActions() const { return pending_.size(); }
    std::uint64_t RejectedActions() const { return rejected_; }
    std::uint64_t PlanCount() const { return plan_count_; }

private:
    void Argument(const std::string &argument);
    void Accumulate(const motion_planner::PlannerInput &input);
    void Enqueue(const std::string &action, double count = 1.0);
    void Publish(const std::string &action, bool neutral);

    ActionPlannerConfig config_;
    std::size_t mode_ = 0;
    std::deque<std::string> pending_;
    std::string current_;
    std::shared_ptr<const motion_planner::PlannerOutput> output_;
    bool neutral_ = false;
    bool acknowledged_ = false;
    bool stopped_ = false;
    int held_direction_ = -1;
    double next_due_s_ = 0.0;
    double last_time_s_ = 0.0;
    std::uint64_t last_sequence_ = 0;
    std::uint64_t rejected_ = 0;
    std::uint64_t plan_count_ = 0;
};

}  // namespace policy_adapter
}  // namespace behavior_manager
#endif  // BT_ACTION_PLANNER_H
