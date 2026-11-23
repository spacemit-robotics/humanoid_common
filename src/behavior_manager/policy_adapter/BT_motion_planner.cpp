/**
 * SPDX-License-Identifier: Apache-2.0
 * Register the code-driven selector through unchanged Common PR38 interfaces.
 * This is distinct from sonic_motion_planner.cpp and its model inference.
 */
#include "BT_action_planner.h"

#include <algorithm>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "robot_base.h"

namespace behavior_manager {
namespace policy_adapter {
namespace {

template <typename T>
T Required(const robot_base::YamlFile &yaml, const std::string &path) {
    const auto value = yaml.Read<T>(path);
    if (!value) throw std::invalid_argument("missing or invalid selector field: " + path);
    return *value;
}

template <typename T>
T Optional(const robot_base::YamlFile &yaml, const std::string &path, T fallback) {
    return yaml.Contains(path) ? Required<T>(yaml, path) : std::move(fallback);
}

ActionPlannerConfig LoadConfig(const motion_planner::PlannerContext &context) {
    const auto yaml = robot_base::YamlFile::Load(context.yaml_path);
    const std::string base = context.config_base + ".";
    // The channel is normally physical velocity. This selector requires an
    // explicit, isolated unit-gain profile, never a model planner's profile.
    for (const std::string axis : {"vx", "vy", "wz"})
        if (Required<double>(yaml, base + "command.limits.min_" + axis) != -1 ||
            Required<double>(yaml, base + "command.limits.max_" + axis) != 1)
            throw std::invalid_argument("selector requires explicit [-1,1] command limits");
    for (const std::string preset : {"slow_limits", "fast_limits"})
        if (Required<std::vector<double>>(yaml, base + "command.teleop." + preset) !=
                std::vector<double>{1, 1, 1})
            throw std::invalid_argument("selector requires unit-gain slow/fast presets");
    if (Required<bool>(yaml, base + "command.smoothing") ||
        Required<double>(yaml, "operator_service.teleop.deadzone") != 0)
        throw std::invalid_argument("selector requires smoothing=false and teleop.deadzone=0");

    ActionPlannerConfig config;
    const auto ids = Required<std::vector<std::string>>(yaml, base + "mode_ids");
    const auto initial = Required<std::string>(yaml, base + "initial_mode");
    const auto first = std::find(ids.begin(), ids.end(), initial);
    if (first == ids.end()) throw std::invalid_argument("initial_mode not in mode_ids");
    config.initial_mode = static_cast<std::size_t>(first - ids.begin());
    for (const auto &id : ids) {
        if (id.empty() || id.find_first_of(".[]") != std::string::npos)
            throw std::invalid_argument("invalid mode ID for YAML path");
        ActionPlannerMode mode;
        mode.id = id;
        const std::string prefix = base + "modes." + id + ".";
        mode.neutral = Required<std::string>(yaml, prefix + "neutral");
        std::size_t i = 0;
        for (const std::string direction : {"forward", "backward", "left", "right", "turn_left", "turn_right"})
            mode.directions[i++] = Required<std::string>(yaml, prefix + direction);
        config.modes.push_back(std::move(mode));
    }
    config.once_actions = Optional<std::vector<std::string>>(yaml, base + "once_actions", {});
    config.threshold = static_cast<float>(Optional<double>(yaml, base + "joystick_threshold", 0.5));
    config.period_s = Optional<double>(yaml, base + "action_period_s", 1.5);
    const int capacity = Optional<int>(yaml, base + "max_pending_actions", 32);
    if (capacity < 1 || capacity > 4096) throw std::invalid_argument("invalid pending capacity");
    config.capacity = static_cast<std::size_t>(capacity);
    return config;
}

class CodeDrivenMotionPlanner final : public motion_planner::RealtimePlanner {
public:
    explicit CodeDrivenMotionPlanner(const motion_planner::PlannerContext &context)
        : planner_(LoadConfig(context)) {}

    void Start(const robot_base::RobotData &, double time_s) override {
        planner_.Reset(time_s);
        last_plan_ms_ = 0;
    }
    void Update(const motion_planner::PlannerInput &input) override {
        const auto start = std::chrono::steady_clock::now();
        const auto previous = planner_.PlanCount();
        planner_.Update(input);
        if (planner_.PlanCount() != previous)
            last_plan_ms_ = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
    }
    std::shared_ptr<const motion_planner::PlannerOutput> Output() override {
        return planner_.Output();
    }
    bool Idle() const override { return planner_.Idle(); }
    motion_planner::PlannerStats Stats() const override {
        return {planner_.PlanCount(), last_plan_ms_};
    }

private:
    CodeDrivenActionPlanner planner_;
    double last_plan_ms_ = 0;
};

const motion_planner::PlannerTypeRegistration kCodeDrivenActionPlannerTypeRegistration(
    "code_driven_action_planner", {
        motion_planner::Category::REALTIME,
        motion_planner::Kind::BEHAVIOR_TREE,
        [](const motion_planner::PlannerContext &context) -> std::unique_ptr<motion_planner::Planner> {
            return std::make_unique<CodeDrivenMotionPlanner>(context);
        },
    });

}  // namespace
}  // namespace policy_adapter
}  // namespace behavior_manager
