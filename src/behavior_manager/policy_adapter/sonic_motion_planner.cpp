/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file sonic_motion_planner.cpp
 * @brief 把 SONIC kinematic planner 注册为实时规划器类型 sonic_planner
 */

#include <charconv>
#include <memory>
#include <string_view>
#include <stdexcept>
#include <string>

#include "motion_planner/motion_planner.h"
#include "robot_base.h"
#include "sonic_planner.h"

namespace behavior_manager {
namespace policy_adapter {
namespace {

SonicPlannerConfig LoadConfig(const motion_planner::PlannerContext &context) {
    const robot_base::YamlFile yaml = robot_base::YamlFile::Load(context.yaml_path);
    SonicPlannerConfig config = LoadSonicPlannerConfig(
        yaml, context.config_base, context.robot_dir, context.reference_fps);
    config.thread = context.thread;
    config.on_warmup = context.on_warmup;
    return config;
}

// 速度指令 -> 模型推理 -> 机器人关节顺序的参考轨迹，根位姿与机器人状态同一坐标系。
class SonicMotionPlanner final : public motion_planner::RealtimePlanner {
public:
    explicit SonicMotionPlanner(const motion_planner::PlannerContext &context)
        : planner_(LoadConfig(context)) {}

    void Start(const robot_base::RobotData &robot, double time_s) override {
        planner_.Reset(robot, time_s);
    }

    void Update(const motion_planner::PlannerInput &input) override {
        if (input.argument_sequence != sequence_) {
            sequence_ = input.argument_sequence;
            mode_ = -1;
            height_ = -1;
            if (!input.argument.empty()) {
                const std::string_view value(input.argument);
                const auto split = value.find(':');
                const auto parse = [](std::string_view s) {
                    int n;
                    const auto end = s.data() + s.size();
                    const auto result = std::from_chars(s.data(), end, n);
                    if (result.ec != std::errc{} || result.ptr != end)
                        throw std::invalid_argument("invalid SONIC planner preset");
                    return n;
                };
                mode_ = parse(value.substr(0, split));
                if (mode_ < -1 || mode_ > 26) throw std::invalid_argument("invalid SONIC locomotion mode");
                if (split != value.npos) {
                    const int cm = parse(value.substr(split + 1));
                    if (cm < 20 || cm > 80) throw std::invalid_argument("invalid SONIC target height");
                    height_ = cm * 0.01;
                }
            }
        }
        command_.vx = input.command.vx;
        command_.vy = input.command.vy;
        command_.wz = input.command.wz;
        planner_.Update(input.robot, command_, input.time_s, input.stopping ? 0 : mode_, input.stopping ? -1 : height_);
    }

    std::shared_ptr<const motion_planner::PlannerOutput> Output() override {
        return planner_.Reference();
    }

    bool Idle() const override { return planner_.Idle(); }

    void Stop() override { planner_.Stop(); }

    motion_planner::PlannerStats Stats() const override { return planner_.Stats(); }

private:
    uint64_t sequence_ = 0;
    int mode_ = -1;
    double height_ = -1;
    SonicPlanner planner_;
    robot_base::Command command_;
};

const motion_planner::PlannerTypeRegistration kSonicPlannerType("sonic_planner", {
    motion_planner::Category::REALTIME,
    motion_planner::Kind::MODEL,
    [](const motion_planner::PlannerContext &context) -> std::unique_ptr<motion_planner::Planner> {
        return std::make_unique<SonicMotionPlanner>(context);
    },
});

}  // namespace
}  // namespace policy_adapter
}  // namespace behavior_manager
