/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file fake_planners.h
 * @brief Scripted planner types for tests: realtime trajectory, realtime action choice and offline generation
 */

#ifndef FAKE_PLANNERS_H
#define FAKE_PLANNERS_H

#include <atomic>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>

#include "motion_planner/motion_planner.h"

namespace fake_planners {

using behavior_manager::motion_planner::Category;
using behavior_manager::motion_planner::GenerationRequest;
using behavior_manager::motion_planner::Kind;
using behavior_manager::motion_planner::MotionCommand;
using behavior_manager::motion_planner::OfflinePlanner;
using behavior_manager::motion_planner::Planner;
using behavior_manager::motion_planner::PlannerContext;
using behavior_manager::motion_planner::PlannerInput;
using behavior_manager::motion_planner::PlannerOutput;
using behavior_manager::motion_planner::PlannerStats;
using behavior_manager::motion_planner::PlannerTypeRegistration;
using behavior_manager::motion_planner::RealtimePlanner;
using behavior_manager::motion_planner::ReferenceTrajectory;

// Knobs and counters shared by every fake planner; a test resets them first.
struct Script {
    std::atomic<int> constructed{0};
    std::atomic<int> destroyed{0};
    std::atomic<int> started{0};
    std::atomic<int> stopped{0};
    std::atomic<int> generated{0};
    std::atomic<int> load_delay_ms{0};
    std::atomic<int> generate_delay_ms{0};
    std::atomic<bool> fail_load{false};
    std::atomic<bool> fail_start{false};
    std::atomic<bool> fail_update{false};
    std::atomic<bool> fail_generate{false};
    std::atomic<bool> invalid_output{false};
    std::atomic<bool> settled{true};      // Lets a realtime planner report Idle() on a zero command.
    std::atomic<bool> saw_cancel{false};  // An offline generation observed its cancel flag.
    std::atomic<float> generate_progress{0.0F};  // Reported by an offline generation when not zero.
    std::atomic<float> last_vx{0.0F};
    std::atomic<uint64_t> argument_sequence{0};
    std::mutex mutex;
    std::string argument;     // Guarded by mutex.
    std::string thread_name;  // Guarded by mutex: name of the thread that loaded the planner.

    void Reset() {
        constructed = destroyed = started = stopped = generated = 0;
        load_delay_ms = generate_delay_ms = 0;
        fail_load = fail_start = fail_update = fail_generate = invalid_output = false;
        settled = true;
        saw_cancel = false;
        generate_progress = 0.0F;
        last_vx = 0.0F;
        argument_sequence = 0;
        std::lock_guard<std::mutex> lock(mutex);
        argument.clear();
        thread_name.clear();
    }
};

inline Script &GetScript() {
    static Script script;
    return script;
}

// `frames` frames at 50 fps; joint j holds value + 0.001 * j, the root stays upright at the origin.
inline PlannerOutput ConstantTrajectory(int num_dof, int frames, float value, double start_time_s) {
    PlannerOutput output;
    auto &trajectory = output.trajectory;
    trajectory.num_dof = num_dof;
    trajectory.fps = 50.0;
    trajectory.start_time_s = start_time_s;
    trajectory.joint_pos.resize(static_cast<std::size_t>(frames) * num_dof);
    for (std::size_t index = 0; index < trajectory.joint_pos.size(); ++index) {
        trajectory.joint_pos[index] = value + 0.001F * static_cast<float>(index % num_dof);
    }
    trajectory.joint_vel.assign(static_cast<std::size_t>(frames) * num_dof, 0.0F);
    trajectory.root_pos.assign(static_cast<std::size_t>(frames) * 3, 0.0F);
    trajectory.root_quat.assign(static_cast<std::size_t>(frames) * 4, 0.0F);
    for (int frame = 0; frame < frames; ++frame) trajectory.root_quat[frame * 4] = 1.0F;
    if (GetScript().invalid_output) trajectory.joint_pos[0] = std::numeric_limits<float>::quiet_NaN();
    return output;
}

class Loaded {
public:
    explicit Loaded(const PlannerContext &context) {
        Script &script = GetScript();
        if (context.on_warmup) context.on_warmup();
        std::this_thread::sleep_for(std::chrono::milliseconds(script.load_delay_ms.load()));
        if (script.fail_load) throw std::runtime_error("scripted load failure");
        ++script.constructed;
    }
    ~Loaded() { ++GetScript().destroyed; }
};

// Model-style realtime planner: a trajectory whose joint positions follow 0.5 + vx.
class TrajectoryPlanner final : public RealtimePlanner {
public:
    explicit TrajectoryPlanner(const PlannerContext &context)
        : loaded_(context), num_dof_(context.num_dof) {}

    void Start(const robot_base::RobotData &, double time_s) override {
        Script &script = GetScript();
        if (script.fail_start) throw std::runtime_error("scripted start failure");
        ++script.started;
        vx_ = 0.0F;
        plans_ = 0;
        Publish(time_s);
    }

    void Update(const PlannerInput &input) override {
        Script &script = GetScript();
        if (script.fail_update) throw std::runtime_error("scripted update failure");
        script.last_vx = input.command.vx;
        if (input.argument_sequence != script.argument_sequence) {
            script.argument_sequence = input.argument_sequence;
            std::lock_guard<std::mutex> lock(script.mutex);
            script.argument = input.argument;
        }
        if (input.command.vx != vx_) {
            vx_ = input.command.vx;
            Publish(input.time_s);
        }
    }

    std::shared_ptr<const PlannerOutput> Output() override { return output_; }
    bool Idle() const override { return vx_ == 0.0F && GetScript().settled; }
    void Stop() override {
        ++GetScript().stopped;
        output_.reset();
    }
    PlannerStats Stats() const override { return {plans_, 1.5}; }

private:
    void Publish(double time_s) {
        output_ = std::make_shared<PlannerOutput>(ConstantTrajectory(num_dof_, 100, 0.5F + vx_, time_s));
        ++plans_;
    }

    Loaded loaded_;
    int num_dof_;
    float vx_ = 0.0F;
    uint64_t plans_ = 0;
    std::shared_ptr<const PlannerOutput> output_;
};

// Decision-style realtime planner: plays catalog action "gesture" while vx > 0 and
// restarts it whenever it finishes; on a zero command it lets the action finish and
// then selects "idle".
class ChoicePlanner final : public RealtimePlanner {
public:
    explicit ChoicePlanner(const PlannerContext &context) : loaded_(context) {}

    void Start(const robot_base::RobotData &, double) override {
        ++GetScript().started;
        current_.clear();
        output_.reset();
    }

    void Update(const PlannerInput &input) override {
        const bool finished = input.playback.active && input.playback.finished;
        if (input.command.vx > 0.0F) {
            if (current_ != "gesture" || finished) Select("gesture");
        } else if (current_ == "gesture" && finished) {
            Select("idle");
        } else if (current_.empty()) {
            Select(input.argument.empty() ? "idle" : input.argument);
        }
        settled_ = input.command.vx == 0.0F && current_ != "gesture";
    }

    std::shared_ptr<const PlannerOutput> Output() override { return output_; }
    bool Idle() const override { return settled_; }
    void Stop() override { ++GetScript().stopped; }

private:
    void Select(const std::string &action) {
        current_ = action;
        auto output = std::make_shared<PlannerOutput>();
        output->action = action;
        output_ = std::move(output);
    }

    Loaded loaded_;
    std::string current_;
    bool settled_ = false;
    std::shared_ptr<const PlannerOutput> output_;
};

// Offline planner: the argument "action:<name>" selects a catalog action, anything
// else produces a 26-frame (0.5 s) trajectory holding 0.7.
class GeneratorPlanner final : public OfflinePlanner {
public:
    explicit GeneratorPlanner(const PlannerContext &context)
        : loaded_(context), num_dof_(context.num_dof) {}

    PlannerOutput Generate(const GenerationRequest &request) override {
        Script &script = GetScript();
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(script.generate_delay_ms.load());
        if (script.generate_progress != 0.0F) request.report_progress(script.generate_progress);
        while (std::chrono::steady_clock::now() < deadline) {
            if (request.cancelled.load()) {
                script.saw_cancel = true;
                throw std::runtime_error("generation cancelled");
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (script.fail_generate) throw std::runtime_error("scripted generation failure");
        ++script.generated;
        {
            std::lock_guard<std::mutex> lock(script.mutex);
            script.argument = request.argument;
        }
        if (request.argument.rfind("action:", 0) == 0) {
            PlannerOutput output;
            output.action = request.argument.substr(7);
            return output;
        }
        return ConstantTrajectory(num_dof_, 26, 0.7F, 0.0);
    }

private:
    Loaded loaded_;
    int num_dof_;
};

// Registers the fake types once per process; call from main() before using them.
inline void Register() {
    using behavior_manager::motion_planner::RegisterPlannerType;
    static const bool registered = [] {
        RegisterPlannerType("fake_trajectory", {Category::REALTIME, Kind::MODEL,
            [](const PlannerContext &context) -> std::unique_ptr<Planner> {
                return std::make_unique<TrajectoryPlanner>(context);
            }});
        RegisterPlannerType("fake_choice", {Category::REALTIME, Kind::BEHAVIOR_TREE,
            [](const PlannerContext &context) -> std::unique_ptr<Planner> {
                return std::make_unique<ChoicePlanner>(context);
            }});
        RegisterPlannerType("fake_generator", {Category::OFFLINE, Kind::MODEL,
            [](const PlannerContext &context) -> std::unique_ptr<Planner> {
                return std::make_unique<GeneratorPlanner>(context);
            }});
        return true;
    }();
    static_cast<void>(registered);
}

}  // namespace fake_planners

#endif  // FAKE_PLANNERS_H
