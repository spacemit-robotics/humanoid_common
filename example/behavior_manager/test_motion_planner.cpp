/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_motion_planner.cpp
 * @brief 规划器类型注册表、输出校验、规划器宿主状态机及其与跟踪策略衔接的离线测试
 */

#include "fake_planners.h"
#include "motion_planner/motion_planner.h"
#include "motion_planner/planner_binding.h"
#include "motion_planner/planner_host.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

using behavior_manager::motion_planner::Category;
using behavior_manager::motion_planner::FindPlannerType;
using behavior_manager::motion_planner::HostConfig;
using behavior_manager::motion_planner::Kind;
using behavior_manager::motion_planner::MotionCommand;
using behavior_manager::motion_planner::PlannerConfig;
using behavior_manager::motion_planner::PlannerHost;
using behavior_manager::motion_planner::PlannerOutput;
using behavior_manager::motion_planner::RegisteredPlannerTypes;
using behavior_manager::motion_planner::RegisterPlannerType;
using behavior_manager::motion_planner::TrackerState;
using behavior_manager::motion_planner::ValidateOutput;
using fake_planners::GetScript;
using Phase = robot_base::PlannerStatus::Phase;
using Operation = robot_base::PlannerRequest::Operation;

constexpr int kDof = 3;

void Require(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

bool Contains(const std::string &text, const std::string &part) {
    return text.find(part) != std::string::npos;
}

PlannerConfig MakePlanner(const std::string &name, const std::string &type,
    Category category = Category::REALTIME) {
    PlannerConfig config;
    config.name = name;
    config.type = type;
    config.category = category;
    config.minimum_command = {-1.0F, -1.0F, -1.0F};
    config.maximum_command = {1.0F, 1.0F, 1.0F};
    config.context.name = name;
    config.context.config_base = "motion_planners." + name;
    config.context.num_dof = kDof;
    return config;
}

HostConfig MakeHost(std::vector<PlannerConfig> planners, const std::string &auto_start = "") {
    HostConfig config;
    config.planners = std::move(planners);
    config.auto_start = auto_start;
    return config;
}

// Drives a host the way a tracker's inference thread does: one Step per 20 ms of RL time.
class Tracker {
public:
    explicit Tracker(HostConfig config) : host(std::move(config)) {
        robot = robot_base::RobotData::Create(kDof);
        robot.environment = robot_base::RuntimeEnvironment::SIMULATION;
    }

    void Step(int count = 1) {
        for (int index = 0; index < count; ++index) {
            const auto start = std::chrono::steady_clock::now();
            host.Step(robot, command, time_s, request, state);
            worst_step_ms = std::max(worst_step_ms, std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count());
            time_s += 0.02;
        }
    }

    // Steps until the condition holds; background jobs finish in real time.
    bool Until(const std::function<bool()> &done, int timeout_ms = 3000) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (true) {
            Step();
            if (done()) return true;
            if (std::chrono::steady_clock::now() >= deadline) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    bool UntilPhase(Phase phase, int timeout_ms = 3000) {
        return Until([&] { return host.Status().phase == phase; }, timeout_ms);
    }

    void Start(const std::string &planner, const std::string &argument = "") {
        ++request.sequence;
        request.operation = Operation::START;
        request.planner = planner;
        request.argument = argument;
    }

    void Stop() {
        ++request.sequence;
        request.operation = Operation::STOP;
        request.planner.clear();
        request.argument.clear();
    }

    Phase phase() const { return host.Status().phase; }
    const robot_base::PlannerStatus &status() const { return host.Status(); }

    PlannerHost host;
    robot_base::RobotData robot;
    MotionCommand command;
    robot_base::PlannerRequest request;
    TrackerState state;
    double time_s = 0.0;
    double worst_step_ms = 0.0;
};

void TestRegistry() {
    const auto *sonic = FindPlannerType("sonic_planner");
    Require(sonic && sonic->category == Category::REALTIME && sonic->kind == Kind::MODEL && sonic->factory,
        "the library must register its SONIC planner type when it loads");
    const auto *choice = FindPlannerType("fake_choice");
    Require(choice && choice->kind == Kind::BEHAVIOR_TREE &&
        FindPlannerType("fake_generator")->category == Category::OFFLINE,
        "types registered by another translation unit must be found by name");
    Require(!FindPlannerType("absent"), "an unregistered type must not be found");
    const auto names = RegisteredPlannerTypes();
    Require(std::is_sorted(names.begin(), names.end()) &&
        std::count(names.begin(), names.end(), "sonic_planner") == 1 &&
        std::count(names.begin(), names.end(), "fake_trajectory") == 1,
        "registered type names must be listed once, in order");
    const auto rejected = [](const std::string &type, bool with_factory) {
        behavior_manager::motion_planner::PlannerType entry;
        if (with_factory) entry.factory = FindPlannerType("fake_trajectory")->factory;
        try {
            RegisterPlannerType(type, entry);
        } catch (const std::invalid_argument &) {
            return true;
        }
        return false;
    };
    Require(rejected("fake_trajectory", true), "a duplicate type name must be rejected");
    Require(rejected("", true) && rejected("no_factory", false),
        "a type needs a name and a factory");
    std::cout << "planner type registry: PASS" << std::endl;
}

void TestValidateOutput() {
    const auto valid = fake_planners::ConstantTrajectory(kDof, 4, 0.1F, 1.0);
    Require(ValidateOutput(valid, kDof).empty(), "a well-formed trajectory must pass");
    Require(!ValidateOutput(valid, kDof + 1).empty(), "a trajectory for another joint count must fail");
    auto output = valid;
    output.trajectory.fps = 0.0;
    Require(!ValidateOutput(output, kDof).empty(), "a non-positive frame rate must fail");
    output = valid;
    output.trajectory.joint_vel.pop_back();
    Require(!ValidateOutput(output, kDof).empty(), "arrays with different frame counts must fail");
    output = valid;
    output.trajectory.root_pos[2] = std::numeric_limits<float>::infinity();
    Require(!ValidateOutput(output, kDof).empty(), "a non-finite value must fail");
    output = valid;
    output.trajectory.root_quat[4] = 0.0F;
    Require(!ValidateOutput(output, kDof).empty(), "a zero quaternion must fail");
    Require(!ValidateOutput(fake_planners::ConstantTrajectory(kDof, 1, 0.1F, 0.0), kDof).empty(),
        "a single frame cannot be interpolated");
    PlannerOutput action;
    action.action = "wave";
    Require(ValidateOutput(action, kDof).empty(), "an action name needs no trajectory");
    action.trajectory = valid.trajectory;
    Require(!ValidateOutput(action, kDof).empty(), "an output is either an action or a trajectory");
    Require(!ValidateOutput(PlannerOutput{}, kDof).empty(), "an empty output must fail");
    std::cout << "planner output validation: PASS" << std::endl;
}

void TestRealtimeLifecycle() {
    auto &script = GetScript();
    script.Reset();
    {
        Tracker tracker(MakeHost({MakePlanner("walker", "fake_trajectory"),
            MakePlanner("chooser", "fake_choice")}));
        Require(tracker.phase() == Phase::IDLE && !tracker.host.Engaged() && !tracker.host.Output() &&
            script.constructed == 0, "nothing loads before the first start request");
        tracker.Step(3);
        Require(tracker.phase() == Phase::IDLE, "an idle host must stay idle");

        tracker.Start("walker", "first");
        tracker.Step();
        Require(tracker.status().request_accepted && tracker.status().sequence == tracker.request.sequence &&
            tracker.status().planner == "walker" && tracker.host.Engaged() && !tracker.host.Output() &&
            (tracker.phase() == Phase::LOADING || tracker.phase() == Phase::WARMING),
            "an accepted start must load in the background while the tracker keeps its standby");
        Require(tracker.UntilPhase(Phase::RUNNING), "the planner did not start");
        Require(script.constructed == 1 && script.started == 1 && tracker.host.Output() &&
            !tracker.host.OutputFinite() && tracker.host.Engaged(),
            "a running realtime planner must publish a continuous reference");
        const auto first = tracker.host.Output();
        tracker.Step();
        Require(tracker.host.Output() == first, "an unchanged publication must keep its identity");
        Require(tracker.status().plan_count == 1 && std::abs(tracker.status().last_plan_ms - 1.5F) < 1.0e-6F &&
            tracker.status().reference_remaining_s > 1.0F, "planner statistics must reach the status");

        tracker.command.vx = 0.4F;
        Require(tracker.Until([&] { return tracker.host.Output() != first; }) && script.last_vx == 0.4F &&
            tracker.host.Output()->trajectory.joint_pos[0] == 0.9F,
            "the motion command must reach the planner and its new plan must be published");
        {
            std::lock_guard<std::mutex> lock(script.mutex);
            Require(script.argument == "first", "the start argument must reach the planner");
        }
        // The same planner started again only receives the new argument.
        tracker.Start("walker", "second");
        tracker.Step();
        {
            std::lock_guard<std::mutex> lock(script.mutex);
            Require(tracker.status().request_accepted && tracker.phase() == Phase::RUNNING &&
                script.started == 1 && script.argument == "second",
                "restarting the running planner must only update its argument");
        }
        tracker.Start("chooser");
        tracker.Step();
        Require(!tracker.status().request_accepted && Contains(tracker.status().detail, "stop it first") &&
            tracker.phase() == Phase::RUNNING && tracker.status().planner == "walker",
            "another planner must be refused while one holds the reference");
        const uint64_t handled = tracker.status().sequence;
        tracker.request.sequence = 1;  // A stale request left in the command packet.
        tracker.request.operation = Operation::STOP;
        tracker.Step();
        Require(tracker.phase() == Phase::RUNNING && tracker.status().sequence == handled,
            "a request with an old sequence must be ignored");
        tracker.request.sequence = handled;

        // Stopping feeds a zero command until the planner has settled.
        script.settled = false;
        tracker.Stop();
        tracker.Step(5);
        Require(tracker.status().request_accepted && tracker.phase() == Phase::STOPPING &&
            tracker.host.Engaged() && tracker.host.Output() && script.last_vx == 0.0F &&
            tracker.command.vx == 0.4F, "a stopping planner must be driven to rest before release");
        script.settled = true;
        Require(tracker.Until([&] { return !tracker.host.Output(); }), "the reference was not released");
        Require(tracker.UntilPhase(Phase::READY) && !tracker.host.Engaged() && script.stopped == 1 &&
            script.destroyed == 0, "a stopped planner stays loaded by default");

        tracker.command.vx = 0.0F;
        tracker.Start("walker");
        Require(tracker.UntilPhase(Phase::RUNNING) && script.constructed == 1 && script.started == 2,
            "a loaded planner must restart without loading again");
        Require(tracker.status().plan_count == 1, "statistics restart with the planner");
        tracker.Stop();
        Require(tracker.UntilPhase(Phase::READY), "the second stop did not finish");
        tracker.Stop();
        tracker.Step();
        Require(tracker.status().request_accepted && tracker.phase() == Phase::READY,
            "stopping without an active planner is accepted and changes nothing");
    }
    Require(script.destroyed == 1, "destroying the host must release its planner");
    std::cout << "realtime planner lifecycle: PASS" << std::endl;
}

void TestUnloadAndSwitch() {
    auto &script = GetScript();
    script.Reset();
    auto walker = MakePlanner("walker", "fake_trajectory");
    walker.keep_loaded = false;
    Tracker tracker(MakeHost({walker, MakePlanner("chooser", "fake_choice")}));
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::RUNNING), "the planner did not start");
    tracker.Stop();
    Require(tracker.UntilPhase(Phase::IDLE) && script.destroyed == 1,
        "keep_loaded: false must unload the planner once it has stopped");

    tracker.Start("chooser");
    Require(tracker.UntilPhase(Phase::RUNNING) && tracker.status().planner == "chooser",
        "the second planner did not start");
    tracker.Stop();
    Require(tracker.UntilPhase(Phase::READY) && script.constructed == 2 && script.destroyed == 1,
        "the second planner stays loaded");
    // Only one planner is loaded at a time; starting another replaces it.
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::RUNNING) && tracker.status().planner == "walker" &&
        script.constructed == 3 && script.destroyed == 2,
        "starting another planner must unload the one that was kept");
    std::cout << "planner unload and replacement: PASS" << std::endl;
}

void TestRejections() {
    auto &script = GetScript();
    script.Reset();
    auto sim_only = MakePlanner("sim_only", "fake_trajectory");
    sim_only.environments.real = false;
    Tracker tracker(MakeHost({MakePlanner("walker", "fake_trajectory"), sim_only}));

    tracker.Start("absent");
    tracker.Step();
    Require(!tracker.status().request_accepted && tracker.status().sequence == tracker.request.sequence &&
        Contains(tracker.status().detail, "not registered") && tracker.phase() == Phase::IDLE,
        "a planner the policy does not list must be refused");
    tracker.robot.environment = robot_base::RuntimeEnvironment::REAL;
    tracker.Start("sim_only");
    tracker.Step();
    Require(!tracker.status().request_accepted && Contains(tracker.status().detail, "real") &&
        tracker.phase() == Phase::IDLE, "a planner disabled in the driver environment must be refused");
    tracker.robot.environment = robot_base::RuntimeEnvironment::UNKNOWN;
    tracker.Start("sim_only");
    tracker.Step();
    Require(!tracker.status().request_accepted,
        "an environment-limited planner must wait until the driver has reported its environment");
    tracker.state.busy = true;
    tracker.Start("walker");
    tracker.Step();
    Require(!tracker.status().request_accepted && Contains(tracker.status().detail, "tracker") &&
        script.constructed == 0, "a start must be refused while the tracker runs an action");
    tracker.Stop();
    tracker.Step();
    Require(tracker.status().request_accepted && tracker.status().detail.empty(),
        "a stop request clears the last refusal");
    tracker.state.busy = false;
    tracker.robot.environment = robot_base::RuntimeEnvironment::SIMULATION;
    tracker.Start("sim_only");
    Require(tracker.UntilPhase(Phase::RUNNING), "the planner must start in its enabled environment");
    std::cout << "planner start refusals: PASS" << std::endl;
}

void TestBackgroundLoad() {
    auto &script = GetScript();
    script.Reset();
    script.load_delay_ms = 300;
    Tracker tracker(MakeHost({MakePlanner("walker", "fake_trajectory")}));
    tracker.Start("walker");
    tracker.worst_step_ms = 0.0;
    bool warming = false;
    Require(tracker.Until([&] {
        warming = warming || tracker.phase() == Phase::WARMING;
        return tracker.phase() == Phase::RUNNING;
    }), "the planner did not finish loading");
    Require(warming, "the warm-up reported by the planner must be visible as a phase");
    Require(tracker.worst_step_ms < 100.0, "the tracker step waited for the load: " +
        std::to_string(tracker.worst_step_ms) + " ms");

    // A stop during the load cancels the start; the loaded planner is kept but never started.
    script.Reset();
    script.load_delay_ms = 200;
    Tracker cancelled(MakeHost({MakePlanner("walker", "fake_trajectory")}));
    cancelled.Start("walker");
    cancelled.Step(2);
    Require(cancelled.phase() == Phase::LOADING || cancelled.phase() == Phase::WARMING,
        "the load must be in flight");
    cancelled.Stop();
    cancelled.Step();
    Require(cancelled.status().request_accepted && !cancelled.host.Engaged(),
        "a stop must cancel a start that is still loading");
    Require(cancelled.UntilPhase(Phase::READY) && script.constructed == 1 && script.started == 0 &&
        !cancelled.host.Output(), "a cancelled start must never publish a reference");

    // RequestRelease is the same cancellation issued by the tracker before a policy switch.
    cancelled.Start("walker");
    Require(cancelled.UntilPhase(Phase::RUNNING), "the kept planner did not start");
    cancelled.host.RequestRelease();
    Require(cancelled.phase() == Phase::STOPPING && cancelled.UntilPhase(Phase::READY) &&
        !cancelled.host.Output(), "a release request must return the reference without an operator request");
    std::cout << "planner background load and cancellation: PASS" << std::endl;
}

void TestFailures() {
    auto &script = GetScript();
    script.Reset();
    Tracker tracker(MakeHost({MakePlanner("walker", "fake_trajectory")}));

    script.fail_load = true;
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::FAILED) && Contains(tracker.status().detail, "failed to load") &&
        Contains(tracker.status().detail, "scripted load failure") && !tracker.host.Engaged() &&
        !tracker.host.Output() && tracker.status().request_accepted,
        "a load failure must be reported without taking the reference");
    tracker.Stop();
    tracker.Step();
    Require(tracker.phase() == Phase::IDLE && tracker.status().detail.empty(), "a stop clears the failure");

    script.fail_load = false;
    script.fail_start = true;
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::FAILED) && Contains(tracker.status().detail, "failed to start"),
        "a start failure must be reported");
    Require(tracker.Until([&] { return script.destroyed == 1; }), "a planner that failed to start is not reused");

    // A failure does not need to be cleared before the next start.
    script.fail_start = false;
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::RUNNING) && script.constructed == 2, "a start after a failure must reload");
    script.fail_update = true;
    tracker.Step();
    Require(tracker.phase() == Phase::FAILED && Contains(tracker.status().detail, "scripted update failure") &&
        !tracker.host.Output() && !tracker.host.Engaged(),
        "an exception from a running planner must release the reference");
    Require(tracker.Until([&] { return script.destroyed == 2; }), "a failed planner must be unloaded");
    Require(tracker.phase() == Phase::FAILED, "the failure stays visible until the next request");

    script.fail_update = false;
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::RUNNING), "the planner did not restart");
    script.invalid_output = true;
    tracker.command.vx = 0.2F;
    Require(tracker.UntilPhase(Phase::FAILED) && Contains(tracker.status().detail, "invalid reference") &&
        !tracker.host.Output(), "an invalid publication must never reach the tracker");

    script.invalid_output = false;
    tracker.command.vx = 0.0F;
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::RUNNING), "the planner did not restart");
    tracker.host.Fail("tracker cannot follow it");
    Require(tracker.phase() == Phase::FAILED && tracker.status().detail == "tracker cannot follow it" &&
        !tracker.host.Output(), "the tracker must be able to reject a publication");

    // A type that does not implement the interface of its configured category.
    Tracker mismatched(MakeHost({MakePlanner("generator", "fake_trajectory", Category::OFFLINE)}));
    mismatched.Start("generator");
    Require(mismatched.UntilPhase(Phase::FAILED) && Contains(mismatched.status().detail, "does not implement"),
        "a planner of the wrong category must fail to load");
    std::cout << "planner failures: PASS" << std::endl;
}

void TestSupersededStart() {
    for (bool old_start_fails : {false, true}) {
        auto &script = GetScript();
        script.Reset();
        script.hold_start = true;
        script.fail_start = old_start_fails;
        auto first = MakePlanner("first", "fake_trajectory");
        first.preload = true;
        Tracker tracker(MakeHost({first, MakePlanner("second", "fake_choice")}));
        tracker.Start("first");
        tracker.Step();
        tracker.Stop();
        tracker.Step();
        tracker.Start("second", "gesture");
        tracker.Step();
        script.hold_start = false;
        Require(tracker.UntilPhase(Phase::RUNNING) && tracker.host.Output() &&
            tracker.host.Output()->action == "gesture",
            "a cancelled start must neither complete nor fail the replacement request");
    }
}

void TestStopTimeout() {
    auto &script = GetScript();
    script.Reset();
    auto walker = MakePlanner("walker", "fake_trajectory");
    walker.stop_timeout_s = 0.2;
    Tracker tracker(MakeHost({walker}));
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::RUNNING), "the planner did not start");
    script.settled = false;
    tracker.Stop();
    tracker.Step(8);  // 0.16 s of RL time.
    Require(tracker.phase() == Phase::STOPPING && tracker.host.Output(),
        "the planner must be given its stop timeout to settle");
    tracker.Step(4);
    Require(!tracker.host.Output() && Contains(tracker.status().detail, "stop timeout"),
        "a planner that does not settle must be released after the timeout");
    Require(tracker.UntilPhase(Phase::READY), "the forced release did not finish");

    // stop_timeout: 0 waits for the planner without a limit.
    script.Reset();
    walker.stop_timeout_s = 0.0;
    Tracker patient(MakeHost({walker}));
    patient.Start("walker");
    Require(patient.UntilPhase(Phase::RUNNING), "the planner did not start");
    script.settled = false;
    patient.Stop();
    patient.Step(600);  // 12 s of RL time.
    Require(patient.phase() == Phase::STOPPING && patient.host.Output(),
        "without a timeout the host must keep waiting");
    script.settled = true;
    Require(patient.UntilPhase(Phase::READY), "the planner did not stop once settled");
    std::cout << "planner stop timeout: PASS" << std::endl;
}

void TestPreloadAndAutoStart() {
    auto &script = GetScript();
    script.Reset();
    script.load_delay_ms = 50;
    auto walker = MakePlanner("walker", "fake_trajectory");
    walker.preload = true;
    {
        Tracker tracker(MakeHost({walker}));
        Require(script.constructed == 1 && tracker.phase() == Phase::READY && !tracker.host.Engaged(),
            "a preloaded planner must be loaded when the host is constructed");
        std::vector<Phase> phases;
        tracker.Start("walker");
        Require(tracker.Until([&] {
            if (phases.empty() || phases.back() != tracker.phase()) phases.push_back(tracker.phase());
            return tracker.phase() == Phase::RUNNING;
        }), "the preloaded planner did not start");
        Require(script.constructed == 1 &&
            std::count(phases.begin(), phases.end(), Phase::LOADING) == 0,
            "a preloaded planner must start without loading again");
    }
    script.Reset();
    script.fail_load = true;
    bool rejected = false;
    try {
        Tracker tracker(MakeHost({walker}));
    } catch (const std::runtime_error &error) {
        rejected = Contains(error.what(), "walker");
    }
    Require(rejected, "a failed preload must fail the construction and name the planner");

    script.Reset();
    Tracker automatic(MakeHost({MakePlanner("walker", "fake_trajectory")}, "walker"));
    automatic.state.busy = true;
    automatic.Step(3);
    Require(automatic.phase() == Phase::IDLE, "auto start waits until the tracker is free");
    automatic.state.busy = false;
    Require(automatic.UntilPhase(Phase::RUNNING) && automatic.status().sequence == 0,
        "the policy's auto planner must start without an operator request");
    automatic.Stop();
    Require(automatic.UntilPhase(Phase::READY), "the auto-started planner did not stop");
    automatic.Step(10);
    Require(automatic.phase() == Phase::READY, "a stopped auto planner must not restart by itself");
    automatic.host.Reset();
    automatic.request = {};
    automatic.time_s = 0.0;
    Require(automatic.UntilPhase(Phase::RUNNING), "entering the policy again must start the auto planner again");

    script.Reset();
    auto sim_only = MakePlanner("walker", "fake_trajectory");
    sim_only.environments.real = false;
    Tracker disabled(MakeHost({sim_only}, "walker"));
    disabled.robot.environment = robot_base::RuntimeEnvironment::REAL;
    disabled.Step(3);
    Require(disabled.phase() == Phase::IDLE && Contains(disabled.status().detail, "auto-start skipped") &&
        script.constructed == 0, "an auto planner disabled in this environment must be skipped");

    bool invalid = false;
    try {
        Tracker tracker(MakeHost({MakePlanner("walker", "fake_trajectory")}, "absent"));
    } catch (const std::invalid_argument &) {
        invalid = true;
    }
    Require(invalid, "auto_start must name a listed planner");
    invalid = false;
    try {
        Tracker tracker(MakeHost({MakePlanner("walker", "fake_trajectory"),
            MakePlanner("walker", "fake_choice")}));
    } catch (const std::invalid_argument &) {
        invalid = true;
    }
    Require(invalid, "planner names must be unique");
    std::cout << "planner preload and auto start: PASS" << std::endl;
}

void TestChoicePlanner() {
    auto &script = GetScript();
    script.Reset();
    Tracker tracker(MakeHost({MakePlanner("chooser", "fake_choice")}));
    tracker.Start("chooser");
    Require(tracker.Until([&] { return tracker.host.Output() != nullptr; }), "the planner selected nothing");
    Require(tracker.phase() == Phase::RUNNING && tracker.host.Output()->action == "idle" &&
        tracker.host.Output()->trajectory.FrameCount() == 0 && !tracker.host.OutputFinite(),
        "a decision planner publishes the name of a catalog action");
    tracker.state.playback = {true, 1.0F, true};
    tracker.command.vx = 0.3F;
    tracker.Step();
    const auto gesture = tracker.host.Output();
    Require(gesture->action == "gesture", "the command must change the selected action");
    tracker.state.playback = {true, 0.4F, false};
    tracker.Step();
    Require(tracker.host.Output() == gesture && std::abs(tracker.status().progress - 0.4F) < 1.0e-6F &&
        tracker.status().reference_remaining_s == 0.0F,
        "the progress of the selected action must be reported while it plays");
    // Finishing the action while the command persists publishes it again as a new object.
    tracker.state.playback = {true, 1.0F, true};
    tracker.Step();
    Require(tracker.host.Output() != gesture && tracker.host.Output()->action == "gesture",
        "a repeated selection must be a new publication so that the tracker replays it");

    // A stop lets the current action finish before the reference is returned.
    tracker.state.playback = {true, 0.2F, false};
    tracker.Stop();
    tracker.Step(5);
    Require(tracker.phase() == Phase::STOPPING && tracker.host.Output(),
        "the planner must be allowed to finish its action");
    tracker.state.playback = {true, 1.0F, true};
    Require(tracker.Until([&] { return !tracker.host.Output(); }) && tracker.UntilPhase(Phase::READY),
        "the planner did not return the reference after its action");
    std::cout << "decision planner publication: PASS" << std::endl;
}

void TestOfflinePlanner() {
    auto &script = GetScript();
    script.Reset();
    script.generate_delay_ms = 100;
    Tracker tracker(MakeHost({MakePlanner("generator", "fake_generator", Category::OFFLINE),
        MakePlanner("walker", "fake_trajectory")}));
    tracker.Start("generator", "wave to the left");
    bool generating = false;
    Require(tracker.Until([&] {
        generating = generating || tracker.phase() == Phase::GENERATING;
        if (tracker.phase() != Phase::RUNNING) {
            Require(!tracker.host.Output(), "the standby reference must be kept during generation");
        }
        return tracker.phase() == Phase::RUNNING;
    }), "generation did not finish");
    {
        std::lock_guard<std::mutex> lock(script.mutex);
        Require(generating && script.generated == 1 && script.argument == "wave to the left",
            "the task text must reach the offline planner");
    }
    Require(tracker.host.Output() && tracker.host.OutputFinite() &&
        tracker.host.Output()->trajectory.FrameCount() == 26 && tracker.host.Engaged(),
        "a generated motion is a finite reference");

    // A second request is refused until the motion has been played.
    tracker.Start("generator", "again");
    tracker.Step();
    Require(!tracker.status().request_accepted && Contains(tracker.status().detail, "stop it first"),
        "an offline planner handles one request at a time");
    tracker.state.playback = {true, 0.5F, false};
    tracker.Step();
    Require(tracker.phase() == Phase::RUNNING && std::abs(tracker.status().progress - 0.5F) < 1.0e-6F,
        "playback progress must be reported");
    tracker.state.playback = {true, 1.0F, true};
    tracker.Step();
    Require(!tracker.host.Output() && !tracker.host.Engaged(), "the reference returns once the motion has played");
    tracker.state.playback = {};
    Require(tracker.UntilPhase(Phase::READY) && script.generated == 1, "the planner stays loaded for the next task");

    // The next task reuses the loaded planner; an action name is a valid result.
    tracker.Start("generator", "action:bow");
    Require(tracker.UntilPhase(Phase::RUNNING) && script.constructed == 1 &&
        tracker.host.Output()->action == "bow" && tracker.host.OutputFinite(),
        "an offline planner may select a catalog action");
    tracker.Stop();
    tracker.Step();
    Require(!tracker.host.Output(), "a stop interrupts the generated motion");
    Require(tracker.UntilPhase(Phase::READY), "the planner did not become ready again");

    // Stopping during generation sets the cancel flag and drops the result.
    script.generate_delay_ms = 2000;
    tracker.Start("generator", "long task");
    Require(tracker.UntilPhase(Phase::GENERATING), "generation did not begin");
    tracker.Stop();
    tracker.Step();
    Require(tracker.status().request_accepted && !tracker.host.Engaged(), "a stop must cancel generation");
    Require(tracker.Until([&] { return script.saw_cancel.load(); }, 1000),
        "the planner was not told about the cancellation");
    Require(tracker.UntilPhase(Phase::READY) && !tracker.host.Output() && script.generated == 2,
        "a cancelled generation must not publish a motion");

    script.generate_delay_ms = 0;
    script.fail_generate = true;
    tracker.Start("generator", "impossible");
    Require(tracker.UntilPhase(Phase::FAILED) && Contains(tracker.status().detail, "failed to generate") &&
        !tracker.host.Output(), "a generation failure must be reported");
    script.fail_generate = false;
    script.invalid_output = true;
    tracker.Start("generator", "broken");
    Require(tracker.UntilPhase(Phase::FAILED) && Contains(tracker.status().detail, "invalid reference"),
        "an invalid generated motion must be rejected before it reaches the tracker");
    script.invalid_output = false;

    // Realtime and offline planners of one policy replace each other on request.
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::RUNNING) && !tracker.host.OutputFinite() &&
        tracker.status().planner == "walker", "the realtime planner did not replace the offline one");
    std::cout << "offline planner generation: PASS" << std::endl;
}

void TestSuspendAndReset() {
    auto &script = GetScript();
    script.Reset();
    Tracker tracker(MakeHost({MakePlanner("walker", "fake_trajectory")}));
    tracker.Start("walker");
    Require(tracker.UntilPhase(Phase::RUNNING), "the planner did not start");
    // Leaving RL: the inference thread has stopped, so nothing steps the host any more.
    tracker.host.Suspend();
    Require(!tracker.host.Output() && !tracker.host.Engaged(), "suspending must return the reference at once");
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
    while (script.destroyed == 0 && std::chrono::steady_clock::now() < deadline) {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    Require(script.destroyed == 1 && script.stopped == 1, "suspending must stop and unload the planner");
    tracker.host.Suspend();  // Idempotent.

    // Entering RL again starts from a clean state and accepts low sequence numbers.
    tracker.host.Reset();
    tracker.request = {};
    tracker.time_s = 0.0;
    Require(tracker.phase() == Phase::IDLE && tracker.status().sequence == 0 && tracker.status().detail.empty(),
        "reset must clear the previous session");
    tracker.Start("walker");
    Require(tracker.request.sequence == 1 && tracker.UntilPhase(Phase::RUNNING) && script.constructed == 2,
        "a new session must be able to start the planner again");

    // Suspending during a load leaves the job to finish; the planner is released afterwards.
    script.Reset();
    script.load_delay_ms = 150;
    {
        Tracker loading(MakeHost({MakePlanner("walker", "fake_trajectory")}));
        loading.Start("walker");
        loading.Step(2);
        loading.host.Suspend();
        Require(!loading.host.Engaged(), "suspending must cancel a pending start");
    }
    Require(script.constructed == script.destroyed, "a planner loaded after the suspend must still be released");
    std::cout << "planner suspend and reset: PASS" << std::endl;
}

// Progress while a start is pending: estimated from the expected load time, and reported
// by an offline planner while it generates.
void TestPreparationProgress() {
    auto &script = GetScript();
    script.Reset();
    script.load_delay_ms = 60;
    // Samples the progress of the load until the planner runs.
    struct Samples {
        float peak = 0.0F;
        bool ordered = true;
    };
    const auto load = [](Tracker &tracker, const std::string &name) {
        Samples samples;
        float previous = 0.0F;
        tracker.Start(name);
        Require(tracker.Until([&] {
            const float progress = tracker.status().progress;
            if (tracker.phase() == Phase::LOADING || tracker.phase() == Phase::WARMING) {
                samples.ordered = samples.ordered && progress >= previous;
                samples.peak = std::max(samples.peak, progress);
                previous = progress;
            }
            return tracker.phase() == Phase::RUNNING;
        }), "the planner did not start: " + name);
        return samples;
    };
    {
        Tracker tracker(MakeHost({MakePlanner("load_unknown", "fake_trajectory")}));
        Require(load(tracker, "load_unknown").peak == 0.0F,
            "a load without an expected duration must not report progress");
    }
    {
        // The configured duration is far longer than the load, so the estimate stays low.
        auto planner = MakePlanner("load_estimated", "fake_trajectory");
        planner.prepare_time_s = 1000.0;
        planner.keep_loaded = false;
        Tracker tracker(MakeHost({planner}));
        auto samples = load(tracker, "load_estimated");
        Require(samples.ordered && samples.peak > 0.0F && samples.peak < 0.05F,
            "the estimate must follow the configured load time");
        Require(tracker.status().progress == 0.0F, "a running trajectory planner reports no progress");
        tracker.Stop();
        Require(tracker.UntilPhase(Phase::IDLE), "the planner was not unloaded");
        // The duration measured by the first load replaces the configured one.
        samples = load(tracker, "load_estimated");
        Require(samples.ordered && samples.peak > 0.3F && samples.peak <= 0.95F,
            "a measured load time must replace the configured estimate");
        // A load that takes longer than expected never reports completion.
        tracker.Stop();
        Require(tracker.UntilPhase(Phase::IDLE), "the planner was not unloaded");
        script.load_delay_ms = 400;
        samples = load(tracker, "load_estimated");
        Require(samples.ordered && samples.peak == 0.95F, "an overdue load must hold below completion");
        script.load_delay_ms = 60;
    }
    {
        // Restarting a planner that stayed loaded involves no load and no estimate.
        auto planner = MakePlanner("load_kept", "fake_trajectory");
        planner.prepare_time_s = 1000.0;
        Tracker tracker(MakeHost({planner}));
        Require(load(tracker, "load_kept").peak > 0.0F, "the first start must load the planner");
        tracker.Stop();
        Require(tracker.UntilPhase(Phase::READY), "the planner did not stop");
        Require(load(tracker, "load_kept").peak == 0.0F && script.constructed == 5,
            "a restart without a load must not report progress");
        // A cancelled load leaves no progress behind.
        tracker.host.Suspend();
        tracker.host.Reset();
        tracker.request = {};
        Require(tracker.UntilPhase(Phase::IDLE), "the planner was not unloaded");
        script.load_delay_ms = 300;
        tracker.Start("load_kept");
        Require(tracker.Until([&] { return tracker.status().progress > 0.0F; }),
            "the load did not report progress");
        tracker.Stop();
        tracker.Step();
        Require(tracker.status().progress == 0.0F, "a cancelled load must clear its progress");
        Require(tracker.Until([&] { return tracker.phase() == Phase::READY ||
            tracker.phase() == Phase::IDLE; }), "the cancelled load did not settle");
        script.load_delay_ms = 0;
    }
    {
        // An offline planner reports its own progress while generating.
        script.generate_delay_ms = 150;
        script.generate_progress = 0.4F;
        Tracker tracker(MakeHost({MakePlanner("writer", "fake_generator", Category::OFFLINE)}));
        tracker.Start("writer", "wave");
        Require(tracker.Until([&] { return tracker.phase() == Phase::GENERATING &&
            tracker.status().progress == 0.4F; }), "generation progress was not reported");
        Require(tracker.UntilPhase(Phase::RUNNING), "the generated motion did not start");
        Require(tracker.status().progress == 0.0F, "playback progress must replace generation progress");
        tracker.state.playback = {true, 1.0F, true};
        Require(tracker.UntilPhase(Phase::READY), "the generated motion did not finish");
        tracker.state.playback = {};
        // Values outside [0, 1] are clamped so the status stays transmittable.
        script.generate_progress = 7.0F;
        tracker.Start("writer", "wave");
        Require(tracker.Until([&] { return tracker.phase() == Phase::GENERATING &&
            tracker.status().progress == 1.0F; }), "an out-of-range progress was not clamped");
        script.generate_progress = std::numeric_limits<float>::quiet_NaN();
        tracker.Stop();
        Require(tracker.Until([&] { return tracker.phase() == Phase::READY; }), "the generation was not cancelled");
        tracker.Start("writer", "wave");
        Require(tracker.UntilPhase(Phase::RUNNING) && tracker.status().progress == 0.0F,
            "a non-finite progress must be ignored");
    }
    script.Reset();
    std::cout << "preparation progress: PASS" << std::endl;
}

// Records what the binding asks of a tracker's reference.
class RecordingSink final : public behavior_manager::motion_planner::ReferenceSink {
public:
    std::string FollowAction(const std::string &action, double) override {
        if (action == missing_action) return "action " + action + " is not in the catalog";
        calls.push_back("action:" + action);
        return {};
    }

    std::string FollowTrajectory(const behavior_manager::motion_planner::ReferenceTrajectory &trajectory,
        const behavior_manager::motion_planner::InstallReference &install,
        bool finite, bool replace, double) override {
        if (reject_trajectory) return "scripted rejection";
        if (install) install();
        calls.push_back(std::string(finite ? "finite" : "continuous") + (replace ? ":replace" : ""));
        value = trajectory.joint_pos[0];
        return {};
    }

    void FollowStandby(double) override { calls.push_back("standby"); }

    behavior_manager::motion_planner::PlaybackState Playback(double) const override {
        ++playback_queries;
        return playback;
    }

    // Calls recorded since the last Take().
    std::vector<std::string> Take() { return std::exchange(calls, {}); }

    std::vector<std::string> calls;
    std::string missing_action;
    bool reject_trajectory = false;
    float value = 0.0F;
    behavior_manager::motion_planner::PlaybackState playback;
    mutable int playback_queries = 0;
};

// Drives a binding the way a tracker's inference thread does.
class BoundTracker {
public:
    explicit BoundTracker(HostConfig config) : binding(std::move(config), sink) {
        robot = robot_base::RobotData::Create(kDof);
        robot.environment = robot_base::RuntimeEnvironment::SIMULATION;
        binding.Reset();
    }

    void Step() {
        binding.SetRequest(request);
        binding.Step(robot, command, time_s, busy);
        time_s += 0.02;
    }

    bool Until(const std::function<bool()> &done, int timeout_ms = 3000) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms);
        while (true) {
            Step();
            if (done()) return true;
            if (std::chrono::steady_clock::now() >= deadline) return false;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    bool UntilPhase(Phase phase) { return Until([&] { return binding.Status().phase == phase; }); }

    void Start(const std::string &planner, const std::string &argument = "") {
        request = {request.sequence + 1, Operation::START, planner, argument};
    }

    void Stop() { request = {request.sequence + 1, Operation::STOP, "", ""}; }

    RecordingSink sink;
    behavior_manager::motion_planner::PlannerBinding binding;
    robot_base::RobotData robot;
    MotionCommand command;
    robot_base::PlannerRequest request;
    bool busy = false;
    double time_s = 0.0;
};

// The contract between the binding and a tracker: which reference change each publication
// of the planner turns into.
void TestTrajectoryHandoff() {
    auto &script = GetScript();
    script.Reset();
    auto planner = MakePlanner("fixed", "fake_trajectory");
    planner.minimum_command = {};
    planner.maximum_command = {};
    auto config = MakeHost({planner});
    const auto inference = std::this_thread::get_id();
    std::atomic<bool> prepared{false};
    std::atomic<bool> release{false};
    bool installed = false;
    config.prepare_trajectory = [&](const auto &) {
        Require(std::this_thread::get_id() != inference, "conversion ran on the inference thread");
        prepared = true;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
        while (!release && std::chrono::steady_clock::now() < deadline) std::this_thread::yield();
        return [&] { installed = std::this_thread::get_id() == inference; };
    };
    BoundTracker tracker(std::move(config));
    tracker.command.vx = 0.8F;
    tracker.Start("fixed");
    Require(tracker.Until([&] { return prepared.load(); }), "worker did not receive the trajectory");
    tracker.Step();
    Require(!tracker.binding.Following() && !installed && script.last_vx == 0.0F,
        "a zero-speed planner must stay still, with the tracker in standby until the trajectory is ready");
    release = true;
    Require(tracker.Until([&] { return installed && tracker.binding.Following(); }),
        "ready trajectory was not installed on the inference thread");
}

void TestBinding() {
    using Calls = std::vector<std::string>;
    auto &script = GetScript();
    script.Reset();
    const auto host = [] {
        return MakeHost({MakePlanner("walker", "fake_trajectory"), MakePlanner("chooser", "fake_choice"),
            MakePlanner("writer", "fake_generator", Category::OFFLINE)});
    };
    for (const bool offline : {false, true}) {
        auto config = host();
        config.action_environments["gesture"].real = false;
        BoundTracker tracker(std::move(config));
        tracker.robot.environment = robot_base::RuntimeEnvironment::REAL;
        tracker.Start(offline ? "writer" : "chooser", offline ? "action:gesture" : "gesture");
        Require(tracker.UntilPhase(Phase::FAILED) && tracker.sink.calls.empty() &&
            Contains(tracker.binding.Status().detail, "gesture is not enabled"),
            "a sim-only planner action must never reach the tracker on hardware");
        tracker.robot.environment = robot_base::RuntimeEnvironment::SIMULATION;
        tracker.Start(offline ? "writer" : "chooser", offline ? "action:gesture" : "gesture");
        Require(tracker.Until([&] { return tracker.binding.Following(); }) &&
            tracker.sink.Take() == Calls{"action:gesture"}, "the same action must work in simulation");
    }
    {
        // Realtime trajectories: one crossfade in, in-place replacements, one crossfade out.
        BoundTracker tracker(host());
        tracker.Step();
        Require(tracker.sink.calls.empty() && !tracker.binding.Engaged() && !tracker.binding.Following(),
            "an idle binding must leave the reference alone");
        tracker.busy = true;
        tracker.Start("walker");
        tracker.Step();
        Require(!tracker.binding.Status().request_accepted && !tracker.binding.Engaged(),
            "a busy tracker must refuse a planner start");
        tracker.busy = false;
        tracker.Start("walker");
        tracker.Step();
        Require(tracker.binding.Engaged() && !tracker.binding.Following() && tracker.sink.calls.empty(),
            "the reference stays with the tracker while the planner loads");
        Require(tracker.Until([&] { return tracker.binding.Following(); }), "the planner did not publish");
        Require(tracker.sink.Take() == Calls{"continuous"} && tracker.sink.value == 0.5F,
            "the first plan must be blended in");
        tracker.command.vx = 0.2F;
        Require(tracker.Until([&] { return tracker.sink.value == 0.7F; }), "the next plan was not delivered");
        Require(tracker.sink.Take() == Calls{"continuous:replace"}, "later plans replace the trajectory in place");
        for (int index = 0; index < 5; ++index) tracker.Step();
        Require(tracker.sink.calls.empty(), "an unchanged publication must not be delivered again");
        Require(tracker.sink.playback_queries == 0, "a continuous trajectory has no playback progress");
        tracker.command.vx = 0.0F;
        Require(tracker.Until([&] { return tracker.sink.value == 0.5F; }), "the zero command was not planned");
        tracker.sink.Take();
        tracker.Stop();
        Require(tracker.Until([&] { return !tracker.binding.Engaged(); }), "the planner did not stop");
        Require(tracker.sink.Take() == Calls{"standby"} && !tracker.binding.Following(),
            "stopping returns to the standby reference once");

        // A trajectory the tracker cannot use: the first one leaves the standby in place,
        // a later one returns to it.
        tracker.sink.reject_trajectory = true;
        tracker.Start("walker");
        Require(tracker.UntilPhase(Phase::FAILED) &&
            Contains(tracker.binding.Status().detail, "scripted rejection"),
            "a rejected trajectory must fail the planner");
        Require(tracker.sink.calls.empty() && !tracker.binding.Following() && !tracker.binding.Engaged(),
            "a rejected first plan must not change the reference");
        tracker.sink.reject_trajectory = false;
        tracker.Start("walker");
        Require(tracker.Until([&] { return tracker.binding.Following(); }), "the planner did not restart");
        tracker.sink.Take();
        tracker.sink.reject_trajectory = true;
        tracker.command.vx = 0.2F;
        Require(tracker.UntilPhase(Phase::FAILED), "a rejected replacement must fail the planner");
        Require(tracker.sink.Take() == Calls{"standby"} && !tracker.binding.Following(),
            "a rejected replacement returns to the standby reference");
        tracker.sink.reject_trajectory = false;
        tracker.command.vx = 0.0F;

        // A policy switch asks the planner to hand the reference back.
        tracker.Start("walker");
        Require(tracker.Until([&] { return tracker.binding.Following(); }), "the planner did not restart");
        tracker.sink.Take();
        tracker.binding.RequestRelease();
        Require(tracker.Until([&] { return !tracker.binding.Engaged(); }), "the release was not honoured");
        Require(tracker.sink.Take() == Calls{"standby"}, "a release returns to the standby reference");

        // Entering RL again: the tracker has reset its own reference.
        tracker.Start("walker");
        Require(tracker.Until([&] { return tracker.binding.Following(); }), "the planner did not restart");
        tracker.sink.Take();
        tracker.binding.Suspend();
        tracker.binding.Reset();
        tracker.request = {};
        Require(!tracker.binding.Following() && !tracker.binding.Engaged() && tracker.sink.calls.empty() &&
            tracker.binding.Status().sequence == 0, "a reset must not drive the reference");
    }
    {
        // Decision planner: named actions, with the playback progress fed back to it.
        BoundTracker tracker(host());
        tracker.Start("chooser");
        Require(tracker.Until([&] { return tracker.binding.Following(); }), "the planner did not select");
        Require(tracker.sink.Take() == Calls{"action:idle"}, "the selected action must be played");
        tracker.command.vx = 0.3F;
        Require(tracker.Until([&] { return !tracker.sink.calls.empty(); }), "the planner did not react");
        Require(tracker.sink.Take() == Calls{"action:gesture"}, "a new selection must be played");
        const int queries = tracker.sink.playback_queries;
        tracker.sink.playback = {true, 0.4F, false};
        tracker.Step();
        Require(tracker.sink.playback_queries == queries + 1 && tracker.binding.Status().progress == 0.4F,
            "playback progress of a named action must reach the planner status");
        // Each publication is a new instruction, even for the same action.
        tracker.sink.playback = {true, 1.0F, true};
        Require(tracker.Until([&] { return !tracker.sink.calls.empty(); }), "the action was not replayed");
        Require(tracker.sink.Take() == Calls{"action:gesture"}, "a repeated selection restarts the action");
        tracker.sink.missing_action = "gesture";
        Require(tracker.UntilPhase(Phase::FAILED) &&
            Contains(tracker.binding.Status().detail, "not in the catalog"),
            "an action the tracker does not have must fail the planner");
        Require(tracker.sink.Take() == Calls{"standby"}, "a failed selection returns to the standby reference");
    }
    {
        // Offline planner: one finite reference, released when its playback ends.
        BoundTracker tracker(host());
        tracker.Start("writer", "wave");
        Require(tracker.Until([&] { return tracker.binding.Following(); }), "nothing was generated");
        Require(tracker.sink.Take() == Calls{"finite"} && tracker.sink.value == 0.7F,
            "a generated motion is played once from its first frame");
        tracker.sink.playback = {true, 0.5F, false};
        tracker.Step();
        Require(tracker.binding.Status().progress == 0.5F && tracker.binding.Status().phase == Phase::RUNNING,
            "playback progress of a generated motion must be reported");
        tracker.sink.playback = {true, 1.0F, true};
        Require(tracker.Until([&] { return !tracker.binding.Engaged(); }), "the motion did not finish");
        Require(tracker.sink.Take() == Calls{"standby"}, "a finished motion returns to the standby reference");
        tracker.sink.playback = {};
        tracker.Start("writer", "action:gesture");
        Require(tracker.Until([&] { return tracker.binding.Following(); }), "nothing was selected");
        Require(tracker.sink.Take() == Calls{"action:gesture"}, "an offline planner may name a catalog action");
    }
    script.Reset();
    std::cout << "planner binding: PASS" << std::endl;
}

}  // namespace

int main() {
    try {
        fake_planners::Register();
        TestRegistry();
        TestValidateOutput();
        TestRealtimeLifecycle();
        TestUnloadAndSwitch();
        TestRejections();
        TestBackgroundLoad();
        TestSupersededStart();
        TestFailures();
        TestStopTimeout();
        TestPreloadAndAutoStart();
        TestChoicePlanner();
        TestOfflinePlanner();
        TestSuspendAndReset();
        TestPreparationProgress();
        TestTrajectoryHandoff();
        TestBinding();
    } catch (const std::exception &error) {
        std::cerr << "motion planner test failed: " << error.what() << std::endl;
        return 1;
    }
    std::cout << "motion planner framework: PASS" << std::endl;
    return 0;
}
