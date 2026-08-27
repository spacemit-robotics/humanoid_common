/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file benchmark_humanoid_policy.cpp
 * @brief StateRL 异步策略运行时 benchmark
 *
 * 通过真实 FSM 和 StateRL 驱动控制线程、推理线程、传感器快照、PolicyAdapter、
 * PolicyExecutor、动作缓存及关节目标映射。传感器数据由工具生成；transport、
 * 关节总线和执行器闭环不在本工具范围内。
 */

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "benchmark_build_metadata.h"
#include "behavior_fsm.h"
#include "rl_benchmark/benchmark_common.h"
#include "robot_base.h"
#include "state_factory.h"

namespace {

using behavior_manager::FSM;
using behavior_manager::RLConfig;
using behavior_manager::RLRuntimeClock;
using behavior_manager::RLRuntimeEvent;
using behavior_manager::RLRuntimeEventType;
using behavior_manager::RLRuntimeObserver;
using behavior_manager::StateName;
using rl_benchmark::Options;
using rl_benchmark::Stats;

constexpr std::int64_t kUnsetTimestamp = 0;

std::int64_t ToNanoseconds(RLRuntimeClock::time_point timestamp) {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        timestamp.time_since_epoch())
        .count();
}

double NanosecondsToMilliseconds(std::int64_t nanoseconds) {
    return static_cast<double>(nanoseconds) / 1.0e6;
}

void StoreMaximum(
    std::atomic<std::uint64_t> *target, std::uint64_t value) noexcept {
    std::uint64_t current = target->load(std::memory_order_relaxed);
    while (current < value &&
        !target->compare_exchange_weak(current, value,
            std::memory_order_release, std::memory_order_relaxed)) {
    }
}

struct AtomicTrace {
    std::atomic<std::int64_t> release_ns{kUnsetTimestamp};
    std::atomic<std::int64_t> inference_start_ns{kUnsetTimestamp};
    std::atomic<std::int64_t> inference_finish_ns{kUnsetTimestamp};
    std::atomic<std::int64_t> published_ns{kUnsetTimestamp};
    std::atomic<std::int64_t> applied_ns{kUnsetTimestamp};
};

struct TraceSnapshot {
    std::int64_t release_ns = kUnsetTimestamp;
    std::int64_t inference_start_ns = kUnsetTimestamp;
    std::int64_t inference_finish_ns = kUnsetTimestamp;
    std::int64_t published_ns = kUnsetTimestamp;
    std::int64_t applied_ns = kUnsetTimestamp;
};

class RuntimeCollector final : public RLRuntimeObserver {
public:
    explicit RuntimeCollector(std::size_t maximum_release_sequence)
        : capacity_(maximum_release_sequence + 1)
        , traces_(std::make_unique<AtomicTrace[]>(capacity_)) {}

    void OnRuntimeInitialized(
        const rl_policy::InferenceRuntimeInfo &runtime) override {
        runtime_ = runtime;
        runtime_ready_ = true;
    }

    void OnRuntimeEvent(const RLRuntimeEvent &event) noexcept override {
        if (event.release_sequence >= capacity_) {
            overflow_events_.fetch_add(1, std::memory_order_relaxed);
            return;
        }
        AtomicTrace &trace = traces_[event.release_sequence];
        const std::int64_t timestamp = ToNanoseconds(event.timestamp);
        switch (event.type) {
        case RLRuntimeEventType::RELEASE:
            trace.release_ns.store(timestamp, std::memory_order_release);
            StoreMaximum(&maximum_release_, event.release_sequence);
            break;
        case RLRuntimeEventType::INFERENCE_START:
            trace.inference_start_ns.store(
                timestamp, std::memory_order_release);
            break;
        case RLRuntimeEventType::INFERENCE_FINISH:
            trace.inference_finish_ns.store(
                timestamp, std::memory_order_release);
            break;
        case RLRuntimeEventType::ACTION_PUBLISHED:
            trace.published_ns.store(timestamp, std::memory_order_release);
            StoreMaximum(&maximum_published_, event.release_sequence);
            break;
        case RLRuntimeEventType::ACTION_APPLIED:
            trace.applied_ns.store(timestamp, std::memory_order_release);
            StoreMaximum(&maximum_applied_, event.release_sequence);
            break;
        }
    }

    bool RuntimeReady() const { return runtime_ready_; }

    const rl_policy::InferenceRuntimeInfo &RuntimeInfo() const {
        return runtime_;
    }

    std::uint64_t MaximumRelease() const {
        return maximum_release_.load(std::memory_order_acquire);
    }

    std::uint64_t MaximumPublished() const {
        return maximum_published_.load(std::memory_order_acquire);
    }

    std::uint64_t MaximumApplied() const {
        return maximum_applied_.load(std::memory_order_acquire);
    }

    std::uint64_t OverflowEvents() const {
        return overflow_events_.load(std::memory_order_acquire);
    }

    TraceSnapshot Snapshot(std::uint64_t sequence) const {
        if (sequence >= capacity_) {
            throw std::out_of_range("runtime trace sequence exceeds capacity");
        }
        const AtomicTrace &trace = traces_[sequence];
        TraceSnapshot snapshot;
        snapshot.release_ns = trace.release_ns.load(std::memory_order_acquire);
        snapshot.inference_start_ns =
            trace.inference_start_ns.load(std::memory_order_acquire);
        snapshot.inference_finish_ns =
            trace.inference_finish_ns.load(std::memory_order_acquire);
        snapshot.published_ns =
            trace.published_ns.load(std::memory_order_acquire);
        snapshot.applied_ns = trace.applied_ns.load(std::memory_order_acquire);
        return snapshot;
    }

private:
    std::size_t capacity_;
    std::unique_ptr<AtomicTrace[]> traces_;
    rl_policy::InferenceRuntimeInfo runtime_;
    bool runtime_ready_ = false;
    std::atomic<std::uint64_t> maximum_release_{0};
    std::atomic<std::uint64_t> maximum_published_{0};
    std::atomic<std::uint64_t> maximum_applied_{0};
    std::atomic<std::uint64_t> overflow_events_{0};
};

class StateRLSession {
public:
    StateRLSession(const RLConfig &config, robot_base::RobotData *sensor,
        robot_base::Command *command, behavior_manager::ControlOutput *output) {
        fsm_.AddState(
            StateName::POWER_OFF, behavior_manager::CreateStatePowerOff());
        fsm_.AddState(StateName::RL, behavior_manager::CreateStateRl(config));
        fsm_.SetDataPointers(sensor, command, output);
    }

    ~StateRLSession() {
        if (active_) {
            Stop();
        }
    }

    void Start() {
        fsm_.Init();
        fsm_.ForceSwitch(StateName::RL, "runtime benchmark");
        active_ = true;
    }

    void Stop() noexcept {
        try {
            fsm_.ForceSwitch(
                StateName::POWER_OFF, "runtime benchmark complete");
        } catch (...) {
        }
        active_ = false;
    }

    void Step(float control_dt, float rl_dt) { fsm_.Step(control_dt, rl_dt); }

private:
    FSM fsm_;
    bool active_ = false;
};

struct ControlLoopSamples {
    std::vector<double> interval_error_ms;
    std::uint64_t ticks = 0;
    std::uint64_t ticks_reusing_cached_action = 0;
    std::uint64_t max_release_lag = 0;
};

struct RuntimeSample {
    std::uint64_t release_sequence = 0;
    bool inference_started = false;
    bool action_published = false;
    bool action_applied = false;
    double release_jitter_ms = std::numeric_limits<double>::quiet_NaN();
    double wake_latency_ms = std::numeric_limits<double>::quiet_NaN();
    double inference_service_ms = std::numeric_limits<double>::quiet_NaN();
    double publish_response_ms = std::numeric_limits<double>::quiet_NaN();
    double apply_wait_ms = std::numeric_limits<double>::quiet_NaN();
    double action_age_ms = std::numeric_limits<double>::quiet_NaN();
    double deadline_lateness_ms = std::numeric_limits<double>::quiet_NaN();
};

struct RuntimeResult {
    std::vector<RuntimeSample> samples;
    std::vector<double> release_jitter_ms;
    std::vector<double> wake_latency_ms;
    std::vector<double> inference_service_ms;
    std::vector<double> publish_response_ms;
    std::vector<double> apply_wait_ms;
    std::vector<double> action_age_ms;
    std::vector<double> action_update_interval_ms;
    std::vector<double> deadline_lateness_ms;
    std::uint64_t inference_count = 0;
    std::uint64_t applied_count = 0;
    std::uint64_t coalesced_releases = 0;
    std::uint64_t incomplete_inferences = 0;
    std::uint64_t superseded_actions = 0;
    std::uint64_t deadline_misses = 0;
};

void PrintUsage(const char *program) {
    std::cerr << "用法: " << program;
    std::cerr << " <yaml_path> <policy_name> <robot_dir> [options]\n";
    rl_benchmark::PrintCommonUsage(std::cerr);
    std::cerr << "  --reference-loop            ";
    std::cerr << "内存中覆盖参考动作为循环，不修改 YAML\n";
    std::cerr << "\n本工具运行真实 StateRL 周期路径，仅支持 --mode periodic；";
    std::cerr << "未指定 --mode 时默认 periodic。\n";
}

bool HasOption(int argc, char *argv[], const std::string &name) {
    for (int index = 4; index < argc; ++index) {
        if (std::string(argv[index]) == name) {
            return true;
        }
    }
    return false;
}

Options ParseRuntimeOptions(
    int argc, char *argv[], double default_hz, bool *reference_loop) {
    const bool mode_overridden = HasOption(argc, argv, "--mode");
    if (HasOption(argc, argv, "--overrun")) {
        throw std::invalid_argument(
            "StateRL uses its production latest-snapshot semantics; "
            "--overrun is not configurable");
    }

    std::vector<char *> common_argv;
    common_argv.reserve(argc);
    for (int index = 0; index < argc; ++index) {
        if (index >= 4 && std::string(argv[index]) == "--reference-loop") {
            if (*reference_loop) {
                throw std::invalid_argument(
                    "--reference-loop may only be specified once");
            }
            *reference_loop = true;
            continue;
        }
        common_argv.push_back(argv[index]);
    }

    Options options =
        rl_benchmark::ParseOptions(static_cast<int>(common_argv.size()),
            common_argv.data(), 4, default_hz);
    if (!mode_overridden) {
        options.mode = rl_benchmark::Mode::PERIODIC;
    }
    if (options.mode != rl_benchmark::Mode::PERIODIC) {
        throw std::invalid_argument(
            "StateRL runtime benchmark supports --mode periodic only");
    }
    return options;
}

robot_base::RobotData MakeSyntheticRobot(const RLConfig &config) {
    robot_base::RobotData robot;
    robot.num_dof = static_cast<int>(config.policy.rl_default_pos.size());
    if (robot.num_dof <= 0) {
        throw std::runtime_error("rl_default_pos is empty");
    }
    robot.base_pos = {0.0, 0.0, 1.0};
    robot.base_quat = {1.0, 0.0, 0.0, 0.0};
    robot.base_vel = {0.0, 0.0, 0.0, 0.01, -0.02, 0.03};
    robot.rpy = {0.0, 0.0, 0.0};
    robot.gyro = {0.01, -0.02, 0.03};
    robot.joint_pos = config.policy.rl_default_pos;
    robot.joint_vel.assign(robot.num_dof, 0.0);
    robot.joint_torque.assign(robot.num_dof, 0.0);
    robot.joint_temperature.assign(robot.num_dof, 25.0);
    robot.joint_error.assign(robot.num_dof, 0);
    return robot;
}

behavior_manager::ControlOutput MakeInitialOutput(const RLConfig &config) {
    behavior_manager::ControlOutput output;
    const std::size_t num_dof = config.policy.rl_default_pos.size();
    output.target_pos = config.policy.rl_default_pos;
    output.target_vel.assign(num_dof, 0.0);
    output.target_torque.assign(num_dof, 0.0);
    output.kp = config.kp;
    output.kd = config.kd;
    return output;
}

std::chrono::steady_clock::duration DurationFromSeconds(double seconds) {
    const auto duration = std::chrono::duration_cast<RLRuntimeClock::duration>(
        std::chrono::duration<double>(seconds));
    if (duration <= RLRuntimeClock::duration::zero()) {
        throw std::invalid_argument("period is below steady_clock resolution");
    }
    return duration;
}

void UpdateControlStats(const RuntimeCollector &collector,
    std::uint64_t applied_before, double actual_control_dt,
    double configured_control_dt, ControlLoopSamples *samples) {
    if (!samples) {
        return;
    }
    samples->interval_error_ms.push_back(
        (actual_control_dt - configured_control_dt) * 1000.0);
    ++samples->ticks;
    if (collector.MaximumApplied() == applied_before) {
        ++samples->ticks_reusing_cached_action;
    }
    const std::uint64_t released = collector.MaximumRelease();
    const std::uint64_t applied = collector.MaximumApplied();
    samples->max_release_lag = std::max(
        samples->max_release_lag, released >= applied ? released - applied : 0);
}

void DriveUntilRelease(StateRLSession *session, RuntimeCollector *collector,
    robot_base::RobotData *robot, double control_dt, double rl_dt,
    std::uint64_t target_release, RLRuntimeClock::time_point deadline,
    RLRuntimeClock::time_point *last_control_time,
    ControlLoopSamples *control_samples) {
    const auto control_period = DurationFromSeconds(control_dt);
    while (collector->MaximumRelease() < target_release) {
        const auto scheduled = *last_control_time + control_period;
        std::this_thread::sleep_until(scheduled);
        const auto now = RLRuntimeClock::now();
        const double actual_control_dt =
            std::chrono::duration<double>(now - *last_control_time).count();
        *last_control_time = now;
        robot->time += actual_control_dt;
        const std::uint64_t applied_before = collector->MaximumApplied();
        session->Step(
            static_cast<float>(actual_control_dt), static_cast<float>(rl_dt));
        UpdateControlStats(*collector, applied_before, actual_control_dt,
            control_dt, control_samples);
        if (now > deadline) {
            throw std::runtime_error(
                "timed out while driving StateRL releases");
        }
    }
}

void DrainUntilApplied(StateRLSession *session, RuntimeCollector *collector,
    double control_dt, double rl_dt, std::uint64_t target_release,
    RLRuntimeClock::time_point deadline,
    RLRuntimeClock::time_point *last_control_time,
    ControlLoopSamples *control_samples) {
    const auto control_period = DurationFromSeconds(control_dt);
    while (collector->MaximumApplied() < target_release) {
        const auto scheduled = *last_control_time + control_period;
        std::this_thread::sleep_until(scheduled);
        const auto now = RLRuntimeClock::now();
        const double actual_control_dt =
            std::chrono::duration<double>(now - *last_control_time).count();
        *last_control_time = now;
        const std::uint64_t applied_before = collector->MaximumApplied();

        // 测量窗口的最后一次 release 后冻结推理触发累计，仅继续真实控制线程
        // 的动作缓存消费，确保最后一个 action 在下一个控制周期被应用。
        session->Step(0.0f, static_cast<float>(rl_dt));
        UpdateControlStats(*collector, applied_before, actual_control_dt,
            control_dt, control_samples);
        if (now > deadline) {
            throw std::runtime_error(
                "timed out while waiting for the final StateRL action");
        }
    }
}

RuntimeResult AnalyzeRuntime(const RuntimeCollector &collector,
    std::uint64_t first_sequence, int rounds, double rl_dt) {
    RuntimeResult result;
    result.samples.reserve(rounds);
    const double period_ms = rl_dt * 1000.0;
    std::int64_t previous_release_ns = kUnsetTimestamp;
    std::int64_t previous_applied_ns = kUnsetTimestamp;

    for (int index = 0; index < rounds; ++index) {
        RuntimeSample sample;
        sample.release_sequence = first_sequence + index;
        const TraceSnapshot trace = collector.Snapshot(sample.release_sequence);
        if (trace.release_ns == kUnsetTimestamp) {
            throw std::runtime_error("missing StateRL release trace");
        }

        if (previous_release_ns != kUnsetTimestamp) {
            sample.release_jitter_ms =
                NanosecondsToMilliseconds(
                    trace.release_ns - previous_release_ns) -
                period_ms;
            result.release_jitter_ms.push_back(sample.release_jitter_ms);
        }
        previous_release_ns = trace.release_ns;

        sample.inference_started = trace.inference_start_ns != kUnsetTimestamp;
        sample.action_published = trace.published_ns != kUnsetTimestamp;
        sample.action_applied = trace.applied_ns != kUnsetTimestamp;

        if (!sample.inference_started) {
            ++result.coalesced_releases;
        } else {
            ++result.inference_count;
            sample.wake_latency_ms = NanosecondsToMilliseconds(
                trace.inference_start_ns - trace.release_ns);
            result.wake_latency_ms.push_back(sample.wake_latency_ms);

            if (trace.inference_finish_ns == kUnsetTimestamp ||
                !sample.action_published) {
                ++result.incomplete_inferences;
            } else {
                sample.inference_service_ms = NanosecondsToMilliseconds(
                    trace.inference_finish_ns - trace.inference_start_ns);
                sample.publish_response_ms = NanosecondsToMilliseconds(
                    trace.published_ns - trace.release_ns);
                result.inference_service_ms.push_back(
                    sample.inference_service_ms);
                result.publish_response_ms.push_back(
                    sample.publish_response_ms);
            }
        }

        if (sample.action_published && !sample.action_applied) {
            ++result.superseded_actions;
        }
        if (sample.action_applied) {
            ++result.applied_count;
            sample.action_age_ms =
                NanosecondsToMilliseconds(trace.applied_ns - trace.release_ns);
            result.action_age_ms.push_back(sample.action_age_ms);
            if (sample.action_published) {
                sample.apply_wait_ms = NanosecondsToMilliseconds(
                    trace.applied_ns - trace.published_ns);
                result.apply_wait_ms.push_back(sample.apply_wait_ms);
            }
            sample.deadline_lateness_ms =
                std::max(0.0, sample.action_age_ms - period_ms);
            result.deadline_lateness_ms.push_back(sample.deadline_lateness_ms);
            if (sample.deadline_lateness_ms > 0.0) {
                ++result.deadline_misses;
            }
            if (previous_applied_ns != kUnsetTimestamp) {
                result.action_update_interval_ms.push_back(
                    NanosecondsToMilliseconds(
                        trace.applied_ns - previous_applied_ns));
            }
            previous_applied_ns = trace.applied_ns;
        }
        result.samples.push_back(sample);
    }
    return result;
}

void PrintStats(const std::string &label, const std::vector<double> &values) {
    if (values.empty()) {
        std::cout << std::left << std::setw(28) << label << " no samples\n";
        return;
    }
    const Stats stats = rl_benchmark::ComputeStats(values);
    std::cout << std::left << std::setw(28) << label;
    std::cout << " min=" << std::right << std::fixed << std::setprecision(3);
    std::cout << stats.min << " avg=" << stats.avg;
    std::cout << " std=" << stats.std_dev << " p50=" << stats.p50;
    std::cout << " p95=" << stats.p95 << " p99=" << stats.p99;
    if (stats.has_p999) {
        std::cout << " p99.9=" << stats.p999;
    }
    if (stats.has_p9999) {
        std::cout << " p99.99=" << stats.p9999;
    }
    std::cout << " max=" << stats.max << " ms\n";
}

void WriteMaybe(std::ostream &output, double value) {
    if (std::isfinite(value)) {
        output << value;
    }
}

void WriteCsv(const std::string &path, const std::string &policy_name,
    const RLConfig &config, const Options &options, double control_dt,
    bool reference_loop_override, const std::string &affinity_before_init,
    const std::string &affinity_after_init,
    const rl_policy::InferenceRuntimeInfo &runtime, const RuntimeResult &result,
    const ControlLoopSamples &control,
    const std::vector<double> &final_target_position) {
    if (path.empty()) {
        return;
    }
    auto output = rl_benchmark::OpenCsv(path);
    const std::string adapter = config.policy_adapter.Enabled()
        ? config.policy_adapter.type
        : "none";
    output << "# benchmark=humanoid_state_rl_runtime\n";
    output << "# schema_version=2\n";
    output << "# policy=" << policy_name << "\n";
    output << "# model=" << config.policy.model_path << "\n";
    output << "# adapter=" << adapter << "\n";
    output << "# robot_state=synthetic_static\n";
    output << "# runtime_path=FSM+StateRL+ThreadLoop\n";
    output << "# excluded=transport,joint_bus,actuator_closed_loop\n";
    output << "# build_type=" << RL_BENCHMARK_BUILD_TYPE << "\n";
    output << "# cxx_flags=" << RL_BENCHMARK_CXX_FLAGS << "\n";
    output << "# mode=periodic\n";
    output << "# control_dt=" << control_dt << "\n";
    output << "# rl_dt=" << config.rl_dt << "\n";
    output << "# hz=" << options.hz << "\n";
    output << "# warmup=" << options.warmup << "\n";
    output << "# rounds=" << options.rounds << "\n";
    output << "# reference_loop_override=";
    output << (reference_loop_override ? "true" : "false") << "\n";
    output << "# provider_requested=" << runtime.requested_provider << "\n";
    output << "# provider_initialized=" << runtime.initialized_provider << "\n";
    output << "# provider_status=" << runtime.provider_status << "\n";
    output << "# threads=" << options.threads << "\n";
    output << "# ep_threads_requested=" << runtime.ep_threads << "\n";
    output << "# ort_spinning=" << (runtime.ort_spinning ? "on" : "off");
    output << "\n";
    output << "# affinity_requested=" << options.affinity << "\n";
    output << "# affinity_effective_before_init=" << affinity_before_init;
    output << "\n";
    output << "# affinity_effective_after_init=" << affinity_after_init;
    output << "\n";
    output << "# affinity_ep_requested=" << runtime.affinity << "\n";
    output << "# measure_start_delay_ms=" << options.measure_start_delay_ms;
    output << "\n";
    output << "# release_jitter_definition=";
    output << "inter_release_interval_minus_rl_dt\n";
    output << "# planned_releases=" << options.rounds << "\n";
    output << "# inference_count=" << result.inference_count << "\n";
    output << "# applied_count=" << result.applied_count << "\n";
    output << "# coalesced_releases=" << result.coalesced_releases << "\n";
    output << "# incomplete_inferences=" << result.incomplete_inferences << "\n";
    output << "# superseded_actions=" << result.superseded_actions << "\n";
    output << "# deadline_misses=" << result.deadline_misses << "\n";
    output << "# control_ticks=" << control.ticks << "\n";
    output << "# control_ticks_reusing_cached_action=";
    output << control.ticks_reusing_cached_action << "\n";
    output << "# max_release_lag=" << control.max_release_lag << "\n";
    rl_benchmark::WriteVectorEvidence(
        output, "final_target_position", final_target_position);
    output << "iteration,release_sequence,inference_started,action_published,";
    output << "action_applied,release_jitter_ms,wake_latency_ms,service_ms,";
    output << "publish_response_ms,apply_wait_ms,response_ms,";
    output << "deadline_lateness_ms\n";
    for (std::size_t index = 0; index < result.samples.size(); ++index) {
        const RuntimeSample &sample = result.samples[index];
        output << index << ',' << sample.release_sequence << ',';
        output << (sample.inference_started ? 1 : 0) << ',';
        output << (sample.action_published ? 1 : 0) << ',';
        output << (sample.action_applied ? 1 : 0) << ',';
        WriteMaybe(output, sample.release_jitter_ms);
        output << ',';
        WriteMaybe(output, sample.wake_latency_ms);
        output << ',';
        WriteMaybe(output, sample.inference_service_ms);
        output << ',';
        WriteMaybe(output, sample.publish_response_ms);
        output << ',';
        WriteMaybe(output, sample.apply_wait_ms);
        output << ',';
        WriteMaybe(output, sample.action_age_ms);
        output << ',';
        WriteMaybe(output, sample.deadline_lateness_ms);
        output << '\n';
    }
}

void PrintVerbose(const RuntimeResult &result) {
    for (const RuntimeSample &sample : result.samples) {
        std::cout << "release=" << sample.release_sequence;
        std::cout << " infer=" << (sample.inference_started ? "yes" : "no");
        std::cout << " published=";
        std::cout << (sample.action_published ? "yes" : "no");
        std::cout << " applied=" << (sample.action_applied ? "yes" : "no");
        if (std::isfinite(sample.wake_latency_ms)) {
            std::cout << " wake_ms=" << sample.wake_latency_ms;
            std::cout << " service_ms=" << sample.inference_service_ms;
            std::cout << " response_ms=" << sample.action_age_ms;
        }
        std::cout << '\n';
    }
}

}  // namespace

int main(int argc, char *argv[]) {
    if (argc < 4) {
        PrintUsage(argv[0]);
        return 1;
    }

    try {
        const std::string yaml_path = argv[1];
        const std::string policy_name = argv[2];
        const std::string robot_dir = argv[3];
        RLConfig config = behavior_manager::LoadRLStateConfig(
            yaml_path, policy_name, robot_dir);

        bool reference_loop_override = false;
        Options options = ParseRuntimeOptions(
            argc, argv, 1.0 / config.rl_dt, &reference_loop_override);
        config.rl_dt = 1.0 / options.hz;
        if (reference_loop_override && !config.policy_adapter.Enabled()) {
            throw std::invalid_argument(
                "--reference-loop requires a configured policy_adapter");
        }
        if (reference_loop_override) {
            config.policy_adapter.loop = true;
            config.policy_adapter.loop_pause = 0.0;
        }

        const auto yaml = robot_base::YamlFile::Load(yaml_path);
        const double control_dt =
            yaml.Read<double>("behavior_manager.control_dt").value_or(0.02);
        if (!std::isfinite(control_dt) || control_dt <= 0.0) {
            throw std::runtime_error(
                "behavior_manager.control_dt must be positive");
        }
        config.infer_thread_cfg =
            robot_base::ThreadLoop::FromYaml(yaml, "rl_infer");

        const std::string affinity_before_init =
            rl_benchmark::ApplyAndGetAffinity(options.affinity);
        const std::string requested_ep_affinity = options.ep_affinity.empty()
            ? options.affinity
            : options.ep_affinity;
        config.policy.runtime.provider = options.provider;
        config.policy.runtime.threads = options.threads;
        config.policy.runtime.affinity = rl_benchmark::EpAffinityFromCpuList(
            requested_ep_affinity, options.threads,
            rl_benchmark::ShouldConfigureSpaceMITProvider(options.provider));
        config.policy.runtime.ep_dump_subgraphs = options.ep_dump_subgraphs;
        config.policy.runtime.ep_profile_prefix = options.ep_profile_prefix;
        config.policy.runtime.ort_spinning = options.ort_spinning;

        RuntimeCollector collector(
            static_cast<std::size_t>(options.warmup + options.rounds));
        config.runtime_observer = &collector;

        robot_base::RobotData robot = MakeSyntheticRobot(config);
        robot_base::Command command;
        command.vx = static_cast<float>(config.command_init[0]);
        command.vy = static_cast<float>(config.command_init[1]);
        command.wz = static_cast<float>(config.command_init[2]);
        behavior_manager::ControlOutput output = MakeInitialOutput(config);

        StateRLSession session(config, &robot, &command, &output);
        const auto init_start = RLRuntimeClock::now();
        session.Start();
        const double init_ms =
            rl_benchmark::ToMilliseconds(RLRuntimeClock::now() - init_start);
        if (!collector.RuntimeReady()) {
            throw std::runtime_error(
                "StateRL did not initialize policy runtime");
        }
        const std::string affinity_after_init =
            rl_benchmark::ApplyAndGetAffinity("");
        const auto runtime = collector.RuntimeInfo();

        const std::string adapter = config.policy_adapter.Enabled()
            ? config.policy_adapter.type
            : "none";
        const std::string host_affinity = options.affinity.empty()
            ? "not-set"
            : options.affinity;
        const std::string reference_loop = reference_loop_override
            ? "benchmark override"
            : "YAML setting";
        std::cout << "\nHumanoid StateRL asynchronous runtime benchmark\n";
        std::cout << "Policy:      " << policy_name << "\n";
        std::cout << "Model:       " << config.policy.model_path << "\n";
        std::cout << "Adapter:     " << adapter << "\n";
        std::cout << "Semantics:   synthetic sensors driving production ";
        std::cout << "FSM/StateRL/ThreadLoop; ";
        std::cout << "no transport/joint bus/actuator\n";
        std::cout << "Control/RL:  " << control_dt << " s / ";
        std::cout << config.rl_dt << " s (" << options.hz << " Hz)\n";
        std::cout << "Provider:    requested=" << runtime.requested_provider;
        std::cout << ", initialized=" << runtime.initialized_provider << "\n";
        std::cout << "EP status:   " << runtime.provider_status << "\n";
        std::cout << "Threads:     ORT intra=" << runtime.ort_intra_threads;
        std::cout << ", ORT inter=" << runtime.ort_inter_threads;
        std::cout << ", EP requested=" << runtime.ep_threads << "\n";
        std::cout << "ORT spin:    ";
        std::cout << (runtime.ort_spinning ? "on" : "off") << "\n";
        std::cout << "Host affinity: requested=" << host_affinity;
        std::cout << ", before-init=" << affinity_before_init;
        std::cout << ", after-init=" << affinity_after_init << "\n";
        std::cout << "Inference thread: cpu=";
        std::cout << config.infer_thread_cfg.cpu_id;
        std::cout << ", sched=" << config.infer_thread_cfg.sched;
        std::cout << ", priority=" << config.infer_thread_cfg.priority << "\n";
        std::cout << "Reference loop: " << reference_loop << "\n";
        std::cout << "Init time:   " << init_ms << " ms\n";
        std::cout << "Warmup/test: " << options.warmup << '/';
        std::cout << options.rounds << " releases\n";
        rl_benchmark::PrintBuildMetadata();

        auto control_time = RLRuntimeClock::now();
        if (options.warmup > 0) {
            std::cout << "Warmup...\n" << std::flush;
            const std::uint64_t warmup_target =
                static_cast<std::uint64_t>(options.warmup);
            const auto warmup_timeout = RLRuntimeClock::now() +
                std::chrono::duration_cast<RLRuntimeClock::duration>(
                    std::chrono::duration<double>(
                        std::max(30.0, options.warmup * config.rl_dt * 3.0)));
            DriveUntilRelease(&session, &collector, &robot, control_dt,
                config.rl_dt, warmup_target, warmup_timeout, &control_time,
                nullptr);
            DrainUntilApplied(&session, &collector, control_dt, config.rl_dt,
                warmup_target, warmup_timeout, &control_time, nullptr);
        }

        const std::uint64_t first_measured_sequence =
            collector.MaximumRelease() + 1;
        const std::uint64_t final_measured_sequence = first_measured_sequence +
            static_cast<std::uint64_t>(options.rounds) - 1;
        rl_benchmark::BeginMeasuredRegion(options);
        control_time = RLRuntimeClock::now();
        ControlLoopSamples control_samples;
        control_samples.interval_error_ms.reserve(
            static_cast<std::size_t>(options.rounds) *
            static_cast<std::size_t>(
                std::max(1.0, std::ceil(config.rl_dt / control_dt))));
        const auto measure_timeout = RLRuntimeClock::now() +
            std::chrono::duration_cast<RLRuntimeClock::duration>(
                std::chrono::duration<double>(
                    std::max(30.0, options.rounds * config.rl_dt * 3.0)));
        DriveUntilRelease(&session, &collector, &robot, control_dt,
            config.rl_dt, final_measured_sequence, measure_timeout,
            &control_time, &control_samples);
        DrainUntilApplied(&session, &collector, control_dt, config.rl_dt,
            final_measured_sequence, measure_timeout, &control_time,
            &control_samples);
        rl_benchmark::EndMeasuredRegion();

        const std::vector<double> final_target_position = output.target_pos;
        session.Stop();
        if (collector.OverflowEvents() != 0) {
            throw std::runtime_error("StateRL trace capacity was exceeded");
        }
        const RuntimeResult result = AnalyzeRuntime(
            collector, first_measured_sequence, options.rounds, config.rl_dt);

        PrintStats("Control interval error", control_samples.interval_error_ms);
        PrintStats("RL release interval jitter", result.release_jitter_ms);
        PrintStats("Inference wake latency", result.wake_latency_ms);
        PrintStats("Inference service", result.inference_service_ms);
        PrintStats("Release-to-publish", result.publish_response_ms);
        PrintStats("Publish-to-apply", result.apply_wait_ms);
        PrintStats("Action age", result.action_age_ms);
        PrintStats("Action update interval", result.action_update_interval_ms);
        PrintStats("Deadline lateness", result.deadline_lateness_ms);

        const std::uint64_t unapplied =
            static_cast<std::uint64_t>(options.rounds) - result.applied_count;
        const std::uint64_t failed_releases =
            unapplied + result.deadline_misses;
        std::cout << "StateRL releases: planned=" << options.rounds;
        std::cout << ", inferred=" << result.inference_count;
        std::cout << ", applied=" << result.applied_count;
        std::cout << ", coalesced=" << result.coalesced_releases;
        std::cout << ", incomplete=" << result.incomplete_inferences;
        std::cout << ", superseded=" << result.superseded_actions << "\n";
        std::cout << "Periodic result (" << options.hz;
        std::cout << " Hz): deadline_misses=" << result.deadline_misses;
        std::cout << ", unapplied=" << unapplied;
        std::cout << ", failed_releases=" << failed_releases << '/';
        std::cout << options.rounds << "\n";
        std::cout << "Control ticks: total=" << control_samples.ticks;
        std::cout << ", reusing_cached_action=";
        std::cout << control_samples.ticks_reusing_cached_action;
        std::cout << ", max_release_lag=" << control_samples.max_release_lag;
        std::cout << "\n";
        if (options.rounds < 100000) {
            std::cout << "P99.99 not reported: use at least 100000 rounds ";
            std::cout << "with --csv.\n";
        }

        WriteCsv(options.csv_path, policy_name, config, options, control_dt,
            reference_loop_override, affinity_before_init, affinity_after_init,
            runtime, result, control_samples, final_target_position);
        if (!options.csv_path.empty()) {
            std::cout << "Raw CSV: " << options.csv_path << "\n";
        }
        if (options.verbose_after_timing) {
            PrintVerbose(result);
        }
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "[benchmark_humanoid_policy] " << error.what() << '\n';
        return 2;
    }
}
