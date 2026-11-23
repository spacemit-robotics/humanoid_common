/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file planner_host.cpp
 * @brief 规划器宿主实现：后台任务线程与跟踪侧状态机
 */

#include "planner_host.h"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "planner_catalog.h"
#include "runtime_logger.h"

namespace behavior_manager {
namespace motion_planner {
namespace {

using Phase = robot_base::PlannerStatus::Phase;

constexpr std::chrono::milliseconds kWorkerPoll{50};
constexpr std::chrono::milliseconds kPreloadPoll{5};
constexpr double kRateWindowS = 2.0;
// An estimated preparation never reports completion before the planner is ready.
constexpr float kEstimatedProgressLimit = 0.95F;

// Durations of completed loads in this process, by planner name. A host is
// created per RL session, so the measurements outlive it. Used on the tracker
// side when a start begins and when it completes.
class PrepareTimes {
public:
    double Expected(const std::string &planner, double configured) {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto measured = measured_.find(planner);
        return measured != measured_.end() ? measured->second : configured;
    }

    void Record(const std::string &planner, double duration_s) {
        if (!std::isfinite(duration_s) || duration_s <= 0.0) return;
        std::lock_guard<std::mutex> lock(mutex_);
        measured_[planner] = duration_s;
    }

private:
    std::mutex mutex_;
    std::map<std::string, double> measured_;
};

PrepareTimes &MeasuredPrepareTimes() {
    static PrepareTimes times;
    return times;
}

// Planner-reported values reach the status packet, which rejects non-finite fields.
float NonNegative(double value) {
    return std::isfinite(value) && value > 0.0 ? static_cast<float>(value) : 0.0F;
}

float UnitInterval(float value) {
    return std::isfinite(value) ? std::clamp(value, 0.0F, 1.0F) : 0.0F;
}

// Job mailbox stages. The tracker side posts (IDLE -> POSTED) and collects
// (DONE -> IDLE); the worker completes (POSTED -> DONE).
constexpr int kStageIdle = 0;
constexpr int kStagePosted = 1;
constexpr int kStageDone = 2;

enum class JobKind { NONE, LOAD, START, GENERATE, PREPARE, STOP };

const char *PhaseName(Phase phase) {
    switch (phase) {
        case Phase::IDLE: return "IDLE";
        case Phase::LOADING: return "LOADING";
        case Phase::WARMING: return "WARMING";
        case Phase::READY: return "READY";
        case Phase::GENERATING: return "GENERATING";
        case Phase::RUNNING: return "RUNNING";
        case Phase::STOPPING: return "STOPPING";
        case Phase::FAILED: return "FAILED";
    }
    return "UNKNOWN";
}

struct Job {
    JobKind kind = JobKind::NONE;
    int index = -1;        // LOAD: planner to create
    bool destroy = false;  // STOP: also release the planner
    double time_s = 0.0;   // START
    std::shared_ptr<const PlannerOutput> publication;  // PREPARE
    std::string argument;  // GENERATE
    robot_base::RobotData robot;
    std::shared_ptr<std::atomic<bool>> cancelled;  // GENERATE
};

// State of the worker-owned planner after a job, plus the job's own outcome.
struct JobResult {
    InstallReference install;
    std::string error;
    Planner *planner = nullptr;
    int loaded_index = -1;
    bool started = false;
    std::shared_ptr<const PlannerOutput> output;
};

}  // namespace

HostConfig LoadHostConfig(const std::string &yaml_path,
        const std::string &policy_name,
        const std::string &robot_dir,
        double reference_fps) {
    const robot_base::YamlFile yaml = robot_base::YamlFile::Load(yaml_path);
    const std::string adapter_base = runtime_config::PolicyBase(policy_name) + ".policy_adapter";
    const std::string auto_start =
        yaml.Read<std::string>(adapter_base + ".auto_planner").value_or("");
    const auto entries = runtime_config::LoadPolicyPlanners(yaml, policy_name);

    HostConfig host;
    if (entries.empty()) {
        if (!auto_start.empty()) {
            throw std::runtime_error("[motion_planner] " + adapter_base +
                ".auto_planner 需要同时在 planners 中列出该规划器");
        }
        return host;
    }
    const int num_dof = yaml.Read<int>("robot_base.num_dof").value_or(0);
    if (num_dof <= 0) {
        throw std::runtime_error("[motion_planner] 使用规划器需要配置 robot_base.num_dof");
    }

    host.action_environments = runtime_config::LoadActionEnvironments(
        yaml, yaml.Read<std::string>(adapter_base + ".catalog").value_or(""));
    int preloaded = 0;
    for (const auto &entry : entries) {
        const PlannerType *type = FindPlannerType(entry.type);
        if (!type) {
            std::string registered;
            for (const auto &name : RegisteredPlannerTypes()) {
                registered += (registered.empty() ? "" : ", ") + name;
            }
            throw std::runtime_error("[motion_planner] " + entry.base + ".type 未注册: " +
                entry.type + "（已注册: " + (registered.empty() ? "无" : registered) + "）");
        }
        // The operator service reads the category from YAML without access to
        // the type registry, so the two must agree.
        if (entry.category != CategoryName(type->category)) {
            throw std::runtime_error("[motion_planner] " + entry.base + ".category 为 " +
                entry.category + "，但类型 " + entry.type + " 注册为 " +
                CategoryName(type->category) + "；请在 YAML 中写明 category: " +
                CategoryName(type->category));
        }

        PlannerConfig config;
        config.name = entry.name;
        config.type = entry.type;
        config.category = type->category;
        config.environments = entry.environments;
        config.minimum_command = {entry.limits.min_vx, entry.limits.min_vy, entry.limits.min_wz};
        config.maximum_command = {entry.limits.max_vx, entry.limits.max_vy, entry.limits.max_wz};
        config.preload = yaml.Read<bool>(entry.base + ".preload").value_or(false);
        config.keep_loaded = yaml.Read<bool>(entry.base + ".keep_loaded").value_or(true);
        config.stop_timeout_s = yaml.Read<double>(entry.base + ".stop_timeout").value_or(10.0);
        if (!std::isfinite(config.stop_timeout_s) || config.stop_timeout_s < 0.0) {
            throw std::runtime_error("[motion_planner] " + entry.base +
                ".stop_timeout 必须为非负有限数");
        }
        config.prepare_time_s = yaml.Read<double>(entry.base + ".prepare_time").value_or(0.0);
        if (!std::isfinite(config.prepare_time_s) || config.prepare_time_s < 0.0) {
            throw std::runtime_error("[motion_planner] " + entry.base +
                ".prepare_time 必须为非负有限数");
        }
        preloaded += config.preload ? 1 : 0;

        config.context.name = entry.name;
        config.context.yaml_path = yaml_path;
        config.context.config_base = entry.base;
        config.context.robot_dir = robot_dir;
        config.context.num_dof = num_dof;
        config.context.reference_fps = reference_fps;
        const std::string thread_name =
            yaml.Read<std::string>(entry.base + ".thread").value_or("motion_planner");
        config.context.thread = robot_base::ThreadLoop::FromYaml(yaml, thread_name);
        if (config.context.thread.cpu_id < 0 && config.context.thread.cpu_affinity.empty()) {
            config.context.thread.cpu_affinity =
                robot_base::ThreadLoop::Background(thread_name).cpu_affinity;
        }
        host.planners.push_back(std::move(config));
    }
    if (preloaded > 1) {
        throw std::runtime_error("[motion_planner] " +
            runtime_config::PolicyPlannerList(policy_name) + " 中至多一个规划器可设 preload");
    }
    if (!auto_start.empty()) {
        const auto planner = std::find_if(host.planners.begin(), host.planners.end(),
            [&](const PlannerConfig &config) { return config.name == auto_start; });
        if (planner == host.planners.end() || planner->category != Category::REALTIME) {
            throw std::runtime_error("[motion_planner] " + adapter_base +
                ".auto_planner 必须是 planners 中列出的实时规划器: " + auto_start);
        }
        host.auto_start = auto_start;
    }
    return host;
}

class PlannerHost::Impl {
public:
    explicit Impl(HostConfig config) : config_(std::move(config)) {
        if (config_.planners.empty()) {
            throw std::invalid_argument("[motion_planner] 规划器宿主至少需要一个规划器");
        }
        num_dof_ = config_.planners.front().context.num_dof;
        for (std::size_t i = 0; i < config_.planners.size(); ++i) {
            const auto &planner = config_.planners[i];
            if (planner.name.empty() || planner.context.num_dof != num_dof_ || num_dof_ <= 0 ||
                Find(planner.name) != static_cast<int>(i)) {
                throw std::invalid_argument(
                    "[motion_planner] 规划器名为空或重复，或关节数不一致: " + planner.name);
            }
        }
        if (!config_.auto_start.empty() && Find(config_.auto_start) < 0) {
            throw std::invalid_argument(
                "[motion_planner] auto_start 不在规划器列表中: " + config_.auto_start);
        }
        // Sized once, so posting a job from the tracker thread only copies values.
        job_.robot = robot_base::RobotData::Create(num_dof_);
        job_.argument.reserve(robot_base::kPlannerArgumentMaxLength);
        argument_.reserve(robot_base::kPlannerArgumentMaxLength);
        worker_ = std::thread(&Impl::WorkerLoop, this);
        try {
            Preload();
        } catch (...) {
            Shutdown();
            throw;
        }
        Reset();
    }

    ~Impl() { Shutdown(); }

    void Reset() {
        Collect();
        CancelGeneration();
        start_pending_ = false;
        running_ = false;
        releasing_ = false;
        failed_ = false;
        auto_start_pending_ = !config_.auto_start.empty();
        last_sequence_ = 0;
        environment_ = robot_base::RuntimeEnvironment::UNKNOWN;
        time_s_ = 0.0;
        output_.reset();
        seen_output_.reset();
        status_ = {};
        if (loaded_ >= 0) status_.planner = config_.planners[loaded_].name;
        status_.phase = CurrentPhase();
    }

    void Suspend() {
        Collect();
        CancelGeneration();
        start_pending_ = false;
        running_ = false;
        releasing_ = false;
        auto_start_pending_ = false;
        output_.reset();
        seen_output_.reset();
        status_.progress = 0.0F;
        // A job still in flight is left to finish; the planner is then released
        // by the next Step() or by the destructor.
        if (job_kind_ == JobKind::NONE && loaded_ >= 0) PostStop(true);
        UpdatePhase();
    }

    void Step(const robot_base::RobotData &robot,
            const MotionCommand &command,
            double time_s,
            const robot_base::PlannerRequest &request,
            const TrackerState &tracker) {
        environment_ = robot.environment;
        time_s_ = time_s;
        Collect();
        HandleRequest(request, tracker.busy);
        if (auto_start_pending_ && !tracker.busy) AutoStart();
        if (running_) UpdateRunning(robot, command, tracker.playback);
        if (output_ && !output_->action.empty() &&
            !runtime_config::ActionEnabled(config_.action_environments, output_->action, environment_)) {
            Fail("action " + output_->action + " is not enabled in the " +
                robot_base::RuntimeEnvironmentName(environment_) + " environment");
        }
        Advance(robot);
        UpdatePreparation();
        UpdatePhase();
    }

    const std::shared_ptr<const PlannerOutput> &Output() const { return output_; }

    const InstallReference &PreparedReference() const { return install_; }

    bool OutputFinite() const {
        return output_ && active_ >= 0 && Active().category == Category::OFFLINE;
    }

    bool Engaged() const { return start_pending_ || running_; }

    void RequestRelease() {
        auto_start_pending_ = false;
        if (start_pending_) {
            CancelStart();
        } else if (running_ && !releasing_) {
            BeginRelease();
        }
        UpdatePhase();
    }

    void Fail(const std::string &detail) {
        const std::string name = active_ >= 0 ? Active().name : status_.planner;
        CancelGeneration();
        failed_ = true;
        start_pending_ = false;
        running_ = false;
        releasing_ = false;
        output_.reset();
        seen_output_.reset();
        status_.progress = 0.0F;
        status_.detail = detail;
        runtime_logging::Log(runtime_logging::Level::kError,
            "motion planner " + name + " failed: " + detail, false);
        UpdatePhase();
    }

    const robot_base::PlannerStatus &Status() const { return status_; }

private:
    const PlannerConfig &Active() const { return config_.planners[active_]; }

    int Find(const std::string &name) const {
        for (std::size_t i = 0; i < config_.planners.size(); ++i) {
            if (config_.planners[i].name == name) return static_cast<int>(i);
        }
        return -1;
    }

    // ==================== 跟踪侧：请求处理 ====================

    void HandleRequest(const robot_base::PlannerRequest &request, bool tracker_busy) {
        using Operation = robot_base::PlannerRequest::Operation;
        if (request.operation == Operation::NONE || request.sequence == 0 ||
            request.sequence <= last_sequence_) {
            return;
        }
        last_sequence_ = request.sequence;
        status_.sequence = request.sequence;
        if (request.operation == Operation::STOP) {
            status_.request_accepted = true;
            status_.detail.clear();
            failed_ = false;
            auto_start_pending_ = false;
            if (start_pending_) {
                CancelStart();
            } else if (running_ && !releasing_) {
                BeginRelease();
            }
            return;
        }

        const int index = Find(request.planner);
        if (index < 0) {
            Reject("planner " + request.planner + " is not registered for this policy");
            return;
        }
        const auto &config = config_.planners[index];
        if (!config.environments.Allows(environment_)) {
            Reject("planner " + config.name + " is not enabled in the " +
                robot_base::RuntimeEnvironmentName(environment_) + " environment");
            return;
        }
        if (start_pending_ || running_) {
            if (index == active_ && !releasing_ && config.category == Category::REALTIME) {
                // Already starting or running: the request only updates the argument.
                status_.request_accepted = true;
                status_.detail.clear();
                if (request.argument != argument_) {
                    argument_ = request.argument;
                    ++argument_sequence_;
                }
                return;
            }
            Reject(releasing_ ? "planner " + Active().name + " is stopping"
                : "planner " + Active().name + " is active; stop it first");
            return;
        }
        if (tracker_busy) {
            Reject("the tracker is executing an action or switching policy");
            return;
        }
        auto_start_pending_ = false;
        Begin(index, request.argument);
        status_.request_accepted = true;
    }

    void Reject(const std::string &detail) {
        status_.request_accepted = false;
        status_.detail = detail;
    }

    void AutoStart() {
        auto_start_pending_ = false;
        if (start_pending_ || running_) return;
        const int index = Find(config_.auto_start);
        const auto &config = config_.planners[index];
        if (!config.environments.Allows(environment_)) {
            status_.detail = "auto-start skipped: planner " + config.name +
                " is not enabled in the " +
                robot_base::RuntimeEnvironmentName(environment_) + " environment";
            return;
        }
        Begin(index, std::string());
    }

    void Begin(int index, const std::string &argument) {
        ++start_id_;
        active_ = index;
        argument_ = argument;
        ++argument_sequence_;
        start_pending_ = true;
        failed_ = false;
        releasing_ = false;
        loading_ = false;
        expected_prepare_s_ = 0.0;
        generation_progress_.store(0.0F, std::memory_order_release);
        status_.planner = Active().name;
        status_.detail.clear();
        status_.progress = 0.0F;
        status_.plan_count = 0;
        status_.plan_rate_hz = 0.0F;
        status_.last_plan_ms = 0.0F;
        status_.reference_remaining_s = 0.0F;
    }

    void CancelStart() {
        start_pending_ = false;
        status_.progress = 0.0F;
        CancelGeneration();
    }

    void CancelGeneration() {
        if (cancelled_) cancelled_->store(true, std::memory_order_release);
    }

    void BeginRelease() {
        releasing_ = true;
        release_started_s_ = time_s_;
    }

    void Release() {
        running_ = false;
        releasing_ = false;
        output_.reset();
        seen_output_.reset();
        status_.progress = 0.0F;
        status_.reference_remaining_s = 0.0F;
    }

    // ==================== 跟踪侧：运行中的规划器 ====================

    void UpdateRunning(const robot_base::RobotData &robot,
            const MotionCommand &command,
            const PlaybackState &playback) {
        const PlannerConfig &config = Active();
        if (config.category == Category::OFFLINE) {
            status_.progress = playback.active ? UnitInterval(playback.progress) : 0.0F;
            if (releasing_ || (playback.active && playback.finished)) Release();
            return;
        }

        auto *planner = static_cast<RealtimePlanner *>(planner_);
        try {
            const auto &lo = config.minimum_command;
            const auto &hi = config.maximum_command;
            const MotionCommand limited{runtime_config::ClampVelocity(command.vx, lo.vx, hi.vx),
                runtime_config::ClampVelocity(command.vy, lo.vy, hi.vy),
                runtime_config::ClampVelocity(command.wz, lo.wz, hi.wz)};
            const PlannerInput input{robot, releasing_ ? MotionCommand{} : limited, time_s_,
                playback, argument_, argument_sequence_, releasing_};
            planner->Update(input);
            auto output = planner->Output();
            if (output && output != seen_output_ && job_kind_ == JobKind::NONE) {
                seen_output_ = output;
                if (output->action.empty()) {
                    job_.kind = JobKind::PREPARE;
                    job_.publication = std::move(output);
                    Post();
                } else {
                    const auto error = ValidateOutput(*output, num_dof_);
                    if (!error.empty()) {
                        Fail("published an invalid reference: " + error);
                        return;
                    }
                    output_ = std::move(output);
                }
            }
            UpdateStats(planner->Stats(), playback);
            if (!releasing_) return;
            const bool timed_out = config.stop_timeout_s > 0.0 &&
                time_s_ - release_started_s_ >= config.stop_timeout_s;
            if (!timed_out && !planner->Idle()) return;
            if (timed_out) {
                status_.detail = "planner " + config.name +
                    " did not settle before the stop timeout; reference released";
                runtime_logging::Log(runtime_logging::Level::kWarning,
                    "motion planner: " + status_.detail, false);
            }
            Release();
        } catch (const std::exception &error) {
            Fail(error.what());
        }
    }

    void UpdateStats(const PlannerStats &stats, const PlaybackState &playback) {
        status_.plan_count = static_cast<uint32_t>(
            std::min<uint64_t>(stats.plan_count, UINT32_MAX));
        status_.last_plan_ms = NonNegative(stats.last_plan_ms);
        if (time_s_ - rate_window_start_s_ >= kRateWindowS) {
            status_.plan_rate_hz = stats.plan_count >= rate_window_count_
                ? NonNegative((stats.plan_count - rate_window_count_) /
                    (time_s_ - rate_window_start_s_))
                : 0.0F;
            rate_window_start_s_ = time_s_;
            rate_window_count_ = stats.plan_count;
        }
        if (output_ && output_->action.empty()) {
            const auto &trajectory = output_->trajectory;
            status_.reference_remaining_s = NonNegative(
                trajectory.start_time_s + trajectory.Duration() - time_s_);
            status_.progress = 0.0F;
        } else {
            status_.reference_remaining_s = 0.0F;
            if (output_) status_.progress = playback.active ? UnitInterval(playback.progress) : 0.0F;
        }
    }

    // ==================== 跟踪侧：后台任务 ====================

    // Chooses the next background job once the previous one has been collected.
    void Advance(const robot_base::RobotData &robot) {
        if (job_kind_ != JobKind::NONE) return;
        if (start_pending_) {
            if (loaded_ != active_) {
                if (loaded_ >= 0) {
                    PostStop(true);  // Another planner occupies the slot.
                } else {
                    PostLoad(active_);
                }
            } else if (Active().category == Category::OFFLINE) {
                PostGenerate(robot);
            } else if (started_) {
                PostStop(false);  // Left running by a cancelled start; restart cleanly.
            } else {
                PostStart(robot);
            }
            return;
        }
        if (running_ || loaded_ < 0) return;
        const bool destroy = failed_ || !config_.planners[loaded_].keep_loaded;
        if (started_ || destroy) PostStop(destroy);
    }

    void PostLoad(int index) {
        job_.kind = JobKind::LOAD;
        job_.index = index;
        warming_.store(false, std::memory_order_release);
        if (start_pending_) {
            const PlannerConfig &config = config_.planners[index];
            loading_ = true;
            prepare_started_s_ = time_s_;
            expected_prepare_s_ = MeasuredPrepareTimes().Expected(config.name, config.prepare_time_s);
        }
        Post();
    }

    void PostStart(const robot_base::RobotData &robot) {
        job_.kind = JobKind::START;
        job_.robot = robot;
        job_.time_s = time_s_;
        Post();
    }

    void PostGenerate(const robot_base::RobotData &robot) {
        job_.kind = JobKind::GENERATE;
        job_.robot = robot;
        job_.argument = argument_;
        cancelled_ = std::make_shared<std::atomic<bool>>(false);
        job_.cancelled = cancelled_;
        generation_progress_.store(0.0F, std::memory_order_release);
        Post();
    }

    void PostStop(bool destroy) {
        job_.kind = JobKind::STOP;
        job_.destroy = destroy;
        Post();
    }

    void Post() {
        job_start_id_ = start_id_;
        job_kind_ = job_.kind;
        stage_.store(kStagePosted, std::memory_order_release);
        // Without the worker mutex: a wakeup lost to the race costs one kWorkerPoll.
        worker_cv_.notify_one();
    }

    // Takes over the result of a finished job; never waits for the worker.
    void Collect() {
        if (job_kind_ == JobKind::NONE ||
            stage_.load(std::memory_order_acquire) != kStageDone) {
            return;
        }
        const JobKind kind = job_kind_;
        job_kind_ = JobKind::NONE;
        JobResult &result = result_;  // Retired buffers are destroyed by the next worker job.
        job_.cancelled.reset();
        stage_.store(kStageIdle, std::memory_order_release);

        planner_ = result.planner;
        loaded_ = result.loaded_index;
        started_ = result.started;
        // Always reconcile the worker-owned slot, but never publish an old request's result.
        if (job_start_id_ != start_id_) return;
        if (kind == JobKind::PREPARE) {
            if (!running_) return;
            if (!result.error.empty()) {
                Fail(result.error);
            } else {
                output_ = result.output;
                install_.swap(result.install);
            }
            return;
        }
        if (!result.error.empty()) {
            const std::string name = kind == JobKind::LOAD
                ? config_.planners[job_.index].name : status_.planner;
            const char *step = kind == JobKind::LOAD ? "load" : kind == JobKind::START ? "start"
                : kind == JobKind::GENERATE ? "generate" : "stop";
            if (start_pending_ && kind != JobKind::STOP) {
                Fail(std::string("failed to ") + step + ": " + result.error);
            } else {
                runtime_logging::Log(runtime_logging::Level::kWarning,
                    "motion planner " + name + " failed to " + step + ": " + result.error, false);
            }
            return;
        }
        if (!start_pending_) return;
        // A load ends when the planner can be used: a realtime planner after its
        // first start, an offline planner once it has been created.
        const bool offline = Active().category == Category::OFFLINE;
        if (loading_ && (kind == JobKind::START || (kind == JobKind::LOAD && offline))) {
            loading_ = false;
            MeasuredPrepareTimes().Record(Active().name, time_s_ - prepare_started_s_);
        }
        if (kind == JobKind::START) {
            start_pending_ = false;
            running_ = true;
            rate_window_start_s_ = time_s_;
            rate_window_count_ = 0;
        } else if (kind == JobKind::GENERATE && result.output) {
            start_pending_ = false;
            running_ = true;
            output_ = result.output;
            install_.swap(result.install);
        }
    }

    // Progress shown while a start is pending: reported by an offline planner
    // during generation, otherwise estimated from the expected load time.
    void UpdatePreparation() {
        if (!start_pending_) return;
        if (job_start_id_ != start_id_) return;
        if (job_kind_ == JobKind::GENERATE) {
            status_.progress = UnitInterval(generation_progress_.load(std::memory_order_acquire));
        } else if (loading_ && expected_prepare_s_ > 0.0) {
            status_.progress = std::min(kEstimatedProgressLimit, UnitInterval(
                static_cast<float>((time_s_ - prepare_started_s_) / expected_prepare_s_)));
        } else {
            status_.progress = 0.0F;
        }
    }

    Phase CurrentPhase() const {
        if (failed_) return Phase::FAILED;
        if (start_pending_) {
            const bool offline = Active().category == Category::OFFLINE;
            if (job_kind_ != JobKind::NONE && job_start_id_ != start_id_) return Phase::LOADING;
            if (job_kind_ == JobKind::GENERATE || (offline && loaded_ == active_)) {
                return Phase::GENERATING;
            }
            if (job_kind_ == JobKind::START || (job_kind_ == JobKind::LOAD &&
                    warming_.load(std::memory_order_acquire))) {
                return Phase::WARMING;
            }
            return Phase::LOADING;
        }
        if (running_) {
            if (releasing_) return Phase::STOPPING;
            return output_ ? Phase::RUNNING : Phase::WARMING;
        }
        if (job_kind_ != JobKind::NONE) return Phase::STOPPING;
        if (loaded_ < 0) return Phase::IDLE;
        // A planner left started, or one that is not kept, is stopped by Advance().
        return started_ || !config_.planners[loaded_].keep_loaded ? Phase::STOPPING : Phase::READY;
    }

    void UpdatePhase() {
        const Phase phase = CurrentPhase();
        if (phase != status_.phase) {
            runtime_logging::Log(runtime_logging::Level::kInfo,
                "motion planner " + (status_.planner.empty() ? std::string("-") : status_.planner) +
                    ": " + PhaseName(status_.phase) + " -> " + PhaseName(phase), false);
        }
        status_.phase = phase;
    }

    // ==================== 构造与销毁 ====================

    void Preload() {
        for (std::size_t i = 0; i < config_.planners.size(); ++i) {
            if (!config_.planners[i].preload) continue;
            PostLoad(static_cast<int>(i));
            while (stage_.load(std::memory_order_acquire) != kStageDone) {
                std::this_thread::sleep_for(kPreloadPoll);
            }
            const std::string error = result_.error;
            Collect();
            if (!error.empty()) {
                throw std::runtime_error("[motion_planner] 规划器 " + config_.planners[i].name +
                    " 预加载失败: " + error);
            }
        }
    }

    void Shutdown() {
        CancelGeneration();
        quit_.store(true, std::memory_order_release);
        worker_cv_.notify_all();
        if (worker_.joinable()) worker_.join();
        ReleaseOwned();
        planner_ = nullptr;
        loaded_ = -1;
        started_ = false;
    }

    // ==================== 后台线程 ====================

    void WorkerLoop() {
        const robot_base::ThreadLoop background = robot_base::ThreadLoop::Background("planner_host");
        background.Apply();
        while (true) {
            {
                std::unique_lock<std::mutex> lock(worker_mutex_);
                worker_cv_.wait_for(lock, kWorkerPoll, [this] {
                    return stage_.load(std::memory_order_acquire) == kStagePosted ||
                        quit_.load(std::memory_order_acquire);
                });
            }
            if (stage_.load(std::memory_order_acquire) == kStagePosted) {
                RunJob(background);
                stage_.store(kStageDone, std::memory_order_release);
                continue;
            }
            if (quit_.load(std::memory_order_acquire)) return;
        }
    }

    void RunJob(const robot_base::ThreadLoop &background) {
        JobResult result;
        try {
            switch (job_.kind) {
                case JobKind::LOAD:
                    Load(background);
                    break;
                case JobKind::START:
                    OwnedRealtime().Start(job_.robot, job_.time_s);
                    owned_started_ = true;
                    break;
                case JobKind::GENERATE:
                    result.output = Generate(background);
                    if (result.output && result.output->action.empty() && config_.prepare_trajectory) {
                        result.install = config_.prepare_trajectory(result.output->trajectory);
                    }
                    break;
                case JobKind::PREPARE: {
                    result.output = std::move(job_.publication);
                    const auto error = ValidateOutput(*result.output, num_dof_);
                    if (!error.empty()) throw std::runtime_error("published an invalid reference: " + error);
                    if (config_.prepare_trajectory) {
                        result.install = config_.prepare_trajectory(result.output->trajectory);
                    }
                    break;
                }
                case JobKind::STOP:
                    StopOwned(job_.destroy);
                    break;
                case JobKind::NONE:
                    break;
            }
        } catch (const std::exception &error) {
            result.error = error.what();
        } catch (...) {
            result.error = "unknown error";
        }
        // A planner that failed to load, start or stop is not used again.
        if (!result.error.empty() && job_.kind != JobKind::GENERATE && job_.kind != JobKind::PREPARE) ReleaseOwned();
        result.planner = owned_.get();
        result.loaded_index = owned_index_;
        result.started = owned_started_;
        result_ = std::move(result);
    }

    void Load(const robot_base::ThreadLoop &background) {
        ReleaseOwned();
        const PlannerConfig &config = config_.planners.at(job_.index);
        const PlannerType *type = FindPlannerType(config.type);
        if (!type) throw std::runtime_error("planner type is not registered: " + config.type);
        PlannerContext context = config.context;
        context.on_warmup = [this] { warming_.store(true, std::memory_order_release); };
        // Threads created while loading inherit the planner's CPU set and scheduling.
        context.thread.Apply();
        std::unique_ptr<Planner> planner;
        try {
            planner = type->factory(context);
        } catch (...) {
            background.Apply();
            throw;
        }
        background.Apply();
        const bool implemented = config.category == Category::REALTIME
            ? dynamic_cast<RealtimePlanner *>(planner.get()) != nullptr
            : dynamic_cast<OfflinePlanner *>(planner.get()) != nullptr;
        if (!implemented) {
            throw std::runtime_error("planner type " + config.type + " does not implement the " +
                CategoryName(config.category) + " planner interface");
        }
        owned_ = std::move(planner);
        owned_index_ = job_.index;
        owned_started_ = false;
    }

    RealtimePlanner &OwnedRealtime() {
        auto *planner = dynamic_cast<RealtimePlanner *>(owned_.get());
        if (!planner) throw std::runtime_error("no realtime planner is loaded");
        return *planner;
    }

    std::shared_ptr<const PlannerOutput> Generate(const robot_base::ThreadLoop &background) {
        auto *planner = dynamic_cast<OfflinePlanner *>(owned_.get());
        if (!planner) throw std::runtime_error("no offline planner is loaded");
        const std::atomic<bool> &cancelled = *job_.cancelled;
        const GenerationRequest request{job_.argument, job_.robot, cancelled, [this](float progress) {
            generation_progress_.store(progress, std::memory_order_release);
        }};
        config_.planners.at(owned_index_).context.thread.Apply();
        std::shared_ptr<PlannerOutput> output;
        try {
            output = std::make_shared<PlannerOutput>(planner->Generate(request));
        } catch (...) {
            background.Apply();
            throw;
        }
        background.Apply();
        if (cancelled.load(std::memory_order_acquire)) return nullptr;
        const std::string error = ValidateOutput(*output, num_dof_);
        if (!error.empty()) throw std::runtime_error("generated an invalid reference: " + error);
        return output;
    }

    void StopOwned(bool destroy) {
        if (!owned_) return;
        if (owned_started_) {
            owned_started_ = false;
            OwnedRealtime().Stop();
        }
        if (destroy) ReleaseOwned();
    }

    // Stops and destroys the worker-owned planner; never throws.
    void ReleaseOwned() {
        if (!owned_) return;
        try {
            if (owned_started_) OwnedRealtime().Stop();
        } catch (const std::exception &error) {
            runtime_logging::Log(runtime_logging::Level::kWarning,
                std::string("motion planner failed to stop: ") + error.what(), false);
        }
        owned_started_ = false;
        owned_index_ = -1;
        owned_.reset();
    }

    InstallReference install_;
    HostConfig config_;
    int num_dof_ = 0;

    // Tracker side. planner_ / loaded_ / started_ mirror the worker-owned
    // planner as of the last collected job and are only used while no job is
    // in flight.
    Planner *planner_ = nullptr;
    int loaded_ = -1;
    bool started_ = false;
    int active_ = -1;  // Planner of the accepted START, kept after it stops.
    std::string argument_;
    uint64_t argument_sequence_ = 0;
    bool start_pending_ = false;  // START accepted; background steps not finished.
    bool running_ = false;        // Realtime planner started, or offline result published.
    bool releasing_ = false;      // Running, waiting for the planner to settle.
    bool failed_ = false;
    bool auto_start_pending_ = false;
    uint64_t last_sequence_ = 0;
    uint64_t start_id_ = 0;
    uint64_t job_start_id_ = 0;
    robot_base::RuntimeEnvironment environment_ = robot_base::RuntimeEnvironment::UNKNOWN;
    double time_s_ = 0.0;
    double release_started_s_ = 0.0;
    bool loading_ = false;             // The pending start includes loading the planner.
    double prepare_started_s_ = 0.0;   // RL time at which that load was posted.
    double expected_prepare_s_ = 0.0;  // Estimated duration of that load; 0 when unknown.
    double rate_window_start_s_ = 0.0;
    uint64_t rate_window_count_ = 0;
    JobKind job_kind_ = JobKind::NONE;  // Job posted and not yet collected.
    std::shared_ptr<std::atomic<bool>> cancelled_;
    std::shared_ptr<const PlannerOutput> output_;
    std::shared_ptr<const PlannerOutput> seen_output_;  // Last object the planner returned.
    robot_base::PlannerStatus status_;

    // Shared. job_ is written by the tracker side before kStagePosted and
    // result_ by the worker before kStageDone.
    std::atomic<int> stage_{kStageIdle};
    std::atomic<bool> warming_{false};
    std::atomic<float> generation_progress_{0.0F};  // Reported by the offline planner.
    std::atomic<bool> quit_{false};
    Job job_;
    JobResult result_;
    std::mutex worker_mutex_;
    std::condition_variable worker_cv_;

    // Worker side.
    std::unique_ptr<Planner> owned_;
    int owned_index_ = -1;
    bool owned_started_ = false;
    std::thread worker_;
};

PlannerHost::PlannerHost(HostConfig config) : impl_(std::make_unique<Impl>(std::move(config))) {}

PlannerHost::~PlannerHost() = default;

void PlannerHost::Reset() { impl_->Reset(); }

void PlannerHost::Suspend() { impl_->Suspend(); }

void PlannerHost::Step(const robot_base::RobotData &robot,
        const MotionCommand &command,
        double time_s,
        const robot_base::PlannerRequest &request,
        const TrackerState &tracker) {
    impl_->Step(robot, command, time_s, request, tracker);
}

const std::shared_ptr<const PlannerOutput> &PlannerHost::Output() const {
    return impl_->Output();
}

const InstallReference &PlannerHost::PreparedReference() const { return impl_->PreparedReference(); }

bool PlannerHost::OutputFinite() const { return impl_->OutputFinite(); }

bool PlannerHost::Engaged() const { return impl_->Engaged(); }

void PlannerHost::RequestRelease() { impl_->RequestRelease(); }

void PlannerHost::Fail(const std::string &detail) { impl_->Fail(detail); }

const robot_base::PlannerStatus &PlannerHost::Status() const { return impl_->Status(); }

}  // namespace motion_planner
}  // namespace behavior_manager
