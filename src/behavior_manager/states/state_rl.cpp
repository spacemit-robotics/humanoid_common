/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_rl.cpp
 * @brief RL 控制状态 — 异步推理输出关节目标
 *
 * 双频异步架构：
 * - 推理线程：以 rl_dt 为周期独立运行 AssembleObs + Infer，产出动作缓存
 * - 控制线程：以 control_dt 为周期高频读取最新动作缓存，输出关节目标
 */

#include <Eigen/Dense>
#include <time.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "behavior_state.h"
#include "policy_adapter/policy_adapter.h"
#include "rl_service.h"
#include "runtime_logger.h"
#include "snapshot_worker.h"
#include "state_factory.h"
namespace behavior_manager {

class StateRL : public State {
public:
    ~StateRL() override { StopInference(); trace_worker_.reset(); progress_worker_.reset(); }

    void SetConfig(const RLConfig &cfg) {
        ValidateRLTargetPositionLimits(cfg);
        config_ = cfg;
    }

    void PrepareRuntime() {
        policy_adapter_.reset();
        policy_.Init(config_.policy);
        if (config_.policy_adapter.Enabled()) {
            policy_adapter_ = policy_adapter::Create(
                config_.policy_adapter, config_.policy);
        }
        runtime_prepared_ = true;
    }

    void OnEnter() override {
        entry_kp_ = output_ ? output_->kp : std::vector<double>{};
        entry_kd_ = output_ ? output_->kd : std::vector<double>{};
        infer_elapsed_ = 0.0;
        motion_finished_.store(false, std::memory_order_relaxed);
        switch_requested_.store(false, std::memory_order_relaxed);
        trace_worker_.reset();
        progress_worker_.reset();
        progress_sequence_.store(0, std::memory_order_relaxed);
        runtime_logging::Log(runtime_logging::Level::kInfo, "[StateRL] 进入 RL 控制状态（异步推理）");

        if (output_) {
            output_->actuation_mode = robot_base::ActuationMode::HYBRID;
            output_->target_torque.assign(output_->target_pos.size(), 0.0);
            if (!live_handoff_) {
                output_->kp = config_.kp;
                output_->kd = config_.kd;
            }
        }

        safety_triggered_ = false;
        switch_ready_.store(false, std::memory_order_relaxed);
        {
            std::lock_guard<std::mutex> lock(mutex_fault_);
            fault_ = {};
        }
        {
            std::lock_guard<std::mutex> lock(mutex_interaction_status_);
            interaction_status_ = {};
        }

        running_.store(false, std::memory_order_release);
        start_reference_requested_.store(false, std::memory_order_release);
        has_action_ = false;
        cached_action_.clear();
        cached_kp_ = config_.kp;
        cached_kd_ = config_.kd;
        action_sequence_ = 0;
        applied_action_sequence_ = 0;
        release_sequence_ = 0;
        sample_release_sequence_ = 0;
        cached_release_sequence_ = 0;
        cached_timing_ = {};
        last_action_published_ns_.store(0, std::memory_order_relaxed);
        entry_target_pos_ = output_ ? output_->target_pos : std::vector<double>{};
        entry_target_transition_elapsed_ = 0.0;
        infer_count_window_ = 0;
        time_since_last_infer_ = 0.0f;
        new_data_ready_ = false;
        policy_trace_enabled_ = false;
        policy_trace_stream_.clear();
        policy_apply_trace_stream_.clear();
        policy_trace_header_.clear();
        policy_apply_trace_header_.clear();
        entry_interaction_sequence_ = command_
            ? command_->interaction.sequence : 0;
        if (config_.rl_freq_hz) {
            config_.rl_freq_hz->store(0.0, std::memory_order_relaxed);
        }

        // BehaviorManager 会在 POWER_OFF/DAMP 中预先创建运行时。直接使用 StateRL
        // 的测试和旧调用路径仍在这里同步初始化，保持内部工厂兼容。
        try {
            if (!runtime_prepared_) PrepareRuntime();
            const auto ndof = config_.policy.rl_default_pos.size();
            sample_joint_pos_.resize(ndof);
            sample_joint_vel_.resize(ndof);
            infer_joint_pos_.resize(ndof);
            infer_joint_vel_.resize(ndof);
            target_buffer_.resize(ndof);
            applied_target_buffer_.resize(ndof);
            infer_snapshot_ = robot_base::RobotData::Create(static_cast<int>(ndof));
            infer_obs_.resize(policy_.ObsDim());
            infer_action_.reserve(policy_.ActionDim());
            infer_raw_action_.reserve(policy_.ActionDim());
            infer_executor_action_.reserve(policy_.ActionDim());
            cached_action_.reserve(policy_.ActionDim());
            sample_interaction_request_.action.reserve(128);
            ConfigurePolicyTrace();
            runtime_logging::RecordArtifact(
                "policy_model", config_.policy_name, config_.policy.model_path);
            if (!config_.policy_adapter.reference_file.empty()) {
                runtime_logging::RecordArtifact("policy_reference",
                    config_.policy_name, config_.policy_adapter.reference_file);
            }
            for (const auto &action :
                    config_.policy_adapter.joint_trajectory.actions) {
                runtime_logging::RecordArtifact(
                    "interaction_action", action.name, action.file);
            }
            if (config_.runtime_observer) {
                config_.runtime_observer->OnRuntimeInitialized(
                    policy_.GetRuntimeInfo());
            }
        } catch (const std::exception &error) {
            ReportFault(robot_base::FaultCode::INTERNAL_ERROR,
                std::string("RL runtime initialization failed: ") + error.what());
            return;
        }

        // 可选策略适配器已随策略运行时创建；进入状态时只重置时序数据。
        if (config_.policy_adapter.Enabled() && sensor_) {
            try {
                policy_adapter_->Reset(*sensor_);
                t_enter_ = std::chrono::steady_clock::now();
                runtime_logging::Log(runtime_logging::Level::kInfo,
                    std::string("[StateRL] policy_adapter 启用: type=") + policy_adapter_->Type() +
                    ", reference=" + config_.policy_adapter.reference_file);
            } catch (const std::exception &e) {
                std::cerr << "[StateRL] policy_adapter 加载失败，切安全态: "
                    << e.what() << std::endl;
                policy_adapter_.reset();
                ReportFault(robot_base::FaultCode::INTERNAL_ERROR,
                    std::string("policy adapter initialization failed: ") + e.what());
                return;
            }
        }

        // 启动推理线程（配置从 RLConfig.infer_thread_cfg 注入）
        infer_loop_ = config_.infer_thread_cfg;
        entered_at_ = std::chrono::steady_clock::now();
        infer_window_start_ = entered_at_;
        running_.store(true, std::memory_order_release);
        try {
            infer_loop_.Start([this] { return InferStep(); });
        } catch (const std::exception &error) {
            running_.store(false, std::memory_order_release);
            ReportFault(robot_base::FaultCode::INTERNAL_ERROR,
                std::string("RL inference thread failed to start: ") + error.what());
        }
    }

    void Run(float control_dt, float rl_dt) override {
        if (!sensor_ || !output_ || !command_)
            return;

        if (!std::isfinite(control_dt) || control_dt <= 0.0f ||
            !std::isfinite(rl_dt) || rl_dt <= 0.0f) {
            ReportFault(robot_base::FaultCode::INVALID_DATA,
                "RL runtime received an invalid control period");
            return;
        }

        // 安全检查：IMU 倾角
        if (std::abs(sensor_->rpy[0]) > config_.max_roll ||
            std::abs(sensor_->rpy[1]) > config_.max_pitch) {
            std::cerr << "[StateRL] IMU 倾角超限! roll=" << sensor_->rpy[0]
                    << ", pitch=" << sensor_->rpy[1] << std::endl;
            std::ostringstream detail;
            detail << "RL attitude limit exceeded: roll=" << sensor_->rpy[0]
                << ", pitch=" << sensor_->rpy[1];
            ReportFault(robot_base::FaultCode::LIMIT_EXCEEDED, detail.str());
            return;
        }

        // 基于控制循环时间的推理触发机制
        if (command_->key == robot_base::kCommandStartReference) {
            start_reference_requested_.store(true, std::memory_order_release);
            command_->key = 0;
        }
        time_since_last_infer_ += control_dt;
        if (time_since_last_infer_ >= rl_dt) {
            // 保留余数但丢弃已错过的整周期，避免连续追赶补发。
            time_since_last_infer_ = std::fmod(time_since_last_infer_, rl_dt);

            const auto release_time = RLRuntimeClock::now();
            std::uint64_t release_sequence = 0;

            // 1) 写入最新传感器数据和命令到共享区（供推理线程读取）
            {
                std::unique_lock<std::mutex> lock(mutex_sensor_, std::try_to_lock);
                if (lock.owns_lock()) {
                    release_sequence = ++release_sequence_;
                    sample_release_sequence_ = release_sequence;
                    sample_release_time_ = release_time;
                    sample_gyro_ = sensor_->gyro;
                    sample_rpy_ = sensor_->rpy;
                    sample_joint_pos_ = sensor_->joint_pos;
                    sample_joint_vel_ = sensor_->joint_vel;
                    sample_base_pos_ = sensor_->base_pos;
                    sample_base_quat_ = sensor_->base_quat;
                    sample_base_vel_ = {
                        {sensor_->base_vel[0], sensor_->base_vel[1], sensor_->base_vel[2]}};
                    sample_device_time_ = sensor_->time;
                    sample_cmd_vx_ = command_->vx;
                    sample_cmd_vy_ = command_->vy;
                    sample_cmd_wz_ = command_->wz;
                    sample_interaction_request_ = command_->interaction;
                    if (sample_interaction_request_.sequence == entry_interaction_sequence_) {
                        sample_interaction_request_.operation =
                            robot_base::InteractionRequest::Operation::NONE;
                    }
                    sample_rl_dt_ = rl_dt;
                    new_data_ready_ = true;
                }
            }
            // 通知推理线程有新数据到达
            if (release_sequence != 0) {
                cv_sensor_.notify_one();
                NotifyRuntimeEvent(RLRuntimeEventType::RELEASE, release_sequence, release_time);
            }
        }

        // 2) 读取最新推理结果，映射到关节目标
        bool action_applied = false;
        std::uint64_t applied_release_sequence = 0;
        RLRuntimeClock::time_point action_applied_time;
        PolicyTiming applied_timing;
        auto &applied_target_pos = applied_target_buffer_;
        const auto now = RLRuntimeClock::now();
        {
            std::unique_lock<std::mutex> lock(mutex_action_, std::try_to_lock);
            if (!lock.owns_lock()) {
                // A descheduled publisher must neither block control nor renew
                // the command lease indefinitely through lock contention.
                const auto published = last_action_published_ns_.load(std::memory_order_acquire);
                const double age = published == 0 ?
                    std::chrono::duration<double>(now - entered_at_).count() :
                    std::chrono::duration<double>(now.time_since_epoch()).count() - published * 1.0e-9;
                const double limit = published == 0 ? config_.first_action_timeout_s : config_.max_action_age_s;
                if (limit > 0.0 && age > limit &&
                    published == last_action_published_ns_.load(std::memory_order_acquire)) {
                    ReportFault(published == 0 ? robot_base::FaultCode::INFERENCE_TIMEOUT :
                        robot_base::FaultCode::COMMAND_TIMEOUT, "RL action publisher deadline expired");
                }
                return;
            }
            if (!has_action_) {
                if (config_.first_action_timeout_s > 0.0 &&
                    std::chrono::duration<double>(now - entered_at_).count() >
                        config_.first_action_timeout_s) {
                    ReportFault(robot_base::FaultCode::INFERENCE_TIMEOUT,
                        "RL first action deadline expired");
                }
                return;
            }
            if (config_.max_action_age_s > 0.0 &&
                std::chrono::duration<double>(
                    now - cached_timing_.action_published).count() >
                    config_.max_action_age_s) {
                ReportFault(robot_base::FaultCode::COMMAND_TIMEOUT,
                    "RL action age exceeded the configured limit");
                return;
            }
            const bool has_new_action =
                action_sequence_ != applied_action_sequence_;
            if (has_action_ && has_new_action) {
                auto &target_pos = target_buffer_;
                policy_.MapActionToTargetPos(cached_action_, target_pos);
                ApplyEntryTargetTransition(
                    target_pos, rl_dt,
                    action_sequence_ - applied_action_sequence_);
                if (!ClampTargetPosition(target_pos)) return;
                output_->target_pos = target_pos;
                output_->kp = cached_kp_;
                output_->kd = cached_kd_;
                ApplyHandoffGains();
                applied_action_sequence_ = action_sequence_;
                int ndof = static_cast<int>(output_->target_pos.size());
                output_->target_vel.assign(ndof, 0.0);
                output_->enable = true;
                if (has_new_action) {
                    applied_release_sequence = cached_release_sequence_;
                    action_applied_time = RLRuntimeClock::now();
                    applied_timing = cached_timing_;
                    if (policy_trace_enabled_)
                        applied_target_pos = output_->target_pos;
                    action_applied = true;
                }
            }
        }
        if (action_applied) {
            NotifyRuntimeEvent(RLRuntimeEventType::ACTION_APPLIED,
                applied_release_sequence, action_applied_time);
            RecordPolicyApplyTrace(
                applied_timing, action_applied_time, applied_target_pos);
        }
    }

    StateName CheckTransition() override {
        // 安全触发 → SAFETY
        if (safety_triggered_.load(std::memory_order_acquire)) {
            return StateName::SAFETY;
        }
        // key=1 → 退回阻尼状态
        if (command_ && command_->key == 1) {
            command_->key = 0;
            return StateName::DAMP;
        }
        // key=-1 → 完全失力
        if (command_ && command_->key == -1) {
            command_->key = 0;
            return StateName::POWER_OFF;
        }
        return StateName::RL;
    }

    void OnExit() override {
        StopInference();
        live_handoff_ = false;
        // 历史观测、last_action 和模型反馈状态已被本轮推理修改。下次进入必须
        // 使用 BehaviorManager 在安全状态重新准备的新实例，兼容路径则重新 Init。
        runtime_prepared_ = false;
        if (config_.rl_freq_hz) {
            config_.rl_freq_hz->store(0.0, std::memory_order_relaxed);
        }
        runtime_logging::Log(runtime_logging::Level::kInfo, "[StateRL] 退出 RL 控制状态");
    }

    void SuspendForReplacement() override {
        // The inference thread uses private sensor snapshots, never output_.
        // Stop publishing new work; joining and releasing models is deferred.
        running_.store(false, std::memory_order_release);
        cv_sensor_.notify_one();
        runtime_prepared_ = false;
    }

    bool ReadyForPolicySwitch() const override {
        return switch_requested_.load(std::memory_order_acquire) &&
            switch_ready_.load(std::memory_order_acquire);
    }

    void RequestPolicySwitch(bool requested) override {
        if (switch_requested_.exchange(requested, std::memory_order_acq_rel) != requested)
            switch_ready_.store(false, std::memory_order_release);
    }

    bool PolicyFinished() const override {
        return motion_finished_.load(std::memory_order_acquire);
    }

    void OnPolicyHandoff() override {
        live_handoff_ = true;
        OnEnter();
    }

    robot_base::FaultStatus CurrentFault() const override {
        std::lock_guard<std::mutex> lock(mutex_fault_);
        return fault_;
    }

    robot_base::InteractionStatus CurrentInteractionStatus() const override {
        std::lock_guard<std::mutex> lock(mutex_interaction_status_);
        return interaction_status_;
    }

private:
    bool runtime_prepared_ = false;
    std::atomic<bool> switch_ready_{false};
    std::atomic<bool> switch_requested_{false};
    std::atomic<bool> motion_finished_{false};
    bool live_handoff_ = false;
    double infer_elapsed_ = 0.0;
    std::vector<double> entry_kp_, entry_kd_;

    bool HasMotionPhase() const {
        for (const auto &segment : config_.policy.obs_segments)
            if (std::find(segment.terms.begin(), segment.terms.end(), "ref_motion_phase") !=
                segment.terms.end()) return true;
        return false;
    }

    void StopInference() noexcept {
        running_.store(false, std::memory_order_release);
        cv_sensor_.notify_one();
        if (infer_loop_.IsRunning()) {
            try {
                policy_.RequestInferenceTermination();
                if (policy_adapter_) policy_adapter_->RequestInferenceTermination();
            } catch (const std::exception &error) {
                std::cerr << "[StateRL] 请求终止推理失败: "
                    << error.what() << std::endl;
            }
        }
        infer_loop_.Stop();
    }

    void ApplyEntryTargetTransition(std::vector<double> &target,
                                    float rl_dt,
                                    std::uint64_t inference_steps) {
        const double duration = live_handoff_ ? config_.switch_transition_duration :
            config_.entry_target_transition_duration;
        if (duration <= 0.0 || entry_target_pos_.size() != target.size()) return;

        entry_target_transition_elapsed_ = std::min(duration,
            entry_target_transition_elapsed_ +
                std::max(0.0, static_cast<double>(rl_dt)) * inference_steps);
        const double progress = entry_target_transition_elapsed_ / duration;
        const double alpha = progress * progress;
        for (std::size_t i = 0; i < target.size(); ++i) {
            target[i] = (1.0 - alpha) * entry_target_pos_[i] + alpha * target[i];
        }
    }

    void ApplyHandoffGains() {
        const double duration = config_.switch_transition_duration;
        if (!live_handoff_ || duration <= 0.0 || entry_kp_.size() != output_->kp.size() ||
            entry_kd_.size() != output_->kd.size()) return;
        const double progress = std::clamp(entry_target_transition_elapsed_ / duration, 0.0, 1.0);
        const double alpha = progress * progress;
        for (std::size_t i = 0; i < output_->kp.size(); ++i) {
            output_->kp[i] = (1.0 - alpha) * entry_kp_[i] + alpha * output_->kp[i];
            output_->kd[i] = (1.0 - alpha) * entry_kd_[i] + alpha * output_->kd[i];
        }
    }

    bool ClampTargetPosition(std::vector<double> &target) {
        if (config_.target_position_lower.empty()) return true;
        if (target.size() != config_.target_position_lower.size()) {
            ReportFault(robot_base::FaultCode::INVALID_DATA,
                "RL target position dimensions do not match configured limits");
            return false;
        }
        for (std::size_t i = 0; i < target.size(); ++i) {
            if (!std::isfinite(target[i])) {
                ReportFault(robot_base::FaultCode::INVALID_DATA,
                    "RL target position is not finite at joint " + std::to_string(i));
                return false;
            }
            target[i] = std::clamp(target[i],
                config_.target_position_lower[i] + config_.target_limit_margin,
                config_.target_position_upper[i] - config_.target_limit_margin);
        }
        return true;
    }

    // ==================== 推理线程回调 ====================

    bool InferStep() {
        std::array<double, 3> gyro, rpy, base_pos, base_vel;
        std::array<double, 4> base_quat;
        auto &joint_pos = infer_joint_pos_;
        auto &joint_vel = infer_joint_vel_;
        double cmd_vx, cmd_vy, cmd_wz;
        InferenceDiagnostics diagnostics;
        double device_time;
        float rl_dt;
        std::uint64_t release_sequence = 0;
        RLRuntimeClock::time_point release_time;
        RLRuntimeClock::time_point inference_start;
        double cpu_start_s = 0.0;

        // 等待控制循环的主线程发来通知（严格同步到 control_dt 驱动的时钟）
        {
            std::unique_lock<std::mutex> lock(mutex_sensor_);
            cv_sensor_.wait(lock, [this] {
                return new_data_ready_ ||
                    !running_.load(std::memory_order_acquire);
            });
            if (!running_.load(std::memory_order_acquire)) {
                return false;  // 退出线程
            }
            inference_start = RLRuntimeClock::now();
            cpu_start_s = ThreadCpuSeconds();
            new_data_ready_ = false;
            release_sequence = sample_release_sequence_;
            release_time = sample_release_time_;

            // 拷贝传感器快照
            gyro = sample_gyro_;
            rpy = sample_rpy_;
            joint_pos = sample_joint_pos_;
            joint_vel = sample_joint_vel_;
            base_pos = sample_base_pos_;
            base_quat = sample_base_quat_;
            base_vel = sample_base_vel_;
            device_time = sample_device_time_;
            cmd_vx = sample_cmd_vx_;
            cmd_vy = sample_cmd_vy_;
            cmd_wz = sample_cmd_wz_;
            diagnostics.request = sample_interaction_request_;
            rl_dt = sample_rl_dt_;
        }
        auto stage_start = RLRuntimeClock::now();
        diagnostics.snapshot_ms = DurationMilliseconds(stage_start, inference_start);
        NotifyRuntimeEvent(RLRuntimeEventType::INFERENCE_START,
            release_sequence, inference_start);
        progress_sequence_.store(release_sequence, std::memory_order_relaxed);

        auto &obs = infer_obs_;
        auto &raw_action = infer_raw_action_;
        auto &executor_action = infer_executor_action_;
        auto &action = infer_action_;
        auto &joint_kp = infer_kp_;
        auto &joint_kd = infer_kd_;
        joint_kp = config_.kp;
        joint_kd = config_.kd;
        const bool switching = switch_requested_.load(std::memory_order_acquire);
        bool adapter_ready = !policy_adapter_;
        bool finished = false;
        try {
            auto &executor = policy_adapter_
                ? policy_adapter_->SelectExecutor({cmd_vx, cmd_vy, cmd_wz}, policy_) : policy_;
            // 在 AssembleObs 前注入 tracker-specific 输入。
            if (policy_adapter_) {
                const auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t_enter_).count();
                if (start_reference_requested_.exchange(
                        false, std::memory_order_acq_rel) && !switching &&
                    policy_adapter_->StartPlayback(elapsed)) {
                    runtime_logging::Log(runtime_logging::Level::kInfo,
                        "manual reference playback started", false);
                }
                if (!switching) policy_adapter_->HandleInteractionRequest(diagnostics.request, elapsed);
                policy_adapter_->PreparePolicySwitch(switching, elapsed);
                auto &snapshot = infer_snapshot_;
                snapshot.num_dof = static_cast<int>(joint_pos.size());
                snapshot.base_pos = base_pos;
                snapshot.base_quat = base_quat;
                snapshot.base_vel = {
                    base_vel[0], base_vel[1], base_vel[2],
                    gyro[0], gyro[1], gyro[2]};
                snapshot.rpy = rpy;
                snapshot.gyro = gyro;
                snapshot.joint_pos = joint_pos;
                snapshot.joint_vel = joint_vel;
                policy_adapter_->PrepareInputs(snapshot, elapsed, executor);
            }
            auto stage_finish = RLRuntimeClock::now();
            diagnostics.prepare_inputs_ms = DurationMilliseconds(stage_finish, stage_start);
            stage_start = stage_finish;
            RecordProgress(ProgressStage::PREPARE_DONE, release_sequence, stage_finish);

            // 组装观测 + 推理
            executor.AssembleObs(gyro,
                                rpy,
                                cmd_vx,
                                cmd_vy,
                                cmd_wz,
                                joint_pos,
                                joint_vel,
                                base_quat,
                                base_vel,
                                rl_dt,
                                obs);
            infer_elapsed_ += rl_dt;
            stage_finish = RLRuntimeClock::now();
            diagnostics.assemble_obs_ms = DurationMilliseconds(stage_finish, stage_start);
            stage_start = stage_finish;
            RecordProgress(ProgressStage::OBS_DONE, release_sequence, stage_finish);
            if (policy_trace_enabled_) {
                executor.Infer(obs, action, &raw_action);
                executor_action = action;
            } else {
                executor.Infer(obs, action);
            }
            stage_finish = RLRuntimeClock::now();
            diagnostics.policy_infer_ms = DurationMilliseconds(stage_finish, stage_start);
            stage_start = stage_finish;
            RecordProgress(ProgressStage::MODEL_DONE, release_sequence, stage_finish);
            const bool already_late = config_.discard_late_inference &&
                std::chrono::duration<double>(stage_finish - release_time).count() >
                    config_.inference_deadline_s;
            if (policy_adapter_ && !already_late) policy_adapter_->OnAction(action);
            if (policy_adapter_) {
                adapter_ready = policy_adapter_->ReadyForPolicySwitch();
                finished = policy_adapter_->PlaybackFinished();
                policy_adapter_->ApplyJointGains(joint_kp, joint_kd);
                diagnostics.model_variant = policy_adapter_->ActiveModelName();
                diagnostics.interaction = policy_adapter_->GetInteractionStatus();
                std::lock_guard<std::mutex> lock(mutex_interaction_status_);
                interaction_status_ = diagnostics.interaction;
            }
        } catch (const std::exception &error) {
            RecordProgress(ProgressStage::ABORTED, release_sequence, RLRuntimeClock::now());
            if (!running_.load(std::memory_order_acquire)) return false;
            ReportFault(robot_base::FaultCode::INTERNAL_ERROR,
                std::string("RL inference failed: ") + error.what());
            return false;
        }
        const auto inference_finish = RLRuntimeClock::now();
        diagnostics.postprocess_ms = DurationMilliseconds(inference_finish, stage_start);
        diagnostics.thread_cpu_ms = (ThreadCpuSeconds() - cpu_start_s) * 1000.0;
        PolicyTiming timing{release_sequence, release_time, inference_start,
            inference_finish, {}};
        NotifyRuntimeEvent(RLRuntimeEventType::INFERENCE_FINISH,
            release_sequence, inference_finish);
        UpdateRlFreq();

        if (config_.inference_deadline_s > 0.0 &&
            std::chrono::duration<double>(
                inference_finish - release_time).count() >
                config_.inference_deadline_s) {
            if (!config_.discard_late_inference)
                ReportFault(robot_base::FaultCode::INFERENCE_TIMEOUT,
                    InferenceTimeoutDetail(timing, diagnostics));
            else
                NotifyRuntimeEvent(RLRuntimeEventType::INFERENCE_DISCARDED,
                    release_sequence, inference_finish);
            RecordPolicyTrace(obs, raw_action, executor_action, action,
                device_time, timing, diagnostics, cmd_vx, cmd_vy, cmd_wz,
                false);
            return config_.discard_late_inference;
        }
        if (action.size() != static_cast<size_t>(policy_.ActionDim()) ||
            !std::all_of(action.begin(), action.end(),
                [](double value) { return std::isfinite(value); })) {
            ReportFault(robot_base::FaultCode::INVALID_DATA,
                "RL inference returned an invalid action");
            return false;
        }

        if (!policy_adapter_ && HasMotionPhase()) {
            finished = config_.policy.motion_length > 0.0 &&
                infer_elapsed_ >= config_.policy.motion_length;
            adapter_ready = finished;
        }
        motion_finished_.store(finished, std::memory_order_release);
        switch_ready_.store(switching && adapter_ready &&
            std::abs(cmd_vx) < 1.0e-6 && std::abs(cmd_vy) < 1.0e-6 && std::abs(cmd_wz) < 1.0e-6,
            std::memory_order_release);

        // 写入动作缓存
        RLRuntimeClock::time_point action_published;
        {
            std::lock_guard<std::mutex> lock(mutex_action_);
            cached_action_ = action;
            cached_kp_ = joint_kp;
            cached_kd_ = joint_kd;
            has_action_ = true;
            cached_release_sequence_ = release_sequence;
            action_published = RLRuntimeClock::now();
            timing.action_published = action_published;
            cached_timing_ = timing;
            ++action_sequence_;
            last_action_published_ns_.store(std::chrono::duration_cast<std::chrono::nanoseconds>(
                action_published.time_since_epoch()).count(), std::memory_order_release);
        }
        NotifyRuntimeEvent(RLRuntimeEventType::ACTION_PUBLISHED,
            release_sequence, action_published);
        RecordPolicyTrace(obs, raw_action, executor_action, action,
            device_time, timing, diagnostics, cmd_vx, cmd_vy, cmd_wz, true);

        return true;  // 继续循环
    }

    // ==================== 配置与运行时 ====================

    RLConfig config_;
    rl_policy::PolicyExecutor policy_;

    // 推理线程
    robot_base::ThreadLoop infer_loop_;

    // 安全标志
    std::atomic<bool> safety_triggered_{false};
    mutable std::mutex mutex_fault_;
    robot_base::FaultStatus fault_;
    mutable std::mutex mutex_interaction_status_;
    robot_base::InteractionStatus interaction_status_;

    // 策略输入协议适配器（policy_adapter.type 为空则不启用）
    std::unique_ptr<policy_adapter::PolicyAdapter> policy_adapter_;
    std::chrono::steady_clock::time_point t_enter_;
    std::atomic<bool> start_reference_requested_{false};

    // 推理频率统计（推理线程内读写）
    int infer_count_window_ = 0;
    std::chrono::steady_clock::time_point infer_window_start_ = std::chrono::steady_clock::now();

    // ==================== 线程间共享数据 ====================

    // 传感器快照（主线程写，推理线程读）
    std::mutex mutex_sensor_;
    std::condition_variable cv_sensor_;
    float time_since_last_infer_ = 0.0f;
    bool new_data_ready_ = false;
    std::atomic<bool> running_{false};

    std::array<double, 3> sample_gyro_ = {};
    std::array<double, 3> sample_rpy_ = {};
    std::vector<double> sample_joint_pos_;
    std::vector<double> sample_joint_vel_;
    std::array<double, 3> sample_base_pos_ = {};
    std::array<double, 4> sample_base_quat_ = {};
    std::array<double, 3> sample_base_vel_ = {};
    double sample_device_time_ = 0.0;
    double sample_cmd_vx_ = 0;
    double sample_cmd_vy_ = 0;
    double sample_cmd_wz_ = 0;
    robot_base::InteractionRequest sample_interaction_request_;
    uint64_t entry_interaction_sequence_ = 0;
    float sample_rl_dt_ = 0.02f;

    // 动作缓存（推理线程写，主线程读）
    std::mutex mutex_action_;
    std::vector<double> cached_action_;
    std::vector<double> cached_kp_;
    std::vector<double> cached_kd_;
    std::atomic<int64_t> last_action_published_ns_{0};
    std::vector<double> target_buffer_, applied_target_buffer_;
    std::vector<double> infer_joint_pos_, infer_joint_vel_, infer_action_;
    std::vector<double> infer_raw_action_, infer_executor_action_, infer_kp_, infer_kd_;
    Eigen::VectorXf infer_obs_;
    robot_base::RobotData infer_snapshot_;
    bool has_action_ = false;
    std::uint64_t action_sequence_ = 0;
    std::uint64_t applied_action_sequence_ = 0;
    std::uint64_t release_sequence_ = 0;
    std::uint64_t sample_release_sequence_ = 0;
    RLRuntimeClock::time_point sample_release_time_;
    std::uint64_t cached_release_sequence_ = 0;
    std::vector<double> entry_target_pos_;
    double entry_target_transition_elapsed_ = 0.0;

    bool policy_trace_enabled_ = false;
    std::string policy_trace_stream_;
    std::string policy_apply_trace_stream_;
    std::string policy_trace_header_;
    std::string policy_apply_trace_header_;

    struct PolicyTiming {
        std::uint64_t release_sequence = 0;
        RLRuntimeClock::time_point release;
        RLRuntimeClock::time_point inference_start;
        RLRuntimeClock::time_point inference_finish;
        RLRuntimeClock::time_point action_published;
    };

    struct InferenceDiagnostics {
        double snapshot_ms = 0.0;
        double prepare_inputs_ms = 0.0;
        double assemble_obs_ms = 0.0;
        double policy_infer_ms = 0.0;
        double postprocess_ms = 0.0;
        double thread_cpu_ms = std::numeric_limits<double>::quiet_NaN();
        robot_base::InteractionRequest request;
        robot_base::InteractionStatus interaction;
        std::string model_variant;
    };

    // Allocated once at entry. The two realtime producers only copy plain values;
    // string/CSV construction and logger queue access happen in the consumer.
    struct PolicyTraceSnapshot {
        std::array<float, 8192> obs{};
        std::array<double, 128> raw{}, executor{}, adapter{}, target{};
        size_t obs_size = 0, raw_size = 0, executor_size = 0, adapter_size = 0, target_size = 0;
        PolicyTiming timing;
        RLRuntimeClock::time_point applied;
        double wall = 0, device = 0, vx = 0, vy = 0, wz = 0;
        double snapshot = 0, prepare = 0, assemble = 0, infer = 0, post = 0, cpu = 0;
        uint64_t interaction_sequence = 0, request_sequence = 0;
        int phase = 0, operation = 0;
        char interaction[128]{}, request[128]{}, variant[128]{};
        bool apply = false, published = false;
    };
    std::unique_ptr<runtime_logging::SnapshotWorker<PolicyTraceSnapshot, 16>> trace_worker_;

    enum class ProgressStage {
        RELEASE, START, PREPARE_DONE, OBS_DONE, MODEL_DONE, FINISH,
        PUBLISHED, APPLIED, DISCARDED, FAULT, ABORTED
    };
    struct ProgressSnapshot {
        double wall = 0, monotonic = 0, cpu = 0;
        uint64_t sequence = 0;
        long tid = 0, minor_faults = -1, major_faults = -1, voluntary = -1, involuntary = -1;
        ProgressStage stage{};
    };
    std::unique_ptr<runtime_logging::SnapshotWorker<ProgressSnapshot, 256>> progress_worker_;
    std::atomic<uint64_t> progress_sequence_{0};

    void RecordProgress(ProgressStage stage, uint64_t sequence,
        RLRuntimeClock::time_point timestamp) noexcept {
        if (!progress_worker_) return;
        ProgressSnapshot sample;
        sample.stage = stage;
        sample.sequence = sequence;
        sample.monotonic = ClockSeconds(timestamp);
        sample.wall = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        static thread_local const long tid = syscall(SYS_gettid);
        sample.tid = tid;
        sample.cpu = ThreadCpuSeconds();
        rusage usage{};
        if (getrusage(RUSAGE_THREAD, &usage) == 0) {
            sample.minor_faults = usage.ru_minflt;
            sample.major_faults = usage.ru_majflt;
            sample.voluntary = usage.ru_nvcsw;
            sample.involuntary = usage.ru_nivcsw;
        }
        progress_worker_->Publish(sample);
    }

    void FormatProgress(const ProgressSnapshot &sample) const {
        static const char *const stages[] = {"release", "start", "prepare_done", "obs_done", "model_done",
            "finish", "published", "applied", "discarded", "fault", "aborted"};
        std::ostringstream row;
        row << std::fixed << std::setprecision(9) << sample.wall << ',' << sample.monotonic
            << ',' << sample.sequence << ',' << stages[static_cast<size_t>(sample.stage)]
            << ',' << sample.tid << ',' << sample.cpu << ',' << sample.minor_faults
            << ',' << sample.major_faults << ',' << sample.voluntary << ',' << sample.involuntary;
        runtime_logging::RecordCsv("control_inference_progress_" + config_.policy_name,
            "wall_time_s,monotonic_s,release_sequence,stage,tid,thread_cpu_s,minor_faults,major_faults,"
            "voluntary_switches,involuntary_switches", row.str());
    }

    PolicyTiming cached_timing_;
    RLRuntimeClock::time_point entered_at_;

    void ReportFault(robot_base::FaultCode code,
            const std::string &detail) {
        {
            std::lock_guard<std::mutex> lock(mutex_fault_);
            if (!fault_.latched) {
                RecordProgress(ProgressStage::FAULT, progress_sequence_.load(std::memory_order_relaxed),
                    RLRuntimeClock::now());
                fault_.active = true;
                fault_.latched = true;
                fault_.source = robot_base::FaultSource::POLICY;
                fault_.code = code;
                fault_.timestamp_s = ClockSeconds(RLRuntimeClock::now());
                fault_.detail = detail;
                runtime_logging::Log(runtime_logging::Level::kError,
                    std::string("RL safety fault: ") + detail, false);
            }
        }
        safety_triggered_.store(true, std::memory_order_release);
    }

    static double ClockSeconds(RLRuntimeClock::time_point timestamp) {
        return std::chrono::duration<double>(timestamp.time_since_epoch()).count();
    }

    static double DurationMilliseconds(
        RLRuntimeClock::time_point finish,
        RLRuntimeClock::time_point start) {
        return std::chrono::duration<double, std::milli>(finish - start).count();
    }

    static double ThreadCpuSeconds() {
        timespec timestamp{};
        if (clock_gettime(CLOCK_THREAD_CPUTIME_ID, &timestamp) != 0) {
            return std::numeric_limits<double>::quiet_NaN();
        }
        return static_cast<double>(timestamp.tv_sec) + timestamp.tv_nsec * 1.0e-9;
    }

    std::string InferenceTimeoutDetail(const PolicyTiming &timing,
        const InferenceDiagnostics &diagnostics) const {
        std::ostringstream detail;
        detail << std::fixed << std::setprecision(3)
            << "RL inference missed its release-to-finish deadline"
            << ": policy=" << std::quoted(config_.policy_name)
            << ", release_sequence=" << timing.release_sequence
            << ", total_ms=" << DurationMilliseconds(timing.inference_finish, timing.release)
            << ", deadline_ms=" << config_.inference_deadline_s * 1000.0
            << ", wait_ms=" << DurationMilliseconds(timing.inference_start, timing.release)
            << ", snapshot_ms=" << diagnostics.snapshot_ms
            << ", prepare_inputs_ms=" << diagnostics.prepare_inputs_ms
            << ", assemble_obs_ms=" << diagnostics.assemble_obs_ms
            << ", policy_infer_ms=" << diagnostics.policy_infer_ms
            << ", postprocess_ms=" << diagnostics.postprocess_ms
            << ", inference_thread_cpu_ms=" << diagnostics.thread_cpu_ms
            << ", interaction_sequence=" << diagnostics.interaction.sequence
            << ", interaction_action=" << std::quoted(diagnostics.interaction.action)
            << ", interaction_phase=" << static_cast<int>(diagnostics.interaction.phase)
            << ", request_sequence=" << diagnostics.request.sequence
            << ", request_operation=" << static_cast<int>(diagnostics.request.operation)
            << ", request_action=" << std::quoted(diagnostics.request.action);
        return detail.str();
    }

    void ConfigurePolicyTrace() {
        const auto logging_config = runtime_logging::GetConfig();
        if (logging_config.telemetry_enabled) {
            progress_worker_ = std::make_unique<runtime_logging::SnapshotWorker<ProgressSnapshot, 256>>(
                [this](const ProgressSnapshot &sample) { FormatProgress(sample); });
        }
        policy_trace_enabled_ = logging_config.telemetry_enabled &&
            logging_config.policy_trace_enabled;
        if (policy_.ObsDim() > 8192 || policy_.ActionDim() > 128 || config_.kp.size() > 128) {
            policy_trace_enabled_ = false;
            runtime_logging::Log(runtime_logging::Level::kWarning,
                "policy trace exceeds snapshot capacity; inference is unaffected");
        }
        if (!policy_trace_enabled_) return;

        policy_trace_stream_ = "control_policy_" +
            (config_.policy_name.empty() ? std::string("unnamed") : config_.policy_name);
        policy_apply_trace_stream_ = "control_policy_apply_" +
            (config_.policy_name.empty() ? std::string("unnamed") : config_.policy_name);
        std::ostringstream header;
        header << "wall_time_s,device_time_s,release_sequence,release_time_s,"
            "inference_start_time_s,inference_finish_time_s,action_published_time_s,"
            "release_to_start_ms,inference_ms,finish_to_publish_ms,release_to_publish_ms,"
            "vx,vy,wz";
        for (int i = 0; i < policy_.ObsDim(); ++i) {
            header << ",obs_" << i;
        }
        for (int i = 0; i < policy_.ActionDim(); ++i) {
            header << ",raw_action_" << i;
        }
        for (int i = 0; i < policy_.ActionDim(); ++i) {
            header << ",executor_action_" << i;
        }
        for (int i = 0; i < policy_.ActionDim(); ++i) {
            header << ",adapter_action_" << i;
        }
        header << ",result,release_to_finish_ms,inference_deadline_ms,snapshot_ms,"
            "prepare_inputs_ms,assemble_obs_ms,policy_infer_ms,postprocess_ms,"
            "inference_thread_cpu_ms,interaction_sequence,interaction_action,interaction_phase,"
            "request_sequence,request_operation,request_action,model_variant";
        policy_trace_header_ = header.str();

        std::ostringstream apply_header;
        apply_header << "wall_time_s,release_sequence,action_published_time_s,"
            "action_applied_time_s,publish_to_apply_ms,release_to_apply_ms";
        for (size_t i = 0; i < config_.kp.size(); ++i)
            apply_header << ",target_pos_" << i;
        policy_apply_trace_header_ = apply_header.str();
        trace_worker_ = std::make_unique<
            runtime_logging::SnapshotWorker<PolicyTraceSnapshot, 16>>(
                [this](const PolicyTraceSnapshot &sample) { FormatPolicyTrace(sample); });
    }

    void RecordPolicyTrace(const Eigen::VectorXf &obs,
                            const std::vector<double> &raw_action,
                            const std::vector<double> &executor_action,
                            const std::vector<double> &adapter_action,
                            double device_time,
                            const PolicyTiming &timing,
                            const InferenceDiagnostics &diagnostics,
                            double cmd_vx,
                            double cmd_vy,
                            double cmd_wz,
                            bool published) const {
        if (!policy_trace_enabled_ || !trace_worker_) return;
        PolicyTraceSnapshot sample;
        if (obs.size() > static_cast<int>(sample.obs.size()) ||
            raw_action.size() > sample.raw.size() || executor_action.size() > sample.executor.size() ||
            adapter_action.size() > sample.adapter.size()) return;
        sample.wall = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        sample.device = device_time;
        sample.timing = timing;
        sample.vx = cmd_vx; sample.vy = cmd_vy; sample.wz = cmd_wz;
        sample.published = published;
        sample.obs_size = obs.size();
        sample.raw_size = raw_action.size();
        sample.executor_size = executor_action.size();
        sample.adapter_size = adapter_action.size();
        std::copy_n(obs.data(), sample.obs_size, sample.obs.begin());
        std::copy(raw_action.begin(), raw_action.end(), sample.raw.begin());
        std::copy(executor_action.begin(), executor_action.end(), sample.executor.begin());
        std::copy(adapter_action.begin(), adapter_action.end(), sample.adapter.begin());
        sample.snapshot = diagnostics.snapshot_ms;
        sample.prepare = diagnostics.prepare_inputs_ms;
        sample.assemble = diagnostics.assemble_obs_ms;
        sample.infer = diagnostics.policy_infer_ms;
        sample.post = diagnostics.postprocess_ms;
        sample.cpu = diagnostics.thread_cpu_ms;
        sample.interaction_sequence = diagnostics.interaction.sequence;
        sample.request_sequence = diagnostics.request.sequence;
        sample.phase = static_cast<int>(diagnostics.interaction.phase);
        sample.operation = static_cast<int>(diagnostics.request.operation);
        std::copy_n(diagnostics.interaction.action.data(),
            std::min(diagnostics.interaction.action.size(), sizeof(sample.interaction) - 1),
            sample.interaction);
        std::copy_n(diagnostics.request.action.data(),
            std::min(diagnostics.request.action.size(), sizeof(sample.request) - 1), sample.request);
        std::copy_n(diagnostics.model_variant.data(),
            std::min(diagnostics.model_variant.size(), sizeof(sample.variant) - 1), sample.variant);
        trace_worker_->Publish(sample);
    }

    void FormatPolicyTrace(const PolicyTraceSnapshot &sample) const {
        if (sample.apply) {
            FormatPolicyApplyTrace(sample);
            return;
        }
        const auto &timing = sample.timing;
        const bool published = sample.published;
        const double unavailable = std::numeric_limits<double>::quiet_NaN();
        std::ostringstream row;
        row << std::fixed << std::setprecision(9)
            << sample.wall << "," << sample.device << "," << timing.release_sequence
            << "," << ClockSeconds(timing.release)
            << "," << ClockSeconds(timing.inference_start)
            << "," << ClockSeconds(timing.inference_finish)
            << "," << (published ? ClockSeconds(timing.action_published) : unavailable)
            << "," << DurationMilliseconds(timing.inference_start, timing.release)
            << "," << DurationMilliseconds(
                timing.inference_finish, timing.inference_start)
            << "," << (published ? DurationMilliseconds(
                timing.action_published, timing.inference_finish) : unavailable)
            << "," << (published ? DurationMilliseconds(
                timing.action_published, timing.release) : unavailable)
            << "," << sample.vx << "," << sample.vy << "," << sample.wz;
        for (size_t i = 0; i < sample.obs_size; ++i) row << "," << sample.obs[i];
        for (size_t i = 0; i < sample.raw_size; ++i) row << "," << sample.raw[i];
        for (size_t i = 0; i < sample.executor_size; ++i) row << "," << sample.executor[i];
        for (size_t i = 0; i < sample.adapter_size; ++i) row << "," << sample.adapter[i];
        row << "," << (published ? "published" : "inference_timeout")
            << "," << DurationMilliseconds(timing.inference_finish, timing.release)
            << "," << config_.inference_deadline_s * 1000.0
            << "," << sample.snapshot << "," << sample.prepare << "," << sample.assemble
            << "," << sample.infer << "," << sample.post << "," << sample.cpu
            << "," << sample.interaction_sequence
            << "," << std::quoted(sample.interaction, '"', '"') << "," << sample.phase
            << "," << sample.request_sequence << "," << sample.operation
            << "," << std::quoted(sample.request, '"', '"')
            << "," << std::quoted(sample.variant, '"', '"');
        runtime_logging::RecordCsv(
            policy_trace_stream_, policy_trace_header_, row.str());
    }

    void RecordPolicyApplyTrace(const PolicyTiming &timing,
                                RLRuntimeClock::time_point action_applied,
                                const std::vector<double> &target_pos) const {
        if (!policy_trace_enabled_ || !trace_worker_ || target_pos.size() > 128) return;
        PolicyTraceSnapshot sample;
        sample.apply = true;
        sample.wall = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        sample.timing = timing;
        sample.applied = action_applied;
        sample.target_size = target_pos.size();
        std::copy(target_pos.begin(), target_pos.end(), sample.target.begin());
        trace_worker_->Publish(sample);
    }

    void FormatPolicyApplyTrace(const PolicyTraceSnapshot &sample) const {
        const auto &timing = sample.timing;
        const auto action_applied = sample.applied;
        std::ostringstream row;
        row << std::fixed << std::setprecision(9)
            << sample.wall << "," << timing.release_sequence
            << "," << ClockSeconds(timing.action_published)
            << "," << ClockSeconds(action_applied)
            << "," << DurationMilliseconds(action_applied, timing.action_published)
            << "," << DurationMilliseconds(action_applied, timing.release);
        for (size_t i = 0; i < sample.target_size; ++i) row << "," << sample.target[i];
        runtime_logging::RecordCsv(policy_apply_trace_stream_,
            policy_apply_trace_header_, row.str());
    }

    void NotifyRuntimeEvent(RLRuntimeEventType type,
        std::uint64_t release_sequence,
        RLRuntimeClock::time_point timestamp) noexcept {
        switch (type) {
        case RLRuntimeEventType::RELEASE:
            RecordProgress(ProgressStage::RELEASE, release_sequence, timestamp); break;
        case RLRuntimeEventType::INFERENCE_START:
            RecordProgress(ProgressStage::START, release_sequence, timestamp); break;
        case RLRuntimeEventType::INFERENCE_FINISH:
            RecordProgress(ProgressStage::FINISH, release_sequence, timestamp); break;
        case RLRuntimeEventType::ACTION_PUBLISHED:
            RecordProgress(ProgressStage::PUBLISHED, release_sequence, timestamp); break;
        case RLRuntimeEventType::ACTION_APPLIED:
            RecordProgress(ProgressStage::APPLIED, release_sequence, timestamp); break;
        case RLRuntimeEventType::INFERENCE_DISCARDED:
            RecordProgress(ProgressStage::DISCARDED, release_sequence, timestamp); break;
        }
        if (!config_.runtime_observer) {
            return;
        }
        config_.runtime_observer->OnRuntimeEvent(
            {type, release_sequence, timestamp});
    }

    void UpdateRlFreq() {
        ++infer_count_window_;
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - infer_window_start_).count();
        if (elapsed >= 0.5) {
            const double hz = static_cast<double>(infer_count_window_) / elapsed;
            if (config_.rl_freq_hz) {
                config_.rl_freq_hz->store(hz, std::memory_order_relaxed);
            }
            infer_count_window_ = 0;
            infer_window_start_ = now;
        }
    }
};

// 工厂函数
std::unique_ptr<State> CreateStateRl(const RLConfig &cfg) {
    auto s = std::make_unique<StateRL>();
    s->SetConfig(cfg);
    return s;
}

std::unique_ptr<State> CreatePreparedStateRl(const RLConfig &cfg) {
    auto s = std::make_unique<StateRL>();
    s->SetConfig(cfg);
    s->PrepareRuntime();
    return s;
}

}  // namespace behavior_manager
