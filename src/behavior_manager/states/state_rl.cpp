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
#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <iomanip>
#include <iostream>
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
#include "state_factory.h"
namespace behavior_manager {

class StateRL : public State {
public:
    void SetConfig(const RLConfig &cfg) { config_ = cfg; }

    void OnEnter() override {
        std::cout << "[StateRL] 进入 RL 控制状态（异步推理）" << std::endl;

        if (output_) {
            output_->actuation_mode = robot_base::ActuationMode::HYBRID;
            output_->target_torque.assign(output_->target_pos.size(), 0.0);
            output_->kp = config_.kp;
            output_->kd = config_.kd;
        }

        // 每次进入都重新初始化策略运行时（支持策略切换后重新加载模型）。
        // PolicyExecutorConfig 整体透传，新增底层配置项无需在 common 逐字段同步。
        policy_.Init(config_.policy);
        ConfigurePolicyTrace();
        if (config_.runtime_observer) {
            config_.runtime_observer->OnRuntimeInitialized(
                policy_.GetRuntimeInfo());
        }

        safety_triggered_ = false;
        has_action_ = false;
        action_sequence_ = 0;
        applied_action_sequence_ = 0;
        release_sequence_ = 0;
        sample_release_sequence_ = 0;
        cached_release_sequence_ = 0;
        entry_target_transition_elapsed_ = 0.0;
        infer_count_window_ = 0;
        infer_window_start_ = std::chrono::steady_clock::now();
        if (config_.rl_freq_hz) {
            config_.rl_freq_hz->store(0.0, std::memory_order_relaxed);
        }

        time_since_last_infer_ = 0.0f;
        new_data_ready_ = false;
        running_ = true;

        // 可选策略适配器：负责参考动作、模型特殊输入和时序状态。
        policy_adapter_.reset();
        if (config_.policy_adapter.Enabled() && sensor_) {
            try {
                policy_adapter_ = policy_adapter::Create(
                    config_.policy_adapter, config_.policy);
                policy_adapter_->Reset(*sensor_);
                t_enter_ = std::chrono::steady_clock::now();
                std::cout << "[StateRL] policy_adapter 启用: type="
                    << policy_adapter_->Type() << ", reference="
                    << config_.policy_adapter.reference_file << std::endl;
            } catch (const std::exception &e) {
                std::cerr << "[StateRL] policy_adapter 加载失败，切安全态: "
                    << e.what() << std::endl;
                policy_adapter_.reset();
                safety_triggered_ = true;
                return;
            }
        }

        // 启动推理线程（配置从 RLConfig.infer_thread_cfg 注入）
        infer_loop_ = config_.infer_thread_cfg;
        infer_loop_.Start([this] { return InferStep(); });
    }

    void Run(float control_dt, float rl_dt) override {
        if (!sensor_ || !output_ || !command_)
            return;

        // 安全检查：IMU 倾角
        if (std::abs(sensor_->rpy[0]) > config_.max_roll ||
            std::abs(sensor_->rpy[1]) > config_.max_pitch) {
            std::cerr << "[StateRL] IMU 倾角超限! roll=" << sensor_->rpy[0]
                    << ", pitch=" << sensor_->rpy[1] << std::endl;
            safety_triggered_ = true;
            return;
        }

        // 基于控制循环时间的推理触发机制
        time_since_last_infer_ += control_dt;
        if (time_since_last_infer_ >= rl_dt) {
            // 保留调度余量，避免控制循环频率不能整除 rl_dt 时长期降频。
            time_since_last_infer_ -= rl_dt;

            const auto release_time = RLRuntimeClock::now();
            std::uint64_t release_sequence = 0;

            // 1) 写入最新传感器数据和命令到共享区（供推理线程读取）
            {
                std::lock_guard<std::mutex> lock(mutex_sensor_);
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
                sample_rl_dt_ = rl_dt;
                new_data_ready_ = true;
            }
            // 通知推理线程有新数据到达
            cv_sensor_.notify_one();
            NotifyRuntimeEvent(
                RLRuntimeEventType::RELEASE, release_sequence, release_time);
        }

        // 2) 读取最新推理结果，映射到关节目标
        bool action_applied = false;
        std::uint64_t applied_release_sequence = 0;
        RLRuntimeClock::time_point action_applied_time;
        PolicyTiming applied_timing;
        std::vector<double> applied_target_pos;
        {
            std::lock_guard<std::mutex> lock(mutex_action_);
            const bool transition_enabled =
                config_.entry_target_transition_duration > 0.0;
            const bool has_new_action =
                action_sequence_ != applied_action_sequence_;
            if (has_action_ && (!transition_enabled ||
                                has_new_action)) {
                std::vector<double> target_pos;
                policy_.MapActionToTargetPos(cached_action_, target_pos);
                ApplyEntryTargetTransition(
                    target_pos, rl_dt,
                    action_sequence_ - applied_action_sequence_);
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
        if (safety_triggered_) {
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
        running_ = false;
        cv_sensor_.notify_one();  // 唤醒阻塞的推理线程使其能够退出
        infer_loop_.Stop();
        if (config_.rl_freq_hz) {
            config_.rl_freq_hz->store(0.0, std::memory_order_relaxed);
        }
        std::cout << "[StateRL] 退出 RL 控制状态" << std::endl;
    }

private:
    void ApplyEntryTargetTransition(const std::vector<double> &raw_target,
                                    float rl_dt,
                                    std::uint64_t inference_steps) {
        const double duration = config_.entry_target_transition_duration;
        if (duration <= 0.0 || output_->target_pos.size() != raw_target.size()) {
            output_->target_pos = raw_target;
            return;
        }

        entry_target_transition_elapsed_ = std::min(duration,
            entry_target_transition_elapsed_ +
                std::max(0.0, static_cast<double>(rl_dt)) * inference_steps);
        const double progress = entry_target_transition_elapsed_ / duration;
        const double alpha = progress * progress;
        for (std::size_t i = 0; i < raw_target.size(); ++i) {
            output_->target_pos[i] =
                (1.0 - alpha) * output_->target_pos[i] + alpha * raw_target[i];
        }
    }

    // ==================== 推理线程回调 ====================

    bool InferStep() {
        std::array<double, 3> gyro, rpy, base_pos, base_vel;
        std::array<double, 4> base_quat;
        std::vector<double> joint_pos, joint_vel;
        double cmd_vx, cmd_vy, cmd_wz;
        double device_time;
        float rl_dt;
        std::uint64_t release_sequence = 0;
        RLRuntimeClock::time_point release_time;
        RLRuntimeClock::time_point inference_start;

        // 等待控制循环的主线程发来通知（严格同步到 control_dt 驱动的时钟）
        {
            std::unique_lock<std::mutex> lock(mutex_sensor_);
            cv_sensor_.wait(lock, [this] { return new_data_ready_ || !running_; });
            if (!running_) {
                return false;  // 退出线程
            }
            inference_start = RLRuntimeClock::now();
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
            rl_dt = sample_rl_dt_;
        }
        NotifyRuntimeEvent(RLRuntimeEventType::INFERENCE_START,
            release_sequence, inference_start);

        // 在 AssembleObs 前注入 tracker-specific 输入。
        if (policy_adapter_) {
            const auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_enter_).count();
            robot_base::RobotData snapshot;
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
            policy_adapter_->PrepareInputs(snapshot, elapsed, policy_);
        }

        // 组装观测 + 推理
        Eigen::VectorXf obs;
        policy_.AssembleObs(gyro,
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

        std::vector<double> raw_action;
        std::vector<double> executor_action;
        std::vector<double> action;
        if (policy_trace_enabled_) {
            policy_.Infer(obs, action, &raw_action);
            executor_action = action;
        } else {
            policy_.Infer(obs, action);
        }
        if (policy_adapter_) {
            policy_adapter_->OnAction(action);
        }
        const auto inference_finish = RLRuntimeClock::now();
        NotifyRuntimeEvent(RLRuntimeEventType::INFERENCE_FINISH,
            release_sequence, inference_finish);
        UpdateRlFreq();

        // 写入动作缓存
        RLRuntimeClock::time_point action_published;
        PolicyTiming timing;
        {
            std::lock_guard<std::mutex> lock(mutex_action_);
            cached_action_ = action;
            has_action_ = true;
            cached_release_sequence_ = release_sequence;
            action_published = RLRuntimeClock::now();
            timing = {release_sequence, release_time, inference_start,
                inference_finish, action_published};
            cached_timing_ = timing;
            ++action_sequence_;
        }
        NotifyRuntimeEvent(RLRuntimeEventType::ACTION_PUBLISHED,
            release_sequence, action_published);
        RecordPolicyTrace(obs, raw_action, executor_action, action,
            device_time, timing, cmd_vx, cmd_vy, cmd_wz);

        return true;  // 继续循环
    }

    // ==================== 配置与运行时 ====================

    RLConfig config_;
    rl_policy::PolicyExecutor policy_;

    // 推理线程
    robot_base::ThreadLoop infer_loop_;

    // 安全标志
    bool safety_triggered_ = false;

    // 策略输入协议适配器（policy_adapter.type 为空则不启用）
    std::unique_ptr<policy_adapter::PolicyAdapter> policy_adapter_;
    std::chrono::steady_clock::time_point t_enter_;

    // 推理频率统计（推理线程内读写）
    int infer_count_window_ = 0;
    std::chrono::steady_clock::time_point infer_window_start_ = std::chrono::steady_clock::now();

    // ==================== 线程间共享数据 ====================

    // 传感器快照（主线程写，推理线程读）
    std::mutex mutex_sensor_;
    std::condition_variable cv_sensor_;
    float time_since_last_infer_ = 0.0f;
    bool new_data_ready_ = false;
    bool running_ = true;

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
    float sample_rl_dt_ = 0.02f;

    // 动作缓存（推理线程写，主线程读）
    std::mutex mutex_action_;
    std::vector<double> cached_action_;
    bool has_action_ = false;
    std::uint64_t action_sequence_ = 0;
    std::uint64_t applied_action_sequence_ = 0;
    std::uint64_t release_sequence_ = 0;
    std::uint64_t sample_release_sequence_ = 0;
    RLRuntimeClock::time_point sample_release_time_;
    std::uint64_t cached_release_sequence_ = 0;
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

    PolicyTiming cached_timing_;

    static double ClockSeconds(RLRuntimeClock::time_point timestamp) {
        return std::chrono::duration<double>(timestamp.time_since_epoch()).count();
    }

    static double DurationMilliseconds(
        RLRuntimeClock::time_point finish,
        RLRuntimeClock::time_point start) {
        return std::chrono::duration<double, std::milli>(finish - start).count();
    }

    void ConfigurePolicyTrace() {
        const auto logging_config = runtime_logging::GetConfig();
        policy_trace_enabled_ = logging_config.telemetry_enabled &&
            logging_config.level == runtime_logging::Level::kDebug;
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
        policy_trace_header_ = header.str();

        std::ostringstream apply_header;
        apply_header << "wall_time_s,release_sequence,action_published_time_s,"
            "action_applied_time_s,publish_to_apply_ms,release_to_apply_ms";
        for (size_t i = 0; i < config_.kp.size(); ++i)
            apply_header << ",target_pos_" << i;
        policy_apply_trace_header_ = apply_header.str();
    }

    void RecordPolicyTrace(const Eigen::VectorXf &obs,
                            const std::vector<double> &raw_action,
                            const std::vector<double> &executor_action,
                            const std::vector<double> &adapter_action,
                            double device_time,
                            const PolicyTiming &timing,
                            double cmd_vx,
                            double cmd_vy,
                            double cmd_wz) const {
        if (!policy_trace_enabled_) return;

        const double wall_time = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::ostringstream row;
        row << std::fixed << std::setprecision(9)
            << wall_time << "," << device_time << "," << timing.release_sequence
            << "," << ClockSeconds(timing.release)
            << "," << ClockSeconds(timing.inference_start)
            << "," << ClockSeconds(timing.inference_finish)
            << "," << ClockSeconds(timing.action_published)
            << "," << DurationMilliseconds(timing.inference_start, timing.release)
            << "," << DurationMilliseconds(
                timing.inference_finish, timing.inference_start)
            << "," << DurationMilliseconds(
                timing.action_published, timing.inference_finish)
            << "," << DurationMilliseconds(timing.action_published, timing.release)
            << "," << cmd_vx << "," << cmd_vy << "," << cmd_wz;
        for (int i = 0; i < obs.size(); ++i) {
            row << "," << obs[i];
        }
        for (double value : raw_action) {
            row << "," << value;
        }
        for (double value : executor_action) {
            row << "," << value;
        }
        for (double value : adapter_action) {
            row << "," << value;
        }
        runtime_logging::RecordCsv(
            policy_trace_stream_, policy_trace_header_, row.str());
    }

    void RecordPolicyApplyTrace(const PolicyTiming &timing,
                                RLRuntimeClock::time_point action_applied,
                                const std::vector<double> &target_pos) const {
        if (!policy_trace_enabled_) return;

        const double wall_time = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        std::ostringstream row;
        row << std::fixed << std::setprecision(9)
            << wall_time << "," << timing.release_sequence
            << "," << ClockSeconds(timing.action_published)
            << "," << ClockSeconds(action_applied)
            << "," << DurationMilliseconds(action_applied, timing.action_published)
            << "," << DurationMilliseconds(action_applied, timing.release);
        for (double value : target_pos) row << "," << value;
        runtime_logging::RecordCsv(policy_apply_trace_stream_,
            policy_apply_trace_header_, row.str());
    }

    void NotifyRuntimeEvent(RLRuntimeEventType type,
        std::uint64_t release_sequence,
        RLRuntimeClock::time_point timestamp) noexcept {
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

}  // namespace behavior_manager
