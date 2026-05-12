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
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <filesystem>  // NOLINT(build/c++17)
#include <iostream>
#include <mutex>
#include <vector>
#include <memory>
#include <utility>

#include "behavior_state.h"
#include "motion_tracking_helper.h"
#include "rl_service.h"
#include "state_factory.h"
namespace behavior_manager {

class StateRL : public State {
public:
    void SetConfig(const RLConfig &cfg) { config_ = cfg; }

    void OnEnter() override {
        std::cout << "[StateRL] 进入 RL 控制状态（异步推理）" << std::endl;

        // 恢复 kp/kd（从 POWER_OFF 清零状态恢复）
        if (output_) {
            output_->kp = config_.kp;
            output_->kd = config_.kd;
        }

        // 每次进入都重新初始化策略运行时（支持策略切换后重新加载模型）
        rl_policy::PolicyExecutorConfig runtime_cfg;
        runtime_cfg.model_path = config_.model_path;
        runtime_cfg.action_scale = config_.action_scale;
        runtime_cfg.action_blend_ratio = config_.action_blend_ratio;
        runtime_cfg.rl_default_pos = config_.rl_default_pos;
        runtime_cfg.action_joint_index = config_.action_joint_index;
        runtime_cfg.obs_segments = config_.obs_segments;
        runtime_cfg.ang_vel_scale = config_.ang_vel_scale;
        runtime_cfg.dof_pos_scale = config_.dof_pos_scale;
        runtime_cfg.dof_vel_scale = config_.dof_vel_scale;
        runtime_cfg.euler_angle_scale = config_.euler_angle_scale;
        runtime_cfg.command_scale = config_.command_scale;
        runtime_cfg.dof_pos_subtract_default = config_.dof_pos_subtract_default;
        runtime_cfg.phase_period = config_.phase_period;
        runtime_cfg.gait_cycle = config_.gait_cycle;
        runtime_cfg.gait_left_offset = config_.gait_left_offset;
        runtime_cfg.gait_right_offset = config_.gait_right_offset;
        runtime_cfg.gait_left_ratio = config_.gait_left_ratio;
        runtime_cfg.gait_right_ratio = config_.gait_right_ratio;
        runtime_cfg.motion_length = config_.motion_length;
        runtime_cfg.strict_obs_dim_check = config_.strict_obs_dim_check;
        runtime_cfg.custom_scalar_defaults = config_.custom_scalar_defaults;
        runtime_cfg.custom_array_dims = config_.custom_array_dims;
        policy_.Init(runtime_cfg);

        safety_triggered_ = false;
        has_action_ = false;
        infer_count_window_ = 0;
        infer_window_start_ = std::chrono::steady_clock::now();
        if (config_.rl_freq_hz) {
            config_.rl_freq_hz->store(0.0, std::memory_order_relaxed);
        }

        time_since_last_infer_ = 0.0f;
        new_data_ready_ = false;
        running_ = true;

        // BeyondMimic 风格 tracking 支持：若策略配置了 motion_file，加载 npz + reset yaw 对齐
        tracking_helper_.reset();
        if (!config_.motion_file.empty() && sensor_) {
            try {
                tracking_helper_ = std::make_unique<MotionTrackingHelper>();
                tracking_helper_->Load(config_.motion_file, config_.motion_fps);
                // 当前仅 G1 torso_link 支持，腰关节固定取 [12,13,14]
                const double yaw_q = (sensor_->joint_pos.size() > 14) ? sensor_->joint_pos[12] : 0.0;
                const double roll_q = (sensor_->joint_pos.size() > 14) ? sensor_->joint_pos[13] : 0.0;
                const double pitch_q = (sensor_->joint_pos.size() > 14) ? sensor_->joint_pos[14] : 0.0;
                tracking_helper_->Reset(*sensor_, yaw_q, roll_q, pitch_q, config_.anchor_yaw_align);
                t_enter_ = std::chrono::steady_clock::now();
                std::cout << "[StateRL] tracking 启用: motion=" << config_.motion_file
                        << ", duration=" << tracking_helper_->Duration() << "s" << std::endl;
            } catch (const std::exception &e) {
                std::cerr << "[StateRL] tracking 加载失败，回退到普通 RL: " << e.what() << std::endl;
                tracking_helper_.reset();
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
            time_since_last_infer_ = 0.0f;  // 重置累加器

            // 1) 写入最新传感器数据和命令到共享区（供推理线程读取）
            {
                std::lock_guard<std::mutex> lock(mutex_sensor_);
                sample_gyro_ = sensor_->gyro;
                sample_rpy_ = sensor_->rpy;
                sample_joint_pos_ = sensor_->joint_pos;
                sample_joint_vel_ = sensor_->joint_vel;
                sample_base_pos_ = sensor_->base_pos;
                sample_base_quat_ = sensor_->base_quat;
                sample_base_vel_ = {
                    {sensor_->base_vel[0], sensor_->base_vel[1], sensor_->base_vel[2]}};
                sample_cmd_vx_ = command_->vx;
                sample_cmd_vy_ = command_->vy;
                sample_cmd_wz_ = command_->wz;
                sample_rl_dt_ = rl_dt;
                new_data_ready_ = true;
            }
            // 通知推理线程有新数据到达
            cv_sensor_.notify_one();
        }

        // 2) 读取最新推理结果，映射到关节目标
        {
            std::lock_guard<std::mutex> lock(mutex_action_);
            if (has_action_) {
                std::vector<double> target_pos;
                policy_.MapActionToTargetPos(cached_action_, target_pos);
                output_->target_pos = std::move(target_pos);
                int ndof = static_cast<int>(output_->target_pos.size());
                output_->target_vel.assign(ndof, 0.0);
                output_->enable = true;
            }
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
    // ==================== 推理线程回调 ====================

    bool InferStep() {
        std::array<double, 3> gyro, rpy, base_pos, base_vel;
        std::array<double, 4> base_quat;
        std::vector<double> joint_pos, joint_vel;
        double cmd_vx, cmd_vy, cmd_wz;
        float rl_dt;

        // 等待控制循环的主线程发来通知（严格同步到 control_dt 驱动的时钟）
        {
            std::unique_lock<std::mutex> lock(mutex_sensor_);
            cv_sensor_.wait(lock, [this] { return new_data_ready_ || !running_; });
            if (!running_) {
                return false;  // 退出线程
            }
            new_data_ready_ = false;

            // 拷贝传感器快照
            gyro = sample_gyro_;
            rpy = sample_rpy_;
            joint_pos = sample_joint_pos_;
            joint_vel = sample_joint_vel_;
            base_pos = sample_base_pos_;
            base_quat = sample_base_quat_;
            base_vel = sample_base_vel_;
            cmd_vx = sample_cmd_vx_;
            cmd_vy = sample_cmd_vy_;
            cmd_wz = sample_cmd_wz_;
            rl_dt = sample_rl_dt_;
        }

        // BeyondMimic 风格 tracking：每帧把 motion + anchor 数据推给 policy（在 AssembleObs 前）
        if (tracking_helper_) {
            const auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - t_enter_).count();
            const double yaw_q = (joint_pos.size() > 14) ? joint_pos[12] : 0.0;
            const double roll_q = (joint_pos.size() > 14) ? joint_pos[13] : 0.0;
            const double pitch_q = (joint_pos.size() > 14) ? joint_pos[14] : 0.0;
            robot_base::RobotData snapshot;
            snapshot.base_pos = base_pos;
            snapshot.base_quat = base_quat;
            snapshot.joint_pos = joint_pos;
            tracking_helper_->Update(elapsed, snapshot, yaw_q, roll_q, pitch_q);

            policy_.SetCustomArray("motion_command",
                                    tracking_helper_->MotionCommand().data(),
                                    static_cast<int>(tracking_helper_->MotionCommand().size()));
            policy_.SetCustomArray("motion_anchor_pos_b",
                                    tracking_helper_->AnchorPosB().data(), 3);
            policy_.SetCustomArray("motion_anchor_ori_b",
                                    tracking_helper_->AnchorOriB().data(), 6);

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

        std::vector<double> action;
        policy_.Infer(obs, action);
        UpdateRlFreq();

        // 写入动作缓存
        {
            std::lock_guard<std::mutex> lock(mutex_action_);
            cached_action_ = std::move(action);
            has_action_ = true;
        }

        return true;  // 继续循环
    }

    // ==================== 配置与运行时 ====================

    RLConfig config_;
    rl_policy::PolicyExecutor policy_;

    // 推理线程
    robot_base::ThreadLoop infer_loop_;

    // 安全标志
    bool safety_triggered_ = false;

    // BeyondMimic 风格 tracking 辅助（motion_file 为空则不启用）
    std::unique_ptr<MotionTrackingHelper> tracking_helper_;
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
    double sample_cmd_vx_ = 0;
    double sample_cmd_vy_ = 0;
    double sample_cmd_wz_ = 0;
    float sample_rl_dt_ = 0.02f;

    // 动作缓存（推理线程写，主线程读）
    std::mutex mutex_action_;
    std::vector<double> cached_action_;
    bool has_action_ = false;

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
