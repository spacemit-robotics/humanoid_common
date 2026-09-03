/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_zero.cpp
 * @brief 回零位状态 — 五次多项式平滑到 RL 训练初始位置
 */

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "runtime_logger.h"
#include "state_factory.h"
namespace behavior_manager {

class StateZero : public State {
public:
    void SetDefaultPos(const std::vector<double> &pos) { default_pos_ = pos; }

    void SetTransitionConfig(const ZeroTransitionConfig &config) {
        transition_config_ = config;
    }

    void SetKpKd(const std::vector<double> &kp, const std::vector<double> &kd) {
        kp_ = kp;
        kd_ = kd;
    }

    void OnEnter() override {
        std::cout << "[StateZero] 进入回零位状态" << std::endl;
        timer_ = 0;
        settled_duration_ = 0;
        max_position_error_ = std::numeric_limits<double>::infinity();
        max_position_error_index_ = 0;
        max_velocity_ = std::numeric_limits<double>::infinity();
        finished_ = false;
        feedback_valid_ = false;
        blocked_request_active_ = false;

        // 恢复 kp/kd（从 POWER_OFF 清零状态恢复）
        if (output_) {
            output_->actuation_mode = robot_base::ActuationMode::HYBRID;
            output_->target_torque.assign(default_pos_.size(), 0.0);
            output_->kp = kp_;
            output_->kd = kd_;
        }

        // 从当前实际关节位置开始插值，确保第一帧目标与实际位置一致
        if (sensor_) {
            init_pos_ = sensor_->joint_pos;
        }
    }

    void Run(float control_dt, float rl_dt) override {
        (void)rl_dt;
        if (!output_ || init_pos_.empty() || default_pos_.empty())
            return;

        int ndof = static_cast<int>(default_pos_.size());
        output_->target_pos.resize(ndof);
        output_->target_vel.resize(ndof);
        output_->enable = true;

        if (timer_ < transition_config_.move_duration) {
            // 五次多项式插值: s(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5
            double s = timer_ / transition_config_.move_duration;
            double alpha = 10 * s * s * s - 15 * s * s * s * s + 6 * s * s * s * s * s;
            double alpha_dot = (30 * s * s - 60 * s * s * s +
                30 * s * s * s * s) / transition_config_.move_duration;

            for (int i = 0; i < ndof; i++) {
                double p0 = (i < static_cast<int>(init_pos_.size())) ? init_pos_[i] : 0.0;
                double p1 = default_pos_[i];
                output_->target_pos[i] = p0 + alpha * (p1 - p0);
                output_->target_vel[i] = alpha_dot * (p1 - p0);
            }
        } else {
            // 不按插值时间放行；等待关节反馈满足收敛条件。
            for (int i = 0; i < ndof; i++) {
                output_->target_pos[i] = default_pos_[i];
                output_->target_vel[i] = 0;
            }
            UpdateReadiness(control_dt);
        }

        timer_ += control_dt;
    }

    StateName CheckTransition() override {
        // key=3 且已到位 → RL 控制
        if (command_ && command_->key == 3 && finished_) {
            command_->key = 0;
            return StateName::RL;
        }
        if (command_ && command_->key == 3 && !finished_) {
            ReportBlockedTransition();
            blocked_request_active_ = true;
        } else {
            blocked_request_active_ = false;
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
        return StateName::ZERO;
    }

    bool ReadyForRl() const override { return finished_; }

    void OnExit() override {
        std::cout << "[StateZero] 退出回零位状态 (用时: " << timer_ << "s)" << std::endl;
    }

private:
    bool MeasurePose() {
        feedback_valid_ = false;
        if (!sensor_ || sensor_->joint_pos.size() != default_pos_.size() ||
            sensor_->joint_vel.size() != default_pos_.size()) {
            return false;
        }

        max_position_error_ = 0.0;
        max_position_error_index_ = 0;
        max_velocity_ = 0.0;
        for (std::size_t i = 0; i < default_pos_.size(); ++i) {
            if (!std::isfinite(default_pos_[i]) ||
                !std::isfinite(sensor_->joint_pos[i]) ||
                !std::isfinite(sensor_->joint_vel[i])) {
                return false;
            }
            const double position_error =
                std::abs(default_pos_[i] - sensor_->joint_pos[i]);
            if (position_error > max_position_error_) {
                max_position_error_ = position_error;
                max_position_error_index_ = i;
            }
            max_velocity_ = std::max(max_velocity_,
                std::abs(sensor_->joint_vel[i]));
        }
        feedback_valid_ = true;
        return true;
    }

    void UpdateReadiness(double control_dt) {
        if (finished_) return;

        const bool converged = MeasurePose() &&
            std::isfinite(control_dt) && control_dt > 0.0 &&
            max_position_error_ <= transition_config_.position_tolerance &&
            max_velocity_ <= transition_config_.velocity_tolerance;
        settled_duration_ = converged ? settled_duration_ + control_dt : 0.0;
        if (settled_duration_ + 1.0e-12 < transition_config_.settle_duration) {
            return;
        }

        finished_ = true;
        std::cout << "[StateZero] 真实关节已稳定到位："
            << FormatPoseStatus() << std::endl;
    }

    static double RadiansToDegrees(double radians) {
        constexpr double kRadiansToDegrees =
            180.0 / 3.14159265358979323846;
        return radians * kRadiansToDegrees;
    }

    std::string FormatPoseStatus() const {
        if (!feedback_valid_) return "关节反馈无效";

        const double position_error = RadiansToDegrees(max_position_error_);
        const double position_limit =
            RadiansToDegrees(transition_config_.position_tolerance);
        const double velocity = RadiansToDegrees(max_velocity_);
        const double velocity_limit =
            RadiansToDegrees(transition_config_.velocity_tolerance);

        std::ostringstream stream;
        stream << std::fixed << std::setprecision(2)
            << "关节[" << max_position_error_index_
            << "]最大位置误差=" << position_error << "°"
            << "（限值=" << position_limit << "°";
        if (position_error > position_limit) {
            stream << "，超出=" << position_error - position_limit << "°";
        }
        stream << "），最大速度=" << velocity << "°/s"
            << "（限值=" << velocity_limit << "°/s";
        if (velocity > velocity_limit) {
            stream << "，超出=" << velocity - velocity_limit << "°/s";
        }
        stream << "），稳定计时=" << settled_duration_ << "/"
            << transition_config_.settle_duration << "s";
        return stream.str();
    }

    void ReportBlockedTransition() {
        if (blocked_request_active_) return;

        MeasurePose();
        const std::string pose_status = FormatPoseStatus();
        const std::string message =
            "ZERO -> RL blocked by pose readiness: " + pose_status;
        std::cout << "[StateZero] 未通过到位校验，不能进入 RL："
            << pose_status << std::endl;
        runtime_logging::Log(
            runtime_logging::Level::kWarning, message, false);
    }

    std::vector<double> default_pos_;  // RL 训练初始位置
    std::vector<double> init_pos_;     // 运动起始位置
    std::vector<double> kp_;           // PD 增益（OnEnter 时恢复）
    std::vector<double> kd_;
    ZeroTransitionConfig transition_config_;
    double timer_ = 0;
    double settled_duration_ = 0;
    double max_position_error_ = std::numeric_limits<double>::infinity();
    std::size_t max_position_error_index_ = 0;
    double max_velocity_ = std::numeric_limits<double>::infinity();
    bool finished_ = false;
    bool feedback_valid_ = false;
    bool blocked_request_active_ = false;
};

// 工厂函数（带默认位置参数）
std::unique_ptr<State> CreateStateZero(const std::vector<double> &default_pos,
                                        const ZeroTransitionConfig &transition_config,
                                        const std::vector<double> &kp,
                                        const std::vector<double> &kd) {
    auto s = std::make_unique<StateZero>();
    s->SetDefaultPos(default_pos);
    s->SetTransitionConfig(transition_config);
    s->SetKpKd(kp, kd);
    return s;
}

}  // namespace behavior_manager
