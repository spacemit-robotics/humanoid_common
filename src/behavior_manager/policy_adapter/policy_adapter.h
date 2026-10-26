/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file policy_adapter.h
 * @brief RL 策略输入协议适配器内部接口
 */

#ifndef POLICY_ADAPTER_H
#define POLICY_ADAPTER_H

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rl_service.h"
#include "robot_base.h"

namespace behavior_manager {
namespace policy_adapter {

/**
 * @brief MJLab 参考动作输出合成配置
 */
struct ReferenceActionConfig {
    std::vector<int> joint_indices;
    double residual_scale = 0.0;
    double residual_clip = 1.0;

    bool Enabled() const { return !joint_indices.empty(); }
};

/**
 * @brief 一条外部关节轨迹及其控制权范围
 */
struct JointTrajectoryActionConfig {
    std::string name;
    std::string file;
    std::vector<int> joint_indices;
    double motion_fps = 50.0;
    double playback_speed = 1.0;
    bool hold_last_frame = false;
    double max_hold_duration = 0.0;  // Zero preserves indefinite hold until CANCEL.
    std::vector<double> kp;
    std::vector<double> kd;
};

/**
 * @brief RL 内部关节轨迹接管配置
 */
struct JointTrajectoryConfig {
    std::string applied_action_term;
    double blend_in_duration = 0.25;
    double blend_out_duration = 0.25;
    // Positive: enter from measured positions at this rad/s with full trajectory gains.
    double measured_entry_max_velocity = 0.0;
    bool linear_blend = false;
    std::vector<JointTrajectoryActionConfig> actions;

    bool Enabled() const { return !actions.empty(); }
};

/**
 * @brief 一条由策略完整跟踪的参考动作
 */
struct ReferenceMotionConfig {
    std::string name;
    std::string file;
    double motion_fps = 50.0;
    double playback_speed = 1.0;
    bool loop = false;
    double loop_pause = 0.0;
};

/**
 * @brief 同一策略可选择的参考动作目录
 */
struct ReferenceMotionCatalogConfig {
    std::string default_action;
    double transition_duration = 0.25;
    std::vector<ReferenceMotionConfig> actions;

    bool Enabled() const { return !actions.empty(); }
};

/**
 * @brief 单个策略的适配器配置
 *
 * 该结构仅在 behavior_manager 内部使用。机型仓库通过 YAML 提供参数，
 * robot_base 和 model_zoo/rl 不感知具体 tracker 协议。
 */
struct Config {
    std::string type;
    std::string reference_file;
    double motion_fps = 50.0;
    double playback_speed = 1.0;
    bool loop = false;
    double loop_pause = 0.0;
    bool manual_start = false;

    int anchor_body_index = -1;
    std::vector<int> anchor_waist_joint_indices;
    bool anchor_yaw_align = true;

    int future_frames = 0;
    int future_step = 1;
    std::vector<int> future_steps;

    ReferenceActionConfig reference_action;
    JointTrajectoryConfig joint_trajectory;
    ReferenceMotionCatalogConfig reference_motion_catalog;

    struct HistoryActorConfig {
        struct Motion {
            std::string name;
            std::string model_path;
            std::string file;
            bool root_pose = false;  // CSV: latent + xyz + quaternion(wxyz).
            std::string standby_file;
            double start_time = 0.0;
            double end_time = -1.0;  // Exclusive; -1 reads through the final CSV frame.
            bool mask_root_position = false;
        };
        std::string command_type;
        std::string observation_term;
        std::string command_input;
        int history_frames = 0;
        double dt = 0.02;
        double gyro_scale = 0.2;
        double velocity_scale = 0.05;
        double observation_clip = 100.0;
        double gait_frequency = 2.0;
        double frequency_center = 2.0;
        double stance_ratio = 0.5;
        double command_scale = 0.2;
        double command_filter_alpha = 1.0;
        double stand_threshold = 0.1;
        double transition_duration = 0.5;
        double root_position_error_clip = 1.0;
        std::vector<Motion> motions;  // First entry is the stationary default.
    } history_actor;

    // A paired locomotion controller shares one observation/action history.
    struct StanceMotionConfig {
        std::string motion_model_path;
        std::string run_model_path;
        bool gravity_observation = false;
        bool scaled_action_history = true;
        std::string observation_term;
        std::string command_input;
        std::string history_input;
        int history_frames = 10;
        std::array<double, 3> command_up = {};
        std::array<double, 3> command_down = {};
        std::array<double, 3> stand_threshold = {};
        std::array<double, 3> stance_command = {};
        std::array<double, 2> forward_threshold = {};
        double turn_scale = 1.0;
        std::string orientation_frame = "world";
        bool transpose_rotation = false;
        std::vector<int> interaction_joint_indices;
        std::vector<double> interaction_neutral_positions;
        int interaction_release_frames = 0;
    } stance_motion;

    bool Enabled() const { return !type.empty(); }
};

/**
 * @brief 从机型 YAML 加载指定策略的 policy_adapter 配置
 */
Config LoadConfig(const std::string &yaml_path,
    const std::string &policy_name,
    const std::string &robot_dir);

/**
 * @brief StateRL 使用的策略适配器内部接口
 */
class PolicyAdapter {
public:
    virtual ~PolicyAdapter() = default;

    /** @brief 每次进入 RL 状态时重置参考时间轴和模型侧状态 */
    virtual void Reset(const robot_base::RobotData &robot) = 0;

    /** @brief Select a preloaded executor without leaving RL or resetting history. */
    virtual rl_policy::PolicyExecutor &SelectExecutor(
        const std::array<double, 3> &command, rl_policy::PolicyExecutor &primary) {
        (void)command;
        return primary;
    }

    /** @brief Interrupt inference owned by this adapter before joining its thread. */
    virtual void RequestInferenceTermination() {}

    /** @brief Label of the selected internal model for inference diagnostics. */
    virtual std::string ActiveModelName() const { return {}; }

    /** @brief Ready for a running handoff once stationary and without an active action. */
    virtual bool ReadyForPolicySwitch() const { return false; }

    /** @brief 推理前根据机器人状态和参考数据注入自定义模型输入 */
    virtual void PrepareInputs(const robot_base::RobotData &robot,
        double elapsed_s,
        rl_policy::PolicyExecutor &policy) = 0;

    /**
     * @brief 推理后按策略协议处理 action 副本
     *
     * 适配器可以保存模型输出供下一帧使用，也可以改写送往通用关节目标映射的
     * action。PolicyExecutor 内部状态不受改写影响。
     */
    virtual void OnAction(std::vector<double> &action) {}

    /** @brief Current external takeover weight, evaluated before building observations. */
    virtual double InteractionWeight() const { return 0.0; }

    /** @brief Overlay gains for this action sample; called on the inference thread. */
    virtual void ApplyJointGains(std::vector<double> &kp, std::vector<double> &kd) const {
        (void)kp;
        (void)kd;
    }

    /** @brief 开始手动参考时间轴；重复请求不会重新计时。 */
    virtual bool StartPlayback(double elapsed_s) { return false; }

    /** @brief 接收一次带序号的交互动作请求 */
    virtual void HandleInteractionRequest(
        const robot_base::InteractionRequest &request,
        double elapsed_s) {
        (void)request;
        (void)elapsed_s;
    }

    /** @brief 返回最近一次交互动作执行状态 */
    virtual robot_base::InteractionStatus GetInteractionStatus() const {
        return {};
    }

    /** @brief 适配器类型，用于日志 */
    virtual const char *Type() const = 0;
};

/**
 * @brief 根据 YAML type 创建适配器
 */
std::unique_ptr<PolicyAdapter> Create(
    const Config &config,
    const rl_policy::PolicyExecutorConfig &policy_config);

}  // namespace policy_adapter
}  // namespace behavior_manager

#endif  // POLICY_ADAPTER_H
