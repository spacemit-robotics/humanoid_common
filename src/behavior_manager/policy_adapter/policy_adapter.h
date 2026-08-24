/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file policy_adapter.h
 * @brief RL 策略输入协议适配器内部接口
 */

#ifndef POLICY_ADAPTER_H
#define POLICY_ADAPTER_H

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

    int anchor_body_index = -1;
    std::vector<int> anchor_waist_joint_indices;
    bool anchor_yaw_align = true;

    int future_frames = 0;
    int context_length = 0;
    std::vector<int> future_steps;

    ReferenceActionConfig reference_action;

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
