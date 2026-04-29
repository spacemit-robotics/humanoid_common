/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file behavior_manager.h
 * @brief 行为管理器 - FSM 状态机控制
 *
 * 提供基于有限状态机的机器人行为管理，支持上电、回零、RL 控制等状态。
 * 完全由 YAML 配置驱动，与具体机器人型号解耦。
 */
#ifndef BEHAVIOR_MANAGER_H
#define BEHAVIOR_MANAGER_H

#include <memory>
#include <string>
#include <array>
#include <vector>

#include "robot_base.h"
namespace behavior_manager {

/**
 * @brief 状态枚举
 */
enum class StateName {
    POWER_OFF,  ///< 完全失力：关节零力矩（ESC 随时可触发）
    DAMP,       ///< 阻尼保持：kp=0, kd=配置值，软性支撑防硬倒
    ZERO,       ///< 回零位：平滑到 RL 训练初始位置
    RL,         ///< RL 控制：异步推理输出动作
    SAFETY      ///< 安全保护：IMU 倾角/关节限位触发
};

/**
 * @brief 状态名转字符串
 * @param s 状态枚举
 * @return 状态名称字符串
 */
const char *StateNameStr(StateName s);

/**
 * @brief 控制输出
 */
struct ControlOutput {
    std::vector<double> target_pos;  ///< 目标关节位置 (rad)
    std::vector<double> target_vel;  ///< 目标关节速度 (rad/s)
    std::vector<double> kp, kd;      ///< PD 参数
    bool enable = false;             ///< 使能标志
};

// ==================== 对外接口 ====================

/**
 * @brief 行为管理器 — 公开接口
 *
 * 封装 FSM + ThreadLoop + 状态注册，提供简洁的对外 API。
 * application 层仅需与此类交互。
 */
class BehaviorManagerClass {
public:
    /**
     * @brief 构造函数
     * @param config_path 机器人配置文件路径（YAML）
     */
    explicit BehaviorManagerClass(const std::string &config_path);
    ~BehaviorManagerClass();

    // 禁止拷贝
    BehaviorManagerClass(const BehaviorManagerClass &) = delete;
    BehaviorManagerClass &operator=(const BehaviorManagerClass &) = delete;

    /**
     * @brief 初始化（加载配置、注册状态、初始化 FSM）
     */
    void Init();

    /**
     * @brief 执行一步控制
     * @param control_dt 控制周期 (s)，对应主循环频率
     * @param rl_dt RL 推理周期 (s)，对应训练时的推理频率
     */
    void Step(float control_dt, float rl_dt);

    /**
     * @brief 设置传感器数据
     */
    void SetSensorData(const robot_base::RobotData &data);

    /**
     * @brief 设置命令
     */
    void SetCommand(const robot_base::Command &cmd);

    /**
     * @brief 获取控制输出
     */
    const ControlOutput &GetOutput() const;

    /**
     * @brief 获取当前状态名
     */
    StateName CurrentState() const;

    /**
     * @brief 获取当前生效策略名
     */
    std::string CurrentPolicyName() const;

    /**
     * @brief 获取 RL 实时推理频率（Hz）
     */
    double GetRlFreq() const;

    /**
     * @brief 是否正在运行
     */
    bool IsRunning() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace behavior_manager

#endif  // BEHAVIOR_MANAGER_H
