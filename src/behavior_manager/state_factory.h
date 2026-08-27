/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file state_factory.h
 * @brief 状态工厂函数声明
 *
 * 各状态的创建函数，供 BehaviorManager 注册使用。
 */

#ifndef STATE_FACTORY_H
#define STATE_FACTORY_H

#include <atomic>
#include <array>
#include <chrono>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "behavior_state.h"  // 内部实现，位于 src/
#include "policy_adapter/policy_adapter.h"
#include "robot_base.h"      // robot_base::ThreadLoop
#include "rl_service.h"      // rl_policy::PolicyExecutorConfig
namespace behavior_manager {

using RLRuntimeClock = std::chrono::steady_clock;

enum class RLRuntimeEventType {
    RELEASE,
    INFERENCE_START,
    INFERENCE_FINISH,
    ACTION_PUBLISHED,
    ACTION_APPLIED,
};

struct RLRuntimeEvent {
    RLRuntimeEventType type;
    std::uint64_t release_sequence;
    RLRuntimeClock::time_point timestamp;
};

/**
 * @brief StateRL 可选的低开销运行观测接口。
 *
 * 生产路径默认不注入 observer。benchmark 注入时，事件在 StateRL 的真实控制线程
 * 和推理线程边界产生；实现不得阻塞、分配内存或抛出异常。
 */
class RLRuntimeObserver {
public:
    virtual ~RLRuntimeObserver() = default;

    virtual void OnRuntimeInitialized(
        const rl_policy::InferenceRuntimeInfo &runtime) = 0;
    virtual void OnRuntimeEvent(const RLRuntimeEvent &event) noexcept = 0;
};

// StateRL 的配置结构（需要在工厂函数中传入）
struct RLConfig {
    // 完整透传底层执行器配置，避免新增 RL YAML 字段时在 common 层漏传。
    rl_policy::PolicyExecutorConfig policy;
    std::array<double, 3> command_init = {0.0, 0.0, 0.0};
    double rl_dt = 0.02;                      // 策略推理周期（秒）
    int infer_decimation = 4;                  // 推理降频
    robot_base::ThreadLoop infer_thread_cfg;  // 推理线程配置（robot_base.threads.rl_infer）
    double max_roll = 0.7;                    // 最大翻滚角 (rad)
    double max_pitch = 0.7;                   // 最大俯仰角 (rad)

    // ---- 策略输入协议适配（可选；type 为空表示普通 RL 策略）----
    policy_adapter::Config policy_adapter;
    std::vector<double> zero_target_pos;  // ZERO 阶段目标位姿（可选，空则用 rl_default_pos）
    double entry_target_transition_duration = 0.0;  // RL 入场目标位置过渡时长（秒）

    // ---- RL 策略增益 ----
    std::vector<double> kp;
    std::vector<double> kd;

    // ---- 运行时统计（由 behavior_manager 注入）----
    std::atomic<double> *rl_freq_hz = nullptr;
    RLRuntimeObserver *runtime_observer = nullptr;  // 非持有；仅在 StateRL 生命周期内使用
};

/**
 * @brief 从机型 YAML 装配一个策略对应的完整 StateRL 配置。
 */
RLConfig LoadRLStateConfig(const std::string &yaml_path,
    const std::string &policy_name,
    const std::string &robot_dir);

// 工厂函数
std::unique_ptr<State> CreateStatePowerOff();
std::unique_ptr<State> CreateStateDamp(const std::vector<double> &damp_kd);
std::unique_ptr<State> CreateStateHome(const std::vector<double> &default_pos,
        double gain_ramp_duration,
        double move_duration,
        const std::vector<double> &kp,
        const std::vector<double> &kd);
std::unique_ptr<State> CreateStateZero(const std::vector<double> &default_pos,
                                        double duration,
                                        const std::vector<double> &kp,
                                        const std::vector<double> &kd);
std::unique_ptr<State> CreateStateRl(const RLConfig &cfg);
std::unique_ptr<State> CreateStateSafety();

}  // namespace behavior_manager

#endif  // STATE_FACTORY_H
