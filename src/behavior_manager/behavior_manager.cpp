/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file behavior_manager.cpp
 * @brief BehaviorManager 实现
 *
 * 封装 FSM + 状态注册 + 数据管理。
 * application 层仅需与此类交互。
 *
 * 配置：behavior_manager 命名空间
 */

#include "behavior_manager.h"  // 对外接口，位于 include/

#include <atomic>
#include <iostream>
#include <string>
#include <vector>

#include "behavior_fsm.h"      // 内部实现，位于 src/
#include "state_factory.h"     // 内部实现，位于 src/
namespace behavior_manager {

const char *StateNameStr(StateName s) {
    switch (s) {
    case StateName::POWER_OFF:
        return "POWER_OFF";
    case StateName::DAMP:
        return "DAMP";
    case StateName::ZERO:
        return "ZERO";
    case StateName::RL:
        return "RL";
    case StateName::SAFETY:
        return "SAFETY";
    default:
        return "UNKNOWN";
    }
}

namespace {

RLConfig ToRLConfig(const rl_policy::LoadedPolicyConfig &loaded_cfg) {
    RLConfig rc;
    rc.model_path = loaded_cfg.exec_cfg.model_path;
    rc.action_scale = loaded_cfg.exec_cfg.action_scale;
    rc.action_blend_ratio = loaded_cfg.exec_cfg.action_blend_ratio;
    rc.rl_default_pos = loaded_cfg.exec_cfg.rl_default_pos;
    rc.action_joint_index = loaded_cfg.exec_cfg.action_joint_index;
    rc.infer_decimation = loaded_cfg.infer_decimation > 0 ? loaded_cfg.infer_decimation : 1;
    rc.max_roll = loaded_cfg.max_roll;
    rc.max_pitch = loaded_cfg.max_pitch;
    rc.obs_segments = loaded_cfg.exec_cfg.obs_segments;
    rc.ang_vel_scale = loaded_cfg.exec_cfg.ang_vel_scale;
    rc.dof_pos_scale = loaded_cfg.exec_cfg.dof_pos_scale;
    rc.dof_vel_scale = loaded_cfg.exec_cfg.dof_vel_scale;
    rc.euler_angle_scale = loaded_cfg.exec_cfg.euler_angle_scale;
    rc.command_scale = loaded_cfg.exec_cfg.command_scale;
    rc.dof_pos_subtract_default = loaded_cfg.exec_cfg.dof_pos_subtract_default;
    rc.phase_period = loaded_cfg.exec_cfg.phase_period;
    rc.gait_cycle = loaded_cfg.exec_cfg.gait_cycle;
    rc.gait_left_offset = loaded_cfg.exec_cfg.gait_left_offset;
    rc.gait_right_offset = loaded_cfg.exec_cfg.gait_right_offset;
    rc.gait_left_ratio = loaded_cfg.exec_cfg.gait_left_ratio;
    rc.gait_right_ratio = loaded_cfg.exec_cfg.gait_right_ratio;
    rc.motion_length = loaded_cfg.exec_cfg.motion_length;
    rc.strict_obs_dim_check = loaded_cfg.exec_cfg.strict_obs_dim_check;
    rc.custom_scalar_defaults = loaded_cfg.exec_cfg.custom_scalar_defaults;
    return rc;
}

}  // namespace

class BehaviorManagerClass::Impl {
public:
    FSM fsm;
    robot_base::RobotData sensor;
    robot_base::Command command;
    ControlOutput output;
    std::string config_path;
    std::string robot_dir;  // 机器人资源根目录（绝对路径）
    std::vector<double> init_kp;
    std::vector<double> init_kd;
    bool initialized = false;
    std::string pending_policy;               // 待生效的策略名
    std::string active_policy;                // 当前已加载的策略名
    bool has_rl = false;                      // 是否配置了 RL 状态
    std::atomic<double> rl_freq_hz{0.0};      // RL 实时推理频率（Hz）
    robot_base::ThreadLoop infer_thread_cfg;  // 推理线程配置（robot_base.threads.rl_infer）

    void LoadConfig(const std::string &path) {
        config_path = path;

        // 从 YAML 初始化 RobotBase（获取 num_dof 并初始化关节向量大小）
        sensor = robot_base::RobotData::FromYaml(path);
        int num_dof = sensor.num_dof;

        robot_base::YamlFile yaml_file = robot_base::YamlFile::Load(path);

        // 解析 robot_dir（绝对路径）
        robot_dir =
            yaml_file.ToAbsPath(yaml_file.Read<std::string>("robot_base.robot_dir").value());

        // 读取推理线程配置（robot_base.threads.rl_infer）
        infer_thread_cfg = robot_base::ThreadLoop::FromYaml(yaml_file, "rl_infer");

        // 机器人基本信息（用于日志输出，从 robot_base 获取）
        std::cout << "[BehaviorManager] 机器人: " << sensor.name << ", 自由度: " << num_dof
                << std::endl;

        // 加载 ZERO 状态目标关节位置
        auto zero_pos_opt = yaml_file.Read<std::vector<double>>("behavior_manager.zero_pos");
        std::vector<double> zero_pos = zero_pos_opt.value_or(std::vector<double>(num_dof, 0.0));
        if (!zero_pos_opt) {
            std::cout << "[BehaviorManager] 警告: 未配置 zero_pos，使用零位" << std::endl;
        }

        // 加载 PD 参数
        auto load_array = [&](const std::string &key) {
            return yaml_file.Read<std::vector<double>>("behavior_manager." + key)
                .value_or(std::vector<double>{});
        };

        output.kp = load_array("kp");
        if (output.kp.empty()) {
            output.kp.assign(num_dof, 100.0);
            std::cout << "[BehaviorManager] 警告: 未配置 kp，使用默认值 100.0" << std::endl;
        }
        output.kd = load_array("kd");
        if (output.kd.empty()) {
            output.kd.assign(num_dof, 2.0);
            std::cout << "[BehaviorManager] 警告: 未配置 kd，使用默认值 2.0" << std::endl;
        }

        init_kp = output.kp;
        init_kd = output.kd;

        // 加载回零时间
        double zero_duration =
            yaml_file.Read<double>("behavior_manager.zero_duration").value_or(2.0);

        // 初始化输出维度
        output.target_pos.assign(num_dof, 0.0);
        output.target_vel.assign(num_dof, 0.0);

        // 加载 DAMP 状态阻尼 kd
        std::vector<double> damp_kd = load_array("damp_kd");
        if (damp_kd.empty()) {
            throw std::runtime_error("[BehaviorManager] 缺少配置项 behavior_manager.damp_kd");
        }

        // 注册状态
        fsm.AddState(StateName::POWER_OFF, CreateStatePowerOff());
        fsm.AddState(StateName::DAMP, CreateStateDamp(damp_kd));
        fsm.AddState(StateName::ZERO, CreateStateZero(zero_pos, zero_duration, init_kp, init_kd));

        // RL 状态配置：调用 policy_executor 提供的统一解析接口
        auto rl_type_opt = yaml_file.Read<std::string>("rl_policy.type");
        if (rl_type_opt) {
            const std::string rl_type = rl_type_opt.value();

            // 当前仅支持 onnx_infer 后端，未来可扩展 torch 等
            if (rl_type != "onnx_infer") {
                throw std::runtime_error("[BehaviorManager] 不支持的 rl_policy type: " + rl_type);
            }

            // 记录初始策略名（pending_policy 与 active_policy 保持一致，避免启动时触发切换）
            active_policy =
                yaml_file.Read<std::string>("rl_policy.onnx_infer.default_policy").value_or("");
            pending_policy = active_policy;

            const rl_policy::LoadedPolicyConfig loaded_cfg =
                rl_policy::LoadPolicyConfigFromYaml(path, active_policy, robot_dir);
            RLConfig rc = ToRLConfig(loaded_cfg);
            rc.infer_thread_cfg = infer_thread_cfg;
            rc.kp = init_kp;
            rc.kd = init_kd;
            rc.rl_freq_hz = &rl_freq_hz;

            fsm.AddState(StateName::RL, CreateStateRl(rc));
            has_rl = true;

            std::cout << "[BehaviorManager] RL 状态: 已加载 (" << rc.model_path << ")" << std::endl;
        } else {
            std::cout << "[BehaviorManager] RL 状态: 未配置" << std::endl;
        }

        fsm.AddState(StateName::SAFETY, CreateStateSafety());

        // 设置共享数据指针
        fsm.SetDataPointers(&sensor, &command, &output);
    }
};

BehaviorManagerClass::BehaviorManagerClass(const std::string &config_path)
    : impl_(std::make_unique<Impl>()) {
    impl_->LoadConfig(config_path);
}

BehaviorManagerClass::~BehaviorManagerClass() = default;

void BehaviorManagerClass::Init() {
    impl_->fsm.Init();
    impl_->initialized = true;
    std::cout << "[BehaviorManager] 初始化完成" << std::endl;
}

void BehaviorManagerClass::Step(float control_dt, float rl_dt) {
    if (!impl_->initialized)
        return;

    // 若 pending_policy 与 active_policy 不同，且当前不在 RL 状态，重新加载 RL 状态
    if (impl_->has_rl && impl_->pending_policy != impl_->active_policy &&
        impl_->fsm.CurrentState() != StateName::RL) {
        try {
            const rl_policy::LoadedPolicyConfig loaded_cfg = rl_policy::LoadPolicyConfigFromYaml(
                impl_->config_path, impl_->pending_policy, impl_->robot_dir);
            RLConfig rc = ToRLConfig(loaded_cfg);
            rc.infer_thread_cfg = impl_->infer_thread_cfg;
            rc.kp = impl_->init_kp;
            rc.kd = impl_->init_kd;
            rc.rl_freq_hz = &impl_->rl_freq_hz;
            impl_->fsm.AddState(StateName::RL, CreateStateRl(rc));
            // 重新注入数据指针（动态添加的状态需要手动注入）
            impl_->fsm.SetDataPointers(&impl_->sensor, &impl_->command, &impl_->output);
            impl_->active_policy = impl_->pending_policy;
            std::cout << "[BehaviorManager] 策略已切换: " << impl_->active_policy << " ("
                    << rc.model_path << ")" << std::endl;
        } catch (const std::exception &e) {
            std::cerr << "[BehaviorManager] 策略切换失败: " << e.what() << std::endl;
            impl_->pending_policy = impl_->active_policy;  // 回滚
        }
    }

    impl_->fsm.Step(control_dt, rl_dt);
}

void BehaviorManagerClass::SetSensorData(const robot_base::RobotData &data) {
    impl_->sensor = data;
}

void BehaviorManagerClass::SetCommand(const robot_base::Command &cmd) {
    impl_->command = cmd;
    // 只有非 RL 状态才允许切换策略，RL 运行中直接丢弃
    if (!cmd.switch_policy.empty()) {
        StateName cur = impl_->fsm.CurrentState();
        if (cur != StateName::RL) {
            impl_->pending_policy = cmd.switch_policy;
        }
    }
}

const ControlOutput &BehaviorManagerClass::GetOutput() const {
    return impl_->output;
}

StateName BehaviorManagerClass::CurrentState() const {
    return impl_->fsm.CurrentState();
}

std::string BehaviorManagerClass::CurrentPolicyName() const {
    return impl_->active_policy;
}

double BehaviorManagerClass::GetRlFreq() const {
    return impl_->rl_freq_hz.load(std::memory_order_relaxed);
}

bool BehaviorManagerClass::IsRunning() const {
    return impl_->initialized;
}

}  // namespace behavior_manager
