/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file control_sim2sim_demo.cpp
 * @brief 简化 sim2sim 控制演示程序（不经过 behavior FSM）
 *
 * 该程序是 control_demo 的精简版本：
 * - 控制周期使用 YAML 中 rl_policy.rl_dt
 * - 不使用 behavior_manager FSM
 * - 不接收 HMI 命令，仅使用 YAML 中的固定速度命令
 * - 只负责：接收 Driver 状态 → 调用 RL 策略 → 发送控制命令
 *
 * 调用的模块：
 * - rl_service: RL 策略推理
 * - transport_executor: 统一传输接口（含 RobotBase 等类型）
 */

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>
#include <string>
#include <vector>

#include "transport_executor.h"
#include "rl_service.h"
#include "robot_base.h"
using rl_policy::LoadedPolicyConfig;
using rl_policy::LoadPolicyConfigFromYaml;
using rl_policy::PolicyExecutor;

namespace {
volatile std::sig_atomic_t g_running = 1;
void OnSignal(int) {
    g_running = 0;
}
}  // namespace

int main(int argc, char *argv[]) {
    std::signal(SIGINT, OnSignal);

    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " ../config/g1.yaml\n";
        return 1;
    }
    const std::string yaml_path = argv[1];

    // 从 YAML 读取 RL 推理周期和控制参数
    robot_base::YamlFile yaml_file = robot_base::YamlFile::Load(yaml_path);
    float rl_dt = static_cast<float>(yaml_file.Read<double>("rl_policy.rl_dt").value_or(0.02));

    // 读取 kp/kd 配置（用于发送到 driver）
    std::vector<double> default_kp =
        yaml_file.Read<std::vector<double>>("simulation.mujoco.controller.kp")
            .value_or(std::vector<double>{});
    std::vector<double> default_kd =
        yaml_file.Read<std::vector<double>>("simulation.mujoco.controller.kd")
            .value_or(std::vector<double>{});

    // 解析 robot_dir
    const std::string robot_dir =
        yaml_file.ToAbsPath(yaml_file.Read<std::string>("robot_base.robot_dir").value());

    // 初始化 RL 策略
    const std::string policy_name =
        yaml_file.Read<std::string>("rl_policy.onnx_infer.default_policy").value();
    const LoadedPolicyConfig loaded_cfg =
        LoadPolicyConfigFromYaml(yaml_path, policy_name, robot_dir);
    const double cmd_vx = loaded_cfg.command_init[0];
    const double cmd_vy = loaded_cfg.command_init[1];
    const double cmd_wz = loaded_cfg.command_init[2];

    PolicyExecutor policy;
    policy.Init(loaded_cfg.exec_cfg);

    // 初始化传输（Control 角色）
    auto transport = transport::Create(yaml_path);
    if (!transport->Init(yaml_path, transport::Role::CONTROL)) {
        std::cerr << "[control_sim2sim_demo] 传输初始化失败\n";
        return 1;
    }

    robot_base::RobotData latest_state;
    bool has_state = false;

    int infer_count = 0;
    auto last_print_time = std::chrono::steady_clock::now();
    auto last_infer_time = std::chrono::steady_clock::now();

    std::vector<double> action(policy.ActionDim(), 0.0);
    std::vector<double> target_pos;
    Eigen::VectorXf obs;

    std::cout << "[control_sim2sim_demo] 启动（直接 RL，无 FSM，使用 transport_executor）\n"
            << "  RL 推理频率: " << (1.0 / rl_dt) << " Hz"
            << " (rl_dt=" << rl_dt << "s)\n"
            << "  init_cmd(vx,vy,wz)=(" << cmd_vx << "," << cmd_vy << "," << cmd_wz << ")\n";

    while (g_running) {
        // 1) 收 Driver 状态（非阻塞）
        if (transport->RecvState(latest_state)) {
            has_state = true;
        }

        // 2) 基于时间触发 RL 推理周期
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_infer_time).count();
        if (elapsed >= rl_dt && has_state) {
            last_infer_time = now;

            policy.AssembleObs(
                latest_state.gyro,
                latest_state.rpy,
                cmd_vx,
                cmd_vy,
                cmd_wz,
                latest_state.joint_pos,
                latest_state.joint_vel,
                latest_state.base_quat,
                {{latest_state.base_vel[0], latest_state.base_vel[1], latest_state.base_vel[2]}},
                rl_dt,
                obs);
            policy.Infer(obs, action);
            infer_count++;

            policy.MapActionToTargetPos(action, target_pos);

            robot_base::ControlCmd ctrl;
            ctrl.enable = true;
            ctrl.target_pos = target_pos;
            ctrl.target_vel.assign(policy.ActionDim(), 0.0);  // 位置控制，目标速度为 0
            ctrl.kp = default_kp;
            ctrl.kd = default_kd;
            transport->SendControl(ctrl);
        }

        // 打印推理频率（每秒一次）
        double elapsed_print = std::chrono::duration<double>(now - last_print_time).count();
        if (elapsed_print >= 1.0) {
            double infer_freq = infer_count / elapsed_print;
            std::cout << "[control_sim2sim_demo] 推理频率: " << std::fixed << std::setprecision(2)
                    << infer_freq << " Hz" << std::endl;
            infer_count = 0;
            last_print_time = now;
        }

        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    std::cout << "[control_sim2sim_demo] 退出\n";
    return 0;
}
