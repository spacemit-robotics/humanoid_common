/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file control_demo.cpp
 * @brief 主要控制演示程序
 *
 * 该程序是应用层的核心控制循环，集成 behavior_manager 模块实现机器人行为状态机管理。
 * 它接收来自 HMI 的键盘命令和来自 driver 的机器人状态，通过 behavior_manager FSM
 * 执行状态转换和控制逻辑，最后将控制命令发送给 driver。
 *
 * 调用的模块：
 * - behavior_manager: 行为状态机管理（POWER_OFF → DAMP → ZERO → RL）
 * - transport_executor: 统一传输接口
 * - robot_base: 机器人状态数据结构
 */

#include <algorithm>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>
#include <string>

#include "transport_executor.h"
#include "behavior_manager.h"
#include "robot_base.h"
using behavior_manager::BehaviorManagerClass;
using behavior_manager::StateNameStr;

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
    std::string yaml_path = argv[1];

    // 从配置读取控制频率参数
    robot_base::YamlFile yaml_file = robot_base::YamlFile::Load(yaml_path);

    // 配置主控制循环线程调度（robot_base.threads.control_main）
    robot_base::ThreadLoop::FromYaml(yaml_file, "control_main").Apply();

    // behavior_manager.control_dt: 控制循环周期（未来异步推理时可独立配置为高频）
    float control_dt =
        static_cast<float>(yaml_file.Read<double>("behavior_manager.control_dt").value_or(0.02));

    // rl_policy.rl_dt: RL 推理周期（对应训练时的推理频率）
    float rl_dt = static_cast<float>(yaml_file.Read<double>("rl_policy.rl_dt").value_or(0.02));

    // 初始化行为管理器
    BehaviorManagerClass bm(yaml_path);
    bm.Init();

    // 初始化传输（Control 角色）
    auto transport = transport::Create(yaml_path);
    if (!transport->Init(yaml_path, transport::Role::CONTROL)) {
        std::cerr << "[control_demo] 传输初始化失败\n";
        return 1;
    }

    robot_base::Command cmd;
    robot_base::RobotData latest_state;
    bool has_state = false;

    auto last_control_time = std::chrono::steady_clock::now();
    int step_count = 0;
    auto last_print_time = std::chrono::steady_clock::now();
    const double target_control_hz = (control_dt > 1e-6f) ? (1.0 / control_dt) : 0.0;
    const double target_rl_hz = (rl_dt > 1e-6f) ? (1.0 / rl_dt) : 0.0;

    std::cout << "[control_demo] 启动（使用 transport_executor）\n";
    std::cout << "\n\n\n";

    while (g_running) {
        // 1) drain 命令（读空 buffer，取最新）
        {
            robot_base::Command hmi;
            while (transport->RecvCommand(hmi)) {
                cmd = hmi;
            }
        }

        // 2) drain Driver 状态（读空 buffer，取最新）
        {
            robot_base::RobotData state;
            while (transport->RecvState(state)) {
                latest_state = state;
                has_state = true;
            }
        }

        // 3) 基于时间触发控制周期
        auto now = std::chrono::steady_clock::now();
        double elapsed = std::chrono::duration<double>(now - last_control_time).count();
        if (elapsed >= control_dt && has_state) {
            last_control_time = now;

            bm.SetSensorData(latest_state);
            bm.SetCommand(cmd);
            bm.Step(control_dt, rl_dt);
            const auto &out = bm.GetOutput();
            robot_base::ControlCmd ctrl;
            ctrl.enable = out.enable;
            ctrl.target_pos = out.target_pos;
            ctrl.target_vel = out.target_vel;
            ctrl.kp = out.kp;
            ctrl.kd = out.kd;
            // 透传 FSM 状态作为通用控制模式（driver 侧自主解释；mujoco 据此调整悬挂、实机据此判定阈值/恢复）
            ctrl.mode = static_cast<robot_base::ControlMode>(bm.CurrentState());
            transport->SendControl(ctrl);

            step_count++;
        }

        // 固定 3 行动态刷新，避免增量日志刷屏
        double elapsed_print = std::chrono::duration<double>(now - last_print_time).count();
        if (elapsed_print >= 0.1) {
            const double control_freq = step_count / elapsed_print;
            const double rl_freq = bm.GetRlFreq();
            const std::string policy =
                bm.CurrentPolicyName().empty() ? "-" : bm.CurrentPolicyName();

            std::ostringstream line0, line1, line2;
            line0 << "[control] state=" << StateNameStr(bm.CurrentState()) << " policy=" << policy;
            line1 << std::fixed << std::setprecision(4) << "control_dt=" << control_dt
                << "s target=" << std::setprecision(1) << target_control_hz
                << "Hz actual=" << std::setprecision(2) << control_freq << "Hz";
            line2 << std::fixed << std::setprecision(4) << "rl_dt=" << rl_dt
                << "s target=" << std::setprecision(1) << target_rl_hz
                << "Hz actual=" << std::setprecision(2) << rl_freq << "Hz";

            std::cout << "\033[3A"
                    << "\r\033[2K" << line0.str() << "\n"
                    << "\r\033[2K" << line1.str() << "\n"
                    << "\r\033[2K" << line2.str() << "\n"
                    << std::flush;
            step_count = 0;
            last_print_time = now;
        }

        // 4) 小睡，避免空转
        std::this_thread::sleep_for(std::chrono::microseconds(100));
    }

    std::cout << "\n[control_demo] 退出\n";
    return 0;
}
