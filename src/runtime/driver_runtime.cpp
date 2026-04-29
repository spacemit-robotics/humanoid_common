/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file driver_demo.cpp
 * @brief MuJoCo 仿真驱动程序
 *
 * 该程序管理 MuJoCo 物理仿真环境，模拟真实机器人的运动学和动力学特性。
 * 它接收来自 control 的控制命令，应用到仿真模型，并实时反馈机器人状态。
 *
 * 主要功能：
 * - 初始化和重置 MuJoCo 仿真场景
 * - 接收控制命令并应用关节位置控制
 * - 执行仿真步进并计算机器人状态
 * - 发送状态反馈（关节位置/速度、IMU 数据等）
 *
 * 调用的模块：
 * - mujoco_sim: MuJoCo 仿真引擎封装
 * - transport_executor: 统一传输接口
 * - robot_base: 机器人状态数据结构
 */

#include <csignal>
#include <iostream>
#include <optional>
#include <string>

#include "transport_executor.h"
#include "mujoco_sim.h"
#include "robot_base.h"
using mujoco_sim::SimState;
using mujoco_sim::SimControl;
using mujoco_sim::Simulator;

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

    // 配置仿真驱动主线程调度（robot_base.threads.driver_main）
    robot_base::YamlFile yaml_file = robot_base::YamlFile::Load(yaml_path);
    robot_base::ThreadLoop::FromYaml(yaml_file, "driver_main").Apply();

    // 从 robot_base 读取机器人基础信息
    const auto robot_name = yaml_file.Read<std::string>("robot_base.name").value();
    const int num_dof = yaml_file.Read<int>("robot_base.num_dof").value();
    const std::string robot_dir =
        yaml_file.ToAbsPath(yaml_file.Read<std::string>("robot_base.robot_dir").value());
    const std::string xml_path = robot_dir + "/resources/xml/scene.xml";

    // 初始化仿真
    Simulator sim(yaml_path, robot_name, num_dof, xml_path, true);

    // 初始化传输（Driver 角色）
    auto transport = transport::Create(yaml_path);
    if (!transport->Init(yaml_path, transport::Role::DRIVER)) {
        std::cerr << "[driver_demo] 传输初始化失败\n";
        return 1;
    }

    std::cout << "[driver_demo] 启动（使用 transport_executor）\n";

    sim.Run(
        [&](const SimState &sim_state) -> std::optional<SimControl> {
            // SimState → RobotData（transport 层使用 robot_base 类型）
            robot_base::RobotData state;
            state.num_dof = sim_state.num_dof;
            state.joint_pos = sim_state.joint_pos;
            state.joint_vel = sim_state.joint_vel;
            state.base_pos = sim_state.base_pos;
            state.base_quat = sim_state.base_quat;
            state.base_vel = sim_state.base_vel;
            state.gyro = sim_state.gyro;
            state.rpy = sim_state.rpy;
            state.time = sim_state.time;
            transport->SendState(state);

            // drain 控制包，取最新（避免 FSM 高频发送导致包积压、控制命令滞后）
            robot_base::ControlCmd cmd;
            bool has_cmd = false;
            robot_base::ControlCmd tmp;
            while (transport->RecvControl(tmp)) {
                cmd = tmp;
                has_cmd = true;
            }
            if (!has_cmd) {
                return std::nullopt;
            }

            // ControlCmd → SimControl
            SimControl ctrl;
            ctrl.enable = cmd.enable;
            ctrl.target_pos = cmd.target_pos;
            ctrl.target_vel = cmd.target_vel;
            ctrl.kp = cmd.kp;
            ctrl.kd = cmd.kd;
            return ctrl;
        },
        [&]() { return g_running != 0; });

    std::cout << "[driver_demo] 退出\n";
    return 0;
}
