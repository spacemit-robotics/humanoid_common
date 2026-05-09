/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_transport_executor.cpp
 * @brief TransportBase 接口使用示例 && 完整测试
 *
 * 演示 transport_executor 所有对外接口的使用方法：
 * - 工厂创建（Create）
 * - 三种角色初始化（Init / Role::DRIVER / Role::CONTROL / Role::HMI）
 * - 状态通道收发（SendState / RecvState）
 * - 控制通道收发（SendControl / RecvControl）
 * - 命令通道收发（SendCommand / RecvCommand）
 *
 * 传输后端由 YAML 中 transport.type 字段决定（udp / shm），
 * 切换后端只需传入不同的配置文件，测试代码无需修改。
 *
 * 用法: ./test_transport_executor <yaml_path>
 * 示例:
 *   ./test_transport_executor ../../../application/config/g1.yaml        # UDP
 *   ./test_transport_executor ../example/config_example.yaml             # 按 type 字段决定
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <string>

#include "transport_executor.h"
#include "robot_base.h"
int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <yaml_path>\n"
                << "示例: ./test_transport_executor ../../../application/config/g1.yaml\n";
        return 1;
    }
    const std::string yaml_path = argv[1];

    // ==================== 创建 Driver 端 ====================

    auto driver = transport::Create(yaml_path);
    if (!driver->Init(yaml_path, transport::Role::DRIVER)) {
        std::cerr << "[test] Driver 初始化失败\n";
        return 1;
    }
    std::cout << "[test] Driver 初始化成功\n";

    // ==================== 创建 Control 端 ====================

    auto control = transport::Create(yaml_path);
    if (!control->Init(yaml_path, transport::Role::CONTROL)) {
        std::cerr << "[test] Control 初始化失败\n";
        return 1;
    }
    std::cout << "[test] Control 初始化成功\n";

    // ==================== 测试状态通道 ====================

    std::cout << "\n--- 测试状态通道 (Driver → Control) ---\n";

    robot_base::RobotData state = robot_base::RobotData::FromYaml(yaml_path);
    state.time = 1.234;
    state.rpy = {0.1, 0.2, 0.3};
    state.gyro = {0.01, 0.02, 0.03};
    for (int i = 0; i < state.num_dof; ++i) {
        state.joint_pos[i] = 0.1 * i;
        state.joint_vel[i] = 0.01 * i;
    }

    driver->SendState(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    robot_base::RobotData recv_state;
    if (control->RecvState(recv_state)) {
        std::cout << "[test] 状态接收成功: num_dof=" << recv_state.num_dof
                << ", time=" << recv_state.time << ", rpy=(" << recv_state.rpy[0] << ","
                << recv_state.rpy[1] << "," << recv_state.rpy[2] << ")\n";
        bool ok = (recv_state.num_dof == state.num_dof) &&
                (std::abs(recv_state.time - state.time) < 1e-6);
        std::cout << "[test] 状态数据验证: " << (ok ? "通过" : "失败") << "\n";
    } else {
        std::cerr << "[test] 状态接收失败\n";
    }

    // ==================== 测试控制通道 ====================

    std::cout << "\n--- 测试控制通道 (Control → Driver) ---\n";

    // 真实 RL 控制下发包含完整四个字段：target_pos / target_vel / kp / kd
    // 每帧 kp/kd 也会随控制命令下发（per-policy 训练增益），demo 这里给出示例值
    robot_base::ControlCmd cmd;
    cmd.enable = true;
    cmd.target_pos.resize(state.num_dof);
    cmd.target_vel.assign(state.num_dof, 0.0);
    cmd.kp.assign(state.num_dof, 100.0);
    cmd.kd.assign(state.num_dof, 2.0);
    for (int i = 0; i < state.num_dof; ++i) {
        cmd.target_pos[i] = 0.5 * i;
    }

    control->SendControl(cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    robot_base::ControlCmd recv_cmd;
    if (driver->RecvControl(recv_cmd)) {
        std::cout << "[test] 控制命令接收成功: enable=" << recv_cmd.enable
                << ", num_dof=" << recv_cmd.target_pos.size()
                << ", kp[0]=" << recv_cmd.kp[0] << ", kd[0]=" << recv_cmd.kd[0] << "\n";
        bool ok = recv_cmd.enable
                && (recv_cmd.target_pos.size() == cmd.target_pos.size())
                && (recv_cmd.target_vel.size() == cmd.target_vel.size())
                && (recv_cmd.kp.size() == cmd.kp.size())
                && (recv_cmd.kd.size() == cmd.kd.size());
        std::cout << "[test] 控制数据验证: " << (ok ? "通过" : "失败") << "\n";
    } else {
        std::cerr << "[test] 控制命令接收失败\n";
    }

    // ==================== 测试命令通道 ====================

    std::cout << "\n--- 测试命令通道 (Hmi → Control) ---\n";

    auto hmi = transport::Create(yaml_path);
    if (!hmi->Init(yaml_path, transport::Role::HMI)) {
        std::cerr << "[test] Hmi 初始化失败\n";
        return 1;
    }

    robot_base::Command hmi_cmd;
    hmi_cmd.key = 3;
    hmi_cmd.vx = 0.5f;
    hmi_cmd.vy = 0.1f;
    hmi_cmd.wz = 0.2f;

    hmi->SendCommand(hmi_cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    robot_base::Command recv_hmi;
    if (control->RecvCommand(recv_hmi)) {
        std::cout << "[test] 命令接收成功: key=" << recv_hmi.key << ", vx=" << recv_hmi.vx
                << ", vy=" << recv_hmi.vy << ", wz=" << recv_hmi.wz << "\n";
        bool ok = (recv_hmi.key == 3) && (std::abs(recv_hmi.vx - 0.5f) < 1e-4f);
        std::cout << "[test] 命令数据验证: " << (ok ? "通过" : "失败") << "\n";
    } else {
        std::cerr << "[test] 命令接收失败\n";
    }

    std::cout << "\n[test] 全部测试完成\n";
    return 0;
}
