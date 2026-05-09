/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_behavior.cpp
 * @brief BehaviorManagerClass 接口使用示例 && 完整测试
 *
 * 演示 BehaviorManagerClass 所有对外接口的使用方法：
 * - 构造与初始化（BehaviorManagerClass / Init）
 * - 数据输入（SetSensorData / SetCommand）
 * - 状态机驱动（Step）
 * - 状态查询（CurrentState / CurrentPolicyName / GetRlFreq / IsRunning）
 * - 控制输出获取（GetOutput）
 * - FSM 状态切换流程（不含 RL）：POWER_OFF → DAMP → ZERO → POWER_OFF
 *
 * 本 demo 使用的 yaml 不含 rl_policy 节点，ZERO 状态用 behavior_manager.zero_pos
 * 作目标位置，kp/kd 为空（由下游 robot_base/MuJoCo 的默认增益兜底）。完整 FSM
 * （含 RL）的端到端测试在机型仓库 humanoid_unitree_g1 等的启动脚本中验证。
 */

#include <chrono>
#include <iostream>
#include <thread>
#include <string>

#include "behavior_manager.h"
#include "robot_base.h"
using behavior_manager::BehaviorManagerClass;
using behavior_manager::StateNameStr;
using robot_base::RobotData;
using robot_base::Command;

void PrintHelp() {
    std::cout << "behavior_manager 模块测试\n\n";
    std::cout << "用法: test_behavior <yaml配置文件路径>\n\n";
    std::cout << "示例:\n";
    std::cout << "  ./test_behavior ../../application/config/g1.yaml\n";
    std::cout << "  ./test_behavior /absolute/path/to/g1.yaml\n";
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "[错误] 请指定 YAML 配置文件路径\n\n";
        PrintHelp();
        return 1;
    }

    std::string config_path = argv[1];

    std::cout << "========================================" << std::endl;
    std::cout << "  behavior_manager 测试" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "配置文件: " << config_path << std::endl;

    try {
        // 创建并初始化
        BehaviorManagerClass bm(config_path);
        bm.Init();

        float control_dt = 0.002f;  // 500Hz 控制频率
        float rl_dt = 0.02f;        // 50Hz RL 推理频率

        // ========== 接口查询 ==========
        std::cout << "\n--- 接口查询 ---" << std::endl;
        std::cout << "IsRunning: " << (bm.IsRunning() ? "是" : "否") << std::endl;
        std::cout << "CurrentState: " << StateNameStr(bm.CurrentState()) << std::endl;
        std::cout << "CurrentPolicyName: " << bm.CurrentPolicyName() << std::endl;
        std::cout << "GetRlFreq: " << bm.GetRlFreq() << " Hz" << std::endl;

        // 从 YAML 初始化 RobotBase（获取 num_dof 并初始化关节向量）
        RobotData sensor = RobotData::FromYaml(config_path);
        // 初始化数据
        sensor.rpy = {0, 0, 0};
        sensor.gyro = {0, 0, 0};
        // joint_pos 和 joint_vel 已在 FromYaml() 中初始化大小

        Command cmd;

        // ========== 阶段1: POWER_OFF ==========
        std::cout << "\n--- 阶段1: POWER_OFF (2秒) ---" << std::endl;
        for (int i = 0; i < 1000; i++) {
            sensor.time = i * control_dt;
            bm.SetSensorData(sensor);
            bm.SetCommand(cmd);
            bm.Step(control_dt, rl_dt);
        }
        std::cout << "当前状态: " << StateNameStr(bm.CurrentState()) << std::endl;
        const auto &out1 = bm.GetOutput();
        std::cout << "使能: " << (out1.enable ? "是" : "否") << std::endl;

        // ========== 阶段2: 切换到 DAMP ==========
        std::cout << "\n--- 阶段2: key=1 → DAMP ---" << std::endl;
        cmd.key = 1;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        cmd.key = 0;
        std::cout << "当前状态: " << StateNameStr(bm.CurrentState()) << std::endl;
        const auto &out2 = bm.GetOutput();
        std::cout << "使能: " << (out2.enable ? "是" : "否") << std::endl;

        // 运行一段时间
        for (int i = 0; i < 500; i++) {
            bm.SetSensorData(sensor);
            bm.SetCommand(cmd);
            bm.Step(control_dt, rl_dt);
        }

        // ========== 阶段3: 切换到 ZERO ==========
        std::cout << "\n--- 阶段3: key=2 → ZERO (回零位) ---" << std::endl;
        cmd.key = 2;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        cmd.key = 0;
        std::cout << "当前状态: " << StateNameStr(bm.CurrentState()) << std::endl;

        // 运行回零过程（2秒）
        for (int i = 0; i < 1000; i++) {
            sensor.time = i * control_dt;
            bm.SetSensorData(sensor);
            bm.SetCommand(cmd);
            bm.Step(control_dt, rl_dt);
        }

        // 打印回零后的目标位置
        const auto &out3 = bm.GetOutput();
        std::cout << "回零完成，目标位置: [";
        for (size_t i = 0; i < out3.target_pos.size() && i < 6; i++) {
            std::cout << out3.target_pos[i];
            if (i < 5)
                std::cout << ", ";
        }
        if (out3.target_pos.size() > 6)
            std::cout << ", ...";
        std::cout << "]" << std::endl;
        // 无 rl_policy 时 kp/kd 为空，由下游 robot_base/MuJoCo 默认增益兜底
        std::cout << "kp.size()=" << out3.kp.size() << ", kd.size()=" << out3.kd.size()
            << "（无 RL 策略，由下游兜底）" << std::endl;

        // ========== 阶段4: 回到 POWER_OFF ==========
        std::cout << "\n--- 阶段4: key=-1 → POWER_OFF ---" << std::endl;
        cmd.key = -1;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        cmd.key = 0;
        std::cout << "当前状态: " << StateNameStr(bm.CurrentState()) << std::endl;

        std::cout << "\n========================================" << std::endl;
        std::cout << "  测试完成 ✓" << std::endl;
        std::cout << "========================================" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[错误] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
