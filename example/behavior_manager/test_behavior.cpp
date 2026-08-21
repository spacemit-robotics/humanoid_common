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
 * - 状态查询（CurrentState / IsZeroReady / CurrentPolicyName / GetRlFreq / IsRunning）
 * - 控制输出获取（GetOutput）
 * - FSM 状态切换流程（不含 RL）：POWER_OFF → DAMP → HOME → ZERO → POWER_OFF
 *
 * HOME 使用 robot_base.default_joint_pos；当 YAML 不含 rl_policy 节点时，ZERO 使用
 * behavior_manager.zero_pos。ZERO 增益独立配置，未配置时回退到 HOME 增益。
 */

#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

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
        const auto yaml = robot_base::YamlFile::Load(config_path);
        // 创建并初始化
        BehaviorManagerClass bm(config_path);
        bm.Init();

        const float control_dt = static_cast<float>(
            yaml.Read<double>("behavior_manager.control_dt").value_or(0.002));
        const float rl_dt = static_cast<float>(
            yaml.Read<double>("rl_policy.rl_dt").value_or(0.02));
        const double home_duration =
            yaml.Read<double>("behavior_manager.home.gain_ramp_duration").value_or(1.0) +
            yaml.Read<double>("behavior_manager.home.move_duration").value_or(5.0);
        const double zero_duration = yaml.Read<double>("behavior_manager.zero.move_duration")
            .value_or(yaml.Read<double>("behavior_manager.zero_duration").value_or(2.0));
        const auto default_joint_pos =
            yaml.Read<std::vector<double>>("robot_base.default_joint_pos").value();
        const auto robot_kp = yaml.Read<std::vector<double>>("robot_base.kp").value();
        const auto robot_kd = yaml.Read<std::vector<double>>("robot_base.kd").value();
        const auto home_kp =
            yaml.Read<std::vector<double>>("behavior_manager.home.kp").value_or(robot_kp);
        const auto home_kd =
            yaml.Read<std::vector<double>>("behavior_manager.home.kd").value_or(robot_kd);
        const auto zero_kp =
            yaml.Read<std::vector<double>>("behavior_manager.zero.kp").value_or(home_kp);
        const auto zero_kd =
            yaml.Read<std::vector<double>>("behavior_manager.zero.kd").value_or(home_kd);

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
        auto run_steps = [&](int count) {
            for (int i = 0; i < count; ++i) {
                sensor.time += control_dt;
                bm.SetSensorData(sensor);
                bm.SetCommand(cmd);
                bm.Step(control_dt, rl_dt);
            }
        };

        // ========== 阶段1: POWER_OFF ==========
        std::cout << "\n--- 阶段1: POWER_OFF ---" << std::endl;
        run_steps(10);
        std::cout << "当前状态: " << StateNameStr(bm.CurrentState()) << std::endl;
        const auto &out1 = bm.GetOutput();
        std::cout << "使能: " << (out1.enable ? "是" : "否") << std::endl;
        assert(bm.CurrentState() == behavior_manager::StateName::POWER_OFF);
        assert(!out1.enable);

        // ========== 阶段2: 切换到 DAMP ==========
        std::cout << "\n--- 阶段2: key=1 → DAMP ---" << std::endl;
        cmd.key = 1;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        cmd.key = 0;
        std::cout << "当前状态: " << StateNameStr(bm.CurrentState()) << std::endl;
        const auto &out2 = bm.GetOutput();
        std::cout << "使能: " << (out2.enable ? "是" : "否") << std::endl;
        assert(bm.CurrentState() == behavior_manager::StateName::DAMP);
        assert(out2.enable);

        run_steps(10);

        // ZERO 不能绕过 HOME 直接进入。
        cmd.key = 2;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentState() == behavior_manager::StateName::DAMP);
        cmd.key = 0;

        // ========== 阶段3: 切换到 HOME ==========
        std::cout << "\n--- 阶段3: key=4 → HOME (机型默认姿态) ---" << std::endl;
        cmd.key = 4;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        cmd.key = 0;
        assert(bm.CurrentState() == behavior_manager::StateName::HOME);

        run_steps(static_cast<int>(std::ceil(home_duration / control_dt)) + 2);
        const auto &home_output = bm.GetOutput();
        assert(home_output.enable);
        assert(home_output.kp.size() == static_cast<size_t>(sensor.num_dof));
        assert(home_output.kd.size() == static_cast<size_t>(sensor.num_dof));
        assert(home_output.target_pos.size() == default_joint_pos.size());
        for (size_t i = 0; i < default_joint_pos.size(); ++i) {
            assert(std::abs(home_output.target_pos[i] - default_joint_pos[i]) < 1.0e-9);
            assert(std::abs(home_output.kp[i] - home_kp[i]) < 1.0e-9);
            assert(std::abs(home_output.kd[i] - home_kd[i]) < 1.0e-9);
        }

        // ========== 阶段4: 切换到 ZERO ==========
        std::cout << "\n--- 阶段4: key=2 → ZERO (策略准备位) ---" << std::endl;
        cmd.key = 2;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        cmd.key = 0;
        std::cout << "当前状态: " << StateNameStr(bm.CurrentState()) << std::endl;
        assert(bm.CurrentState() == behavior_manager::StateName::ZERO);
        if (bm.IsZeroReady()) {
            std::cerr << "[错误] ZERO 刚进入时不应报告 READY" << std::endl;
            return 1;
        }

        run_steps(static_cast<int>(std::ceil(zero_duration / control_dt)) + 2);
        if (!bm.IsZeroReady()) {
            std::cerr << "[错误] ZERO 完成后未报告 READY" << std::endl;
            return 1;
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
        std::cout << "kp.size()=" << out3.kp.size()
            << ", kd.size()=" << out3.kd.size() << std::endl;
        assert(out3.kp.size() == static_cast<size_t>(sensor.num_dof));
        assert(out3.kd.size() == static_cast<size_t>(sensor.num_dof));
        for (size_t i = 0; i < zero_kp.size(); ++i) {
            assert(std::abs(out3.kp[i] - zero_kp[i]) < 1.0e-9);
            assert(std::abs(out3.kd[i] - zero_kd[i]) < 1.0e-9);
        }

        // ========== 阶段5: 回到 POWER_OFF ==========
        std::cout << "\n--- 阶段5: key=-1 → POWER_OFF ---" << std::endl;
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
