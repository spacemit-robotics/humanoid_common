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
#include <limits>
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
            yaml.Read<double>("behavior_manager.home.move_duration").value_or(3.0);
        const double zero_duration = yaml.Read<double>("behavior_manager.zero.move_duration")
            .value_or(yaml.Read<double>("behavior_manager.zero_duration").value_or(3.0));
        const double zero_settle_duration = yaml.Read<double>(
            "behavior_manager.zero.settle_duration").value_or(0.20);
        const double safety_release_duration = yaml.Read<double>(
            "behavior_manager.safety.release_duration_s").value_or(1.0);
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
        robot_base::FaultStatus sensor_fault;
        const auto assert_finite_zero = [](const std::vector<double> &values) {
            for (double value : values) {
                assert(std::isfinite(value));
                assert(value == 0.0);
            }
        };
        auto run_steps = [&](int count, bool follow_target) {
            for (int i = 0; i < count; ++i) {
                const auto &current_output = bm.GetOutput();
                if (follow_target &&
                    current_output.target_pos.size() == sensor.joint_pos.size()) {
                    sensor.joint_pos = current_output.target_pos;
                    sensor.joint_vel = current_output.target_vel;
                }
                sensor.time += control_dt;
                bm.SetSensorData(sensor);
                bm.SetSensorFault(sensor_fault);
                bm.SetCommand(cmd);
                bm.Step(control_dt, rl_dt);
            }
        };

        // ========== 阶段1: POWER_OFF ==========
        std::cout << "\n--- 阶段1: POWER_OFF ---" << std::endl;
        run_steps(10, true);
        std::cout << "当前状态: " << StateNameStr(bm.CurrentState()) << std::endl;
        const auto &out1 = bm.GetOutput();
        std::cout << "使能: " << (out1.enable ? "是" : "否") << std::endl;
        assert(bm.CurrentState() == behavior_manager::StateName::POWER_OFF);
        assert(!out1.enable);
        assert_finite_zero(out1.target_vel);
        assert_finite_zero(out1.target_torque);
        assert_finite_zero(out1.kp);
        assert_finite_zero(out1.kd);

        // ========== 阶段2: 切换到 DAMP ==========
        std::cout << "\n--- 阶段2: key=1 → DAMP ---" << std::endl;
        cmd.key = 1;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        cmd.key = 0;
        std::cout << "当前状态: " << StateNameStr(bm.CurrentState()) << std::endl;
        const auto &power_off_output = bm.GetOutput();
        assert(!power_off_output.enable);
        assert_finite_zero(power_off_output.target_vel);
        assert_finite_zero(power_off_output.target_torque);
        assert_finite_zero(power_off_output.kp);
        assert_finite_zero(power_off_output.kd);
        const auto &out2 = bm.GetOutput();
        std::cout << "使能: " << (out2.enable ? "是" : "否") << std::endl;
        assert(bm.CurrentState() == behavior_manager::StateName::DAMP);
        assert(out2.enable);

        run_steps(10, true);

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

        // HOME 按配置时长完成，不依赖关节到位阈值。
        run_steps(static_cast<int>(std::ceil(home_duration / control_dt)) + 2,
                    true);
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

        // ZERO 需要实际关节反馈满足到位条件。
        sensor.joint_pos[0] += 0.20;
        sensor.joint_vel.assign(sensor.joint_vel.size(), 0.0);
        run_steps(static_cast<int>(std::ceil(zero_duration / control_dt)) + 2,
                    false);
        if (bm.IsZeroReady()) {
            std::cerr << "[错误] ZERO 实际关节未到位时不应报告 READY"
                << std::endl;
            return 1;
        }
        run_steps(static_cast<int>(
            std::ceil(zero_settle_duration / control_dt)) + 2, true);
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

        // ========== 阶段6: 故障锁存、恢复与 POWER_OFF 确认 ==========
        std::cout << "\n--- 阶段6: 故障锁存与确认 ---" << std::endl;
        sensor_fault.active = true;
        sensor_fault.latched = true;
        sensor_fault.source = robot_base::FaultSource::MOTOR;
        sensor_fault.code = robot_base::FaultCode::DEVICE_ERROR;
        sensor_fault.native_code = -3;
        sensor_fault.sequence = 17;
        sensor_fault.detail = "test motor feedback fault";
        sensor.base_quat = {0.0, 0.0, 0.0, 0.0};
        bm.SetSensorData(sensor);
        bm.SetSensorFault(sensor_fault);
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentState() == behavior_manager::StateName::POWER_OFF);
        assert(bm.CurrentFault().active && bm.CurrentFault().latched);
        assert(bm.CurrentFault().source == robot_base::FaultSource::MOTOR);
        assert(bm.CurrentFault().native_code == -3);
        const uint64_t first_fault_sequence = bm.CurrentFault().sequence;
        const std::string first_fault_detail = bm.CurrentFault().detail;

        // 同周期的无效姿态属于次生故障，不能覆盖先观察到的 driver 根故障。
        // 同一 driver 故障更新现场描述时，也不能生成新故障序列。
        sensor_fault.detail = "test motor feedback fault, age=0.125 s";
        bm.SetSensorData(sensor);
        bm.SetSensorFault(sensor_fault);
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentFault().sequence == first_fault_sequence);
        assert(bm.CurrentFault().detail == first_fault_detail);
        assert(bm.CurrentFault().source == robot_base::FaultSource::MOTOR);

        // 活动故障和未确认的恢复故障都必须阻止重新上电。
        cmd.key = 1;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentState() == behavior_manager::StateName::POWER_OFF);
        cmd.key = 0;

        // 如果确认请求与故障重新活动撞在同一周期，恢复后同一序号仍必须可确认。
        bm.AcknowledgeFault(first_fault_sequence);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentFault().active && bm.CurrentFault().latched);
        sensor_fault.active = false;
        sensor.base_quat = {1.0, 0.0, 0.0, 0.0};
        bm.SetSensorData(sensor);
        bm.SetSensorFault(sensor_fault);
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        assert(!bm.CurrentFault().active && bm.CurrentFault().latched);
        bm.AcknowledgeFault(first_fault_sequence);
        bm.Step(control_dt, rl_dt);
        assert(!bm.CurrentFault().latched);
        bm.AcknowledgeFault(0);

        // 未确认的恢复故障再次活动时属于新事件，旧确认序号不得继续有效。
        sensor_fault.active = true;
        bm.SetSensorFault(sensor_fault);
        bm.Step(control_dt, rl_dt);
        const uint64_t second_fault_sequence = bm.CurrentFault().sequence;
        assert(second_fault_sequence > first_fault_sequence);
        sensor_fault.active = false;
        bm.SetSensorFault(sensor_fault);
        bm.Step(control_dt, rl_dt);
        assert(!bm.CurrentFault().active && bm.CurrentFault().latched);
        sensor_fault.active = true;
        bm.SetSensorFault(sensor_fault);
        bm.Step(control_dt, rl_dt);
        const uint64_t reactivated_fault_sequence = bm.CurrentFault().sequence;
        assert(reactivated_fault_sequence > second_fault_sequence);
        sensor_fault.active = false;
        bm.SetSensorFault(sensor_fault);
        bm.Step(control_dt, rl_dt);

        // 过期或串错的确认序列不得清除当前故障。
        bm.AcknowledgeFault(bm.CurrentFault().sequence + 1);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentFault().latched);
        bm.AcknowledgeFault(0);

        // 确认同一 driver 故障序列后，后续重复状态不得立即重新锁存。
        bm.AcknowledgeFault(bm.CurrentFault().sequence);
        bm.Step(control_dt, rl_dt);
        assert(!bm.CurrentFault().latched);
        bm.AcknowledgeFault(0);
        bm.Step(control_dt, rl_dt);
        assert(!bm.CurrentFault().latched);

        cmd.key = 1;
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        cmd.key = 0;
        assert(bm.CurrentState() == behavior_manager::StateName::DAMP);

        // 同一来源故障再次变为 active 时，必须作为新事件进入 SAFETY 并重新确认。
        sensor_fault.active = true;
        bm.SetSensorData(sensor);
        bm.SetSensorFault(sensor_fault);
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentState() == behavior_manager::StateName::SAFETY);
        assert(bm.CurrentFault().active && bm.CurrentFault().latched);
        const auto &safety_output = bm.GetOutput();
        assert(!safety_output.enable);
        assert_finite_zero(safety_output.target_vel);
        assert_finite_zero(safety_output.target_torque);
        assert_finite_zero(safety_output.kp);
        assert_finite_zero(safety_output.kd);

        sensor_fault.active = false;
        run_steps(static_cast<int>(
            std::ceil(safety_release_duration / control_dt)) + 2, true);
        assert(bm.CurrentState() == behavior_manager::StateName::POWER_OFF);
        assert(!bm.CurrentFault().active && bm.CurrentFault().latched);
        const auto &released_output = bm.GetOutput();
        assert(!released_output.enable);
        assert_finite_zero(released_output.target_vel);
        assert_finite_zero(released_output.target_torque);
        assert_finite_zero(released_output.kp);
        assert_finite_zero(released_output.kd);
        bm.AcknowledgeFault(bm.CurrentFault().sequence);
        bm.Step(control_dt, rl_dt);
        assert(!bm.CurrentFault().latched);
        bm.AcknowledgeFault(0);

        // 时间戳和姿态四元数损坏时，即使数组维度正确也必须锁存故障。
        sensor.base_quat = {0.0, 0.0, 0.0, 0.0};
        bm.SetSensorData(sensor);
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentFault().active);
        assert(bm.CurrentFault().code == robot_base::FaultCode::INVALID_DATA);

        sensor.base_quat = {1.0, 0.0, 0.0, 0.0};
        bm.SetSensorData(sensor);
        bm.Step(control_dt, rl_dt);
        bm.AcknowledgeFault(bm.CurrentFault().sequence);
        bm.Step(control_dt, rl_dt);
        assert(!bm.CurrentFault().latched);
        bm.AcknowledgeFault(0);

        // 活动状态下的无效关节反馈不得被 SAFETY 当作缓慢卸力目标继续发送。
        cmd.key = 1;
        bm.SetSensorData(sensor);
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        cmd.key = 0;
        assert(bm.CurrentState() == behavior_manager::StateName::DAMP);
        sensor.joint_pos[0] = std::numeric_limits<double>::quiet_NaN();
        bm.SetSensorData(sensor);
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentState() == behavior_manager::StateName::SAFETY);
        const auto &invalid_state_output = bm.GetOutput();
        assert(!invalid_state_output.enable);
        for (double value : invalid_state_output.target_pos) assert(std::isfinite(value));
        for (double value : invalid_state_output.kp) assert(value == 0.0);
        for (double value : invalid_state_output.kd) assert(value == 0.0);
        sensor.joint_pos[0] = 0.0;
        run_steps(2, true);
        assert(bm.CurrentState() == behavior_manager::StateName::POWER_OFF);
        bm.AcknowledgeFault(bm.CurrentFault().sequence);
        bm.Step(control_dt, rl_dt);
        assert(!bm.CurrentFault().latched);
        bm.AcknowledgeFault(0);

        sensor.time = -1.0;
        bm.SetSensorData(sensor);
        bm.SetCommand(cmd);
        bm.Step(control_dt, rl_dt);
        assert(bm.CurrentFault().active);
        assert(bm.CurrentFault().code == robot_base::FaultCode::INVALID_DATA);

        std::cout << "\n========================================" << std::endl;
        std::cout << "  测试完成 ✓" << std::endl;
        std::cout << "========================================" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[错误] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
