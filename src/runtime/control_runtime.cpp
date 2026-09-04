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
 * - behavior_manager: 行为状态机管理（POWER_OFF → DAMP → HOME → ZERO → RL）
 * - transport_executor: 统一传输接口
 * - robot_base: 机器人状态数据结构
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <thread>

#include "transport_executor.h"
#include "behavior_manager.h"
#include "policy_command_limits.h"
#include "robot_base.h"
#include "runtime_logger.h"
using behavior_manager::BehaviorManagerClass;
using behavior_manager::StateNameStr;

namespace {
volatile std::sig_atomic_t g_running = 1;
void OnSignal(int) {
    g_running = 0;
}

double WallTimeSeconds() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void RecordControlTelemetry(const robot_base::RobotData &state,
    const robot_base::ControlCmd &control, behavior_manager::StateName mode,
    const std::string &policy, bool hmi_connected, const robot_base::Command &command,
    double control_hz, double rl_hz, const std::vector<std::string> &joint_names) {
    const double wall_time = WallTimeSeconds();
    std::ostringstream summary;
    summary << std::fixed << std::setprecision(9)
            << wall_time << "," << state.time << "," << StateNameStr(mode) << ","
            << policy << "," << hmi_connected << "," << command.vx << ","
            << command.vy << "," << command.wz << "," << control_hz << ","
            << rl_hz << "," << control.enable;
    runtime_logging::RecordCsv("control_trace",
        "wall_time_s,device_time_s,state,policy,hmi_connected,vx,vy,wz,"
        "control_hz,rl_hz,enabled",
        summary.str());

    const size_t count = std::min({
        joint_names.size(), state.joint_pos.size(), state.joint_vel.size(),
        state.joint_torque.size(), control.target_pos.size(),
        control.target_vel.size(), control.target_torque.size(),
        control.kp.size(), control.kd.size()});
    std::ostringstream joints;
    joints << std::fixed << std::setprecision(9);
    for (size_t i = 0; i < count; ++i) {
        joints << wall_time << "," << state.time << "," << joint_names[i] << ","
            << state.joint_pos[i] << "," << state.joint_vel[i] << ","
            << state.joint_torque[i] << "," << control.target_pos[i] << ","
            << control.target_vel[i] << "," << control.target_torque[i] << ","
            << control.kp[i] << "," << control.kd[i];
        if (i + 1 < count) joints << "\n";
    }
    runtime_logging::RecordCsv("control_joint",
        "wall_time_s,device_time_s,joint,position,velocity,torque,target_position,"
        "target_velocity,target_torque,kp,kd",
        joints.str());
}

robot_base::ControlMode ToControlMode(behavior_manager::StateName state) {
    switch (state) {
    case behavior_manager::StateName::POWER_OFF:
        return robot_base::ControlMode::POWER_OFF;
    case behavior_manager::StateName::DAMP:
        return robot_base::ControlMode::DAMP;
    case behavior_manager::StateName::HOME:
        return robot_base::ControlMode::HOME;
    case behavior_manager::StateName::ZERO:
        return robot_base::ControlMode::ZERO;
    case behavior_manager::StateName::RL:
        return robot_base::ControlMode::RL;
    case behavior_manager::StateName::SAFETY:
        return robot_base::ControlMode::SAFETY;
    }
    return robot_base::ControlMode::SAFETY;
}
}  // namespace

int main(int argc, char *argv[]) {
    std::signal(SIGINT, OnSignal);

    if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        auto &out = (argc < 2) ? std::cerr : std::cout;
        out << "用法: " << argv[0] << " <config.yaml>\n"
            << "选项:\n"
            << "  <config.yaml>  机器人配置文件路径\n"
            << "  -h, --help     显示此帮助信息\n";
        return (argc < 2) ? 1 : 0;
    }
    std::string yaml_path = argv[1];

    // 从配置读取控制频率参数
    robot_base::YamlFile yaml_file;
    runtime_config::PolicyCommandLimitMap policy_command_limits;
    std::unique_ptr<runtime_logging::Session> logging_session;
    try {
        yaml_file = robot_base::YamlFile::Load(yaml_path);
        policy_command_limits = runtime_config::LoadPolicyCommandLimits(yaml_file);
        logging_session = std::make_unique<runtime_logging::Session>(
            yaml_file, yaml_path, "control", false);
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n"
            << "用法: " << argv[0] << " <config.yaml>\n";
        return 1;
    }

    // 配置主控制循环线程调度（robot_base.threads.control_main）
    robot_base::ThreadLoop::FromYaml(yaml_file, "control_main").Apply();

    // behavior_manager.control_dt: 控制循环周期（未来异步推理时可独立配置为高频）
    float control_dt =
        static_cast<float>(yaml_file.Read<double>("behavior_manager.control_dt").value_or(0.02));

    // rl_policy.rl_dt: RL 推理周期（对应训练时的推理频率）
    float rl_dt = static_cast<float>(yaml_file.Read<double>("rl_policy.rl_dt").value_or(0.02));

    // HMI 心跳边界；速度范围由应用层每个策略的 command.limits 提供。
    const double hmi_command_timeout = std::max(
        0.1, yaml_file.Read<double>("hmi.command_timeout").value_or(0.5));
    const double status_hz = std::max(
        1.0, yaml_file.Read<double>("hmi.status_hz").value_or(20.0));

    // 初始化行为管理器
    BehaviorManagerClass bm(yaml_path);
    bm.Init();

    // 初始化传输（Control 角色）
    auto transport = transport::Create(yaml_path);
    if (!transport->Init(yaml_path, transport::Role::CONTROL)) {
        runtime_logging::Log(
            runtime_logging::Level::kError, "control transport initialization failed", false);
        std::cerr << "[control_demo] 传输初始化失败\n";
        return 1;
    }

    robot_base::Command cmd;
    robot_base::RobotData latest_state;
    robot_base::ControlCmd latest_control;
    bool has_state = false;
    bool has_control = false;
    bool has_hmi = false;
    bool reported_state = false;
    bool previous_hmi_connected = false;
    const auto joint_names =
        yaml_file.Read<std::vector<std::string>>("robot_base.joint_names").value_or(
            std::vector<std::string>{});
    const auto logging_config = runtime_logging::GetConfig();
    const auto telemetry_period =
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(
                1.0 / logging_config.control_telemetry_rate_hz));

    auto last_control_time = std::chrono::steady_clock::now();
    auto last_hmi_time = last_control_time;
    auto last_telemetry_time = last_control_time - telemetry_period;
    auto last_status_time = last_control_time -
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / status_hz));
    int step_count = 0;
    auto last_print_time = std::chrono::steady_clock::now();
    const double target_control_hz = (control_dt > 1e-6f) ? (1.0 / control_dt) : 0.0;
    const double target_rl_hz = (rl_dt > 1e-6f) ? (1.0 / rl_dt) : 0.0;

    runtime_logging::Log(
        runtime_logging::Level::kInfo, "control runtime started", false);
    std::cout << "[control_demo] 启动（使用 transport_executor）\n";
    std::cout << "\n\n\n";

    while (g_running) {
        // 1) drain 命令（读空 buffer，取最新）
        {
            robot_base::Command hmi;
            while (transport->RecvCommand(hmi)) {
                cmd = hmi;
                has_hmi = true;
                last_hmi_time = std::chrono::steady_clock::now();
            }
        }

        // 2) drain Driver 状态（读空 buffer，取最新）
        {
            robot_base::RobotData state;
            while (transport->RecvState(state)) {
                latest_state = state;
                has_state = true;
                if (!reported_state) {
                    runtime_logging::Log(runtime_logging::Level::kInfo,
                        "driver state channel connected", false);
                    reported_state = true;
                }
            }
        }

        // 3) 基于时间触发控制周期
        auto now = std::chrono::steady_clock::now();
        const bool hmi_connected = has_hmi &&
            std::chrono::duration<double>(now - last_hmi_time).count()
                <= hmi_command_timeout;
        if (hmi_connected != previous_hmi_connected) {
            runtime_logging::Log(
                hmi_connected ? runtime_logging::Level::kInfo
                    : runtime_logging::Level::kWarning,
                hmi_connected ? "HMI command channel connected"
                    : "HMI command channel timed out",
                false);
            previous_hmi_connected = hmi_connected;
        }
        if (!hmi_connected) {
            // HMI 退出或链路中断后不保留旧速度和一次性请求。
            cmd.key = 0;
            cmd.vx = 0.0f;
            cmd.vy = 0.0f;
            cmd.wz = 0.0f;
            cmd.switch_policy.clear();
        }
        if (bm.CurrentState() == behavior_manager::StateName::RL) {
            const auto *limits = runtime_config::FindPolicyCommandLimits(
                policy_command_limits, bm.CurrentPolicyName());
            runtime_config::ApplyPolicyCommandLimits(limits, &cmd);
        } else {
            // 禁止在非 RL 状态预置速度，避免进入 RL 时突然起步。
            cmd.vx = 0.0f;
            cmd.vy = 0.0f;
            cmd.wz = 0.0f;
        }
        double elapsed = std::chrono::duration<double>(now - last_control_time).count();
        if (elapsed >= control_dt && has_state) {
            last_control_time = now;
            const float actual_control_dt = static_cast<float>(elapsed);

            bm.SetSensorData(latest_state);
            bm.SetCommand(cmd);
            bm.Step(actual_control_dt, rl_dt);
            const auto &out = bm.GetOutput();
            robot_base::ControlCmd ctrl;
            ctrl.enable = out.enable;
            ctrl.actuation_mode = out.actuation_mode;
            ctrl.target_pos = out.target_pos;
            ctrl.target_vel = out.target_vel;
            ctrl.target_torque = out.target_torque;
            ctrl.kp = out.kp;
            ctrl.kd = out.kd;
            // 透传 FSM 状态作为通用控制模式，由 driver backend 自主解释。
            ctrl.mode = ToControlMode(bm.CurrentState());
            transport->SendControl(ctrl);
            latest_control = ctrl;
            has_control = true;

            if (bm.CurrentState() != behavior_manager::StateName::RL) {
                cmd.vx = 0.0f;
                cmd.vy = 0.0f;
                cmd.wz = 0.0f;
            }

            step_count++;
        }

        if (logging_config.telemetry_enabled && has_state && has_control &&
            now - last_telemetry_time >= telemetry_period) {
            const double actual_control_hz = elapsed > 1.0e-9 ? 1.0 / elapsed : 0.0;
            RecordControlTelemetry(latest_state, latest_control, bm.CurrentState(),
                bm.CurrentPolicyName(), hmi_connected, cmd, actual_control_hz,
                bm.GetRlFreq(), joint_names);
            last_telemetry_time = now;
        }

        // 真实 FSM、策略和 control 采用速度定频回传给 HMI。
        if (std::chrono::duration<double>(now - last_status_time).count()
            >= 1.0 / status_hz) {
            robot_base::ControlStatus status;
            status.mode = ToControlMode(bm.CurrentState());
            status.zero_ready = bm.IsZeroReady();
            status.hmi_connected = hmi_connected;
            status.vx = cmd.vx;
            status.vy = cmd.vy;
            status.wz = cmd.wz;
            status.rl_frequency_hz = static_cast<float>(bm.GetRlFreq());
            status.active_policy = bm.CurrentPolicyName();
            transport->SendStatus(status);
            last_status_time = now;
        }

        // 固定 3 行动态刷新，避免增量日志刷屏
        double elapsed_print = std::chrono::duration<double>(now - last_print_time).count();
        if (elapsed_print >= 0.1) {
            const double control_freq = step_count / elapsed_print;
            const double rl_freq = bm.GetRlFreq();
            const std::string policy =
                bm.CurrentPolicyName().empty() ? "-" : bm.CurrentPolicyName();

            std::ostringstream line0, line1, line2;
            line0 << "[control] state=" << StateNameStr(bm.CurrentState())
                << " policy=" << policy
                << " hmi=" << (hmi_connected ? "online" : "timeout")
                << std::fixed << std::setprecision(2)
                << " cmd=(" << cmd.vx << "," << cmd.vy << "," << cmd.wz << ")";
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

    runtime_logging::Log(
        runtime_logging::Level::kInfo, "control runtime stopped", false);
    std::cout << "\n[control_demo] 退出\n";
    return 0;
}
