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
 * - Control 状态回传（SendStatus / RecvStatus）
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
#include <cstdint>
#include <cmath>
#include <iostream>
#include <string>
#include <thread>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include "robot_base.h"
#include "shm_transport.h"
#include "transport_executor.h"

namespace {

bool TestStaleShmDataDiscard() {
    const std::string channel =
        "/hmrs_stale_test_" + std::to_string(static_cast<long long>(getpid()));
    shm_unlink(channel.c_str());

    transport_shm::ShmConfig config;
    config.channel_name = channel;
    config.capacity = 8;
    config.slot_size = sizeof(uint64_t);
    config.create_if_not_exist = true;

    const pid_t child = fork();
    if (child < 0) {
        std::cerr << "[test] fork failed\n";
        return false;
    }
    if (child == 0) {
        umask(0077);
        config.role = transport_shm::Role::WRITER;
        transport_shm::Shm stale_writer;
        const uint64_t stale_command = 0x484f4d45ULL;
        const bool ok = stale_writer.Init(config) &&
            stale_writer.Write(&stale_command, sizeof(stale_command));
        _exit(ok ? 0 : 1);
    }

    int child_status = 0;
    if (waitpid(child, &child_status, 0) != child ||
        !WIFEXITED(child_status) || WEXITSTATUS(child_status) != 0) {
        std::cerr << "[test] failed to create stale SHM data\n";
        shm_unlink(channel.c_str());
        return false;
    }

    const int shm_fd = shm_open(channel.c_str(), O_RDWR, 0);
    struct stat shm_stat {};
    if (shm_fd < 0 || fstat(shm_fd, &shm_stat) < 0 ||
        (shm_stat.st_mode & 0777) != 0660 ||
        shm_stat.st_gid != getegid()) {
        std::cerr << "[test] SHM group permissions were not applied\n";
        if (shm_fd >= 0)
            close(shm_fd);
        shm_unlink(channel.c_str());
        return false;
    }
    close(shm_fd);

    config.role = transport_shm::Role::READER;
    transport_shm::Shm reader;
    if (!reader.Init(config)) {
        std::cerr << "[test] stale-data reader init failed\n";
        shm_unlink(channel.c_str());
        return false;
    }

    uint64_t received = 0;
    std::size_t received_size = 0;
    if (reader.Read(&received, sizeof(received), received_size)) {
        std::cerr << "[test] stale SHM command was consumed\n";
        shm_unlink(channel.c_str());
        return false;
    }

    config.role = transport_shm::Role::WRITER;
    transport_shm::Shm writer;
    const uint64_t fresh_command = 0x504f5745524f4646ULL;
    if (!writer.Init(config) ||
        !writer.Write(&fresh_command, sizeof(fresh_command)) ||
        !reader.Read(&received, sizeof(received), received_size) ||
        received_size != sizeof(received) || received != fresh_command) {
        std::cerr << "[test] fresh SHM command was not received\n";
        shm_unlink(channel.c_str());
        return false;
    }

    std::cout << "[test] SHM permissions and stale-data discard: PASS\n";
    return true;
}

}  // namespace

int main(int argc, char *argv[]) {
    if (argc < 2) {
        std::cerr << "用法: " << argv[0] << " <yaml_path>\n"
                << "示例: ./test_transport_executor ../../../application/config/g1.yaml\n";
        return 1;
    }
    const std::string yaml_path = argv[1];
    bool all_ok = true;

    all_ok = TestStaleShmDataDiscard() && all_ok;

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
    state.acceleration = {0.4, 0.5, 9.7};
    for (int i = 0; i < state.num_dof; ++i) {
        state.joint_pos[i] = 0.1 * i;
        state.joint_vel[i] = 0.01 * i;
        state.joint_torque[i] = 0.02 * i;
        state.joint_temperature[i] = 30.0 + i;
        state.joint_error[i] = static_cast<uint32_t>(i);
    }

    driver->SendState(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    robot_base::RobotData recv_state;
    if (control->RecvState(recv_state)) {
        std::cout << "[test] 状态接收成功: num_dof=" << recv_state.num_dof
                << ", time=" << recv_state.time << ", rpy=(" << recv_state.rpy[0] << ","
                << recv_state.rpy[1] << "," << recv_state.rpy[2] << ")\n";
        bool ok = (recv_state.num_dof == state.num_dof) &&
                (std::abs(recv_state.time - state.time) < 1e-6) &&
                (recv_state.acceleration == state.acceleration) &&
                (recv_state.joint_torque == state.joint_torque) &&
                (recv_state.joint_temperature == state.joint_temperature) &&
                (recv_state.joint_error == state.joint_error);
        all_ok = all_ok && ok;
        std::cout << "[test] 状态数据验证: " << (ok ? "通过" : "失败") << "\n";
    } else {
        std::cerr << "[test] 状态接收失败\n";
        all_ok = false;
    }

    // ==================== 测试控制通道 ====================

    std::cout << "\n--- 测试控制通道 (Control → Driver) ---\n";

    // 使用非默认 TORQUE 模式，验证模式枚举和全部命令字段能完整往返。
    // 此示例只测通信，不执行电机命令。
    robot_base::ControlCmd cmd;
    cmd.enable = true;
    cmd.actuation_mode = robot_base::ActuationMode::TORQUE;
    cmd.target_pos.resize(state.num_dof);
    cmd.target_vel.assign(state.num_dof, 0.0);
    cmd.target_torque.assign(state.num_dof, 0.25);
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
                && (recv_cmd.actuation_mode == cmd.actuation_mode)
                && (recv_cmd.target_pos.size() == cmd.target_pos.size())
                && (recv_cmd.target_vel.size() == cmd.target_vel.size())
                && (recv_cmd.target_torque == cmd.target_torque)
                && (recv_cmd.kp.size() == cmd.kp.size())
                && (recv_cmd.kd.size() == cmd.kd.size());
        all_ok = all_ok && ok;
        std::cout << "[test] 控制数据验证: " << (ok ? "通过" : "失败") << "\n";
    } else {
        std::cerr << "[test] 控制命令接收失败\n";
        all_ok = false;
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
        all_ok = all_ok && ok;
        std::cout << "[test] 命令数据验证: " << (ok ? "通过" : "失败") << "\n";
    } else {
        std::cerr << "[test] 命令接收失败\n";
        all_ok = false;
    }

    // ==================== 测试 Control 状态回传 ====================

    std::cout << "\n--- 测试状态回传 (Control → Hmi) ---\n";

    robot_base::ControlStatus status;
    status.mode = robot_base::ControlMode::RL;
    status.zero_ready = false;
    status.hmi_connected = true;
    status.vx = 0.3f;
    status.vy = -0.1f;
    status.wz = 0.2f;
    status.rl_frequency_hz = 49.8f;
    status.active_policy = "test_policy";
    control->SendStatus(status);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    robot_base::ControlStatus recv_status;
    if (!hmi->RecvStatus(recv_status)) {
        std::cerr << "[test] Control 状态接收失败\n";
        return 1;
    }
    const bool status_ok = recv_status.mode == robot_base::ControlMode::RL &&
        recv_status.hmi_connected &&
        std::abs(recv_status.vx - status.vx) < 1e-4f &&
        recv_status.active_policy == status.active_policy;
    std::cout << "[test] 状态回传: mode="
        << static_cast<int>(recv_status.mode)
        << ", policy=" << recv_status.active_policy
        << ", vx=" << recv_status.vx << "\n";
    std::cout << "[test] 状态回传验证: "
        << (status_ok ? "通过" : "失败") << "\n";
    all_ok = all_ok && status_ok;

    if (!all_ok) return 1;
    std::cout << "\n[test] 全部测试完成\n";
    return 0;
}
