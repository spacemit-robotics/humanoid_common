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
#include <limits>
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
#include "transport_packet.h"

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

bool TestPacketValidation() {
    transport::HmiCmdPacket hmi{};
    hmi.header.type = static_cast<uint16_t>(transport::MsgType::HMI_CMD);
    if (!transport::ValidHmiCmdPacket(hmi)) return false;
    hmi.vx = std::numeric_limits<float>::quiet_NaN();
    if (transport::ValidHmiCmdPacket(hmi)) return false;

    transport::RobotStatePacket state{};
    state.header.type = static_cast<uint16_t>(transport::MsgType::ROBOT_STATE);
    state.num_dof = 2;
    if (!transport::ValidRobotStatePacket(state)) return false;
    state.joint_pos[1] = std::numeric_limits<double>::infinity();
    if (transport::ValidRobotStatePacket(state)) return false;

    transport::ControlCmdPacket control{};
    control.header.type = static_cast<uint16_t>(transport::MsgType::CONTROL_CMD);
    control.num_dof = 2;
    control.control_mode = static_cast<int8_t>(robot_base::ControlMode::RL);
    control.actuation_mode = static_cast<int8_t>(robot_base::ActuationMode::HYBRID);
    if (!transport::ValidControlCmdPacket(control)) return false;
    control.enable = 2;
    if (transport::ValidControlCmdPacket(control)) return false;
    control.enable = 0;
    control.control_mode = 127;
    if (transport::ValidControlCmdPacket(control)) return false;

    transport::ControlStatusPacket status{};
    status.header.type = static_cast<uint16_t>(transport::MsgType::CONTROL_STATUS);
    if (!transport::ValidControlStatusPacket(status)) return false;
    status.hmi_connected = 2;
    if (transport::ValidControlStatusPacket(status)) return false;

    transport::FaultPacket fault{};
    fault.active = 1;
    if (transport::ValidFaultPacket(fault)) return false;

    robot_base::FaultStatus long_fault;
    long_fault.active = true;
    long_fault.latched = true;
    long_fault.source = robot_base::FaultSource::MOTOR;
    long_fault.code = robot_base::FaultCode::FEEDBACK_TIMEOUT;
    long_fault.sequence = 1;
    long_fault.timestamp_s = 1.0;
    long_fault.detail.assign(transport::kFaultDetailLength * 2, 'x');
    if (!transport::CanEncodeFault(long_fault)) return false;
    transport::EncodeFault(long_fault, &fault);
    robot_base::FaultStatus decoded_fault;
    if (!transport::DecodeFault(fault, &decoded_fault) ||
        decoded_fault.detail.size() >= transport::kFaultDetailLength ||
        decoded_fault.detail.find("...[truncated]") == std::string::npos) {
        return false;
    }

    long_fault.detail = std::string("invalid\0detail", 14);
    if (transport::CanEncodeFault(long_fault)) return false;

    std::cout << "[test] V5 packet validation: PASS\n";
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

    all_ok = TestPacketValidation() && all_ok;
    all_ok = TestStaleShmDataDiscard() && all_ok;

    // ==================== 创建 Driver 端 ====================

    auto driver = transport::CreateV2(yaml_path);
    if (!driver->Init(yaml_path, transport::Role::DRIVER)) {
        std::cerr << "[test] Driver 初始化失败\n";
        return 1;
    }
    std::cout << "[test] Driver 初始化成功\n";

    // ==================== 创建 Control 端 ====================

    auto control = transport::CreateV2(yaml_path);
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
    robot_base::FaultStatus state_fault;
    state_fault.active = true;
    state_fault.latched = true;
    state_fault.source = robot_base::FaultSource::IMU;
    state_fault.code = robot_base::FaultCode::FEEDBACK_TIMEOUT;
    state_fault.native_code = -7;
    state_fault.sequence = 42;
    state_fault.timestamp_s = 1.2;
    state_fault.detail = "test IMU timeout";

    if (!driver->SendStateV2(state, state_fault)) {
        std::cerr << "[test] 合法状态发送失败\n";
        all_ok = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    robot_base::RobotData recv_state;
    robot_base::FaultStatus recv_state_fault;
    if (control->RecvStateV2(recv_state, recv_state_fault)) {
        std::cout << "[test] 状态接收成功: num_dof=" << recv_state.num_dof
                << ", time=" << recv_state.time << ", rpy=(" << recv_state.rpy[0] << ","
                << recv_state.rpy[1] << "," << recv_state.rpy[2] << ")\n";
        bool ok = (recv_state.num_dof == state.num_dof) &&
                (std::abs(recv_state.time - state.time) < 1e-6) &&
                (recv_state.acceleration == state.acceleration) &&
                (recv_state.joint_torque == state.joint_torque) &&
                (recv_state.joint_temperature == state.joint_temperature) &&
                (recv_state.joint_error == state.joint_error) &&
                recv_state_fault.source == state_fault.source &&
                recv_state_fault.code == state_fault.code &&
                recv_state_fault.native_code == state_fault.native_code &&
                recv_state_fault.sequence == state_fault.sequence &&
                recv_state_fault.detail == state_fault.detail;
        all_ok = all_ok && ok;
        std::cout << "[test] 状态数据验证: " << (ok ? "通过" : "失败") << "\n";
    } else {
        std::cerr << "[test] 状态接收失败\n";
        all_ok = false;
    }

    robot_base::RobotData invalid_state = state;
    invalid_state.num_dof = transport::kMaxDof + 1;
    invalid_state.InitJointVectors();
    bool invalid_state_rejected = !driver->SendStateV2(invalid_state, state_fault);
    invalid_state = state;
    invalid_state.joint_vel.pop_back();
    invalid_state_rejected = !driver->SendStateV2(invalid_state, state_fault) &&
        invalid_state_rejected;
    invalid_state = state;
    invalid_state.rpy[0] = std::numeric_limits<double>::quiet_NaN();
    invalid_state_rejected = !driver->SendStateV2(invalid_state, state_fault) &&
        invalid_state_rejected;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    robot_base::RobotData unexpected_state;
    robot_base::FaultStatus unexpected_state_fault;
    invalid_state_rejected =
        !control->RecvStateV2(unexpected_state, unexpected_state_fault) &&
        invalid_state_rejected;
    all_ok = invalid_state_rejected && all_ok;
    std::cout << "[test] 非法状态发送拒绝: "
        << (invalid_state_rejected ? "通过" : "失败") << "\n";

    // 旧 TransportBase 调用仍按原数据结构工作；扩展故障只走 v2 旁路。
    transport::TransportBase &legacy_driver = *driver;
    transport::TransportBase &legacy_control = *control;
    state.time = 2.345;
    legacy_driver.SendState(state);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    robot_base::RobotData legacy_state;
    const bool legacy_state_ok = legacy_control.RecvState(legacy_state) &&
        legacy_state.time == state.time &&
        legacy_state.joint_pos == state.joint_pos;
    all_ok = legacy_state_ok && all_ok;
    std::cout << "[test] 旧状态接口兼容: "
        << (legacy_state_ok ? "通过" : "失败") << "\n";

    // ==================== 测试控制通道 ====================

    std::cout << "\n--- 测试控制通道 (Control → Driver) ---\n";

    // 使用非默认 TORQUE 模式，验证模式枚举和全部命令字段能完整往返。
    // 此示例只测通信，不执行电机命令。
    robot_base::ControlCmd cmd;
    cmd.enable = true;
    cmd.mode = robot_base::ControlMode::RL;
    cmd.actuation_mode = robot_base::ActuationMode::TORQUE;
    cmd.target_pos.resize(state.num_dof);
    cmd.target_vel.assign(state.num_dof, 0.0);
    cmd.target_torque.assign(state.num_dof, 0.25);
    cmd.kp.assign(state.num_dof, 100.0);
    cmd.kd.assign(state.num_dof, 2.0);
    for (int i = 0; i < state.num_dof; ++i) {
        cmd.target_pos[i] = 0.5 * i;
    }

    if (!control->SendControlV2(cmd)) {
        std::cerr << "[test] 合法控制命令发送失败\n";
        all_ok = false;
    }
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

    robot_base::ControlCmd invalid_cmd = cmd;
    invalid_cmd.target_vel.pop_back();
    bool invalid_control_rejected = !control->SendControlV2(invalid_cmd);
    invalid_cmd = cmd;
    invalid_cmd.target_pos[0] = std::numeric_limits<double>::quiet_NaN();
    invalid_control_rejected = !control->SendControlV2(invalid_cmd) &&
        invalid_control_rejected;
    invalid_cmd = cmd;
    invalid_cmd.kp[0] = -1.0;
    invalid_control_rejected = !control->SendControlV2(invalid_cmd) &&
        invalid_control_rejected;
    invalid_cmd = cmd;
    const std::size_t oversized_dof = transport::kMaxDof + 1;
    invalid_cmd.target_pos.assign(oversized_dof, 0.0);
    invalid_cmd.target_vel.assign(oversized_dof, 0.0);
    invalid_cmd.target_torque.assign(oversized_dof, 0.0);
    invalid_cmd.kp.assign(oversized_dof, 0.0);
    invalid_cmd.kd.assign(oversized_dof, 0.0);
    invalid_control_rejected = !control->SendControlV2(invalid_cmd) &&
        invalid_control_rejected;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    robot_base::ControlCmd unexpected_cmd;
    invalid_control_rejected = !driver->RecvControl(unexpected_cmd) &&
        invalid_control_rejected;
    all_ok = invalid_control_rejected && all_ok;
    std::cout << "[test] 非法控制命令发送拒绝: "
        << (invalid_control_rejected ? "通过" : "失败") << "\n";

    // ==================== 测试命令通道 ====================

    std::cout << "\n--- 测试命令通道 (Hmi → Control) ---\n";

    auto hmi = transport::CreateV2(yaml_path);
    if (!hmi->Init(yaml_path, transport::Role::HMI)) {
        std::cerr << "[test] Hmi 初始化失败\n";
        return 1;
    }

    robot_base::Command hmi_cmd;
    hmi_cmd.key = 3;
    hmi_cmd.vx = 0.5f;
    hmi_cmd.vy = 0.1f;
    hmi_cmd.wz = 0.2f;

    if (!hmi->SendCommandV2(hmi_cmd, 42)) {
        std::cerr << "[test] 合法 HMI 命令发送失败\n";
        all_ok = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    robot_base::Command recv_hmi;
    uint64_t recv_fault_ack_sequence = 0;
    if (control->RecvCommandV2(recv_hmi, recv_fault_ack_sequence)) {
        std::cout << "[test] 命令接收成功: key=" << recv_hmi.key << ", vx=" << recv_hmi.vx
                << ", vy=" << recv_hmi.vy << ", wz=" << recv_hmi.wz << "\n";
        bool ok = (recv_hmi.key == 3) &&
            (std::abs(recv_hmi.vx - 0.5f) < 1e-4f) &&
            recv_fault_ack_sequence == 42;
        all_ok = all_ok && ok;
        std::cout << "[test] 命令数据验证: " << (ok ? "通过" : "失败") << "\n";
    } else {
        std::cerr << "[test] 命令接收失败\n";
        all_ok = false;
    }

    robot_base::Command invalid_hmi_cmd = hmi_cmd;
    invalid_hmi_cmd.vx = std::numeric_limits<float>::quiet_NaN();
    bool invalid_hmi_rejected = !hmi->SendCommandV2(invalid_hmi_cmd, 0);
    invalid_hmi_cmd = hmi_cmd;
    invalid_hmi_cmd.switch_policy.assign(transport::kPolicyNameLength, 'x');
    invalid_hmi_rejected = !hmi->SendCommandV2(invalid_hmi_cmd, 0) &&
        invalid_hmi_rejected;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    robot_base::Command unexpected_hmi_cmd;
    uint64_t unexpected_acknowledgement = 0;
    invalid_hmi_rejected =
        !control->RecvCommandV2(unexpected_hmi_cmd, unexpected_acknowledgement) &&
        invalid_hmi_rejected;
    all_ok = invalid_hmi_rejected && all_ok;
    std::cout << "[test] 非法 HMI 命令发送拒绝: "
        << (invalid_hmi_rejected ? "通过" : "失败") << "\n";

    transport::TransportBase &legacy_hmi = *hmi;
    hmi_cmd.key = 1;
    legacy_hmi.SendCommand(hmi_cmd);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    robot_base::Command legacy_command;
    const bool legacy_command_ok = legacy_control.RecvCommand(legacy_command) &&
        legacy_command.key == hmi_cmd.key &&
        legacy_command.switch_policy == hmi_cmd.switch_policy;
    all_ok = legacy_command_ok && all_ok;
    std::cout << "[test] 旧命令接口兼容: "
        << (legacy_command_ok ? "通过" : "失败") << "\n";

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
    robot_base::FaultStatus status_fault;
    status_fault.latched = true;
    status_fault.source = robot_base::FaultSource::POLICY;
    status_fault.code = robot_base::FaultCode::INFERENCE_TIMEOUT;
    status_fault.native_code = 9;
    status_fault.sequence = 11;
    status_fault.detail = "test policy timeout";
    if (!control->SendStatusV2(status, status_fault)) {
        std::cerr << "[test] 合法 Control 状态发送失败\n";
        all_ok = false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));

    robot_base::ControlStatus recv_status;
    robot_base::FaultStatus recv_status_fault;
    if (!hmi->RecvStatusV2(recv_status, recv_status_fault)) {
        std::cerr << "[test] Control 状态接收失败\n";
        return 1;
    }
    const bool status_ok = recv_status.mode == robot_base::ControlMode::RL &&
        recv_status.hmi_connected &&
        std::abs(recv_status.vx - status.vx) < 1e-4f &&
        recv_status.active_policy == status.active_policy &&
        recv_status_fault.latched &&
        recv_status_fault.source == status_fault.source &&
        recv_status_fault.code == status_fault.code &&
        recv_status_fault.detail == status_fault.detail;
    std::cout << "[test] 状态回传: mode="
        << static_cast<int>(recv_status.mode)
        << ", policy=" << recv_status.active_policy
        << ", vx=" << recv_status.vx << "\n";
    std::cout << "[test] 状态回传验证: "
        << (status_ok ? "通过" : "失败") << "\n";
    all_ok = all_ok && status_ok;

    robot_base::ControlStatus invalid_status = status;
    invalid_status.rl_frequency_hz = std::numeric_limits<float>::infinity();
    bool invalid_status_rejected = !control->SendStatusV2(invalid_status, status_fault);
    invalid_status = status;
    invalid_status.active_policy.assign(transport::kPolicyNameLength, 'x');
    invalid_status_rejected = !control->SendStatusV2(invalid_status, status_fault) &&
        invalid_status_rejected;
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    robot_base::ControlStatus unexpected_status;
    robot_base::FaultStatus unexpected_status_fault;
    invalid_status_rejected =
        !hmi->RecvStatusV2(unexpected_status, unexpected_status_fault) &&
        invalid_status_rejected;
    all_ok = invalid_status_rejected && all_ok;
    std::cout << "[test] 非法 Control 状态发送拒绝: "
        << (invalid_status_rejected ? "通过" : "失败") << "\n";

    status.active_policy = "legacy_policy";
    legacy_control.SendStatus(status);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    robot_base::ControlStatus legacy_status;
    const bool legacy_status_ok = legacy_hmi.RecvStatus(legacy_status) &&
        legacy_status.active_policy == status.active_policy;
    all_ok = legacy_status_ok && all_ok;
    std::cout << "[test] 旧状态回传接口兼容: "
        << (legacy_status_ok ? "通过" : "失败") << "\n";

    if (!all_ok) return 1;
    std::cout << "\n[test] 全部测试完成\n";
    return 0;
}
