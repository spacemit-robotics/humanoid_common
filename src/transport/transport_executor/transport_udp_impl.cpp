/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file transport_udp_impl.cpp
 * @brief UDP 传输实现
 *
 * 将 robot_base 类型与 UDP POD 数据包之间的序列化/反序列化封装在内部。
 */

#include "transport_udp_impl.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

namespace transport {

// UDP 默认超时（毫秒）
constexpr int kUdpTimeoutMs = 2;

TransportUdpImpl::TransportUdpImpl() = default;
TransportUdpImpl::~TransportUdpImpl() = default;

bool TransportUdpImpl::Init(const std::string& yaml_path, Role role) {
    role_ = role;

    // 读取传输配置
    robot_base::YamlFile yaml_file = robot_base::YamlFile::Load(yaml_path);
    std::string driver_ip = "127.0.0.1";
    std::string control_ip = "127.0.0.1";
    std::string hmi_ip = "127.0.0.1";
    int state_port = 8800;
    int control_port = 8801;
    int hmi_port = 8802;
    int status_port = 8803;

    const std::string udp_prefix = "transport.udp";
    const std::string legacy_ip = yaml_file.Read<std::string>(udp_prefix + ".ip").value_or("");
    driver_ip = yaml_file.Read<std::string>(udp_prefix + ".driver_ip")
                    .value_or(!legacy_ip.empty() ? legacy_ip : driver_ip);
    control_ip = yaml_file.Read<std::string>(udp_prefix + ".control_ip")
                    .value_or(!legacy_ip.empty() ? legacy_ip : control_ip);
    hmi_ip = yaml_file.Read<std::string>(udp_prefix + ".hmi_ip")
                    .value_or(!legacy_ip.empty() ? legacy_ip : driver_ip);
    state_port = yaml_file.Read<int>(udp_prefix + ".state_port").value_or(state_port);
    control_port = yaml_file.Read<int>(udp_prefix + ".control_port").value_or(control_port);
    hmi_port = yaml_file.Read<int>(udp_prefix + ".hmi_port").value_or(control_port + 1);
    status_port = yaml_file.Read<int>(udp_prefix + ".status_port").value_or(hmi_port + 1);

    if (role == Role::DRIVER) {
        // Driver: 发送状态 → control_ip:state_port
        state_sender_ = std::make_unique<transport_udp::Udp>();
        transport_udp::UdpConfig sc;
        sc.role = transport_udp::Role::CLIENT;
        sc.remote_ip = control_ip;
        sc.remote_port = state_port;
        sc.timeout_ms = kUdpTimeoutMs;
        if (!state_sender_->Init(sc)) {
            std::cerr << "[transport_executor] state_sender 初始化失败\n";
            return false;
        }

        // Driver: 接收控制 ← 0.0.0.0:control_port
        control_receiver_ = std::make_unique<transport_udp::Udp>();
        transport_udp::UdpConfig rc;
        rc.role = transport_udp::Role::SERVER;
        rc.local_ip = "0.0.0.0";
        rc.local_port = control_port;
        rc.timeout_ms = kUdpTimeoutMs;
        if (!control_receiver_->Init(rc)) {
            std::cerr << "[transport_executor] control_receiver 初始化失败\n";
            return false;
        }

    } else if (role == Role::CONTROL) {
        // Control: 接收状态 ← 0.0.0.0:state_port
        state_receiver_ = std::make_unique<transport_udp::Udp>();
        transport_udp::UdpConfig sc;
        sc.role = transport_udp::Role::SERVER;
        sc.local_ip = "0.0.0.0";
        sc.local_port = state_port;
        sc.timeout_ms = kUdpTimeoutMs;
        if (!state_receiver_->Init(sc)) {
            std::cerr << "[transport_executor] state_receiver 初始化失败\n";
            return false;
        }

        // Control: 发送控制 → driver_ip:control_port
        control_sender_ = std::make_unique<transport_udp::Udp>();
        transport_udp::UdpConfig cc;
        cc.role = transport_udp::Role::CLIENT;
        cc.remote_ip = driver_ip;
        cc.remote_port = control_port;
        cc.timeout_ms = kUdpTimeoutMs;
        if (!control_sender_->Init(cc)) {
            std::cerr << "[transport_executor] control_sender 初始化失败\n";
            return false;
        }

        // Control: 接收 HMI ← 0.0.0.0:hmi_port
        hmi_receiver_ = std::make_unique<transport_udp::Udp>();
        transport_udp::UdpConfig hc;
        hc.role = transport_udp::Role::SERVER;
        hc.local_ip = "0.0.0.0";
        hc.local_port = hmi_port;
        hc.timeout_ms = kUdpTimeoutMs;
        if (!hmi_receiver_->Init(hc)) {
            std::cerr << "[transport_executor] hmi_receiver 初始化失败\n";
            return false;
        }

        // Control: 回传运行状态 → hmi_ip:status_port
        status_sender_ = std::make_unique<transport_udp::Udp>();
        transport_udp::UdpConfig ss;
        ss.role = transport_udp::Role::CLIENT;
        ss.remote_ip = hmi_ip;
        ss.remote_port = status_port;
        ss.timeout_ms = kUdpTimeoutMs;
        if (!status_sender_->Init(ss)) {
            std::cerr << "[transport_executor] status_sender 初始化失败\n";
            return false;
        }

    } else if (role == Role::HMI) {
        // Hmi: 发送 HMI → control_ip:hmi_port
        hmi_sender_ = std::make_unique<transport_udp::Udp>();
        transport_udp::UdpConfig hc;
        hc.role = transport_udp::Role::CLIENT;
        hc.remote_ip = control_ip;
        hc.remote_port = hmi_port;
        hc.timeout_ms = kUdpTimeoutMs;
        if (!hmi_sender_->Init(hc)) {
            std::cerr << "[transport_executor] hmi_sender 初始化失败\n";
            return false;
        }

        // Hmi: 接收 Control 运行状态 ← 0.0.0.0:status_port
        status_receiver_ = std::make_unique<transport_udp::Udp>();
        transport_udp::UdpConfig sr;
        sr.role = transport_udp::Role::SERVER;
        sr.local_ip = "0.0.0.0";
        sr.local_port = status_port;
        sr.timeout_ms = kUdpTimeoutMs;
        if (!status_receiver_->Init(sr)) {
            std::cerr << "[transport_executor] status_receiver 初始化失败\n";
            return false;
        }
    }

    return true;
}

// ==================== 状态通道 ====================

void TransportUdpImpl::SendState(const robot_base::RobotData& state) {
    (void)SendStateV2(state, {});
}

bool TransportUdpImpl::SendStateV2(const robot_base::RobotData &state,
        const robot_base::FaultStatus &fault) {
    if (!state_sender_ || !CanEncodeRobotState(state, fault)) return false;

    RobotStatePacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::ROBOT_STATE);
    p.header.seq = state_seq_ + 1;
    p.num_dof = state.num_dof;
    p.time = state.time;
    std::memcpy(p.rpy, state.rpy.data(), sizeof(p.rpy));
    std::memcpy(p.gyro, state.gyro.data(), sizeof(p.gyro));
    std::memcpy(p.acceleration, state.acceleration.data(), sizeof(p.acceleration));
    std::memcpy(p.base_pos, state.base_pos.data(), sizeof(p.base_pos));
    std::memcpy(p.base_quat, state.base_quat.data(), sizeof(p.base_quat));
    std::memcpy(p.base_vel, state.base_vel.data(), sizeof(p.base_vel));

    for (int i = 0; i < p.num_dof; ++i) {
        p.joint_pos[i] = (i < static_cast<int>(state.joint_pos.size())) ? state.joint_pos[i] : 0.0;
        p.joint_vel[i] = (i < static_cast<int>(state.joint_vel.size())) ? state.joint_vel[i] : 0.0;
        p.joint_torque[i] =
            (i < static_cast<int>(state.joint_torque.size())) ? state.joint_torque[i] : 0.0;
        p.joint_temperature[i] = (i < static_cast<int>(state.joint_temperature.size()))
            ? state.joint_temperature[i]
            : 0.0;
        p.joint_error[i] =
            (i < static_cast<int>(state.joint_error.size())) ? state.joint_error[i] : 0U;
    }
    EncodeFault(fault, &p.fault);

    if (!ValidRobotStatePacket(p)) return false;
    if (state_sender_->Send(&p, sizeof(p)) != static_cast<int>(sizeof(p))) return false;
    state_seq_ = p.header.seq;
    return true;
}

bool TransportUdpImpl::RecvState(robot_base::RobotData& state) {
    robot_base::FaultStatus ignored_fault;
    return RecvStateV2(state, ignored_fault);
}

bool TransportUdpImpl::RecvStateV2(robot_base::RobotData &state,
        robot_base::FaultStatus &fault) {
    if (!state_receiver_)
        return false;

    RobotStatePacket p{};
    int n = state_receiver_->Recv(&p, sizeof(p));
    if (n != static_cast<int>(sizeof(p)) || !ValidRobotStatePacket(p))
        return false;

    state.num_dof = p.num_dof;
    state.InitJointVectors();
    state.time = p.time;
    std::memcpy(state.rpy.data(), p.rpy, sizeof(p.rpy));
    std::memcpy(state.gyro.data(), p.gyro, sizeof(p.gyro));
    std::memcpy(state.acceleration.data(), p.acceleration, sizeof(p.acceleration));
    std::memcpy(state.base_pos.data(), p.base_pos, sizeof(p.base_pos));
    std::memcpy(state.base_quat.data(), p.base_quat, sizeof(p.base_quat));
    std::memcpy(state.base_vel.data(), p.base_vel, sizeof(p.base_vel));

    for (int i = 0; i < state.num_dof; ++i) {
        state.joint_pos[i] = p.joint_pos[i];
        state.joint_vel[i] = p.joint_vel[i];
        state.joint_torque[i] = p.joint_torque[i];
        state.joint_temperature[i] = p.joint_temperature[i];
        state.joint_error[i] = p.joint_error[i];
    }

    if (!DecodeFault(p.fault, &fault)) return false;

    return true;
}

// ==================== 控制通道 ====================

void TransportUdpImpl::SendControl(const robot_base::ControlCmd& cmd) {
    (void)SendControlV2(cmd);
}

bool TransportUdpImpl::SendControlV2(const robot_base::ControlCmd &cmd) {
    if (!control_sender_ || !CanEncodeControl(cmd)) return false;

    ControlCmdPacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::CONTROL_CMD);
    p.header.seq = control_seq_ + 1;
    p.num_dof = static_cast<int32_t>(cmd.target_pos.size());
    p.enable = cmd.enable ? 1 : 0;
    p.control_mode = static_cast<int8_t>(cmd.mode);
    p.actuation_mode = static_cast<int8_t>(cmd.actuation_mode);

    for (int i = 0; i < p.num_dof; ++i) {
        p.target_pos[i] = cmd.target_pos[i];
        p.target_vel[i] = cmd.target_vel[i];
        p.target_torque[i] = cmd.target_torque.empty() ? 0.0 : cmd.target_torque[i];
        p.kp[i] = cmd.kp[i];
        p.kd[i] = cmd.kd[i];
    }

    if (!ValidControlCmdPacket(p)) return false;
    if (control_sender_->Send(&p, sizeof(p)) != static_cast<int>(sizeof(p))) return false;
    control_seq_ = p.header.seq;
    return true;
}

bool TransportUdpImpl::RecvControl(robot_base::ControlCmd& cmd) {
    if (!control_receiver_)
        return false;

    ControlCmdPacket p{};
    int n = control_receiver_->Recv(&p, sizeof(p));
    if (n != static_cast<int>(sizeof(p)) || !ValidControlCmdPacket(p))
        return false;

    int ndof = p.num_dof;
    const auto actuation_mode = static_cast<robot_base::ActuationMode>(p.actuation_mode);
    cmd.enable = (p.enable != 0);
    cmd.mode = static_cast<robot_base::ControlMode>(p.control_mode);
    cmd.actuation_mode = actuation_mode;
    cmd.target_pos.resize(ndof);
    cmd.target_vel.resize(ndof);
    cmd.target_torque.resize(ndof);
    cmd.kp.resize(ndof);
    cmd.kd.resize(ndof);

    for (int i = 0; i < ndof; ++i) {
        cmd.target_pos[i] = p.target_pos[i];
        cmd.target_vel[i] = p.target_vel[i];
        cmd.target_torque[i] = p.target_torque[i];
        cmd.kp[i] = p.kp[i];
        cmd.kd[i] = p.kd[i];
    }

    return true;
}

// ==================== 命令通道 ====================

void TransportUdpImpl::SendCommand(const robot_base::Command& cmd) {
    (void)SendCommandV2(cmd, 0);
}

bool TransportUdpImpl::SendCommandV2(const robot_base::Command &cmd,
        uint64_t acknowledge_fault_sequence) {
    if (!hmi_sender_ || !CanEncodeCommand(cmd)) return false;

    HmiCmdPacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::HMI_CMD);
    p.header.seq = hmi_seq_ + 1;
    p.key = cmd.key;
    p.vx = cmd.vx;
    p.vy = cmd.vy;
    p.wz = cmd.wz;
    p.acknowledge_fault_sequence = acknowledge_fault_sequence;
    std::strncpy(p.switch_policy, cmd.switch_policy.c_str(), sizeof(p.switch_policy) - 1);
    p.interaction_sequence = cmd.interaction.sequence;
    p.interaction_operation = static_cast<uint8_t>(cmd.interaction.operation);
    std::strncpy(p.interaction_action, cmd.interaction.action.c_str(),
        sizeof(p.interaction_action) - 1);

    if (!ValidHmiCmdPacket(p)) return false;
    if (hmi_sender_->Send(&p, sizeof(p)) != static_cast<int>(sizeof(p))) return false;
    hmi_seq_ = p.header.seq;
    return true;
}

bool TransportUdpImpl::RecvCommand(robot_base::Command& cmd) {
    uint64_t ignored_acknowledgement = 0;
    return RecvCommandV2(cmd, ignored_acknowledgement);
}

bool TransportUdpImpl::RecvCommandV2(robot_base::Command &cmd,
        uint64_t &acknowledge_fault_sequence) {
    if (!hmi_receiver_)
        return false;

    HmiCmdPacket p{};
    int n = hmi_receiver_->Recv(&p, sizeof(p));
    if (n != static_cast<int>(sizeof(p)) || !ValidHmiCmdPacket(p))
        return false;

    cmd.key = p.key;
    cmd.vx = p.vx;
    cmd.vy = p.vy;
    cmd.wz = p.wz;
    acknowledge_fault_sequence = p.acknowledge_fault_sequence;
    cmd.switch_policy = DecodeText(p.switch_policy);
    cmd.interaction.sequence = p.interaction_sequence;
    cmd.interaction.operation =
        static_cast<robot_base::InteractionRequest::Operation>(
            p.interaction_operation);
    cmd.interaction.action = DecodeText(p.interaction_action);

    return true;
}

// ==================== Control 状态回传通道 ====================

void TransportUdpImpl::SendStatus(const robot_base::ControlStatus& status) {
    (void)SendStatusV2(status, {});
}

bool TransportUdpImpl::SendStatusV2(const robot_base::ControlStatus &status,
        const robot_base::FaultStatus &fault) {
    if (!status_sender_ || !CanEncodeStatus(status, fault)) return false;

    ControlStatusPacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::CONTROL_STATUS);
    p.header.seq = status_seq_ + 1;
    p.control_mode = static_cast<int8_t>(status.mode);
    p.zero_ready = status.zero_ready ? 1 : 0;
    p.hmi_connected = status.hmi_connected ? 1 : 0;
    p.vx = status.vx;
    p.vy = status.vy;
    p.wz = status.wz;
    p.rl_frequency_hz = status.rl_frequency_hz;
    std::strncpy(p.active_policy, status.active_policy.c_str(),
        sizeof(p.active_policy) - 1);
    p.interaction_sequence = status.interaction.sequence;
    p.interaction_request_accepted =
        status.interaction.request_accepted ? 1 : 0;
    p.interaction_phase = static_cast<uint8_t>(status.interaction.phase);
    p.interaction_progress = status.interaction.progress;
    std::strncpy(p.interaction_action, status.interaction.action.c_str(),
        sizeof(p.interaction_action) - 1);
    EncodeFault(fault, &p.fault);

    if (!ValidControlStatusPacket(p)) return false;
    if (status_sender_->Send(&p, sizeof(p)) != static_cast<int>(sizeof(p))) return false;
    status_seq_ = p.header.seq;
    return true;
}

bool TransportUdpImpl::RecvStatus(robot_base::ControlStatus& status) {
    robot_base::FaultStatus ignored_fault;
    return RecvStatusV2(status, ignored_fault);
}

bool TransportUdpImpl::RecvStatusV2(robot_base::ControlStatus &status,
        robot_base::FaultStatus &fault) {
    if (!status_receiver_)
        return false;

    ControlStatusPacket p{};
    int n = status_receiver_->Recv(&p, sizeof(p));
    if (n != static_cast<int>(sizeof(p)) || !ValidControlStatusPacket(p)) {
        return false;
    }

    status.mode = static_cast<robot_base::ControlMode>(p.control_mode);
    status.zero_ready = (p.zero_ready != 0);
    status.hmi_connected = (p.hmi_connected != 0);
    status.vx = p.vx;
    status.vy = p.vy;
    status.wz = p.wz;
    status.rl_frequency_hz = p.rl_frequency_hz;
    status.active_policy = DecodeText(p.active_policy);
    status.interaction.sequence = p.interaction_sequence;
    status.interaction.request_accepted =
        p.interaction_request_accepted != 0;
    status.interaction.phase =
        static_cast<robot_base::InteractionStatus::Phase>(
            p.interaction_phase);
    status.interaction.progress = p.interaction_progress;
    status.interaction.action = DecodeText(p.interaction_action);
    return DecodeFault(p.fault, &fault);
}

}  // namespace transport
