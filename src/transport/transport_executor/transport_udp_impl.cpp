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
    int state_port = 8800;
    int control_port = 8801;
    int hmi_port = 8802;

    const std::string udp_prefix = "transport.udp";
    const std::string legacy_ip = yaml_file.Read<std::string>(udp_prefix + ".ip").value_or("");
    driver_ip = yaml_file.Read<std::string>(udp_prefix + ".driver_ip")
                    .value_or(!legacy_ip.empty() ? legacy_ip : driver_ip);
    control_ip = yaml_file.Read<std::string>(udp_prefix + ".control_ip")
                    .value_or(!legacy_ip.empty() ? legacy_ip : control_ip);
    state_port = yaml_file.Read<int>(udp_prefix + ".state_port").value_or(state_port);
    control_port = yaml_file.Read<int>(udp_prefix + ".control_port").value_or(control_port);
    hmi_port = yaml_file.Read<int>(udp_prefix + ".hmi_port").value_or(control_port + 1);

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
    }

    return true;
}

// ==================== 状态通道 ====================

void TransportUdpImpl::SendState(const robot_base::RobotData& state) {
    if (!state_sender_)
        return;

    RobotStatePacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::ROBOT_STATE);
    p.header.seq = ++state_seq_;
    p.num_dof = std::clamp(state.num_dof, 0, kMaxDof);
    p.time = state.time;
    std::memcpy(p.rpy, state.rpy.data(), sizeof(p.rpy));
    std::memcpy(p.gyro, state.gyro.data(), sizeof(p.gyro));
    std::memcpy(p.base_pos, state.base_pos.data(), sizeof(p.base_pos));
    std::memcpy(p.base_quat, state.base_quat.data(), sizeof(p.base_quat));
    std::memcpy(p.base_vel, state.base_vel.data(), sizeof(p.base_vel));

    for (int i = 0; i < p.num_dof; ++i) {
        p.joint_pos[i] = (i < static_cast<int>(state.joint_pos.size())) ? state.joint_pos[i] : 0.0;
        p.joint_vel[i] = (i < static_cast<int>(state.joint_vel.size())) ? state.joint_vel[i] : 0.0;
    }

    state_sender_->Send(&p, sizeof(p));
}

bool TransportUdpImpl::RecvState(robot_base::RobotData& state) {
    if (!state_receiver_)
        return false;

    RobotStatePacket p{};
    int n = state_receiver_->Recv(&p, sizeof(p));
    if (n != static_cast<int>(sizeof(p)) || !ValidHeader(p.header, MsgType::ROBOT_STATE))
        return false;

    state.num_dof = std::clamp(p.num_dof, 0, kMaxDof);
    state.InitJointVectors();
    state.time = p.time;
    std::memcpy(state.rpy.data(), p.rpy, sizeof(p.rpy));
    std::memcpy(state.gyro.data(), p.gyro, sizeof(p.gyro));
    std::memcpy(state.base_pos.data(), p.base_pos, sizeof(p.base_pos));
    std::memcpy(state.base_quat.data(), p.base_quat, sizeof(p.base_quat));
    std::memcpy(state.base_vel.data(), p.base_vel, sizeof(p.base_vel));

    for (int i = 0; i < state.num_dof; ++i) {
        state.joint_pos[i] = p.joint_pos[i];
        state.joint_vel[i] = p.joint_vel[i];
    }

    return true;
}

// ==================== 控制通道 ====================

void TransportUdpImpl::SendControl(const robot_base::ControlCmd& cmd) {
    if (!control_sender_)
        return;

    ControlCmdPacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::CONTROL_CMD);
    p.header.seq = ++control_seq_;
    p.num_dof = std::clamp(static_cast<int>(cmd.target_pos.size()), 0, kMaxDof);
    p.enable = cmd.enable ? 1 : 0;
    p.control_mode = static_cast<int8_t>(cmd.mode);

    for (int i = 0; i < p.num_dof; ++i) {
        p.target_pos[i] = cmd.target_pos[i];
        p.target_vel[i] = (i < static_cast<int>(cmd.target_vel.size())) ? cmd.target_vel[i] : 0.0;
        p.kp[i] = (i < static_cast<int>(cmd.kp.size())) ? cmd.kp[i] : 0.0;
        p.kd[i] = (i < static_cast<int>(cmd.kd.size())) ? cmd.kd[i] : 0.0;
    }

    control_sender_->Send(&p, sizeof(p));
}

bool TransportUdpImpl::RecvControl(robot_base::ControlCmd& cmd) {
    if (!control_receiver_)
        return false;

    ControlCmdPacket p{};
    int n = control_receiver_->Recv(&p, sizeof(p));
    if (n != static_cast<int>(sizeof(p)) || !ValidHeader(p.header, MsgType::CONTROL_CMD))
        return false;

    int ndof = std::clamp(p.num_dof, 0, kMaxDof);
    cmd.enable = (p.enable != 0);
    cmd.mode = static_cast<robot_base::ControlMode>(p.control_mode);
    cmd.target_pos.resize(ndof);
    cmd.target_vel.resize(ndof);
    cmd.kp.resize(ndof);
    cmd.kd.resize(ndof);

    for (int i = 0; i < ndof; ++i) {
        cmd.target_pos[i] = p.target_pos[i];
        cmd.target_vel[i] = p.target_vel[i];
        cmd.kp[i] = p.kp[i];
        cmd.kd[i] = p.kd[i];
    }

    return true;
}

// ==================== 命令通道 ====================

void TransportUdpImpl::SendCommand(const robot_base::Command& cmd) {
    if (!hmi_sender_)
        return;

    HmiCmdPacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::HMI_CMD);
    p.header.seq = ++hmi_seq_;
    p.key = cmd.key;
    p.vx = cmd.vx;
    p.vy = cmd.vy;
    p.wz = cmd.wz;
    std::strncpy(p.switch_policy, cmd.switch_policy.c_str(), sizeof(p.switch_policy) - 1);

    hmi_sender_->Send(&p, sizeof(p));
}

bool TransportUdpImpl::RecvCommand(robot_base::Command& cmd) {
    if (!hmi_receiver_)
        return false;

    HmiCmdPacket p{};
    int n = hmi_receiver_->Recv(&p, sizeof(p));
    if (n != static_cast<int>(sizeof(p)) || !ValidHeader(p.header, MsgType::HMI_CMD))
        return false;

    cmd.key = p.key;
    cmd.vx = p.vx;
    cmd.vy = p.vy;
    cmd.wz = p.wz;
    cmd.switch_policy = p.switch_policy;

    return true;
}

}  // namespace transport
