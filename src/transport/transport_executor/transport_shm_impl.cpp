/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file transport_shm_impl.cpp
 * @brief SHM 传输实现
 *
 * 将 robot_base 类型与 POD 数据包之间的序列化/反序列化封装在内部。
 */

#include "transport_shm_impl.h"

#include <algorithm>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>

namespace transport {

TransportShmImpl::TransportShmImpl() = default;
TransportShmImpl::~TransportShmImpl() = default;

bool TransportShmImpl::Init(const std::string& yaml_path, Role role) {
    role_ = role;

    // 读取传输配置
    robot_base::YamlFile yaml_file = robot_base::YamlFile::Load(yaml_path);
    std::string prefix = "robot_name";
    uint32_t capacity = 4;
    uint32_t slot_size = 4096;

    const std::string shm_prefix = "transport.shm";
    prefix = yaml_file.Read<std::string>(shm_prefix + ".prefix").value_or(prefix);
    capacity =
        static_cast<uint32_t>(yaml_file.Read<int>(shm_prefix + ".capacity").value_or(capacity));
    slot_size =
        static_cast<uint32_t>(yaml_file.Read<int>(shm_prefix + ".slot_size").value_or(slot_size));

    if (role == Role::DRIVER) {
        // Driver: 写状态
        state_writer_ = std::make_unique<transport_shm::Shm>();
        transport_shm::ShmConfig sc;
        sc.channel_name = "/hmrs_" + prefix + "_state";
        sc.role = transport_shm::Role::WRITER;
        sc.capacity = capacity;
        sc.slot_size = slot_size;
        if (!state_writer_->Init(sc)) {
            std::cerr << "[transport_shm] state_writer 初始化失败\n";
            return false;
        }

        // Driver: 读控制
        control_reader_ = std::make_unique<transport_shm::Shm>();
        transport_shm::ShmConfig cc;
        cc.channel_name = "/hmrs_" + prefix + "_control";
        cc.role = transport_shm::Role::READER;
        cc.capacity = capacity;
        cc.slot_size = slot_size;
        cc.create_if_not_exist = true;  // Reader 也可创建，支持任意启动顺序
        if (!control_reader_->Init(cc)) {
            std::cerr << "[transport_shm] control_reader 初始化失败\n";
            return false;
        }

    } else if (role == Role::CONTROL) {
        // Control: 读状态
        state_reader_ = std::make_unique<transport_shm::Shm>();
        transport_shm::ShmConfig sc;
        sc.channel_name = "/hmrs_" + prefix + "_state";
        sc.role = transport_shm::Role::READER;
        sc.capacity = capacity;
        sc.slot_size = slot_size;
        sc.create_if_not_exist = true;  // Reader 也可创建，支持任意启动顺序
        if (!state_reader_->Init(sc)) {
            std::cerr << "[transport_shm] state_reader 初始化失败\n";
            return false;
        }

        // Control: 写控制
        control_writer_ = std::make_unique<transport_shm::Shm>();
        transport_shm::ShmConfig cc;
        cc.channel_name = "/hmrs_" + prefix + "_control";
        cc.role = transport_shm::Role::WRITER;
        cc.capacity = capacity;
        cc.slot_size = slot_size;
        if (!control_writer_->Init(cc)) {
            std::cerr << "[transport_shm] control_writer 初始化失败\n";
            return false;
        }

        // Control: 读 HMI
        hmi_reader_ = std::make_unique<transport_shm::Shm>();
        transport_shm::ShmConfig hc;
        hc.channel_name = "/hmrs_" + prefix + "_hmi";
        hc.role = transport_shm::Role::READER;
        hc.capacity = capacity;
        hc.slot_size = slot_size;
        hc.create_if_not_exist = true;  // Reader 也可创建，支持任意启动顺序
        if (!hmi_reader_->Init(hc)) {
            std::cerr << "[transport_shm] hmi_reader 初始化失败\n";
            return false;
        }

        // Control: 写运行状态
        status_writer_ = std::make_unique<transport_shm::Shm>();
        transport_shm::ShmConfig ss;
        ss.channel_name = "/hmrs_" + prefix + "_status";
        ss.role = transport_shm::Role::WRITER;
        ss.capacity = capacity;
        ss.slot_size = slot_size;
        if (!status_writer_->Init(ss)) {
            std::cerr << "[transport_shm] status_writer 初始化失败\n";
            return false;
        }

    } else if (role == Role::HMI) {
        // Hmi: 写 HMI
        hmi_writer_ = std::make_unique<transport_shm::Shm>();
        transport_shm::ShmConfig hc;
        hc.channel_name = "/hmrs_" + prefix + "_hmi";
        hc.role = transport_shm::Role::WRITER;
        hc.capacity = capacity;
        hc.slot_size = slot_size;
        if (!hmi_writer_->Init(hc)) {
            std::cerr << "[transport_shm] hmi_writer 初始化失败\n";
            return false;
        }

        // Hmi: 读 Control 运行状态
        status_reader_ = std::make_unique<transport_shm::Shm>();
        transport_shm::ShmConfig sr;
        sr.channel_name = "/hmrs_" + prefix + "_status";
        sr.role = transport_shm::Role::READER;
        sr.capacity = capacity;
        sr.slot_size = slot_size;
        sr.create_if_not_exist = true;
        if (!status_reader_->Init(sr)) {
            std::cerr << "[transport_shm] status_reader 初始化失败\n";
            return false;
        }
    }

    return true;
}

// ==================== 状态通道 ====================

void TransportShmImpl::SendState(const robot_base::RobotData& state) {
    if (!state_writer_)
        return;

    RobotStatePacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::ROBOT_STATE);
    p.header.seq = ++state_seq_;
    p.num_dof = std::clamp(state.num_dof, 0, kMaxDof);
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

    state_writer_->Write(&p, sizeof(p));
}

bool TransportShmImpl::RecvState(robot_base::RobotData& state) {
    if (!state_reader_)
        return false;

    RobotStatePacket p{};
    std::size_t actual_len = 0;
    if (!state_reader_->Read(&p, sizeof(p), actual_len))
        return false;
    if (actual_len != sizeof(p) || !ValidHeader(p.header, MsgType::ROBOT_STATE))
        return false;

    state.num_dof = std::clamp(p.num_dof, 0, kMaxDof);
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

    return true;
}

// ==================== 控制通道 ====================

void TransportShmImpl::SendControl(const robot_base::ControlCmd& cmd) {
    if (!control_writer_)
        return;

    ControlCmdPacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::CONTROL_CMD);
    p.header.seq = ++control_seq_;
    p.num_dof = std::clamp(static_cast<int>(cmd.target_pos.size()), 0, kMaxDof);
    p.enable = cmd.enable ? 1 : 0;
    p.control_mode = static_cast<int8_t>(cmd.mode);
    p.actuation_mode = static_cast<int8_t>(cmd.actuation_mode);

    for (int i = 0; i < p.num_dof; ++i) {
        p.target_pos[i] = cmd.target_pos[i];
        p.target_vel[i] = (i < static_cast<int>(cmd.target_vel.size())) ? cmd.target_vel[i] : 0.0;
        p.target_torque[i] = (i < static_cast<int>(cmd.target_torque.size()))
            ? cmd.target_torque[i]
            : 0.0;
        p.kp[i] = (i < static_cast<int>(cmd.kp.size())) ? cmd.kp[i] : 0.0;
        p.kd[i] = (i < static_cast<int>(cmd.kd.size())) ? cmd.kd[i] : 0.0;
    }

    control_writer_->Write(&p, sizeof(p));
}

bool TransportShmImpl::RecvControl(robot_base::ControlCmd& cmd) {
    if (!control_reader_)
        return false;

    ControlCmdPacket p{};
    std::size_t actual_len = 0;
    if (!control_reader_->Read(&p, sizeof(p), actual_len))
        return false;
    if (actual_len != sizeof(p) || !ValidHeader(p.header, MsgType::CONTROL_CMD))
        return false;

    int ndof = std::clamp(p.num_dof, 0, kMaxDof);
    const auto actuation_mode = static_cast<robot_base::ActuationMode>(p.actuation_mode);
    if (!robot_base::IsValidActuationMode(actuation_mode)) return false;
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

void TransportShmImpl::SendCommand(const robot_base::Command& cmd) {
    if (!hmi_writer_)
        return;

    HmiCmdPacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::HMI_CMD);
    p.header.seq = ++hmi_seq_;
    p.key = cmd.key;
    p.vx = cmd.vx;
    p.vy = cmd.vy;
    p.wz = cmd.wz;
    std::strncpy(p.switch_policy, cmd.switch_policy.c_str(), 63);
    p.switch_policy[63] = '\0';

    hmi_writer_->Write(&p, sizeof(p));
}

bool TransportShmImpl::RecvCommand(robot_base::Command& cmd) {
    if (!hmi_reader_)
        return false;

    HmiCmdPacket p{};
    std::size_t actual_len = 0;
    if (!hmi_reader_->Read(&p, sizeof(p), actual_len))
        return false;
    if (actual_len != sizeof(p) || !ValidHeader(p.header, MsgType::HMI_CMD))
        return false;

    cmd.key = p.key;
    cmd.vx = p.vx;
    cmd.vy = p.vy;
    cmd.wz = p.wz;
    cmd.switch_policy = p.switch_policy;

    return true;
}

// ==================== Control 状态回传通道 ====================

void TransportShmImpl::SendStatus(const robot_base::ControlStatus& status) {
    if (!status_writer_)
        return;

    ControlStatusPacket p{};
    p.header.type = static_cast<uint16_t>(MsgType::CONTROL_STATUS);
    p.header.seq = ++status_seq_;
    p.control_mode = static_cast<int8_t>(status.mode);
    p.zero_ready = status.zero_ready ? 1 : 0;
    p.hmi_connected = status.hmi_connected ? 1 : 0;
    p.vx = status.vx;
    p.vy = status.vy;
    p.wz = status.wz;
    p.rl_frequency_hz = status.rl_frequency_hz;
    std::strncpy(p.active_policy, status.active_policy.c_str(),
        sizeof(p.active_policy) - 1);

    status_writer_->Write(&p, sizeof(p));
}

bool TransportShmImpl::RecvStatus(robot_base::ControlStatus& status) {
    if (!status_reader_)
        return false;

    ControlStatusPacket p{};
    std::size_t actual_len = 0;
    if (!status_reader_->Read(&p, sizeof(p), actual_len))
        return false;
    if (actual_len != sizeof(p) ||
        !ValidHeader(p.header, MsgType::CONTROL_STATUS)) {
        return false;
    }

    status.mode = static_cast<robot_base::ControlMode>(p.control_mode);
    status.zero_ready = (p.zero_ready != 0);
    status.hmi_connected = (p.hmi_connected != 0);
    status.vx = p.vx;
    status.vy = p.vy;
    status.wz = p.wz;
    status.rl_frequency_hz = p.rl_frequency_hz;
    status.active_policy = p.active_policy;
    return true;
}

}  // namespace transport
