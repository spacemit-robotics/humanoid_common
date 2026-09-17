/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file transport_packet.h
 * @brief 传输层公共 POD 协议定义（UDP / SHM / DDS 共用）
 *
 * 定义传输层使用的二进制数据包格式。所有传输实现（UDP、SHM、DDS）
 * 共享相同的序列化格式，保证协议一致性和互操作性。
 *
 * 设计说明：
 * - 使用 #pragma pack(1) 保证内存布局紧凑，可直接 memcpy
 * - 固定数组大小 kMaxDof=64，覆盖所有机器人自由度
 * - PacketHeader 包含 magic/version/type/seq，用于校验和调试
 *
 * 与 robot_base 的关系：
 * - robot_base 使用 std::vector 等动态容器，便于应用层使用
 * - transport_packet 使用固定大小 POD 结构，便于二进制传输
 * - 各传输实现负责两者之间的序列化/反序列化转换
 */
#ifndef TRANSPORT_PACKET_H
#define TRANSPORT_PACKET_H

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iterator>
#include <string>
#include <vector>

#include "robot_base.h"

namespace transport {

// ==================== 协议常量 ====================

static constexpr uint32_t kMagic = 0x484D5253;  ///< "HMRS" 魔数
static constexpr uint16_t kVersion = 6;         ///< 协议版本
static constexpr int kMaxDof = 64;              ///< 最大自由度数
static constexpr std::size_t kPolicyNameLength = 64;
static constexpr std::size_t kInteractionNameLength = 64;
static constexpr std::size_t kFaultDetailLength = 160;

// ==================== 消息类型 ====================

enum class MsgType : uint16_t {
    HMI_CMD = 1,      ///< HMI 命令
    ROBOT_STATE = 2,  ///< 机器人状态
    CONTROL_CMD = 3,  ///< 控制命令
    CONTROL_STATUS = 4,  ///< Control → HMI 运行状态
};

// ==================== 数据包定义 ====================

#pragma pack(push, 1)

/**
 * @brief 数据包头部
 */
struct PacketHeader {
    uint32_t magic = kMagic;
    uint16_t version = kVersion;
    uint16_t type = 0;
    uint32_t seq = 0;
};

/** @brief Fixed-layout representation of robot_base::FaultStatus. */
struct FaultPacket {
    uint64_t sequence = 0;
    double timestamp_s = 0.0;
    int32_t native_code = 0;
    int16_t code = 0;
    uint8_t source = 0;
    uint8_t active = 0;
    uint8_t latched = 0;
    char detail[kFaultDetailLength] = {};
};

/**
 * @brief HMI 命令数据包
 */
struct HmiCmdPacket {
    PacketHeader header{};
    int32_t key = 0;
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    uint64_t acknowledge_fault_sequence = 0;
    char switch_policy[kPolicyNameLength] = {};  ///< 策略切换请求，空字符串表示无切换
    uint64_t interaction_sequence = 0;
    uint8_t interaction_operation = 0;
    char interaction_action[kInteractionNameLength] = {};
};

/**
 * @brief 机器人状态数据包
 */
struct RobotStatePacket {
    PacketHeader header{};
    int32_t num_dof = 0;
    double time = 0.0;
    double rpy[3] = {0.0, 0.0, 0.0};
    double gyro[3] = {0.0, 0.0, 0.0};
    double acceleration[3] = {0.0, 0.0, 0.0};
    // base_pos / base_quat / base_vel 用于 motion tracking 等需要全局位姿/速度的 RL 策略
    // dance/kungfu/motion 这类用 rpy+gyro 就够，这几项可保持默认值
    double base_pos[3] = {0.0, 0.0, 0.0};        ///< 世界系位置 (m)
    double base_quat[4] = {1.0, 0.0, 0.0, 0.0};  ///< 姿态四元数 (w, x, y, z)
    double base_vel[6] = {0.0};                  ///< 线速度+角速度 (m/s, rad/s)
    double joint_pos[kMaxDof] = {0.0};
    double joint_vel[kMaxDof] = {0.0};
    double joint_torque[kMaxDof] = {0.0};
    double joint_temperature[kMaxDof] = {0.0};
    uint32_t joint_error[kMaxDof] = {0};
    FaultPacket fault{};
};

/**
 * @brief 控制命令数据包
 */
struct ControlCmdPacket {
    PacketHeader header{};
    int32_t num_dof = 0;
    uint8_t enable = 0;
    // robot_base::ControlMode: POWER_OFF/DAMP/HOME/ZERO/RL/SAFETY.
    int8_t control_mode = 0;
    // robot_base::ActuationMode: HYBRID/POSITION/VELOCITY/TORQUE.
    int8_t actuation_mode = 0;
    double target_pos[kMaxDof] = {0.0};
    double target_vel[kMaxDof] = {0.0};
    double target_torque[kMaxDof] = {0.0};
    double kp[kMaxDof] = {0.0};
    double kd[kMaxDof] = {0.0};
};

/**
 * @brief Control → HMI 运行状态数据包
 */
struct ControlStatusPacket {
    PacketHeader header{};
    int8_t control_mode = 0;
    uint8_t zero_ready = 0;
    uint8_t hmi_connected = 0;
    uint8_t reserved = 0;
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    float rl_frequency_hz = 0.0f;
    char active_policy[kPolicyNameLength] = {};
    uint64_t interaction_sequence = 0;
    uint8_t interaction_request_accepted = 0;
    uint8_t interaction_phase = 0;
    float interaction_progress = 0.0f;
    char interaction_action[kInteractionNameLength] = {};
    FaultPacket fault{};
};

#pragma pack(pop)

// ==================== 工具函数 ====================

/**
 * @brief 校验数据包头部
 * @param h 数据包头部
 * @param t 期望的消息类型
 * @return 校验通过返回 true
 */
inline bool ValidHeader(const PacketHeader& h, MsgType t) {
    return h.magic == kMagic && h.version == kVersion && h.type == static_cast<uint16_t>(t);
}

template <typename T, std::size_t Size>
inline bool FinitePrefix(const T (&values)[Size], std::size_t count) {
    if (count > Size) return false;
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(static_cast<double>(values[i]))) return false;
    }
    return true;
}

template <typename T, std::size_t Size>
inline bool AllFinite(const T (&values)[Size]) {
    return FinitePrefix(values, Size);
}

template <typename T, std::size_t Size>
inline bool AllFinite(const std::array<T, Size> &values) {
    return std::all_of(values.begin(), values.end(), [](T value) {
        return std::isfinite(static_cast<double>(value));
    });
}

inline bool AllFinite(const std::vector<double> &values) {
    return std::all_of(values.begin(), values.end(),
        [](double value) { return std::isfinite(value); });
}

template <std::size_t Size>
inline bool HasNullTerminator(const char (&text)[Size]) {
    return std::find(std::begin(text), std::end(text), '\0') != std::end(text);
}

template <size_t Size>
inline std::string DecodeText(const char (&source)[Size]) {
    const auto end = std::find(std::begin(source), std::end(source), '\0');
    return std::string(source, end);
}

inline bool ValidControlModeValue(int8_t value) {
    switch (static_cast<robot_base::ControlMode>(value)) {
    case robot_base::ControlMode::POWER_OFF:
    case robot_base::ControlMode::DAMP:
    case robot_base::ControlMode::HOME:
    case robot_base::ControlMode::ZERO:
    case robot_base::ControlMode::RL:
    case robot_base::ControlMode::SAFETY:
        return true;
    }
    return false;
}

inline bool ValidCommandKey(int32_t key) {
    return key == -1 || (key >= 0 && key <= robot_base::kCommandStartReference);
}

inline bool ValidInteractionOperationValue(uint8_t value) {
    using Operation = robot_base::InteractionRequest::Operation;
    switch (static_cast<Operation>(value)) {
    case Operation::NONE:
    case Operation::START:
    case Operation::CANCEL:
        return true;
    }
    return false;
}

inline bool ValidInteractionPhaseValue(uint8_t value) {
    using Phase = robot_base::InteractionStatus::Phase;
    switch (static_cast<Phase>(value)) {
    case Phase::IDLE:
    case Phase::BLEND_IN:
    case Phase::PLAYING:
    case Phase::HOLDING:
    case Phase::BLEND_OUT:
    case Phase::FINISHED:
    case Phase::REJECTED:
        return true;
    }
    return false;
}

inline bool CanEncodeInteractionRequest(
        const robot_base::InteractionRequest &request) {
    if (!ValidInteractionOperationValue(
            static_cast<uint8_t>(request.operation)) ||
        request.action.size() >= kInteractionNameLength ||
        request.action.find('\0') != std::string::npos) {
        return false;
    }
    if (request.operation == robot_base::InteractionRequest::Operation::START) {
        return request.sequence != 0 && !request.action.empty();
    }
    if (request.operation == robot_base::InteractionRequest::Operation::CANCEL) {
        return request.sequence != 0 && request.action.empty();
    }
    return request.action.empty();
}

inline bool CanEncodeInteractionStatus(
        const robot_base::InteractionStatus &status) {
    if (!ValidInteractionPhaseValue(static_cast<uint8_t>(status.phase)) ||
        !std::isfinite(status.progress) || status.progress < 0.0F ||
        status.progress > 1.0F ||
        status.action.size() >= kInteractionNameLength ||
        status.action.find('\0') != std::string::npos) {
        return false;
    }
    if (status.phase == robot_base::InteractionStatus::Phase::IDLE) {
        return status.action.empty();
    }
    return status.sequence != 0 && !status.action.empty();
}

inline bool ValidFaultPacket(const FaultPacket &source) {
    if (source.active > 1 || source.latched > 1) return false;
    if (source.active != 0 && source.latched == 0) return false;
    if (!std::isfinite(source.timestamp_s) || source.timestamp_s < 0.0 ||
        !HasNullTerminator(source.detail)) {
        return false;
    }
    const auto fault_source = static_cast<robot_base::FaultSource>(source.source);
    const auto fault_code = static_cast<robot_base::FaultCode>(source.code);
    if (!robot_base::IsValidFaultSource(fault_source) ||
        !robot_base::IsValidFaultCode(fault_code)) {
        return false;
    }
    if (source.latched == 0) {
        return fault_source == robot_base::FaultSource::NONE &&
            fault_code == robot_base::FaultCode::NONE && source.native_code == 0 &&
            source.sequence == 0 && source.timestamp_s == 0.0 && source.detail[0] == '\0';
    }
    return fault_source != robot_base::FaultSource::NONE &&
        fault_code != robot_base::FaultCode::NONE && source.sequence != 0;
}

inline bool CanEncodeFault(const robot_base::FaultStatus &fault) {
    if (!robot_base::IsValidFaultSource(fault.source) ||
        !robot_base::IsValidFaultCode(fault.code) || !std::isfinite(fault.timestamp_s) ||
        fault.timestamp_s < 0.0 || (fault.active && !fault.latched) ||
        fault.detail.find('\0') != std::string::npos) {
        return false;
    }
    if (!fault.latched) {
        return fault.source == robot_base::FaultSource::NONE &&
            fault.code == robot_base::FaultCode::NONE && fault.native_code == 0 &&
            fault.sequence == 0 && fault.timestamp_s == 0.0 && fault.detail.empty();
    }
    return fault.source != robot_base::FaultSource::NONE &&
        fault.code != robot_base::FaultCode::NONE && fault.sequence != 0;
}

inline std::string FaultDetailForPacket(const std::string &detail) {
    constexpr char kTruncationMarker[] = "...[truncated]";
    constexpr std::size_t kMaximumSize = kFaultDetailLength - 1;
    constexpr std::size_t kMarkerSize = sizeof(kTruncationMarker) - 1;
    static_assert(kMarkerSize < kMaximumSize);
    if (detail.size() <= kMaximumSize) return detail;

    std::size_t prefix_size = kMaximumSize - kMarkerSize;
    while (prefix_size > 0 &&
        (static_cast<unsigned char>(detail[prefix_size]) & 0xc0U) == 0x80U) {
        --prefix_size;
    }
    return detail.substr(0, prefix_size) + kTruncationMarker;
}

inline bool CanEncodeRobotState(const robot_base::RobotData &state,
        const robot_base::FaultStatus &fault) {
    if (state.num_dof <= 0 || state.num_dof > kMaxDof ||
        state.joint_pos.size() != static_cast<std::size_t>(state.num_dof) ||
        state.joint_vel.size() != static_cast<std::size_t>(state.num_dof) ||
        (!state.joint_torque.empty() &&
            state.joint_torque.size() != static_cast<std::size_t>(state.num_dof)) ||
        (!state.joint_temperature.empty() &&
            state.joint_temperature.size() != static_cast<std::size_t>(state.num_dof)) ||
        (!state.joint_error.empty() &&
            state.joint_error.size() != static_cast<std::size_t>(state.num_dof)) ||
        !std::isfinite(state.time) || state.time < 0.0 || !AllFinite(state.rpy) ||
        !AllFinite(state.gyro) || !AllFinite(state.acceleration) ||
        !AllFinite(state.base_pos) || !AllFinite(state.base_quat) ||
        !AllFinite(state.base_vel) || !AllFinite(state.joint_pos) ||
        !AllFinite(state.joint_vel) || !AllFinite(state.joint_torque) ||
        !AllFinite(state.joint_temperature) || !CanEncodeFault(fault)) {
        return false;
    }
    double quaternion_norm_squared = 0.0;
    for (double value : state.base_quat) quaternion_norm_squared += value * value;
    return quaternion_norm_squared > 1.0e-12;
}

inline bool CanEncodeControl(const robot_base::ControlCmd &command) {
    const std::size_t num_dof = command.target_pos.size();
    if (num_dof == 0 || num_dof > static_cast<std::size_t>(kMaxDof) ||
        command.target_vel.size() != num_dof || command.kp.size() != num_dof ||
        command.kd.size() != num_dof ||
        (!command.target_torque.empty() && command.target_torque.size() != num_dof) ||
        !ValidControlModeValue(static_cast<int8_t>(command.mode)) ||
        !robot_base::IsValidActuationMode(command.actuation_mode) ||
        !AllFinite(command.target_pos) || !AllFinite(command.target_vel) ||
        !AllFinite(command.target_torque) || !AllFinite(command.kp) ||
        !AllFinite(command.kd)) {
        return false;
    }
    for (std::size_t i = 0; i < num_dof; ++i) {
        if (command.kp[i] < 0.0 || command.kd[i] < 0.0) return false;
    }
    return true;
}

inline bool CanEncodeCommand(const robot_base::Command &command) {
    return ValidCommandKey(command.key) && std::isfinite(command.vx) &&
        std::isfinite(command.vy) && std::isfinite(command.wz) &&
        command.switch_policy.size() < kPolicyNameLength &&
        command.switch_policy.find('\0') == std::string::npos &&
        CanEncodeInteractionRequest(command.interaction);
}

inline bool CanEncodeStatus(const robot_base::ControlStatus &status,
        const robot_base::FaultStatus &fault) {
    return ValidControlModeValue(static_cast<int8_t>(status.mode)) &&
        std::isfinite(status.vx) && std::isfinite(status.vy) &&
        std::isfinite(status.wz) && std::isfinite(status.rl_frequency_hz) &&
        status.rl_frequency_hz >= 0.0f &&
        status.active_policy.size() < kPolicyNameLength &&
        status.active_policy.find('\0') == std::string::npos &&
        CanEncodeInteractionStatus(status.interaction) &&
        CanEncodeFault(fault);
}

inline bool ValidHmiCmdPacket(const HmiCmdPacket &packet) {
    return ValidHeader(packet.header, MsgType::HMI_CMD) &&
        ValidCommandKey(packet.key) && std::isfinite(packet.vx) &&
        std::isfinite(packet.vy) && std::isfinite(packet.wz) &&
        HasNullTerminator(packet.switch_policy) &&
        ValidInteractionOperationValue(packet.interaction_operation) &&
        HasNullTerminator(packet.interaction_action) &&
        CanEncodeInteractionRequest({
            packet.interaction_sequence,
            static_cast<robot_base::InteractionRequest::Operation>(
                packet.interaction_operation),
            DecodeText(packet.interaction_action)});
}

inline bool ValidRobotStatePacket(const RobotStatePacket &packet) {
    if (!ValidHeader(packet.header, MsgType::ROBOT_STATE) ||
        packet.num_dof <= 0 || packet.num_dof > kMaxDof ||
        !std::isfinite(packet.time) || packet.time < 0.0 ||
        !AllFinite(packet.rpy) || !AllFinite(packet.gyro) ||
        !AllFinite(packet.acceleration) || !AllFinite(packet.base_pos) ||
        !AllFinite(packet.base_quat) || !AllFinite(packet.base_vel) ||
        !FinitePrefix(packet.joint_pos, packet.num_dof) ||
        !FinitePrefix(packet.joint_vel, packet.num_dof) ||
        !FinitePrefix(packet.joint_torque, packet.num_dof) ||
        !FinitePrefix(packet.joint_temperature, packet.num_dof) ||
        !ValidFaultPacket(packet.fault)) {
        return false;
    }
    double quaternion_norm_squared = 0.0;
    for (double value : packet.base_quat)
        quaternion_norm_squared += value * value;
    return quaternion_norm_squared > 1.0e-12;
}

inline bool ValidControlCmdPacket(const ControlCmdPacket &packet) {
    if (!ValidHeader(packet.header, MsgType::CONTROL_CMD) ||
        packet.num_dof <= 0 || packet.num_dof > kMaxDof || packet.enable > 1 ||
        !ValidControlModeValue(packet.control_mode) ||
        !robot_base::IsValidActuationMode(
            static_cast<robot_base::ActuationMode>(packet.actuation_mode)) ||
        !FinitePrefix(packet.target_pos, packet.num_dof) ||
        !FinitePrefix(packet.target_vel, packet.num_dof) ||
        !FinitePrefix(packet.target_torque, packet.num_dof) ||
        !FinitePrefix(packet.kp, packet.num_dof) ||
        !FinitePrefix(packet.kd, packet.num_dof)) {
        return false;
    }
    for (int i = 0; i < packet.num_dof; ++i) {
        if (packet.kp[i] < 0.0 || packet.kd[i] < 0.0) return false;
    }
    return true;
}

inline bool ValidControlStatusPacket(const ControlStatusPacket &packet) {
    return ValidHeader(packet.header, MsgType::CONTROL_STATUS) &&
        ValidControlModeValue(packet.control_mode) && packet.zero_ready <= 1 &&
        packet.hmi_connected <= 1 && packet.reserved == 0 &&
        std::isfinite(packet.vx) && std::isfinite(packet.vy) &&
        std::isfinite(packet.wz) && std::isfinite(packet.rl_frequency_hz) &&
        packet.rl_frequency_hz >= 0.0f && HasNullTerminator(packet.active_policy) &&
        packet.interaction_request_accepted <= 1 &&
        ValidInteractionPhaseValue(packet.interaction_phase) &&
        std::isfinite(packet.interaction_progress) &&
        HasNullTerminator(packet.interaction_action) &&
        CanEncodeInteractionStatus({
            packet.interaction_sequence,
            packet.interaction_request_accepted != 0,
            static_cast<robot_base::InteractionStatus::Phase>(
                packet.interaction_phase),
            packet.interaction_progress,
            DecodeText(packet.interaction_action)}) &&
        ValidFaultPacket(packet.fault);
}

inline void EncodeFault(
        const robot_base::FaultStatus &source, FaultPacket *destination) {
    if (!destination) return;
    *destination = {};
    destination->active = source.active ? 1 : 0;
    destination->latched = source.latched ? 1 : 0;
    destination->source = static_cast<uint8_t>(source.source);
    destination->code = static_cast<int16_t>(source.code);
    destination->native_code = source.native_code;
    destination->sequence = source.sequence;
    destination->timestamp_s = source.timestamp_s;
    const std::string detail = FaultDetailForPacket(source.detail);
    std::memcpy(destination->detail, detail.data(), detail.size());
}

inline bool DecodeFault(
        const FaultPacket &source, robot_base::FaultStatus *destination) {
    if (!destination) return false;
    if (!ValidFaultPacket(source)) return false;
    const auto fault_source = static_cast<robot_base::FaultSource>(source.source);
    const auto fault_code = static_cast<robot_base::FaultCode>(source.code);
    destination->active = source.active != 0;
    destination->latched = source.latched != 0;
    destination->source = fault_source;
    destination->code = fault_code;
    destination->native_code = source.native_code;
    destination->sequence = source.sequence;
    destination->timestamp_s = source.timestamp_s;
    const auto detail_end = std::find(
        std::begin(source.detail), std::end(source.detail), '\0');
    destination->detail.assign(source.detail, detail_end);
    return true;
}

}  // namespace transport

#endif  // TRANSPORT_PACKET_H
