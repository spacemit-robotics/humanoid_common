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

#include <cstdint>

namespace transport {

// ==================== 协议常量 ====================

static constexpr uint32_t kMagic = 0x484D5253;  ///< "HMRS" 魔数
static constexpr uint16_t kVersion = 1;         ///< 协议版本
static constexpr int kMaxDof = 64;              ///< 最大自由度数

// ==================== 消息类型 ====================

enum class MsgType : uint16_t {
    HMI_CMD = 1,      ///< HMI 命令
    ROBOT_STATE = 2,  ///< 机器人状态
    CONTROL_CMD = 3,  ///< 控制命令
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

/**
 * @brief HMI 命令数据包
 */
struct HmiCmdPacket {
    PacketHeader header{};
    int32_t key = 0;
    float vx = 0.0f;
    float vy = 0.0f;
    float wz = 0.0f;
    char switch_policy[64] = {};  ///< 策略切换请求，空字符串表示无切换
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
    // base_pos / base_quat / base_vel 用于 BeyondMimic 风格 tracking 等需要全局位姿/速度的 RL 策略
    // dance/kungfu/motion 这类用 rpy+gyro 就够，这几项可保持默认值
    double base_pos[3] = {0.0, 0.0, 0.0};        ///< 世界系位置 (m)
    double base_quat[4] = {1.0, 0.0, 0.0, 0.0};  ///< 姿态四元数 (w, x, y, z)
    double base_vel[6] = {0.0};                  ///< 线速度+角速度 (m/s, rad/s)
    double joint_pos[kMaxDof] = {0.0};
    double joint_vel[kMaxDof] = {0.0};
};

/**
 * @brief 控制命令数据包
 */
struct ControlCmdPacket {
    PacketHeader header{};
    int32_t num_dof = 0;
    uint8_t enable = 0;
    int8_t control_mode = 0;  // robot_base::ControlMode（POWER_OFF=0/DAMP=1/ZERO=2/RL=3/SAFETY=4）
    double target_pos[kMaxDof] = {0.0};
    double target_vel[kMaxDof] = {0.0};
    double kp[kMaxDof] = {0.0};
    double kd[kMaxDof] = {0.0};
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

}  // namespace transport

#endif  // TRANSPORT_PACKET_H
