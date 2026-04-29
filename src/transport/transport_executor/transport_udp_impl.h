/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file transport_udp_impl.h
 * @brief UDP 传输实现（内部头文件，不对外暴露）
 */
#ifndef TRANSPORT_UDP_IMPL_H
#define TRANSPORT_UDP_IMPL_H

#include <memory>
#include <string>

#include "transport_executor.h"
#include "transport_packet.h"
#include "udp_transport.h"
namespace transport {

/**
 * @brief UDP 传输实现
 *
 * 内部使用 POD 结构进行序列化，对外只暴露 robot_base 类型。
 * POD 协议定义在 transport_packet.h 中，与 SHM/DDS 共享。
 */
class TransportUdpImpl : public TransportBase {
public:
    TransportUdpImpl();
    ~TransportUdpImpl() override;

    bool Init(const std::string& yaml_path, Role role) override;

    void SendState(const robot_base::RobotData& state) override;
    bool RecvState(robot_base::RobotData& state) override;

    void SendControl(const robot_base::ControlCmd& cmd) override;
    bool RecvControl(robot_base::ControlCmd& cmd) override;

    void SendCommand(const robot_base::Command& cmd) override;
    bool RecvCommand(robot_base::Command& cmd) override;

private:
    // ==================== 成员变量 ====================

    Role role_ = Role::DRIVER;

    // Driver: state_sender_ + control_receiver_
    // Control: state_receiver_ + control_sender_ + hmi_receiver_
    // Hmi: hmi_sender_
    std::unique_ptr<transport_udp::Udp> state_sender_;
    std::unique_ptr<transport_udp::Udp> state_receiver_;
    std::unique_ptr<transport_udp::Udp> control_sender_;
    std::unique_ptr<transport_udp::Udp> control_receiver_;
    std::unique_ptr<transport_udp::Udp> hmi_sender_;
    std::unique_ptr<transport_udp::Udp> hmi_receiver_;

    uint32_t state_seq_ = 0;
    uint32_t control_seq_ = 0;
    uint32_t hmi_seq_ = 0;
};

}  // namespace transport

#endif  // TRANSPORT_UDP_IMPL_H
