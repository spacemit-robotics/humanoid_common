/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file transport_shm_impl.h
 * @brief SHM 传输实现（内部头文件，不对外暴露）
 */
#ifndef TRANSPORT_SHM_IMPL_H
#define TRANSPORT_SHM_IMPL_H

#include <memory>
#include <string>

#include "transport_executor.h"
#include "transport_packet.h"
#include "shm_transport.h"
namespace transport {

/**
 * @brief SHM 传输实现
 *
 * 使用 POSIX 共享内存 + 无锁环形缓冲区实现进程间通信。
 * POD 协议定义在 transport_packet.h 中，与 UDP 共享。
 */
class TransportShmImpl : public TransportBase {
public:
    TransportShmImpl();
    ~TransportShmImpl() override;

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

    // Driver: state_writer_ + control_reader_
    // Control: state_reader_ + control_writer_ + hmi_reader_
    // Hmi: hmi_writer_
    std::unique_ptr<transport_shm::Shm> state_writer_;
    std::unique_ptr<transport_shm::Shm> state_reader_;
    std::unique_ptr<transport_shm::Shm> control_writer_;
    std::unique_ptr<transport_shm::Shm> control_reader_;
    std::unique_ptr<transport_shm::Shm> hmi_writer_;
    std::unique_ptr<transport_shm::Shm> hmi_reader_;

    uint32_t state_seq_ = 0;
    uint32_t control_seq_ = 0;
    uint32_t hmi_seq_ = 0;
};

}  // namespace transport

#endif  // TRANSPORT_SHM_IMPL_H
