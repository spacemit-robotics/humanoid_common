/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file transport_executor.h
 * @brief 统一传输接口
 *
 * 提供基于 robot_base 类型的高层传输接口，支持多种传输方式（UDP、SHM、DDS 等）。
 * application 层通过工厂函数创建实例，通过 YAML 配置切换传输方式，无需修改代码。
 */
#ifndef TRANSPORT_EXECUTOR_H
#define TRANSPORT_EXECUTOR_H

#include <memory>
#include <string>

#include "robot_base.h"
namespace transport {

/**
 * @brief 传输角色
 */
enum class Role {
    DRIVER,   ///< 驱动端（发送状态，接收控制）
    CONTROL,  ///< 控制端（接收状态，发送控制，接收 HMI）
    HMI       ///< HMI 端（发送 HMI 命令）
};

/**
 * @brief 传输接口基类
 *
 * 定义统一的传输接口，各实现类（UDP、SHM、DDS）继承此接口。
 */
class TransportBase {
public:
    virtual ~TransportBase() = default;

    /**
     * @brief 初始化传输
     * @param yaml_path 配置文件路径
     * @param role 传输角色
     * @return 成功返回 true
     */
    virtual bool Init(const std::string& yaml_path, Role role) = 0;

    // ==================== 状态通道 ====================

    /**
     * @brief 发送机器人状态
     * @param state 机器人状态
     */
    virtual void SendState(const robot_base::RobotData& state) = 0;

    /**
     * @brief 接收机器人状态
     * @param state 输出：机器人状态
     * @return 成功接收返回 true
     */
    virtual bool RecvState(robot_base::RobotData& state) = 0;

    // ==================== 控制通道 ====================

    /**
     * @brief 发送控制命令
     * @param cmd 控制命令
     */
    virtual void SendControl(const robot_base::ControlCmd& cmd) = 0;

    /**
     * @brief 接收控制命令
     * @param cmd 输出：控制命令
     * @return 成功接收返回 true
     */
    virtual bool RecvControl(robot_base::ControlCmd& cmd) = 0;

    // ==================== 命令通道 ====================

    /**
     * @brief 发送命令
     * @param cmd 命令
     */
    virtual void SendCommand(const robot_base::Command& cmd) = 0;

    /**
     * @brief 接收命令
     * @param cmd 输出：命令
     * @return 成功接收返回 true
     */
    virtual bool RecvCommand(robot_base::Command& cmd) = 0;
};

/**
 * @brief 工厂函数：根据 YAML 配置创建传输实例
 *
 * 读取 YAML 中 transport.type 字段，创建对应的传输实现。
 * 支持的类型：udp、shm、dds（未来）
 *
 * @param yaml_path 配置文件路径
 * @return 传输实例（智能指针）
 * @throws std::runtime_error 如果类型不支持或配置错误
 */
std::unique_ptr<TransportBase> Create(const std::string& yaml_path);

}  // namespace transport

#endif  // TRANSPORT_EXECUTOR_H
