/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file udp_transport.h
 * @brief UDP 通信模块
 *
 * 提供简单的 UDP 字节流收发接口，与机器人数据格式解耦。
 * application 层负责定义具体的数据包格式和语义。
 */
#ifndef UDP_TRANSPORT_H
#define UDP_TRANSPORT_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace transport_udp {

/**
 * @brief UDP 通信角色
 */
enum class Role {
    SERVER,  ///< 监听本地端口，接收对端数据
    CLIENT   ///< 主动向对端发送数据
};

/**
 * @brief UDP 通信配置
 */
struct UdpConfig {
    std::string local_ip = "0.0.0.0";  ///< 本地绑定 IP（Server 使用）
    int local_port = 0;                ///< 本地端口（Server 必填）

    std::string remote_ip = "127.0.0.1";  ///< 对端 IP
    int remote_port = 0;                  ///< 对端端口（Client 必填）

    Role role = Role::SERVER;  ///< 通信角色

    int timeout_ms = 10;  ///< 接收超时（毫秒）
};

/**
 * @brief UDP 通信类
 *
 * 只负责字节流收发，由 application 层决定数据包格式和语义。
 */
class Udp {
public:
    Udp();
    ~Udp();

    Udp(const Udp &) = delete;
    Udp &operator=(const Udp &) = delete;
    Udp(Udp &&) noexcept;
    Udp &operator=(Udp &&) noexcept;

    /**
     * @brief 初始化 UDP 通信
     * @param cfg UDP 配置
     * @return 成功返回 true，失败返回 false
     */
    bool Init(const UdpConfig &cfg);

    /**
     * @brief 发送数据
     * @param data 数据指针
     * @param len 数据长度
     * @return 实际发送的字节数，失败返回负数
     */
    int Send(const void *data, std::size_t len);

    /**
     * @brief 接收数据
     * @param data 接收缓冲区
     * @param max_len 缓冲区最大长度
     * @return 实际接收的字节数，超时返回 0，失败返回负数
     */
    int Recv(void *data, std::size_t max_len);

    /**
     * @brief 查询连接状态
     * @return 已初始化返回 true
     */
    bool IsConnected() const;

    /**
     * @brief 关闭连接
     */
    void Close();

    /**
     * @brief 获取配置
     * @return UDP 配置引用
     */
    const UdpConfig &GetConfig() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace transport_udp

#endif  // UDP_TRANSPORT_H
