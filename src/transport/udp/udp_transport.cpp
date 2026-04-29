/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file udp_transport.cpp
 * @brief UDP 通信实现
 */

#include "udp_transport.h"

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <atomic>
#include <iostream>
#include <memory>
namespace transport_udp {

class Udp::Impl {
public:
    UdpConfig cfg;
    int sockfd = -1;
    std::atomic<bool> connected{false};
    sockaddr_in local_addr{};
    sockaddr_in remote_addr{};
    socklen_t addr_len = sizeof(sockaddr_in);
};

Udp::Udp() : impl_(std::make_unique<Impl>()) {}

Udp::~Udp() {
    Close();
}

Udp::Udp(Udp &&other) noexcept = default;
Udp &Udp::operator=(Udp &&other) noexcept = default;

bool Udp::Init(const UdpConfig &cfg) {
    if (!impl_) {
        impl_ = std::make_unique<Impl>();
    }
    if (impl_->sockfd > 0) {
        // 已经初始化
        return false;
    }

    impl_->cfg = cfg;

    // 创建 socket
    impl_->sockfd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (impl_->sockfd < 0) {
        std::cerr << "[Udp] socket 创建失败" << std::endl;
        return false;
    }

    // 设置非阻塞
    int flags = fcntl(impl_->sockfd, F_GETFL, 0);
    fcntl(impl_->sockfd, F_SETFL, flags | O_NONBLOCK);

    // 设置接收超时
    timeval timeout{};
    timeout.tv_sec = cfg.timeout_ms / 1000;
    timeout.tv_usec = (cfg.timeout_ms % 1000) * 1000;
    ::setsockopt(impl_->sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    // 允许端口复用
    int reuse = 1;
    ::setsockopt(impl_->sockfd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    if (cfg.role == Role::SERVER) {
        // 服务端：绑定本地端口
        impl_->local_addr.sin_family = AF_INET;
        impl_->local_addr.sin_port = htons(static_cast<uint16_t>(cfg.local_port));
        impl_->local_addr.sin_addr.s_addr =
            cfg.local_ip.empty() ? INADDR_ANY : ::inet_addr(cfg.local_ip.c_str());

        if (::bind(impl_->sockfd,
                    reinterpret_cast<sockaddr *>(&impl_->local_addr),
                    sizeof(impl_->local_addr)) < 0) {
            std::cerr << "[Udp] bind 失败, ip=" << cfg.local_ip << " port=" << cfg.local_port
                    << std::endl;
            ::close(impl_->sockfd);
            impl_->sockfd = -1;
            return false;
        }
        std::cout << "[Udp] Server 监听 " << cfg.local_ip << ":" << cfg.local_port << std::endl;
    } else {
        // 客户端：配置远端地址
        impl_->remote_addr.sin_family = AF_INET;
        impl_->remote_addr.sin_port = htons(static_cast<uint16_t>(cfg.remote_port));
        impl_->remote_addr.sin_addr.s_addr = ::inet_addr(cfg.remote_ip.c_str());
        impl_->connected = true;
        std::cout << "[Udp] Client 目标 " << cfg.remote_ip << ":" << cfg.remote_port << std::endl;
    }

    return true;
}

int Udp::Send(const void *data, std::size_t len) {
    if (!impl_ || impl_->sockfd < 0) {
        return -1;
    }

    int sent = -1;
    if (impl_->cfg.role == Role::SERVER) {
        // Server：发送给最近一次收到数据的对端
        if (!impl_->connected) {
            return -1;
        }
        sent = ::sendto(impl_->sockfd,
                        data,
                        len,
                        0,
                        reinterpret_cast<sockaddr *>(&impl_->remote_addr),
                        impl_->addr_len);
    } else {
        // Client：发送给配置的 remote
        sent = ::sendto(impl_->sockfd,
                        data,
                        len,
                        0,
                        reinterpret_cast<sockaddr *>(&impl_->remote_addr),
                        impl_->addr_len);
    }

    return sent;
}

int Udp::Recv(void *data, std::size_t max_len) {
    if (!impl_ || impl_->sockfd < 0) {
        return -1;
    }

    sockaddr_in sender_addr{};
    socklen_t sender_len = sizeof(sender_addr);

    int received = ::recvfrom(impl_->sockfd,
                            data,
                            static_cast<int>(max_len),
                            0,
                            reinterpret_cast<sockaddr *>(&sender_addr),
                            &sender_len);

    if (received > 0) {
        if (impl_->cfg.role == Role::SERVER) {
            // 记录最近的对端地址，供 send 使用
            impl_->remote_addr = sender_addr;
            impl_->connected = true;
        }
        return received;
    }

    // 无数据或超时
    return 0;
}

bool Udp::IsConnected() const {
    return impl_ && impl_->connected.load();
}

void Udp::Close() {
    if (impl_ && impl_->sockfd > 0) {
        ::close(impl_->sockfd);
        impl_->sockfd = -1;
        impl_->connected = false;
    }
}

const UdpConfig &Udp::GetConfig() const {
    return impl_->cfg;
}

}  // namespace transport_udp
