/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file shm_transport.h
 * @brief 共享内存通信模块（POSIX + 无锁环形缓冲区）
 *
 * 提供基于 POSIX 共享内存的进程间通信接口，使用无锁环形缓冲区实现
 * 单生产者单消费者（SPSC）模式，适合高频低延迟场景。
 */
#ifndef SHM_TRANSPORT_H
#define SHM_TRANSPORT_H

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace transport_shm {

/**
 * @brief 共享内存角色
 */
enum class Role {
    WRITER,  ///< 写入者（生产者）
    READER   ///< 读取者（消费者）
};

/**
 * @brief 共享内存配置
 */
struct ShmConfig {
    std::string channel_name;         ///< 共享内存名称（如 "/hmrs_state"）
    Role role = Role::WRITER;         ///< 角色
    uint32_t capacity = 4;            ///< 环形缓冲区容量（槽位数）
    uint32_t slot_size = 4096;        ///< 每个槽位大小（字节）
    bool create_if_not_exist = true;  ///< Writer 自动创建
};

/**
 * @brief 共享内存通信类
 *
 * 使用无锁环形缓冲区实现 SPSC 通信，适合单机多进程场景。
 */
class Shm {
public:
    Shm();
    ~Shm();

    Shm(const Shm&) = delete;
    Shm& operator=(const Shm&) = delete;
    Shm(Shm&&) noexcept;
    Shm& operator=(Shm&&) noexcept;

    /**
     * @brief 初始化共享内存
     * @param cfg 配置
     * @return 成功返回 true
     */
    bool Init(const ShmConfig& cfg);

    /**
     * @brief 写入数据（非阻塞）
     * @param data 数据指针
     * @param len 数据长度
     * @return 成功返回 true，失败返回 false
     */
    bool Write(const void* data, std::size_t len);

    /**
     * @brief 读取最新数据（非阻塞）
     * @param data 接收缓冲区
     * @param max_len 缓冲区最大长度
     * @param actual_len 输出：实际读取长度
     * @return 有新数据返回 true，无数据返回 false
     */
    bool Read(void* data, std::size_t max_len, std::size_t& actual_len);

    /**
     * @brief 关闭共享内存
     */
    void Close();

    /**
     * @brief 查询是否已打开
     * @return 已打开返回 true
     */
    bool IsOpen() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace transport_shm

#endif  // SHM_TRANSPORT_H
