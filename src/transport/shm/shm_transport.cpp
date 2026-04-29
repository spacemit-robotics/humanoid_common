/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file shm_transport.cpp
 * @brief 共享内存通信实现（POSIX + 无锁环形缓冲区）
 */

#include "shm_transport.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <iostream>
#include <algorithm>
namespace transport_shm {

// ==================== 共享内存布局 ====================

#pragma pack(push, 1)
struct ShmChannelHeader {
    std::atomic<uint32_t> write_index{0};  // 写指针
    std::atomic<uint32_t> read_index{0};   // 读指针
    uint32_t capacity{0};                  // 槽位数
    uint32_t slot_size{0};                 // 槽位大小
    uint8_t padding[48]{0};                // 缓存行对齐
};
#pragma pack(pop)

// ==================== Shm::Impl ====================

class Shm::Impl {
public:
    Impl() = default;
    ~Impl() { Close(); }

    bool Init(const ShmConfig& cfg);
    bool Write(const void* data, std::size_t len);
    bool Read(void* data, std::size_t max_len, std::size_t& actual_len);
    void Close();
    bool IsOpen() const { return shm_ptr_ != nullptr; }

private:
    ShmConfig cfg_;
    int shm_fd_ = -1;
    void* shm_ptr_ = nullptr;
    std::size_t shm_size_ = 0;

    ShmChannelHeader* header_ = nullptr;
    uint8_t* data_region_ = nullptr;
};

bool Shm::Impl::Init(const ShmConfig& cfg) {
    cfg_ = cfg;

    // 计算共享内存大小
    shm_size_ = sizeof(ShmChannelHeader) + cfg.capacity * cfg.slot_size;

    // 打开或创建共享内存
    int flags = O_RDWR;
    if (cfg.create_if_not_exist) {
        flags |= O_CREAT;
    }

    shm_fd_ = shm_open(cfg.channel_name.c_str(), flags, 0666);
    if (shm_fd_ < 0) {
        std::cerr << "[shm] shm_open 失败: " << cfg.channel_name << "\n";
        return false;
    }

    // 设置大小（首次创建时需要，已存在时 ftruncate 不会缩小）
    if (cfg.create_if_not_exist) {
        if (ftruncate(shm_fd_, shm_size_) < 0) {
            std::cerr << "[shm] ftruncate 失败\n";
            ::close(shm_fd_);
            shm_fd_ = -1;
            return false;
        }
    }

    // 映射到进程地址空间
    shm_ptr_ = mmap(nullptr, shm_size_, PROT_READ | PROT_WRITE, MAP_SHARED, shm_fd_, 0);
    if (shm_ptr_ == MAP_FAILED) {
        std::cerr << "[shm] mmap 失败\n";
        ::close(shm_fd_);
        shm_fd_ = -1;
        shm_ptr_ = nullptr;
        return false;
    }

    header_ = static_cast<ShmChannelHeader*>(shm_ptr_);
    data_region_ = static_cast<uint8_t*>(shm_ptr_) + sizeof(ShmChannelHeader);

    // Writer 初始化 header
    if (cfg.role == Role::WRITER) {
        header_->write_index.store(0, std::memory_order_relaxed);
        header_->read_index.store(0, std::memory_order_relaxed);
        header_->capacity = cfg.capacity;
        header_->slot_size = cfg.slot_size;
    } else if (header_->capacity == 0) {
        // Reader 首次创建时也需要初始化 header（Writer 尚未启动）
        header_->write_index.store(0, std::memory_order_relaxed);
        header_->read_index.store(0, std::memory_order_relaxed);
        header_->capacity = cfg.capacity;
        header_->slot_size = cfg.slot_size;
    }

    return true;
}

bool Shm::Impl::Write(const void* data, std::size_t len) {
    if (!IsOpen() || cfg_.role != Role::WRITER)
        return false;
    if (len > header_->slot_size)
        return false;

    uint32_t write_idx = header_->write_index.load(std::memory_order_relaxed);
    uint32_t next_write = (write_idx + 1) % header_->capacity;

    // 写入数据到槽位
    void* slot = data_region_ + (write_idx * header_->slot_size);
    std::memcpy(slot, data, len);

    // 更新写指针
    header_->write_index.store(next_write, std::memory_order_release);
    return true;
}

bool Shm::Impl::Read(void* data, std::size_t max_len, std::size_t& actual_len) {
    if (!IsOpen() || cfg_.role != Role::READER)
        return false;

    uint32_t write_idx = header_->write_index.load(std::memory_order_acquire);
    uint32_t read_idx = header_->read_index.load(std::memory_order_relaxed);

    // 检查是否有新数据
    if (read_idx == write_idx)
        return false;

    // 读取最新数据（跳过中间数据）
    uint32_t latest_idx = (write_idx == 0) ? (header_->capacity - 1) : (write_idx - 1);
    void* slot = data_region_ + (latest_idx * header_->slot_size);

    std::size_t copy_len = std::min(max_len, static_cast<std::size_t>(header_->slot_size));
    std::memcpy(data, slot, copy_len);
    actual_len = copy_len;

    // 更新读指针到最新
    header_->read_index.store(write_idx, std::memory_order_release);
    return true;
}

void Shm::Impl::Close() {
    if (shm_ptr_ != nullptr && shm_ptr_ != MAP_FAILED) {
        munmap(shm_ptr_, shm_size_);
        shm_ptr_ = nullptr;
    }

    if (shm_fd_ >= 0) {
        ::close(shm_fd_);
        shm_fd_ = -1;
    }

    // Writer 负责删除共享内存
    if (cfg_.role == Role::WRITER && !cfg_.channel_name.empty()) {
        shm_unlink(cfg_.channel_name.c_str());
    }
}

// ==================== Shm 公共接口 ====================

Shm::Shm() : impl_(std::make_unique<Impl>()) {}
Shm::~Shm() = default;
Shm::Shm(Shm&&) noexcept = default;
Shm& Shm::operator=(Shm&&) noexcept = default;

bool Shm::Init(const ShmConfig& cfg) {
    return impl_->Init(cfg);
}
bool Shm::Write(const void* data, std::size_t len) {
    return impl_->Write(data, len);
}
bool Shm::Read(void* data, std::size_t max_len, std::size_t& actual_len) {
    return impl_->Read(data, max_len, actual_len);
}
void Shm::Close() {
    impl_->Close();
}
bool Shm::IsOpen() const {
    return impl_->IsOpen();
}

}  // namespace transport_shm
