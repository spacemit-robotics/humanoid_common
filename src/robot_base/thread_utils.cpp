/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file thread_utils.cpp
 * @brief 线程配置与管理工具实现
 */

#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <malloc.h>

#include <iostream>
#include <cerrno>
#include <cstdlib>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_base.h"
namespace robot_base {

namespace {

std::vector<int> CurrentCpuAffinity() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    std::vector<int> result;
    if (pthread_getaffinity_np(pthread_self(), sizeof(cpuset), &cpuset) == 0) {
        for (int cpu = 0; cpu < CPU_SETSIZE; ++cpu) {
            if (CPU_ISSET(cpu, &cpuset)) result.push_back(cpu);
        }
    }
    return result;
}

// Captured before main applies any per-thread affinity; respects launch-time taskset/cpuset.
const std::vector<int> kInitialCpuAffinity = CurrentCpuAffinity();

void PrepareRealtimeMemory() {
    static std::once_flag once;
    std::call_once(once, [] {
        // Keep reusable heap pages resident instead of repeatedly returning them
        // to the kernel. Large allocations are served from retained arenas.
        mallopt(M_TRIM_THRESHOLD, -1);
        mallopt(M_MMAP_MAX, 0);
        constexpr size_t kReserveBytes = 32 * 1024 * 1024;
        auto *reserve = static_cast<volatile unsigned char *>(std::malloc(kReserveBytes));
        if (reserve) {
            for (size_t i = 0; i < kReserveBytes; i += 4096) reserve[i] = 0;
            std::free(const_cast<unsigned char *>(reserve));
        }
        // ONFAULT avoids populating unused virtual arenas, while retaining all
        // touched model/buffer pages and locking future allocations on access.
        if (mlockall(MCL_CURRENT | MCL_FUTURE | MCL_ONFAULT) == 0)
            std::cout << "[ThreadLoop] realtime memory locked (on-fault), heap prewarmed\n";
        else
            std::cerr << "[ThreadLoop] realtime memory lock failed errno=" << errno
                << "; check LimitMEMLOCK\n";
    });
    volatile unsigned char stack[256 * 1024];
    for (size_t i = 0; i < sizeof(stack); i += 4096) stack[i] = 0;
}

int SchedPolicyFromString(const std::string &sched) {
    if (sched == "fifo")
        return SCHED_FIFO;
    if (sched == "rr")
        return SCHED_RR;
    return SCHED_OTHER;
}

std::string FormatCpuAffinity(const std::vector<int> &cpu_ids) {
    std::ostringstream output;
    for (std::size_t index = 0; index < cpu_ids.size(); ++index) {
        if (index != 0) output << ',';
        output << cpu_ids[index];
    }
    return output.str();
}

void SetCpuAffinity(const std::string &name, const std::vector<int> &cpu_ids) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    for (const int cpu_id : cpu_ids) {
        if (cpu_id < 0 || cpu_id >= CPU_SETSIZE) {
            std::cerr << "[ThreadUtils] " << name
                    << " CPU 绑定失败：编号越界 (cpu=" << cpu_id << ")" << std::endl;
            return;
        }
        CPU_SET(cpu_id, &cpuset);
    }
    int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        std::cerr << "[ThreadUtils] " << name << " CPU 绑定失败 (cpus="
                << FormatCpuAffinity(cpu_ids)
                << ", errno=" << ret << ")" << std::endl;
    }
}

void SetSchedParam(const std::string &name, const std::string &sched, int priority) {
    int policy = SchedPolicyFromString(sched);
    struct sched_param param;
    param.sched_priority = policy == SCHED_OTHER ? 0 : priority;
    int ret = pthread_setschedparam(pthread_self(), policy, &param);
    if (ret != 0) {
        std::cerr << "[ThreadUtils] " << name << " 调度策略设置失败 (sched=" << sched
                << ", priority=" << priority << ", errno=" << ret
                << ")，请检查配置和实时调度权限"
                << std::endl;
    }
}

}  // namespace

// ==================== ThreadLoop ====================

ThreadLoop ThreadLoop::FromYaml(const YamlFile &yaml_file, const std::string &thread_name) {
    const std::string prefix = "robot_base.threads." + thread_name;
    ThreadLoop t;
    t.name = thread_name;
    t.cpu_id = yaml_file.Read<int>(prefix + ".cpu_id").value_or(-1);
    t.cpu_affinity = yaml_file.Read<std::vector<int>>(
        prefix + ".cpu_affinity").value_or(std::vector<int>{});
    if (!t.cpu_affinity.empty() && t.cpu_id >= 0) {
        throw std::runtime_error("[ThreadLoop] " + prefix +
            " 不能同时配置 cpu_id 和 cpu_affinity");
    }
    for (const int cpu_id : t.cpu_affinity) {
        if (cpu_id < 0) {
            throw std::runtime_error("[ThreadLoop] " + prefix +
                ".cpu_affinity 不能包含负数");
        }
    }
    t.sched = yaml_file.Read<std::string>(prefix + ".sched").value_or("other");
    t.priority = yaml_file.Read<int>(prefix + ".priority").value_or(0);
    t.lock_memory = yaml_file.Read<bool>(prefix + ".lock_memory").value_or(false);
    return t;
}

ThreadLoop ThreadLoop::Background(const std::string &thread_name) {
    ThreadLoop config;
    config.name = thread_name;
    config.cpu_affinity = kInitialCpuAffinity;
    return config;
}

void ThreadLoop::Apply() const {
    if (lock_memory) PrepareRealtimeMemory();
    pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());
    if (!cpu_affinity.empty()) {
        SetCpuAffinity(name, cpu_affinity);
    } else if (cpu_id >= 0) {
        SetCpuAffinity(name, std::vector<int>{cpu_id});
    }
    SetSchedParam(name, sched, priority);

    const std::string cpu_config = cpu_affinity.empty()
        ? std::to_string(cpu_id) : FormatCpuAffinity(cpu_affinity);
    int actual_policy = 0;
    sched_param actual{};
    const int result = pthread_getschedparam(pthread_self(), &actual_policy, &actual);
    if (result != 0) {
        std::cerr << "[ThreadLoop] " << name << " 无法读取实际调度 (errno="
            << result << ")\n";
        return;
    }
    const char *actual_sched = actual_policy == SCHED_FIFO ? "fifo" :
        (actual_policy == SCHED_RR ? "rr" : (actual_policy == SCHED_OTHER ? "other" : "other-policy"));
    std::cout << "[ThreadLoop] " << name << " requested(cpu=" << cpu_config
        << ",sched=" << sched << ",priority=" << priority << ") actual(cpus="
        << FormatCpuAffinity(CurrentCpuAffinity()) << ",sched=" << actual_sched
        << ",priority=" << actual.sched_priority << ")\n";
}

ThreadLoop::~ThreadLoop() {
    Stop();
}

void ThreadLoop::Start(std::function<bool()> func) {
    if (running_.load(std::memory_order_acquire)) {
        std::cerr << "[ThreadLoop] 线程 " << name << " 已在运行" << std::endl;
        return;
    }
    if (thread_.joinable()) {
        thread_.join();
    }
    running_.store(true, std::memory_order_release);
    try {
        thread_ = std::thread([this, func]() {
            Apply();
            std::cout << "[ThreadLoop] 线程 " << name << " 启动" << std::endl;
            while (running_.load(std::memory_order_acquire)) {
                if (!func()) {
                    break;
                }
            }
            std::cout << "[ThreadLoop] 线程 " << name << " 退出" << std::endl;
            running_.store(false, std::memory_order_release);
        });
    } catch (...) {
        running_.store(false, std::memory_order_release);
        throw;
    }
}

void ThreadLoop::Stop() {
    running_.store(false, std::memory_order_release);
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool ThreadLoop::IsRunning() const {
    return running_.load(std::memory_order_acquire);
}

}  // namespace robot_base
