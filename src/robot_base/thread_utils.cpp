/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file thread_utils.cpp
 * @brief 线程配置与管理工具实现
 */

#include <pthread.h>
#include <sched.h>

#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "robot_base.h"
namespace robot_base {

namespace {

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
    if (policy == SCHED_OTHER) {
        return;  // 默认 CFS 调度器无需设置
    }
    struct sched_param param;
    param.sched_priority = priority;
    int ret = pthread_setschedparam(pthread_self(), policy, &param);
    if (ret != 0) {
        std::cerr << "[ThreadUtils] " << name << " 调度策略设置失败 (sched=" << sched
                << ", priority=" << priority << ", errno=" << ret << ")，可能需要 root 权限"
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
    return t;
}

void ThreadLoop::Apply() const {
    pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());
    if (!cpu_affinity.empty()) {
        SetCpuAffinity(name, cpu_affinity);
    } else if (cpu_id >= 0) {
        SetCpuAffinity(name, std::vector<int>{cpu_id});
    }
    SetSchedParam(name, sched, priority);

    const std::string cpu_config = cpu_affinity.empty()
        ? std::to_string(cpu_id) : FormatCpuAffinity(cpu_affinity);
    std::cout << "[ThreadLoop] 线程 " << name << " 配置完成 (cpu=" << cpu_config
            << ", sched=" << sched << ", priority=" << priority << ")" << std::endl;
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
