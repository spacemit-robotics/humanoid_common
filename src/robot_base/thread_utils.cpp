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
#include <string>

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

void SetCpuAffinity(const std::string &name, int cpu_id) {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(cpu_id, &cpuset);
    int ret = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
    if (ret != 0) {
        std::cerr << "[ThreadUtils] " << name << " CPU 绑定失败 (cpu=" << cpu_id
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
    t.sched = yaml_file.Read<std::string>(prefix + ".sched").value_or("other");
    t.priority = yaml_file.Read<int>(prefix + ".priority").value_or(0);
    return t;
}

void ThreadLoop::Apply() const {
    pthread_setname_np(pthread_self(), name.substr(0, 15).c_str());
    if (cpu_id >= 0) {
        SetCpuAffinity(name, cpu_id);
    }
    SetSchedParam(name, sched, priority);

    std::cout << "[ThreadLoop] 线程 " << name << " 配置完成 (cpu=" << cpu_id << ", sched=" << sched
            << ", priority=" << priority << ")" << std::endl;
}

ThreadLoop::~ThreadLoop() {
    Stop();
}

void ThreadLoop::Start(std::function<bool()> func) {
    if (running_) {
        std::cerr << "[ThreadLoop] 线程 " << name << " 已在运行" << std::endl;
        return;
    }
    running_ = true;

    thread_ = std::thread([this, func]() {
        Apply();
        std::cout << "[ThreadLoop] 线程 " << name << " 启动" << std::endl;
        while (running_) {
            if (!func()) {
                break;
            }
        }
        std::cout << "[ThreadLoop] 线程 " << name << " 退出" << std::endl;
    });
}

void ThreadLoop::Stop() {
    running_ = false;
    if (thread_.joinable()) {
        thread_.join();
    }
}

bool ThreadLoop::IsRunning() const {
    return running_;
}

}  // namespace robot_base
