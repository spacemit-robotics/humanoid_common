/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_thread_scheduling.cpp
 * @brief Check background scheduling isolation and asynchronous console output.
 */

#include <pthread.h>
#include <sched.h>

#include <chrono>
#include <condition_variable>
#include <future>
#include <iostream>
#include <mutex>
#include <streambuf>
#include <string>
#include <thread>

#include "robot_base.h"
#include "runtime_logger.h"

namespace {

bool TestBackgroundScheduling() {
    return std::async(std::launch::async, [] {
        cpu_set_t before;
        CPU_ZERO(&before);
        if (pthread_getaffinity_np(pthread_self(), sizeof(before), &before) != 0) return false;
        int first_cpu = 0;
        while (first_cpu < CPU_SETSIZE && !CPU_ISSET(first_cpu, &before)) ++first_cpu;
        if (first_cpu == CPU_SETSIZE) return false;

        robot_base::ThreadLoop pinned;
        pinned.name = "test_parent";
        pinned.cpu_id = first_cpu;
        pinned.Apply();
        sched_param param{};
        // SCHED_BATCH can be selected without acquiring real-time privileges.
        if (pthread_setschedparam(pthread_self(), SCHED_BATCH, &param) != 0) return false;
        robot_base::ThreadLoop::Background("test_background").Apply();
        int policy = -1;
        cpu_set_t after;
        CPU_ZERO(&after);
        return pthread_getschedparam(pthread_self(), &policy, &param) == 0 &&
            policy == SCHED_OTHER && param.sched_priority == 0 &&
            pthread_getaffinity_np(pthread_self(), sizeof(after), &after) == 0 &&
            CPU_EQUAL(&before, &after);
    }).get();
}

class BlockingConsole : public std::streambuf {
public:
    bool Entered() {
        std::lock_guard<std::mutex> lock(mutex_);
        return entered_;
    }
    bool WaitUntilBlocked() {
        std::unique_lock<std::mutex> lock(mutex_);
        return ready_.wait_for(lock, std::chrono::seconds(2), [this] { return entered_; });
    }
    void Release() {
        std::lock_guard<std::mutex> lock(mutex_);
        released_ = true;
        ready_.notify_all();
    }

protected:
    std::streamsize xsputn(const char *, std::streamsize count) override {
        Wait();
        return count;
    }
    int_type overflow(int_type value) override {
        Wait();
        return traits_type::not_eof(value);
    }

private:
    void Wait() {
        std::unique_lock<std::mutex> lock(mutex_);
        entered_ = true;
        ready_.notify_all();
        ready_.wait(lock, [this] { return released_; });
    }
    std::mutex mutex_;
    std::condition_variable ready_;
    bool entered_ = false;
    bool released_ = false;
};

bool TestBlockedConsole(const char *config_path) {
    BlockingConsole sink;
    auto *original = std::cerr.rdbuf(&sink);
    bool passed = false;
    {
        const auto yaml = robot_base::YamlFile::Load(config_path);
        runtime_logging::Session session(yaml, config_path, "console_test");
        auto caller = std::async(std::launch::async, [&] {
            // Event enqueue is deliberately try-lock/drop. Retry a dropped probe
            // until the sink is entered; a blocked Log call still fails below.
            for (int i = 0; i < 200 && !sink.Entered(); ++i) {
                runtime_logging::Log(runtime_logging::Level::kWarning, "blocked console probe");
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
        const bool blocked = sink.WaitUntilBlocked();
        passed = blocked && caller.wait_for(std::chrono::milliseconds(100)) ==
            std::future_status::ready;
        sink.Release();
        caller.get();
    }
    std::cerr.rdbuf(original);
    return passed;
}

class FlushConsole : public std::streambuf {
public:
    bool WaitForMessage(std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
        std::unique_lock<std::mutex> lock(mutex_);
        return ready_.wait_for(lock, timeout, [this] { return flushed_; });
    }

protected:
    std::streamsize xsputn(const char *data, std::streamsize count) override {
        std::lock_guard<std::mutex> lock(mutex_);
        buffered_.append(data, static_cast<std::size_t>(count));
        return count;
    }
    int_type overflow(int_type value) override {
        if (!traits_type::eq_int_type(value, traits_type::eof())) {
            const char character = traits_type::to_char_type(value);
            xsputn(&character, 1);
        }
        return traits_type::not_eof(value);
    }
    int sync() override {
        std::lock_guard<std::mutex> lock(mutex_);
        if (buffered_.find("startup ready probe") != std::string::npos) {
            flushed_ = true;
            ready_.notify_all();
        }
        return 0;
    }

private:
    std::mutex mutex_;
    std::condition_variable ready_;
    std::string buffered_;
    bool flushed_ = false;
};

bool TestConsoleFlush(const char *config_path) {
    FlushConsole sink;
    auto *original = std::cout.rdbuf(&sink);
    bool passed = false;
    {
        const auto yaml = robot_base::YamlFile::Load(config_path);
        runtime_logging::Session session(yaml, config_path, "flush_test");
        for (int i = 0; i < 100 && !passed; ++i) {
            runtime_logging::Log(runtime_logging::Level::kInfo, "startup ready probe");
            passed = sink.WaitForMessage(std::chrono::milliseconds(10));
        }
    }
    std::cout.rdbuf(original);
    return passed;
}

}  // namespace

int main(int argc, char **argv) {
    if (argc != 2) return 1;
    if (!TestBackgroundScheduling()) {
        std::cerr << "background thread retained inherited scheduling or affinity\n";
        return 1;
    }
    if (!TestBlockedConsole(argv[1])) {
        std::cerr << "console output blocked the logging caller\n";
        return 1;
    }
    if (!TestConsoleFlush(argv[1])) {
        std::cerr << "console event remained buffered until shutdown\n";
        return 1;
    }
    std::cout << "thread scheduling, blocked console and prompt flush: PASS\n";
    return 0;
}
