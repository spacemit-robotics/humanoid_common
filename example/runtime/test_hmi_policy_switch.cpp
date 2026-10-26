/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_hmi_policy_switch.cpp
 * @brief Exercise the real HMI terminal against a deterministic Control peer
 */

#include <fcntl.h>
#include <pty.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>

#include "robot_base.h"
#include "transport_executor.h"

namespace {
namespace fs = std::filesystem;
using Clock = std::chrono::steady_clock;
using Phase = robot_base::PolicySwitchStatus::Phase;
using Mode = robot_base::ControlMode;

void Require(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

class Harness {
public:
    Harness() {
        const auto nonce = Clock::now().time_since_epoch().count();
        const std::string prefix = "hmi_switch_" + std::to_string(nonce);
        directory_ = fs::temp_directory_path() / prefix;
        Require(fs::create_directory(directory_), "cannot create fixture directory");
        Require(chmod(directory_.c_str(), 0700) == 0, "cannot secure fixture directory");
        config_ = (directory_ / "config.yaml").string();
        connection_ = (directory_ / "connection.json").string();
        std::ofstream yaml(config_);
        yaml << "robot_base: {name: hmi_switch_test, robot_dir: ., num_dof: 3}\n"
            "operator_service:\n"
            "  terminal_only: true\n"
            "  port: 19766\n"
            "  connection_file: " << connection_ << "\n"
            "rl_policy:\n"
            "  onnx_infer:\n"
            "    default_policy: good\n"
            "    policy_names: [good, missing, chain]\n"
            "logging:\n"
            "  directory: logs\n"
            "  console: {enabled: false}\n"
            "hmi: {heartbeat_hz: 50, status_timeout: 0.6, request_timeout: 15}\n"
            "transport:\n"
            "  type: shm\n"
            "  shm: {prefix: " << prefix << ", capacity: 4, slot_size: 4096}\n";
        Require(yaml.good(), "cannot write HMI configuration");
    }

    ~Harness() {
        for (auto *child : {&child_, &service_}) {
            if (*child <= 0) continue;
            kill(*child, SIGTERM);
            for (int i = 0; i < 100; ++i) {
                if (waitpid(*child, nullptr, WNOHANG) == *child) {
                    *child = -1;
                    break;
                }
                DrainTerminal();
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            if (*child > 0) {
                kill(*child, SIGKILL);
                waitpid(*child, nullptr, 0);
            }
        }
        if (terminal_ >= 0) close(terminal_);
        // Driver owns the state channel; Control owns control/status and HMI owns hmi.
        control_.reset();
        driver_.reset();
        std::error_code error;
        fs::remove_all(directory_, error);
    }

    void Start(const char *binary, const char *tui) {
        driver_ = transport::CreateV2(config_);
        control_ = transport::CreateV2(config_);
        Require(driver_->Init(config_, transport::Role::DRIVER), "driver peer init failed");
        Require(control_->Init(config_, transport::Role::CONTROL), "control peer init failed");
        status.mode = Mode::DAMP;
        status.active_policy = "good";
        status.hmi_connected = true;
        service_ = fork();
        if (service_ == 0) {
            execl(binary, binary, config_.c_str(), "--real", "--terminal-only",
                static_cast<char *>(nullptr));
            _exit(127);
        }
        Require(service_ > 0, "service fork failed");
        Until([this] { return fs::exists(connection_); });
        int slave = -1;
        Require(openpty(&terminal_, &slave, nullptr, nullptr, nullptr) == 0, "openpty failed");
        child_ = fork();
        if (child_ == 0) {
            dup2(slave, STDIN_FILENO);
            dup2(slave, STDOUT_FILENO);
            dup2(slave, STDERR_FILENO);
            close(slave);
            close(terminal_);
            execl(tui, tui, "--connection", connection_.c_str(), static_cast<char *>(nullptr));
            _exit(127);
        }
        close(slave);
        Require(child_ > 0, "fork failed");
        Require(fcntl(terminal_, F_SETFL, O_NONBLOCK) == 0, "cannot set nonblocking terminal");
        Until([this] { return output.find("CONTROL ONLINE") != std::string::npos; });
        Keys("l");
        Until([this] { return output.find("已获得控制权") != std::string::npos; });
    }

    void Keys(const std::string &keys) {
        output.clear();
        Require(write(terminal_, keys.data(), keys.size()) == static_cast<ssize_t>(keys.size()),
            "cannot write terminal input");
    }

    void Pump() {
        Require(control_->SendStatusV2(status, {}), "cannot send Control status");
        uint64_t fault_ack = 0;
        while (control_->RecvCommandV2(command, fault_ack)) {}
        DrainTerminal();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    void Until(const std::function<bool()> &predicate) {
        const auto deadline = Clock::now() + std::chrono::seconds(2);
        do {
            Pump();
            if (predicate()) return;
        } while (Clock::now() < deadline);
        throw std::runtime_error("HMI response timeout; terminal: " +
            output.substr(output.size() > 4000 ? output.size() - 4000 : 0));
    }

    void CheckExit(bool manual) {
        // Stop execution so both signals and the socket EOF are pending on resume.
        if (manual) {
            Require(kill(child_, SIGSTOP) == 0, "cannot pause test TUI");
            int status = 0;
            Require(waitpid(child_, &status, WUNTRACED) == child_ && WIFSTOPPED(status),
                "test TUI did not pause");
            Require(kill(child_, SIGINT) == 0, "cannot interrupt test TUI");
        }
        Require(kill(service_, SIGTERM) == 0, "cannot stop test service");
        Require(waitpid(service_, nullptr, 0) == service_, "cannot reap test service");
        service_ = -1;
        if (manual) Require(kill(child_, SIGCONT) == 0, "cannot resume test TUI");
        int status = 0;
        Until([&] { return waitpid(child_, &status, WNOHANG) == child_; });
        child_ = -1;
        DrainTerminal();
        Require(WIFEXITED(status) && WEXITSTATUS(status) == (manual ? 0 : 1),
            "TUI exit code does not distinguish interruption from connection failure");
        Require((output.find("连接已断开") != std::string::npos) != manual,
            "TUI reported a normal interruption as disconnection or hid a real failure");
    }

    uint64_t Select(const std::string &policy, const std::string &keys) {
        const auto previous = status.policy_switch.sequence;
        Keys(keys);
        Until([&] { return command.switch_policy == policy &&
            command.policy_request_sequence > previous; });
        status.policy_switch = {command.policy_request_sequence, Phase::PREPARING, policy, {}};
        return command.policy_request_sequence;
    }

    robot_base::ControlStatus status;
    robot_base::Command command;
    std::string output;

private:
    void DrainTerminal() {
        if (terminal_ < 0) return;
        char buffer[8192];
        ssize_t count = 0;
        while ((count = read(terminal_, buffer, sizeof(buffer))) > 0) {
            output.append(buffer, static_cast<std::size_t>(count));
        }
    }

    fs::path directory_;
    std::string config_;
    std::string connection_;
    std::unique_ptr<transport::TransportBaseV2> driver_;
    std::unique_ptr<transport::TransportBaseV2> control_;
    pid_t child_ = -1;
    pid_t service_ = -1;
    int terminal_ = -1;
};

void Run(const char *binary, const char *tui) {
    Harness test;
    test.Start(binary, tui);
    const auto sequence = test.Select("missing", "pj\n");
    test.Keys("h");
    test.Until([&] { return test.output.find("上一请求尚未完成") != std::string::npos; });
    Require(test.command.key != 4, "HOME was allowed before a switch result");

    test.status.policy_switch = {sequence - 1, Phase::FAILED, "missing", "stale"};
    for (int i = 0; i < 20; ++i) test.Pump();
    Require(test.command.switch_policy == "missing", "stale reply completed a new request");
    test.status.policy_switch = {sequence, Phase::FAILED, "missing", "missing.onnx not found"};
    test.Until([&] { return test.command.switch_policy.empty() &&
        test.output.find("missing.onnx not found") != std::string::npos; });
    test.Keys("h");
    test.Until([&] { return test.command.key == 4; });
    test.status.mode = Mode::HOME;
    test.Until([&] { return test.command.key == 0; });
    test.status.mode = Mode::DAMP;
    test.Keys("p");
    test.Until([&] { return test.output.find("策略选择") != std::string::npos; });

    const auto retry = test.Select("missing", "j\n");
    Require(retry > sequence, "retry reused an old sequence");
    test.status.policy_switch.phase = Phase::REJECTED;
    test.status.policy_switch.detail = "test rejection";
    test.Until([&] { return test.command.switch_policy.empty(); });

    test.Select("chain", "pjj\n");
    test.status.policy_switch.phase = Phase::READY;
    // active_policy is still good: this must acknowledge a prepared prerequisite chain.
    test.Until([&] { return test.command.switch_policy.empty() &&
        test.output.find("请求已确认") != std::string::npos; });

    test.Select("missing", "pj\n");
    test.Keys("f");
    test.Until([&] { return test.command.key == -1 && test.command.switch_policy.empty(); });
    test.status.mode = Mode::POWER_OFF;
    test.Until([&] { return test.command.key == 0; });
    // POWER_OFF itself must not short-circuit a fresh power-off request during loading.
    test.Select("missing", "pj\n");
    test.Keys("f");
    test.Until([&] { return test.command.key == -1 && test.command.switch_policy.empty(); });
}
}  // namespace

int main(int argc, char *argv[]) {
    if (argc != 3) return 2;
    try {
        Run(argv[1], argv[2]);
        for (bool manual : {true, false}) {
            Harness test;
            test.Start(argv[1], argv[2]);
            test.CheckExit(manual);
        }
        std::cout << "HMI policy switch regression: PASS\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
