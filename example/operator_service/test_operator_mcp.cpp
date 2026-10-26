/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_operator_mcp.cpp
 * @brief MCP integration tests with HMI and a synthetic control endpoint.
 */

/*
 * 端到端：MCP JSON-RPC → humanoid_operator_mcp → WebSocket → 真实 hmi_runtime → 合成 control。
 * 合成 control 按 humanoid_common 的 example/operator_service/test_operator_network.cpp 的做法，
 * 用公开的 transport_executor 收命令、回状态，动作按时长从 PLAYING 走到 FINISHED。
 *
 * 用法: test_operator_mcp <hmi_runtime> <humanoid_operator_mcp>
 */

#include <poll.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>

#include "operator_client.h"
#include "robot_base.h"
#include "transport_executor.h"

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "ASSERTION FAILED: " << message << std::endl;
        std::exit(1);
    }
}

bool Wait(const std::function<bool()>& predicate, int timeout_ms = 3000) {
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    while (Clock::now() < deadline) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    return predicate();
}

// 合成 control：RL 状态、test_policy；wave 播 2.5 s（超过 1 s 租约，必须续租），long_wave 播 10 s，
// refuse 直接拒绝。同一序号只处理一次（HMI 会重复发同一条命令）。
class FakeControl {
public:
    explicit FakeControl(const std::string& config) {
        transport_ = transport::CreateV2(config);
        require(transport_ && transport_->Init(config, transport::Role::CONTROL), "control transport");
        thread_ = std::thread([this]() { Loop(); });
    }
    ~FakeControl() {
        running_ = false;
        thread_.join();
    }
    int starts() const { return starts_; }
    // 打断了正在播放的动作的 cancel 次数（操作服务每次释放控制权都会发一次 cancel，空闲时的不算）。
    int cancels() const { return cancels_; }
    std::string phase() {
        std::lock_guard<std::mutex> lock(mutex_);
        return PhaseName(status_.interaction.phase);
    }

private:
    using Phase = robot_base::InteractionStatus::Phase;
    using Operation = robot_base::InteractionRequest::Operation;

    static const char* PhaseName(Phase phase) {
        switch (phase) {
        case Phase::BLEND_IN: return "BLEND_IN";
        case Phase::PLAYING: return "PLAYING";
        case Phase::HOLDING: return "HOLDING";
        case Phase::BLEND_OUT: return "BLEND_OUT";
        case Phase::FINISHED: return "FINISHED";
        case Phase::REJECTED: return "REJECTED";
        default: return "IDLE";
        }
    }

    void Loop() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            status_.mode = robot_base::ControlMode::RL;
            status_.active_policy = "test_policy";
            status_.zero_ready = true;
            status_.rl_frequency_hz = 50;
            status_.hmi_connected = true;
        }
        uint64_t handled = 0;
        Clock::time_point phase_end;
        while (running_) {
            Step(&handled, &phase_end);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    void Step(uint64_t* handled_sequence, Clock::time_point* end) {
        uint64_t& handled = *handled_sequence;
        Clock::time_point& phase_end = *end;
        robot_base::Command command;
        uint64_t acknowledge = 0;
        std::lock_guard<std::mutex> lock(mutex_);
        while (transport_->RecvCommandV2(command, acknowledge)) {
            const auto& request = command.interaction;
            if (request.operation == Operation::NONE || request.sequence <= handled) continue;
            handled = request.sequence;
            auto& s = status_.interaction;
            s.sequence = request.sequence;
            if (request.operation == Operation::START) {
                ++starts_;
                s.action = request.action;
                s.request_accepted = request.action != "refuse";
                s.phase = s.request_accepted ? Phase::PLAYING : Phase::REJECTED;
                phase_end = Clock::now() + std::chrono::milliseconds(request.action == "long_wave" ? 10000 : 2500);
            } else {
                const bool busy = s.phase == Phase::PLAYING || s.phase == Phase::BLEND_IN;
                s.request_accepted = busy;
                if (busy) {
                    ++cancels_;
                    s.phase = Phase::BLEND_OUT;
                    phase_end = Clock::now() + std::chrono::milliseconds(200);
                }
            }
        }
        auto& s = status_.interaction;
        if (s.phase == Phase::PLAYING && Clock::now() >= phase_end) s.phase = Phase::FINISHED;
        if (s.phase == Phase::BLEND_OUT && Clock::now() >= phase_end) {
            s.phase = Phase::IDLE;
            s.action.clear();  // transport 规定 IDLE 不带动作名，否则整条状态编码失败、HMI 判离线
        }
        require(transport_->SendStatusV2(status_, {}), "synthetic control status must encode");
    }

    std::unique_ptr<transport::TransportBaseV2> transport_;
    std::thread thread_;
    std::atomic<bool> running_{true};
    std::atomic<int> starts_{0};
    std::atomic<int> cancels_{0};
    std::mutex mutex_;
    robot_base::ControlStatus status_;
};

// 用管道和 humanoid_operator_mcp 子进程说 MCP。
class McpProcess {
public:
    McpProcess(const std::string& binary, const std::vector<std::string>& args) {
        int to_child[2];
        int from_child[2];
        require(pipe(to_child) == 0 && pipe(from_child) == 0, "pipes");
        pid_ = fork();
        require(pid_ >= 0, "fork");
        if (pid_ == 0) {
            prctl(PR_SET_PDEATHSIG, SIGKILL);
            dup2(to_child[0], STDIN_FILENO);
            dup2(from_child[1], STDOUT_FILENO);
            close(to_child[1]);
            close(from_child[0]);
            std::vector<char*> argv;
            argv.push_back(const_cast<char*>(binary.c_str()));
            for (const auto& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
            argv.push_back(nullptr);
            execv(binary.c_str(), argv.data());
            _exit(127);
        }
        close(to_child[0]);
        close(from_child[1]);
        in_ = to_child[1];
        out_ = from_child[0];
    }
    ~McpProcess() {
        CloseInput();
        if (pid_ > 0) {
            kill(pid_, SIGKILL);
            waitpid(pid_, nullptr, 0);
        }
        if (out_ >= 0) close(out_);
    }

    void Send(const json& message) {
        const std::string line = message.dump() + "\n";
        require(write(in_, line.data(), line.size()) == static_cast<ssize_t>(line.size()), "write to MCP server");
    }

    json Read(int timeout_ms = 8000) {
        for (;;) {
            const size_t newline = buffer_.find('\n');
            if (newline != std::string::npos) {
                const std::string line = buffer_.substr(0, newline);
                buffer_.erase(0, newline + 1);
                return json::parse(line);
            }
            pollfd pfd{out_, POLLIN, 0};
            require(poll(&pfd, 1, timeout_ms) == 1, "timed out waiting for the MCP server");
            char buf[4096];
            const ssize_t n = read(out_, buf, sizeof(buf));
            require(n > 0, "MCP server closed stdout");
            buffer_.append(buf, static_cast<size_t>(n));
        }
    }

    json Request(const std::string& method, const json& params = json::object()) {
        const int id = ++next_id_;
        Send({{"jsonrpc", "2.0"}, {"id", id}, {"method", method}, {"params", params}});
        for (;;) {
            json reply = Read();
            if (reply.contains("id") && reply["id"] == id) return reply;  // 跳过 list_changed 之类的通知
        }
    }

    json Call(const std::string& tool, const json& args = json::object()) {
        const json reply = Request("tools/call", {{"name", tool}, {"arguments", args}});
        require(reply.contains("result"), "tools/call failed: " + reply.dump());
        return reply["result"];
    }

    void Initialize() {
        const json reply = Request("initialize", {{"protocolVersion", "2024-11-05"},
            {"capabilities", json::object()}, {"clientInfo", {{"name", "test"}, {"version", "1"}}}});
        require(reply["result"]["serverInfo"]["name"] == "humanoid_operator_mcp", "initialize: " + reply.dump());
        Send({{"jsonrpc", "2.0"}, {"method", "notifications/initialized"}});
    }

    void CloseInput() {
        if (in_ >= 0) close(in_);
        in_ = -1;
    }

    // 等子进程自己退出（stdin EOF 之后）。
    bool WaitExit(int timeout_ms) {
        const bool exited = Wait([this]() {
            int status = 0;
            return waitpid(pid_, &status, WNOHANG) == pid_;
        }, timeout_ms);
        if (exited) pid_ = -1;
        return exited;
    }

private:
    pid_t pid_ = -1;
    int in_ = -1;
    int out_ = -1;
    int next_id_ = 0;
    std::string buffer_;
};

std::string Text(const json& result) {
    return result["content"][0]["text"].get<std::string>();
}

bool Contains(const std::string& text, const std::string& part) {
    return text.find(part) != std::string::npos;
}

}  // namespace

int main(int argc, char* argv[]) {
    require(argc == 3, "usage: test_operator_mcp <hmi_runtime> <humanoid_operator_mcp>");
    const std::string hmi_runtime = argv[1];
    const std::string server = argv[2];

    char directory[] = "/tmp/operator-mcp-test-XXXXXX";
    require(mkdtemp(directory), "mkdtemp");
    const std::filesystem::path root(directory);
    require(setenv("XDG_STATE_HOME", (root / "state").c_str(), 1) == 0, "XDG_STATE_HOME");
    require(setenv("XDG_RUNTIME_DIR", root.c_str(), 1) == 0, "XDG_RUNTIME_DIR");
    const int port = 21000 + (getpid() % 2000) * 5;
    const auto config = (root / "config.yaml").string();
    std::ofstream(config) << "robot_base:\n  name: voice_mcp_test\n"
        "driver:\n  backend: whole_body\n"
        "operator_service:\n  port: " << port << "\n  lease_ms: 1000\n"
        "transport:\n  type: udp\n  udp:\n"
        "    driver_ip: 127.0.0.1\n    control_ip: 127.0.0.1\n    hmi_ip: 127.0.0.1\n"
        "    state_port: " << port + 1 << "\n    control_port: " << port + 2 << "\n"
        "    hmi_port: " << port + 3 << "\n    status_port: " << port + 4 << "\n"
        "rl_policy:\n  onnx_infer:\n    policy_names: [test_policy]\n    policies:\n      test_policy:\n"
        "        command:\n          limits: {max_vx: 0.1, max_vy: 0.1, max_wz: 0.2}\n"
        "        policy_adapter:\n          type: joint_trajectory\n"
        "          action_names: [wave, long_wave, refuse]\n          actions:\n"
        "            wave: {display_name: Wave}\n            long_wave: {display_name: Long Wave}\n"
        "            refuse: {display_name: Refuse}\n";
    const auto connection_file = (root / "state/humanoid-operator/voice_mcp_test/connection.json").string();

    // 服务没起来：工具报错而不是挂住。
    {
        McpProcess mcp(server, {"--connection", connection_file});
        mcp.Initialize();
        const json tools = mcp.Request("tools/list")["result"]["tools"];
        require(tools.size() == 2 && tools[0]["name"] == "perform_action" && tools[1]["name"] == "stop_action",
            "tools without a service: " + tools.dump());
        require(!tools[0]["inputSchema"]["properties"]["action"].contains("enum"), "no enum without a catalog");
        const json result = mcp.Call("perform_action", {{"action", "wave"}});
        require(result["isError"] == true && Contains(Text(result), "没有运行"), "service down: " + result.dump());
    }

    FakeControl control(config);
    const pid_t hmi = fork();
    require(hmi >= 0, "fork hmi_runtime");
    if (hmi == 0) {
        prctl(PR_SET_PDEATHSIG, SIGTERM);  // 测试中途失败退出时 hmi_runtime 跟着退，不留孤儿占住端口和输出管道
        execl(hmi_runtime.c_str(), hmi_runtime.c_str(), config.c_str(), "--sim", nullptr);
        _exit(127);
    }
    require(Wait([&]() { return access(connection_file.c_str(), R_OK) == 0; }, 10000), "hmi_runtime connection file");

    operator_service::Client phone;  // 扮演手机 / 终端
    auto connection = operator_service::Client::ReadConnectionFile(connection_file);
    connection.name = "phone";
    std::string error;
    require(phone.Connect(connection, &error), "observer connect: " + error);
    require(Wait([&]() { return phone.LatestStatus().online; }), "control online");

    {
        McpProcess mcp(server, {"--connection", connection_file, "--name", "voice-test"});
        mcp.Initialize();
        const json tools = mcp.Request("tools/list")["result"]["tools"];
        const json action = tools[0]["inputSchema"]["properties"]["action"];
        require(action["enum"] == json({"wave", "long_wave", "refuse"}), "live catalog: " + action.dump());
        require(Contains(tools[0]["description"].get<std::string>(), "wave=Wave"), "catalog names in description");

        // 正常动作：2.5 s 长于 1 s 租约，全程不能被取消；做完释放控制权。
        json result = mcp.Call("perform_action", {{"action", "wave"}});
        require(result["isError"] == false && Contains(Text(result), "Wave"), "perform wave: " + result.dump());
        require(control.starts() == 1 && phone.LatestStatus().owner.rfind("voice-test#", 0) == 0, "voice owns control while playing");
        require(Wait([&]() { return control.phase() == "FINISHED"; }, 5000), "wave finishes");
        require(control.cancels() == 0, "lease must be renewed during the action");
        require(Wait([&]() { return phone.LatestStatus().owner.empty(); }, 2000), "control released after the action");

        // 未知动作、被拒绝的动作。
        result = mcp.Call("perform_action", {{"action", "dance"}});
        require(result["isError"] == true && Contains(Text(result), "Long Wave"), "unknown action: " + result.dump());
        result = mcp.Call("perform_action", {{"action", "refuse"}});
        require(result["isError"] == true, "rejected action: " + result.dump());
        require(Wait([&]() { return phone.LatestStatus().owner.empty(); }, 2000), "released after rejection");

        // 手机持有控制权：不抢。
        require(phone.AcquireControl().ok, "phone acquires");
        result = mcp.Call("perform_action", {{"action", "wave"}});
        require(result["isError"] == true && Contains(Text(result), "正在手动控制"), "busy: " + result.dump());
        require(phone.ReleaseControl().ok, "phone releases");

        // 播放中再要一个动作：报忙，原动作不受影响。
        const int cancels_before = control.cancels();
        result = mcp.Call("perform_action", {{"action", "long_wave"}});
        require(result["isError"] == false, "perform long_wave: " + result.dump());
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        result = mcp.Call("perform_action", {{"action", "wave"}});
        require(result["isError"] == true && Contains(Text(result), "正在做「Long Wave」"), "busy while playing: " + result.dump());
        std::this_thread::sleep_for(std::chrono::milliseconds(1500));
        require(control.cancels() == cancels_before && control.phase() == "PLAYING",
            "a refused request must not drop the running action's lease");

        // 停止：取消动作、撤销控制权。
        result = mcp.Call("stop_action");
        require(result["isError"] == false, "stop: " + result.dump());
        require(Wait([&]() { return control.cancels() == cancels_before + 1; }), "stop cancels the action");
        require(Wait([&]() { return phone.LatestStatus().owner.empty(); }), "stop clears the owner");
        // 等客户端看到的状态（推送有 ~100 ms 延迟）也回到 IDLE；BLEND_OUT 期间报忙是对的。
        require(Wait([&]() { return phone.LatestStatus().interaction_phase == "IDLE"; }),
            "action blended out: phone sees " + phone.LatestStatus().interaction_phase + " online=" +
            std::to_string(phone.LatestStatus().online) + " connected=" + std::to_string(phone.Connected()) +
            " error=" + phone.LastError() + " age_ms=" + std::to_string(phone.LatestStatus().age_ms) +
            " state=" + phone.LatestStatus().state + " request=" + phone.LatestStatus().request.operation + "/" +
            phone.LatestStatus().request.phase + " hmi_alive=" + std::to_string(kill(hmi, 0) == 0 &&
            waitpid(hmi, nullptr, WNOHANG) == 0) + ", control has " + control.phase());

        // MCP 客户端退出时动作还在做：取消并释放，不留没人续租的动作。
        result = mcp.Call("perform_action", {{"action", "long_wave"}});
        require(result["isError"] == false, "perform before exit: " + result.dump());
        const int cancels_at_exit = control.cancels();
        mcp.CloseInput();
        require(mcp.WaitExit(5000), "server exits on stdin EOF");
        require(Wait([&]() { return control.cancels() == cancels_at_exit + 1; }), "exit cancels the action");
        require(Wait([&]() { return phone.LatestStatus().owner.empty(); }), "exit releases control");
    }

    // 允许列表 + 自定义显示名：只列配置的动作，名字用配置的。
    {
        require(Wait([&]() { return phone.LatestStatus().interaction_phase == "IDLE"; }), "idle before allow-list test");
        McpProcess mcp(server, {"--connection", connection_file, "--actions", "wave=挥手, missing"});
        mcp.Initialize();
        const json action = mcp.Request("tools/list")["result"]["tools"][0]["inputSchema"]["properties"]["action"];
        require(action["enum"] == json({"wave", "missing"}), "configured enum: " + action.dump());
        json result = mcp.Call("perform_action", {{"action", "long_wave"}});
        require(result["isError"] == true && Contains(Text(result), "挥手"), "not allowed: " + result.dump());
        result = mcp.Call("perform_action", {{"action", "missing"}});
        require(result["isError"] == true, "configured but not on the robot: " + result.dump());
        result = mcp.Call("perform_action", {{"action", "wave"}});
        require(result["isError"] == false && Text(result) == "已开始做「挥手」。", "display name: " + result.dump());
        require(Wait([&]() { return phone.LatestStatus().owner.empty(); }, 5000), "released after allowed action");
    }

    phone.Disconnect();
    kill(hmi, SIGTERM);
    waitpid(hmi, nullptr, 0);
    std::filesystem::remove_all(root);
    std::cout << "test_operator_mcp: all passed" << std::endl;
    return 0;
}
