/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file operator_tools.cpp
 * @brief Operator action tools and control lease management.
 */

#include "operator_tools.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace humanoid_mcp {

using json = nlohmann::json;
using Clock = std::chrono::steady_clock;

namespace {

constexpr char kPerformAction[] = "perform_action";
constexpr char kStopAction[] = "stop_action";

bool InteractionBusy(const std::string& phase) {
    return phase == "BLEND_IN" || phase == "PLAYING" || phase == "HOLDING" || phase == "BLEND_OUT";
}

std::string Label(const ActionChoice& action) {
    return action.display_name.empty() ? action.key : action.display_name;
}

std::string Join(const std::vector<ActionChoice>& actions, bool with_keys) {
    std::string text;
    for (const auto& action : actions) {
        if (!text.empty()) text += "、";
        text += with_keys && !action.display_name.empty() ? action.key + "=" + action.display_name : Label(action);
    }
    return text;
}

std::vector<std::string> Keys(const std::vector<ActionChoice>& actions) {
    std::vector<std::string> keys;
    for (const auto& action : actions) keys.push_back(action.key);
    return keys;
}

std::string Trim(const std::string& text) {
    const auto begin = text.find_first_not_of(" \t");
    if (begin == std::string::npos) return "";
    return text.substr(begin, text.find_last_not_of(" \t") - begin + 1);
}

ToolResult Error(const std::string& text) {
    return {true, text};
}

}  // namespace

std::vector<ActionChoice> ParseActionList(const std::string& text) {
    std::vector<ActionChoice> actions;
    size_t start = 0;
    while (start <= text.size()) {
        const size_t comma = text.find(',', start);
        const std::string item = Trim(text.substr(start, comma == std::string::npos ? std::string::npos : comma - start));
        if (!item.empty()) {
            const size_t equal = item.find('=');
            ActionChoice action;
            action.key = Trim(item.substr(0, equal));
            if (equal != std::string::npos) action.display_name = Trim(item.substr(equal + 1));
            if (!action.key.empty()) actions.push_back(action);
        }
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return actions;
}

OperatorTools::OperatorTools(OperatorToolsOptions options) : options_(std::move(options)) {}

OperatorTools::~OperatorTools() {
    bool running = false;
    {
        std::lock_guard<std::mutex> lock(lease_mutex_);
        running = lease_thread_.joinable();
    }
    // MCP 客户端退出时动作还在做：释放控制权，操作服务释放时会取消动作，不留一个没人续租的动作。
    StopLease(running);
    client_.Disconnect();
}

bool OperatorTools::HasTool(const std::string& name) const {
    return name == kPerformAction || name == kStopAction;
}

bool OperatorTools::EnsureConnected(std::string* error) {
    if (connected_ && client_.Connected()) return true;
    // Connect 不能和其他调用并发：先停掉续租线程。断线后服务端已经取消了动作、撤销了控制权。
    StopLease(false);
    connected_ = false;
    operator_service::Connection connection;
    try {
        connection = operator_service::Client::ReadConnectionFile(options_.connection_file);
    } catch (const std::exception&) {
        *error = "机器人控制服务没有运行（找不到 " + options_.connection_file + "）。";
        return false;
    }
    connection.name = options_.client_name;
    std::string connect_error;
    if (!client_.Connect(connection, &connect_error)) {
        *error = "连不上机器人控制服务：" + connect_error;
        return false;
    }
    connected_ = true;
    return true;
}

std::vector<ActionChoice> OperatorTools::Available(const operator_service::Status& status) const {
    std::vector<operator_service::Action> live;
    if (status.trajectory_enabled) {
        live = client_.Actions();
    } else {
        for (const auto& policy : client_.Policies()) {
            if (policy.name == status.policy) live = policy.actions;
        }
    }
    std::vector<ActionChoice> available;
    if (options_.actions.empty()) {
        for (const auto& action : live) available.push_back({action.key, action.display_name});
        return available;
    }
    for (const auto& wanted : options_.actions) {
        const auto it = std::find_if(live.begin(), live.end(),
            [&](const operator_service::Action& action) { return action.key == wanted.key; });
        if (it == live.end()) continue;
        available.push_back({wanted.key, wanted.display_name.empty() ? it->display_name : wanted.display_name});
    }
    return available;
}

std::vector<ToolSpec> OperatorTools::List() {
    std::vector<ActionChoice> actions = options_.actions;
    std::string unavailable;
    if (actions.empty()) {
        // 没配允许列表：用机器人当前的动作目录（连不上就先不列，之后通过 list_changed 补上）。
        if (EnsureConnected(&unavailable)) actions = Available(client_.LatestStatus());
    }
    served_keys_ = Keys(actions);

    std::string description =
        "让机器人原地做一个上半身动作（只动手臂，不走路）。调用前先用一句十二个字左右的话告诉用户要做什么并提醒"
        "注意安全，比如“好的，我来挥手，请注意安全。”。动作被机器人确认开始后就返回，不等做完。";
    json action_schema = {{"type", "string"}, {"description", "动作名"}};
    if (actions.empty()) {
        description += "现在拿不到动作列表（机器人控制服务没有连上，或当前模式没有上半身动作）。";
    } else {
        description += "可选动作：" + Join(actions, true) + "。";
        action_schema["enum"] = Keys(actions);
    }

    ToolSpec perform;
    perform.name = kPerformAction;
    perform.description = description;
    perform.input_schema = {
        {"type", "object"},
        {"properties", {{"action", action_schema}}},
        {"required", {"action"}},
    };
    ToolSpec stop;
    stop.name = kStopAction;
    stop.description =
        "立即停止机器人当前的动作和速度。用户说“停”“停下”“别动了”时调用。手机或终端正在控制时也会让它停下。";
    return {perform, stop};
}

bool OperatorTools::TakeListChanged() {
    if (!options_.actions.empty() || !connected_ || !client_.Connected()) return false;
    const auto keys = Keys(Available(client_.LatestStatus()));
    if (keys == served_keys_ || keys == notified_keys_) return false;
    notified_keys_ = keys;
    return true;
}

ToolResult OperatorTools::Call(const std::string& name, const json& args) {
    if (name == kStopAction) return StopAction();
    const auto action = args.find("action");
    if (action == args.end() || !action->is_string() || action->get<std::string>().empty()) {
        return Error("缺少参数 action（动作名）。");
    }
    return PerformAction(action->get<std::string>());
}

ToolResult OperatorTools::PerformAction(const std::string& key) {
    std::string error;
    if (!EnsureConnected(&error)) return Error(error);

    const auto status = client_.LatestStatus();
    if (!status.online) return Error("机器人控制链路不在线，现在不能做动作。");
    const auto available = Available(status);
    const auto action = std::find_if(available.begin(), available.end(),
        [&](const ActionChoice& choice) { return choice.key == key; });
    if (action == available.end()) {
        if (available.empty()) {
            return Error("机器人当前模式没有可用的上半身动作，需要先用手机或终端让它站好并切到动作策略。");
        }
        return Error("现在做不了「" + key + "」。可以做的动作有：" + Join(available, false) + "。");
    }
    // 先判断忙再动续租线程：正在播放时停掉续租，1 s 后操作服务就会取消这个动作。
    if (InteractionBusy(status.interaction_phase)) {
        const auto playing = std::find_if(available.begin(), available.end(),
            [&](const ActionChoice& choice) { return choice.key == status.interaction_action; });
        const std::string name = playing == available.end() ? status.interaction_action : Label(*playing);
        return Error("机器人正在做「" + name + "」，等它做完再来。");
    }
    StopLease(false);  // 上一个动作的续租线程（可能还在宽限期里）先收掉，控制权留着

    const auto acquired = client_.AcquireControl();  // 已经是自己的控制权时也会成功
    if (!acquired.ok) {
        if (acquired.code == "busy") return Error("有人正在手动控制机器人，等他释放控制权后我才能做动作。");
        return Error("拿不到机器人控制权：" + acquired.message);
    }
    StartLease();
    const auto reply = client_.StartInteraction(key);
    if (!reply.ok) {
        StopLease(true);
        return Error("机器人没有接受这个动作：" + reply.message);
    }

    // reply 只表示操作服务已转发；以 control 回报的请求结果或动作阶段为准。动作阶段可能还停在上一个
    // 请求的 REJECTED / FINISHED 上，所以拒绝只按本次请求的序号判断。
    const auto deadline = Clock::now() + std::chrono::milliseconds(options_.start_timeout_ms);
    while (Clock::now() < deadline) {
        const auto now = client_.LatestStatus();
        if (InteractionBusy(now.interaction_phase) && now.interaction_action == key) break;
        if (now.request.sequence == reply.request.sequence) {
            if (now.request.phase == "completed") break;
            if (now.request.phase != "accepted") {
                return Error("机器人没有执行这个动作（" + now.request.phase + "：" + now.request.message + "）。");
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    if (Clock::now() >= deadline) {
        return Error("机器人没有确认开始「" + Label(*action) + "」，请看一下它的状态。");
    }
    return {false, "已开始做「" + Label(*action) + "」。"};
}

ToolResult OperatorTools::StopAction() {
    std::string error;
    if (!EnsureConnected(&error)) return Error(error + "请用手机或终端停止机器人。");
    const auto reply = client_.Stop();
    StopLease(false);  // stop 已经撤销了控制权
    if (!reply.ok) return Error("停止失败：" + reply.message + "。请用手机或终端停止机器人。");
    return {false, "已停止：动作已取消，速度已清零。"};
}

void OperatorTools::StartLease() {
    StopLease(false);
    std::lock_guard<std::mutex> lock(lease_mutex_);
    lease_stop_ = false;
    lease_thread_ = std::thread([this]() { LeaseLoop(); });
}

void OperatorTools::StopLease(bool release) {
    std::thread thread;
    {
        std::lock_guard<std::mutex> lock(lease_mutex_);
        lease_stop_ = true;
        thread = std::move(lease_thread_);
    }
    lease_cv_.notify_all();
    if (thread.joinable()) thread.join();
    if (release && connected_) client_.ReleaseControl();
}

// 续租直到动作做完（请求已结束且不在播放），然后释放控制权。被别人 stop、DAMP 或断线时
// renew 会失败，直接退出：此时控制权已经不在我们手里。
void OperatorTools::LeaseLoop() {
    const auto started = Clock::now();
    const auto grace = std::chrono::milliseconds(500);  // 刚发出请求时状态推送可能还没跟上
    bool cancel_sent = false;
    std::unique_lock<std::mutex> lock(lease_mutex_);
    while (!lease_stop_) {
        lease_cv_.wait_for(lock, std::chrono::milliseconds(options_.renew_ms), [this]() { return lease_stop_; });
        if (lease_stop_) break;
        lock.unlock();
        bool keep = client_.Connected();
        if (keep) {
            const auto status = client_.LatestStatus();
            const auto elapsed = Clock::now() - started;
            const bool pending = status.request.phase == "accepted";
            if (!pending && !InteractionBusy(status.interaction_phase) && elapsed > grace) {
                client_.ReleaseControl();
                keep = false;
            } else {
                if (!cancel_sent && elapsed > std::chrono::milliseconds(options_.max_action_ms)) {
                    client_.CancelInteraction();
                    cancel_sent = true;
                }
                keep = client_.RenewControl().ok;
            }
        }
        lock.lock();
        if (!keep) break;
    }
}

}  // namespace humanoid_mcp
