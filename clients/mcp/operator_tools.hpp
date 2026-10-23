/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file operator_tools.hpp
 * @brief MCP action tools backed by the public operator client.
 */

/*
 * 把 humanoid_common 操作服务的上半身交互动作包装成 MCP 工具
 *
 *   perform_action(action)：拿控制权 → 开始动作 → 动作播放期间每 renew_ms 续租 → 做完释放。
 *                           动作被 control 确认开始后就返回，不等做完。
 *   stop_action()：操作服务的 stop（清零速度、取消动作、撤销控制权；任何已认证客户端都能发）。
 *
 * 只用 liboperator_client 公开接口，和手机 / 网页 / 终端是同一层的客户端；
 * 不切状态、不切策略、不发速度。
 */

#ifndef OPERATOR_TOOLS_HPP
#define OPERATOR_TOOLS_HPP

#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "mcp_stdio_server.hpp"
#include "operator_client.h"

namespace humanoid_mcp {

struct ActionChoice {
    std::string key;
    std::string display_name;  // 空 = 用机器人动作目录里的名字
};

struct OperatorToolsOptions {
    std::string connection_file;
    std::string client_name = "voice";
    std::vector<ActionChoice> actions;  // 允许的动作；空 = 机器人当前动作目录里的全部
    int renew_ms = 300;
    int start_timeout_ms = 3000;
    int max_action_ms = 60000;  // 超过就发 cancel，防止异常情况下一直占着控制权
};

class OperatorTools : public ToolSet {
public:
    explicit OperatorTools(OperatorToolsOptions options);
    ~OperatorTools() override;

    OperatorTools(const OperatorTools&) = delete;
    OperatorTools& operator=(const OperatorTools&) = delete;

    std::vector<ToolSpec> List() override;
    bool HasTool(const std::string& name) const override;
    ToolResult Call(const std::string& name, const nlohmann::json& args) override;
    bool TakeListChanged() override;

private:
    ToolResult PerformAction(const std::string& action);
    ToolResult StopAction();
    bool EnsureConnected(std::string* error);
    // 机器人当前能做、且在允许列表里的动作。
    std::vector<ActionChoice> Available(const operator_service::Status& status) const;
    void StartLease();
    void StopLease(bool release);
    void LeaseLoop();

    const OperatorToolsOptions options_;
    operator_service::Client client_;
    bool connected_ = false;
    std::vector<std::string> served_keys_;    // 上次 tools/list 列出的动作
    std::vector<std::string> notified_keys_;  // 上次发 list_changed 时的动作

    std::mutex lease_mutex_;
    std::condition_variable lease_cv_;
    bool lease_stop_ = false;
    std::thread lease_thread_;
};

// 解析 "wave_hello" 或 "wave_hello=挥手"，逗号分隔。
std::vector<ActionChoice> ParseActionList(const std::string& text);

}  // namespace humanoid_mcp

#endif  // OPERATOR_TOOLS_HPP
