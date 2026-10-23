/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mcp_stdio_server.hpp
 * @brief Robot-independent stdio MCP server and tool interfaces.
 */

/*
 * 最小 MCP 服务端（JSON-RPC 2.0，stdio，每行一条消息）
 *
 * 只实现工具相关的方法：initialize、ping、tools/list、tools/call；通知一律忽略。
 * 与机器人无关，具体工具由 ToolSet 提供。
 */

#ifndef MCP_STDIO_SERVER_HPP
#define MCP_STDIO_SERVER_HPP

#include <cstdio>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace humanoid_mcp {

struct ToolSpec {
    std::string name;
    std::string description;
    nlohmann::json input_schema = {{"type", "object"}, {"properties", nlohmann::json::object()}};
};

struct ToolResult {
    bool is_error = false;
    std::string text;
};

class ToolSet {
public:
    virtual ~ToolSet() = default;
    virtual std::vector<ToolSpec> List() = 0;
    // 工具名是否存在；要便宜，不能连机器人。
    virtual bool HasTool(const std::string& name) const = 0;
    // name 一定满足 HasTool；args 一定是 JSON object。
    virtual ToolResult Call(const std::string& name, const nlohmann::json& args) = 0;
    // 每处理完一条消息调用一次；返回 true 时服务端发 notifications/tools/list_changed。
    virtual bool TakeListChanged() { return false; }
};

class McpStdioServer {
public:
    McpStdioServer(std::string name, std::string version, ToolSet* tools);

    // 处理一行输入，返回要写回的行（不含换行；可能为空，或一条回复加一条通知）。
    std::vector<std::string> HandleLine(const std::string& line);

    // 从 in 逐行读到 EOF，回复写到 out。
    void Run(std::FILE* in, std::FILE* out);

private:
    nlohmann::json HandleRequest(const std::string& method, const nlohmann::json& params);

    std::string name_;
    std::string version_;
    ToolSet* tools_;
};

}  // namespace humanoid_mcp

#endif  // MCP_STDIO_SERVER_HPP
