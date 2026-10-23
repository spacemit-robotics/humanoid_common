/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file mcp_stdio_server.cpp
 * @brief Line-oriented JSON-RPC handling for MCP tools.
 */

#include "mcp_stdio_server.hpp"

#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace humanoid_mcp {

using json = nlohmann::json;

namespace {

constexpr int kParseError = -32700;
constexpr int kInvalidRequest = -32600;
constexpr int kMethodNotFound = -32601;
constexpr int kInvalidParams = -32602;
constexpr int kInternalError = -32603;

// 客户端请求的版本我们支持就原样回，否则回最老的那个（SDK 的 C++ 客户端用 2024-11-05）。
const char* const kProtocolVersions[] = {"2025-06-18", "2025-03-26", "2024-11-05"};

struct RpcError {
    int code;
    std::string message;
};

std::string Dump(const json& message) {
    return message.dump(-1, ' ', false, json::error_handler_t::replace);
}

json ErrorResponse(const json& id, int code, const std::string& message) {
    return {{"jsonrpc", "2.0"}, {"id", id}, {"error", {{"code", code}, {"message", message}}}};
}

}  // namespace

McpStdioServer::McpStdioServer(std::string name, std::string version, ToolSet* tools)
    : name_(std::move(name)), version_(std::move(version)), tools_(tools) {}

json McpStdioServer::HandleRequest(const std::string& method, const json& params) {
    if (method == "initialize") {
        std::string version = kProtocolVersions[2];
        const std::string requested = params.value("protocolVersion", "");
        for (const char* supported : kProtocolVersions) {
            if (requested == supported) version = supported;
        }
        return {
            {"protocolVersion", version},
            {"capabilities", {{"tools", {{"listChanged", true}}}}},
            {"serverInfo", {{"name", name_}, {"version", version_}}},
        };
    }
    if (method == "ping") return json::object();
    if (method == "tools/list") {
        json tools = json::array();
        for (const auto& tool : tools_->List()) {
            tools.push_back({
                {"name", tool.name},
                {"description", tool.description},
                {"inputSchema", tool.input_schema},
            });
        }
        return {{"tools", tools}};
    }
    if (method == "tools/call") {
        const auto name = params.find("name");
        if (name == params.end() || !name->is_string()) throw RpcError{kInvalidParams, "params.name must be a string"};
        json args = json::object();
        if (auto it = params.find("arguments"); it != params.end() && !it->is_null()) {
            if (!it->is_object()) throw RpcError{kInvalidParams, "params.arguments must be an object"};
            args = *it;
        }
        if (!tools_->HasTool(name->get<std::string>())) {
            throw RpcError{kInvalidParams, "unknown tool: " + name->get<std::string>()};
        }
        const ToolResult result = tools_->Call(name->get<std::string>(), args);
        return {
            {"content", json::array({{{"type", "text"}, {"text", result.text}}})},
            {"isError", result.is_error},
        };
    }
    throw RpcError{kMethodNotFound, "method not found: " + method};
}

std::vector<std::string> McpStdioServer::HandleLine(const std::string& line) {
    std::vector<std::string> out;
    const json message = json::parse(line, nullptr, false);
    if (message.is_discarded()) {
        out.push_back(Dump(ErrorResponse(nullptr, kParseError, "parse error")));
        return out;
    }
    if (!message.is_object() || message.value("jsonrpc", "") != "2.0" ||
            !message.contains("method") || !message["method"].is_string()) {
        // 不是请求或通知（比如客户端发来的 response），有 id 才回错误。
        if (message.is_object() && message.contains("id") && !message.contains("result") &&
                !message.contains("error")) {
            out.push_back(Dump(ErrorResponse(message["id"], kInvalidRequest, "invalid request")));
        } else if (!message.is_object()) {
            out.push_back(Dump(ErrorResponse(nullptr, kInvalidRequest, "invalid request")));
        }
        return out;
    }
    const std::string method = message["method"].get<std::string>();
    const bool is_request = message.contains("id") && !message["id"].is_null();
    if (is_request) {
        const json& id = message["id"];
        json params = json::object();
        if (auto it = message.find("params"); it != message.end() && it->is_object()) params = *it;
        try {
            out.push_back(Dump({{"jsonrpc", "2.0"}, {"id", id}, {"result", HandleRequest(method, params)}}));
        } catch (const RpcError& e) {
            out.push_back(Dump(ErrorResponse(id, e.code, e.message)));
        } catch (const std::exception& e) {
            out.push_back(Dump(ErrorResponse(id, kInternalError, e.what())));
        }
    }
    if (tools_->TakeListChanged()) {
        out.push_back(Dump({{"jsonrpc", "2.0"}, {"method", "notifications/tools/list_changed"}}));
    }
    return out;
}

void McpStdioServer::Run(std::FILE* in, std::FILE* out) {
    std::string line;
    for (int c = std::fgetc(in); c != EOF; c = std::fgetc(in)) {
        if (c != '\n') {
            line.push_back(static_cast<char>(c));
            continue;
        }
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!line.empty()) {
            for (const auto& reply : HandleLine(line)) {
                std::fputs(reply.c_str(), out);
                std::fputc('\n', out);
            }
            std::fflush(out);
        }
        line.clear();
    }
}

}  // namespace humanoid_mcp
