/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_operator_mcp_protocol.cpp
 * @brief Offline tests for MCP JSON-RPC and stdio framing.
 */

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

#include "mcp_stdio_server.hpp"

using json = nlohmann::json;
using humanoid_mcp::McpStdioServer;
using humanoid_mcp::ToolResult;
using humanoid_mcp::ToolSpec;

namespace {

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "ASSERTION FAILED: " << message << std::endl;
        std::exit(1);
    }
}

class FakeTools : public humanoid_mcp::ToolSet {
public:
    std::vector<ToolSpec> List() override {
        ++list_calls;
        ToolSpec echo;
        echo.name = "echo";
        echo.description = "echo text";
        echo.input_schema = {{"type", "object"}, {"properties", {{"text", {{"type", "string"}}}}}};
        return {echo};
    }
    bool HasTool(const std::string& name) const override { return name == "echo"; }
    ToolResult Call(const std::string&, const json& args) override {
        last_args = args;
        const std::string text = args.value("text", "");
        if (text == "throw") throw std::runtime_error("boom");
        if (text == "bad-utf8") return {false, "\xff"};
        return {text == "fail", text};
    }
    bool TakeListChanged() override {
        const bool changed = pending_change;
        pending_change = false;
        return changed;
    }

    int list_calls = 0;
    bool pending_change = false;
    json last_args;
};

json Request(const json& id, const std::string& method, const json& params = json::object()) {
    return {{"jsonrpc", "2.0"}, {"id", id}, {"method", method}, {"params", params}};
}

json One(McpStdioServer* server, const json& message) {
    const auto lines = server->HandleLine(message.dump());
    require(lines.size() == 1, "expected exactly one line for " + message.dump());
    return json::parse(lines[0]);
}

void testHandshakeAndList() {
    FakeTools tools;
    McpStdioServer server("test_server", "1.2.3", &tools);

    json reply = One(&server, Request(1, "initialize", {{"protocolVersion", "2024-11-05"},
        {"capabilities", json::object()}, {"clientInfo", {{"name", "c"}, {"version", "1"}}}}));
    require(reply["id"] == 1 && reply["result"]["protocolVersion"] == "2024-11-05", "initialize echoes 2024-11-05");
    require(reply["result"]["capabilities"]["tools"]["listChanged"] == true, "tools capability");
    require(reply["result"]["serverInfo"]["name"] == "test_server" &&
        reply["result"]["serverInfo"]["version"] == "1.2.3", "server info");
    reply = One(&server, Request("s", "initialize", {{"protocolVersion", "2025-06-18"}}));
    require(reply["id"] == "s" && reply["result"]["protocolVersion"] == "2025-06-18", "newer supported version");
    reply = One(&server, Request(2, "initialize", {{"protocolVersion", "1999-01-01"}}));
    require(reply["result"]["protocolVersion"] == "2024-11-05", "unknown version falls back");

    require(server.HandleLine(R"({"jsonrpc":"2.0","method":"notifications/initialized"})").empty(),
        "notifications get no reply");
    require(server.HandleLine(R"({"jsonrpc":"2.0","id":9,"result":{}})").empty(), "client responses are ignored");

    reply = One(&server, Request(3, "ping"));
    require(reply["result"].is_object() && reply["result"].empty(), "ping");

    reply = One(&server, Request(4, "tools/list"));
    const auto& tool = reply["result"]["tools"][0];
    require(tool["name"] == "echo" && tool["description"] == "echo text" &&
        tool["inputSchema"]["properties"]["text"]["type"] == "string", "tools/list: " + reply.dump());
}

void testCalls() {
    FakeTools tools;
    McpStdioServer server("test_server", "1", &tools);

    json reply = One(&server, Request(1, "tools/call", {{"name", "echo"}, {"arguments", {{"text", "你好"}}}}));
    require(reply["result"]["isError"] == false && reply["result"]["content"][0]["type"] == "text" &&
        reply["result"]["content"][0]["text"] == "你好", "successful call: " + reply.dump());
    reply = One(&server, Request(2, "tools/call", {{"name", "echo"}, {"arguments", {{"text", "fail"}}}}));
    require(reply["result"]["isError"] == true && reply["result"]["content"][0]["text"] == "fail",
        "tool errors are results with isError");
    reply = One(&server, Request(3, "tools/call", {{"name", "echo"}}));
    require(reply["result"]["isError"] == false && tools.last_args.is_object() && tools.last_args.empty(),
        "missing arguments become {}");

    reply = One(&server, Request(4, "tools/call", {{"name", "nope"}}));
    require(reply["error"]["code"] == -32602, "unknown tool is invalid params");
    reply = One(&server, Request(5, "tools/call", {{"arguments", json::object()}}));
    require(reply["error"]["code"] == -32602, "missing name");
    reply = One(&server, Request(6, "tools/call", {{"name", "echo"}, {"arguments", json::array()}}));
    require(reply["error"]["code"] == -32602, "arguments must be an object");
    reply = One(&server, Request(7, "tools/call", {{"name", "echo"}, {"arguments", {{"text", "throw"}}}}));
    require(reply["error"]["code"] == -32603 && reply["error"]["message"] == "boom", "exceptions are internal errors");
    reply = One(&server, Request(8, "resources/list"));
    require(reply["error"]["code"] == -32601, "unknown method");

    reply = json::parse(server.HandleLine("{not json")[0]);
    require(reply["error"]["code"] == -32700 && reply["id"].is_null(), "parse error");
    reply = json::parse(server.HandleLine("[1,2]")[0]);
    require(reply["error"]["code"] == -32600, "batches are not supported");
    reply = json::parse(server.HandleLine(R"({"jsonrpc":"1.0","id":3,"method":"ping"})")[0]);
    require(reply["error"]["code"] == -32600 && reply["id"] == 3, "wrong jsonrpc version");

    reply = One(&server, Request(9, "tools/call", {{"name", "echo"}, {"arguments", {{"text", "bad-utf8"}}}}));
    require(reply["result"]["content"][0]["text"].is_string(), "invalid UTF-8 is replaced, not thrown");

    tools.pending_change = true;
    const auto lines = server.HandleLine(Request(10, "ping").dump());
    require(lines.size() == 2 && json::parse(lines[0])["id"] == 10 &&
        json::parse(lines[1])["method"] == "notifications/tools/list_changed",
        "list_changed follows the response");
}

void testRunLoop() {
    FakeTools tools;
    McpStdioServer server("test_server", "1", &tools);
    std::FILE* in = std::tmpfile();
    std::FILE* out = std::tmpfile();
    require(in && out, "tmpfile");
    const std::string input = Request(1, "ping").dump() + "\r\n\n" +
        R"({"jsonrpc":"2.0","method":"notifications/initialized"})" + "\n" +
        Request(2, "tools/call", {{"name", "echo"}, {"arguments", {{"text", "x"}}}}).dump() + "\n" +
        Request(3, "ping").dump();  // 最后一行没有换行：EOF 时不处理半行
    std::fputs(input.c_str(), in);
    std::rewind(in);
    server.Run(in, out);
    std::rewind(out);
    std::vector<json> replies;
    char buf[4096];
    while (std::fgets(buf, sizeof(buf), out)) replies.push_back(json::parse(buf));
    require(replies.size() == 2 && replies[0]["id"] == 1 && replies[1]["id"] == 2 &&
        replies[1]["result"]["content"][0]["text"] == "x", "run loop handles CRLF, blank lines and notifications");
    std::fclose(in);
    std::fclose(out);
}

}  // namespace

int main() {
    testHandshakeAndList();
    testCalls();
    testRunLoop();
    std::cout << "test_operator_mcp_protocol: all passed" << std::endl;
    return 0;
}
