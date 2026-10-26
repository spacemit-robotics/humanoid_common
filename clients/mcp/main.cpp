/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file main.cpp
 * @brief Stdio MCP entry point for the humanoid operator client.
 */

/*
 * humanoid_operator_mcp：stdio MCP 服务，由 MCP 客户端作为子进程拉起。
 *
 *   humanoid_operator_mcp --robot <name> [--actions wave_hello,heart_both=比心] [--name voice]
 *   humanoid_operator_mcp --connection /path/to/connection.json
 */

#include <signal.h>
#include <unistd.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>

#include "mcp_stdio_server.hpp"
#include "operator_tools.hpp"

namespace {

constexpr char kVersion[] = "0.1.0";

void Usage(const char* program) {
    std::cerr << "用法: " << program << " (--robot <name> | --connection <file>) [选项]\n"
        << "  --robot <name>          机型名，连接文件取\n"
        << "                          ${XDG_STATE_HOME:-~/.local/state}/humanoid-operator/<name>/connection.json\n"
        << "  --connection <file>     直接指定 hmi_runtime 写出的连接文件\n"
        << "  --actions <list>        允许的动作，逗号分隔，可写 key=显示名；默认机器人当前动作目录里的全部\n"
        << "  --name <name>           在操作服务里显示的客户端名 (默认: voice)\n"
        << "  --renew-ms <ms>         动作期间续租间隔 (默认: 300，须小于服务的 lease_ms)\n"
        << "stdin/stdout 是 MCP 通道（每行一条 JSON-RPC），日志写 stderr。\n";
}

std::string DefaultConnectionFile(const std::string& robot) {
    const char* state = std::getenv("XDG_STATE_HOME");
    std::string root;
    if (state && *state) {
        root = state;
    } else {
        const char* home = std::getenv("HOME");
        root = std::string(home ? home : "") + "/.local/state";
    }
    return root + "/humanoid-operator/" + robot + "/connection.json";
}

void OnTerminate(int) {
    // 不设 SA_RESTART：阻塞中的 read 返回 EINTR，主循环按 EOF 退出，析构里取消动作、释放控制权。
}

}  // namespace

int main(int argc, char* argv[]) {
    humanoid_mcp::OperatorToolsOptions options;
    std::string robot;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const bool has_value = i + 1 < argc;
        if (arg == "--robot" && has_value) {
            robot = argv[++i];
        } else if (arg == "--connection" && has_value) {
            options.connection_file = argv[++i];
        } else if (arg == "--actions" && has_value) {
            options.actions = humanoid_mcp::ParseActionList(argv[++i]);
        } else if (arg == "--name" && has_value) {
            options.client_name = argv[++i];
        } else if (arg == "--renew-ms" && has_value) {
            options.renew_ms = std::atoi(argv[++i]);
        } else if (arg == "--help" || arg == "-h") {
            Usage(argv[0]);
            return 0;
        } else {
            std::cerr << "未知参数: " << arg << "\n";
            Usage(argv[0]);
            return 2;
        }
    }
    if (options.connection_file.empty() && !robot.empty()) options.connection_file = DefaultConnectionFile(robot);
    if (options.connection_file.empty() || options.renew_ms < 50 || options.renew_ms > 900) {
        Usage(argv[0]);
        return 2;
    }

    // stdout 专用于 MCP：协议走一个复制出来的 fd，fd 1 指向 stderr，任何库往 stdout 打印都不会弄坏协议。
    const int protocol_fd = dup(STDOUT_FILENO);
    std::FILE* protocol = protocol_fd >= 0 ? fdopen(protocol_fd, "w") : nullptr;
    if (!protocol || dup2(STDERR_FILENO, STDOUT_FILENO) < 0) {
        std::perror("humanoid_operator_mcp: stdout");
        return 1;
    }
    signal(SIGPIPE, SIG_IGN);
    struct sigaction action {};
    action.sa_handler = OnTerminate;
    sigemptyset(&action.sa_mask);
    sigaction(SIGTERM, &action, nullptr);
    sigaction(SIGINT, &action, nullptr);

    std::cerr << "[humanoid_operator_mcp] " << kVersion << " connection=" << options.connection_file << "\n";
    {
        humanoid_mcp::OperatorTools tools(options);
        humanoid_mcp::McpStdioServer server("humanoid_operator_mcp", kVersion, &tools);
        server.Run(stdin, protocol);
    }
    std::fclose(protocol);
    return 0;
}
