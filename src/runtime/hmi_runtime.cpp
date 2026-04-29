/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file hmi_demo.cpp
 * @brief 人机接口演示程序
 *
 * 该程序提供键盘交互界面，允许用户通过键盘控制机器人的行为状态转换。
 * 它捕获键盘输入，转换为 Command，并通过 transport_executor 发送给 control_demo。
 *
 * 支持的键盘命令：
 * - 'f': 任意状态 → POWER_OFF (完全失力)
 * - 'o'  : → DAMP (阻尼保持)
 * - 'z'  : → ZERO (回零位)
 * - 'r'  : → RL (进入 RL 控制)
 * - '1'~'9': 切换到对应序号的策略
 * - 'w/s': 前进/后退 (vx ± 0.1)
 * - 'a/d': 左/右 (vy ± 0.1)
 * - 'q/e': 旋转 (wz ± 0.1)
 * - '空格': 速度清零
 *
 * 调用的模块：
 * - transport_executor: 统一传输接口
 * - robot_base: Command 数据结构
 */

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "transport_executor.h"
#include "robot_base.h"
namespace {
volatile std::sig_atomic_t g_running = 1;

void OnSignal(int) {
    g_running = 0;
}

class TerminalRawGuard {
public:
    TerminalRawGuard() {
        tcgetattr(STDIN_FILENO, &old_);
        termios raw = old_;
        raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 1;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    }
    ~TerminalRawGuard() { tcsetattr(STDIN_FILENO, TCSANOW, &old_); }

private:
    termios old_{};
};

// 从 YAML 读取可用策略列表
std::vector<std::string> LoadPolicyNames(const std::string &yaml_path) {
    robot_base::YamlFile yaml = robot_base::YamlFile::Load(yaml_path);
    auto names = yaml.Read<std::vector<std::string>>("rl_policy.onnx_infer.policy_names");
    if (names && !names->empty()) {
        return *names;
    }
    return {};
}

// UI 固定行数（不含最后一行"最近操作"）
static constexpr int kUiLines = 14;  // 实际打印了 14 行（包含“最近操作”）

// 计算带 ANSI 控制符的字符串在终端中显示的实际宽度（忽略 \033[...m）
int GetVisibleWidth(const std::string &s) {
    int width = 0;
    bool in_escape = false;
    for (size_t i = 0; i < s.length(); ++i) {
        if (s[i] == '\033') {
            in_escape = true;
        } else if (in_escape) {
            if (s[i] == 'm') {
                in_escape = false;
            }
        } else {
            // 简单处理：中文字符（UTF-8）首字节最高位为 1，假设每个中文占 2 个显示宽度
            if ((s[i] & 0x80) != 0) {
                // 仅在 UTF-8 的首字节计算宽度
                if ((s[i] & 0xC0) == 0xC0) {
                    width += 2;
                }
            } else {
                width += 1;
            }
        }
    }
    return width;
}

// 打印带右边框的一行（固定内宽 42）
void PrintLineWithBorder(const std::string &content) {
    const int kInnerWidth = 42;
    int visible_w = GetVisibleWidth(content);
    int padding = kInnerWidth - visible_w;
    if (padding < 0)
        padding = 0;
    printf("║%s%*s║\n", content.c_str(), padding, "");
}

void PrintUi(const std::string &fsm_state,
            const std::vector<std::string> &policies,
            int policy_idx,
            float vx,
            float vy,
            float wz,
            const std::string &last_action) {
    // 上移 kUiLines 行，清除到屏幕底部，原地重绘
    printf("\033[%dA\033[J", kUiLines);

    printf("╔══════════════════════════════════════════╗\n");
    PrintLineWithBorder("  HMI 控制端");
    printf("╠══════════════════════════════════════════╣\n");

    // FSM 状态行
    std::string fsm_line = "  FSM: ";
    const char *states[] = {"POWER_OFF", "DAMP", "ZERO", "RL"};
    for (const char *s : states) {
        if (fsm_state == s) {
            fsm_line += "\033[7m ";
            fsm_line += s;
            fsm_line += " \033[0m";
        } else {
            fsm_line += " ";
            fsm_line += s;
            fsm_line += " ";
        }
    }
    PrintLineWithBorder(fsm_line);

    // 策略行
    if (!policies.empty()) {
        std::string policy_line = "  策略: ";
        for (int i = 0; i < static_cast<int>(policies.size()); ++i) {
            if (i == policy_idx) {
                policy_line += "\033[7m[" + std::to_string(i + 1) + "]" + policies[i] + "\033[0m ";
            } else {
                policy_line += " " + std::to_string(i + 1) + "." + policies[i] + " ";
            }
        }
        PrintLineWithBorder(policy_line);
        PrintLineWithBorder("");
    } else {
        PrintLineWithBorder("  策略: (无 RL 策略)");
        PrintLineWithBorder("");
    }

    printf("╠══════════════════════════════════════════╣\n");
    char speed_buf[128];
    snprintf(speed_buf, sizeof(speed_buf), "  速度: vx=%-5.2f  vy=%-5.2f  wz=%-5.2f", vx, vy, wz);
    PrintLineWithBorder(speed_buf);
    printf("╠══════════════════════════════════════════╣\n");
    PrintLineWithBorder("  f=POWER_OFF  o=DAMP  z=ZERO  r=RL");
    PrintLineWithBorder("  1/2/3...=切换策略  w/s=vx  a/d=vy");
    PrintLineWithBorder("  q/e=wz  空格=速度清零");
    printf("╚══════════════════════════════════════════╝\n");
    printf("最近操作: %s\n", last_action.c_str());
    fflush(stdout);
}

void PrintUiInit(const std::string &fsm_state,
                const std::vector<std::string> &policies,
                int policy_idx,
                float vx,
                float vy,
                float wz,
                const std::string &last_action) {
    printf("╔══════════════════════════════════════════╗\n");
    PrintLineWithBorder("  HMI 控制端");
    printf("╠══════════════════════════════════════════╣\n");

    std::string fsm_line = "  FSM: ";
    const char *states[] = {"POWER_OFF", "DAMP", "ZERO", "RL"};
    for (const char *s : states) {
        if (fsm_state == s) {
            fsm_line += "\033[7m ";
            fsm_line += s;
            fsm_line += " \033[0m";
        } else {
            fsm_line += " ";
            fsm_line += s;
            fsm_line += " ";
        }
    }
    PrintLineWithBorder(fsm_line);

    if (!policies.empty()) {
        std::string policy_line = "  策略: ";
        for (int i = 0; i < static_cast<int>(policies.size()); ++i) {
            if (i == policy_idx) {
                policy_line += "\033[7m[" + std::to_string(i + 1) + "]" + policies[i] + "\033[0m ";
            } else {
                policy_line += " " + std::to_string(i + 1) + "." + policies[i] + " ";
            }
        }
        PrintLineWithBorder(policy_line);
        PrintLineWithBorder("");
    } else {
        PrintLineWithBorder("  策略: (无 RL 策略)");
        PrintLineWithBorder("");
    }

    printf("╠══════════════════════════════════════════╣\n");
    char speed_buf[128];
    snprintf(speed_buf, sizeof(speed_buf), "  速度: vx=%-5.2f  vy=%-5.2f  wz=%-5.2f", vx, vy, wz);
    PrintLineWithBorder(speed_buf);
    printf("╠══════════════════════════════════════════╣\n");
    PrintLineWithBorder("  f=POWER_OFF  o=DAMP  z=ZERO  r=RL");
    PrintLineWithBorder("  1/2/3...=切换策略  w/s=vx  a/d=vy");
    PrintLineWithBorder("  q/e=wz  空格=速度清零");
    printf("╚══════════════════════════════════════════╝\n");
    printf("最近操作: %s\n", last_action.c_str());
    fflush(stdout);
}

}  // namespace

int main(int argc, char *argv[]) {
    std::signal(SIGINT, OnSignal);

    if (argc < 2) {
        fprintf(stderr, "用法: %s ../config/g1.yaml\n", argv[0]);
        return 1;
    }
    std::string yaml_path = argv[1];

    // 加载可用策略列表
    std::vector<std::string> policies = LoadPolicyNames(yaml_path);

    // 初始化传输（Hmi 角色）
    auto transport = transport::Create(yaml_path);
    if (!transport->Init(yaml_path, transport::Role::HMI)) {
        fprintf(stderr, "[hmi_demo] 传输初始化失败\n");
        return 1;
    }

    TerminalRawGuard guard;

    // 设置 STDIN 为非阻塞模式
    int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

    robot_base::Command cmd;
    std::string fsm_state = "POWER_OFF";  // 本地推断的 FSM 状态
    int policy_idx = 0;                   // 当前选中策略序号
    std::string last_action = "(等待输入)";
    bool sent_exit_signal = false;

    // 初始绘制 UI
    PrintUiInit(fsm_state, policies, policy_idx, cmd.vx, cmd.vy, cmd.wz, last_action);

    while (g_running) {
        char c = 0;
        ssize_t n = ::read(STDIN_FILENO, &c, 1);
        if (n <= 0) {
            usleep(10000);  // 10ms
            if (!g_running && !sent_exit_signal) {
                cmd.key = -1;
                cmd.switch_policy.clear();
                transport->SendCommand(cmd);
                sent_exit_signal = true;
                break;
            }
            continue;
        }

        cmd.key = 0;
        cmd.switch_policy.clear();

        switch (c) {
        case 'f':
            cmd.key = -1;
            fsm_state = "POWER_OFF";
            last_action = "[f] → POWER_OFF (完全失力)";
            break;
        case 'o':
            cmd.key = 1;
            fsm_state = "DAMP";
            last_action = "[o] → DAMP (阻尼保持)";
            break;
        case 'z':
            cmd.key = 2;
            fsm_state = "ZERO";
            last_action = "[z] → ZERO (回零位)";
            break;
        case 'r':
            cmd.key = 3;
            fsm_state = "RL";
            last_action = "[r] → RL (进入 RL 控制)";
            break;
        case 'w':
            cmd.vx += 0.1f;
            last_action = "[w] vx += 0.1";
            break;
        case 's':
            cmd.vx -= 0.1f;
            last_action = "[s] vx -= 0.1";
            break;
        case 'a':
            cmd.vy += 0.1f;
            last_action = "[a] vy += 0.1";
            break;
        case 'd':
            cmd.vy -= 0.1f;
            last_action = "[d] vy -= 0.1";
            break;
        case 'q':
            cmd.wz += 0.1f;
            last_action = "[q] wz += 0.1";
            break;
        case 'e':
            cmd.wz -= 0.1f;
            last_action = "[e] wz -= 0.1";
            break;
        case ' ':
            cmd.vx = cmd.vy = cmd.wz = 0.0f;
            last_action = "[空格] 速度清零";
            break;
        default:
            // 数字键 1~9：切换策略
            if (c >= '1' && c <= '9') {
                int idx = c - '1';
                if (idx < static_cast<int>(policies.size())) {
                    policy_idx = idx;
                    cmd.switch_policy = policies[idx];
                    last_action = std::string("[") + c + "] 切换策略 → " + policies[idx];
                } else {
                    last_action = std::string("[") + c + "] 无效策略序号";
                }
            } else {
                continue;
            }
            break;
        }

        transport->SendCommand(cmd);
        PrintUi(fsm_state, policies, policy_idx, cmd.vx, cmd.vy, cmd.wz, last_action);
    }

    return 0;
}
