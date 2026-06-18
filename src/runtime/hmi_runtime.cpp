/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file hmi_runtime.cpp
 * @brief HMI 控制端（ANSI TUI）
 *
 * 使用备用屏幕缓冲区 + 光标绝对定位实现全屏 TUI，
 * 退出时自动恢复原终端内容。
 */

#include <fcntl.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "transport_executor.h"
#include "robot_base.h"

namespace {

volatile std::sig_atomic_t g_running = 1;
void OnSignal(int) { g_running = 0; }

// ============================================================
// 终端管理
// ============================================================

class Terminal {
public:
    Terminal() {
        tcgetattr(STDIN_FILENO, &orig_);
        termios raw = orig_;
        raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        tcsetattr(STDIN_FILENO, TCSANOW, &raw);
        printf("\033[?1049h");  // 切到备用屏幕
        printf("\033[?25l");    // 隐藏光标
        fflush(stdout);
    }

    ~Terminal() {
        printf("\033[?25h");    // 恢复光标
        printf("\033[?1049l");  // 切回主屏幕
        fflush(stdout);
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    }

    static void GetSize(int &rows, int &cols) {
        struct winsize ws {};
        if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0) {
            rows = ws.ws_row;
            cols = ws.ws_col;
        } else {
            rows = 24;
            cols = 80;
        }
    }

    static int ReadKey() {
        char c = 0;
        if (::read(STDIN_FILENO, &c, 1) == 1) return c;
        return -1;
    }

private:
    termios orig_{};
};

// ============================================================
// ANSI 绘制工具
// ============================================================

static void MoveTo(int row, int col) { printf("\033[%d;%dH", row, col); }
static void ClearScreen() { printf("\033[2J"); }
static void SetReverse() { printf("\033[7m"); }
static void SetBold() { printf("\033[1m"); }
static void SetDim() { printf("\033[2m"); }
static void ResetAttr() { printf("\033[0m"); }

static void HLine(
        int row, int col, int width,
        const char *left, const char *fill, const char *right) {
    MoveTo(row, col);
    printf("%s", left);
    for (int i = 0; i < width - 2; ++i) printf("%s", fill);
    printf("%s", right);
}

static void DrawBox(int top, int left, int width, int height) {
    HLine(top, left, width, "┌", "─", "┐");
    for (int r = top + 1; r < top + height - 1; ++r) {
        MoveTo(r, left);
        printf("│");
        MoveTo(r, left + width - 1);
        printf("│");
    }
    HLine(top + height - 1, left, width, "└", "─", "┘");
}

static void DrawHSep(int row, int left, int width) {
    HLine(row, left, width, "├", "─", "┤");
}

static void PadWrite(int row, int col, int field_width, const char *text) {
    MoveTo(row, col);
    int len = static_cast<int>(strlen(text));
    printf("%s", text);
    for (int i = len; i < field_width; ++i) printf(" ");
}

// ============================================================
// YAML 读取
// ============================================================

std::vector<std::string> LoadPolicyNames(const std::string &yaml_path) {
    robot_base::YamlFile yaml = robot_base::YamlFile::Load(yaml_path);
    auto names = yaml.Read<std::vector<std::string>>(
        "rl_policy.onnx_infer.policy_names");
    if (names && !names->empty()) return *names;
    return {};
}

// ============================================================
// TUI 渲染
// ============================================================

static constexpr int kBoxWidth = 54;
static constexpr int kBoxLeft = 2;
static constexpr int kContentLeft = kBoxLeft + 2;
static constexpr int kContentWidth = kBoxWidth - 4;

void Render(const std::string &fsm_state,
            const std::vector<std::string> &policies,
            int policy_idx, float vx, float vy, float wz,
            const std::string &last_action) {
    int row = 2;

    // 标题
    DrawBox(row, kBoxLeft, kBoxWidth, 3);
    MoveTo(row + 1, kContentLeft);
    SetBold();
    printf("HMI 控制端");
    ResetAttr();
    row += 3;

    // FSM 状态区
    const char *states[] = {"POWER_OFF", "DAMP", "ZERO", "RL"};
    DrawBox(row, kBoxLeft, kBoxWidth, 4);
    MoveTo(row + 1, kContentLeft);
    SetDim();
    printf("FSM");
    ResetAttr();
    MoveTo(row + 2, kContentLeft);
    for (const char *s : states) {
        if (fsm_state == s) {
            SetReverse();
            printf(" %s ", s);
            ResetAttr();
        } else {
            printf(" %s ", s);
        }
    }
    row += 4;

    // 策略区
    int total = static_cast<int>(policies.size());
    int policy_rows = std::max(1, (total + 3) / 4);
    int policy_box_h = 2 + policy_rows + 1;
    DrawBox(row, kBoxLeft, kBoxWidth, policy_box_h);
    MoveTo(row + 1, kContentLeft);
    SetDim();
    printf("策略");
    ResetAttr();
    if (policies.empty()) {
        MoveTo(row + 2, kContentLeft);
        printf("(无 RL 策略)");
    } else {
        int kPerRow = 4;
        for (int r = 0; r < policy_rows; ++r) {
            MoveTo(row + 2 + r, kContentLeft);
            int start = r * kPerRow;
            int end = std::min(start + kPerRow, total);
            for (int i = start; i < end; ++i) {
                if (i == policy_idx) {
                    SetReverse();
                    printf("[%d]%s", i + 1, policies[i].c_str());
                    ResetAttr();
                    printf(" ");
                } else {
                    printf("%d.%s ", i + 1, policies[i].c_str());
                }
            }
        }
    }
    row += policy_box_h;

    // 速度区
    DrawBox(row, kBoxLeft, kBoxWidth, 4);
    MoveTo(row + 1, kContentLeft);
    SetDim();
    printf("速度");
    ResetAttr();
    MoveTo(row + 2, kContentLeft);
    printf("vx=%-6.2f  vy=%-6.2f  wz=%-6.2f", vx, vy, wz);
    row += 4;

    // 按键说明区
    DrawBox(row, kBoxLeft, kBoxWidth, 6);
    MoveTo(row + 1, kContentLeft);
    SetDim();
    printf("快捷键");
    ResetAttr();
    MoveTo(row + 2, kContentLeft);
    printf("f=POWER_OFF  o=DAMP  z=ZERO  r=RL");
    MoveTo(row + 3, kContentLeft);
    printf("1-9=切换策略  w/s=vx  a/d=vy");
    MoveTo(row + 4, kContentLeft);
    printf("q/e=wz  空格=速度清零  Ctrl+C=退出");
    row += 6;

    // 状态栏
    DrawHSep(row, kBoxLeft, kBoxWidth);
    row += 1;
    MoveTo(row, kContentLeft);
    printf("%-*s", kContentWidth, "");
    MoveTo(row, kContentLeft);
    printf("最近: %s", last_action.c_str());

    fflush(stdout);
}

}  // namespace

// ============================================================
// main
// ============================================================

int main(int argc, char *argv[]) {
    std::signal(SIGINT, OnSignal);

    if (argc < 2 || std::string(argv[1]) == "-h" || std::string(argv[1]) == "--help") {
        fprintf((argc < 2) ? stderr : stdout,
            "用法: %s <config.yaml>\n"
            "选项:\n"
            "  <config.yaml>  机器人配置文件路径\n"
            "  -h, --help     显示此帮助信息\n", argv[0]);
        return (argc < 2) ? 1 : 0;
    }
    std::string yaml_path = argv[1];

    std::vector<std::string> policies;
    try {
        policies = LoadPolicyNames(yaml_path);
    } catch (const std::exception &e) {
        fprintf(stderr, "%s\n用法: %s <config.yaml>\n", e.what(), argv[0]);
        return 1;
    }

    auto transport = transport::Create(yaml_path);
    if (!transport->Init(yaml_path, transport::Role::HMI)) {
        fprintf(stderr, "[hmi_runtime] 传输初始化失败\n");
        return 1;
    }

    Terminal term;
    ClearScreen();

    robot_base::Command cmd;
    std::string fsm_state = "POWER_OFF";
    int policy_idx = 0;
    std::string last_action = "(等待输入)";
    bool sent_exit_signal = false;

    Render(fsm_state, policies, policy_idx, cmd.vx, cmd.vy, cmd.wz, last_action);

    while (g_running) {
        int c = Terminal::ReadKey();
        if (c < 0) {
            usleep(10000);
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
            if (c >= '1' && c <= '9') {
                int idx = c - '1';
                if (idx < static_cast<int>(policies.size())) {
                    policy_idx = idx;
                    cmd.switch_policy = policies[idx];
                    last_action = std::string("[") + static_cast<char>(c)
                        + "] 切换策略 → " + policies[idx];
                } else {
                    last_action = std::string("[") + static_cast<char>(c)
                        + "] 无效策略序号";
                }
            } else {
                continue;
            }
            break;
        }

        transport->SendCommand(cmd);
        Render(fsm_state, policies, policy_idx,
            cmd.vx, cmd.vy, cmd.wz, last_action);
    }

    return 0;
}
