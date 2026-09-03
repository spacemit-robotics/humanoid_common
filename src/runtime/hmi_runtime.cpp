/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file hmi_runtime.cpp
 * @brief HMI 控制端（带状态回传的 ANSI TUI）
 *
 * Control 端是真实状态的唯一来源。HMI 只发送 FSM、策略和速度请求，界面显示
 * 以 ControlStatus 回传为准；HMI 退出或链路超时后，control_runtime 会将速度清零。
 */

#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iterator>
#include <memory>
#include <string>
#include <vector>

#include "policy_command_limits.h"
#include "robot_base.h"
#include "runtime_logger.h"
#include "transport_executor.h"

namespace {

using Clock = std::chrono::steady_clock;
using robot_base::ControlMode;

volatile std::sig_atomic_t g_running = 1;
bool g_color_enabled = true;

void OnSignal(int) { g_running = 0; }

// ============================================================
// 终端管理
// ============================================================

class Terminal {
public:
    Terminal() {
        if (tcgetattr(STDIN_FILENO, &orig_) == 0) {
            termios raw = orig_;
            raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
            raw.c_cc[VMIN] = 0;
            raw.c_cc[VTIME] = 0;
            raw_enabled_ = tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0;
        }
        printf("\033[?1049h\033[?25l");
        fflush(stdout);
    }

    ~Terminal() {
        printf("\033[0m\033[?25h\033[?1049l");
        fflush(stdout);
        if (raw_enabled_) tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
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
        if (::read(STDIN_FILENO, &c, 1) == 1) {
            return static_cast<unsigned char>(c);
        }
        return -1;
    }

private:
    termios orig_{};
    bool raw_enabled_ = false;
};

// ============================================================
// ANSI 绘制工具
// ============================================================

enum class Color : int {
    BLACK = 30,
    RED = 31,
    GREEN = 32,
    YELLOW = 33,
    BLUE = 34,
    MAGENTA = 35,
    CYAN = 36,
    WHITE = 37,
    GRAY = 90,
    BRIGHT_RED = 91,
    BRIGHT_GREEN = 92,
    BRIGHT_YELLOW = 93,
    BRIGHT_CYAN = 96,
};

void MoveTo(int row, int col) { printf("\033[%d;%dH", row, col); }
void ClearScreen() { printf("\033[2J\033[H"); }
void ResetAttr() { printf("\033[0m"); }
void SetBold() { printf("\033[1m"); }
void SetDim() { printf("\033[2m"); }
void SetReverse() { printf("\033[7m"); }

void SetFg(Color color) {
    if (g_color_enabled) printf("\033[%dm", static_cast<int>(color));
}

void SetBg(Color color) {
    if (g_color_enabled) printf("\033[%dm", static_cast<int>(color) + 10);
}

void HLine(int row, int col, int width, const char *left,
        const char *fill, const char *right) {
    MoveTo(row, col);
    printf("%s", left);
    for (int i = 0; i < width - 2; ++i) printf("%s", fill);
    printf("%s", right);
}

void DrawBox(int top, int left, int width, int height) {
    SetFg(Color::GRAY);
    HLine(top, left, width, "┌", "─", "┐");
    for (int row = top + 1; row < top + height - 1; ++row) {
        MoveTo(row, left);
        printf("│");
        MoveTo(row, left + width - 1);
        printf("│");
    }
    HLine(top + height - 1, left, width, "└", "─", "┘");
    ResetAttr();
}

void DrawHSep(int row, int left, int width) {
    SetFg(Color::GRAY);
    HLine(row, left, width, "├", "─", "┤");
    ResetAttr();
}

struct Layout {
    int left = 2;
    int width = 66;
    int content_left = 4;
};

Layout GetLayout() {
    int rows = 0;
    int cols = 0;
    Terminal::GetSize(rows, cols);
    (void)rows;
    Layout layout;
    layout.width = std::min(66, std::max(54, cols - 2));
    layout.left = std::max(1, (cols - layout.width) / 2 + 1);
    layout.content_left = layout.left + 2;
    return layout;
}

// ============================================================
// 配置与 UI 状态
// ============================================================

struct HmiConfig {
    double heartbeat_hz = 20.0;
    double status_timeout = 0.6;
    double request_timeout = 15.0;
    int key_highlight_ms = 180;
    float step_vx = 0.1f;
    float step_vy = 0.1f;
    float step_wz = 0.1f;
};

struct UiConfig {
    HmiConfig hmi;
    std::vector<std::string> policies;
    runtime_config::PolicyCommandLimitMap command_limits;
    int default_policy_idx = 0;
};

UiConfig LoadUiConfig(const std::string &yaml_path) {
    robot_base::YamlFile yaml = robot_base::YamlFile::Load(yaml_path);
    UiConfig config;
    config.policies = yaml.Read<std::vector<std::string>>(
        "rl_policy.onnx_infer.policy_names").value_or(
            std::vector<std::string>{});

    const auto default_policy = yaml.Read<std::string>(
        "rl_policy.onnx_infer.default_policy");
    if (default_policy) {
        const auto it = std::find(config.policies.begin(),
            config.policies.end(), *default_policy);
        if (it != config.policies.end()) {
            config.default_policy_idx = static_cast<int>(
                std::distance(config.policies.begin(), it));
        }
    }

    config.hmi.heartbeat_hz = std::max(1.0,
        yaml.Read<double>("hmi.command_heartbeat_hz").value_or(20.0));
    config.hmi.status_timeout = std::max(0.1,
        yaml.Read<double>("hmi.status_timeout").value_or(0.6));
    config.hmi.request_timeout = std::max(0.5,
        yaml.Read<double>("hmi.request_timeout").value_or(15.0));
    config.hmi.key_highlight_ms = std::max(0,
        yaml.Read<int>("hmi.key_highlight_ms").value_or(180));
    config.hmi.step_vx = static_cast<float>(std::abs(
        yaml.Read<double>("hmi.velocity.step_vx").value_or(0.1)));
    config.hmi.step_vy = static_cast<float>(std::abs(
        yaml.Read<double>("hmi.velocity.step_vy").value_or(0.1)));
    config.hmi.step_wz = static_cast<float>(std::abs(
        yaml.Read<double>("hmi.velocity.step_wz").value_or(0.1)));
    config.command_limits = runtime_config::LoadPolicyCommandLimits(yaml);
    return config;
}

enum class HmiPage {
    MAIN,
    POLICY_SELECT,
    VELOCITY,
};

struct PendingTransition {
    bool active = false;
    ControlMode source = ControlMode::POWER_OFF;
    ControlMode target = ControlMode::POWER_OFF;
    int key = 0;
    Clock::time_point requested_at{};
};

struct UiState {
    HmiPage page = HmiPage::MAIN;
    robot_base::ControlStatus status;
    bool has_status = false;
    bool status_online = false;
    Clock::time_point last_status_at{};
    robot_base::Command target_command;
    PendingTransition transition;
    std::vector<std::string> policies;
    runtime_config::PolicyCommandLimitMap command_limits;
    int active_policy_idx = 0;
    int policy_cursor_idx = 0;
    std::string pending_policy;
    std::string policy_source;
    Clock::time_point policy_requested_at{};
    std::string last_action = "等待 Control 状态回传";
    int highlighted_key = -1;
    Clock::time_point highlight_until{};
};

const runtime_config::PolicyCommandLimits *ActiveCommandLimits(
        const UiState &state) {
    return runtime_config::FindPolicyCommandLimits(
        state.command_limits, state.status.active_policy);
}

const char *ModeName(ControlMode mode) {
    switch (mode) {
    case ControlMode::POWER_OFF:
        return "POWER_OFF";
    case ControlMode::DAMP:
        return "DAMP";
    case ControlMode::HOME:
        return "HOME";
    case ControlMode::ZERO:
        return "ZERO";
    case ControlMode::RL:
        return "RL";
    case ControlMode::SAFETY:
        return "SAFETY";
    }
    return "UNKNOWN";
}

Color ModeColor(ControlMode mode) {
    switch (mode) {
    case ControlMode::POWER_OFF:
        return Color::BRIGHT_RED;
    case ControlMode::DAMP:
        return Color::BRIGHT_YELLOW;
    case ControlMode::HOME:
        return Color::BLUE;
    case ControlMode::ZERO:
        return Color::BRIGHT_CYAN;
    case ControlMode::RL:
        return Color::BRIGHT_GREEN;
    case ControlMode::SAFETY:
        return Color::MAGENTA;
    }
    return Color::WHITE;
}

void PrintConnection(const UiState &state) {
    if (!state.status_online) {
        SetFg(Color::BRIGHT_RED);
        SetBold();
        printf("● CONTROL OFFLINE");
    } else if (!state.status.hmi_connected) {
        SetFg(Color::BRIGHT_YELLOW);
        SetBold();
        printf("● HEARTBEAT WAIT");
    } else {
        SetFg(Color::BRIGHT_GREEN);
        SetBold();
        printf("● CONTROL ONLINE");
    }
    ResetAttr();
}

void PrintHeader(const Layout &layout, const UiState &state,
        const char *title) {
    DrawBox(1, layout.left, layout.width, 3);
    MoveTo(2, layout.content_left);
    SetFg(Color::BRIGHT_CYAN);
    SetBold();
    printf("SpaceMIT Humanoid · %s", title);
    ResetAttr();
    MoveTo(2, layout.left + layout.width - 22);
    PrintConnection(state);
}

void PrintModeToken(ControlMode token, const UiState &state) {
    const bool active = state.status_online && state.status.mode == token;
    const bool pending = state.transition.active &&
        state.transition.target == token;
    if (active) {
        SetFg(ModeColor(token));
        SetBold();
        printf("▶[%s]", ModeName(token));
    } else if (pending) {
        SetFg(Color::MAGENTA);
        SetBold();
        printf("…[%s]", ModeName(token));
    } else {
        SetDim();
        printf(" [%s]", ModeName(token));
    }
    ResetAttr();
}

void PrintLastAction(const Layout &layout, int row,
        const std::string &last_action) {
    DrawHSep(row, layout.left, layout.width);
    MoveTo(row + 1, layout.content_left);
    SetFg(Color::CYAN);
    printf("最近: %s", last_action.c_str());
    ResetAttr();
}

void RenderMainPage(const UiState &state) {
    const Layout layout = GetLayout();
    ClearScreen();
    PrintHeader(layout, state, "控制台");

    DrawBox(4, layout.left, layout.width, 5);
    MoveTo(5, layout.content_left);
    SetDim();
    printf("FSM 真实状态");
    ResetAttr();
    MoveTo(6, layout.content_left);
    PrintModeToken(ControlMode::POWER_OFF, state);
    printf(" ⇄ ");
    PrintModeToken(ControlMode::DAMP, state);
    printf(" ⇄ ");
    PrintModeToken(ControlMode::HOME, state);
    printf(" → ");
    PrintModeToken(ControlMode::ZERO, state);
    printf(" → ");
    PrintModeToken(ControlMode::RL, state);
    MoveTo(7, layout.content_left);
    if (state.status_online && state.status.mode == ControlMode::ZERO) {
        SetFg(state.status.zero_ready ? Color::BRIGHT_GREEN : Color::BRIGHT_YELLOW);
        printf("回零: %s", state.status.zero_ready ?
            "READY，可按 → 进入 RL" : "进行中，可按 → 排队进入 RL");
    } else if (state.status_online && state.status.mode == ControlMode::SAFETY) {
        SetFg(Color::BRIGHT_RED);
        SetBold();
        printf("SAFETY 正在卸力；等待 Control 自动回到 POWER_OFF");
    } else {
        SetDim();
        printf("← 后退；→ 前进；从 RL 按 ← 直接退回 DAMP");
    }
    ResetAttr();

    DrawBox(9, layout.left, layout.width, 4);
    MoveTo(10, layout.content_left);
    SetDim();
    printf("策略");
    ResetAttr();
    MoveTo(11, layout.content_left);
    const std::string policy = state.status.active_policy.empty()
        ? "(未加载)" : state.status.active_policy;
    SetFg(Color::MAGENTA);
    SetBold();
    printf("当前: %s", policy.c_str());
    ResetAttr();
    printf("    ");
    SetFg(Color::CYAN);
    printf("[P] 选择策略");
    ResetAttr();

    DrawBox(13, layout.left, layout.width, 4);
    MoveTo(14, layout.content_left);
    SetDim();
    printf("Control 实际采用速度");
    ResetAttr();
    MoveTo(15, layout.content_left);
    printf("vx=%+.2f m/s   vy=%+.2f m/s   wz=%+.2f rad/s",
        state.status.vx, state.status.vy, state.status.wz);

    DrawBox(17, layout.left, layout.width, 5);
    MoveTo(18, layout.content_left);
    SetFg(Color::CYAN);
    printf("[←]/[→]");
    ResetAttr();
    printf(" 切换 FSM    ");
    SetFg(Color::CYAN);
    printf("[P]");
    ResetAttr();
    printf(" 策略    ");
    SetFg(Color::CYAN);
    printf("[V]/[Enter]");
    ResetAttr();
    printf(" 速度");
    MoveTo(19, layout.content_left);
    SetFg(Color::BRIGHT_RED);
    printf("[F]");
    ResetAttr();
    printf(" POWER_OFF    [Space] 速度清零    [Ctrl+C] 退出");
    MoveTo(20, layout.content_left);
    SetDim();
    printf("兼容快捷键: o=DAMP  z=ZERO  r=RL（仍由 Control 校验）");
    ResetAttr();

    PrintLastAction(layout, 22, state.last_action);
    fflush(stdout);
}

void RenderPolicySelectPage(const UiState &state) {
    const Layout layout = GetLayout();
    ClearScreen();
    PrintHeader(layout, state, "策略选择");

    const int list_rows = std::max(1, static_cast<int>(state.policies.size()));
    DrawBox(4, layout.left, layout.width, list_rows + 3);
    MoveTo(5, layout.content_left);
    SetDim();
    printf("绿色 ● 为 Control 当前生效策略");
    ResetAttr();
    if (state.policies.empty()) {
        MoveTo(6, layout.content_left);
        printf("(没有配置可选策略)");
    } else {
        for (int i = 0; i < static_cast<int>(state.policies.size()); ++i) {
            MoveTo(6 + i, layout.content_left);
            if (i == state.policy_cursor_idx) {
                SetBg(Color::BLUE);
                SetFg(Color::WHITE);
                SetBold();
            }
            if (i == state.active_policy_idx) {
                SetFg(Color::BRIGHT_GREEN);
                printf("● ");
            } else {
                printf("  ");
            }
            printf("%-28s", state.policies[i].c_str());
            ResetAttr();
        }
    }

    const int operation_row = 4 + list_rows + 3;
    DrawBox(operation_row, layout.left, layout.width, 5);
    MoveTo(operation_row + 1, layout.content_left);
    SetDim();
    printf("操作");
    ResetAttr();
    MoveTo(operation_row + 2, layout.content_left);
    printf("↑/↓ 或 j/k 移动    Enter 确认    Esc/P 返回");
    MoveTo(operation_row + 3, layout.content_left);
    SetFg(Color::BRIGHT_YELLOW);
    printf("仅真实状态为 POWER_OFF 或 DAMP 时允许切换");
    ResetAttr();

    PrintLastAction(layout, operation_row + 5, state.last_action);
    fflush(stdout);
}

void PrintKey(int row, int col, const char *key, const char *action,
        bool highlighted) {
    MoveTo(row, col);
    if (highlighted) {
        SetBg(Color::CYAN);
        SetFg(Color::BLACK);
        SetBold();
        SetReverse();
    } else {
        SetFg(Color::BRIGHT_CYAN);
        SetBold();
    }
    printf(" %-5s ", key);
    ResetAttr();
    printf(" %s", action);
}

void RenderVelocityPage(const UiState &state) {
    const Layout layout = GetLayout();
    ClearScreen();
    PrintHeader(layout, state, "键盘速度控制");

    DrawBox(4, layout.left, layout.width, 5);
    MoveTo(5, layout.content_left);
    const std::string policy = state.status.active_policy.empty()
        ? "-" : state.status.active_policy;
    printf("FSM: ");
    SetFg(ModeColor(state.status.mode));
    SetBold();
    printf("%s", ModeName(state.status.mode));
    ResetAttr();
    printf("    策略: %s    RL: %.1f Hz", policy.c_str(),
        state.status.rl_frequency_hz);
    MoveTo(6, layout.content_left);
    printf("目标  vx=%+.2f   vy=%+.2f   wz=%+.2f",
        state.target_command.vx, state.target_command.vy,
        state.target_command.wz);
    MoveTo(7, layout.content_left);
    SetFg(Color::BRIGHT_GREEN);
    printf("实际  vx=%+.2f   vy=%+.2f   wz=%+.2f",
        state.status.vx, state.status.vy, state.status.wz);
    ResetAttr();

    DrawBox(9, layout.left, layout.width, 9);
    MoveTo(10, layout.content_left);
    SetDim();
    printf("按键按下后短暂高亮；每次按键按配置步长加减速度");
    ResetAttr();
    PrintKey(12, layout.content_left + 2, "Q", "左转",
        state.highlighted_key == 'q');
    PrintKey(12, layout.content_left + 21, "W", "前进",
        state.highlighted_key == 'w');
    PrintKey(12, layout.content_left + 40, "E", "右转",
        state.highlighted_key == 'e');
    PrintKey(14, layout.content_left + 2, "A", "左移",
        state.highlighted_key == 'a');
    PrintKey(14, layout.content_left + 21, "S", "后退",
        state.highlighted_key == 's');
    PrintKey(14, layout.content_left + 40, "D", "右移",
        state.highlighted_key == 'd');
    PrintKey(16, layout.content_left + 21, "SPACE", "清零",
        state.highlighted_key == ' ');

    DrawBox(18, layout.left, layout.width, 4);
    MoveTo(19, layout.content_left);
    printf("Esc/V 返回并清零    ");
    SetFg(Color::BRIGHT_RED);
    printf("F = POWER_OFF");
    ResetAttr();
    MoveTo(20, layout.content_left);
    SetDim();
    printf("离开 RL、HMI 退出或心跳超时，Control 均会将速度清零");
    ResetAttr();

    PrintLastAction(layout, 22, state.last_action);
    fflush(stdout);
}

void Render(const UiState &state) {
    switch (state.page) {
    case HmiPage::POLICY_SELECT:
        RenderPolicySelectPage(state);
        break;
    case HmiPage::VELOCITY:
        RenderVelocityPage(state);
        break;
    case HmiPage::MAIN:
        RenderMainPage(state);
        break;
    }
}

// ============================================================
// 输入与请求处理
// ============================================================

constexpr int kKeyEscape = 27;
constexpr int kKeyUp = 1000;
constexpr int kKeyDown = 1001;
constexpr int kKeyLeft = 1002;
constexpr int kKeyRight = 1003;

int WaitForSequenceByte() {
    for (int i = 0; i < 5; ++i) {
        usleep(1000);
        const int key = Terminal::ReadKey();
        if (key >= 0) return key;
    }
    return -1;
}

int ReadUiKey() {
    const int key = Terminal::ReadKey();
    if (key != kKeyEscape) return key;

    if (WaitForSequenceByte() != '[') return kKeyEscape;
    const int direction = WaitForSequenceByte();
    if (direction == 'A') return kKeyUp;
    if (direction == 'B') return kKeyDown;
    if (direction == 'C') return kKeyRight;
    if (direction == 'D') return kKeyLeft;
    return kKeyEscape;
}

void ZeroVelocity(UiState *state) {
    state->target_command.vx = 0.0f;
    state->target_command.vy = 0.0f;
    state->target_command.wz = 0.0f;
}

robot_base::Command BuildCommand(const UiState &state) {
    robot_base::Command command = state.target_command;
    command.key = state.transition.active ? state.transition.key : 0;
    command.switch_policy = state.pending_policy;
    return command;
}

void SendCommand(transport::TransportBase *transport, const UiState &state) {
    transport->SendCommand(BuildCommand(state));
}

bool RequestTransition(UiState *state, ControlMode target, int key,
        const Clock::time_point &now) {
    if (!state->status_online) {
        state->last_action = "FSM 请求未发送：Control 状态已断开";
        return false;
    }
    if (!state->pending_policy.empty()) {
        state->last_action = "FSM 请求未发送：正在等待策略切换确认";
        return false;
    }
    if (state->status.mode == target) {
        state->last_action = std::string("已经处于 ") + ModeName(target);
        return false;
    }

    state->transition.active = true;
    state->transition.source = state->status.mode;
    state->transition.target = target;
    state->transition.key = key;
    state->transition.requested_at = now;
    state->last_action = std::string("请求 ") + ModeName(state->status.mode)
        + " → " + ModeName(target) + "，等待 Control 确认";
    return true;
}

bool RequestByArrow(UiState *state, bool forward,
                    const Clock::time_point &now) {
    if (!state->status_online) {
        state->last_action = "FSM 请求未发送：Control 状态已断开";
        return false;
    }
    if (state->transition.active) {
        state->last_action = "上一个 FSM 请求仍在等待 Control 确认";
        return false;
    }

    const ControlMode current = state->status.mode;
    if (forward) {
        if (current == ControlMode::POWER_OFF) {
            return RequestTransition(state, ControlMode::DAMP, 1, now);
        }
        if (current == ControlMode::DAMP) {
            return RequestTransition(state, ControlMode::HOME, 4, now);
        }
        if (current == ControlMode::HOME) {
            return RequestTransition(state, ControlMode::ZERO, 2, now);
        }
        if (current == ControlMode::ZERO) {
            return RequestTransition(state, ControlMode::RL, 3, now);
        }
        state->last_action = current == ControlMode::RL
            ? "RL 已是最前状态" : "SAFETY 状态禁止手动前进";
        return false;
    }

    if (current == ControlMode::DAMP) {
        return RequestTransition(state, ControlMode::POWER_OFF, -1, now);
    }
    if (current == ControlMode::HOME) {
        return RequestTransition(state, ControlMode::DAMP, 1, now);
    }
    if (current == ControlMode::ZERO || current == ControlMode::RL) {
        return RequestTransition(state, ControlMode::DAMP, 1, now);
    }
    state->last_action = current == ControlMode::POWER_OFF
        ? "POWER_OFF 已是最后状态" : "SAFETY 正在自动卸力";
    return false;
}

bool RequestShortcut(UiState *state, int key,
        const Clock::time_point &now) {
    const ControlMode current = state->status.mode;
    if (key == 'f') {
        ZeroVelocity(state);
        state->page = HmiPage::MAIN;
        return RequestTransition(state, ControlMode::POWER_OFF, -1, now);
    }
    if (key == 'o') {
        if (current == ControlMode::POWER_OFF || current == ControlMode::HOME ||
            current == ControlMode::ZERO || current == ControlMode::RL) {
            return RequestTransition(state, ControlMode::DAMP, 1, now);
        }
    } else if (key == 'h' && current == ControlMode::DAMP) {
        return RequestTransition(state, ControlMode::HOME, 4, now);
    } else if (key == 'z' && current == ControlMode::HOME) {
        return RequestTransition(state, ControlMode::ZERO, 2, now);
    } else if (key == 'r' && current == ControlMode::ZERO) {
        return RequestTransition(state, ControlMode::RL, 3, now);
    }
    state->last_action = "当前真实 FSM 不接受该快捷键";
    return false;
}

void UpdateActivePolicy(UiState *state) {
    const auto it = std::find(state->policies.begin(), state->policies.end(),
        state->status.active_policy);
    if (it != state->policies.end()) {
        state->active_policy_idx = static_cast<int>(
            std::distance(state->policies.begin(), it));
    }
}

bool StatusChanged(const UiState &state,
        const robot_base::ControlStatus &status) {
    if (!state.has_status) return true;
    return state.status.mode != status.mode ||
        state.status.zero_ready != status.zero_ready ||
        state.status.hmi_connected != status.hmi_connected ||
        state.status.active_policy != status.active_policy ||
        std::abs(state.status.vx - status.vx) > 0.005f ||
        std::abs(state.status.vy - status.vy) > 0.005f ||
        std::abs(state.status.wz - status.wz) > 0.005f ||
        std::abs(state.status.rl_frequency_hz - status.rl_frequency_hz) > 5.0f;
}

void ProcessStatus(UiState *state, const robot_base::ControlStatus &status,
        const Clock::time_point &now, bool *send_immediately) {
    state->status = status;
    state->has_status = true;
    state->last_status_at = now;
    UpdateActivePolicy(state);

    if (state->transition.active) {
        if (status.mode == state->transition.target) {
            state->last_action = std::string("Control 已确认 FSM → ")
                + ModeName(status.mode);
            state->transition.active = false;
            *send_immediately = true;
        } else if (status.mode != state->transition.source) {
            state->last_action = std::string("FSM 请求被中断，实际状态为 ")
                + ModeName(status.mode);
            state->transition.active = false;
            *send_immediately = true;
        }
    }

    if (!state->pending_policy.empty()) {
        if (status.active_policy == state->pending_policy) {
            state->last_action = "Control 已确认策略 → " + status.active_policy;
            state->pending_policy.clear();
            *send_immediately = true;
        } else if (!status.active_policy.empty() &&
            status.active_policy != state->policy_source) {
            state->last_action = "策略链已启动，当前生效 → "
                + status.active_policy;
            state->pending_policy.clear();
            *send_immediately = true;
        }
    }

    if (state->page == HmiPage::VELOCITY &&
        (status.mode != ControlMode::RL || !ActiveCommandLimits(*state))) {
        ZeroVelocity(state);
        state->page = HmiPage::MAIN;
        state->last_action = status.mode == ControlMode::RL
            ? "当前策略未配置速度命令范围，速度已清零"
            : "已离开 RL，速度清零并返回主界面";
        *send_immediately = true;
    }
}

}  // namespace

// ============================================================
// main
// ============================================================

int main(int argc, char *argv[]) {
    std::signal(SIGINT, OnSignal);
    std::signal(SIGTERM, OnSignal);

    if (argc < 2 || std::string(argv[1]) == "-h" ||
        std::string(argv[1]) == "--help") {
        fprintf((argc < 2) ? stderr : stdout,
            "用法: %s <config.yaml>\n"
            "选项:\n"
            "  <config.yaml>  机器人配置文件路径\n"
            "  -h, --help     显示此帮助信息\n", argv[0]);
        return (argc < 2) ? 1 : 0;
    }
    const std::string yaml_path = argv[1];

    UiConfig config;
    std::unique_ptr<runtime_logging::Session> logging_session;
    try {
        config = LoadUiConfig(yaml_path);
        const auto yaml_file = robot_base::YamlFile::Load(yaml_path);
        logging_session = std::make_unique<runtime_logging::Session>(
            yaml_file, yaml_path, "hmi", false);
    } catch (const std::exception &e) {
        fprintf(stderr, "%s\n用法: %s <config.yaml>\n", e.what(), argv[0]);
        return 1;
    }

    auto transport = transport::Create(yaml_path);
    if (!transport->Init(yaml_path, transport::Role::HMI)) {
        runtime_logging::Log(
            runtime_logging::Level::kError, "HMI transport initialization failed", false);
        fprintf(stderr, "[hmi_runtime] 传输初始化失败\n");
        return 1;
    }
    runtime_logging::Log(runtime_logging::Level::kInfo, "HMI runtime started", false);

    g_color_enabled = isatty(STDOUT_FILENO) && std::getenv("NO_COLOR") == nullptr;
    Terminal terminal;

    UiState state;
    state.policies = config.policies;
    state.command_limits = config.command_limits;
    state.active_policy_idx = config.default_policy_idx;
    state.policy_cursor_idx = config.default_policy_idx;

    const double heartbeat_period = 1.0 / config.hmi.heartbeat_hz;
    auto last_command_at = Clock::now() -
        std::chrono::duration_cast<Clock::duration>(
            std::chrono::duration<double>(heartbeat_period));
    bool dirty = true;
    std::string last_logged_action;

    while (g_running) {
        const auto now = Clock::now();
        bool send_immediately = false;

        robot_base::ControlStatus latest_status;
        bool received_status = false;
        while (transport->RecvStatus(latest_status)) received_status = true;
        if (received_status) {
            const bool status_changed = StatusChanged(state, latest_status);
            ProcessStatus(&state, latest_status, now, &send_immediately);
            dirty = dirty || status_changed || send_immediately;
        }

        const bool waiting_for_ack = state.transition.active ||
            !state.pending_policy.empty();
        const double effective_status_timeout = waiting_for_ack
            ? std::max(config.hmi.status_timeout, config.hmi.request_timeout)
            : config.hmi.status_timeout;
        const bool online = state.has_status &&
            std::chrono::duration<double>(now - state.last_status_at).count()
                <= effective_status_timeout;
        if (online != state.status_online) {
            state.status_online = online;
            dirty = true;
            if (!online) {
                state.transition.active = false;
                state.pending_policy.clear();
                ZeroVelocity(&state);
                if (state.page == HmiPage::VELOCITY) state.page = HmiPage::MAIN;
                state.last_action = "Control 状态回传超时，速度已清零";
                send_immediately = true;
            } else {
                state.last_action = "Control 状态通道已连接";
            }
        }

        if (state.transition.active &&
            std::chrono::duration<double>(now - state.transition.requested_at).count()
                > config.hmi.request_timeout) {
            state.transition.active = false;
            state.last_action = "FSM 请求超时，已停止重发";
            send_immediately = true;
            dirty = true;
        }
        if (!state.pending_policy.empty() &&
            std::chrono::duration<double>(now - state.policy_requested_at).count()
                > config.hmi.request_timeout) {
            state.pending_policy.clear();
            state.last_action = "策略切换未获 Control 确认，已停止重发";
            send_immediately = true;
            dirty = true;
        }

        if (state.highlighted_key >= 0 && now >= state.highlight_until) {
            state.highlighted_key = -1;
            dirty = true;
        }

        const int key = ReadUiKey();
        if (key >= 0) {
            dirty = true;
            if (key == 'f') {
                send_immediately = RequestShortcut(&state, key, now) ||
                    send_immediately;
            } else if (state.page == HmiPage::POLICY_SELECT) {
                if ((key == kKeyUp || key == 'k') && !state.policies.empty()) {
                    state.policy_cursor_idx = (state.policy_cursor_idx - 1 +
                        static_cast<int>(state.policies.size())) %
                        static_cast<int>(state.policies.size());
                } else if ((key == kKeyDown || key == 'j') &&
                    !state.policies.empty()) {
                    state.policy_cursor_idx = (state.policy_cursor_idx + 1) %
                        static_cast<int>(state.policies.size());
                } else if ((key == '\r' || key == '\n') &&
                    !state.policies.empty()) {
                    if (!state.status_online) {
                        state.last_action = "策略未切换：Control 状态已断开";
                    } else if (state.status.mode != ControlMode::POWER_OFF &&
                        state.status.mode != ControlMode::DAMP) {
                        state.last_action =
                            "策略未切换：请先进入 POWER_OFF 或 DAMP";
                    } else if (state.policy_cursor_idx ==
                        state.active_policy_idx) {
                        state.last_action = "所选策略已经生效";
                        state.page = HmiPage::MAIN;
                    } else {
                        state.pending_policy =
                            state.policies[state.policy_cursor_idx];
                        state.policy_source = state.status.active_policy;
                        state.policy_requested_at = now;
                        state.last_action = "请求切换策略 → "
                            + state.pending_policy + "，等待 Control 确认";
                        state.page = HmiPage::MAIN;
                        send_immediately = true;
                    }
                } else if (key == kKeyEscape || key == 'p') {
                    state.policy_cursor_idx = state.active_policy_idx;
                    state.page = HmiPage::MAIN;
                    state.last_action = "取消策略选择";
                }
            } else if (state.page == HmiPage::VELOCITY) {
                const auto *limits = ActiveCommandLimits(state);
                if (key == kKeyEscape || key == 'v') {
                    ZeroVelocity(&state);
                    state.page = HmiPage::MAIN;
                    state.last_action = "退出速度控制，速度清零";
                    send_immediately = true;
                } else if (!limits) {
                    ZeroVelocity(&state);
                    state.page = HmiPage::MAIN;
                    state.last_action = "当前策略未配置速度命令范围";
                    send_immediately = true;
                } else if (key == 'w') {
                    state.target_command.vx = std::clamp(
                        state.target_command.vx + config.hmi.step_vx,
                        limits->min_vx, limits->max_vx);
                    state.last_action = "W：增加前进速度";
                    send_immediately = true;
                } else if (key == 's') {
                    state.target_command.vx = std::clamp(
                        state.target_command.vx - config.hmi.step_vx,
                        limits->min_vx, limits->max_vx);
                    state.last_action = "S：增加后退速度";
                    send_immediately = true;
                } else if (key == 'a') {
                    state.target_command.vy = std::clamp(
                        state.target_command.vy + config.hmi.step_vy,
                        limits->min_vy, limits->max_vy);
                    state.last_action = "A：增加左移速度";
                    send_immediately = true;
                } else if (key == 'd') {
                    state.target_command.vy = std::clamp(
                        state.target_command.vy - config.hmi.step_vy,
                        limits->min_vy, limits->max_vy);
                    state.last_action = "D：增加右移速度";
                    send_immediately = true;
                } else if (key == 'q') {
                    state.target_command.wz = std::clamp(
                        state.target_command.wz + config.hmi.step_wz,
                        limits->min_wz, limits->max_wz);
                    state.last_action = "Q：增加左转角速度";
                    send_immediately = true;
                } else if (key == 'e') {
                    state.target_command.wz = std::clamp(
                        state.target_command.wz - config.hmi.step_wz,
                        limits->min_wz, limits->max_wz);
                    state.last_action = "E：增加右转角速度";
                    send_immediately = true;
                } else if (key == ' ') {
                    ZeroVelocity(&state);
                    state.last_action = "SPACE：速度清零";
                    send_immediately = true;
                }
                if (send_immediately && (key == 'w' || key == 's' ||
                    key == 'a' || key == 'd' || key == 'q' ||
                    key == 'e' || key == ' ')) {
                    state.highlighted_key = key;
                    state.highlight_until = now + std::chrono::milliseconds(
                        config.hmi.key_highlight_ms);
                }
            } else {
                if (key == kKeyLeft) {
                    send_immediately = RequestByArrow(&state, false, now) ||
                        send_immediately;
                } else if (key == kKeyRight) {
                    send_immediately = RequestByArrow(&state, true, now) ||
                        send_immediately;
                } else if (key == 'p') {
                    if (state.policies.empty()) {
                        state.last_action = "没有配置可选策略";
                    } else {
                        state.policy_cursor_idx = state.active_policy_idx;
                        state.page = HmiPage::POLICY_SELECT;
                        state.last_action = "选择策略";
                    }
                } else if (key == 'v' || key == '\r' || key == '\n') {
                    if (state.status_online && state.status.hmi_connected &&
                        state.status.mode == ControlMode::RL &&
                        ActiveCommandLimits(state)) {
                        state.page = HmiPage::VELOCITY;
                        state.last_action = "进入键盘速度控制";
                    } else if (state.status_online &&
                        state.status.mode == ControlMode::RL) {
                        state.last_action =
                            "当前策略未配置 command.limits，不接受速度命令";
                    } else {
                        state.last_action =
                            "速度页仅在真实 FSM=RL 且心跳正常时开放";
                    }
                } else if (key == 'o' || key == 'h' || key == 'z' ||
                    key == 'r') {
                    send_immediately = RequestShortcut(&state, key, now) ||
                        send_immediately;
                } else if (key == ' ') {
                    ZeroVelocity(&state);
                    state.last_action = "速度清零";
                    send_immediately = true;
                } else if (key == 'w' || key == 's' || key == 'a' ||
                    key == 'd' || key == 'q' || key == 'e') {
                    state.last_action = "请按 V 或 Enter 进入速度控制页";
                }
            }
        }

        if (!state.last_action.empty() && state.last_action != last_logged_action) {
            runtime_logging::Log(
                runtime_logging::Level::kInfo, state.last_action, false);
            last_logged_action = state.last_action;
        }

        if (send_immediately ||
            std::chrono::duration<double>(now - last_command_at).count()
                >= heartbeat_period) {
            SendCommand(transport.get(), state);
            last_command_at = now;
        }

        if (dirty) {
            Render(state);
            dirty = false;
        }
        usleep(5000);
    }

    // 退出前最后发送一次速度清零 + POWER_OFF；control 端另有心跳超时兜底。
    state.transition.active = true;
    state.transition.key = -1;
    state.pending_policy.clear();
    ZeroVelocity(&state);
    SendCommand(transport.get(), state);
    runtime_logging::Log(runtime_logging::Level::kInfo,
        "HMI runtime stopped after requesting POWER_OFF", false);
    return 0;
}
