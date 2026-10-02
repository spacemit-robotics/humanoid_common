/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file terminal_ui.cpp
 * @brief ANSI pages extracted from the original HMI runtime
 */
#include "terminal_ui.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cwchar>
#include <string>
#include <sys/ioctl.h>
#include <unistd.h>
#include <vector>

namespace operator_terminal {
namespace {
using UiState = View;
using HmiPage = Page;
void GetSize(int &rows, int &cols) {
    struct winsize ws {};
    const bool valid = ioctl(STDOUT_FILENO, TIOCGWINSZ, &ws) == 0 && ws.ws_col > 0;
    rows = valid ? ws.ws_row : 24;
    cols = valid ? ws.ws_col : 80;
}
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
    printf("\033[%dm", static_cast<int>(color));
}

void SetBg(Color color) {
    printf("\033[%dm", static_cast<int>(color) + 10);
}

std::string Fit(const std::string &text, int columns) {
    std::mbstate_t conversion{};
    std::string result;
    int width = 0;
    for (size_t offset = 0; offset < text.size();) {
        wchar_t value = 0;
        const size_t length = std::mbrtowc(&value, text.data() + offset, text.size() - offset, &conversion);
        if (length == 0 || length == static_cast<size_t>(-1) || length == static_cast<size_t>(-2)) break;
        const int cells = wcwidth(value);
        if (cells > 0) {
            if (width + cells > columns - 1) return result + "~";
            result.append(text, offset, length);
            width += cells;
        }
        offset += length;
    }
    return result;
}

void HLine(int row, int col, int width, const char *left, const char *fill, const char *right) {
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
    GetSize(rows, cols);
    (void)rows;
    Layout layout;
    layout.width = std::min(66, std::max(54, cols - 2));
    layout.left = std::max(1, (cols - layout.width) / 2 + 1);
    layout.content_left = layout.left + 2;
    return layout;
}

const operator_service::Policy *ActivePolicy(const UiState &state) {
    const auto it = std::find_if(state.policies.begin(), state.policies.end(),
        [&](const auto &policy) { return policy.name == state.status.policy; });
    return it == state.policies.end() ? nullptr : &*it;
}
bool IsManualReferencePolicy(const UiState &state) {
    const auto *policy = ActivePolicy(state);
    return policy && policy->manual_reference;
}
const std::vector<operator_service::Action> *ActiveInteractionActions(const UiState &state) {
    if (state.status.trajectory_enabled) return &state.actions;
    const auto *policy = ActivePolicy(state);
    return policy ? &policy->actions : nullptr;
}
bool InteractionIsBusy(const std::string &phase) {
    return phase == "BLEND_IN" || phase == "PLAYING" || phase == "HOLDING" || phase == "BLEND_OUT";
}
const char *InteractionPhaseName(const std::string &phase) {
    if (phase == "BLEND_IN") return "进入";
    if (phase == "PLAYING") return "播放";
    if (phase == "HOLDING") return "保持";
    if (phase == "BLEND_OUT") return "收回";
    if (phase == "REJECTED") return "拒绝";
    if (phase == "FINISHED") return "完成";
    return "就绪";
}
std::string InteractionDisplayName(const UiState &state, const std::string &key) {
    const auto *actions = ActiveInteractionActions(state);
    if (actions) {
        const auto it =
            std::find_if(actions->begin(), actions->end(), [&](const auto &action) { return action.key == key; });
        if (it != actions->end()) return it->display_name;
    }
    return key;
}
bool ActiveModeHasInteractions(const UiState &state) {
    const auto *actions = ActiveInteractionActions(state);
    return state.status.online && state.status.hmi_connected &&
        state.status.state == (state.status.trajectory_enabled ? "TRAJECTORY" : "RL") && actions &&
        !actions->empty();
}
Color InteractionColor(const std::string &phase) {
    if (phase == "REJECTED") return Color::BRIGHT_RED;
    if (phase == "FINISHED") return Color::BRIGHT_GREEN;
    return Color::BRIGHT_CYAN;
}

void PrintInteractionProgress(const UiState &state, int width) {
    const auto &phase = state.status.interaction_phase;
    const float progress = std::clamp(state.status.interaction_progress, 0.0, 1.0);
    const int thumb = static_cast<int>(std::round(progress * std::max(width - 1, 0)));
    const Color fill_color = phase == "FINISHED" ? Color::GREEN : (phase == "REJECTED" ? Color::RED : Color::CYAN);
    ResetAttr();
    for (int index = 0; index < width; ++index) {
        SetFg(index <= thumb ? fill_color : Color::GRAY);
        if (index == thumb) {
            printf("●");
        } else if (index == 0 || index == width - 1) {
            printf("•");
        } else {
            printf("%s", index < thumb ? "━" : "─");
        }
    }
    ResetAttr();
    SetFg(fill_color);
    printf("  %3.0f%%", progress * 100.0F);
    ResetAttr();
}

const char *ModeName(const std::string &mode) { return mode == "TRAJECTORY" ? "动作" : mode.c_str(); }
Color ModeColor(const std::string &mode) {
    if (mode == "POWER_OFF") return Color::BRIGHT_RED;
    if (mode == "DAMP") return Color::BRIGHT_YELLOW;
    if (mode == "HOME") return Color::BLUE;
    if (mode == "ZERO") return Color::BRIGHT_CYAN;
    if (mode == "RL" || mode == "TRAJECTORY") return Color::BRIGHT_GREEN;
    if (mode == "SAFETY") return Color::MAGENTA;
    return Color::WHITE;
}

void PrintConnection(const UiState &state) {
    if (!state.status.online) {
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

void PrintHeader(const Layout &layout, const UiState &state, const char *title) {
    DrawBox(1, layout.left, layout.width, 3);
    MoveTo(2, layout.content_left);
    SetFg(Color::BRIGHT_CYAN);
    SetBold();
    printf("SpaceMIT Humanoid · %s", title);
    ResetAttr();
    MoveTo(2, layout.left + layout.width - 22);
    PrintConnection(state);
}

void PrintModeToken(const std::string &token, const UiState &state) {
    const char *label = ModeName(token);
    if (state.status.trajectory_enabled) {
        if (token == "POWER_OFF") label = "掉电";
        if (token == "DAMP") label = "阻尼";
        if (token == "HOME") label = "复位";
        if (token == "ZERO") label = "准备";
    }
    const bool active = state.status.online && state.status.state == token;
    const bool pending = !state.transition_target.empty() && state.transition_target == token;
    if (active) {
        SetFg(ModeColor(token));
        SetBold();
        printf("▶[%s]", label);
    } else if (pending) {
        SetFg(Color::MAGENTA);
        SetBold();
        printf("…[%s]", label);
    } else {
        SetDim();
        printf(" [%s]", label);
    }
    ResetAttr();
}

void PrintLastAction(const Layout &layout, int row, const std::string &last_action) {
    DrawHSep(row, layout.left, layout.width);
    MoveTo(row + 1, layout.content_left);
    SetFg(Color::CYAN);
    printf("最近: %s", Fit(last_action, layout.width - 10).c_str());
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
    MoveTo(5, layout.left + 18);
    SetFg(state.status.owns_control ? Color::BRIGHT_GREEN : Color::BRIGHT_YELLOW);
    SetBold();
    printf("控制权: %s",
        state.status.owns_control ? "本终端 [U] 释放"
            : (state.status.owner.empty() ? "无人持有 [L] 申请控制权"
                : Fit(state.status.owner, layout.width - 28).c_str()));
    ResetAttr();
    MoveTo(6, layout.content_left);
    PrintModeToken("POWER_OFF", state);
    printf(" ⇄ ");
    PrintModeToken("DAMP", state);
    printf(" ⇄ ");
    PrintModeToken("HOME", state);
    printf(" → ");
    PrintModeToken("ZERO", state);
    printf(" → ");
    PrintModeToken(state.status.trajectory_enabled ? "TRAJECTORY" : "RL", state);
    MoveTo(7, layout.content_left);
    if (state.status.online && state.status.fault.latched) {
        SetFg(state.status.fault.active ? Color::BRIGHT_RED : Color::BRIGHT_YELLOW);
        SetBold();
        const auto fault = Fit(state.status.fault.source + "/" + state.status.fault.code, layout.width - 30);
        printf("故障: %s %s%s", fault.c_str(),
            state.status.fault.active ? "ACTIVE" : "LATCHED", state.status.fault.active ? "" : "  [X] 确认");
    } else if (state.status.online && state.status.state == "SAFETY") {
        SetFg(Color::BRIGHT_RED);
        SetBold();
        printf("SAFETY 正在卸力；等待 Control 自动回到 POWER_OFF");
    } else if (state.status.online && !state.status.owns_control) {
        SetFg(Color::BRIGHT_YELLOW);
        SetBold();
        printf("%s", state.status.owner.empty() ? "已连接；按 L 申请控制权后可切换状态"
            : "当前仅查看；控制权由其他客户端持有");
    } else if (state.status.online && state.status.state == "ZERO") {
        SetFg(state.status.zero_ready ? Color::BRIGHT_GREEN : Color::BRIGHT_YELLOW);
        printf("回零: %s", !state.status.zero_ready ? "进行中，等待到位"
            : (state.status.trajectory_enabled ? "READY，可按 → 进入动作" : "READY，可按 → 进入 RL"));
    } else if (state.status.online && state.status.state == "RL" && IsManualReferencePolicy(state)) {
        SetFg(state.reference_start_requested ? Color::BRIGHT_GREEN : Color::BRIGHT_YELLOW);
        printf("%s",
            state.reference_start_requested ? "开始命令已发送；未动作可再次按 [G]"
                                            : "RL 已接管，等待人工确认    [G] 开始动作");
    } else if (ActiveModeHasInteractions(state)) {
        SetFg(Color::BRIGHT_CYAN);
        SetBold();
        if (InteractionIsBusy(state.status.interaction_phase)) {
            printf("交互动作执行中    [A] 动作列表    [C] 平滑取消");
        } else if (state.status.interaction_phase == "FINISHED") {
            printf("交互动作已完成    [A] 选择下一个动作");
        } else {
            printf("%s    [A] 选择交互动作", state.status.trajectory_enabled ? "动作模式已就绪" : "RL 策略已接管");
        }
    } else {
        SetDim();
        printf("← 后退；→ 前进；从%s按 ← 退回 DAMP", state.status.trajectory_enabled ? "动作" : " RL ");
    }
    ResetAttr();

    if (state.status.trajectory_enabled) {
        DrawBox(9, layout.left, layout.width, 8);
        MoveTo(10, layout.content_left);
        printf("动作 · TRAJECTORY    [A] 动作列表");
        MoveTo(12, layout.content_left);
        const auto action = InteractionDisplayName(state, state.status.interaction_action);
        printf("当前: %s", Fit(action.empty() ? "尚未播放" : action, layout.width - 10).c_str());
        MoveTo(13, layout.content_left);
        printf("状态: %s", InteractionPhaseName(state.status.interaction_phase));
        MoveTo(15, layout.content_left);
        PrintInteractionProgress(state, layout.width - 14);
    } else {
        DrawBox(9, layout.left, layout.width, 4);
        MoveTo(10, layout.content_left);
        SetDim();
        printf("策略");
        ResetAttr();
        MoveTo(11, layout.content_left);
        const std::string policy = state.status.policy.empty() ? "(未加载)" : state.status.policy;
        SetFg(Color::MAGENTA);
        SetBold();
        printf("当前: %s", Fit(policy, layout.width - 28).c_str());
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
        printf("vx=%+.2f m/s   vy=%+.2f m/s   wz=%+.2f rad/s", state.status.velocity.vx, state.status.velocity.vy,
            state.status.velocity.wz);
    }

    DrawBox(17, layout.left, layout.width, 5);
    MoveTo(18, layout.content_left);
    SetFg(Color::CYAN);
    printf("[←/→]");
    ResetAttr();
    printf(" FSM  ");
    SetFg(Color::CYAN);
    printf("%s", state.status.trajectory_enabled ? "[A]" : "[P]");
    ResetAttr();
    printf("%s", state.status.trajectory_enabled ? " 动作  " : " 策略  ");
    SetFg(Color::CYAN);
    printf("%s", state.status.trajectory_enabled ? "[C]" : "[V]");
    ResetAttr();
    printf("%s", state.status.trajectory_enabled ? " 取消动作" : " 速度  [Space] 清零");
    MoveTo(19, layout.content_left);
    SetFg(Color::BRIGHT_YELLOW);
    SetBold();
    printf("[L] 申请控制权");
    ResetAttr();
    printf("  [U] 释放  ");
    SetFg(Color::BRIGHT_RED);
    printf("[F]");
    ResetAttr();
    printf(" 掉电");
    MoveTo(20, layout.content_left);
    SetDim();
    printf("%s", state.status.trajectory_enabled ? "[A]动作 [C]取消 [X]确认 [Ctrl+C]退出"
        : "[A]动作 [G]开始 [C]取消 [X]确认 [Ctrl+C]退出");
    ResetAttr();

    PrintLastAction(layout, 22, state.last_action);
    fflush(stdout);
}

void RenderPolicySelectPage(const UiState &state) {
    const Layout layout = GetLayout();
    ClearScreen();
    PrintHeader(layout, state, "策略选择");

    const int count = static_cast<int>(state.policies.size());
    const int list_rows = std::min(9, std::max(1, count));
    const int first = std::clamp(state.policy_cursor_idx - 4, 0, std::max(0, count - list_rows));
    DrawBox(4, layout.left, layout.width, list_rows + 3);
    MoveTo(5, layout.content_left);
    SetDim();
    printf("绿色 ● 为 Control 当前生效策略");
    ResetAttr();
    if (state.policies.empty()) {
        MoveTo(6, layout.content_left);
        printf("(没有配置可选策略)");
    } else {
        for (int row = 0; row < list_rows; ++row) {
            const int i = first + row;
            MoveTo(6 + row, layout.content_left);
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
            printf("%-28s", Fit(state.policies[i].name, layout.width - 8).c_str());
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

int InteractionVisibleRows(const UiState &state) {
    const auto *actions = ActiveInteractionActions(state);
    return std::min(9, std::max(1, actions ? static_cast<int>(actions->size()) : 0));
}

void RenderInteractionSelectPage(const UiState &state) {
    const Layout layout = GetLayout();
    ClearScreen();
    PrintHeader(layout, state, "交互动作");

    const auto *actions = ActiveInteractionActions(state);
    constexpr int kVisibleRows = 9;
    const int count = actions ? static_cast<int>(actions->size()) : 0;
    const int cursor = count == 0 ? 0 : std::clamp(state.interaction_cursor_idx, 0, count - 1);
    const int first = std::clamp(cursor - kVisibleRows / 2, 0, std::max(0, count - kVisibleRows));
    const int shown = InteractionVisibleRows(state);
    DrawBox(4, layout.left, layout.width, shown + 3);
    MoveTo(5, layout.content_left);
    SetDim();
    if (!state.status.trajectory_enabled)
        printf("策略: %s    ", Fit(state.status.policy, layout.width - 28).c_str());
    printf("动作 %d/%d",
        count == 0 ? 0 : cursor + 1, count);
    ResetAttr();
    if (count == 0) {
        MoveTo(6, layout.content_left);
        printf("%s", state.status.trajectory_enabled ? "(没有配置动作)" : "(当前策略没有注册交互动作)");
    } else {
        for (int row = 0; row < shown; ++row) {
            const int index = first + row;
            MoveTo(6 + row, layout.content_left);
            if (index == cursor) {
                SetBg(Color::BLUE);
                SetFg(Color::WHITE);
                SetBold();
            }
            const auto &action = (*actions)[index];
            const bool current_action = state.status.interaction_action == action.key;
            if (current_action) {
                printf("%s ", state.status.interaction_phase == "FINISHED" ? "✓" : "▶");
            } else {
                printf("  ");
            }
            printf("%s", Fit(action.display_name, layout.width - 8).c_str());
            ResetAttr();
        }
    }

    const int operation_row = 4 + shown + 3;
    DrawBox(operation_row, layout.left, layout.width, 7);
    MoveTo(operation_row + 1, layout.content_left);
    const std::string current_action = InteractionDisplayName(state, state.status.interaction_action);
    printf("当前: %s", state.status.interaction_action.empty() ? "尚未播放"
        : Fit(current_action, layout.width - 25).c_str());
    printf("   ·   ");
    SetFg(InteractionColor(state.status.interaction_phase));
    SetBold();
    printf("%s",
        state.status.interaction_phase == "FINISHED" ? "✓ 已完成"
            : InteractionPhaseName(state.status.interaction_phase));
    ResetAttr();
    MoveTo(operation_row + 2, layout.content_left);
    PrintInteractionProgress(state, layout.width - 14);
    MoveTo(operation_row + 3, layout.content_left);
    printf("↑/↓ 或 j/k 移动    Enter 播放    C 取消    Esc/A 返回");
    MoveTo(operation_row + 4, layout.content_left);
    SetFg(InteractionColor(state.status.interaction_phase));
    if (InteractionIsBusy(state.status.interaction_phase)) {
        printf("执行中；可浏览其他动作，完成后按 Enter 播放下一项");
    } else if (state.status.interaction_phase == "FINISHED") {
        printf("动作已完成，可以直接选择并播放下一项");
    } else {
        printf("%s", state.status.trajectory_enabled ? "动作" : "动作由当前 RL 策略及其适配器执行");
    }
    ResetAttr();
    PrintLastAction(layout, operation_row + 7, state.last_action);
    fflush(stdout);
}

void PrintKey(int row, int col, const char *key, const char *action, bool highlighted) {
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
    const std::string policy = state.status.policy.empty() ? "-" : state.status.policy;
    printf("FSM: ");
    SetFg(ModeColor(state.status.state));
    SetBold();
    printf("%s", ModeName(state.status.state));
    ResetAttr();
    printf("    策略: %s    RL: %.1f Hz", Fit(policy, layout.width - 43).c_str(), state.status.rl_hz);
    MoveTo(6, layout.content_left);
    printf("目标  vx=%+.2f   vy=%+.2f   wz=%+.2f", state.target_command.vx, state.target_command.vy,
        state.target_command.wz);
    MoveTo(7, layout.content_left);
    SetFg(Color::BRIGHT_GREEN);
    printf("实际  vx=%+.2f   vy=%+.2f   wz=%+.2f", state.status.velocity.vx, state.status.velocity.vy,
        state.status.velocity.wz);
    ResetAttr();

    DrawBox(9, layout.left, layout.width, 9);
    MoveTo(10, layout.content_left);
    SetDim();
    printf("每次按键按配置步长加减，速度范围按当前策略限制");
    ResetAttr();
    PrintKey(12, layout.content_left + 2, "Q", "左转", state.highlighted_key == 'q');
    PrintKey(12, layout.content_left + 21, "W", "前进", state.highlighted_key == 'w');
    PrintKey(12, layout.content_left + 40, "E", "右转", state.highlighted_key == 'e');
    PrintKey(14, layout.content_left + 2, "A", "左移", state.highlighted_key == 'a');
    PrintKey(14, layout.content_left + 21, "S", "后退", state.highlighted_key == 's');
    PrintKey(14, layout.content_left + 40, "D", "右移", state.highlighted_key == 'd');
    PrintKey(16, layout.content_left + 21, "SPACE", "清零", state.highlighted_key == ' ');

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

} // namespace

void Render(const View &state) {
    printf("\033[?2026h");
    switch (state.page) {
    case HmiPage::POLICY_SELECT:
        RenderPolicySelectPage(state);
        break;
    case HmiPage::INTERACTION_SELECT:
        RenderInteractionSelectPage(state);
        break;
    case HmiPage::VELOCITY:
        RenderVelocityPage(state);
        break;
    case HmiPage::MAIN:
        RenderMainPage(state);
        break;
    }
    printf("\033[?2026l");
    fflush(stdout);
}

} // namespace operator_terminal
