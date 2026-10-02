/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file hmi_tui.cpp
 * @brief Original HMI interaction through the public operator client
 */
#include "operator_client.h"
#include "terminal_ui.h"
#include <algorithm>
#include <cctype>
#include <chrono>
#include <clocale>
#include <csignal>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <termios.h>
#include <thread>
#include <unistd.h>

namespace {
using Clock = std::chrono::steady_clock;
using operator_terminal::Page;
volatile std::sig_atomic_t running = 1;
void Signal(int) { running = 0; }
constexpr int kUp = 1000, kDown = 1001, kLeft = 1002, kRight = 1003;

class Terminal {
public:
    Terminal() {
        if (!isatty(STDOUT_FILENO) || tcgetattr(STDIN_FILENO, &original_) != 0)
            throw std::runtime_error("TUI 需要交互终端；单次查询使用 --status");
        auto raw = original_;
        raw.c_lflag &= static_cast<unsigned>(~(ICANON | ECHO));
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) throw std::runtime_error("无法设置终端");
        std::printf("\033[?1049h\033[?25l");
    }
    ~Terminal() {
        std::printf("\033[0m\033[?25h\033[?1049l");
        std::fflush(stdout);
        tcsetattr(STDIN_FILENO, TCSANOW, &original_);
    }
    int Read() const {
        const int key = Byte();
        if (key != 27) return key;
        if (SequenceByte() != '[') return 27;
        switch (SequenceByte()) {
        case 'A':
            return kUp;
        case 'B':
            return kDown;
        case 'C':
            return kRight;
        case 'D':
            return kLeft;
        default:
            return 27;
        }
    }

private:
    static int Byte() {
        unsigned char byte;
        return ::read(STDIN_FILENO, &byte, 1) == 1 ? byte : -1;
    }
    static int SequenceByte() {
        for (int i = 0; i < 5; ++i) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            const int key = Byte();
            if (key >= 0) return key;
        }
        return -1;
    }
    termios original_{};
};

std::string Next(const std::string &s, bool trajectory_enabled) {
    if (s == "POWER_OFF") return "DAMP";
    if (s == "DAMP") return "HOME";
    if (s == "HOME") return "ZERO";
    if (s == "ZERO") return trajectory_enabled ? "TRAJECTORY" : "RL";
    return s;
}
std::string ReplyMessage(const operator_service::Reply &reply) {
    if (reply.ok) {
        if (reply.message == "control acquired") return "已获得控制权";
        if (reply.message == "released") return "已释放控制权";
        return "已受理";
    }
    if (reply.code == "not_owner") return "请先按 L 申请控制权";
    if (reply.code == "busy") {
        if (reply.message == "another client owns control") return "其他客户端持有控制权，请等待释放";
        return "上一请求尚未完成，请稍候";
    }
    if (reply.message == "waiting for fresh control status" || reply.message == "control feedback is unavailable")
        return "等待 Control 就绪后再操作";
    if (reply.code == "disconnected") return "与 HMI 服务的连接已断开";
    if (reply.code == "timeout") return "HMI 服务响应超时";
    return reply.code + " " + reply.message;
}
void Run(operator_service::Client *client) {
    Terminal terminal;
    operator_terminal::View view;
    view.policies = client->Policies();
    view.actions = client->Actions();
    const auto step = client->VelocityStep();
    auto last_renew = Clock::now(), last_velocity = last_renew, highlight_until = last_renew;
    uint64_t pending = 0;
    const auto reply = [&](const operator_service::Reply &r, const std::string &label) {
        view.last_action = label + "：" + ReplyMessage(r);
        if (r.ok && r.request.phase == "accepted") pending = r.request.sequence;
    };
    const auto clear = [&] {
        view.target_command = {};
        if (view.status.owns_control && view.status.state == "RL") client->SetVelocity({});
    };
    const auto transition = [&](const std::string &s) {
        clear();
        const auto r = client->RequestState(s);
        reply(r, "切换 " + s);
        view.transition_target = r.ok && r.request.phase == "accepted" ? s : "";
    };
    while (running && client->Connected()) {
        const auto now = Clock::now();
        view.status = client->LatestStatus();
        const auto &s = view.status;
        if (!s.owns_control || !s.online || s.state != "RL") view.target_command = {};
        if (pending && s.request.sequence == pending && s.request.phase != "accepted") {
            view.last_action =
                s.request.phase == "completed" ? "请求已确认" : s.request.phase + ": " + s.request.message;
            pending = 0;
            view.transition_target.clear();
        }
        if (s.owns_control && now - last_renew >= std::chrono::milliseconds(250)) {
            const auto r = client->RenewControl();
            if (!r.ok) reply(r, "控制权续期");
            last_renew = now;
        }
        const auto policy =
            std::find_if(view.policies.begin(), view.policies.end(), [&](const auto &p) { return p.name == s.policy; });
        view.active_policy_idx = static_cast<int>(policy - view.policies.begin());
        const int raw = terminal.Read();
        const int key = raw >= 0 && raw < 256 ? std::tolower(raw) : raw;
        if (raw >= 0) {
            view.highlighted_key = key;
            highlight_until = now + std::chrono::milliseconds(180);
        } else if (now >= highlight_until) view.highlighted_key = -1;

        if (key == 'f') {
            transition("POWER_OFF");
            view.page = Page::MAIN;
        } else if (key == 'o') transition("DAMP");
        else if (key == 'l') reply(client->AcquireControl(), "申请控制权");
        else if (key == 'u') {
            clear();
            reply(client->ReleaseControl(), "释放控制权");
        } else if (key == ' ') {
            clear();
            view.last_action = "速度已清零";
        } else if (key == 'x') reply(client->AcknowledgeFault(), "确认故障");
        else if (key == 'c') reply(client->CancelInteraction(), "取消动作");
        else if (key == 'g' && !s.trajectory_enabled) {
            const auto r = client->StartReference();
            reply(r, "开始参考动作");
            view.reference_start_requested = r.ok;
        } else if (key == 27) {
            clear();
            view.page = Page::MAIN;
        } else if (view.page == Page::VELOCITY) {
            if (key == 'v') {
                clear();
                view.page = Page::MAIN;
            }
            if (s.owns_control && s.state == "RL" && policy != view.policies.end()) {
                auto &v = view.target_command;
                if (key == 'w') v.vx += step.vx;
                if (key == 's') v.vx -= step.vx;
                if (key == 'a') v.vy += step.vy;
                if (key == 'd') v.vy -= step.vy;
                if (key == 'q') v.wz += step.wz;
                if (key == 'e') v.wz -= step.wz;
                v.vx = std::clamp(v.vx, policy->minimum.vx, policy->maximum.vx);
                v.vy = std::clamp(v.vy, policy->minimum.vy, policy->maximum.vy);
                v.wz = std::clamp(v.wz, policy->minimum.wz, policy->maximum.wz);
            }
        } else if (view.page == Page::POLICY_SELECT || view.page == Page::INTERACTION_SELECT) {
            const bool policies = view.page == Page::POLICY_SELECT;
            const auto *actions = s.trajectory_enabled ? &view.actions
                : (policy == view.policies.end() ? nullptr : &policy->actions);
            const int count = policies          ? view.policies.size()
                : actions ? actions->size() : 0;
            int &cursor = policies ? view.policy_cursor_idx : view.interaction_cursor_idx;
            cursor = std::clamp(cursor, 0, std::max(0, count - 1));
            if (key == kUp || key == 'k') cursor = std::max(0, cursor - 1);
            if (key == kDown || key == 'j') cursor = std::min(std::max(0, count - 1), cursor + 1);
            if ((key == '\r' || key == '\n') && count) {
                if (policies) reply(client->SelectPolicy(view.policies[cursor].name), "选择策略");
                else reply(client->StartInteraction((*actions)[cursor].key), "播放动作");
            }
            if ((policies && key == 'p') || (!policies && key == 'a')) view.page = Page::MAIN;
        } else {
            if (key == 'p' && !s.trajectory_enabled) {
                view.page = Page::POLICY_SELECT;
                view.policy_cursor_idx = view.active_policy_idx;
            }
            if (key == 'a') {
                view.page = Page::INTERACTION_SELECT;
                view.interaction_cursor_idx = 0;
            }
            if (key == 'v' || key == '\r' || key == '\n')
                view.page = s.trajectory_enabled ? Page::INTERACTION_SELECT : Page::VELOCITY;
            if (key == kRight) transition(Next(s.state, s.trajectory_enabled));
            if (key == kLeft) transition(s.state == "DAMP" ? "POWER_OFF" : "DAMP");
            if (key == 'h') transition("HOME");
            if (key == 'z') transition("ZERO");
            if (key == 'r') transition(s.trajectory_enabled ? "TRAJECTORY" : "RL");
        }
        if (s.owns_control && s.state == "RL" && now - last_velocity >= std::chrono::milliseconds(100)) {
            const auto r = client->SetVelocity(view.target_command);
            if (!r.ok) {
                view.target_command = {};
                reply(r, "速度请求");
            }
            last_velocity = now;
        }
        operator_terminal::Render(view);
        std::this_thread::sleep_for(std::chrono::milliseconds(40));
    }
    if (client->Connected()) client->ReleaseControl();
}
} // namespace

int main(int argc, char *argv[]) {
    std::setlocale(LC_CTYPE, "");
    std::string file;
    bool once = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--connection" && i + 1 < argc) file = argv[++i];
        else if (arg == "--status") once = true;
        else {
            std::printf("Usage: %s --connection FILE [--status]\n", argv[0]);
            return arg == "--help" ? 0 : 1;
        }
    }
    if (file.empty()) {
        std::fprintf(stderr, "--connection is required\n");
        return 1;
    }
    try {
        operator_service::Client client;
        std::string error;
        if (!client.Connect(operator_service::Client::ReadConnectionFile(file), &error))
            throw std::runtime_error(error);
        if (once) {
            const auto s = client.LatestStatus();
            std::printf("robot=%s state=%s online=%d zero_ready=%d owner=%s", client.RobotName().c_str(),
                s.state.c_str(), s.online, s.zero_ready, s.owner.c_str());
            if (!s.trajectory_enabled) std::printf(" policy=%s", s.policy.c_str());
            std::puts("");
            return 0;
        }
        std::signal(SIGINT, Signal);
        std::signal(SIGTERM, Signal);
        Run(&client);
        if (!client.Connected()) throw std::runtime_error("连接已断开：" + client.LastError());
    } catch (const std::exception &e) {
        std::fprintf(stderr, "%s\n", e.what());
        return 1;
    }
    return 0;
}
