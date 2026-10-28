/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file hmi_gamepad.cpp
 * @brief Local gamepad operator client with explicit ownership and reconnect
 */
#include "evdev_device.h"
#include "gamepad_config.h"
#include "operator_client.h"
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <thread>

namespace {
using Clock = std::chrono::steady_clock;
using operator_service::Client;
using operator_service::Velocity;
using operator_gamepad::Binding;
using operator_gamepad::Config;
volatile std::sig_atomic_t running = 1;
void Signal(int) { running = 0; }

void Report(const std::string &label, const operator_service::Reply &reply) {
    std::printf("[gamepad] %s: %s %s\n", label.c_str(), reply.ok ? "accepted" : reply.code.c_str(),
        reply.message.c_str());
}
void Run(const Config &config, const std::string &connection_path, bool dry_run,
    const std::string &dry_policy) {
    Client client;
    operator_gamepad::Device device;
    operator_gamepad::Chords chords(config.bindings);
    operator_gamepad::SuccessFeedback feedback;
    operator_gamepad::Rumble rumble;
    bool armed = false, fast = false, owned = false;
    Velocity output{};
    std::string previous_error, previous_policy, previous_state, previous_fault;
    auto next_device = Clock::now(), next_connection = next_device, last_renew = next_device;
    auto last_send = next_device;
    const auto clear = [&] {
        armed = false;
        output = {};
        if (!dry_run && client.Connected() && owned) client.SetVelocity({});
    };
    const auto release = [&] {
        feedback.Clear();
        clear();
        if (!dry_run && client.Connected() && owned) Report("release", client.ReleaseControl());
        owned = false;
    };
    const auto error_once = [&](const std::string &error) {
        if (error != previous_error) {
            std::printf("[gamepad] %s\n", error.c_str());
            previous_error = error;
        }
    };
    std::printf("[gamepad] %s; no automatic ownership or motor enable\n",
        dry_run ? "READ-ONLY INPUT TEST" : "local operator client");
    while (running) {
        const auto now = Clock::now();
        if (!device.Opened() && now >= next_device) {
            try {
                device.Open(config.device, config.vendor, config.product, config.layout,
                    config.input, config.deadzone, &chords);
                armed = false;
                previous_error.clear();
                if (!dry_run) rumble.Start(config.feedback, config.device, config.vendor, config.product);
                std::printf("[gamepad] connected %s (%s); release buttons and center sticks\n",
                    config.device.c_str(), config.layout.c_str());
            } catch (const std::exception &e) { error_once(e.what()); }
            next_device = now + std::chrono::seconds(1);
        }
        if (!dry_run && !client.Connected() && now >= next_connection) {
            owned = false;
            armed = false;
            output = {};
            try {
                auto connection = Client::ReadConnectionFile(connection_path);
                connection.name = "gamepad";
                // Local loopback, not phone Wi-Fi. A slow reply does not close the connection.
                connection.timeout_ms = 500;
                std::string error;
                if (!client.Connect(connection, &error)) throw std::runtime_error(error);
                std::printf("[gamepad] operator service connected; waiting for explicit takeover\n");
            } catch (const std::exception &e) { error_once(e.what()); }
            next_connection = Clock::now() + std::chrono::seconds(1);
        }
        if (!device.Opened()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }
        std::vector<Binding> commands;
        try {
            const auto status = client.LatestStatus();
            chords.SetPolicy(dry_run ? dry_policy : (status.online ? status.policy : ""));
            bool resynced = false;
            commands = device.Read(&chords, &resynced);
            if (resynced) clear();
        } catch (const std::exception &e) {
            release();
            device.Close();
            rumble.Stop();
            error_once(e.what());
            continue;
        }
        const auto input = device.Velocity();
        auto status = client.LatestStatus();
        if (!dry_run) {
            if (feedback.Update(status)) rumble.Pulse();
            owned = status.owns_control;
            if (!status.online || !owned) {
                armed = false;
                output = {};
            }
            if (status.policy != previous_policy || status.state != previous_state) {
                armed = false;
                output = {};
                previous_policy = status.policy;
                previous_state = status.state;
                std::printf("[gamepad] state=%s policy=%s\n", status.state.c_str(), status.policy.c_str());
            }
            const auto fault = status.fault.latched ? status.fault.source + "/" + status.fault.code : "";
            if (fault != previous_fault) {
                previous_fault = fault;
                std::printf("[gamepad] fault=%s\n", fault.empty() ? "none" : fault.c_str());
            }
        }
        for (const auto &command : commands) {
            const auto label = command.operation + " " + command.value;
            if (dry_run) {
                std::printf("[gamepad] would request %s\n", label.c_str());
                continue;
            }
            status = client.LatestStatus();
            // Explicit operator power-off does not require a fresh status or
            // ownership. Match the service's existing safety-request contract.
            if (client.Connected() && operator_gamepad::IsPowerOffRequest(command)) {
                armed = false;
                output = {};
                const auto reply = client.RequestState("POWER_OFF");
                Report(label, reply);
                feedback.Track(command.operation, command.value, status, reply);
                continue;
            }
            if (!client.Connected() || !status.online) {
                std::printf("[gamepad] %s ignored: operator service not ready\n", label.c_str());
                continue;
            }
            // A mode can change while input is drained or a preceding request completes.
            if (!operator_gamepad::Applies(command, status.policy)) continue;
            if (command.operation == "control") {
                if (owned) release();
                else if (operator_gamepad::Neutral(input)) {
                    const auto reply = client.AcquireControl();
                    Report("acquire", reply);
                    owned = reply.ok;
                    fast = status.teleop_preset == "fast";
                    armed = false;
                } else std::printf("[gamepad] center sticks before takeover\n");
                continue;
            }
            if (!owned) {
                std::printf("[gamepad] acquire control with the configured binding first\n");
                continue;
            }
            if (command.operation == "preset") {
                const bool next_fast = command.value == "toggle" ? status.teleop_preset != "fast" :
                    command.value == "fast";
                if (next_fast != (status.teleop_preset == "fast")) {
                    const auto reply = client.SetTeleopPreset(next_fast ? "fast" : "slow");
                    Report(label, reply);
                    if (reply.ok) fast = next_fast;
                }
                continue;
            }
            clear();
            operator_service::Reply reply;
            if (command.operation == "state") {
                if (command.value == "RL" && !operator_gamepad::Neutral(input)) {
                    std::printf("[gamepad] center sticks before entering RL\n");
                    continue;
                }
                reply = client.RequestState(command.value);
            } else if (command.operation == "policy") reply = client.SelectPolicy(command.value);
            else if (command.operation == "action") reply = client.StartInteraction(command.value);
            else if (command.operation == "cancel") reply = client.CancelInteraction();
            else if (command.operation == "ack") reply = client.AcknowledgeFault();
            Report(label, reply);
            feedback.Track(command.operation, command.value, status, reply);
        }
        if (dry_run) {
            // Direction bands, not raw samples: no output while sticks are held still.
            const auto band = [](double x) { return static_cast<int>(std::round(x * 4)); };
            static std::array<int, 3> last{};
            const std::array<int, 3> current{band(input.vx), band(input.vy), band(input.wz)};
            if (current != last) {
                last = current;
                std::printf("[gamepad] input forward=%+.2f left=%+.2f turn_left=%+.2f\n",
                    input.vx, input.vy, input.wz);
            }
        } else if (client.Connected()) {
            status = client.LatestStatus();
            owned = status.owns_control;
            fast = status.teleop_preset == "fast";
            if (owned && now - last_renew >= std::chrono::milliseconds(250)) {
                const auto reply = client.RenewControl();
                if (!reply.ok) error_once("renew: " + reply.code);
                last_renew = Clock::now();
            }
            const bool ready = owned && status.online && status.state == "RL" &&
                !status.fault.latched && status.request.phase != "accepted" &&
                (status.interaction_phase == "IDLE" || status.interaction_phase == "FINISHED" ||
                status.interaction_phase == "REJECTED");
            if (!ready) {
                armed = false;
                output = {};
            } else if (operator_gamepad::Neutral(input)) armed = true;
            if (owned && status.online && status.state == "RL" &&
                now - last_send >= std::chrono::milliseconds(20)) {
                const auto policies = client.Policies();
                const auto it = std::find_if(policies.begin(), policies.end(),
                    [&](const auto &p) { return p.name == status.policy; });
                Velocity target{};
                const double dt = std::clamp(std::chrono::duration<double>(now - last_send).count(), 0.0, 0.1);
                if (ready && armed && it != policies.end()) {
                    target = operator_gamepad::Scale(input, *it, fast ? config.fast : config.slow);
                    if (it->client_smoothing) {
                        target = {
                            operator_gamepad::Advance(output.vx, target.vx, config.acceleration[0],
                                config.deceleration[0], dt),
                            operator_gamepad::Advance(output.vy, target.vy, config.acceleration[1],
                                config.deceleration[1], dt),
                            operator_gamepad::Advance(output.wz, target.wz, config.acceleration[2],
                                config.deceleration[2], dt)};
                    }
                }
                const auto reply = client.SetVelocity(target, 300);
                if (reply.ok) output = target;
                else {
                    armed = false;
                    output = {};
                    error_once("velocity: " + reply.code + " " + reply.message);
                }
                last_send = Clock::now();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    release();
}
}  // namespace

int main(int argc, char **argv) {
    std::setvbuf(stdout, nullptr, _IOLBF, 0);
    std::signal(SIGINT, Signal);
    std::signal(SIGTERM, Signal);
    try {
        if (argc < 2) throw std::runtime_error(
            "usage: hmi_gamepad CONFIG [--connection FILE] [--dry-run [--policy NAME]|--check-config]");
        std::string connection, dry_policy;
        bool dry_run = false, check = false;
        for (int i = 2; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--connection" && i + 1 < argc) connection = argv[++i];
            else if (arg == "--policy" && i + 1 < argc) dry_policy = argv[++i];
            else if (arg == "--dry-run") dry_run = true;
            else if (arg == "--check-config") check = true;
            else throw std::runtime_error("unknown gamepad argument");
        }
        const auto root = YAML::LoadFile(argv[1]);
        if (!root["operator_gamepad"]["enabled"].as<bool>(false)) return 0;
        const auto config = operator_gamepad::ReadConfig(root);
        if (check) {
            std::printf("gamepad configuration valid: %zu bindings\n", config.bindings.size());
            return 0;
        }
        if (!dry_run && !dry_policy.empty()) throw std::runtime_error("--policy requires --dry-run");
        if (dry_run) {
            const auto policies = root["rl_policy"]["onnx_infer"]["policy_names"].as<std::vector<std::string>>();
            if (dry_policy.empty()) dry_policy = policies.at(0);
            if (std::find(policies.begin(), policies.end(), dry_policy) == policies.end())
                throw std::runtime_error("unknown dry-run policy");
            std::printf("[gamepad] input-test policy=%s (no policy or FSM changes)\n", dry_policy.c_str());
        }
        if (!dry_run && connection.empty()) throw std::runtime_error("--connection required");
        Run(config, connection, dry_run, dry_policy);
    } catch (const std::exception &e) {
        std::fprintf(stderr, "[gamepad] %s\n", e.what());
        return 1;
    }
    return 0;
}
