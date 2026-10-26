/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file operator_audio.cpp
 * @brief Optional non-realtime status observer and bounded WAV playback process
 */
#include <sys/resource.h>
#include <yaml-cpp/yaml.h>

#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem> // NOLINT(build/c++17): SDK requires C++17.
#include <stdexcept>
#include <string>
#include <thread>

#include "audio_player.h"
#include "input_connection.h"
#include "operator_client.h"
#include "prompt_events.h"

namespace {
using operator_audio::Clip;
using operator_audio::Config;
using operator_audio::Player;
volatile std::sig_atomic_t stopped = 0;
void Stop(int) { stopped = 1; }
Config Load(const std::string &path) {
    const auto yaml = YAML::LoadFile(path);
    const auto audio = yaml["audio_prompts"];
    Config config;
    if (!audio || !audio["enabled"].as<bool>(false)) return config;
    config.enabled = true;
    config.player = audio["player"].as<std::vector<std::string>>();
    config.timeout = audio["max_play_seconds"].as<double>(5);
    config.fault_repeat_interval_s = audio["fault_repeat_interval_s"].as<double>(0);
    if (config.player.empty() || !(config.timeout > 0 && config.timeout <= 10))
        throw std::runtime_error("invalid audio player or timeout");
    if (!std::isfinite(config.fault_repeat_interval_s) || config.fault_repeat_interval_s < 0)
        throw std::runtime_error("invalid fault repeat interval");
    const auto robot_dir = std::filesystem::path(path).parent_path() /
        yaml["robot_base"]["robot_dir"].as<std::string>("..");
    const auto directory = robot_dir / audio["directory"].as<std::string>();
    for (const auto &clip : audio["clips"]) {
        const auto value = clip.second;
        Clip spec;
        spec.music = value.IsMap() && value["music"].as<bool>(false);
        spec.timeout = value.IsMap() ? value["max_play_seconds"].as<double>(config.timeout) : config.timeout;
        if (!(spec.timeout > 0 && spec.timeout <= (spec.music ? 180 : 10)))
            throw std::runtime_error("invalid clip timeout");
        const auto file = directory / (value.IsMap() ? value["file"].as<std::string>() : value.as<std::string>());
        spec.file = file.lexically_normal().string();
        if (value.IsMap() && value["prefix"])
            spec.prefix = (directory / value["prefix"].as<std::string>()).lexically_normal().string();
        config.clips.emplace(clip.first.as<std::string>(), std::move(spec));
    }
    return config;
}
operator_audio::InputConnection LoadGamepad(const std::string &path) {
    const auto gamepad = YAML::LoadFile(path)["operator_gamepad"];
    if (!gamepad || !gamepad["enabled"].as<bool>(false)) return {"", 0, 0};
    return {gamepad["device"].as<std::string>(), gamepad["vendor"].as<int>(),
            gamepad["product"].as<int>()};
}
} // namespace

int main(int argc, char **argv) {
    if (argc < 2 || std::string(argv[1]) == "--help") {
        std::puts("Usage: operator_audio config.yaml --connection connection.json [--dry-run]\n"
            "       operator_audio config.yaml --once EVENT [--dry-run]\n"
            "       operator_audio config.yaml --service-failure");
        return argc < 2 ? 1 : 0;
    }
    try {
        std::string connection, once;
        bool dry_run = false, service_failure = false;
        for (int i = 2; i < argc; ++i) {
            const std::string option = argv[i];
            if (option == "--dry-run") dry_run = true;
            else if (option == "--service-failure") service_failure = true;
            else if ((option == "--connection" || option == "--once") && i + 1 < argc) {
                (option == "--connection" ? connection : once) = argv[++i];
            } else throw std::runtime_error("unknown or incomplete option");
        }
        if (service_failure) {
            const char *result = std::getenv("SERVICE_RESULT");
            // systemd invokes ExecStopPost on intentional shutdown too.
            if (!result || std::string(result) == "success") return 0;
            once = "service.restarting";
        }
        const auto config = Load(argv[1]);
        if (!config.enabled) return 0;
        if (connection.empty() == once.empty()) throw std::runtime_error("choose connection or one event");
        std::signal(SIGINT, Stop);
        std::signal(SIGTERM, Stop);
        // Playback is background work; children inherit this non-realtime niceness.
        if (setpriority(PRIO_PROCESS, 0, 10) != 0)
            std::fprintf(stderr, "[audio] cannot lower process priority: %d\n", errno);
        Player player(config, dry_run);
        if (!once.empty()) {
            player.Queue(once);
            while (!stopped && player.Busy()) {
                player.Tick();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
            }
            return 0;
        }
        bool announce_ready = true;
        operator_service::PromptEvents events(config.fault_repeat_interval_s);
        operator_service::GamepadEvents gamepad_events;
        auto gamepad = LoadGamepad(argv[1]);
        while (!stopped) {
            operator_service::Client client;
            try {
                auto options = operator_service::Client::ReadConnectionFile(connection);
                options.name = "audio-observer";
                std::string error;
                if (!client.Connect(options, &error)) throw std::runtime_error(error);
                events.Reset();
                while (!stopped && client.Connected()) {
                    const auto status = client.LatestStatus();
                    player.Synchronize(status);
                    if (!status.online) player.Clear();
                    else if (announce_ready) {
                        announce_ready = false;
                        player.Queue(status.fault.latched ? "fault" : "service.ready");
                    }
                    for (const auto &cue : events.Update(status)) player.Queue(cue);
                    if (status.online) {
                        const auto cue = gamepad_events.Update(gamepad.Connected());
                        if (!cue.empty()) player.Queue(cue);
                    }
                    player.Tick();
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
            } catch (const std::exception &error) {
                std::fprintf(stderr, "[audio] waiting for local HMI: %s\n", error.what());
            }
            player.Clear();
            // Observation only: no acquire, renew, velocity, state or stop request.
            for (int i = 0; i < 20 && !stopped; ++i)
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    } catch (const std::exception &error) {
        std::fprintf(stderr, "[audio] %s\n", error.what());
        return 1;
    }
    return 0;
}
