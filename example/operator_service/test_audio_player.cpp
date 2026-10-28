/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_audio_player.cpp
 * @brief Test action music lifetime and prompt priority without an audio device
 */
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <thread>

#include "audio_player.h"

int main() {
    char directory[] = "/tmp/operator_audio_player_XXXXXX";
    assert(mkdtemp(directory));
    const std::string file = std::string(directory) + "/clip.wav";
    FILE *stream = std::fopen(file.c_str(), "wb");
    assert(stream);
    std::fclose(stream);
    operator_audio::Config config;
    config.player = {"/bin/sh", "-c", "exec sleep 30", "fake-audio-player"};
    config.clips = {{"action.rhythm", {file, 0.3, true}}, {"fault", {file, 0.1, false}},
        {"state.DAMP", {file, 0.1, false}}, {"policy.dance", {file, 0.1, false}}};
    operator_service::Status status;
    status.online = true;
    status.state = "RL";
    status.policy = "dance";
    status.interaction_action = "rhythm";
    status.interaction_phase = "PLAYING";
    operator_audio::Player player(config, false);
    const auto start_music = [&] {
        player.Queue("action.rhythm");
        player.Synchronize(status);
        player.Tick();
        assert(player.Busy());
    };
    start_music();
    std::this_thread::sleep_for(std::chrono::milliseconds(130));
    player.Tick();
    assert(player.Busy());  // Music has its own duration, not the short prompt deadline.
    status.interaction_phase = "BLEND_OUT";
    player.Synchronize(status);
    assert(!player.Busy());
    status.interaction_phase = "PLAYING";
    start_music();
    status.fault.latched = true;
    player.Synchronize(status);
    assert(!player.Busy());
    status.fault.latched = false;
    start_music();
    status.state = "POWER_OFF";
    player.Synchronize(status);
    assert(!player.Busy());
    status.state = "RL";
    start_music();
    status.online = false;
    player.Synchronize(status);
    assert(!player.Busy());
    status.online = true;
    player.Queue("policy.dance");
    player.Tick();
    player.Queue("action.rhythm");  // Queued music is cancelled as well.
    status.interaction_phase = "IDLE";
    player.Synchronize(status);
    std::this_thread::sleep_for(std::chrono::milliseconds(140));
    player.Tick();
    assert(!player.Busy());
    status.interaction_phase = "PLAYING";
    start_music();
    player.Queue("fault");
    player.Tick();
    std::this_thread::sleep_for(std::chrono::milliseconds(140));
    player.Tick();
    assert(!player.Busy());
    status.fault.latched = true;
    player.Queue("fault");
    player.Synchronize(status);
    player.Tick();
    assert(player.Busy());
    status.fault.latched = false;
    player.Synchronize(status);
    assert(!player.Busy());  // Confirmation cancels an in-progress reminder.
    player.Queue("fault");
    player.Synchronize(status);
    assert(!player.Busy());  // It also removes a queued reminder before playback.
    const std::string prefix = std::string(directory) + "/prefix.wav";
    const std::string log = std::string(directory) + "/played.txt";
    std::ofstream(prefix).close();
    config.clips["action.rhythm"].prefix = prefix;
    config.player = {"/bin/sh", "-c", "printf '%s\\n' \"$1\" >> \"$0\"", log};
    const auto finish = [](operator_audio::Player *p) {
        for (int i = 0; i < 100 && p->Busy(); ++i) {
            p->Tick();
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        assert(!p->Busy());
    };
    operator_audio::Player sequence(config, false);
    sequence.Queue("action.rhythm");
    finish(&sequence);
    std::ifstream played(log);
    std::string line;
    assert(std::getline(played, line) && line == prefix);
    assert(std::getline(played, line) && line == file);
    assert(!std::getline(played, line));
    played.close();
    config.player = {"/bin/sh", "-c", "exit 1", "failed-prefix"};
    operator_audio::Player failed(config, false);
    failed.Queue("action.rhythm");
    finish(&failed);  // No continuation after an unsuccessful prefix.
    config.player = {"/bin/sh", "-c", "exec sleep 30", "slow-prefix"};
    operator_audio::Player interrupted(config, false);
    interrupted.Queue("action.rhythm");
    interrupted.Tick();
    assert(interrupted.Busy());
    status.interaction_phase = "BLEND_OUT";
    interrupted.Synchronize(status);
    interrupted.Tick();
    assert(!interrupted.Busy());  // Cancelling during the prefix must not start music.
    std::remove(prefix.c_str());
    std::remove(log.c_str());
    std::remove(file.c_str());
    rmdir(directory);
    std::puts("audio player: music duration, cancel, fault, power-off and offline passed");
}
