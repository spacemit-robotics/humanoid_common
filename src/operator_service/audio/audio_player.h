/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file audio_player.h
 * @brief Internal non-realtime WAV player and action-bound background music
 */
#ifndef AUDIO_PLAYER_H
#define AUDIO_PLAYER_H
#include <fcntl.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <deque>
#include <filesystem> // NOLINT(build/c++17): SDK requires C++17.
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "operator_types.h"

extern char **environ;
namespace operator_audio {
using Clock = std::chrono::steady_clock;
struct Clip {
    std::string file;
    double timeout = 5;
    bool music = false;
    std::string prefix;
};
struct Config {
    bool enabled = false;
    double timeout = 5;
    double fault_repeat_interval_s = 0;
    std::vector<std::string> player;
    std::map<std::string, Clip> clips;
};

inline std::string Group(const std::string &key) { return key.substr(0, key.find('.')); }
class Player {
public:
    Player(Config config, bool dry_run) : config_(std::move(config)), dry_run_(dry_run) {}
    ~Player() { Cancel(); }
    void Synchronize(const operator_service::Status &status) {
        const bool playing = status.online && status.state == "RL" && !status.fault.latched &&
            (status.interaction_phase == "BLEND_IN" || status.interaction_phase == "PLAYING");
        const auto expected = playing ? "action." + status.interaction_action : "";
        const auto stale = [&](const std::string &key) {
            if (key == "fault" && !status.fault.latched) return true;
            const auto clip = config_.clips.find(key);
            return clip != config_.clips.end() && clip->second.music && key != expected;
        };
        pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [&](const Cue &cue) {
            return stale(cue.key);
        }), pending_.end());
        if (stale(active_)) Cancel();
        if (!status.online) Clear();
    }
    void Queue(const std::string &key) {
        if (!config_.clips.count(key)) return;
        const auto group = Group(key);
        const bool urgent = key == "fault" || key == "state.POWER_OFF" || key == "state.SAFETY";
        if (urgent) {
            pending_.clear();
            Cancel();
        } else {
            // Replace stale announcements instead of reciting every intermediate mode.
            pending_.erase(std::remove_if(pending_.begin(), pending_.end(), [&](const Cue &cue) {
                return Group(cue.key) == group || (group == "state" && Group(cue.key) == "action");
            }), pending_.end());
            const auto active_clip = config_.clips.find(active_);
            if (Group(active_) == group || (group == "state" && Group(active_) == "action") ||
                (active_clip != config_.clips.end() && active_clip->second.music &&
                    !config_.clips.at(key).music)) Cancel();
        }
        if (pending_.size() >= 4) pending_.pop_front();
        pending_.push_back({key, Clock::now()});
    }
    void Clear() {
        pending_.clear();
        Cancel();
    }
    bool Busy() const { return pid_ > 0 || !pending_.empty(); }
    void Tick() {
        if (pid_ > 0) {
            int status = 0;
            const auto result = waitpid(pid_, &status, WNOHANG);
            if (result < 0 && errno == EINTR) return;
            if (result != 0) {
                if (result > 0 && (!WIFEXITED(status) || WEXITSTATUS(status) != 0))
                    std::fprintf(stderr, "[audio] playback failed: %s status=%d\n", active_.c_str(), status);
                if (result > 0 && WIFEXITED(status) && WEXITSTATUS(status) == 0 && active_prefix_)
                    pending_.push_front({active_, Clock::now(), true});
                pid_ = -1;
                active_.clear();
            } else if (std::chrono::duration<double>(Clock::now() - started_).count() >
                (active_prefix_ ? config_.timeout : config_.clips.at(active_).timeout)) {
                std::fprintf(stderr, "[audio] playback timed out: %s\n", active_.c_str());
                Cancel();
            }
        }
        if (pid_ > 0) return;
        while (!pending_.empty()) {
            const auto cue = pending_.front();
            pending_.pop_front();
            if (Clock::now() - cue.at > std::chrono::seconds(3)) continue;
            const auto &clip = config_.clips.at(cue.key);
            const bool prefix = !cue.continuation && !clip.prefix.empty();
            const auto &file = prefix ? clip.prefix : clip.file;
            if (!std::filesystem::is_regular_file(file)) {
                std::fprintf(stderr, "[audio] missing WAV: %s\n", file.c_str());
                continue;
            }
            std::printf("[audio] %s -> %s\n", cue.key.c_str(), file.c_str());
            std::fflush(stdout);
            if (dry_run_) {
                if (prefix) pending_.push_front({cue.key, Clock::now(), true});
                continue;
            }
            auto arguments = config_.player;
            arguments.push_back(file);
            std::vector<char *> argv;
            for (auto &argument : arguments) argv.push_back(argument.data());
            argv.push_back(nullptr);
            posix_spawn_file_actions_t actions;
            posix_spawn_file_actions_init(&actions);
            posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0);
            const int result = posix_spawnp(&pid_, argv[0], &actions, nullptr, argv.data(), environ);
            posix_spawn_file_actions_destroy(&actions);
            if (result != 0) {
                pid_ = -1;
                std::fprintf(stderr, "[audio] cannot start player: error=%d\n", result);
                continue;
            }
            active_ = cue.key;
            active_prefix_ = prefix;
            started_ = Clock::now();
            return;
        }
    }

private:
    struct Cue { std::string key; Clock::time_point at; bool continuation = false; };
    void Cancel() {
        if (pid_ > 0) {
            // Only this process's own WAV player; never a robot runtime.
            kill(pid_, SIGKILL);
            while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {}
            pid_ = -1;
        }
        active_.clear();
        active_prefix_ = false;
    }
    Config config_;
    bool dry_run_;
    std::deque<Cue> pending_;
    pid_t pid_ = -1;
    std::string active_;
    bool active_prefix_ = false;
    Clock::time_point started_{};
};

} // namespace operator_audio
#endif // AUDIO_PLAYER_H
