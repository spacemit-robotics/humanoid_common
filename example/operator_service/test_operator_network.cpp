/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_operator_network.cpp
 * @brief Real WebSocket SDK round-trip against an isolated synthetic control
 * endpoint
 */
#include <sys/prctl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cassert>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <filesystem> // NOLINT(build/c++17): the operator service uses C++17.
#include <fstream>
#include <functional>
#include <string>
#include <thread>

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <yaml-cpp/yaml.h>

#include "operator_client.h"
#include "service.h"
#include "transport_executor.h"

namespace {
bool Wait(const std::function<bool()> &predicate) {
    for (int i = 0; i < 60; ++i) {
        if (predicate()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    return false;
}
boost::beast::http::response<boost::beast::http::string_body> Get(
    unsigned short port, const std::string &path, const std::string &host, const std::string &origin = "",
    boost::beast::http::verb method = boost::beast::http::verb::get) {
    namespace http = boost::beast::http;
    boost::asio::io_context io;
    boost::beast::tcp_stream stream(io);
    stream.expires_after(std::chrono::seconds(2));
    stream.connect({boost::asio::ip::make_address("127.0.0.1"), port});
    http::request<http::empty_body> request(method, path, 11);
    request.set(http::field::host, host);
    if (!origin.empty()) request.set(http::field::origin, origin);
    http::write(stream, request);
    boost::beast::flat_buffer buffer;
    http::response_parser<http::string_body> parser;
    parser.skip(method == http::verb::head);
    http::read(stream, buffer, parser);
    return parser.release();
}
} // namespace

int main(int argc, char *argv[]) {
    assert(argc >= 3);
    bool terminal_only = false;
    bool trajectory = false;
    for (int i = 3; i < argc; ++i) {
        terminal_only = terminal_only || std::string(argv[i]) == "--sim";
        trajectory = trajectory || std::string(argv[i]) == "--trajectory";
    }
    const auto active_state = trajectory ? "TRAJECTORY" : "RL";
    assert(setenv("XDG_RUNTIME_DIR", "/tmp", 1) == 0);
    assert(setenv("XDG_STATE_HOME", "/tmp/operator-network-state", 1) == 0);
    char directory[] = "/tmp/operator-download-XXXXXX";
    assert(mkdtemp(directory));
    const std::filesystem::path root(directory);
    std::filesystem::create_directory(root / "downloads");
    const auto apk_path = root / "downloads/SpacemiT-Operator.apk";
    const std::string apk_data = "PK" + std::string(65536, 'x');
    std::ofstream(apk_path, std::ios::binary) << apk_data;
    auto yaml = YAML::LoadFile(argv[2]);
    if (trajectory) {
        yaml.remove("rl_policy");
        auto config = yaml["behavior_manager"]["trajectory"];
        config["enabled"] = true;
        config["catalog_file"] = "actions.yaml";
        config["catalog"] = "standalone_actions";
        std::ofstream(root / "actions.yaml") << "standalone_actions:\n"
            "  action_names: [wave]\n  actions:\n    wave: {display_name: Wave}\n";
    }
    yaml["operator_service"]["web_root"] = root.string();
    const auto config_path = (root / "config.yaml").string();
    std::ofstream(config_path) << yaml;
    auto control = transport::CreateV2(config_path);
    assert(control->Init(config_path, transport::Role::CONTROL));
    std::atomic<bool> running{true};
    std::thread producer([&] {
        robot_base::ControlStatus status;
        status.active_policy = trajectory ? "" : "test_policy";
        status.zero_ready = true;
        status.rl_frequency_hz = trajectory ? 0 : 50;
        status.hmi_connected = true;
        while (running) {
            robot_base::Command command;
            uint64_t acknowledge = 0;
            while (control->RecvCommandV2(command, acknowledge)) {
                if (command.key == -1) status.mode = robot_base::ControlMode::POWER_OFF;
                if (command.key == 1) status.mode = robot_base::ControlMode::DAMP;
                if (command.key == 4) status.mode = robot_base::ControlMode::HOME;
                if (command.key == 2) status.mode = robot_base::ControlMode::ZERO;
                if (command.key == 3) status.mode = trajectory
                    ? robot_base::ControlMode::TRAJECTORY : robot_base::ControlMode::RL;
                status.vx = command.vx;
                status.vy = command.vy;
                status.wz = command.wz;
                if (command.interaction.operation != robot_base::InteractionRequest::Operation::NONE) {
                    status.interaction.sequence = command.interaction.sequence;
                    status.interaction.request_accepted = true;
                    status.interaction.phase =
                        command.interaction.operation == robot_base::InteractionRequest::Operation::START
                        ? robot_base::InteractionStatus::Phase::PLAYING
                        : robot_base::InteractionStatus::Phase::IDLE;
                    status.interaction.action = command.interaction.action;
                }
            }
            control->SendStatusV2(status, {});
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    });
    const pid_t child = fork();
    assert(child >= 0);
    if (child == 0) {
        prctl(PR_SET_PDEATHSIG, SIGTERM);
        execl(argv[1], argv[1], config_path.c_str(), terminal_only ? "--sim" : "--real", nullptr);
        _exit(127);
    }
    const auto file = operator_service::LoadConfig(config_path).connection_file;
    const bool exists = Wait([&] { return access(file.c_str(), R_OK) == 0; });
    if (!exists) {
        running = false;
        producer.join();
        kill(child, SIGTERM);
        waitpid(child, nullptr, 0);
        return 1;
    }
    {
        operator_service::Client alice, bob;
        auto connection = operator_service::Client::ReadConnectionFile(file);
        std::string error;
        assert(Wait([&] {
            try {
                connection = operator_service::Client::ReadConnectionFile(file);
                return alice.Connect(connection, &error);
            } catch (const std::exception &) {
                return false;
            }
        }));
        connection.name = "observer";
        assert(bob.Connect(connection, &error));
        assert(Wait([&] { return alice.LatestStatus().online; }));
        assert(alice.LatestStatus().trajectory_enabled == trajectory);
        if (trajectory) {
            assert(alice.Policies().empty() && alice.LatestStatus().policy.empty());
            assert(alice.Actions().size() == 1 && alice.Actions()[0].display_name == "Wave");
        } else {
            assert(alice.Policies().size() == 1 && alice.Policies()[0].actions.size() == 1);
            assert(alice.Actions().empty());
        }
        assert(alice.AcquireControl().ok);
        assert(!bob.AcquireControl().ok);
        for (const auto *target : {"DAMP", "HOME", "ZERO", active_state}) {
            assert(alice.RenewControl().ok);
            assert(alice.RequestState(target).ok);
            assert(Wait([&] { return alice.LatestStatus().state == target; }));
        }
        if (trajectory) {
            assert(!alice.SetVelocity({1, 0, 0}, 300).ok);
        } else {
            assert(alice.SetVelocity({1, 0, 0}, 300).ok);
            assert(Wait([&] { return alice.LatestStatus().velocity.vx > 0.09; }));
            assert(Wait([&] { return alice.LatestStatus().velocity.vx == 0; }));
        }
        auto pairing = operator_service::LoadConfig(config_path);
        const auto host = "127.0.0.1:" + std::to_string(pairing.port);
        const std::string apk_url = "/downloads/SpacemiT-Operator.apk";
        if (terminal_only) {
            assert(alice.PairingUrl().empty());
            for (const auto *path : {"/", "/app.js", "/api/v1/local-session", "/pairing.svg"})
                assert(Get(pairing.port, path, host).result_int() == 404);
            assert(Get(pairing.port, apk_url, host).result_int() == 404);
        } else {
            namespace http = boost::beast::http;
            const auto header = Get(pairing.port, apk_url, host, "", http::verb::head);
            assert(header.result_int() == 200 && header.body().empty());
            assert(header[http::field::content_length] == std::to_string(apk_data.size()));
            const auto download = Get(pairing.port, apk_url, host);
            assert(download.result_int() == 200 && download.body() == apk_data);
            assert(download[http::field::content_type] == "application/vnd.android.package-archive");
            assert(Get(pairing.port, apk_url, host, "", http::verb::post).result_int() == 404);
            assert(Get(pairing.port, "/downloads/../config.yaml", host).result_int() == 404);
            std::filesystem::remove(apk_path);
            assert(Get(pairing.port, apk_url, host).result_int() == 404);
            assert(Get(pairing.port, apk_url, host, "", http::verb::head).result_int() == 404);
            assert(alice.PairingUrl().find("/#token=" + connection.token) != std::string::npos);
            const auto bootstrap = Get(pairing.port, "/api/v1/local-session", host);
            assert(bootstrap.result_int() == 200 && bootstrap.body().find(connection.token) != std::string::npos);
            for (const auto *path : {"/api/v1/local-session", "/pairing.svg"}) {
                assert(Get(pairing.port, path, host).result_int() == 200);
                assert(Get(pairing.port, path, "untrusted.example:" + std::to_string(pairing.port)).result_int() == 403);
                assert(Get(pairing.port, path, host, "http://untrusted.example").result_int() == 403);
            }
            operator_service::PreparePairing(&pairing);
            const auto saved_token = pairing.token;
            const auto saved_qr = operator_service::PairingSvg(operator_service::PairingUrl(pairing));
            operator_service::PreparePairing(&pairing);
            assert(pairing.token == connection.token && pairing.token == saved_token);
            assert(operator_service::PairingSvg(operator_service::PairingUrl(pairing)) == saved_qr);
        }
        alice.Disconnect();
        assert(Wait([&] { return bob.LatestStatus().owner.empty(); }));
        assert(bob.LatestStatus().state == active_state && bob.LatestStatus().velocity.vx == 0);
        assert(bob.AcquireControl().ok);
        assert(bob.StartInteraction("wave").ok);
        const bool playing = Wait([&] { return bob.LatestStatus().interaction_phase == "PLAYING"; });
        if (!playing) {
            const auto s = bob.LatestStatus();
            std::fprintf(stderr, "action status: state=%s owner=%s phase=%s request=%s %s\n", s.state.c_str(),
                s.owner.c_str(), s.interaction_phase.c_str(), s.request.phase.c_str(), s.request.message.c_str());
            std::fprintf(stderr, "client error: %s\n", bob.LastError().c_str());
        }
        assert(playing);
        assert(bob.CancelInteraction().ok);
        assert(Wait([&] { return bob.LatestStatus().interaction_phase == "IDLE"; }));
        assert(bob.Stop().ok);
        assert(Wait([&] { return !bob.LatestStatus().owns_control; }));
        bob.Disconnect();
        connection.token = "wrong";
        assert(!alice.Connect(connection, &error));
        std::puts("operator network tests passed: native client, FSM request, "
            "limits, expiry, disconnect, action, authentication, access mode");
    }
    running = false;
    producer.join();
    kill(child, SIGTERM);
    waitpid(child, nullptr, 0);
    std::filesystem::remove_all(root);
    return 0;
}
