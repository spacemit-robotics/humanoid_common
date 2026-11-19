// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "clock_anchor_server.h"

#include <zmq.h>
#include <cassert>
#include <chrono>
#include <cmath>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

double Mono() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

int main() {
    perception::ClockAnchorServer server("tcp://127.0.0.1:*", .01);
    void *ctx = zmq_ctx_new();
    void *client = zmq_socket(ctx, ZMQ_REQ);
    int timeout = 1000, linger = 0;
    zmq_setsockopt(client, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(client, ZMQ_RCVTIMEO, &timeout, sizeof(timeout));
    assert(zmq_connect(client, server.Endpoint().c_str()) == 0);
    auto query = [&](const char *data) {
        assert(zmq_send(client, data, std::char_traits<char>::length(data), 0) >= 0);
        char response[4096];
        int n = zmq_recv(client, response, sizeof(response), 0);
        assert(n > 0);
        return nlohmann::json::parse(response, response + n);
    };
    assert(!query(R"({"version":1,"op":"clock"})")["valid"].get<bool>());
    const double receive = Mono() - .003;
    server.Observe(100, receive, true);
    auto first = query(R"({"version":1,"op":"clock"})");
    assert(first["valid"].get<bool>());
    assert(std::abs(first["anchor_receive_mono_s"].get<double>() - receive) < 1e-9);
    assert(first["anchor_robot_s"] == 100);
    assert(first["t3_mono_s"].get<double>() >= first["t2_mono_s"].get<double>());
    assert(!query(R"({"version":"bad","op":3})")["valid"].get<bool>());
    assert(query(R"({"version":1,"op":"clock"})")["valid"].get<bool>());
    server.Invalidate();
    assert(!query(R"({"version":1,"op":"clock"})")["valid"].get<bool>());
    server.Observe(1, Mono(), true);
    auto reset = query(R"({"version":1,"op":"clock"})");
    assert(reset["session"] != first["session"]);
    std::this_thread::sleep_for(std::chrono::milliseconds(220));
    assert(!query(R"({"version":1,"op":"clock"})")["valid"].get<bool>());
    zmq_close(client);
    zmq_ctx_term(ctx);
    std::cout << "Clock anchor RPC, invalid requests, stale anchors and clock reset passed\n";
}
