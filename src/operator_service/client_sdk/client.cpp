/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file client.cpp
 * @brief Typed WebSocket client; all network operations use one I/O thread
 */
#include "operator_client.h"

#include <boost/asio.hpp>
#include <boost/beast.hpp>

#include <chrono>
#include <condition_variable>
#include <deque>
#include <fstream>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "protocol.h"

namespace operator_service {
namespace net = boost::asio;
namespace beast = boost::beast;
namespace ws = beast::websocket;
using Tcp = net::ip::tcp;

struct Client::Impl {
    net::io_context io;
    Tcp::resolver resolver{io};
    ws::stream<beast::tcp_stream> socket{io};
    beast::flat_buffer buffer;
    std::thread thread;
    mutable std::mutex mutex;
    std::mutex request_mutex;
    std::condition_variable condition;
    bool connected = false;
    bool finished = false;
    std::string error;
    Connection connection;
    std::string host;
    std::string port;
    std::deque<std::string> writes;
    std::map<uint64_t, Json> replies;
    uint64_t next_id = 0;
    uint64_t waiting_id = 0;
    Status status;
    std::vector<Policy> policies;
    std::vector<Action> actions;
    std::string robot;
    Velocity velocity_step{0.1, 0.1, 0.1};
    std::chrono::steady_clock::time_point last_status{};

    void Fail(const boost::system::error_code &ec, const std::string &detail = "") {
        std::lock_guard<std::mutex> lock(mutex);
        error = detail.empty() ? ec.message() : detail;
        connected = false;
        finished = true;
        status.online = false;
        status.owns_control = false;
        condition.notify_all();
    }
    void Read() {
        socket.async_read(buffer, [this](boost::system::error_code ec, size_t) {
            if (ec) {
                Fail(ec);
                return;
            }
            try {
                const auto message = Json::parse(beast::buffers_to_string(buffer.data()));
                std::lock_guard<std::mutex> lock(mutex);
                if (message.value("v", 0) != 1)
                    throw std::runtime_error("unsupported protocol");
                if (message.value("event", "") == "status") {
                    status = DecodeStatus(message.at("data"));
                    last_status = std::chrono::steady_clock::now();
                } else if (message.contains("id") && message.at("id") == waiting_id) {
                    replies[waiting_id] = message;
                    condition.notify_all();
                }
            } catch (const std::exception &e) {
                Fail(net::error::invalid_argument, e.what());
                return;
            }
            buffer.consume(buffer.size());
            Read();
        });
    }
    void Write() {
        socket.async_write(net::buffer(writes.front()), [this](boost::system::error_code ec, size_t) {
            if (ec) {
                Fail(ec);
                return;
            }
            writes.pop_front();
            if (!writes.empty())
                Write();
        });
    }
    Json Call(const std::string &op, const Json &args = Json::object()) {
        std::lock_guard<std::mutex> serial(request_mutex);
        std::unique_lock<std::mutex> lock(mutex);
        if (!connected)
            return {{"ok", false}, {"code", "disconnected"}, {"message", "service disconnected"}};
        const auto id = ++next_id;
        waiting_id = id;
        const auto message = Json{{"v", 1}, {"id", id}, {"op", op}, {"args", args}}.dump();
        net::post(io, [this, message] {
            const bool idle = writes.empty();
            writes.push_back(message);
            if (idle)
                Write();
        });
        condition.wait_for(
            lock, std::chrono::milliseconds(connection.timeout_ms), [&] { return !connected || replies.count(id); });
        waiting_id = 0;
        const auto it = replies.find(id);
        if (it == replies.end())
            return {{"ok", false}, {"code", "timeout"}, {"message", "result unknown; inspect status before retrying"}};
        Json reply = std::move(it->second);
        replies.erase(it);
        return reply;
    }
    void Start() {
        resolver.async_resolve(host, port, [this](boost::system::error_code ec, Tcp::resolver::results_type endpoints) {
            if (ec) {
                Fail(ec);
                return;
            }
            beast::get_lowest_layer(socket).expires_after(std::chrono::milliseconds(connection.timeout_ms));
            beast::get_lowest_layer(socket).async_connect(
                endpoints, [this](boost::system::error_code error_code, const Tcp::endpoint &) {
                    if (error_code) {
                        Fail(error_code);
                        return;
                    }
                    beast::get_lowest_layer(socket).expires_never();
                    auto options = ws::stream_base::timeout::suggested(beast::role_type::client);
                    options.handshake_timeout = std::chrono::milliseconds(connection.timeout_ms);
                    options.idle_timeout = std::chrono::seconds(5);
                    socket.set_option(options);
                    socket.read_message_max(65536);
                    socket.text(true);
                    socket.async_handshake(host + ":" + port, "/api/v1/ws", [this](boost::system::error_code err) {
                        if (err) {
                            Fail(err);
                            return;
                        }
                        {
                            std::lock_guard<std::mutex> lock(mutex);
                            connected = true;
                            condition.notify_all();
                        }
                        Read();
                    });
                });
        });
        thread = std::thread([this] { io.run(); });
    }
};

Client::Client() : impl_(std::make_unique<Impl>()) {}
Client::~Client() { Disconnect(); }
bool Client::Connect(const Connection &c, std::string *error) {
    Disconnect();
    impl_ = std::make_unique<Impl>();
    impl_->connection = c;
    try {
        if (c.endpoint.rfind("ws://", 0) != 0 || c.timeout_ms < 100 || c.timeout_ms > 10000)
            throw std::runtime_error("expected ws://host:port and bounded timeout");
        const auto authority = c.endpoint.substr(5, c.endpoint.find('/', 5) - 5);
        const auto colon = authority.rfind(':');
        if (colon == std::string::npos || colon == 0)
            throw std::runtime_error("endpoint requires host:port");
        impl_->host = authority.substr(0, colon);
        impl_->port = authority.substr(colon + 1);
        impl_->Start();
        {
            std::unique_lock<std::mutex> lock(impl_->mutex);
            impl_->condition.wait_for(lock, std::chrono::milliseconds(c.timeout_ms * 2),
                [this] { return impl_->connected || impl_->finished; });
            if (!impl_->connected)
                throw std::runtime_error("connect failed: " + impl_->error);
        }
        const auto hello = impl_->Call("hello", {{"token", c.token}, {"name", c.name}});
        if (!hello.at("ok").get<bool>())
            throw std::runtime_error(hello.value("message", "authentication failed"));
        std::lock_guard<std::mutex> lock(impl_->mutex);
        impl_->status = DecodeStatus(hello.at("data").at("status"));
        impl_->last_status = std::chrono::steady_clock::now();
        const auto &catalog = hello.at("data").at("catalog");
        impl_->robot = catalog.at("robot");
        if (catalog.contains("velocity_step")) {
            const auto &step = catalog.at("velocity_step");
            impl_->velocity_step = {step.at("vx").get<double>(), step.at("vy").get<double>(), step.at("wz").get<double>()};
        }
        for (const auto &p : catalog.at("policies"))
            impl_->policies.push_back(DecodePolicy(p));
        for (const auto &a : catalog.value("actions", Json::array()))
            impl_->actions.push_back({a.at("key"), a.at("display_name")});
        return true;
    } catch (const std::exception &e) {
        if (error)
            *error = e.what();
        Disconnect();
        return false;
    }
}
void Client::Disconnect() {
    if (!impl_->thread.joinable())
        return;
    net::post(impl_->io, [this] {
        boost::system::error_code ec;
        beast::get_lowest_layer(impl_->socket).socket().cancel(ec);
        beast::get_lowest_layer(impl_->socket).socket().shutdown(Tcp::socket::shutdown_both, ec);
        beast::get_lowest_layer(impl_->socket).socket().close(ec);
        impl_->io.stop();
    });
    impl_->thread.join();
    std::lock_guard<std::mutex> lock(impl_->mutex);
    impl_->connected = false;
    impl_->status.online = false;
    impl_->status.owns_control = false;
}
bool Client::Connected() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->connected;
}
std::string Client::LastError() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->error;
}
Status Client::LatestStatus() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    auto s = impl_->status;
    if (!impl_->connected || std::chrono::steady_clock::now() - impl_->last_status > std::chrono::seconds(1)) {
        s.online = false;
        s.owns_control = false;
    }
    return s;
}
std::vector<Policy> Client::Policies() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->policies;
}
std::vector<Action> Client::Actions() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->actions;
}
std::string Client::RobotName() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->robot;
}
Velocity Client::VelocityStep() const {
    std::lock_guard<std::mutex> lock(impl_->mutex);
    return impl_->velocity_step;
}
Reply Client::AcquireControl() { return DecodeReply(impl_->Call("acquire")); }
Reply Client::RenewControl() { return DecodeReply(impl_->Call("renew")); }
Reply Client::ReleaseControl() { return DecodeReply(impl_->Call("release")); }
Reply Client::RequestState(const std::string &s) { return DecodeReply(impl_->Call("state", {{"state", s}})); }
Reply Client::SelectPolicy(const std::string &s) { return DecodeReply(impl_->Call("policy", {{"policy", s}})); }
Reply Client::SetVelocity(const Velocity &v, int ttl) {
    auto args = Encode(v);
    args["valid_for_ms"] = ttl;
    return DecodeReply(impl_->Call("velocity", args));
}
Reply Client::Stop() { return DecodeReply(impl_->Call("stop")); }
Reply Client::StartInteraction(const std::string &s) {
    return DecodeReply(impl_->Call("interaction", {{"action", s}}));
}
Reply Client::CancelInteraction() { return DecodeReply(impl_->Call("cancel")); }
Reply Client::StartReference() { return DecodeReply(impl_->Call("reference_start")); }
Reply Client::AcknowledgeFault() { return DecodeReply(impl_->Call("ack")); }
std::string Client::PairingUrl() {
    const auto r = impl_->Call("pairing");
    return r.value("ok", false) ? r.at("data").value("url", "") : "";
}
Connection Client::ReadConnectionFile(const std::string &path) {
    std::ifstream file(path);
    if (!file)
        throw std::runtime_error("connection file unavailable: start hmi_runtime first");
    const auto j = Json::parse(file);
    return {j.at("endpoint"), j.at("token"), "terminal", 1500};
}
} // namespace operator_service
