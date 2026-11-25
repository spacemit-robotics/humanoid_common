/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file whep_proxy.h
 * @brief Local WebRTC signaling. Video packets never pass through the operator event loop.
 */
#ifndef WHEP_PROXY_H
#define WHEP_PROXY_H

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>

namespace operator_service {
class WhepProxy {
public:
    using Callback = std::function<void(bool, std::string)>;
    explicit WhepProxy(boost::asio::io_context &io, uint16_t port) : io_(io), port_(port) {}

    void Close(const std::string &ticket) {
        const auto it = readers_.find(ticket);
        if (it == readers_.end()) return;
        Drop(it->second.path);
        readers_.erase(it);
    }

    template<typename Allowed>
    void Sweep(Allowed allowed) {
        for (auto it = readers_.begin(); it != readers_.end();) {
            if (allowed(it->first)) { ++it; continue; }
            Drop(it->second.path);
            it = readers_.erase(it);
        }
    }

    void Open(const std::string &ticket, std::string offer, Callback reply) {
        if (!port_ || pending_ >= 3 || (!readers_.count(ticket) && readers_.size() >= 3))
            return reply(false, {});
        auto &reader = readers_[ticket];
        Drop(reader.path);
        const auto generation = ++generation_;
        reader = {generation, {}};
        ++pending_;
        Request(boost::beast::http::verb::post, "/preview/whep", std::move(offer),
            [this, ticket, generation, reply = std::move(reply)](auto &response, bool ok) {
                --pending_;
                auto it = readers_.find(ticket);
                const std::string path(response[boost::beast::http::field::location]);
                const bool created = ok && response.result_int() == 201 && ValidPath(path);
                if (it == readers_.end() || it->second.generation != generation) {
                    if (created) Drop(path);
                    return reply(false, {});
                }
                if (!created) {
                    readers_.erase(it);
                    return reply(false, {});
                }
                it->second.path = path;
                reply(true, std::move(response.body()));
            });
    }

private:
    struct Reader { uint64_t generation; std::string path; };
    using HttpResponse = boost::beast::http::response<boost::beast::http::string_body>;
    using Done = std::function<void(HttpResponse &, bool)>;

    static bool ValidPath(const std::string &path) {
        constexpr char prefix[] = "/preview/whep/";
        return path.rfind(prefix, 0) == 0 && path.size() == sizeof(prefix) - 1 + 36 &&
            path.find_first_not_of("0123456789abcdefABCDEF-", sizeof(prefix) - 1) == std::string::npos;
    }
    void Drop(const std::string &path) {
        if (!path.empty()) Request(boost::beast::http::verb::delete_, path, {}, [](auto &, bool) {});
    }
    void Request(boost::beast::http::verb method, const std::string &path, std::string body, Done done) {
        namespace http = boost::beast::http;
        using Tcp = boost::asio::ip::tcp;
        struct Exchange {
            explicit Exchange(boost::asio::io_context &io) : upstream(io) { response.body_limit(64 * 1024); }
            boost::beast::tcp_stream upstream;
            boost::beast::flat_buffer buffer;
            http::request<http::string_body> request;
            http::response_parser<http::string_body> response;
        };
        auto x = std::make_shared<Exchange>(io_);
        x->request = {method, path, 11};
        x->request.set(http::field::host, "127.0.0.1:" + std::to_string(port_));
        if (method == http::verb::post) x->request.set(http::field::content_type, "application/sdp");
        x->request.body() = std::move(body);
        x->request.prepare_payload();
        const auto finish = [x, done = std::move(done)](auto ec) { done(x->response.get(), !ec); };
        x->upstream.expires_after(std::chrono::seconds(15));
        x->upstream.async_connect(Tcp::endpoint(boost::asio::ip::address_v4::loopback(), port_), [x, finish](auto ec) {
            if (ec) return finish(ec);
            http::async_write(x->upstream, x->request, [x, finish](auto e, size_t) {
                if (e) return finish(e);
                http::async_read(x->upstream, x->buffer, x->response, [finish](auto err, size_t) { finish(err); });
            });
        });
    }
    boost::asio::io_context &io_;
    uint16_t port_;
    uint64_t generation_ = 0;
    unsigned pending_ = 0;
    std::map<std::string, Reader> readers_;
};
}  // namespace operator_service
#endif  // WHEP_PROXY_H
