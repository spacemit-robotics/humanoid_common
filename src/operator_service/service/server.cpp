/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file server.cpp
 * @brief Bounded asynchronous server and internal transport bridge
 */
#include "server.h"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/random.h>
#include <sys/stat.h>
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/beast.hpp>
#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <deque>
#include <filesystem> // NOLINT(build/c++17): this module requires C++17.
#include <fstream>
#include <memory>
#include <stdexcept>
#include <utility>

#include "runtime_logger.h"
#include "transport_executor.h"
namespace operator_service {
namespace net = boost::asio;
namespace beast = boost::beast;
namespace http = beast::http;
namespace ws = beast::websocket;
using Tcp = net::ip::tcp;
namespace {
std::string Secret() {
    unsigned char bytes[24];
    if (getrandom(bytes, sizeof(bytes), 0) != sizeof(bytes)) throw std::runtime_error("random token unavailable");
    std::string result;
    for (auto b : bytes) {
        result += "0123456789abcdef"[b >> 4];
        result += "0123456789abcdef"[b & 15];
    }
    return result;
}

void SecureParent(const std::string &path) {
    const auto parent = std::filesystem::path(path).parent_path();
    if (parent.empty()) throw std::runtime_error("operator file requires a parent directory");
    if (std::filesystem::create_directories(parent) && chmod(parent.c_str(), 0700) != 0)
        throw std::runtime_error("cannot secure operator directory");
    struct stat st {};
    if (lstat(parent.c_str(), &st) != 0 || !S_ISDIR(st.st_mode) || st.st_uid != getuid() || (st.st_mode & 0077))
        throw std::runtime_error("operator directory requires owner mode 0700");
}

void WritePrivate(const std::string &path, const std::string &data, int flags) {
    SecureParent(path);
    const int fd = open(path.c_str(), flags | O_WRONLY | O_CLOEXEC | O_NOFOLLOW, 0600);
    if (fd < 0) throw std::runtime_error("cannot write operator file: " + path);
    struct stat st {};
    const bool ok = fstat(fd, &st) == 0 && S_ISREG(st.st_mode) && st.st_uid == getuid() && st.st_nlink == 1 &&
        fchmod(fd, 0600) == 0 && ftruncate(fd, 0) == 0 &&
        write(fd, data.data(), data.size()) == static_cast<ssize_t>(data.size()) && fsync(fd) == 0;
    close(fd);
    if (!ok) throw std::runtime_error("operator file write failed: " + path);
}

std::string PersistentSecret(const std::string &path) {
    SecureParent(path);
    int fd = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    if (fd < 0 && errno == ENOENT) {
        WritePrivate(path, Secret(), O_CREAT | O_EXCL);
        fd = open(path.c_str(), O_RDONLY | O_CLOEXEC | O_NOFOLLOW);
    }
    if (fd < 0) throw std::runtime_error("cannot read operator credential");
    struct stat st {};
    char bytes[48];
    const bool ok = fstat(fd, &st) == 0 && S_ISREG(st.st_mode) && st.st_uid == getuid() && st.st_nlink == 1 &&
        !(st.st_mode & 0077) && st.st_size == sizeof(bytes) && read(fd, bytes, sizeof(bytes)) == sizeof(bytes);
    close(fd);
    const std::string result = ok ? std::string(bytes, sizeof(bytes)) : "";
    if (result.empty() || result.find_first_not_of("0123456789abcdef") != std::string::npos)
        throw std::runtime_error(
            "operator credential must be an owner-only file containing 48 hex characters");
    return result;
}

class ConnectionFile {
public:
    ConnectionFile(const Config &c, const std::string &endpoint) : path_(c.connection_file) {
        SecureParent(path_);
        lock_ = open((path_ + ".lock").c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (lock_ < 0 || flock(lock_, LOCK_EX | LOCK_NB) != 0) {
            if (lock_ >= 0) close(lock_);
            lock_ = -1;
            throw std::runtime_error("operator instance lock unavailable");
        }
        const int fd = open(path_.c_str(), O_CREAT | O_WRONLY | O_TRUNC | O_CLOEXEC | O_NOFOLLOW, 0600);
        if (fd < 0) {
            close(lock_);
            lock_ = -1;
            throw std::runtime_error("cannot create connection file");
        }
        const auto data = Json{{"v", 1}, {"endpoint", endpoint}, {"token", c.token}}.dump();
        const bool ok =
            fchmod(fd, 0600) == 0 && write(fd, data.data(), data.size()) == static_cast<ssize_t>(data.size());
        close(fd);
        if (!ok) {
            unlink(path_.c_str());
            close(lock_);
            lock_ = -1;
            throw std::runtime_error("connection write failed");
        }
    }
    ~ConnectionFile() {
        if (lock_ >= 0) {
            unlink(path_.c_str());
            close(lock_);
        }
    }

private:
    std::string path_;
    int lock_ = -1;
};
} // namespace

void PreparePairing(Config *config) {
    if (config->terminal_only) throw std::runtime_error("simulation has no web, App or QR entry");
    if (config->public_url.empty()) {
        const auto host = config->bind_address == "0.0.0.0" ? "127.0.0.1" : config->bind_address;
        config->public_url = "http://" + host + ":" + std::to_string(config->port);
    }
    while (!config->public_url.empty() && config->public_url.back() == '/') config->public_url.pop_back();
    const auto &url = config->public_url;
    if (url.rfind("http://", 0) != 0 || url.size() <= 7 || url.size() > 512 || url.find('/', 7) != std::string::npos ||
        url.find_first_of("?#\r\n@ ") != std::string::npos)
        throw std::runtime_error(
            "public_url must be an HTTP origin without credentials, query or fragment");
    if (config->credential_file.empty() || config->qr_file.empty() || config->credential_file == config->qr_file ||
        config->credential_file == config->connection_file || config->qr_file == config->connection_file)
        throw std::runtime_error("credential, QR and connection files must be distinct paths");
    config->token = PersistentSecret(config->credential_file);
    WritePrivate(config->qr_file, PairingSvg(PairingUrl(*config)), O_CREAT);
}

class Server;
class Session : public std::enable_shared_from_this<Session> {
public:
    Session(Tcp::socket socket, Server &server);
    ~Session();
    void Start();
    void Send(const Json &message, bool status = false);
    void Stop();
    uint64_t Id() const { return id_; }
    bool AuthenticationExpired() const { return std::chrono::steady_clock::now() - created_ > std::chrono::seconds(5); }

private:
    void ReadHttp();
    void DownloadApk();
    void ReadWs();
    void Write();
    Server &server_;
    beast::tcp_stream stream_;
    std::unique_ptr<ws::stream<beast::tcp_stream>> socket_;
    beast::flat_buffer buffer_;
    http::request<http::string_body> request_;
    http::request_parser<http::string_body> parser_;
    std::deque<std::string> writes_;
    uint64_t id_ = 0;
    bool stopped_ = false;
    int request_count_ = 0;
    std::chrono::steady_clock::time_point window_ = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point created_ = window_;
};
class Server {
public:
    explicit Server(Config config, const std::string &yaml)
        : service(std::move(config)), acceptor(io,
            Tcp::endpoint(net::ip::make_address(service.Configuration().bind_address), service.Configuration().port)),
        timer(io), signals(io, SIGINT, SIGTERM) {
        transport = transport::CreateV2(yaml);
        if (!transport->Init(yaml, transport::Role::HMI))
            throw std::runtime_error("HMI transport initialization failed");
    }
    void RunLoop() {
        signals.async_wait([this](boost::system::error_code, int) {
            for (const auto &w : sessions)
                if (auto s = w.lock()) s->Stop();
            service.Tick();
            auto neutral = service.Command();
            neutral.key = 0;
            neutral.switch_policy.clear();
            neutral.vx = neutral.vy = neutral.wz = 0;
            transport->SendCommandV2(neutral, 0);
            io.stop();
        });
        Accept();
        Pump();
        io.run();
    }
    void Accept() {
        acceptor.async_accept([this](boost::system::error_code ec, Tcp::socket socket) {
            if (!ec && connections < 64) std::make_shared<Session>(std::move(socket), *this)->Start();
            if (acceptor.is_open()) Accept();
        });
    }
    void Pump() {
        robot_base::ControlStatus status;
        robot_base::FaultStatus fault;
        for (int i = 0; i < 32 && transport->RecvStatusV2(status, fault); ++i) service.UpdateStatus(status, fault);
        service.Tick();
        const bool sent = transport->SendCommandV2(service.Command(), service.AcknowledgeSequence());
        if (!sent && !send_failed)
            runtime_logging::Log(runtime_logging::Level::kError, "operator command transport send failed", false);
        send_failed = !sent;
        if (!sent)
            for (const auto &w : sessions)
                if (auto s = w.lock()) s->Stop();
        const auto now = std::chrono::steady_clock::now();
        const bool publish = now - last_publish >= std::chrono::milliseconds(100);
        if (publish) last_publish = now;
        for (auto it = sessions.begin(); it != sessions.end();) {
            const auto session = it->lock();
            if (!session) {
                it = sessions.erase(it);
                continue;
            }
            if (service.Authenticated(session->Id())) {
                if (publish)
                    session->Send(
                        {{"v", 1}, {"event", "status"}, {"data", Encode(service.Snapshot(session->Id()))}}, true);
            } else if (session->AuthenticationExpired()) session->Stop();
            ++it;
        }
        timer.expires_after(std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(1 / service.Configuration().heartbeat_hz)));
        timer.async_wait([this](boost::system::error_code ec) {
            if (!ec) Pump();
        });
    }
    Service service;
    net::io_context io;
    Tcp::acceptor acceptor;
    net::steady_timer timer;
    net::signal_set signals;
    std::unique_ptr<transport::TransportBaseV2> transport;
    std::vector<std::weak_ptr<Session>> sessions;
    int connections = 0;
    bool send_failed = false;
    std::chrono::steady_clock::time_point last_publish{};
};
Session::Session(Tcp::socket socket, Server &server) : server_(server), stream_(std::move(socket)) {
    ++server_.connections;
}
Session::~Session() {
    server_.service.Disconnect(id_);
    --server_.connections;
}
void Session::Stop() {
    if (stopped_) return;
    stopped_ = true;
    server_.service.Disconnect(id_);
    boost::system::error_code ec;
    auto &tcp = socket_ ? beast::get_lowest_layer(*socket_).socket() : stream_.socket();
    tcp.shutdown(Tcp::socket::shutdown_both, ec);
    tcp.close(ec);
}
void Session::Start() { ReadHttp(); }
void Session::ReadHttp() {
    stream_.expires_after(std::chrono::seconds(5));
    parser_.body_limit(0);
    parser_.header_limit(8192);
    http::async_read(stream_, buffer_, parser_, [self = shared_from_this()](boost::system::error_code ec, size_t) {
        if (ec) {
            self->Stop();
            return;
        }
        self->request_ = self->parser_.release();
        const std::string target(self->request_.target());
        if (!self->server_.service.Configuration().terminal_only &&
            target == "/downloads/SpacemiT-Operator.apk" &&
            (self->request_.method() == http::verb::get || self->request_.method() == http::verb::head)) {
            self->DownloadApk();
            return;
        }
        if (ws::is_upgrade(self->request_) && target == "/api/v1/ws") {
            const std::string origin(self->request_[http::field::origin]);
            const std::string host(self->request_[http::field::host]);
            if ((!origin.empty() && self->server_.service.Configuration().terminal_only) ||
                (!origin.empty() && origin != "http://" + host && origin != "https://" + host)) {
                self->Stop();
                return;
            }
            self->id_ = self->server_.service.Open();
            if (!self->id_) {
                self->Stop();
                return;
            }
            self->stream_.expires_never();
            self->socket_ = std::make_unique<ws::stream<beast::tcp_stream>>(std::move(self->stream_));
            self->socket_->set_option(ws::stream_base::timeout::suggested(beast::role_type::server));
            self->socket_->read_message_max(16384);
            self->socket_->async_accept(self->request_, [self](boost::system::error_code error) {
                if (error) {
                    self->Stop();
                    return;
                }
                self->server_.sessions.push_back(self);
                self->ReadWs();
            });
            return;
        }
        auto response = std::make_shared<http::response<http::string_body>>(http::status::ok, 11);
        response->set(http::field::content_type, "text/html; charset=utf-8");
        response->set(http::field::cache_control, "no-store");
        response->set("X-Content-Type-Options", "nosniff");
        response->set("Referrer-Policy", "no-referrer");
        response->set("Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "connect-src 'self'; img-src 'self' data:; frame-ancestors 'none'");
        std::string filename;
        if (target == "/") filename = "index.html";
        if (target == "/app.js") {
            filename = "app.js";
            response->set(http::field::content_type, "text/javascript");
        }
        if (target == "/style.css") {
            filename = "style.css";
            response->set(http::field::content_type, "text/css");
        }
        if (target == "/vendor/lucide.min.js") {
            filename = "vendor/lucide.min.js";
            response->set(http::field::content_type, "text/javascript");
        }
        if (target == "/assets/spacemit-logo.svg") {
            filename = "assets/spacemit-logo.svg";
            response->set(http::field::content_type, "image/svg+xml");
        }
        if (self->server_.service.Configuration().terminal_only) {
            response->result(http::status::not_found);
            response->body() = "Simulation supports the local terminal only";
        } else if (self->request_.method() == http::verb::get &&
            (target == "/api/v1/local-session" || target == "/pairing.svg")) {
            const auto &config = self->server_.service.Configuration();
            boost::system::error_code peer_error;
            const auto peer = self->stream_.socket().remote_endpoint(peer_error);
            const std::string host(self->request_[http::field::host]);
            const std::string origin(self->request_[http::field::origin]);
            const std::string port = ":" + std::to_string(config.port);
            const bool local_host = host == "127.0.0.1" + port || host == "localhost" + port || host == "[::1]" + port;
            // Pairing grants access. Never expose it to LAN callers or
            // DNS-rebound browser origins.
            if (peer_error || !peer.address().is_loopback() || !local_host ||
                (!origin.empty() && origin != "http://" + host)) {
                response->result(http::status::forbidden);
                response->body() = "Scan the operator QR code to connect";
            } else if (target == "/pairing.svg") {
                response->set(http::field::content_type, "image/svg+xml");
                response->body() = PairingSvg(PairingUrl(config));
            } else {
                response->set(http::field::content_type, "application/json");
                response->body() = Json{{"token", config.token}}.dump();
            }
        } else if (self->request_.method() != http::verb::get || filename.empty()) {
            response->result(http::status::not_found);
            response->body() = "Not found";
        } else {
            std::ifstream file(self->server_.service.Configuration().web_root + "/" + filename);
            if (!file) {
                response->result(http::status::not_found);
                response->body() = "Asset unavailable";
            } else response->body() = std::string(std::istreambuf_iterator<char>(file), {});
        }
        response->keep_alive(false);
        response->prepare_payload();
        http::async_write(
            self->stream_, *response, [self, response](boost::system::error_code, size_t) { self->Stop(); });
    });
}
void Session::DownloadApk() {
    const auto path = server_.service.Configuration().web_root + "/downloads/SpacemiT-Operator.apk";
    auto response = std::make_shared<http::response<http::file_body>>(http::status::ok, 11);
    boost::system::error_code ec;
    response->body().open(path.c_str(), beast::file_mode::scan, ec);
    if (ec) {
        auto missing = std::make_shared<http::response<http::empty_body>>(http::status::not_found, 11);
        missing->content_length(0);
        missing->keep_alive(false);
        http::async_write(stream_, *missing,
            [self = shared_from_this(), missing](boost::system::error_code, size_t) { self->Stop(); });
        return;
    }
    response->set(http::field::content_type, "application/vnd.android.package-archive");
    response->set(http::field::content_disposition, "attachment; filename=\"SpacemiT-Operator.apk\"");
    response->set(http::field::cache_control, "no-store");
    response->set("X-Content-Type-Options", "nosniff");
    response->content_length(response->body().size());
    response->keep_alive(false);
    // Stream the package instead of loading it into the control heartbeat loop.
    auto serializer = std::make_shared<http::response_serializer<http::file_body>>(*response);
    auto complete = [self = shared_from_this(), response, serializer](boost::system::error_code, size_t) {
        self->Stop();
    };
    if (request_.method() == http::verb::head)
        http::async_write_header(stream_, *serializer, std::move(complete));
    else
        http::async_write(stream_, *serializer, std::move(complete));
}
void Session::ReadWs() {
    socket_->async_read(buffer_, [self = shared_from_this()](boost::system::error_code ec, size_t) {
        if (ec) {
            self->Stop();
            return;
        }
        const auto now = std::chrono::steady_clock::now();
        if (now - self->window_ > std::chrono::seconds(1)) {
            self->window_ = now;
            self->request_count_ = 0;
        }
        if (++self->request_count_ > 50) {
            self->Stop();
            return;
        }
        try {
            self->Send(
                self->server_.service.Handle(self->id_, Json::parse(beast::buffers_to_string(self->buffer_.data()))));
        } catch (const Json::exception &) {
            self->Send({{"v", 1}, {"id", 0}, {"ok", false}, {"code", "bad_request"}, {"message", "invalid JSON"},
                {"data", Json::object()}});
        }
        self->buffer_.consume(self->buffer_.size());
        if (!self->stopped_) self->ReadWs();
    });
}
void Session::Send(const Json &message, bool status) {
    if (stopped_ || (status && writes_.size() >= 2)) return;
    if (writes_.size() >= 16) {
        Stop();
        return;
    }
    const bool idle = writes_.empty();
    writes_.push_back(message.dump());
    if (idle) Write();
}
void Session::Write() {
    socket_->text(true);
    socket_->async_write(
        net::buffer(writes_.front()), [self = shared_from_this()](boost::system::error_code ec, size_t) {
            if (ec) {
                self->Stop();
                return;
            }
            self->writes_.pop_front();
            if (!self->writes_.empty()) self->Write();
        });
}
int Run(Config config, const std::string &yaml_path) {
    if (config.terminal_only) {
        config.bind_address = "127.0.0.1";
        config.public_url.clear();
        config.token = Secret();
    } else {
        PreparePairing(&config);
    }
    const std::string host = config.bind_address == "0.0.0.0" ? "127.0.0.1" : config.bind_address;
    ConnectionFile file(config, "ws://" + host + ":" + std::to_string(config.port));
    Server server(config, yaml_path);
    if (config.terminal_only) {
        std::puts("[hmi] simulation: local terminal only; web, App and QR disabled");
    } else {
        std::printf("[hmi] operator service: %s\n[hmi] fixed QR: %s\n[hmi] credential file: %s\n",
            config.public_url.c_str(), config.qr_file.c_str(), config.credential_file.c_str());
    }
    std::printf("[hmi] terminal: hmi_tui --connection %s\n", config.connection_file.c_str());
    runtime_logging::Log(runtime_logging::Level::kInfo, "operator service started", false);
    std::fflush(stdout);
    server.RunLoop();
    return 0;
}
} // namespace operator_service
