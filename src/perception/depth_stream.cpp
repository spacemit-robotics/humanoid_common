// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "depth_stream.h"
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <zmq.h>
namespace perception {
namespace {
int Reflect(int i, int n) { return i < 0 ? -i : (i >= n ? 2 * n - 2 - i : i); }
} // namespace
std::vector<float> FillDepthHoles(const std::vector<float> &raw) {
    if (raw.size() != kPixels)
        throw std::runtime_error("depth size must be 64x36");
    auto filled = raw;
    for (int y = 0; y < kHeight; ++y)
        for (int x = 0; x < kWidth; ++x) {
            const int index = y * kWidth + x;
            if (std::isfinite(raw[index]) && raw[index] > 0) continue;
            float value = 2.5f;
            int best = 5;
            // A radius of two pixels fills small 3x3 holes without flooding
            // large missing regions. Ties use ascending row then column.
            for (int dy = -2; dy <= 2; ++dy)
                for (int dx = -2; dx <= 2; ++dx) {
                    const int distance = dx * dx + dy * dy;
                    const int yy = y + dy, xx = x + dx;
                    if (distance >= best || yy < 0 || yy >= kHeight || xx < 0 || xx >= kWidth) continue;
                    const float donor = raw[yy * kWidth + xx];
                    if (std::isfinite(donor) && donor > 0) {
                        best = distance;
                        value = donor;
                    }
                }
            filled[index] = value;
        }
    return filled;
}
std::vector<float> Preprocess(const std::vector<float> &raw) {
    const auto filled = FillDepthHoles(raw);
    std::vector<float> crop(kFrame), result(kFrame);
    for (int y = 0; y < 18; ++y)
        for (int x = 0; x < 32; ++x) {
            crop[y * 32 + x] = filled[(y + 18) * 64 + x + 16];
        }
    const float e = std::exp(-.5f), sum = 1 + 2 * e;
    const float kernel[3] = {e / sum, 1 / sum, e / sum};
    for (int y = 0; y < 18; ++y)
        for (int x = 0; x < 32; ++x) {
            float d = 0;
            for (int j = -1; j <= 1; ++j)
                for (int i = -1; i <= 1; ++i)
                    d += kernel[j + 1] * kernel[i + 1] *
                        crop[Reflect(y + j, 18) * 32 + Reflect(x + i, 32)];
            result[y * 32 + x] = std::clamp(d, 0.f, 2.5f) / 2.5f;
        }
    return result;
}
std::vector<unsigned char> Encode(const Frame &f) {
    if (f.pixels.size() != kPixels || !std::isfinite(f.time) || f.session.empty())
        throw std::runtime_error("invalid depth frame");
    const auto header = nlohmann::json{
        {"version", 1},
        {"profile", kProfile},
        {"width", 64},
        {"height", 36},
        {"dtype", "float32le"},
        {"unit", "m"},
        {"distance", "optical_z"},
        {"frame", "torso_link/rgbd_camera"},
        {"fovy_deg", 58.29},
        {"camera_pos", {.0576235, .01753, .42987}},
        {"camera_xyaxes", {0., -1., 0., .738455340625884, 0., .674302387583723}},
        {"clock", f.clock},
        {"time", f.time},
        {"sequence", f.sequence},
        {"session",
            f.session}}.dump();
    const uint32_t n = header.size();
    std::vector<unsigned char> out(4 + n + kPixels * 4);
    for (int i = 0; i < 4; ++i)
        out[i] = (n >> (8 * i)) & 255;
    std::memcpy(out.data() + 4, header.data(), n);
    std::memcpy(out.data() + 4 + n, f.pixels.data(), kPixels * 4);
    return out;
}
Frame Decode(const void *data, size_t size) {
    if (size < 4)
        throw std::runtime_error("short depth packet");
    const auto *p = static_cast<const unsigned char *>(data);
    uint32_t n = 0;
    for (int i = 0; i < 4; ++i)
        n |= uint32_t(p[i]) << (8 * i);
    if (n > 4096 || size != 4 + n + kPixels * 4)
        throw std::runtime_error("depth packet length mismatch");
    auto h = nlohmann::json::parse(p + 4, p + 4 + n);
    if (h.at("version") != 1 || h.at("profile") != kProfile || h.at("width") != 64 ||
        h.at("height") != 36 || h.at("dtype") != "float32le" || h.at("unit") != "m" ||
        h.at("distance") != "optical_z" || h.at("frame") != "torso_link/rgbd_camera")
        throw std::runtime_error("depth profile mismatch");
    // Intrinsics and extrinsics are part of the wire contract, not trusted labels.
    const double pos[3] = {.0576235, .01753, .42987};
    const double axes[6] = {0, -1, 0, .738455340625884, 0, .674302387583723};
    if (!h.at("fovy_deg").is_number() || std::abs(h.at("fovy_deg").get<double>() - 58.29) > 1e-6)
        throw std::runtime_error("depth FOV mismatch");
    if (h.at("camera_pos").size() != 3 || h.at("camera_xyaxes").size() != 6)
        throw std::runtime_error("depth pose size");
    for (int i = 0; i < 3; ++i)
        if (std::abs(h["camera_pos"][i].get<double>() - pos[i]) > 1e-8)
            throw std::runtime_error("depth pose mismatch");
    for (int i = 0; i < 6; ++i)
        if (std::abs(h["camera_xyaxes"][i].get<double>() - axes[i]) > 1e-8)
            throw std::runtime_error("depth rotation mismatch");
    Frame f;
    if (!h.at("sequence").is_number_unsigned())
        throw std::runtime_error("depth sequence must be an unsigned integer");
    f.time = h.at("time");
    f.sequence = h.at("sequence");
    f.session = h.at("session");
    f.clock = h.at("clock");
    if (!std::isfinite(f.time) || f.time < 0 || f.session.empty() ||
        (f.clock != "sim" && f.clock != "robot"))
        throw std::runtime_error("depth clock/session mismatch");
    f.pixels.resize(kPixels);
    std::memcpy(f.pixels.data(), p + 4 + n, kPixels * 4);
    return f;
}
Receiver::Receiver(const std::string &endpoint, double timeout) : timeout_(timeout) {
    if (!std::isfinite(timeout) || timeout <= 0 || timeout > 1)
        throw std::runtime_error("depth timeout invalid");
    thread_ = std::thread([this, endpoint] { Loop(endpoint); });
}
Receiver::~Receiver() {
    stop_ = true;
    if (thread_.joinable())
        thread_.join();
}
void Receiver::Loop(std::string endpoint) {
    void *ctx = zmq_ctx_new();
    void *socket = ctx ? zmq_socket(ctx, ZMQ_SUB) : nullptr;
    if (!socket) {
        if (ctx)
            zmq_ctx_term(ctx);
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = "depth ZMQ initialization failed";
        return;
    }
    const int wait = 25, linger = 0, hwm = 16, max = 16384;
    const int64_t max_size = max;
    zmq_setsockopt(socket, ZMQ_RCVTIMEO, &wait, sizeof(wait));
    zmq_setsockopt(socket, ZMQ_LINGER, &linger, sizeof(linger));
    zmq_setsockopt(socket, ZMQ_RCVHWM, &hwm, sizeof(hwm));
    zmq_setsockopt(socket, ZMQ_MAXMSGSIZE, &max_size, sizeof(max_size));
    zmq_setsockopt(socket, ZMQ_SUBSCRIBE, "", 0);
    if (zmq_connect(socket, endpoint.c_str()) != 0) {
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = "depth connect failed";
    } else
        while (!stop_) {
            unsigned char packet[16384];
            int size = zmq_recv(socket, packet, sizeof(packet), 0);
            if (size < 0)
                continue;
            try {
                if (size > int(sizeof(packet)))
                    throw std::runtime_error("oversized depth packet");
                Frame f = Decode(packet, size);
                auto processed = Preprocess(f.pixels);
                f.pixels.clear();
                std::lock_guard<std::mutex> lock(mutex_);
                if (!frames_.empty()) {
                    auto &last = frames_.back().frame;
                    if (f.session != last.session || f.clock != last.clock ||
                        f.time < last.time - .1)
                        frames_.clear();
                    else if (f.sequence <= last.sequence || f.time <= last.time)
                        throw std::runtime_error("out-of-order depth");
                }
                frames_.push_back({std::move(f), std::move(processed)});
                while (frames_.size() > 128)
                    frames_.pop_front();
                received_ = std::chrono::steady_clock::now();
                error_.clear();
            } catch (const std::exception &e) {
                std::lock_guard<std::mutex> lock(mutex_);
                ++rejected_;
                error_ = e.what();
            }
        }
    zmq_close(socket);
    zmq_ctx_term(ctx);
}
std::string Receiver::Error(double time, const std::string &clock) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ErrorLocked(time, clock);
}
std::string Receiver::ErrorLocked(double time, const std::string &clock) const {
    if (frames_.empty())
        return error_.empty() ? "depth history empty" : error_;
    if (!error_.empty())
        return error_;
    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count() >
        timeout_)
        return "depth stream stale";
    if (frames_.back().frame.clock != clock)
        return "depth/state clock domains differ";
    if (!std::isfinite(time) || time - frames_.back().frame.time > timeout_ ||
        frames_.back().frame.time - time > .1)
        return "depth/state timestamps differ";
    if (frames_.front().frame.time > time - .7 + 1e-8)
        return "depth warming up (need 0.7 s history)";
    for (int i = 0; i < 8; ++i) {
        const double target = time - (7 - i) * .1;
        const Sample *chosen = nullptr;
        for (const auto &sample : frames_)
            if (sample.frame.time <= target + 1e-8)
                chosen = &sample;
            else
                break;
        if (!chosen || target - chosen->frame.time > timeout_)
            return "depth history gap";
    }
    return {};
}
std::vector<float> Receiver::Stack(double time, const std::string &clock) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto error = ErrorLocked(time, clock);
    if (!error.empty())
        throw std::runtime_error(error);
    std::vector<float> out(kStack);
    for (int i = 0; i < 8; ++i) {
        const double target = time - (7 - i) * .1;
        const Sample *chosen = nullptr;
        for (const auto &s : frames_)
            if (s.frame.time <= target + 1e-8)
                chosen = &s;
            else
                break;
        if (!chosen || target - chosen->frame.time > timeout_)
            throw std::runtime_error("depth history gap");
        std::copy(chosen->image.begin(), chosen->image.end(), out.begin() + i * kFrame);
    }
    return out;
}
nlohmann::json Receiver::Status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {
        {"frames", frames_.size()},
        {"rejected", rejected_},
        {"error", error_},
        {"age_s",
            frames_.empty()
                ? -1
                : std::chrono::duration<double>(std::chrono::steady_clock::now() - received_).count()},
        {"fps", frames_.size() < 2 ? 0
                                    : (frames_.size() - 1) /
                                        (frames_.back().frame.time - frames_.front().frame.time)},
        {"capture_time", frames_.empty() ? 0 : frames_.back().frame.time}};
}
} // namespace perception
