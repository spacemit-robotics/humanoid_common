/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_gamepad_feedback.cpp
 * @brief Verify success-only vibration and motor-only DS4 output without devices
 */
#include "gamepad_feedback.h"
#include <cassert>
#include <cstdio>

namespace {
struct Probe {
    std::mutex mutex;
    std::condition_variable wake;
    std::vector<std::string> starts;
    bool release = false;
    int completed = 0;
};

void WaitFor(const std::shared_ptr<Probe> &probe, const std::function<bool()> &ready) {
    std::unique_lock<std::mutex> lock(probe->mutex);
    assert(probe->wake.wait_for(lock, std::chrono::seconds(2), ready));
}

void TestBlockedOutput() {
    auto probe = std::make_shared<Probe>();
    operator_gamepad::Rumble rumble([probe](const auto &session) {
        {
            std::unique_lock<std::mutex> lock(probe->mutex);
            probe->starts.push_back(session->config.device);
            probe->wake.notify_all();
            if (session->config.device == "blocked")
                probe->wake.wait(lock, [&] { return probe->release; });
        }
        {
            std::unique_lock<std::mutex> lock(session->mutex);
            session->wake.wait(lock, [&] { return session->stop; });
            assert(!session->pulse);
        }
        std::lock_guard<std::mutex> lock(probe->mutex);
        ++probe->completed;
        probe->wake.notify_all();
    });
    operator_gamepad::FeedbackConfig config;
    config.device = "blocked";
    rumble.Start(config, "", 0, 0);
    WaitFor(probe, [&] { return probe->starts.size() == 1; });
    const auto start = std::chrono::steady_clock::now();
    rumble.Pulse();
    rumble.Stop();
    for (int i = 0; i < 32; ++i) {
        config.device = "replaced";
        rumble.Start(config, "", 0, 0);
    }
    config.device = "latest";
    rumble.Start(config, "", 0, 0);
    assert(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500));
    {
        std::lock_guard<std::mutex> lock(probe->mutex);
        assert(probe->starts.size() == 1);  // One worker, even with a blocked USB write.
        probe->release = true;
        probe->wake.notify_all();
    }
    WaitFor(probe, [&] { return probe->starts.size() == 2; });
    {
        std::lock_guard<std::mutex> lock(probe->mutex);
        assert(probe->starts.back() == "latest");
    }
    rumble.Stop();
    WaitFor(probe, [&] { return probe->completed == 2; });
}

void TestBoundedShutdown() {
    auto probe = std::make_shared<Probe>();
    auto rumble = std::make_unique<operator_gamepad::Rumble>([probe](const auto &session) {
        std::unique_lock<std::mutex> lock(probe->mutex);
        probe->starts.push_back(session->config.device);
        probe->wake.notify_all();
        probe->wake.wait(lock, [&] { return probe->release; });
        ++probe->completed;
        probe->wake.notify_all();
    });
    operator_gamepad::FeedbackConfig config;
    config.device = "blocked";
    rumble->Start(config, "", 0, 0);
    WaitFor(probe, [&] { return !probe->starts.empty(); });
    const auto start = std::chrono::steady_clock::now();
    rumble.reset();
    assert(std::chrono::steady_clock::now() - start < std::chrono::milliseconds(500));
    {
        std::lock_guard<std::mutex> lock(probe->mutex);
        probe->release = true;
        probe->wake.notify_all();
    }
    WaitFor(probe, [&] { return probe->completed == 1; });
}
}  // namespace

int main() {
    TestBlockedOutput();
    TestBoundedShutdown();
    operator_gamepad::SuccessFeedback feedback;
    operator_service::Status status;
    status.online = true;
    status.state = "DAMP";
    status.policy = "walk";
    operator_service::Reply reply;
    reply.ok = true;
    reply.request = {1, "state", "accepted", ""};
    feedback.Track("state", "HOME", status, reply);
    status.request = reply.request;
    assert(!feedback.Update(status));
    status.state = "HOME";
    assert(!feedback.Update(status));  // Actual state alone is not a completed request.
    status.request.phase = "completed";
    assert(feedback.Update(status));
    assert(!feedback.Update(status));  // Only one pulse per successful switch.
    reply.request = {2, "policy", "accepted", ""};
    feedback.Track("policy", "dance", status, reply);
    status.request = {2, "policy", "completed", ""};
    status.policy = "dance";
    assert(feedback.Update(status));
    feedback.Track("policy", "dance", status, reply);
    assert(!feedback.Update(status));  // Re-selecting the current mode is silent.
    for (const auto *phase : {"failed", "rejected", "expired", "cancelled"}) {
        feedback.Track("policy", "walk", status, reply);
        status.request.phase = phase;
        assert(!feedback.Update(status));
        status.request.phase = "completed";
        assert(!feedback.Update(status));
    }
    feedback.Track("policy", "walk", status, reply);
    status.online = false;
    assert(!feedback.Update(status));
    status.online = true;
    status.policy = "walk";
    assert(!feedback.Update(status));  // Never replay after a reconnect.
    feedback.Track("action", "wave", status, reply);
    assert(!feedback.Update(status));
    const auto report = operator_gamepad::Ds4RumbleReport(90);
    for (size_t i = 0; i < report.size(); ++i) {
        const int expected = i == 0 ? 5 : i == 1 ? 1 : (i == 4 || i == 5) ? 90 : 0;
        assert(report[i] == expected);
    }
    const auto off = operator_gamepad::Ds4RumbleReport(0);
    assert(off[4] == 0 && off[5] == 0 && off[1] == 1);
    unsigned char descriptor[]{0x75, 8, 0x85, 5, 0x95, 31, 0x91, 2};
    assert(operator_gamepad::HasDs4Output(descriptor, sizeof(descriptor)));
    descriptor[5] = 30;
    assert(!operator_gamepad::HasDs4Output(descriptor, sizeof(descriptor)));
    assert(!operator_gamepad::HasDs4Output(descriptor, 3));
    int attempts = 0, delays = 0;
    assert(operator_gamepad::SendRumbleOff([&](int strength) {
        assert(strength == 0);
        return ++attempts != 1;  // First USB stop report fails; later reports still stop it.
    }, [&] { ++delays; }));
    assert(attempts == 3 && delays == 2);
    attempts = 0;
    assert(!operator_gamepad::SendRumbleOff([&](int strength) {
        assert(strength == 0);
        ++attempts;
        return false;
    }, [] {}));
    assert(attempts == 3);  // A disconnected receiver cannot cause an infinite retry loop.
    std::puts("gamepad feedback: confirmed switches, failures, reconnect and USB report passed");
}
