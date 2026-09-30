/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_runtime_timing.cpp
 * @brief Tests runtime scheduling and state-progress helpers
 */

#include <chrono>
#include <iostream>

#include "runtime_timing.h"

namespace {

bool Expect(bool condition, const char *message) {
    if (condition) return true;
    std::cerr << "[test] " << message << "\n";
    return false;
}

bool TestProgressWatchdog() {
    using Clock = runtime_timing::Clock;
    const Clock::time_point start{};
    runtime_timing::ProgressWatchdog watchdog;

    bool ok = true;
    ok = Expect(!watchdog.HasValue(), "watchdog should start empty") && ok;
    watchdog.Observe(10.0, start);
    ok = Expect(!watchdog.Expired(start + std::chrono::milliseconds(40), 0.05),
        "first state expired too early") && ok;

    watchdog.Observe(10.0, start + std::chrono::milliseconds(20));
    ok = Expect(watchdog.Expired(start + std::chrono::milliseconds(51), 0.05),
        "duplicate state incorrectly refreshed progress") && ok;

    watchdog.Observe(10.1, start + std::chrono::milliseconds(60));
    ok = Expect(!watchdog.Expired(start + std::chrono::milliseconds(100), 0.05),
        "advancing state did not recover progress") && ok;

    watchdog.Observe(9.0, start + std::chrono::milliseconds(105));
    ok = Expect(watchdog.Expired(start + std::chrono::milliseconds(111), 0.05),
        "regressing state incorrectly refreshed progress") && ok;

    watchdog.Observe(10.2, start + std::chrono::milliseconds(120));
    ok = Expect(!watchdog.Expired(start + std::chrono::milliseconds(121), 0.05),
        "newer state did not recover after a regression") && ok;

    watchdog.Reset();
    ok = Expect(!watchdog.HasValue(), "watchdog reset did not clear its baseline") && ok;
    watchdog.Observe(1.0, start + std::chrono::milliseconds(130));
    ok = Expect(!watchdog.Expired(start + std::chrono::milliseconds(131), 0.05),
        "watchdog did not accept a restarted source after reset") && ok;
    return ok;
}

bool TestStateStartupGate() {
    using Clock = runtime_timing::Clock;
    const Clock::time_point start{};
    runtime_timing::StateStartupGate gate;
    bool ok = true;
    ok = Expect(!gate.Ready(), "state stream should start unready") && ok;
    ok = Expect(!gate.Expired(start + std::chrono::seconds(10), 1.5),
        "waiting for a late driver must not start the confirmation timeout") && ok;
    ok = Expect(!gate.Observe(1.0, start, 0.05),
        "first startup packet must not enable control") && ok;
    ok = Expect(!gate.Observe(1.0, start + std::chrono::milliseconds(20), 0.05),
        "duplicate device time must not enable control") && ok;
    ok = Expect(gate.Observe(1.02, start + std::chrono::milliseconds(40), 0.05),
        "fresh advancing state should enable control") && ok;
    ok = Expect(!gate.Expired(start + std::chrono::seconds(10), 1.5),
        "ready stream must use the runtime watchdog instead") && ok;

    gate.Reset();
    ok = Expect(!gate.Observe(2.0, start, 0.05),
        "restarted stream should need a new baseline") && ok;
    ok = Expect(!gate.Observe(2.02, start + std::chrono::milliseconds(100), 0.05),
        "packet gap must restart startup confirmation") && ok;
    ok = Expect(gate.Observe(2.04, start + std::chrono::milliseconds(120), 0.05),
        "stream should recover after two fresh packets") && ok;

    gate.Reset();
    ok = Expect(!gate.Observe(3.0, start, 0.05),
        "fault recovery should need a new baseline") && ok;
    gate.Reset();
    ok = Expect(!gate.Observe(3.02, start + std::chrono::milliseconds(10), 0.05),
        "pre-fault packet must not qualify recovery") && ok;
    ok = Expect(gate.Observe(3.04, start + std::chrono::milliseconds(30), 0.05),
        "post-fault progress should restore readiness") && ok;

    gate.Reset();
    ok = Expect(!gate.Observe(4.0, start, 0.05),
        "new stream should need a second packet") && ok;
    ok = Expect(!gate.Observe(3.99, start + std::chrono::milliseconds(10), 0.05),
        "regressing device time must restart startup confirmation") && ok;
    ok = Expect(gate.Observe(4.01, start + std::chrono::milliseconds(20), 0.05),
        "newer device time should qualify the restarted stream") && ok;

    gate.Reset();
    gate.Observe(5.0, start, 0.05);
    ok = Expect(!gate.Expired(start + std::chrono::milliseconds(1490), 1.5),
        "startup packet loss must retain the grace window") && ok;
    ok = Expect(gate.Expired(start + std::chrono::milliseconds(1510), 1.5),
        "permanent startup packet loss must expire") && ok;

    gate.Reset();
    for (int elapsed_ms = 0; elapsed_ms <= 1520; elapsed_ms += 20) {
        ok = Expect(!gate.Observe(6.0,
            start + std::chrono::milliseconds(elapsed_ms), 0.05),
            "frozen startup time must not enable control") && ok;
    }
    ok = Expect(gate.Expired(start + std::chrono::milliseconds(1520), 1.5),
        "candidate resets must not extend the overall startup deadline") && ok;
    ok = Expect(gate.Observe(6.02, start + std::chrono::milliseconds(1540), 0.05),
        "advancing feedback should recover after a startup timeout") && ok;
    ok = Expect(!gate.Expired(start + std::chrono::milliseconds(1550), 1.5),
        "recovered stream must stop reporting the startup timeout") && ok;
    return ok;
}

}  // namespace

int main() {
    if (!TestProgressWatchdog()) return 1;
    if (!TestStateStartupGate()) return 1;
    std::cout << "[test] runtime timing helpers: PASS\n";
    return 0;
}
