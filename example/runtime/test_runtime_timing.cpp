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

}  // namespace

int main() {
    if (!TestProgressWatchdog()) return 1;
    std::cout << "[test] runtime timing helpers: PASS\n";
    return 0;
}
