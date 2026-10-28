/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_status_reporting.cpp
 * @brief Verify optional interaction failures never suppress core status.
 */
#include <cassert>
#include <limits>

#include "status_reporting.h"
#include "transport_packet.h"

int main() {
    using Phase = robot_base::InteractionStatus::Phase;
    control_runtime::InteractionReportCache cache;
    robot_base::ControlStatus status;
    status.mode = robot_base::ControlMode::RL;
    status.active_policy = "dance";
    status.hmi_connected = true;
    status.interaction.sequence = 10;
    status.interaction.phase = Phase::BLEND_OUT;
    assert(!cache.Update(&status));
    assert(transport::CanEncodeStatus(status, {}));
    assert(status.mode == robot_base::ControlMode::RL && status.hmi_connected);
    assert(status.interaction.phase == Phase::IDLE);
    status.interaction = {11, true, Phase::PLAYING, 0.2F, "rhythm_1"};
    assert(cache.Update(&status));
    status.interaction.progress = std::numeric_limits<float>::quiet_NaN();
    status.vx = 0.1;
    assert(!cache.Update(&status));
    assert(status.interaction.phase == Phase::PLAYING && status.interaction.progress == 0.2F);
    assert(status.vx == 0.1F && transport::CanEncodeStatus(status, {}));
    status.active_policy = "walk";
    status.interaction.phase = Phase::BLEND_OUT;
    status.interaction.action.clear();
    assert(!cache.Update(&status));
    assert(status.interaction.phase == Phase::IDLE);
    // A corrupt mandatory status field is still rejected, never silently fixed.
    status.rl_frequency_hz = std::numeric_limits<float>::infinity();
    assert(cache.Update(&status));
    assert(!transport::CanEncodeStatus(status, {}));
}
