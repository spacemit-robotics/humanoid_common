/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file test_status_reporting.cpp
 * @brief Verify optional interaction failures never suppress core status.
 */
#include <cassert>
#include <limits>
#include <thread>

#include "feedback_sampler.h"
#include "status_reporting.h"
#include "transport_packet.h"

namespace {
void TestFeedbackSnapshot() {
    auto state = robot_base::RobotData::Create(2);
    assert(!transport::MakeFeedbackSnapshot(state, 7, 1, 0, true).available);
    state.has_joint_feedback = true;
    state.has_base_imu = state.has_joint_effort = state.has_acceleration = true;
    state.time = 12.5;
    state.joint_pos = {0.1, -0.2};
    state.joint_vel = {0.3, -0.4};
    state.joint_torque = {1.2, -1.3};
    state.base_quat = {0.5, 0.5, 0.5, 0.5};
    state.gyro = {0.1, 0.2, 0.3};
    state.acceleration = {0, 0, 9.81};
    auto feedback = transport::MakeFeedbackSnapshot(state, 7, 1700000000, 0.002, true);
    feedback.stream_id = 42;
    feedback.packet_sequence = 3;
    assert(feedback.available && feedback.fresh && feedback.num_dof == 2);
    transport::RobotFeedbackPacket packet;
    transport::EncodeFeedback(feedback, &packet);
    const auto restored = transport::DecodeFeedback(packet);
    assert(restored.stream_id == 42 && restored.packet_sequence == 3);
    assert(transport::FeedbackPacketSize(packet.num_dof) < sizeof(packet));
    assert(restored.sequence == 7 && restored.source_time_s == state.time && restored.age_s == 0.002);
    assert(restored.received_at_unix_s == 1700000000);
    assert(restored.joint_pos[1] == -0.2 && restored.joint_vel[1] == -0.4 && restored.joint_torque[1] == -1.3);
    assert(restored.orientation == state.base_quat && restored.angular_velocity == state.gyro);
    assert(restored.acceleration == state.acceleration);
    packet.num_dof = 65;
    assert(!transport::DecodeFeedback(packet).available);
    robot_base::ControlStatus status;
    feedback.joint_pos[0] = std::numeric_limits<double>::quiet_NaN();
    assert(transport::CanEncodeStatus(status, {}));
    transport::EncodeFeedback(feedback, &packet);
    assert(!packet.available && transport::DecodeFeedback(packet).stream_id == feedback.stream_id);
    state.joint_vel.clear();
    assert(!transport::MakeFeedbackSnapshot(state, 7, 0, 0, true).available);
    state = robot_base::RobotData::Create(65);
    assert(!transport::MakeFeedbackSnapshot(state, 7, 0, 0, true).available);
}

void TestFeedbackSampler() {
    transport::FeedbackSampler sampler;
    sampler.SetRate(1000);
    auto state = robot_base::RobotData::Create(2);
    state.has_joint_feedback = true;
    state.time = 1;
    state.joint_pos[1] = 0.5;
    robot_base::RobotFeedback received;
    const auto sink = [&](const transport::RobotFeedbackPacket &packet) {
        received = transport::DecodeFeedback(packet);
    };
    sampler.Observe(state, true, sink);
    const auto first = received;
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
    state.joint_pos[1] = 2.5;
    sampler.Observe(state, false, sink);
    assert(first.available && first.fresh && received.available && !received.fresh);
    assert(received.stream_id == first.stream_id && received.sequence == first.sequence);
    assert(received.joint_pos[1] == 0.5 && received.packet_sequence != first.packet_sequence);
    assert(received.received_at_unix_s == first.received_at_unix_s && received.age_s > first.age_s);
}
}  // namespace

int main() {
    TestFeedbackSnapshot();
    TestFeedbackSampler();
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
