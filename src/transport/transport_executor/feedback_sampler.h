/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file feedback_sampler.h
 * @brief Sample the driver's measurements without consuming the control state channel.
 */
#ifndef FEEDBACK_SAMPLER_H
#define FEEDBACK_SAMPLER_H

#include <algorithm>
#include <chrono>
#include <cmath>

#include "transport_packet.h"

namespace transport {
inline robot_base::RobotFeedback MakeFeedbackSnapshot(const robot_base::RobotData &state,
    uint64_t sequence, double received_at_unix_s, double age_s, bool fresh) {
    robot_base::RobotFeedback result;
    const auto count = static_cast<std::size_t>(state.num_dof);
    if (!state.has_joint_feedback || state.num_dof <= 0 ||
        count > robot_base::RobotFeedback::kMaxJoints || sequence == 0 ||
        state.joint_pos.size() != count || state.joint_vel.size() != count ||
        (state.has_joint_effort && state.joint_torque.size() != count)) return result;
    result.has_base_imu = state.has_base_imu;
    result.has_acceleration = state.has_acceleration;
    result.has_joint_effort = state.has_joint_effort;
    result.available = true;
    result.fresh = fresh;
    result.sequence = sequence;
    result.num_dof = state.num_dof;
    result.source_time_s = state.time;
    result.received_at_unix_s = received_at_unix_s;
    result.age_s = age_s;
    std::copy_n(state.joint_pos.begin(), count, result.joint_pos.begin());
    std::copy_n(state.joint_vel.begin(), count, result.joint_vel.begin());
    if (state.has_joint_effort) std::copy_n(state.joint_torque.begin(), count, result.joint_torque.begin());
    if (state.has_base_imu) {
        result.orientation = state.base_quat;
        result.angular_velocity = state.gyro;
    }
    if (state.has_acceleration) result.acceleration = state.acceleration;
    return result;
}

class FeedbackSampler {
public:
    void SetRate(double hz) {
        if (std::isfinite(hz) && hz > 0) period_ = std::chrono::duration<double>(1.0 / hz);
        stream_id_ = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::system_clock::now().time_since_epoch()).count();
    }

    template <typename Send>
    void Observe(const robot_base::RobotData &state, bool healthy, Send send) {
        const auto now = Clock::now();
        if (state.has_joint_feedback && (sequence_ == 0 || source_time_ != state.time)) {
            ++sequence_;
            source_time_ = state.time;
            received_ = now;
            received_unix_s_ = std::chrono::duration<double>(
                std::chrono::system_clock::now().time_since_epoch()).count();
        }
        if (published_ != Clock::time_point{} && now - published_ < period_) return;
        published_ = now;
        if (state.has_joint_feedback && (!sample_.available || sample_.sequence != sequence_)) {
            sample_ = MakeFeedbackSnapshot(state, sequence_, received_unix_s_, 0, healthy);
        }
        auto sample = state.has_joint_feedback ? sample_ : robot_base::RobotFeedback{};
        sample.stream_id = stream_id_;
        sample.packet_sequence = ++packet_sequence_;
        if (sample.available) {
            sample.fresh = healthy;
            sample.age_s = std::chrono::duration<double>(now - received_).count();
        }
        RobotFeedbackPacket packet{};
        EncodeFeedback(sample, &packet);
        send(packet);
    }

private:
    using Clock = std::chrono::steady_clock;
    Clock::time_point received_{}, published_{};
    std::chrono::duration<double> period_{0.05};
    double received_unix_s_ = 0.0, source_time_ = 0.0;
    uint64_t stream_id_ = 0, sequence_ = 0;
    uint32_t packet_sequence_ = 0;
    robot_base::RobotFeedback sample_;
};
}  // namespace transport
#endif  // FEEDBACK_SAMPLER_H
