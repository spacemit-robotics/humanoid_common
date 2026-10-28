/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_diagnostics.h
 * @brief Driver-runtime rendering and recording of whole-body diagnostics
 */

#ifndef WHOLE_BODY_DIAGNOSTICS_H
#define WHOLE_BODY_DIAGNOSTICS_H

#include "whole_body.h"
#include "whole_body_transport.h"

namespace driver_runtime {

struct WholeBodyDiagnosticsSnapshot {
    whole_body_diagnostics_v2 feedback{};
    whole_body_motor_command_diagnostics_v2 command{};
    whole_body_transport_diagnostics transport{};
    double wall_time = 0.0;
    bool render = false;
    bool record = false;
};

void RenderWholeBodyDiagnostics(
    const whole_body_diagnostics_v2 &diagnostics, double cycle_s, bool imu_configured = true);
void RecordWholeBodyDiagnostics(const whole_body_diagnostics_v2 &diagnostics,
    const whole_body_motor_command_diagnostics_v2 &command_diagnostics, double wall_time_s,
    bool imu_configured = true);
void RecordTransportDiagnostics(const whole_body_transport_diagnostics &diagnostics,
    double wall_time_s);

}  // namespace driver_runtime

#endif  // WHOLE_BODY_DIAGNOSTICS_H
