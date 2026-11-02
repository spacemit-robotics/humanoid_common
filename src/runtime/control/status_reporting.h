/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file status_reporting.h
 * @brief Isolate invalid optional interaction metadata from the control heartbeat.
 */
#ifndef STATUS_REPORTING_H
#define STATUS_REPORTING_H

#include <string>

#include "transport_executor.h"

namespace control_runtime {

class InteractionReportCache {
public:
    // Return false when keeping the last valid interaction report. FSM, fault,
    // policy-switch and velocity data are never substituted or relaxed.
    bool Update(robot_base::ControlStatus *status) {
        if (policy_ != status->active_policy || status->mode != robot_base::ControlMode::RL) {
            last_valid_ = {};
            policy_ = status->active_policy;
        }
        if (transport::IsValidInteractionStatus(status->interaction)) {
            last_valid_ = status->interaction;
            return true;
        }
        status->interaction = last_valid_;
        return false;
    }

private:
    std::string policy_;
    robot_base::InteractionStatus last_valid_;
};

}  // namespace control_runtime
#endif  // STATUS_REPORTING_H
