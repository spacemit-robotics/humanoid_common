/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_adapter.h
 * @brief Internal data conversion between humanoid common and whole_body
 */

#ifndef WHOLE_BODY_ADAPTER_H
#define WHOLE_BODY_ADAPTER_H

#include <cstdint>

#include "robot_base.h"
#include "whole_body.h"

namespace driver_runtime {

bool ConvertWholeBodyState(
    const whole_body_state &source, robot_base::RobotData *destination);

bool ConvertControlCommand(const robot_base::ControlCmd &source, uint32_t num_dof,
    whole_body_joint_command *destination);

}  // namespace driver_runtime

#endif  // WHOLE_BODY_ADAPTER_H
