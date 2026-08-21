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

namespace driver_runtime {

void RenderWholeBodyDiagnostics(
    const whole_body_diagnostics &diagnostics, double cycle_s);
void RecordWholeBodyDiagnostics(const whole_body_diagnostics &diagnostics);

}  // namespace driver_runtime

#endif  // WHOLE_BODY_DIAGNOSTICS_H
