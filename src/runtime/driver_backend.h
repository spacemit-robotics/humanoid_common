/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file driver_backend.h
 * @brief Private driver-runtime backend contract
 */

#ifndef DRIVER_BACKEND_H
#define DRIVER_BACKEND_H

#include <functional>
#include <memory>
#include <optional>
#include <string>

#include "robot_base.h"

namespace driver_runtime {

enum class BackendKind {
    MUJOCO,
    WHOLE_BODY,
};

using ExchangeCallback =
    std::function<std::optional<robot_base::ControlCmd>(const robot_base::RobotData &)>;
using ContinueCallback = std::function<bool()>;

class DriverBackend {
public:
    virtual ~DriverBackend() = default;
    virtual int Run(const ExchangeCallback &exchange, const ContinueCallback &should_continue) = 0;
};

BackendKind ParseBackendKind(const robot_base::YamlFile &yaml_file);
BackendKind ParseBackendKind(const std::optional<std::string> &backend);
bool BackendIsCompiled(BackendKind kind);
std::unique_ptr<DriverBackend> CreateBackend(BackendKind kind, const std::string &yaml_path);

}  // namespace driver_runtime

#endif  // DRIVER_BACKEND_H
