/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file driver_backend_factory.cpp
 * @brief Driver-runtime backend selection and factory
 */

#include <stdexcept>

#include "driver_backend.h"

namespace driver_runtime {

#ifdef HAS_MUJOCO_BACKEND
std::unique_ptr<DriverBackend> CreateMujocoBackend(const std::string &yaml_path);
#endif
#ifdef HAS_WHOLE_BODY_BACKEND
std::unique_ptr<DriverBackend> CreateWholeBodyBackend(const std::string &yaml_path);
#endif

BackendKind ParseBackendKind(const std::optional<std::string> &backend) {
    const std::string name = backend.value_or("mujoco");
    if (name == "mujoco") return BackendKind::MUJOCO;
    if (name == "whole_body") return BackendKind::WHOLE_BODY;
    throw std::runtime_error("unsupported driver.backend: " + name);
}

BackendKind ParseBackendKind(const robot_base::YamlFile &yaml_file) {
    return ParseBackendKind(yaml_file.Read<std::string>("driver.backend"));
}

bool BackendIsCompiled(BackendKind kind) {
    switch (kind) {
        case BackendKind::MUJOCO:
#ifdef HAS_MUJOCO_BACKEND
            return true;
#else
            return false;
#endif
        case BackendKind::WHOLE_BODY:
#ifdef HAS_WHOLE_BODY_BACKEND
            return true;
#else
            return false;
#endif
    }
    return false;
}

std::unique_ptr<DriverBackend> CreateBackend(BackendKind kind, const std::string &yaml_path) {
    switch (kind) {
        case BackendKind::MUJOCO:
#ifdef HAS_MUJOCO_BACKEND
            return CreateMujocoBackend(yaml_path);
#else
            throw std::runtime_error("driver.backend=mujoco was not compiled");
#endif
        case BackendKind::WHOLE_BODY:
#ifdef HAS_WHOLE_BODY_BACKEND
            return CreateWholeBodyBackend(yaml_path);
#else
            throw std::runtime_error("driver.backend=whole_body was not compiled");
#endif
    }
    throw std::runtime_error("invalid driver backend kind");
}

}  // namespace driver_runtime
