/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file server.h
 * @brief Operator HTTP/WebSocket process assembly
 */
#ifndef SERVER_H
#define SERVER_H
#include "service.h"
#include <string>
namespace operator_service {
int Run(Config config, const std::string &yaml_path);
} // namespace operator_service
#endif // SERVER_H
