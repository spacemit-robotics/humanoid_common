/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 * @file protocol.h
 * @brief Versioned operator JSON codecs
 */
#ifndef PROTOCOL_H
#define PROTOCOL_H
#include "operator_types.h"
#include <nlohmann/json.hpp>
namespace operator_service {
using Json = nlohmann::json;
Json Encode(const Velocity &v);
Json Encode(const Policy &v);
Json Encode(const RequestResult &v);
Json Encode(const Status &v);
Status DecodeStatus(const Json &v);
Policy DecodePolicy(const Json &v);
Reply DecodeReply(const Json &v);
} // namespace operator_service
#endif // PROTOCOL_H
