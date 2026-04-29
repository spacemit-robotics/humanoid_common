/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file transport_executor.cpp
 * @brief 传输工厂函数实现
 *
 * 根据 YAML 配置中的 transport.type 字段创建对应的传输实现。
 */

#include "transport_executor.h"
#include "transport_udp_impl.h"

#ifdef HAS_SHM
#include "transport_shm_impl.h"
#endif

#include <stdexcept>
#include <memory>
#include <string>

namespace transport {

std::unique_ptr<TransportBase> Create(const std::string& yaml_path) {
    robot_base::YamlFile yaml_file = robot_base::YamlFile::Load(yaml_path);
    std::string type = yaml_file.Read<std::string>("transport.type").value_or("udp");

#ifdef HAS_UDP
    if (type == "udp") {
        return std::make_unique<TransportUdpImpl>();
    }
#endif

#ifdef HAS_SHM
    if (type == "shm") {
        return std::make_unique<TransportShmImpl>();
    }
#endif

    throw std::runtime_error("[transport_executor] 不支持的传输类型: " + type);
}

}  // namespace transport
