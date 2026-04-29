/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file robot_base.cpp
 * @brief robot_base 实现
 */

#include "robot_base.h"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <sstream>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace robot_base {

// 四元数归一化阈值
constexpr double kQuatNormEpsilon = 1e-10;

namespace {

std::vector<std::string> SplitPath(const std::string &path) {
    std::vector<std::string> parts;
    std::stringstream ss(path);
    std::string item;
    while (std::getline(ss, item, '.')) {
        if (!item.empty()) {
            parts.push_back(item);
        }
    }
    return parts;
}

YAML::Node GetNodeByPath(const YAML::Node &root, const std::string &path) {
    if (path.empty()) {
        return root;
    }

    YAML::Node cur = root;
    for (const auto &seg : SplitPath(path)) {
        if (!cur) {
            return YAML::Node();
        }

        std::size_t pos = 0;
        std::string key;
        while (pos < seg.size() && seg[pos] != '[') {
            key.push_back(seg[pos]);
            ++pos;
        }

        if (!key.empty()) {
            const YAML::Node current = cur;
            YAML::Node next = current[key];
            if (!next || !next.IsDefined()) {
                return YAML::Node();
            }
            cur.reset(next);
        }

        while (pos < seg.size()) {
            if (seg[pos] != '[') {
                return YAML::Node();
            }
            std::size_t close = seg.find(']', pos);
            if (close == std::string::npos || close == pos + 1) {
                return YAML::Node();
            }
            const std::string idx_str = seg.substr(pos + 1, close - pos - 1);
            if (!cur || !cur.IsSequence()) {
                return YAML::Node();
            }
            const std::size_t idx = static_cast<std::size_t>(std::stoul(idx_str));
            if (idx >= cur.size()) {
                return YAML::Node();
            }
            const YAML::Node current = cur;
            cur.reset(current[idx]);
            pos = close + 1;
        }
    }

    return cur;
}

}  // namespace

struct YamlFile::Impl {
    YAML::Node root;
};

YamlFile YamlFile::Load(const std::string &yaml_path) {
    fs::path config_file = fs::absolute(yaml_path);
    if (!fs::exists(config_file)) {
        throw std::runtime_error("配置文件不存在: " + config_file.string());
    }

    YamlFile yaml_file;
    yaml_file.yaml_path_ = config_file.string();
    yaml_file.yaml_dir_ = config_file.parent_path().string();
    yaml_file.impl_ = std::make_shared<Impl>();
    try {
        yaml_file.impl_->root = YAML::LoadFile(yaml_file.yaml_path_);
    } catch (const YAML::Exception &e) {
        throw std::runtime_error("YAML 解析失败: " + std::string(e.what()));
    }
    return yaml_file;
}

template <typename T>
std::optional<T> YamlFile::Read(const std::string &path) const {
    try {
        auto node = GetNodeByPath(impl_->root, path);
        if (!node || node.IsNull() || !node.IsDefined()) {
            return std::nullopt;
        }
        return node.as<T>();
    } catch (...) {
        return std::nullopt;
    }
}

// 显式模板实例化，涵盖所有常用类型
template std::optional<int> YamlFile::Read<int>(const std::string &) const;
template std::optional<double> YamlFile::Read<double>(const std::string &) const;
template std::optional<bool> YamlFile::Read<bool>(const std::string &) const;
template std::optional<std::string> YamlFile::Read<std::string>(const std::string &) const;
template std::optional<std::vector<double>> YamlFile::Read<std::vector<double>>(
    const std::string &) const;
template std::optional<std::vector<int>> YamlFile::Read<std::vector<int>>(
    const std::string &) const;
template std::optional<std::vector<std::string>> YamlFile::Read<std::vector<std::string>>(
    const std::string &) const;

std::string YamlFile::ToAbsPath(const std::string &path) const {
    fs::path p(path);
    if (p.is_absolute()) {
        return fs::weakly_canonical(p).string();
    }
    return fs::weakly_canonical(fs::path(yaml_dir_) / p).string();
}

// ==================== RobotData 工厂方法 ====================

RobotData RobotData::FromYaml(const std::string &yaml_path) {
    robot_base::YamlFile yaml_file = robot_base::YamlFile::Load(yaml_path);

    auto num_dof_opt = yaml_file.Read<int>("robot_base.num_dof");
    if (!num_dof_opt) {
        throw std::runtime_error("配置文件缺少 robot_base.num_dof");
    }
    int num_dof = num_dof_opt.value();

    // 验证 num_dof 有效性
    if (num_dof <= 0) {
        throw std::runtime_error("robot_base.num_dof 必须大于 0，当前值: " +
                                std::to_string(num_dof));
    }

    RobotData base = Create(num_dof);

    // 读取 name（可选）
    base.name = yaml_file.Read<std::string>("robot_base.name").value_or("");

    return base;
}

RobotData RobotData::Create(int num_dof) {
    if (num_dof <= 0) {
        throw std::runtime_error("num_dof 必须大于 0");
    }

    RobotData state;
    state.num_dof = num_dof;
    state.InitJointVectors();
    return state;
}

void RobotData::InitJointVectors() {
    if (num_dof > 0) {
        joint_pos.assign(num_dof, 0.0);
        joint_vel.assign(num_dof, 0.0);
    }
}

// ==================== RobotData 数据处理方法 ====================

void RobotData::UpdateImuData() {
    // 从 base_quat 计算 rpy
    QuatToRpy(base_quat, rpy);

    // 从 base_vel[3:6] 提取 gyro
    gyro[0] = base_vel[3];
    gyro[1] = base_vel[4];
    gyro[2] = base_vel[5];
}

void RobotData::Reset() {
    // 重置基座状态
    base_pos = {0, 0, 0};
    base_quat = {1, 0, 0, 0};
    base_vel = {0};

    // 重置 IMU 数据
    rpy = {0, 0, 0};
    gyro = {0, 0, 0};

    // 重置关节状态
    std::fill(joint_pos.begin(), joint_pos.end(), 0.0);
    std::fill(joint_vel.begin(), joint_vel.end(), 0.0);

    // 重置时间戳
    time = 0.0;
}

bool RobotData::IsValid() const {
    return num_dof > 0 && joint_pos.size() == static_cast<size_t>(num_dof) &&
            joint_vel.size() == static_cast<size_t>(num_dof);
}

// ==================== 工具函数 ====================

void QuatToRpy(const std::array<double, 4> &quat, std::array<double, 3> &rpy) {
    // quat = [w, x, y, z]
    double w = quat[0];
    double x = quat[1];
    double y = quat[2];
    double z = quat[3];

    // roll (x-axis rotation)
    double sinr_cosp = 2.0 * (w * x + y * z);
    double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    rpy[0] = std::atan2(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    double sinp = 2.0 * (w * y - z * x);
    if (std::abs(sinp) >= 1.0) {
        rpy[1] = std::copysign(M_PI / 2.0, sinp);
    } else {
        rpy[1] = std::asin(sinp);
    }

    // yaw (z-axis rotation)
    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    rpy[2] = std::atan2(siny_cosp, cosy_cosp);
}

void RpyToQuat(const std::array<double, 3> &rpy, std::array<double, 4> &quat) {
    // rpy = [roll, pitch, yaw]
    double roll = rpy[0];
    double pitch = rpy[1];
    double yaw = rpy[2];

    double cy = std::cos(yaw * 0.5);
    double sy = std::sin(yaw * 0.5);
    double cp = std::cos(pitch * 0.5);
    double sp = std::sin(pitch * 0.5);
    double cr = std::cos(roll * 0.5);
    double sr = std::sin(roll * 0.5);

    // quat = [w, x, y, z]
    quat[0] = cr * cp * cy + sr * sp * sy;  // w
    quat[1] = sr * cp * cy - cr * sp * sy;  // x
    quat[2] = cr * sp * cy + sr * cp * sy;  // y
    quat[3] = cr * cp * sy - sr * sp * cy;  // z
}

void NormalizeQuat(std::array<double, 4> &quat) {
    double norm =
        std::sqrt(quat[0] * quat[0] + quat[1] * quat[1] + quat[2] * quat[2] + quat[3] * quat[3]);

    if (norm > kQuatNormEpsilon) {
        quat[0] /= norm;
        quat[1] /= norm;
        quat[2] /= norm;
        quat[3] /= norm;
    } else {
        // 如果四元数接近零，重置为单位四元数
        quat[0] = 1.0;
        quat[1] = 0.0;
        quat[2] = 0.0;
        quat[3] = 0.0;
    }
}

}  // namespace robot_base
