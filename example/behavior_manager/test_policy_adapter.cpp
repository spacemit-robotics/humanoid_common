/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_policy_adapter.cpp
 * @brief 验证通用 motion tracker 从上层策略配置获取动态维度
 */

#include "policy_adapter/policy_adapter.h"

#include <chrono>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

namespace {

namespace fs = std::filesystem;
using behavior_manager::policy_adapter::Config;
using behavior_manager::policy_adapter::Create;
using behavior_manager::policy_adapter::LoadConfig;

class TempReference {
public:
    TempReference(const std::string &name, const std::string &contents) {
        const auto nonce = std::chrono::steady_clock::now()
            .time_since_epoch().count();
        path_ = fs::temp_directory_path() /
            (name + "_" + std::to_string(nonce) + ".csv");
        std::ofstream output(path_);
        if (!output) {
            throw std::runtime_error("无法创建临时参考动作");
        }
        output << contents;
    }

    ~TempReference() {
        std::error_code error;
        fs::remove(path_, error);
    }

    const fs::path &Path() const { return path_; }

private:
    fs::path path_;
};

void Require(bool condition, const std::string &message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

rl_policy::PolicyExecutorConfig MakeTwoDofPolicyConfig() {
    rl_policy::PolicyExecutorConfig config;
    config.rl_default_pos = {0.0, 0.0};
    config.action_joint_index = {1, 0};
    return config;
}

robot_base::RobotData MakeTwoDofRobot() {
    robot_base::RobotData robot;
    robot.num_dof = 2;
    robot.base_quat = {1.0, 0.0, 0.0, 0.0};
    robot.joint_pos = {0.1, -0.2};
    robot.joint_vel = {0.3, -0.4};
    return robot;
}

void TestProtomotionsDynamicDimensions() {
    const TempReference reference("protomotions_reference",
        "1,0,0,0,1,0,0,0,1,0.1,-0.2,0.3,-0.4\n");
    Config config;
    config.type = "protomotions";
    config.reference_file = reference.Path().string();
    config.future_steps = {1, 3};
    config.anchor_waist_joint_indices = {0, 1, 0};

    auto adapter = Create(config, MakeTwoDofPolicyConfig());
    Require(adapter && std::string(adapter->Type()) == "protomotions",
        "未创建 ProtoMotions adapter");
    adapter->Reset(MakeTwoDofRobot());
}

void TestConfiguredPolicy(const std::string &yaml_path,
    const std::string &robot_dir,
    const std::string &policy_name) {
    const auto loaded = rl_policy::LoadPolicyConfigFromYaml(
        yaml_path, policy_name, robot_dir);
    const auto adapter_config =
        LoadConfig(yaml_path, policy_name, robot_dir);
    auto adapter = Create(adapter_config, loaded.exec_cfg);
    Require(adapter != nullptr, "配置未创建 policy_adapter");

    robot_base::RobotData robot;
    robot.num_dof = static_cast<int>(loaded.exec_cfg.rl_default_pos.size());
    robot.base_quat = {1.0, 0.0, 0.0, 0.0};
    robot.joint_pos.assign(robot.num_dof, 0.0);
    robot.joint_vel.assign(robot.num_dof, 0.0);
    adapter->Reset(robot);
    std::cout << "已验证策略配置: " << policy_name
        << ", type=" << adapter->Type()
        << ", robot_dof=" << robot.num_dof << std::endl;
}

}  // namespace

int main(int argc, char **argv) {
    try {
        if (argc != 1 && argc != 4) {
            std::cerr << "Usage: " << argv[0]
                << " [YAML ROBOT_DIR POLICY_NAME]" << std::endl;
            return 2;
        }
        TestProtomotionsDynamicDimensions();
        if (argc == 4) {
            TestConfiguredPolicy(argv[1], argv[2], argv[3]);
        }
        std::cout << "policy_adapter 动态维度测试完成" << std::endl;
        return 0;
    } catch (const std::exception &error) {
        std::cerr << "policy_adapter 动态维度测试失败: "
            << error.what() << std::endl;
        return 1;
    }
}
