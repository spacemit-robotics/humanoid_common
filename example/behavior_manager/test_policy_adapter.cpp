/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_policy_adapter.cpp
 * @brief 验证通用 motion tracker 从上层策略配置获取动态维度
 */

#include "policy_adapter/policy_adapter.h"

#include <chrono>
#include <cstdint>
#include <cstring>
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

const char kZip64UnicodeNpz[] =
    "UEsDBBQAAAAAAAAAIQBVhn5qkAAAAJAAAAAPABQAam9pbnRfbmFtZXMubnB5AQAQAJAAAAAA"
    "AAAAkAAAAAAAAACTTlVNUFkBAHYAeydkZXNjcic6ICc8VTInLCAnZm9ydHJhbl9vcmRlcic6"
    "IEZhbHNlLCAnc2hhcGUnOiAoMiwpLCB9ICAgICAgICAgICAgICAgICAgICAgICAgICAgICAg"
    "ICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgCmoAAAAwAAAAagAAADEAAABQSwMELQAA"
    "AAAAAAAhAFglFiP//////////w0AFABqb2ludF9wb3MubnB5AQAQAJAAAAAAAAAAkAAAAAAA"
    "AACTTlVNUFkBAHYAeydkZXNjcic6ICc8ZjQnLCAnZm9ydHJhbl9vcmRlcic6IEZhbHNlLCAn"
    "c2hhcGUnOiAoMiwgMiksIH0gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAg"
    "ICAgICAgICAgICAgICAgICAgICAgCs3MzD3NzEy+zcxMPs3MzL1QSwMELQAAAAAAAAAhAE+k"
    "PBL//////////w0AFABqb2ludF92ZWwubnB5AQAQAJAAAAAAAAAAkAAAAAAAAACTTlVNUFkB"
    "AHYAeydkZXNjcic6ICc8ZjQnLCAnZm9ydHJhbl9vcmRlcic6IEZhbHNlLCAnc2hhcGUnOiAo"
    "MiwgMiksIH0gICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAg"
    "ICAgICAgICAgICAgCgAAAAAAAAAAAAAAAAAAAABQSwMELQAAAAAAAAAhAFpEfnr/////////"
    "/w4AFABib2R5X3Bvc193Lm5weQEAEACYAAAAAAAAAJgAAAAAAAAAk05VTVBZAQB2AHsnZGVz"
    "Y3InOiAnPGY0JywgJ2ZvcnRyYW5fb3JkZXInOiBGYWxzZSwgJ3NoYXBlJzogKDIsIDEsIDMp"
    "LCB9ICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAg"
    "ICAgIAoAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAABQSwMELQAAAAAAAAAhAOfTumP/////////"
    "/w8AFABib2R5X3F1YXRfdy5ucHkBABAAoAAAAAAAAACgAAAAAAAAAJNOVU1QWQEAdgB7J2Rl"
    "c2NyJzogJzxmNCcsICdmb3J0cmFuX29yZGVyJzogRmFsc2UsICdzaGFwZSc6ICgyLCAxLCA0"
    "KSwgfSAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAgICAg"
    "ICAgICAKAACAPwAAAAAAAAAAAAAAAAAAgD8AAAAAAAAAAAAAAABQSwECFAMUAAAAAAAAACEA"
    "VYZ+apAAAACQAAAADwAAAAAAAAAAAAAAgAEAAAAAam9pbnRfbmFtZXMubnB5UEsBAhQDFAAA"
    "AAAAAAAhAFglFiOQAAAAkAAAAA0AAAAAAAAAAAAAAIAB0QAAAGpvaW50X3Bvcy5ucHlQSwEC"
    "FAMUAAAAAAAAACEAT6Q8EpAAAACQAAAADQAAAAAAAAAAAAAAgAGgAQAAam9pbnRfdmVsLm5w"
    "eVBLAQIUAxQAAAAAAAAAIQBaRH56mAAAAJgAAAAOAAAAAAAAAAAAAACAAW8CAABib2R5X3Bv"
    "c193Lm5weVBLAQIUAxQAAAAAAAAAIQDn07pjoAAAAKAAAAAPAAAAAAAAAAAAAACAAUcDAABi"
    "b2R5X3F1YXRfdy5ucHlQSwUGAAAAAAUABQAsAQAAKAQAAAAA";

std::vector<unsigned char> DecodeBase64(const std::string &encoded) {
    static const char kAlphabet[] =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    std::vector<unsigned char> decoded;
    uint32_t accumulator = 0;
    int bits = -8;
    for (const char character : encoded) {
        const char *position = std::strchr(kAlphabet, character);
        if (!position) {
            throw std::runtime_error("测试 NPZ 的 base64 数据非法");
        }
        accumulator = (accumulator << 6) |
            static_cast<uint32_t>(position - kAlphabet);
        bits += 6;
        if (bits >= 0) {
            decoded.push_back(static_cast<unsigned char>(
                (accumulator >> bits) & 0xffU));
            bits -= 8;
        }
    }
    return decoded;
}

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

class TempNpzReference {
public:
    TempNpzReference() {
        const auto nonce = std::chrono::steady_clock::now()
            .time_since_epoch().count();
        path_ = fs::temp_directory_path() /
            ("mjlab_zip64_unicode_" + std::to_string(nonce) + ".npz");
        std::ofstream output(path_, std::ios::binary);
        if (!output) {
            throw std::runtime_error("无法创建临时 NPZ 参考动作");
        }
        const auto bytes = DecodeBase64(kZip64UnicodeNpz);
        output.write(reinterpret_cast<const char *>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
        if (!output) {
            throw std::runtime_error("无法写入临时 NPZ 参考动作");
        }
    }

    ~TempNpzReference() {
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
    config.custom_array_dims["holomotion_obs"] = 46;
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

void TestHolomotionDynamicDimensions() {
    const TempReference reference("holomotion_reference",
        "1,0,0,0,1,0,0,0,0,0,0,0.1,-0.2\n");
    Config config;
    config.type = "holomotion";
    config.reference_file = reference.Path().string();
    config.future_frames = 1;
    config.context_length = 4;

    auto policy_config = MakeTwoDofPolicyConfig();
    auto adapter = Create(config, policy_config);
    Require(adapter && std::string(adapter->Type()) == "holomotion",
        "未创建 HoloMotion adapter");
    adapter->Reset(MakeTwoDofRobot());
    adapter->OnAction({0.1, -0.1});

    bool rejected_wrong_action = false;
    try {
        adapter->OnAction({0.1});
    } catch (const std::runtime_error &) {
        rejected_wrong_action = true;
    }
    Require(rejected_wrong_action, "HoloMotion 未拒绝错误 action 维度");

    policy_config.custom_array_dims.clear();
    bool rejected_missing_obs_dim = false;
    try {
        static_cast<void>(Create(config, policy_config));
    } catch (const std::runtime_error &) {
        rejected_missing_obs_dim = true;
    }
    Require(rejected_missing_obs_dim,
        "HoloMotion 未要求上层声明观测维度");
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

void TestMjlabZip64AndUnicodeMetadata() {
    const TempNpzReference reference;
    Config config;
    config.type = "mjlab";
    config.reference_file = reference.Path().string();
    config.anchor_body_index = 0;
    config.anchor_yaw_align = false;

    auto adapter = Create(config, MakeTwoDofPolicyConfig());
    Require(adapter && std::string(adapter->Type()) == "mjlab",
        "未创建兼容 ZIP64 的 MJLab adapter");
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
        TestHolomotionDynamicDimensions();
        TestProtomotionsDynamicDimensions();
        TestMjlabZip64AndUnicodeMetadata();
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
