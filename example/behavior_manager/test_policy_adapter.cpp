/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_policy_adapter.cpp
 * @brief 验证通用 motion tracker 从上层策略配置获取动态维度
 */

#include "policy_adapter/policy_adapter.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <cnpy.h>

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

// 输入为 SONIC G1 的 1762 维 reference_obs；Gather 输出跨越 joint_pos、
// joint_vel、future orientation 和末尾 padding 的 29 个探针值。
const char kSonicReferenceProbeOnnx[] =
    "CAg66gEKNQoNcmVmZXJlbmNlX29icwoHaW5kaWNlcxIGYWN0aW9uIgZHYXRoZXIqCwoEYXhp"
    "cxgBoAECEhVzb25pY19yZWZlcmVuY2VfcHJvYmUqRwgdEAc6OCAhwgLDAtkE3AThDeEN4Q3h"
    "DeEN4Q3hDeEN4Q3hDeEN4Q3hDeEN4Q3hDeEN4Q3hDeEN4Q3hDeENQgdpbmRpY2VzWiAKDXJl"
    "ZmVyZW5jZV9vYnMSDwoNCAESCQoCCAEKAwjiDVoVCgNvYnMSDgoMCAESCAoCCAEKAggDYhgK"
    "BmFjdGlvbhIOCgwIARIICgIIAQoCCB1CBAoAEA0";

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

class TempSonicReference {
public:
    explicit TempSonicReference(int joint_count)
        : joint_count_(joint_count) {
        const auto nonce = std::chrono::steady_clock::now()
            .time_since_epoch().count();
        path_ = fs::temp_directory_path() /
            ("sonic_reference_" + std::to_string(nonce));
        fs::create_directories(path_);
        WriteJointCsv(path_ / "joint_pos.csv", "joint_", 0.01);
        WriteJointCsv(path_ / "joint_vel.csv", "joint_vel_", 0.02);

        std::ofstream body_quat(path_ / "body_quat.csv");
        if (!body_quat) {
            throw std::runtime_error("无法创建 SONIC 四元数参考动作");
        }
        body_quat << "body_0_w,body_0_x,body_0_y,body_0_z\n"
                << "1,0,0,0\n"
                << "1,0,0,0\n";
    }

    ~TempSonicReference() {
        std::error_code error;
        fs::remove_all(path_, error);
    }

    const fs::path &Path() const { return path_; }

private:
    void WriteJointCsv(const fs::path &path,
        const std::string &prefix,
        double scale) {
        std::ofstream output(path);
        if (!output) {
            throw std::runtime_error("无法创建 SONIC 关节参考动作");
        }
        for (int joint = 0; joint < joint_count_; ++joint) {
            if (joint > 0) {
                output << ',';
            }
            output << prefix << joint;
        }
        output << '\n';
        for (int frame = 0; frame < 2; ++frame) {
            for (int joint = 0; joint < joint_count_; ++joint) {
                if (joint > 0) {
                    output << ',';
                }
                output << scale * (frame + joint);
            }
            output << '\n';
        }
    }

    int joint_count_ = 0;
    fs::path path_;
};

class TempSonicOnnx {
public:
    TempSonicOnnx() {
        const auto nonce = std::chrono::steady_clock::now()
            .time_since_epoch().count();
        path_ = fs::temp_directory_path() /
            ("sonic_reference_probe_" + std::to_string(nonce) + ".onnx");
        std::ofstream output(path_, std::ios::binary);
        if (!output) {
            throw std::runtime_error("无法创建 SONIC 测试 ONNX");
        }
        const auto bytes = DecodeBase64(kSonicReferenceProbeOnnx);
        output.write(reinterpret_cast<const char *>(bytes.data()),
            static_cast<std::streamsize>(bytes.size()));
        if (!output) {
            throw std::runtime_error("无法写入 SONIC 测试 ONNX");
        }
    }

    ~TempSonicOnnx() {
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

rl_policy::PolicyExecutorConfig MakeSonicPolicyConfig(int joint_count) {
    rl_policy::PolicyExecutorConfig config;
    config.rl_default_pos.assign(joint_count, 0.0);
    for (int joint = 0; joint < joint_count; ++joint) {
        config.action_joint_index.push_back(joint);
    }
    return config;
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

void TestMjlabReferenceResidualAction() {
    const TempNpzReference reference;
    Config config;
    config.type = "mjlab";
    config.reference_file = reference.Path().string();
    config.anchor_body_index = 0;
    config.anchor_yaw_align = false;
    config.reference_action.joint_indices = {0, 1};
    config.reference_action.residual_scale = 0.1;
    config.reference_action.residual_clip = 0.5;

    auto policy_config = MakeTwoDofPolicyConfig();
    policy_config.rl_default_pos = {0.5, -0.25};
    policy_config.action_scale = {0.5, 0.25};
    auto adapter = Create(config, policy_config);
    adapter->Reset(MakeTwoDofRobot());

    std::vector<double> action = {1.0, -1.0};
    bool rejected_missing_sample = false;
    try {
        adapter->OnAction(action);
    } catch (const std::runtime_error &) {
        rejected_missing_sample = true;
    }
    Require(rejected_missing_sample,
        "MJLab reference_action 未拒绝缺少参考采样的 action");

    rl_policy::PolicyExecutor policy;
    adapter->PrepareInputs(MakeTwoDofRobot(), 0.0, policy);
    action = {1.0, -1.0};
    adapter->OnAction(action);
    Require(std::abs(action[0] - 0.2) < 1.0e-6,
        "MJLab reference_action 未正确映射关节 1");
    Require(std::abs(action[1] + 1.8) < 1.0e-6,
        "MJLab reference_action 未正确映射关节 0");

    // 50 Hz 参考在 10 ms 时位于两帧中点，验证输出合成复用的正是
    // 当前 observation 所采样的插值参考，而不是上一帧或下一帧。
    adapter->PrepareInputs(MakeTwoDofRobot(), 0.01, policy);
    action = {0.0, 0.0};
    adapter->OnAction(action);
    Require(std::abs(action[0] - 0.2) < 1.0e-6,
        "MJLab reference_action 未使用当前插值参考映射关节 1");
    Require(std::abs(action[1] + 1.4) < 1.0e-6,
        "MJLab reference_action 未使用当前插值参考映射关节 0");

    Config passthrough_config = config;
    passthrough_config.reference_action.joint_indices.clear();
    auto passthrough = Create(passthrough_config, policy_config);
    std::vector<double> passthrough_action = {0.3, -0.4};
    passthrough->OnAction(passthrough_action);
    Require(passthrough_action == std::vector<double>({0.3, -0.4}),
        "未配置 reference_action 时不应改写 action");
}

void TestMjlabReferenceActionYaml() {
    const TempReference yaml("mjlab_reference_action_yaml",
        "rl_policy:\n"
        "  onnx_infer:\n"
        "    policies:\n"
        "      reference_action_policy:\n"
        "        policy_adapter:\n"
        "          type: mjlab\n"
        "          reference_file: motion.npz\n"
        "          reference_action:\n"
        "            joint_indices: [0, 1]\n"
        "            residual_scale: 0.0\n"
        "            residual_clip: 0.75\n");
    const auto config = LoadConfig(
        yaml.Path().string(), "reference_action_policy",
        fs::temp_directory_path().string());
    Require(config.reference_action.joint_indices ==
            std::vector<int>({0, 1}),
        "未加载 reference_action.joint_indices");
    Require(std::abs(config.reference_action.residual_scale) < 1.0e-12,
        "未加载 reference_action.residual_scale");
    Require(std::abs(config.reference_action.residual_clip - 0.75) < 1.0e-12,
        "未加载 reference_action.residual_clip");
}

void TestSonicReferenceAndConfig() {
    constexpr int kTestJointCount = 7;
    const TempSonicReference reference(kTestJointCount);
    Config config;
    config.type = "sonic";
    config.reference_file = reference.Path().string();
    config.future_frames = 3;
    config.future_step = 2;

    auto adapter = Create(config, MakeSonicPolicyConfig(kTestJointCount));
    Require(adapter && std::string(adapter->Type()) == "sonic",
        "未创建 SONIC adapter");

    robot_base::RobotData robot;
    robot.num_dof = kTestJointCount;
    robot.base_quat = {1.0, 0.0, 0.0, 0.0};
    robot.joint_pos.assign(kTestJointCount, 0.0);
    robot.joint_vel.assign(kTestJointCount, 0.0);
    adapter->Reset(robot);

    Config invalid = config;
    invalid.future_step = 0;
    bool rejected_wrong_window = false;
    try {
        static_cast<void>(Create(
            invalid, MakeSonicPolicyConfig(kTestJointCount)));
    } catch (const std::runtime_error &) {
        rejected_wrong_window = true;
    }
    Require(rejected_wrong_window,
        "SONIC 未拒绝非法的未来窗口");

    const TempReference yaml("sonic_adapter_yaml",
        "rl_policy:\n"
        "  onnx_infer:\n"
        "    policies:\n"
        "      sonic:\n"
        "        policy_adapter:\n"
        "          type: sonic\n"
        "          reference_file: " + reference.Path().string() + "\n"
        "          future_frames: 10\n"
        "          future_step: 5\n");
    const auto loaded = LoadConfig(yaml.Path().string(), "sonic",
        fs::temp_directory_path().string());
    Require(loaded.future_frames == 10 && loaded.future_step == 5,
        "未加载 SONIC future window 配置");
}

void TestSonicPrepareInputs() {
    constexpr int kG1JointCount = 29;
    const TempSonicReference reference(kG1JointCount);
    const TempSonicOnnx model;

    Config config;
    config.type = "sonic";
    config.reference_file = reference.Path().string();
    config.future_frames = 10;
    config.future_step = 5;

    auto policy_config = MakeSonicPolicyConfig(kG1JointCount);
    policy_config.model_path = model.Path().string();
    policy_config.action_scale = {1.0};
    policy_config.strict_obs_dim_check = true;
    policy_config.obs_segments.emplace_back();
    policy_config.obs_segments.back().terms = {"gravity"};

    rl_policy::ModelInputBindingConfig input;
    input.name = "reference_obs";
    input.source = rl_policy::ModelInputSource::EXTERNAL;
    policy_config.model_io.inputs.push_back(input);

    input.name = "obs";
    input.source = rl_policy::ModelInputSource::OBSERVATION;
    policy_config.model_io.inputs.push_back(input);

    rl_policy::ModelOutputBindingConfig output;
    output.name = "action";
    output.target = rl_policy::ModelOutputTarget::ACTION;
    policy_config.model_io.outputs.push_back(output);

    auto adapter = Create(config, policy_config);
    robot_base::RobotData robot;
    robot.num_dof = kG1JointCount;
    robot.base_quat = {1.0, 0.0, 0.0, 0.0};
    robot.joint_pos.assign(kG1JointCount, 0.0);
    robot.joint_vel.assign(kG1JointCount, 0.0);
    adapter->Reset(robot);

    rl_policy::PolicyExecutor policy;
    policy.Init(policy_config);
    adapter->PrepareInputs(robot, 0.0, policy);

    Eigen::VectorXf observation = Eigen::VectorXf::Zero(policy.ObsDim());
    std::vector<double> action;
    policy.Infer(observation, action);
    const std::vector<double> expected = {
        0.28, 0.01, 0.56, 0.02, 1.0, 1.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    };
    Require(action.size() == expected.size(),
        "SONIC PrepareInputs 未生成 1762 维模型所需输入");
    for (std::size_t index = 0; index < expected.size(); ++index) {
        Require(std::abs(action[index] - expected[index]) < 1.0e-6,
            "SONIC reference_obs 字段拼接顺序错误: index=" +
            std::to_string(index));
    }
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
    robot.joint_pos = loaded.exec_cfg.rl_default_pos;
    robot.joint_vel.assign(robot.num_dof, 0.0);
    adapter->Reset(robot);

    rl_policy::PolicyExecutor policy;
    policy.Init(loaded.exec_cfg);
    adapter->PrepareInputs(robot, 0.0, policy);
    const std::array<double, 3> zeros3 = {};
    const std::array<double, 4> identity_quat = {1.0, 0.0, 0.0, 0.0};
    Eigen::VectorXf observation;
    policy.AssembleObs(zeros3, zeros3, 0.0, 0.0, 0.0,
        robot.joint_pos, robot.joint_vel, identity_quat, zeros3,
        0.02F, observation);
    Require(observation.size() == policy.ObsDim(),
        "配置策略观测维度错误");

    std::vector<double> action;
    policy.Infer(observation, action);
    const std::vector<double> raw_action = action;
    adapter->OnAction(action);
    std::vector<double> target_pos;
    policy.MapActionToTargetPos(action, target_pos);
    Require(target_pos.size() == loaded.exec_cfg.rl_default_pos.size(),
        "配置策略目标关节维度错误");
    Require(std::all_of(target_pos.begin(), target_pos.end(),
        [](double value) { return std::isfinite(value); }),
        "配置策略目标关节包含非有限值");

    double max_reference_error = 0.0;
    if (adapter_config.reference_action.Enabled()) {
        const auto reference =
            cnpy::npz_load(adapter_config.reference_file, "joint_pos");
        Require(reference.shape.size() == 2 &&
                reference.shape[0] > 0 &&
                reference.shape[1] == target_pos.size() &&
                reference.word_size == sizeof(float),
            "reference_action 集成测试参考轨迹格式错误");
        const float *reference_frame = reference.data<float>();
        for (const int joint_index :
                adapter_config.reference_action.joint_indices) {
            int action_index = joint_index;
            if (!loaded.exec_cfg.action_joint_index.empty()) {
                const auto position = std::find(
                    loaded.exec_cfg.action_joint_index.begin(),
                    loaded.exec_cfg.action_joint_index.end(), joint_index);
                Require(position != loaded.exec_cfg.action_joint_index.end(),
                    "reference_action 关节未被 action 控制");
                action_index = static_cast<int>(std::distance(
                    loaded.exec_cfg.action_joint_index.begin(), position));
            }
            const double residual = std::clamp(raw_action[action_index],
                -adapter_config.reference_action.residual_clip,
                adapter_config.reference_action.residual_clip) *
                adapter_config.reference_action.residual_scale;
            const double expected = reference_frame[joint_index] + residual;
            max_reference_error = std::max(max_reference_error,
                std::abs(target_pos[joint_index] - expected));
        }
        Require(max_reference_error < 1.0e-5,
            "reference_action 最终关节目标与训练公式不一致");
    }
    std::cout << "已验证策略配置: " << policy_name
        << ", type=" << adapter->Type()
        << ", robot_dof=" << robot.num_dof
        << ", obs=" << observation.size()
        << ", action=" << raw_action.size()
        << ", reference_action_max_error=" << max_reference_error
        << std::endl;
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
        TestMjlabZip64AndUnicodeMetadata();
        TestMjlabReferenceResidualAction();
        TestMjlabReferenceActionYaml();
        TestSonicReferenceAndConfig();
        TestSonicPrepareInputs();
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
