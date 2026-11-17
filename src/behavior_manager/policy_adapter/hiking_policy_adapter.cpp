// Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
// SPDX-License-Identifier: Apache-2.0
#include "hiking_policy_adapter.h"
#include "depth_stream.h"
#include "runtime_logger.h"
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <numeric>
#include <openssl/evp.h>
#include <sstream>
#include <stdexcept>

namespace behavior_manager::policy_adapter {
namespace {
void VerifyModel(const std::string &path, const std::string &expected) {
    if (expected.size() != 64 || !std::all_of(expected.begin(), expected.end(), [](char c) {
            return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
        }))
        throw std::runtime_error("Hiking needs a lowercase SHA256 digest for: " + path);
    std::ifstream file(path, std::ios::binary);
    if (!file)
        throw std::runtime_error("Hiking model missing: " + path);
    auto context =
        std::unique_ptr<EVP_MD_CTX, decltype(&EVP_MD_CTX_free)>(EVP_MD_CTX_new(), EVP_MD_CTX_free);
    if (!context || EVP_DigestInit_ex(context.get(), EVP_sha256(), nullptr) != 1)
        throw std::runtime_error("Hiking SHA256 initialization failed");
    char data[65536];
    while (file.read(data, sizeof(data)) || file.gcount())
        if (EVP_DigestUpdate(context.get(), data, file.gcount()) != 1)
            throw std::runtime_error("Hiking SHA256 failed");
    if (!file.eof())
        throw std::runtime_error("Hiking model read failed: " + path);
    unsigned char hash[EVP_MAX_MD_SIZE];
    unsigned int length = 0;
    if (EVP_DigestFinal_ex(context.get(), hash, &length) != 1)
        throw std::runtime_error("Hiking SHA256 failed");
    std::ostringstream value;
    value << std::hex << std::setfill('0');
    for (unsigned int i = 0; i < length; ++i)
        value << std::setw(2) << int(hash[i]);
    if (expected != value.str())
        throw std::runtime_error("Hiking model SHA256 mismatch: " + path);
}
class Hiking final : public PolicyAdapter {
    public:
    Hiking(const Config &cfg, const rl_policy::PolicyExecutorConfig &policy)
        : cfg_(cfg.hiking), receiver_(cfg_.endpoint, cfg_.timeout_s) {
        if (cfg_.clock != "sim" && cfg_.clock != "robot")
            throw std::runtime_error("Hiking clock must be sim or robot");
        VerifyModel(cfg_.encoder, cfg_.encoder_sha256);
        VerifyModel(policy.model_path, cfg_.actor_sha256);
        map_ = policy.action_joint_index;
        signs_ = cfg_.joint_signs;
        if (map_.size() != 29 || signs_.size() != 29)
            throw std::runtime_error("Hiking joint layout mismatch");
        std::vector<int> sorted = map_;
        std::sort(sorted.begin(), sorted.end());
        for (int i = 0; i < 29; ++i)
            if (sorted[i] != i || std::abs(signs_[i]) != 1)
                throw std::runtime_error("Hiking invalid mapping");
        if (policy.action_scale.size() != 29 || policy.rl_default_pos.size() != 29)
            throw std::runtime_error("Hiking action configuration dimensions mismatch");
        defaults_.resize(29);
        for (int i = 0; i < 29; ++i) {
            if (!std::isfinite(policy.action_scale[i]) || signs_[i] * policy.action_scale[i] <= 0 ||
                !std::isfinite(policy.rl_default_pos[map_[i]]))
                throw std::runtime_error("Hiking invalid signed action scale or default pose");
            // YAML stores SDK-order physical poses; observations use model order/signs.
            defaults_[i] = signs_[i] * policy.rl_default_pos[map_[i]];
        }
        physical_defaults_ = policy.rl_default_pos;
        action_scale_ = policy.action_scale;
        action_clip_.assign(29, std::numeric_limits<float>::max());
        if (!policy.clip_actions.empty()) {
            if (policy.clip_actions.size() != 1 && policy.clip_actions.size() != 29)
                throw std::runtime_error("Hiking invalid action clipping dimensions");
            for (int i = 0; i < 29; ++i)
                action_clip_[i] = policy.clip_actions[policy.clip_actions.size() == 1 ? 0 : i];
        }
        if (policy.action_blend_ratio != 1)
            throw std::runtime_error("Hiking action blending must be one");
        rl_policy::PolicyExecutorConfig enc;
        enc.model_path = cfg_.encoder;
        enc.runtime = policy.runtime;
        enc.action_scale = {1};
        enc.action_blend_ratio = 1;
        enc.rl_default_pos.assign(128, 0);
        enc.clip_actions = {1e9};
        enc.action_joint_index.resize(128);
        std::iota(enc.action_joint_index.begin(), enc.action_joint_index.end(), 0);
        enc.custom_array_dims["depth"] = perception::kStack;
        rl_policy::ObsSegmentConfig segment;
        segment.terms = {"depth"};
        enc.obs_segments.push_back(segment);
        rl_policy::ModelInputBindingConfig in;
        in.name = "input";
        in.source = rl_policy::ModelInputSource::OBSERVATION;
        enc.model_io.inputs.push_back(in);
        rl_policy::ModelOutputBindingConfig out;
        out.name = "output";
        out.target = rl_policy::ModelOutputTarget::ACTION;
        enc.model_io.outputs.push_back(out);
        encoder_.Init(enc);
        if (encoder_.ObsDim() != perception::kStack || encoder_.ActionDim() != 128)
            throw std::runtime_error("Hiking encoder dimensions mismatch");
        actor_input_.resize(896);
        depth_obs_.resize(perception::kStack);
        previous_.assign(29, 0);
        history_.assign(768, 0);
        runtime_logging::RecordArtifact("policy_aux_model", "hiking_encoder", cfg_.encoder);
    }
    const char *Type() const override { return "hiking"; }
    std::string EntryError(const robot_base::RobotData &robot) const override {
        return receiver_.Error(robot.time, cfg_.clock);
    }
    void Reset(const robot_base::RobotData &) override {
        std::fill(previous_.begin(), previous_.end(), 0);
        first_ = true;
        last_time_ = -1;
    }
    void ResetForHandoff(const robot_base::RobotData &robot,
                        const std::vector<double> &targets) override {
        Reset(robot);
        if (targets.size() != 29)
            throw std::runtime_error("Hiking handoff needs 29 applied joint targets");
        for (int i = 0; i < 29; ++i) {
            const double action = (targets[map_[i]] - physical_defaults_[map_[i]]) / action_scale_[i];
            if (!std::isfinite(action))
                throw std::runtime_error("Hiking invalid handoff target");
            previous_[i] = std::clamp(action, -action_clip_[i], action_clip_[i]);
        }
        // The first observation fills each proprioceptive history with the new
        // live state; previous actions are expressed in this model's own order.
    }
    rl_policy::PolicyExecutor &SelectExecutor(const std::array<double, 3> &cmd,
        rl_policy::PolicyExecutor &primary) override {
        command_ = cmd;
        return primary;
    }
    void PrepareInputs(const robot_base::RobotData &r, double,
        rl_policy::PolicyExecutor &policy) override {
        if (r.joint_pos.size() != 29 || r.joint_vel.size() != 29)
            throw std::runtime_error("Hiking needs 29 joints");
        if (last_time_ >= 0 && r.time < last_time_)
            throw std::runtime_error("Hiking state time reset; re-enter policy");
        last_time_ = r.time;
        const auto stack = receiver_.Stack(r.time, cfg_.clock);
        // SDK base IMU is pelvis-local. Compose the physical yaw-roll-pitch waist chain.
        const double yaw = r.joint_pos[12], roll = r.joint_pos[13], pitch = r.joint_pos[14];
        Eigen::Matrix3d rz = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()).toRotationMatrix();
        Eigen::Matrix3d rx = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()).toRotationMatrix();
        Eigen::Matrix3d ry = Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix();
        const auto waist = (rz * rx * ry).eval();
        Eigen::Quaterniond pelvis(r.base_quat[0], r.base_quat[1], r.base_quat[2], r.base_quat[3]);
        if (!pelvis.coeffs().allFinite() || pelvis.norm() < 1e-9)
            throw std::runtime_error("Hiking invalid IMU quaternion");
        const auto rotation = (pelvis.normalized().toRotationMatrix() * waist).eval();
        Eigen::Vector3d omega(r.gyro[0], r.gyro[1], r.gyro[2]);
        omega += Eigen::Vector3d::UnitZ() * r.joint_vel[12] +
            rz * Eigen::Vector3d::UnitX() * r.joint_vel[13] +
            rz * rx * Eigen::Vector3d::UnitY() * r.joint_vel[14];
        omega = waist.transpose() * omega;
        const auto gravity = (rotation.transpose() * Eigen::Vector3d(0, 0, -1)).eval();
        std::vector<float> terms;
        for (int i = 0; i < 3; ++i)
            terms.push_back(omega[i] * .25);
        for (int i = 0; i < 3; ++i)
            terms.push_back(gravity[i]);
        for (double v : command_)
            terms.push_back(v);
        for (int i = 0; i < 29; ++i)
            terms.push_back(signs_[i] * r.joint_pos[map_[i]] - defaults_[i]);
        for (int i = 0; i < 29; ++i)
            terms.push_back(signs_[i] * r.joint_vel[map_[i]] * .05);
        terms.insert(terms.end(), previous_.begin(), previous_.end());
        const int sizes[6] = {3, 3, 3, 29, 29, 29};
        int source = 0, dest = 0;
        for (int size : sizes) {
            auto start = history_.begin() + dest;
            if (first_)
                for (int i = 0; i < 8; ++i)
                    std::copy_n(terms.begin() + source, size, start + i * size);
            else {
                std::move(start + size, start + 8 * size, start);
                std::copy_n(terms.begin() + source, size, start + 7 * size);
            }
            source += size;
            dest += 8 * size;
        }
        first_ = false;
        encoder_.SetCustomArray("depth", stack.data(), stack.size());
        const std::array<double, 3> zero = {};
        const std::array<double, 4> identity = {1, 0, 0, 0};
        encoder_.AssembleObs(zero, zero, 0, 0, 0, std::vector<double>(128),
            std::vector<double>(128), identity, {}, .02, depth_obs_);
        encoder_.Infer(depth_obs_, latent_);
        std::copy(history_.begin(), history_.end(), actor_input_.begin());
        std::copy(latent_.begin(), latent_.end(), actor_input_.begin() + 768);
        policy.SetCustomArray("hiking_input", actor_input_.data(), actor_input_.size());
        const auto now = std::chrono::steady_clock::now();
        if (now - reported_ > std::chrono::seconds(1)) {
            runtime_logging::Log(runtime_logging::Level::kInfo,
                "[hiking depth] " + receiver_.Status().dump(), false);
            reported_ = now;
        }
    }
    void OnAction(std::vector<double> &action) override {
        if (action.size() != 29)
            throw std::runtime_error("Hiking invalid action dimensions");
        for (int i = 0; i < 29; ++i)
            previous_[i] = action[i];
    }
    bool ReadyForPolicySwitch() const override {
        return std::abs(command_[0]) + std::abs(command_[1]) + std::abs(command_[2]) < .01;
    }
    void RequestInferenceTermination() override { encoder_.RequestInferenceTermination(); }

    private:
    Config::HikingConfig cfg_;
    perception::Receiver receiver_;
    rl_policy::PolicyExecutor encoder_;
    Eigen::VectorXf depth_obs_;
    std::vector<int> map_;
    std::vector<float> signs_, defaults_, previous_, history_, actor_input_;
    std::vector<double> physical_defaults_, action_scale_, action_clip_;
    std::vector<double> latent_;
    std::array<double, 3> command_ = {};
    bool first_ = true;
    double last_time_ = -1;
    std::chrono::steady_clock::time_point reported_{};
};
} // namespace
std::unique_ptr<PolicyAdapter>
CreateHikingPolicyAdapter(const Config &cfg, const rl_policy::PolicyExecutorConfig &policy) {
    return std::make_unique<Hiking>(cfg, policy);
}
} // namespace behavior_manager::policy_adapter
