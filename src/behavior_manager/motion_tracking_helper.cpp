/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file motion_tracking_helper.cpp
 * @brief MotionTrackingHelper 实现
 */

#include "motion_tracking_helper.h"

#include <cnpy.h>

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace behavior_manager {

namespace {

// anchor body 在 npz body_pos_w / body_quat_w 中的索引由调用方经 anchor_body_index 传入（机型相关，在机型 yaml 配置）。

}  // namespace

// ============================================================
// anchor quat / yaw 提取（工具函数）
// ============================================================

Eigen::Quaternionf MotionTrackingHelper::ComputeAnchorQuat(
    const Eigen::Quaternionf &root_quat,
    double waist_yaw_q,
    double waist_roll_q,
    double waist_pitch_q) {
    // 公式：root_quat × Rz(yaw) × Rx(roll) × Ry(pitch)
    // 参考 unitree_rl_mjlab/deploy/robots/g1/src/State_Mimic.cpp:18-21
    Eigen::Quaternionf q = root_quat;
    q = q * Eigen::AngleAxisf(static_cast<float>(waist_yaw_q), Eigen::Vector3f::UnitZ());
    q = q * Eigen::AngleAxisf(static_cast<float>(waist_roll_q), Eigen::Vector3f::UnitX());
    q = q * Eigen::AngleAxisf(static_cast<float>(waist_pitch_q), Eigen::Vector3f::UnitY());
    return q.normalized();
}

Eigen::Quaternionf MotionTrackingHelper::YawQuaternion(const Eigen::Quaternionf &q) {
    // 提取四元数的 yaw 分量（绕 z 轴旋转部分）
    // q = [w, x, y, z]
    const float yaw = std::atan2(2.0f * (q.w() * q.z() + q.x() * q.y()),
                                1.0f - 2.0f * (q.y() * q.y() + q.z() * q.z()));
    Eigen::Quaternionf yq(std::cos(yaw * 0.5f), 0.0f, 0.0f, std::sin(yaw * 0.5f));
    return yq.normalized();
}

// ============================================================
// Load: cnpy 加载 npz
// ============================================================

void MotionTrackingHelper::Load(const std::string &npz_path, double motion_fps, int anchor_body_index) {
    std::cout << "[MotionTrackingHelper] 加载 npz: " << npz_path << std::endl;

    cnpy::npz_t npz = cnpy::npz_load(npz_path);

    auto check_key = [&](const std::string &key) {
        if (npz.find(key) == npz.end()) {
            throw std::runtime_error("[MotionTrackingHelper] npz 缺少必填字段: " + key);
        }
    };
    check_key("joint_pos");
    check_key("joint_vel");
    check_key("body_pos_w");
    check_key("body_quat_w");

    auto &joint_pos = npz["joint_pos"];    // [T, N_joints]
    auto &joint_vel = npz["joint_vel"];    // [T, N_joints]
    auto &body_pos_w = npz["body_pos_w"];  // [T, N_body, 3]
    auto &body_quat_w = npz["body_quat_w"];  // [T, N_body, 4]

    // 维度校验：各数组 ndim 及彼此维度一致性，不满足即 throw。
    auto require = [](bool ok, const std::string &msg) {
        if (!ok) {
            throw std::runtime_error("[MotionTrackingHelper] npz 维度非法: " + msg);
        }
    };
    require(joint_pos.shape.size() == 2, "joint_pos 应为 2 维 [T, N_joints]");
    require(joint_vel.shape.size() == 2, "joint_vel 应为 2 维 [T, N_joints]");
    require(body_pos_w.shape.size() == 3, "body_pos_w 应为 3 维 [T, N_body, 3]");
    require(body_quat_w.shape.size() == 3, "body_quat_w 应为 3 维 [T, N_body, 4]");
    require(joint_pos.shape[0] > 0 && joint_pos.shape[1] > 0, "joint_pos 帧数/关节数不能为 0");
    require(joint_vel.shape[0] == joint_pos.shape[0] && joint_vel.shape[1] == joint_pos.shape[1],
            "joint_vel 与 joint_pos 维度不一致");
    require(body_pos_w.shape[0] == joint_pos.shape[0], "body_pos_w 帧数与 joint_pos 不一致");
    require(body_quat_w.shape[0] == joint_pos.shape[0], "body_quat_w 帧数与 joint_pos 不一致");
    require(body_pos_w.shape[2] == 3, "body_pos_w 末维应为 3");
    require(body_quat_w.shape[2] == 4, "body_quat_w 末维应为 4");

    num_frames_ = static_cast<int>(joint_pos.shape[0]);
    joint_dim_ = static_cast<int>(joint_pos.shape[1]);
    if (motion_fps > 0.0) {
        dt_ = 1.0 / motion_fps;
    } else {
        dt_ = 0.02;
    }
    duration_ = num_frames_ * dt_;

    ref_joint_pos_.clear();
    ref_joint_vel_.clear();
    ref_root_pos_.clear();
    ref_root_quat_.clear();
    ref_joint_pos_.reserve(num_frames_);
    ref_joint_vel_.reserve(num_frames_);
    ref_root_pos_.reserve(num_frames_);
    ref_root_quat_.reserve(num_frames_);

    const int num_bodies_pos = static_cast<int>(body_pos_w.shape[1]);
    const int num_bodies_quat = static_cast<int>(body_quat_w.shape[1]);
    const size_t body_stride_pos = body_pos_w.shape[1] * body_pos_w.shape[2];   // N_body * 3
    const size_t body_stride_quat = body_quat_w.shape[1] * body_quat_w.shape[2];  // N_body * 4

    if (anchor_body_index < 0 || anchor_body_index >= num_bodies_pos ||
        anchor_body_index >= num_bodies_quat) {
        throw std::runtime_error(
            "[MotionTrackingHelper] anchor_body_index=" + std::to_string(anchor_body_index) +
            " 超出 npz body 数（body_pos_w shape[1]=" + std::to_string(num_bodies_pos) + "）");
    }

    for (int i = 0; i < num_frames_; ++i) {
        Eigen::VectorXf jp(joint_dim_);
        Eigen::VectorXf jv(joint_dim_);
        for (int j = 0; j < joint_dim_; ++j) {
            jp[j] = joint_pos.data<float>()[i * joint_dim_ + j];
            jv[j] = joint_vel.data<float>()[i * joint_dim_ + j];
        }
        ref_joint_pos_.push_back(std::move(jp));
        ref_joint_vel_.push_back(std::move(jv));

        // 取 anchor body 的世界系 pose（npz body 顺序中索引 anchor_body_index）
        Eigen::Vector3f anchor_p = Eigen::Vector3f::Map(
            body_pos_w.data<float>() + i * body_stride_pos + anchor_body_index * 3);
        ref_root_pos_.push_back(anchor_p);

        const float *qd =
            body_quat_w.data<float>() + i * body_stride_quat + anchor_body_index * 4;
        // npz 存储顺序：[w, x, y, z]
        Eigen::Quaternionf rq(qd[0], qd[1], qd[2], qd[3]);
        ref_root_quat_.push_back(rq.normalized());
    }

    motion_command_.assign(joint_dim_ * 2, 0.0f);

    std::cout << "[MotionTrackingHelper] 加载完成: " << num_frames_ << " 帧, dt=" << dt_
            << "s, duration=" << duration_ << "s, joint_dim=" << joint_dim_ << std::endl;
}

// ============================================================
// Reset: enter 时 yaw 对齐 + frame=0
// ============================================================

void MotionTrackingHelper::Reset(const robot_base::RobotData &robot,
                                double waist_yaw_q,
                                double waist_roll_q,
                                double waist_pitch_q,
                                bool yaw_align) {
    frame_ = 0;
    if (num_frames_ == 0) {
        init_quat_ = Eigen::Quaternionf::Identity();
        return;
    }

    if (!yaw_align) {
        init_quat_ = Eigen::Quaternionf::Identity();
        return;
    }

    // 机器人当前 root quat（[w,x,y,z]）
    Eigen::Quaternionf robot_root_quat(static_cast<float>(robot.base_quat[0]),
                                        static_cast<float>(robot.base_quat[1]),
                                        static_cast<float>(robot.base_quat[2]),
                                        static_cast<float>(robot.base_quat[3]));
    robot_root_quat.normalize();

    Eigen::Quaternionf robot_anchor = ComputeAnchorQuat(
        robot_root_quat, waist_yaw_q, waist_roll_q, waist_pitch_q);

    // 参考第 0 帧的 anchor quat：直接用 ref_root_quat_[0]（即 npz body[anchor_body_index] 的 quat）
    Eigen::Quaternionf ref_anchor = ref_root_quat_[0];

    // init_quat = robot_yaw × ref_yaw^(-1)
    // 后续每帧把"参考 motion 在世界系下的朝向"通过 init_quat 旋转到"机器人 yaw"对齐
    init_quat_ = YawQuaternion(robot_anchor) * YawQuaternion(ref_anchor).conjugate();
    init_quat_.normalize();

    std::cout << "[MotionTrackingHelper] Reset 完成" << std::endl;
}

// ============================================================
// Update: 每帧刷新 motion_command + anchor pos_b/ori_b
// ============================================================

void MotionTrackingHelper::Update(double elapsed_s,
                                const robot_base::RobotData &robot,
                                double waist_yaw_q,
                                double waist_roll_q,
                                double waist_pitch_q) {
    if (num_frames_ == 0) {
        return;
    }

    // 1. 推进帧索引（clamp 到末帧）
    double t = std::clamp(elapsed_s, 0.0, duration_);
    int f = static_cast<int>(std::floor(t / dt_));
    if (f >= num_frames_) {
        f = num_frames_ - 1;
    } else if (f < 0) {
        f = 0;
    }
    frame_ = f;

    // 2. 拼装 motion_command (58D = ref_joint_pos[29] + ref_joint_vel[29])
    const auto &jp = ref_joint_pos_[f];
    const auto &jv = ref_joint_vel_[f];
    for (int i = 0; i < joint_dim_; ++i) {
        motion_command_[i] = jp[i];
        motion_command_[joint_dim_ + i] = jv[i];
    }

    // 3. 算 anchor 相对位姿
    // 参考公式：subtract_frame_transforms(A_pos, A_quat, B_pos, B_quat)
    //   = (A_quat^(-1) × (B_pos - A_pos),  A_quat^(-1) × B_quat)
    // 这里：
    //   A = robot_anchor world pose
    //   B = motion_anchor world pose, 经 init_quat 对齐后
    //
    // 公式主源：unitree_rl_mjlab/src/tasks/tracking/mdp/observations.py:18-41

    // 机器人 anchor 世界系位姿
    Eigen::Quaternionf robot_root_q(static_cast<float>(robot.base_quat[0]),
                                    static_cast<float>(robot.base_quat[1]),
                                    static_cast<float>(robot.base_quat[2]),
                                    static_cast<float>(robot.base_quat[3]));
    robot_root_q.normalize();
    Eigen::Vector3f robot_anchor_pos(static_cast<float>(robot.base_pos[0]),
                                    static_cast<float>(robot.base_pos[1]),
                                    static_cast<float>(robot.base_pos[2]));
    Eigen::Quaternionf robot_anchor_q = ComputeAnchorQuat(
        robot_root_q, waist_yaw_q, waist_roll_q, waist_pitch_q);

    // 参考 anchor 世界系位姿
    // ref_root_pos_ / ref_root_quat_ 已经从 npz body[anchor_body_index] 取出，直接使用，
    // 不需要再叠加 waist 关节链（npz body_pos_w/body_quat_w 是 retargeting 时 FK 后的结果）
    Eigen::Vector3f ref_anchor_pos = ref_root_pos_[f];
    Eigen::Quaternionf ref_anchor_q = ref_root_quat_[f];

    // 应用 yaw 对齐变换
    Eigen::Vector3f motion_pos_aligned = init_quat_ * ref_anchor_pos;
    Eigen::Quaternionf motion_quat_aligned = init_quat_ * ref_anchor_q;
    motion_quat_aligned.normalize();

    // pos_b = robot_anchor_q^(-1) × (motion_pos_aligned - robot_anchor_pos)
    Eigen::Quaternionf robot_anchor_q_inv = robot_anchor_q.conjugate();
    Eigen::Vector3f delta_pos = motion_pos_aligned - robot_anchor_pos;
    Eigen::Vector3f pos_b = robot_anchor_q_inv * delta_pos;
    anchor_pos_b_[0] = pos_b.x();
    anchor_pos_b_[1] = pos_b.y();
    anchor_pos_b_[2] = pos_b.z();

    // ori_b = matrix_from_quat(robot_anchor_q^(-1) × motion_quat_aligned)[:, :2].flatten()
    Eigen::Quaternionf rel_q = robot_anchor_q_inv * motion_quat_aligned;
    rel_q.normalize();
    Eigen::Matrix3f R = rel_q.toRotationMatrix();
    // 前两列展平（行优先，跟训练侧 matrix_from_quat[..., :2].reshape 一致）
    anchor_ori_b_[0] = R(0, 0);
    anchor_ori_b_[1] = R(0, 1);
    anchor_ori_b_[2] = R(1, 0);
    anchor_ori_b_[3] = R(1, 1);
    anchor_ori_b_[4] = R(2, 0);
    anchor_ori_b_[5] = R(2, 1);
}

}  // namespace behavior_manager
