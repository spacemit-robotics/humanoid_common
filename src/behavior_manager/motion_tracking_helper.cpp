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

// G1 默认腰关节索引（waist yaw / roll / pitch），与 mjlab State_Mimic.cpp 一致
// 未来支持其他机型时可扩为配置项
constexpr int kWaistYawJoint = 12;
constexpr int kWaistRollJoint = 13;
constexpr int kWaistPitchJoint = 14;

// npz body_pos_w / body_quat_w 中 anchor body 的索引
// 来源：unitree_rl_mjlab/src/assets/robots/unitree_g1/xmls/g1.xml MJCF body 声明顺序
// （IsaacLab csv_to_npz.py 调用 robot.data.body_pos_w[0, :]，即 articulation 的全部 30 个 body）：
//   0=pelvis, 1-6=左腿(hip_pitch/roll/yaw, knee, ankle_pitch/roll),
//   7-12=右腿(同左), 13=waist_yaw_link, 14=waist_roll_link,
//   15=torso_link,  ← anchor_body_name="torso_link" 对应索引 15
//   16-22=左臂(shoulder_pitch/roll/yaw, elbow, wrist_roll/pitch/yaw),
//   23-29=右臂(同左)
constexpr int kAnchorBodyIndex = 15;

}  // namespace

// ============================================================
// torso_quat / yaw 提取（工具函数）
// ============================================================

Eigen::Quaternionf MotionTrackingHelper::ComputeTorsoQuat(
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

void MotionTrackingHelper::Load(const std::string &npz_path, double motion_fps) {
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

    if (kAnchorBodyIndex >= num_bodies_pos || kAnchorBodyIndex >= num_bodies_quat) {
        throw std::runtime_error(
            "[MotionTrackingHelper] npz body 维度不足，无法定位 anchor (torso_link, idx=" +
            std::to_string(kAnchorBodyIndex) + ")，body_pos_w shape[1]=" +
            std::to_string(num_bodies_pos));
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

        // anchor body 在 body_names 中位于索引 kAnchorBodyIndex (torso_link for G1)
        // 训练侧 anchor_body_name="torso_link"，需取对应 body 的世界系 pose（不是 body[0] pelvis）
        Eigen::Vector3f anchor_p = Eigen::Vector3f::Map(
            body_pos_w.data<float>() + i * body_stride_pos + kAnchorBodyIndex * 3);
        ref_root_pos_.push_back(anchor_p);

        const float *qd =
            body_quat_w.data<float>() + i * body_stride_quat + kAnchorBodyIndex * 4;
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

    Eigen::Quaternionf robot_torso = ComputeTorsoQuat(
        robot_root_quat, waist_yaw_q, waist_roll_q, waist_pitch_q);

    // 参考第 0 帧的 torso quat：直接用 ref_root_quat_[0]（即 npz body[kAnchorBodyIndex]=torso 的 quat）
    Eigen::Quaternionf ref_torso = ref_root_quat_[0];

    // init_quat = robot_yaw × ref_yaw^(-1)
    // 后续每帧把"参考 motion 在世界系下的朝向"通过 init_quat 旋转到"机器人 yaw"对齐
    init_quat_ = YawQuaternion(robot_torso) * YawQuaternion(ref_torso).conjugate();
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
    //   A = robot_anchor (torso) world pose
    //   B = motion_anchor (ref torso) world pose, 经 init_quat 对齐后
    //
    // 公式主源：unitree_rl_mjlab/src/tasks/tracking/mdp/observations.py:18-41

    // 机器人 anchor (torso) 世界系位姿
    Eigen::Quaternionf robot_root_q(static_cast<float>(robot.base_quat[0]),
                                    static_cast<float>(robot.base_quat[1]),
                                    static_cast<float>(robot.base_quat[2]),
                                    static_cast<float>(robot.base_quat[3]));
    robot_root_q.normalize();
    Eigen::Vector3f robot_anchor_pos(static_cast<float>(robot.base_pos[0]),
                                    static_cast<float>(robot.base_pos[1]),
                                    static_cast<float>(robot.base_pos[2]));
    Eigen::Quaternionf robot_anchor_q = ComputeTorsoQuat(
        robot_root_q, waist_yaw_q, waist_roll_q, waist_pitch_q);

    // 参考 anchor (ref torso) 世界系位姿
    // ref_root_pos_ / ref_root_quat_ 已经从 npz body[kAnchorBodyIndex]=torso_link 取出，直接使用，
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
