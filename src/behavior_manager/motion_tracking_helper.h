/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file motion_tracking_helper.h
 * @brief BeyondMimic 风格 tracking 策略的 motion 参考数据 + anchor 坐标系计算辅助类
 *
 * 职责：
 *   1. 从 .npz 加载 motion 参考数据（joint_pos / joint_vel / body_pos_w / body_quat_w）
 *   2. enter 时计算 yaw 对齐变换（init_quat），把参考 motion 的世界系朝向对齐到机器人当前朝向
 *   3. 每帧：
 *      - 按时间索引推进 frame
 *      - 拼装 motion_command (58D) = joint_pos[29] + joint_vel[29]
 *      - 计算 motion_anchor_pos_b (3D) / motion_anchor_ori_b (6D) ←
 *        参考 motion 的 anchor 表达在机器人 anchor 本体系下
 *
 * 公式主源：unitree_rl_mjlab/src/tasks/tracking/mdp/observations.py:18-41
 * cnpy 加载参考：unitree_rl_mjlab/deploy/robots/g1/include/State_Mimic.h
 */
#ifndef MOTION_TRACKING_HELPER_H
#define MOTION_TRACKING_HELPER_H

#include <Eigen/Dense>
#include <array>
#include <string>
#include <vector>

#include "robot_base.h"

namespace behavior_manager {

class MotionTrackingHelper {
public:
    MotionTrackingHelper() = default;
    ~MotionTrackingHelper() = default;

    /**
     * @brief 加载 npz 参考动作
     * @param npz_path npz 文件路径（绝对路径）
     * @param motion_fps 训练时 motion 帧率（默认 50 Hz）
     * @throws std::runtime_error 加载失败
     */
    void Load(const std::string &npz_path, double motion_fps = 50.0);

    /**
     * @brief 进入 tracking 状态时调用：算 yaw 对齐 init_quat，帧索引清零
     * @param robot 当前机器人状态（用于读取当前 anchor 朝向）
     * @param waist_yaw_q 腰部 yaw 关节当前角度（用于算 torso_link quat）
     * @param waist_roll_q 腰部 roll 关节当前角度
     * @param waist_pitch_q 腰部 pitch 关节当前角度
     * @param yaw_align 是否启用 yaw 对齐
     */
    void Reset(const robot_base::RobotData &robot,
                double waist_yaw_q,
                double waist_roll_q,
                double waist_pitch_q,
                bool yaw_align);

    /**
     * @brief 每帧推理前调用：刷新 frame index + 计算 anchor 误差
     * @param elapsed_s 自进入 tracking 状态以来的秒数
     * @param robot 当前机器人状态
     * @param waist_yaw_q 腰部 3 个关节当前角度（torso_link quat 计算用）
     * @param waist_roll_q
     * @param waist_pitch_q
     */
    void Update(double elapsed_s,
                const robot_base::RobotData &robot,
                double waist_yaw_q,
                double waist_roll_q,
                double waist_pitch_q);

    /// 当前帧 motion_command (58D = joint_pos[29] + joint_vel[29])
    const std::vector<float> &MotionCommand() const { return motion_command_; }

    /// 当前帧 motion_anchor_pos_b (3D，机器人 anchor 本体系下的参考 anchor 位置)
    const std::array<float, 3> &AnchorPosB() const { return anchor_pos_b_; }

    /// 当前帧 motion_anchor_ori_b (6D，旋转矩阵前两列展平)
    const std::array<float, 6> &AnchorOriB() const { return anchor_ori_b_; }

    /// motion 时长（秒）
    double Duration() const { return duration_; }

    /// 当前帧索引（debug 用）
    int Frame() const { return frame_; }

    /// 是否播完
    bool IsFinished(double elapsed_s) const { return elapsed_s >= duration_; }

private:
    /**
     * @brief 计算 torso_link 在世界系下的 quat
     *
     * 公式（mjlab State_Mimic.cpp:18-21）:
     *   torso_quat = root_quat × Rz(joint[waist_yaw]) × Rx(joint[waist_roll]) × Ry(joint[waist_pitch])
     */
    static Eigen::Quaternionf ComputeTorsoQuat(
        const Eigen::Quaternionf &root_quat,
        double waist_yaw_q,
        double waist_roll_q,
        double waist_pitch_q);

    /// 提取 quaternion 的 yaw 分量（保留绕 z 轴的旋转部分）
    static Eigen::Quaternionf YawQuaternion(const Eigen::Quaternionf &q);

    // ---- npz 加载的参考数据 ----
    std::vector<Eigen::VectorXf> ref_joint_pos_;     // (T, 29)
    std::vector<Eigen::VectorXf> ref_joint_vel_;     // (T, 29)
    std::vector<Eigen::Vector3f> ref_root_pos_;      // (T, 3)
    std::vector<Eigen::Quaternionf> ref_root_quat_;  // (T)，从 body_quat_w[i, 0] 取 root

    int num_frames_ = 0;
    double dt_ = 0.02;
    double duration_ = 0.0;
    int joint_dim_ = 29;

    // ---- enter 时算出的 yaw 对齐量 ----
    Eigen::Quaternionf init_quat_ = Eigen::Quaternionf::Identity();

    // ---- 每帧缓存（被 obs term 通过 SetCustomArray 读走）----
    int frame_ = 0;
    std::vector<float> motion_command_;       // 58D
    std::array<float, 3> anchor_pos_b_ = {};  // 3D
    std::array<float, 6> anchor_ori_b_ = {};  // 6D
};

}  // namespace behavior_manager

#endif  // MOTION_TRACKING_HELPER_H
