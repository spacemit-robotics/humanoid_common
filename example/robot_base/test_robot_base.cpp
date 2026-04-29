/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_robot_base.cpp
 * @brief robot_base 接口使用示例 && 完整测试
 *
 * 演示 robot_base 所有对外接口的使用方法：
 * - YamlFile::Load / Read<T> / ToAbsPath
 * - RobotData::FromYaml / Create / UpdateImuData / Reset / IsValid
 * - ControlCmd / Command 结构体赋值与使用
 * - QuatToRpy / RpyToQuat / NormalizeQuat 工具函数
 * - ThreadLoop 线程管理：后台线程 + 主线程调度配置
 *
 * 用法: ./test_robot_base [yaml配置文件路径]
 */

#include <chrono>
#include <cmath>
#include <iostream>
#include <string>

#include "robot_base.h"

using robot_base::Command;
using robot_base::ControlCmd;
using robot_base::NormalizeQuat;
using robot_base::QuatToRpy;
using robot_base::RobotData;
using robot_base::RpyToQuat;
using robot_base::ThreadLoop;
using robot_base::YamlFile;

int main(int argc, char *argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "  robot_base 测试" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        // ========== YamlFile 接口（需要 YAML 文件）==========
        if (argc >= 2) {
            std::string yaml_path = argv[1];
            std::cout << "配置文件: " << yaml_path << std::endl;

            std::cout << "\n--- YamlFile 接口 ---" << std::endl;
            YamlFile yaml = YamlFile::Load(yaml_path);
            std::cout << "YamlFile::Load() 成功" << std::endl;

            auto num_dof = yaml.Read<int>("robot_base.num_dof");
            auto name = yaml.Read<std::string>("robot_base.name");
            auto robot_dir = yaml.Read<std::string>("robot_base.robot_dir");
            auto missing = yaml.Read<int>("nonexistent.path");

            if (!num_dof || !name || !robot_dir) {
                std::cerr << "[错误] 读取必填字段失败\n";
                return 1;
            }
            std::cout << "Read<int>(\"robot_base.num_dof\")    = " << *num_dof << std::endl;
            std::cout << "Read<string>(\"robot_base.name\")    = " << *name << std::endl;
            std::cout << "Read<string>(\"robot_base.robot_dir\") = " << *robot_dir << std::endl;
            std::cout << "ToAbsPath(robot_dir)               = "
                << yaml.ToAbsPath(*robot_dir) << std::endl;
            std::cout << "Read<int>(\"nonexistent.path\")      = "
                << (missing.has_value() ? std::to_string(*missing) : "nullopt（符合预期）")
                << std::endl;

            // ========== RobotData::FromYaml ==========
            std::cout << "\n--- RobotData::FromYaml ---" << std::endl;
            RobotData from_yaml = RobotData::FromYaml(yaml_path);
            std::cout << "name=" << from_yaml.name << ", num_dof=" << from_yaml.num_dof
                << ", joint_pos.size()=" << from_yaml.joint_pos.size()
                << ", joint_vel.size()=" << from_yaml.joint_vel.size() << std::endl;
            if (!from_yaml.IsValid()) {
                std::cerr << "[错误] RobotData::FromYaml() 返回无效状态\n";
                return 1;
            }
            std::cout << "IsValid() = true" << std::endl;

            // ========== ThreadLoop::FromYaml ==========
            std::cout << "\n--- ThreadLoop::FromYaml ---" << std::endl;
            ThreadLoop tl = ThreadLoop::FromYaml(yaml, "rl_infer");
            if (tl.name != "rl_infer" || tl.IsRunning()) {
                std::cerr << "[错误] FromYaml 字段或状态异常\n";
                return 1;
            }
            std::cout << "FromYaml: name=" << tl.name << ", cpu_id=" << tl.cpu_id
                << ", sched=" << tl.sched << ", priority=" << tl.priority << std::endl;
            std::cout << "ThreadLoop::FromYaml ✓" << std::endl;
        } else {
            std::cout << "\n提示: 提供 YAML 文件路径可运行 YamlFile 和 RobotData::FromYaml 测试"
                << std::endl;
            std::cout << "用法: " << argv[0] << " <yaml配置文件路径>" << std::endl;
        }

        // ========== RobotData::Create ==========
        std::cout << "\n--- RobotData::Create ---" << std::endl;
        RobotData state = RobotData::Create(29);
        std::cout << "num_dof=" << state.num_dof
            << ", joint_pos.size()=" << state.joint_pos.size()
            << ", IsValid()=" << state.IsValid() << std::endl;

        // ========== RobotData 数据赋值 ==========
        std::cout << "\n--- RobotData 数据赋值 ---" << std::endl;
        state.base_pos = {0.0, 0.0, 0.8};
        state.base_quat = {0.9239, 0.3827, 0.0, 0.0};  // 45° roll
        state.base_vel = {0.0, 0.0, 0.0, 0.1, 0.2, 0.3};
        for (int i = 0; i < state.num_dof; ++i) {
            state.joint_pos[i] = 0.1 * i;
            state.joint_vel[i] = 0.05 * i;
        }
        state.time = 1.234;
        std::cout << "base_pos=(" << state.base_pos[0] << "," << state.base_pos[1] << ","
            << state.base_pos[2] << "), time=" << state.time << std::endl;

        // ========== UpdateImuData ==========
        std::cout << "\n--- UpdateImuData ---" << std::endl;
        state.UpdateImuData();
        std::cout << "rpy=(" << state.rpy[0] << "," << state.rpy[1] << "," << state.rpy[2]
            << ") rad, gyro=(" << state.gyro[0] << "," << state.gyro[1] << ","
            << state.gyro[2] << ") rad/s" << std::endl;
        if (std::abs(state.rpy[0] - 0.785398) > 1e-4) {
            std::cerr << "[错误] rpy[0] 期望 0.785398，实际 " << state.rpy[0] << "\n";
            return 1;
        }
        std::cout << "rpy[0] ≈ π/4 ✓" << std::endl;

        // ========== Reset ==========
        std::cout << "\n--- Reset ---" << std::endl;
        state.Reset();
        std::cout << "Reset() 后: base_pos=(" << state.base_pos[0] << "," << state.base_pos[1]
            << "," << state.base_pos[2] << "), time=" << state.time
            << ", num_dof=" << state.num_dof << "（保持不变）" << std::endl;

        // ========== 四元数工具函数 ==========
        std::cout << "\n--- QuatToRpy / RpyToQuat / NormalizeQuat ---" << std::endl;
        std::array<double, 4> quat = {0.9239, 0.3827, 0.0, 0.0};
        std::array<double, 3> rpy;
        QuatToRpy(quat, rpy);
        std::cout << "QuatToRpy: rpy=(" << rpy[0] << "," << rpy[1] << "," << rpy[2] << ") rad"
            << std::endl;

        std::array<double, 4> quat_back;
        RpyToQuat(rpy, quat_back);
        std::cout << "RpyToQuat: quat=(" << quat_back[0] << "," << quat_back[1] << ","
            << quat_back[2] << "," << quat_back[3] << ")" << std::endl;
        if (std::abs(quat[0] - quat_back[0]) > 1e-4 || std::abs(quat[1] - quat_back[1]) > 1e-4) {
            std::cerr << "[错误] 四元数往返转换不一致\n";
            return 1;
        }
        std::cout << "往返转换一致 ✓" << std::endl;

        std::array<double, 4> quat_unnorm = {2.0, 0.0, 0.0, 0.0};
        NormalizeQuat(quat_unnorm);
        std::cout << "NormalizeQuat({2,0,0,0}): (" << quat_unnorm[0] << "," << quat_unnorm[1]
            << "," << quat_unnorm[2] << "," << quat_unnorm[3] << ")" << std::endl;

        // ========== ControlCmd 结构体 ==========
        std::cout << "\n--- ControlCmd 赋值与使用 ---" << std::endl;
        ControlCmd ctrl_cmd;
        ctrl_cmd.enable = true;
        ctrl_cmd.target_pos.resize(29, 0.0);
        ctrl_cmd.target_vel.resize(29, 0.0);
        ctrl_cmd.kp.resize(29, 100.0);
        ctrl_cmd.kd.resize(29, 2.0);
        ctrl_cmd.target_pos[3] = 0.3;  // 第 4 关节目标位置
        ctrl_cmd.kp[0] = 150.0;        // 第 1 关节加大刚度
        std::cout << "enable=" << ctrl_cmd.enable << ", target_pos[3]=" << ctrl_cmd.target_pos[3]
            << ", kp[0]=" << ctrl_cmd.kp[0] << ", kd.size()=" << ctrl_cmd.kd.size()
            << std::endl;

        // 使能清零（进入 POWER_OFF / DAMP 时的典型操作）
        ctrl_cmd.enable = false;
        std::fill(ctrl_cmd.kp.begin(), ctrl_cmd.kp.end(), 0.0);
        std::cout << "清零后: enable=" << ctrl_cmd.enable << ", kp[0]=" << ctrl_cmd.kp[0]
            << std::endl;

        // ========== Command 结构体 ==========
        std::cout << "\n--- Command 赋值与使用 ---" << std::endl;
        Command cmd;
        cmd.key = 1;    // 切换到 DAMP
        cmd.vx = 0.5f;  // 前进 0.5 m/s
        cmd.vy = 0.0f;
        cmd.wz = 0.1f;                 // 旋转 0.1 rad/s
        cmd.switch_policy = "motion";  // 请求切换策略
        std::cout << "key=" << cmd.key << ", vx=" << cmd.vx << ", vy=" << cmd.vy
            << ", wz=" << cmd.wz << ", switch_policy=" << cmd.switch_policy << std::endl;

        // 清零（每帧 step 后通常将瞬时命令清零）
        cmd.key = 0;
        cmd.switch_policy = "";
        std::cout << "清零后: key=" << cmd.key << ", switch_policy=\"" << cmd.switch_policy
            << "\"" << std::endl;

        // ========== ThreadLoop：后台线程 ==========
        std::cout << "\n--- ThreadLoop 后台线程 ---" << std::endl;
        {
            std::atomic<int> count{0};
            ThreadLoop loop;
            loop.name = "test_worker";
            loop.cpu_id = -1;
            loop.sched = "other";
            loop.priority = 0;
            loop.Start([&count] {
                count.fetch_add(1);
                return count.load() < 3;  // 执行 3 次后退出
            });
            // 等待线程自然退出后再 join（Stop() 会设 running_=false 并 join）
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            loop.Stop();
            std::cout << "后台线程执行次数: " << count.load() << "（期望 3）" << std::endl;
            if (count.load() != 3) {
                std::cerr << "[错误] ThreadLoop 执行次数不符\n";
                return 1;
            }
            std::cout << "ThreadLoop 后台线程 ✓" << std::endl;
        }

        // ========== ThreadLoop 拷贝：仅复制配置字段 ==========
        std::cout << "\n--- ThreadLoop 拷贝构造 / 赋值 ---" << std::endl;
        {
            ThreadLoop src;
            src.name = "src_thread";
            src.cpu_id = 2;
            src.sched = "fifo";
            src.priority = 80;

            // 拷贝构造：新对象继承配置，处于未启动状态
            ThreadLoop dst(src);
            if (dst.name != src.name || dst.cpu_id != src.cpu_id || dst.sched != src.sched ||
                dst.priority != src.priority || dst.IsRunning()) {
                std::cerr << "[错误] ThreadLoop 拷贝构造字段不一致或意外启动\n";
                return 1;
            }

            // 赋值：覆盖配置，不影响线程状态
            ThreadLoop dst2;
            dst2 = src;
            if (dst2.name != src.name || dst2.cpu_id != src.cpu_id) {
                std::cerr << "[错误] ThreadLoop 赋值字段不一致\n";
                return 1;
            }
            std::cout << "ThreadLoop 拷贝（配置字段复制，未启动状态）✓" << std::endl;
        }

        // ========== ThreadLoop::Apply()：配置调用线程 ==========
        std::cout << "\n--- ThreadLoop::Apply() 配置主线程 ---" << std::endl;
        {
            ThreadLoop main_cfg;
            main_cfg.name = "test_main";
            main_cfg.cpu_id = -1;  // 不绑定
            main_cfg.sched = "other";
            main_cfg.priority = 0;
            main_cfg.Apply();  // 配置调用线程（本 main 线程）自身
            std::cout << "Apply() 完成（实机可设 sched=fifo 并配合 root 权限启用实时调度）"
                << std::endl;
        }

        std::cout << "\n========================================" << std::endl;
        std::cout << "  测试完成 ✓" << std::endl;
        std::cout << "========================================" << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[错误] " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
