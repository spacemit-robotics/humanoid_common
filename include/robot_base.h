/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file robot_base.h
 * @brief 机器人基础状态定义（所有模块共用）
 *
 * 提供统一的机器人状态数据结构和工具函数。
 * simulation、hardware、behavior_manager、application 等模块都使用此结构。
 */
#ifndef ROBOT_BASE_H
#define ROBOT_BASE_H

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace robot_base {

/**
 * @brief YAML 配置读取入口
 *
 * 封装 yaml 文件加载与按路径读取能力，供各模块统一读取配置。
 * 路径使用 "." 分隔层级，序列下标使用 "[i]"，例如：
 * - "rl_policy.onnx_infer.default_policy"
 * - "rl_policy.onnx_infer.policies.motion.observation.segments[0].terms"
 */
class YamlFile {
public:
    /**
     * @brief 加载 YAML 文件
     * @param yaml_path YAML 文件路径（相对路径会转绝对路径）
     * @return 已加载的 YamlFile
     * @throws std::runtime_error 文件不存在或解析失败时抛出异常
     */
    static YamlFile Load(const std::string &yaml_path);

    /**
     * @brief 读取指定类型的配置
     * @tparam T 配置数据类型，支持以下类型：
     *           int, double, bool, std::string,
     *           std::vector<int>, std::vector<double>, std::vector<std::string>
     * @param path 配置路径，使用 "." 分隔层级，"[i]" 访问序列元素
     *             例如："robot_base.num_dof"、"policies.motion.obs[0].terms"
     * @return 路径存在且类型匹配时返回 std::optional<T>，否则返回 std::nullopt
     */
    template <typename T>
    std::optional<T> Read(const std::string &path) const;

    /**
     * @brief 基于 YAML 文件所在目录获取绝对路径
     * @param path 相对路径或绝对路径
     */
    std::string ToAbsPath(const std::string &path) const;

private:
    struct Impl;
    std::shared_ptr<Impl> impl_;
    std::string yaml_path_;
    std::string yaml_dir_;
};

/**
 * @brief 机器人基础状态
 *
 * 统一的数据结构，用于在各模块间传递机器人状态。
 * 包含机器人物理状态和控制算法所需的所有输入数据。
 */
struct RobotData {
    // ==================== 基础信息 ====================
    std::string name;  // 机器人名称（如 "g1", "qinglong"）
    int num_dof = 0;   // 控制自由度数量

    // ==================== 基座状态 ====================
    std::array<double, 3> base_pos = {0, 0, 0};      // 世界坐标系位置 (m)
    std::array<double, 4> base_quat = {1, 0, 0, 0};  // 姿态四元数 (w, x, y, z)
    std::array<double, 6> base_vel = {0};            // 线速度+角速度 (m/s, rad/s)

    // ==================== IMU 数据（控制算法输入）====================
    std::array<double, 3> rpy = {0, 0, 0};   // 欧拉角 (roll, pitch, yaw) (rad)
    std::array<double, 3> gyro = {0, 0, 0};  // 角速度 (rad/s)

    // ==================== 关节状态 ====================
    std::vector<double> joint_pos;  // 关节位置 (rad)，大小 = num_dof
    std::vector<double> joint_vel;  // 关节速度 (rad/s)，大小 = num_dof

    // ==================== 时间戳 ====================
    double time = 0.0;  // 时间戳 (s)

    // ==================== 工厂方法 ====================

    /**
     * @brief 从 YAML 文件创建状态结构
     * @param yaml_path YAML 配置文件路径
     * @return RobotBase 实例，num_dof 已设置，关节向量已初始化
     *
     * 只读取 YAML 中的 robot_base.num_dof 字段，用于初始化向量大小。
     *
     * @throws std::runtime_error 如果文件不存在或缺少必填字段
     */
    static RobotData FromYaml(const std::string &yaml_path);

    /**
     * @brief 从 num_dof 创建状态结构
     * @param num_dof 控制自由度数量
     * @return RobotData 实例，关节向量已初始化
     */
    static RobotData Create(int num_dof);

    /**
     * @brief 初始化关节向量大小（根据 num_dof）
     *
     * 内部方法，通常不需要手动调用。
     */
    void InitJointVectors();

    // ==================== 数据处理方法 ====================

    /**
     * @brief 从 base_quat 和 base_vel 更新 rpy 和 gyro
     *
     * 自动计算：
     *   - rpy: 从 base_quat（四元数）计算欧拉角
     *   - gyro: 从 base_vel[3:6] 提取角速度
     *
     * 通常在填充完 base_quat 和 base_vel 后调用。
     */
    void UpdateImuData();

    /**
     * @brief 重置状态到初始值（保持 num_dof 不变）
     */
    void Reset();

    /**
     * @brief 验证状态数据有效性
     * @return true 如果状态数据有效（向量大小匹配 num_dof）
     */
    bool IsValid() const;
};

// ==================== 工具函数 ====================

/**
 * @brief 四元数转欧拉角 (roll, pitch, yaw)
 * @param quat 四元数 [w, x, y, z]
 * @param rpy 输出欧拉角 [roll, pitch, yaw] (rad)
 */
void QuatToRpy(const std::array<double, 4> &quat, std::array<double, 3> &rpy);

/**
 * @brief 欧拉角转四元数
 * @param rpy 欧拉角 [roll, pitch, yaw] (rad)
 * @param quat 输出四元数 [w, x, y, z]
 */
void RpyToQuat(const std::array<double, 3> &rpy, std::array<double, 4> &quat);

/**
 * @brief 四元数归一化
 * @param quat 四元数 [w, x, y, z]（会被修改）
 */
void NormalizeQuat(std::array<double, 4> &quat);

/**
 * @brief 行为控制命令
 *
 * 统一的行为控制指令，用于 HMI → control → behavior_manager 全链路传递。
 */
struct Command {
    int key = 0;                ///< 状态切换指令（1=DAMP, 2=ZERO, 3=RL, -1=POWER_OFF）
    float vx = 0.0f;            ///< 前进速度 (m/s)
    float vy = 0.0f;            ///< 横向速度 (m/s)
    float wz = 0.0f;            ///< 旋转角速度 (rad/s)
    std::string switch_policy;  ///< 策略切换请求，空字符串表示无切换
};

/**
 * @brief 控制命令
 *
 * 用于控制器输出的目标关节状态和 PD 参数。
 */
/**
 * @brief 控制模式（通用语义，独立于 behavior_manager FSM）
 *
 * 由 control 端在每帧 ControlCmd 中告知 driver 当前的整体控制语义，便于 driver
 * 据此调整自身行为（mujoco 悬挂、实机故障阈值/恢复策略、日志/遥测打点等）。
 */
enum class ControlMode : int8_t {
    POWER_OFF = 0,  ///< 完全失力
    DAMP = 1,       ///< 阻尼保持
    ZERO = 2,       ///< PD 锁位（回零或保持训练初始位）
    RL = 3,         ///< RL 策略动态控制
    SAFETY = 4      ///< 安全保护
};

struct ControlCmd {
    bool enable = false;             ///< 使能标志
    ControlMode mode = ControlMode::POWER_OFF;  ///< 控制模式（通用）
    std::vector<double> target_pos;  ///< 目标关节位置 (rad)
    std::vector<double> target_vel;  ///< 目标关节速度 (rad/s)
    std::vector<double> kp;          ///< 比例增益
    std::vector<double> kd;          ///< 微分增益
};

// ==================== 线程管理 ====================

/**
 * @brief 线程循环管理，支持 CPU 亲和性与实时调度配置
 *
 * 配置字段为 public，Start() 前直接赋值。
 * Apply() 可单独调用以配置当前线程（如 main 线程）。
 */
class ThreadLoop {
public:
    std::string name = "thread";  ///< 线程名（最长 15 字符）
    int cpu_id = -1;              ///< CPU 亲和性，-1 不绑定
    std::string sched = "other";  ///< 调度策略："other" | "fifo" | "rr"
    int priority = 0;             ///< 优先级（fifo/rr: 1~99；other 忽略）

    ThreadLoop() = default;
    ~ThreadLoop();

    /// 拷贝构造/赋值：仅复制配置字段（name/cpu_id/sched/priority），不复制线程状态
    ThreadLoop(const ThreadLoop &other)
        : name(other.name), cpu_id(other.cpu_id), sched(other.sched), priority(other.priority) {}
    ThreadLoop &operator=(const ThreadLoop &other) {
        name = other.name;
        cpu_id = other.cpu_id;
        sched = other.sched;
        priority = other.priority;
        return *this;
    }

    /**
     * @brief 从 YAML 加载指定线程的配置
     * @param yaml_file 已加载的 YamlFile
     * @param thread_name 线程名，对应 robot_base.threads.{thread_name} 下的配置项
     * @return 配置好的 ThreadLoop（未启动），缺失字段使用默认值
     */
    static ThreadLoop FromYaml(const YamlFile &yaml_file, const std::string &thread_name);

    /// 对调用线程应用当前配置（fifo/rr 需 root）；Start() 内部自动调用
    void Apply() const;

    /// 启动线程，循环执行 func，返回 false 时退出
    void Start(std::function<bool()> func);

    /// 停止线程（阻塞等待）
    void Stop();

    /// 线程是否正在运行
    bool IsRunning() const;

private:
    std::thread thread_;
    std::atomic<bool> running_{false};
};

}  // namespace robot_base

#endif  // ROBOT_BASE_H
