/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file motion_planner.h
 * @brief 参考动作规划器的扩展接口与类型注册表
 *
 * 规划器为跟踪类策略提供参考动作。本文件只定义规划器与跟踪策略之间的
 * 通用约定：输出格式、生命周期接口和按 type 注册的工厂。具体规划器在自己
 * 的源文件中实现接口并注册类型，机型 YAML 通过 type 选用，见同目录 README。
 */

#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "robot_base.h"

namespace behavior_manager {
namespace motion_planner {

/** @brief 规划器类别，决定宿主如何调度 */
enum class Category : uint8_t {
    REALTIME = 0,  ///< 跟踪策略运行期间持续输出参考
    OFFLINE = 1,   ///< 按请求一次性生成有限长度的参考动作
};

/** @brief 实时规划器产生参考的方式；只用于登记信息和日志 */
enum class Kind : uint8_t {
    MODEL = 0,          ///< 模型推理直接输出参考轨迹
    BEHAVIOR_TREE = 1,  ///< 行为树等决策逻辑，在已登记的参考动作之间选择
};

const char *CategoryName(Category category);
const char *KindName(Kind kind);

/**
 * @brief 规划器输出的显式参考轨迹
 *
 * 关节顺序与 RobotData::joint_pos 相同，跟踪策略按自己的模型关节顺序和
 * 未来帧窗口取用。各数组按帧连续存放。
 */
struct ReferenceTrajectory {
    enum class Frame : uint8_t {
        WORLD = 0,  ///< 根位姿与 RobotData::base_pos / base_quat 处于同一坐标系
        LOCAL = 1,  ///< 轨迹自带坐标系；跟踪策略起播时把首帧朝向对齐到机器人
    };

    int num_dof = 0;    ///< 每帧关节数
    double fps = 50.0;  ///< 帧率 (Hz)
    /// 实时规划器：第 0 帧对应的 RL 时间，超出末帧后保持末帧。离线规划器的输出由
    /// 跟踪策略决定起播时刻并只播放一遍，该字段被忽略。
    double start_time_s = 0.0;
    Frame frame = Frame::WORLD;
    std::vector<float> joint_pos;  ///< 帧数 × num_dof (rad)
    std::vector<float> joint_vel;  ///< 帧数 × num_dof (rad/s)
    std::vector<float> root_pos;   ///< 帧数 × 3 (m)
    std::vector<float> root_quat;  ///< 帧数 × 4 (w, x, y, z)

    std::size_t FrameCount() const {
        return num_dof > 0 ? joint_pos.size() / static_cast<std::size_t>(num_dof) : 0;
    }

    /** @brief 首帧到末帧的时长 (s)。 */
    double Duration() const {
        const std::size_t frames = FrameCount();
        return frames > 1 && fps > 0.0 ? static_cast<double>(frames - 1) / fps : 0.0;
    }
};

/**
 * @brief 规划器的一次发布
 *
 * trajectory 与 action 二选一：模型类规划器给出显式轨迹；决策类规划器给出
 * 跟踪策略动作目录中的参考动作名，由跟踪策略负责加载、过渡和播放。每个新的
 * 发布对象都是一次新的指令：给出 action 时跟踪策略从该动作的首帧重新起播。
 */
struct PlannerOutput {
    ReferenceTrajectory trajectory;
    std::string action;
};

/** @brief 操作端给实时规划器的运动指令，已按该规划器的 command.limits 限幅 */
struct MotionCommand {
    float vx = 0.0F;  ///< 前进速度 (m/s)
    float vy = 0.0F;  ///< 横向速度 (m/s)
    float wz = 0.0F;  ///< 转向角速度 (rad/s)
};

/** @brief 跟踪策略对规划器最近一次发布的播放情况 */
struct PlaybackState {
    bool active = false;    ///< 跟踪策略正在跟随规划器最近一次发布
    float progress = 0.0F;  ///< 具名动作或离线参考的播放进度 [0, 1]
    bool finished = false;  ///< 具名动作（非循环）或离线参考已播放到末帧
};

/** @brief 每个跟踪步提交给实时规划器的输入；引用成员只在本次调用期间有效 */
struct PlannerInput {
    const robot_base::RobotData &robot;
    MotionCommand command;
    double time_s = 0.0;  ///< RL 时间 (s)，与 ReferenceTrajectory::start_time_s 同一时间轴
    PlaybackState playback;
    const std::string &argument;     ///< 最近一次 START 请求附带的文本，可为空
    uint64_t argument_sequence = 0;  ///< argument 每更新一次加一
    bool stopping = false;  ///< 退出规划器时从当前姿态回到待命
};

/** @brief 检查一次发布的维度和数值；不合法时返回原因，合法时返回空串。 */
std::string ValidateOutput(const PlannerOutput &output, int num_dof);

/** @brief 实时规划器的统计，供界面显示 */
struct PlannerStats {
    uint64_t plan_count = 0;   ///< 启动后完成的规划次数
    double last_plan_ms = 0.0;  ///< 最近一次规划耗时 (ms)
};

/** @brief 规划器基类；实现 RealtimePlanner 或 OfflinePlanner 之一 */
class Planner {
public:
    virtual ~Planner() = default;
};

/**
 * @brief 实时规划器
 *
 * Start / Stop 在宿主的后台线程调用，可以阻塞。Update / Output / Idle / Stats
 * 在跟踪策略的推理线程每步调用，不得阻塞、不得等待低优先级线程持有的锁；
 * 耗时的规划放在规划器自己的工作线程，或保证单步计算在微秒级完成。
 */
class RealtimePlanner : public Planner {
public:
    /** @brief 以机器人当前状态为起点开始规划；之后的输出都落在 time_s 所在的 RL 时间轴上。 */
    virtual void Start(const robot_base::RobotData &robot, double time_s) = 0;

    /** @brief 提交最新的机器人状态、运动指令、播放情况和 RL 时间。 */
    virtual void Update(const PlannerInput &input) = 0;

    /**
     * @brief 最近一次发布；尚未发布时为空
     *
     * 内容不变时返回同一个对象，宿主据此判断是否有新的发布。规划失败时抛出
     * 异常，宿主随即交还待命参考并上报 FAILED。
     */
    virtual std::shared_ptr<const PlannerOutput> Output() = 0;

    /**
     * @brief 可以交还待命参考
     *
     * 停止时宿主持续提交零运动指令，直到本函数返回 true：已发布的参考处于
     * 静止（轨迹）或收尾动作已播放完毕（具名动作），且没有待处理的指令。
     */
    virtual bool Idle() const { return true; }

    /** @brief 停止规划并回收工作线程；之后可以再次 Start。 */
    virtual void Stop() {}

    virtual PlannerStats Stats() const { return {}; }
};

/** @brief 离线生成请求；引用成员只在 Generate 调用期间有效 */
struct GenerationRequest {
    const std::string &argument;        ///< 操作端随 START 请求传入的任务描述
    const robot_base::RobotData &robot;  ///< 发起请求时的机器人状态
    const std::atomic<bool> &cancelled;  ///< 置位表示宿主已放弃本次生成，应尽快返回或抛出异常
    /// 可选调用：上报生成进度 [0, 1]，供界面显示；不调用时界面只显示正在生成
    std::function<void(float)> report_progress;
};

/**
 * @brief 离线规划器
 *
 * Generate 在宿主的后台线程调用，可以长时间阻塞（本地模型推理或访问远端
 * 生成服务）；生成期间跟踪策略保持待命参考。返回的轨迹只播放一遍，随后
 * 跟踪策略回到待命参考，末帧姿态应接近待命姿态。
 */
class OfflinePlanner : public Planner {
public:
    /** @brief 生成一段有限长度的参考或选定一个具名动作；失败时抛出异常。 */
    virtual PlannerOutput Generate(const GenerationRequest &request) = 0;
};

/** @brief 创建规划器实例时提供的上下文 */
struct PlannerContext {
    std::string name;         ///< 机型 YAML 中的规划器名
    std::string yaml_path;    ///< 机型 YAML 路径
    std::string config_base;  ///< 该规划器的配置节点，如 motion_planners.<name>
    std::string robot_dir;    ///< 机型资源根目录，用于解析相对路径
    int num_dof = 0;          ///< 机器人关节数，即输出轨迹每帧的关节数
    double reference_fps = 50.0;  ///< 跟踪策略取用参考的帧率
    robot_base::ThreadLoop thread =
        robot_base::ThreadLoop::Background("motion_planner");  ///< 规划器工作线程配置
    std::function<void()> on_warmup;  ///< 可选：模型加载完成、开始预热时调用，用于上报进度
};

using PlannerFactory = std::function<std::unique_ptr<Planner>(const PlannerContext &)>;

/** @brief 一个可由 YAML type 选用的规划器类型 */
struct PlannerType {
    Category category = Category::REALTIME;
    Kind kind = Kind::MODEL;
    PlannerFactory factory;  ///< 在宿主后台线程调用；返回对象须实现 category 对应的接口
};

/** @brief 注册规划器类型；type 为空、重复或缺少 factory 时抛出异常。 */
void RegisterPlannerType(const std::string &type, PlannerType entry);

/** @brief 查找已注册的类型；未注册时返回空指针。 */
const PlannerType *FindPlannerType(const std::string &type);

/** @brief 已注册的类型名，按字典序排列。 */
std::vector<std::string> RegisteredPlannerTypes();

/** @brief 在源文件作用域定义一个该类型的对象，即可在库加载时完成注册 */
struct PlannerTypeRegistration {
    PlannerTypeRegistration(const std::string &type, PlannerType entry) {
        RegisterPlannerType(type, std::move(entry));
    }
};

}  // namespace motion_planner
}  // namespace behavior_manager

#endif  // MOTION_PLANNER_H
