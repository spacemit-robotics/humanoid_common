/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file behavior_manager.cpp
 * @brief BehaviorManager 实现
 *
 * 封装 FSM + 状态注册 + 数据管理。
 * application 层仅需与此类交互。
 *
 * 配置：behavior_manager 命名空间
 */

#include "behavior_manager.h"  // 对外接口，位于 include/

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "behavior_fsm.h"      // 内部实现，位于 src/
#include "runtime_logger.h"
#include "state_factory.h"     // 内部实现，位于 src/
namespace behavior_manager {

const char *StateNameStr(StateName s) {
    switch (s) {
    case StateName::POWER_OFF:
        return "POWER_OFF";
    case StateName::DAMP:
        return "DAMP";
    case StateName::ZERO:
        return "ZERO";
    case StateName::RL:
        return "RL";
    case StateName::SAFETY:
        return "SAFETY";
    case StateName::HOME:
        return "HOME";
    default:
        return "UNKNOWN";
    }
}

namespace {

double MonotonicTimeSeconds() {
    return std::chrono::duration<double>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

struct SafetyMonitorConfig {
    double max_roll = 0.0;
    double max_pitch = 0.0;
    double max_angular_velocity = 0.0;
};

enum class SafetyMonitorFault : int32_t {
    kInvalidState = 1,
    kBodyAttitude = 2,
    kAngularVelocity = 3,
};

bool AllFinite(const std::vector<double> &values) {
    return std::all_of(values.begin(), values.end(),
        [](double value) { return std::isfinite(value); });
}

template <std::size_t Size>
bool AllFinite(const std::array<double, Size> &values) {
    return std::all_of(values.begin(), values.end(),
        [](double value) { return std::isfinite(value); });
}

double QuaternionNormSquared(const std::array<double, 4> &quaternion) {
    double norm_squared = 0.0;
    for (double value : quaternion) norm_squared += value * value;
    return norm_squared;
}

void ValidateJointVector(const std::vector<double> &values,
        int num_dof,
        const std::string &path) {
    if (static_cast<int>(values.size()) != num_dof) {
        throw std::runtime_error("[BehaviorManager] " + path + " 维度与 num_dof 不一致");
    }
    for (double value : values) {
        if (!std::isfinite(value)) {
            throw std::runtime_error("[BehaviorManager] " + path + " 包含非有限值");
        }
    }
}

void ValidateGainVector(const std::vector<double> &values,
                        int num_dof,
                        const std::string &path) {
    ValidateJointVector(values, num_dof, path);
    if (std::any_of(values.begin(), values.end(), [](double value) { return value < 0.0; })) {
        throw std::runtime_error("[BehaviorManager] " + path + " 包含负增益");
    }
}

ZeroTransitionConfig LoadZeroTransitionConfig(
        const robot_base::YamlFile &yaml, double move_duration_fallback) {
    const std::string base = "behavior_manager.zero";
    ZeroTransitionConfig config;
    config.move_duration = yaml.Read<double>(base + ".move_duration")
        .value_or(move_duration_fallback);
    config.position_tolerance = yaml.Read<double>(
        base + ".position_tolerance").value_or(0.15);
    config.velocity_tolerance = yaml.Read<double>(
        base + ".velocity_tolerance").value_or(0.10);
    config.settle_duration = yaml.Read<double>(
        base + ".settle_duration").value_or(0.20);

    const auto positive_finite = [](double value) {
        return std::isfinite(value) && value > 0.0;
    };
    if (!positive_finite(config.move_duration) ||
        !positive_finite(config.position_tolerance) ||
        !positive_finite(config.velocity_tolerance) ||
        !positive_finite(config.settle_duration)) {
        throw std::runtime_error(
            "[BehaviorManager] " + base + " 到位参数无效");
    }
    return config;
}

}  // namespace

class BehaviorManagerClass::Impl {
public:
    FSM fsm;
    robot_base::RobotData sensor;
    robot_base::FaultStatus sensor_fault;
    robot_base::Command command;
    ControlOutput output;
    std::string config_path;
    std::string robot_dir;  // 机器人资源根目录（绝对路径）
    ZeroTransitionConfig zero_transition_config;
    std::vector<double> zero_kp;
    std::vector<double> zero_kd;
    bool initialized = false;
    std::string pending_policy;               // 待生效的策略名
    std::string active_policy;                // 当前已加载的策略名
    bool has_rl = false;                      // 是否配置了 RL 状态
    std::atomic<double> rl_freq_hz{0.0};      // RL 实时推理频率（Hz）
    robot_base::ThreadLoop infer_thread_cfg;  // 推理线程配置（robot_base.threads.rl_infer）
    SafetyMonitorConfig safety_monitor;
    double safety_release_duration = 1.0;
    robot_base::FaultStatus fault;
    uint64_t fault_sequence = 0;
    uint64_t previous_fault_ack_sequence = 0;
    bool fault_block_reported = false;
    uint64_t observed_fault_sequence = 0;
    uint64_t acknowledged_sensor_fault_sequence = 0;
    robot_base::FaultSource acknowledged_sensor_fault_source =
        robot_base::FaultSource::NONE;
    robot_base::FaultCode acknowledged_sensor_fault_code =
        robot_base::FaultCode::NONE;
    int32_t acknowledged_sensor_fault_native_code = 0;
    std::vector<robot_base::FaultStatus> fault_observations;

    // 策略链调度（prerequisite）
    struct PrerequisiteEntry {
        std::string policy;
        double duration;
    };
    std::unordered_map<std::string, PrerequisiteEntry> prerequisite_map;
    std::string final_target_policy;  // 用户最终目标策略；空 = 当前无前置链
    double prerequisite_timer = 0.0;  // 前置策略已运行时长（仅 RL 状态累计）
    bool waiting_prerequisite = false;

    // 边沿检测（control_runtime 每帧重发缓存 cmd，需要识别真实的"用户新请求"）
    std::string prev_switch_policy;          // 上一帧 cmd.switch_policy
    StateName prev_fsm_state = StateName::POWER_OFF;  // 上一帧 FSM 状态

    robot_base::FaultStatus MakeFault(robot_base::FaultSource source,
            robot_base::FaultCode code, const std::string &detail,
            int32_t native_code = 0) const {
        robot_base::FaultStatus result;
        result.active = true;
        result.latched = true;
        result.source = source;
        result.code = code;
        result.native_code = native_code;
        result.detail = detail;
        return result;
    }

    bool IsAcknowledgedSensorFault(const robot_base::FaultStatus &candidate) const {
        return candidate.sequence != 0 &&
            candidate.sequence == acknowledged_sensor_fault_sequence &&
            candidate.source == acknowledged_sensor_fault_source &&
            candidate.code == acknowledged_sensor_fault_code &&
            candidate.native_code == acknowledged_sensor_fault_native_code;
    }

    void ClearAcknowledgedSensorFault() {
        acknowledged_sensor_fault_sequence = 0;
        acknowledged_sensor_fault_source = robot_base::FaultSource::NONE;
        acknowledged_sensor_fault_code = robot_base::FaultCode::NONE;
        acknowledged_sensor_fault_native_code = 0;
    }

    std::optional<robot_base::FaultStatus> EvaluateReportedSensorFault() {
        if (sensor_fault.active) {
            if (IsAcknowledgedSensorFault(sensor_fault)) ClearAcknowledgedSensorFault();
            return sensor_fault;
        }

        if (sensor_fault.latched && !IsAcknowledgedSensorFault(sensor_fault))
            return sensor_fault;
        return std::nullopt;
    }

    std::optional<robot_base::FaultStatus> EvaluateSafetyMonitorFault() {
        const bool base_valid = sensor.IsValid() && std::isfinite(sensor.time) &&
            sensor.time >= 0.0 && AllFinite(sensor.rpy) &&
            AllFinite(sensor.gyro) && AllFinite(sensor.acceleration) &&
            AllFinite(sensor.base_pos) && AllFinite(sensor.base_quat) &&
            QuaternionNormSquared(sensor.base_quat) > 1.0e-12 &&
            AllFinite(sensor.base_vel) && AllFinite(sensor.joint_pos) &&
            AllFinite(sensor.joint_vel);
        const bool optional_vectors_valid =
            (sensor.joint_torque.empty() ||
                (sensor.joint_torque.size() == sensor.joint_pos.size() &&
                    AllFinite(sensor.joint_torque))) &&
            (sensor.joint_temperature.empty() ||
                (sensor.joint_temperature.size() == sensor.joint_pos.size() &&
                    AllFinite(sensor.joint_temperature))) &&
            (sensor.joint_error.empty() ||
                sensor.joint_error.size() == sensor.joint_pos.size());
        if (!base_valid || !optional_vectors_valid) {
            return MakeFault(robot_base::FaultSource::SAFETY_MONITOR,
                robot_base::FaultCode::INVALID_DATA,
                "robot state contains invalid dimensions or non-finite values",
                static_cast<int32_t>(SafetyMonitorFault::kInvalidState));
        }

        const StateName state = fsm.CurrentState();
        if (state != StateName::POWER_OFF) {
            if ((safety_monitor.max_roll > 0.0 &&
                    std::abs(sensor.rpy[0]) > safety_monitor.max_roll) ||
                (safety_monitor.max_pitch > 0.0 &&
                    std::abs(sensor.rpy[1]) > safety_monitor.max_pitch)) {
                std::ostringstream detail;
                detail << "body attitude exceeded limit: roll=" << sensor.rpy[0]
                    << ", pitch=" << sensor.rpy[1];
                return MakeFault(robot_base::FaultSource::SAFETY_MONITOR,
                    robot_base::FaultCode::LIMIT_EXCEEDED, detail.str(),
                    static_cast<int32_t>(SafetyMonitorFault::kBodyAttitude));
            }
            if (safety_monitor.max_angular_velocity > 0.0) {
                const auto velocity = std::max_element(
                    sensor.gyro.begin(), sensor.gyro.end(),
                    [](double lhs, double rhs) {
                        return std::abs(lhs) < std::abs(rhs);
                    });
                if (velocity != sensor.gyro.end() &&
                    std::abs(*velocity) > safety_monitor.max_angular_velocity) {
                    std::ostringstream detail;
                    detail << "body angular velocity exceeded limit: "
                        << *velocity << " rad/s";
                    return MakeFault(robot_base::FaultSource::SAFETY_MONITOR,
                        robot_base::FaultCode::LIMIT_EXCEEDED, detail.str(),
                        static_cast<int32_t>(SafetyMonitorFault::kAngularVelocity));
                }
            }
        }

        return std::nullopt;
    }

    void BeginFaultObservationCycle() {
        fault_observations.clear();
    }

    void ObserveFault(const std::optional<robot_base::FaultStatus> &condition) {
        if (!condition || (!condition->active && !condition->latched)) return;
        fault_observations.push_back(*condition);
    }

    bool MatchesCurrentFault(const robot_base::FaultStatus &candidate) const {
        return fault.latched && fault.source == candidate.source &&
            fault.code == candidate.code &&
            fault.native_code == candidate.native_code &&
            observed_fault_sequence == candidate.sequence;
    }

    void LatchFault(const robot_base::FaultStatus &condition) {
        observed_fault_sequence = condition.sequence;
        fault = condition;
        fault.latched = true;
        fault.sequence = ++fault_sequence;
        fault.timestamp_s = MonotonicTimeSeconds();
        runtime_logging::Log(runtime_logging::Level::kError,
            std::string("fault latched: source=") +
                robot_base::FaultSourceName(fault.source) +
                ", code=" + robot_base::FaultCodeName(fault.code) +
                ", detail=" + fault.detail,
            false);
    }

    void ResolveObservedFaults() {
        const auto active_root = std::find_if(
            fault_observations.begin(), fault_observations.end(),
            [this](const robot_base::FaultStatus &candidate) {
                return candidate.active && MatchesCurrentFault(candidate);
            });
        if (fault.latched && fault.active &&
            active_root != fault_observations.end()) {
            return;
        }

        const auto first_active = std::find_if(
            fault_observations.begin(), fault_observations.end(),
            [](const robot_base::FaultStatus &candidate) {
                return candidate.active;
            });
        if (first_active != fault_observations.end()) {
            LatchFault(*first_active);
            return;
        }

        if (fault.latched) {
            fault.active = false;
            return;
        }

        const auto first_latched = std::find_if(
            fault_observations.begin(), fault_observations.end(),
            [](const robot_base::FaultStatus &candidate) {
                return candidate.latched;
            });
        if (first_latched != fault_observations.end()) {
            LatchFault(*first_latched);
        }
    }

    void AcknowledgeFault(uint64_t sequence) {
        if (!fault.latched || fsm.CurrentState() != StateName::POWER_OFF) return;
        if (sequence == 0 || sequence != fault.sequence) {
            runtime_logging::Log(runtime_logging::Level::kWarning,
                "fault acknowledgement rejected because its sequence is stale", false);
            return;
        }
        if (fault.active || sensor_fault.active) {
            runtime_logging::Log(runtime_logging::Level::kWarning,
                "fault acknowledgement rejected because the condition is still active",
                false);
            return;
        }
        runtime_logging::Log(runtime_logging::Level::kInfo,
            std::string("fault acknowledged: source=") +
                robot_base::FaultSourceName(fault.source) +
                ", code=" + robot_base::FaultCodeName(fault.code),
            false);
        if (sensor_fault.latched && !sensor_fault.active &&
            sensor_fault.source == fault.source && sensor_fault.code == fault.code &&
            sensor_fault.native_code == fault.native_code &&
            sensor_fault.sequence == observed_fault_sequence) {
            acknowledged_sensor_fault_sequence = sensor_fault.sequence;
            acknowledged_sensor_fault_source = sensor_fault.source;
            acknowledged_sensor_fault_code = sensor_fault.code;
            acknowledged_sensor_fault_native_code = sensor_fault.native_code;
        }
        fault = {};
        observed_fault_sequence = 0;
        fault_block_reported = false;
    }

    void LoadConfig(const std::string &path) {
        config_path = path;

        // 从 YAML 初始化 RobotBase（获取 num_dof 并初始化关节向量大小）
        sensor = robot_base::RobotData::FromYaml(path);
        int num_dof = sensor.num_dof;

        robot_base::YamlFile yaml_file = robot_base::YamlFile::Load(path);

        // 解析 robot_dir（绝对路径）
        robot_dir =
            yaml_file.ToAbsPath(yaml_file.Read<std::string>("robot_base.robot_dir").value());

        // 读取推理线程配置（robot_base.threads.rl_infer）
        infer_thread_cfg = robot_base::ThreadLoop::FromYaml(yaml_file, "rl_infer");

        safety_monitor.max_roll = yaml_file.Read<double>(
            "behavior_manager.safety.max_roll").value_or(0.0);
        safety_monitor.max_pitch = yaml_file.Read<double>(
            "behavior_manager.safety.max_pitch").value_or(0.0);
        safety_monitor.max_angular_velocity = yaml_file.Read<double>(
            "behavior_manager.safety.max_angular_velocity").value_or(0.0);
        safety_release_duration = yaml_file.Read<double>(
            "behavior_manager.safety.release_duration_s").value_or(1.0);
        const std::array<double, 3> safety_limits = {
            safety_monitor.max_roll, safety_monitor.max_pitch,
            safety_monitor.max_angular_velocity};
        if (!AllFinite(safety_limits) ||
            std::any_of(safety_limits.begin(), safety_limits.end(),
                [](double value) { return value < 0.0; })) {
            throw std::runtime_error(
                "[BehaviorManager] behavior_manager.safety 限值无效");
        }
        if (!std::isfinite(safety_release_duration) || safety_release_duration < 0.0) {
            throw std::runtime_error(
                "[BehaviorManager] behavior_manager.safety.release_duration_s 无效");
        }

        // 机器人基本信息（用于日志输出，从 robot_base 获取）
        std::cout << "[BehaviorManager] 机器人: " << sensor.name << ", 自由度: " << num_dof
                << std::endl;

        const auto default_joint_pos =
            yaml_file.Read<std::vector<double>>("robot_base.default_joint_pos").value_or(
                std::vector<double>{});
        const auto robot_kp = yaml_file.Read<std::vector<double>>("robot_base.kp").value_or(
            std::vector<double>{});
        const auto robot_kd = yaml_file.Read<std::vector<double>>("robot_base.kd").value_or(
            std::vector<double>{});
        ValidateJointVector(default_joint_pos, num_dof, "robot_base.default_joint_pos");
        ValidateGainVector(robot_kp, num_dof, "robot_base.kp");
        ValidateGainVector(robot_kd, num_dof, "robot_base.kd");

        // 无 RL 时，ZERO 默认回退到机型默认姿态。
        auto zero_pos_opt = yaml_file.Read<std::vector<double>>("behavior_manager.zero_pos");
        std::vector<double> zero_pos = zero_pos_opt.value_or(default_joint_pos);
        ValidateJointVector(zero_pos, num_dof, "behavior_manager.zero_pos");

        // 加载 PD 参数（仅 damp_kd，RL 策略 kp/kd 由各策略配置独立提供）
        auto load_array = [&](const std::string &key) {
            return yaml_file.Read<std::vector<double>>("behavior_manager." + key)
                .value_or(std::vector<double>{});
        };

        // 缓存状态参数，供初始注册和运行时策略切换共同使用。
        const double legacy_zero_duration =
            yaml_file.Read<double>("behavior_manager.zero_duration").value_or(3.0);
        zero_transition_config = LoadZeroTransitionConfig(
            yaml_file, legacy_zero_duration);
        const double home_gain_ramp_duration =
            yaml_file.Read<double>("behavior_manager.home.gain_ramp_duration").value_or(1.0);
        const double home_move_duration =
            yaml_file.Read<double>("behavior_manager.home.move_duration").value_or(3.0);
        if (!std::isfinite(home_gain_ramp_duration) ||
            home_gain_ramp_duration <= 0.0 ||
            !std::isfinite(home_move_duration) || home_move_duration <= 0.0) {
            throw std::runtime_error(
                "[BehaviorManager] HOME 切换时长配置无效");
        }

        const auto home_kp =
            yaml_file.Read<std::vector<double>>("behavior_manager.home.kp").value_or(robot_kp);
        const auto home_kd =
            yaml_file.Read<std::vector<double>>("behavior_manager.home.kd").value_or(robot_kd);
        ValidateGainVector(home_kp, num_dof, "behavior_manager.home.kp");
        ValidateGainVector(home_kd, num_dof, "behavior_manager.home.kd");
        const auto zero_kp_opt =
            yaml_file.Read<std::vector<double>>("behavior_manager.zero.kp");
        const auto zero_kd_opt =
            yaml_file.Read<std::vector<double>>("behavior_manager.zero.kd");
        if (zero_kp_opt.has_value() != zero_kd_opt.has_value()) {
            throw std::runtime_error(
                "[BehaviorManager] behavior_manager.zero.kp/kd 必须同时配置");
        }
        if (zero_kp_opt) {
            zero_kp = *zero_kp_opt;
            zero_kd = *zero_kd_opt;
            ValidateGainVector(zero_kp, num_dof, "behavior_manager.zero.kp");
            ValidateGainVector(zero_kd, num_dof, "behavior_manager.zero.kd");
        }

        // 初始化输出维度
        output.target_pos.assign(num_dof, 0.0);
        output.target_vel.assign(num_dof, 0.0);
        output.target_torque.assign(num_dof, 0.0);

        // 加载 DAMP 状态阻尼 kd
        std::vector<double> damp_kd = load_array("damp_kd");
        if (damp_kd.empty()) {
            throw std::runtime_error("[BehaviorManager] 缺少配置项 behavior_manager.damp_kd");
        }
        ValidateGainVector(damp_kd, num_dof, "behavior_manager.damp_kd");

        // 注册固定状态
        fsm.AddState(StateName::POWER_OFF, CreateStatePowerOff());
        fsm.AddState(StateName::DAMP, CreateStateDamp(damp_kd));
        fsm.AddState(StateName::HOME,
            CreateStateHome(default_joint_pos, home_gain_ramp_duration,
                            home_move_duration, home_kp, home_kd));

        // RL 状态配置：调用 policy_executor 提供的统一解析接口
        // ZERO 目标位置跟随当前策略，控制增益使用 behavior_manager.zero。
        auto rl_type_opt = yaml_file.Read<std::string>("rl_policy.type");
        if (rl_type_opt) {
            const std::string rl_type = rl_type_opt.value();

            // 当前仅支持 onnx_infer 后端，未来可扩展 torch 等
            if (rl_type != "onnx_infer") {
                throw std::runtime_error("[BehaviorManager] 不支持的 rl_policy type: " + rl_type);
            }

            // 记录初始策略名（pending_policy 与 active_policy 保持一致，避免启动时触发切换）
            active_policy =
                yaml_file.Read<std::string>("rl_policy.onnx_infer.default_policy").value_or("");
            pending_policy = active_policy;

            RLConfig rc =
                LoadRLStateConfig(path, active_policy, robot_dir);
            rc.infer_thread_cfg = infer_thread_cfg;
            rc.rl_freq_hz = &rl_freq_hz;

            // ZERO 状态目标位姿：优先用 zero_target_pos（如果配了），否则用 rl_default_pos
            const auto &effective_zero_pos = rc.zero_target_pos.empty()
                ? rc.policy.rl_default_pos : rc.zero_target_pos;
            const auto &effective_zero_kp = zero_kp.empty() ? rc.kp : zero_kp;
            const auto &effective_zero_kd = zero_kd.empty() ? rc.kd : zero_kd;
            fsm.AddState(StateName::ZERO,
                CreateStateZero(effective_zero_pos, zero_transition_config,
                                effective_zero_kp, effective_zero_kd));

            fsm.AddState(StateName::RL, CreateStateRl(rc));
            has_rl = true;

            std::cout << "[BehaviorManager] RL 状态: 已加载 (" << rc.policy.model_path << ")"
                    << std::endl;

            // 解析所有策略的可选 prerequisite 子节点，构建策略链 map
            auto policy_names = yaml_file.Read<std::vector<std::string>>(
                "rl_policy.onnx_infer.policy_names").value_or(std::vector<std::string>{});
            for (const auto &pname : policy_names) {
                const std::string base = "rl_policy.onnx_infer.policies." + pname + ".prerequisite";
                auto pre_pol = yaml_file.Read<std::string>(base + ".policy");
                auto pre_dur = yaml_file.Read<double>(base + ".duration");
                if (pre_pol && pre_dur && !pre_pol->empty() && *pre_pol != pname) {
                    prerequisite_map[pname] = {*pre_pol, *pre_dur};
                    std::cout << "[BehaviorManager] 策略链: " << pname << " ← " << *pre_pol
                            << " (" << *pre_dur << "s)" << std::endl;
                }
            }
        } else {
            // 无 RL 策略时，ZERO 使用独立安全增益和可选 zero_pos。
            const auto &effective_zero_kp = zero_kp.empty() ? home_kp : zero_kp;
            const auto &effective_zero_kd = zero_kd.empty() ? home_kd : zero_kd;
            fsm.AddState(StateName::ZERO,
                CreateStateZero(
                    zero_pos, zero_transition_config,
                    effective_zero_kp, effective_zero_kd));
            std::cout << "[BehaviorManager] RL 状态: 未配置" << std::endl;
        }

        fsm.AddState(StateName::SAFETY,
            CreateStateSafety(safety_release_duration, &fault));

        // 设置共享数据指针
        fsm.SetDataPointers(&sensor, &command, &output);
    }
};

BehaviorManagerClass::BehaviorManagerClass(const std::string &config_path)
    : impl_(std::make_unique<Impl>()) {
    impl_->LoadConfig(config_path);
}

BehaviorManagerClass::~BehaviorManagerClass() = default;

void BehaviorManagerClass::Init() {
    impl_->fsm.Init();
    impl_->initialized = true;
    std::cout << "[BehaviorManager] 初始化完成" << std::endl;
}

void BehaviorManagerClass::Step(float control_dt, float rl_dt) {
    if (!impl_->initialized)
        return;

    impl_->BeginFaultObservationCycle();
    impl_->ObserveFault(impl_->EvaluateReportedSensorFault());
    impl_->ObserveFault(impl_->EvaluateSafetyMonitorFault());
    const auto state_fault_before_step = impl_->fsm.CurrentFault();
    if (state_fault_before_step.active || state_fault_before_step.latched) {
        impl_->ObserveFault(state_fault_before_step);
    }
    impl_->ResolveObservedFaults();

    StateName current = impl_->fsm.CurrentState();
    if (impl_->fault.latched && current != StateName::POWER_OFF &&
        current != StateName::SAFETY) {
        impl_->fsm.ForceSwitch(StateName::SAFETY,
            std::string("latched fault: ") +
                robot_base::FaultSourceName(impl_->fault.source) + "/" +
                robot_base::FaultCodeName(impl_->fault.code));
        current = impl_->fsm.CurrentState();
    }
    if (impl_->fault.latched && current == StateName::POWER_OFF &&
        impl_->command.key != 0 && impl_->command.key != -1) {
        impl_->command.key = 0;
        if (!impl_->fault_block_reported) {
            runtime_logging::Log(runtime_logging::Level::kWarning,
                "FSM power-on request blocked by a latched fault", false);
            impl_->fault_block_reported = true;
        }
    }

    // 策略切换：pending_policy 由 SetCommand（POWER_OFF/DAMP 直切）或前置链调度（RL 中到期自动切）触发
    if (impl_->has_rl && impl_->pending_policy != impl_->active_policy) {
        try {
            RLConfig rc = LoadRLStateConfig(
                impl_->config_path, impl_->pending_policy, impl_->robot_dir);
            rc.infer_thread_cfg = impl_->infer_thread_cfg;
            rc.rl_freq_hz = &impl_->rl_freq_hz;

            // 用 ReplaceState 替换 ZERO + RL，安全处理"替换正在运行的当前状态"场景
            // （前置链到期后会在 RL 状态触发 RL→RL 重建，需走 OnExit/OnEnter 重置 LSTM 隐状态）
            const auto &effective_zero_pos = rc.zero_target_pos.empty()
                ? rc.policy.rl_default_pos : rc.zero_target_pos;
            const auto &effective_zero_kp =
                impl_->zero_kp.empty() ? rc.kp : impl_->zero_kp;
            const auto &effective_zero_kd =
                impl_->zero_kd.empty() ? rc.kd : impl_->zero_kd;
            impl_->fsm.ReplaceState(StateName::ZERO,
                CreateStateZero(effective_zero_pos,
                                impl_->zero_transition_config,
                                effective_zero_kp, effective_zero_kd));
            impl_->fsm.ReplaceState(StateName::RL, CreateStateRl(rc));

            impl_->active_policy = impl_->pending_policy;
            impl_->prerequisite_timer = 0.0;  // 切换后重置计时（仅前置策略生效时再启用）
            std::cout << "[BehaviorManager] 策略已切换: " << impl_->active_policy << " ("
                    << rc.policy.model_path << ")" << std::endl;
            runtime_logging::Log(runtime_logging::Level::kInfo,
                "policy switched: " + impl_->active_policy + " (" +
                    rc.policy.model_path + ")",
                false);
        } catch (const std::exception &e) {
            std::cerr << "[BehaviorManager] 策略切换失败: " << e.what() << std::endl;
            runtime_logging::Log(runtime_logging::Level::kError,
                std::string("policy switch failed: ") + e.what(), false);
            impl_->pending_policy = impl_->active_policy;  // 回滚
            impl_->final_target_policy.clear();
            impl_->waiting_prerequisite = false;
        }
    }

    impl_->fsm.Step(control_dt, rl_dt);

    const auto state_fault_after_step = impl_->fsm.CurrentFault();
    if (state_fault_after_step.active || state_fault_after_step.latched) {
        impl_->ObserveFault(state_fault_after_step);
        impl_->ResolveObservedFaults();
        const StateName state = impl_->fsm.CurrentState();
        if (state != StateName::POWER_OFF && state != StateName::SAFETY) {
            impl_->fsm.ForceSwitch(StateName::SAFETY,
                std::string("state fault: ") +
                    robot_base::FaultSourceName(impl_->fault.source) + "/" +
                    robot_base::FaultCodeName(impl_->fault.code));
        }
    }

    // 前置策略链调度：仅在 RL 状态累计时长，到期后设置 pending_policy = final_target
    StateName cur = impl_->fsm.CurrentState();
    if (impl_->waiting_prerequisite && cur == StateName::RL) {
        impl_->prerequisite_timer += control_dt;
        auto it = impl_->prerequisite_map.find(impl_->final_target_policy);
        if (it != impl_->prerequisite_map.end() &&
            impl_->prerequisite_timer >= it->second.duration) {
            std::cout << "[BehaviorManager] 前置策略 " << impl_->active_policy << " 运行 "
                    << impl_->prerequisite_timer << "s 完成，自动切换至目标策略 "
                    << impl_->final_target_policy << std::endl;
            runtime_logging::Log(runtime_logging::Level::kInfo,
                "prerequisite policy completed: " + impl_->active_policy +
                    " -> " + impl_->final_target_policy,
                false);
            impl_->pending_policy = impl_->final_target_policy;
            impl_->final_target_policy.clear();
            impl_->waiting_prerequisite = false;
        }
    }
    // RL → POWER_OFF 边沿：用户中途按 ESC 退回 POWER_OFF，取消前置链调度
    if (impl_->waiting_prerequisite &&
        impl_->prev_fsm_state == StateName::RL && cur == StateName::POWER_OFF) {
        std::cout << "[BehaviorManager] RL → POWER_OFF，取消前置链调度" << std::endl;
        runtime_logging::Log(runtime_logging::Level::kWarning,
            "prerequisite policy chain cancelled by RL -> POWER_OFF", false);
        impl_->final_target_policy.clear();
        impl_->waiting_prerequisite = false;
        impl_->prerequisite_timer = 0.0;
    }
    impl_->prev_fsm_state = cur;
}

void BehaviorManagerClass::SetSensorData(const robot_base::RobotData &data) {
    impl_->sensor = data;
}

void BehaviorManagerClass::SetSensorFault(const robot_base::FaultStatus &fault) {
    impl_->sensor_fault = fault;
}

void BehaviorManagerClass::SetCommand(const robot_base::Command &cmd) {
    impl_->command = cmd;
    // switch_policy 边沿检测：control_runtime 每帧把缓存 cmd 重复喂入，必须只在变化时响应
    const bool edge = (cmd.switch_policy != impl_->prev_switch_policy);
    impl_->prev_switch_policy = cmd.switch_policy;
    if (!edge || cmd.switch_policy.empty()) {
        return;
    }
    // 仅在 POWER_OFF / DAMP 状态允许切换策略；进入 ZERO 后策略已锁定，不再接受切换
    StateName cur = impl_->fsm.CurrentState();
    if (cur != StateName::POWER_OFF && cur != StateName::DAMP) {
        return;
    }
    const std::string &target = cmd.switch_policy;
    auto it = impl_->prerequisite_map.find(target);
    if (it != impl_->prerequisite_map.end()) {
        // 命中前置链：先切前置策略，记录最终目标，等 RL 跑满 duration 后自动切换
        impl_->final_target_policy = target;
        impl_->pending_policy = it->second.policy;
        impl_->prerequisite_timer = 0.0;
        impl_->waiting_prerequisite = true;
        std::cout << "[BehaviorManager] 策略链调度: " << it->second.policy << " ("
                << it->second.duration << "s) → " << target << std::endl;
        runtime_logging::Log(runtime_logging::Level::kInfo,
            "policy chain scheduled: " + it->second.policy + " -> " + target,
            false);
    } else {
        // 无前置链：直接切目标策略，并清除可能残留的前置链状态
        impl_->pending_policy = target;
        impl_->final_target_policy.clear();
        impl_->waiting_prerequisite = false;
        impl_->prerequisite_timer = 0.0;
    }
}

void BehaviorManagerClass::AcknowledgeFault(uint64_t sequence) {
    if (sequence == 0) {
        impl_->previous_fault_ack_sequence = 0;
        return;
    }
    const bool can_acknowledge = impl_->fault.latched &&
        impl_->fsm.CurrentState() == StateName::POWER_OFF &&
        !impl_->fault.active && !impl_->sensor_fault.active;
    if (!can_acknowledge || sequence == impl_->previous_fault_ack_sequence) return;

    impl_->previous_fault_ack_sequence = sequence;
    impl_->AcknowledgeFault(sequence);
}

const ControlOutput &BehaviorManagerClass::GetOutput() const {
    return impl_->output;
}

StateName BehaviorManagerClass::CurrentState() const {
    return impl_->fsm.CurrentState();
}

bool BehaviorManagerClass::IsZeroReady() const {
    return impl_->fsm.ReadyForRl();
}

std::string BehaviorManagerClass::CurrentPolicyName() const {
    return impl_->active_policy;
}

double BehaviorManagerClass::GetRlFreq() const {
    return impl_->rl_freq_hz.load(std::memory_order_relaxed);
}

robot_base::FaultStatus BehaviorManagerClass::CurrentFault() const {
    return impl_->fault;
}

bool BehaviorManagerClass::IsRunning() const {
    return impl_->initialized;
}

}  // namespace behavior_manager
