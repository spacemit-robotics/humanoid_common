/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file whole_body_diagnostics.cpp
 * @brief Whole-body live monitor and telemetry serialization
 */

#include "backends/whole_body_diagnostics.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "robot_base.h"
#include "runtime_logger.h"

namespace driver_runtime {
namespace {

constexpr double kRadiansToDegrees = 180.0 / 3.14159265358979323846;
constexpr size_t kMaxTerminalAlerts = 4;

struct BusSummary {
    std::string name;
    std::string device;
    uint32_t motor_count = 0;
    uint32_t fresh_count = 0;
    uint32_t warning_count = 0;
    uint32_t fault_count = 0;
    bool has_feedback_age = false;
    double max_feedback_age_s = 0.0;
};

struct ExtremeValue {
    std::string name;
    double value = 0.0;
    bool valid = false;
};

const char *HealthName(whole_body_health_state state) {
    switch (state) {
        case WHOLE_BODY_HEALTH_CREATED:
            return "初始化中";
        case WHOLE_BODY_HEALTH_READY:
            return "正常";
        case WHOLE_BODY_HEALTH_READ_ONLY:
            return "只读";
        case WHOLE_BODY_HEALTH_WATCHDOG:
            return "看门狗触发";
        case WHOLE_BODY_HEALTH_ERROR:
            return "故障";
    }
    return "未知";
}

double WallTimeSeconds() {
    return std::chrono::duration<double>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

std::array<double, 3> ImuRpy(const whole_body_imu_diagnostic_v2 &imu) {
    std::array<double, 4> quaternion = {
        imu.quaternion[0], imu.quaternion[1], imu.quaternion[2], imu.quaternion[3]};
    std::array<double, 3> rpy{};
    robot_base::QuatToRpy(quaternion, rpy);
    return rpy;
}

std::string CompactLabel(std::string label, size_t max_length) {
    const std::string suffix = "_joint";
    if (label.size() >= suffix.size() &&
        label.compare(label.size() - suffix.size(), suffix.size(), suffix) == 0) {
        label.erase(label.size() - suffix.size());
    }
    if (label.size() <= max_length) return label;
    label.resize(max_length - 1);
    label.push_back('~');
    return label;
}

void UpdateAbsoluteExtreme(ExtremeValue *extreme, const char *name, double value) {
    if (!extreme || !std::isfinite(value)) return;
    if (!extreme->valid || std::abs(value) > std::abs(extreme->value)) {
        extreme->name = CompactLabel(name ? name : "unknown", 22);
        extreme->value = value;
        extreme->valid = true;
    }
}

void UpdateMaximum(ExtremeValue *extreme, const char *name, double value) {
    if (!extreme || !std::isfinite(value)) return;
    if (!extreme->valid || value > extreme->value) {
        extreme->name = CompactLabel(name ? name : "unknown", 22);
        extreme->value = value;
        extreme->valid = true;
    }
}

BusSummary &FindBusSummary(std::vector<BusSummary> *buses,
    const whole_body_motor_diagnostic_v2 &motor) {
    const auto match = [&motor](const BusSummary &bus) {
        return bus.name == motor.bus && bus.device == motor.device;
    };
    const auto found = std::find_if(buses->begin(), buses->end(), match);
    if (found != buses->end()) return *found;
    BusSummary summary;
    summary.name = motor.bus;
    summary.device = motor.device;
    buses->push_back(summary);
    return buses->back();
}

const char *BusState(const BusSummary &bus) {
    if (bus.fault_count > 0) return "故障";
    if (bus.fresh_count == 0) return "无反馈";
    if (bus.fresh_count < bus.motor_count) return "反馈过期";
    if (bus.warning_count > 0) return "警告";
    return "正常";
}

const char *ImuState(const whole_body_imu_diagnostic_v2 &imu) {
    if (!imu.feedback_received) return "无反馈";
    return imu.feedback_fresh ? "正常" : "反馈过期";
}

std::string CompactJointNames(const char *joint_names) {
    const std::string names = joint_names ? joint_names : "";
    if (names.empty()) return "未映射关节";

    std::string compact;
    size_t begin = 0;
    for (;;) {
        const size_t end = names.find('+', begin);
        std::string joint = names.substr(begin, end - begin);
        const std::string suffix = "_joint";
        if (joint.size() >= suffix.size() &&
            joint.compare(joint.size() - suffix.size(), suffix.size(), suffix) == 0) {
            joint.erase(joint.size() - suffix.size());
        }
        if (!compact.empty()) compact += "+";
        compact += joint;
        if (end == std::string::npos) break;
        begin = end + 1;
    }
    return CompactLabel(compact, 28);
}

std::string MotorAlertTarget(const whole_body_motor_diagnostic_v2 &motor) {
    const char *device = motor.device[0] != '\0' ? motor.device : motor.bus;
    std::ostringstream location;
    location << motor.name << "@" << device << ":0x"
        << std::hex << motor.command_id;
    std::ostringstream output;
    output << CompactJointNames(motor.joint_names) << " ["
        << CompactLabel(location.str(), 30) << "]";
    return output.str();
}

std::string HexCode(uint32_t code) {
    std::ostringstream output;
    output << "0x" << std::hex << code;
    return output.str();
}

std::string SignedValue(double value, int precision) {
    std::ostringstream output;
    output << std::showpos << std::fixed << std::setprecision(precision) << value;
    return output.str();
}

std::string ColumnPrefix(const char *name) {
    std::string prefix = name ? name : "device";
    for (char &character : prefix) {
        if (!std::isalnum(static_cast<unsigned char>(character)) && character != '_')
            character = '_';
    }
    return prefix;
}

}  // namespace

void RenderWholeBodyDiagnostics(
    const whole_body_diagnostics_v2 &diagnostics, double cycle_s) {
    const auto rpy = ImuRpy(diagnostics.imu);
    uint32_t fresh_motor_count = 0;
    uint32_t valid_joint_count = 0;
    uint32_t valid_coupling_count = 0;
    double max_motor_age_s = 0.0;
    bool has_motor_age = false;
    double max_gyro = 0.0;
    std::vector<BusSummary> buses;
    ExtremeValue max_velocity;
    ExtremeValue max_torque;
    ExtremeValue max_temperature;

    for (uint32_t i = 0; i < diagnostics.motor_count; ++i) {
        const auto &motor = diagnostics.motors[i];
        BusSummary &bus = FindBusSummary(&buses, motor);
        ++bus.motor_count;
        if (motor.feedback_fresh) {
            ++fresh_motor_count;
            ++bus.fresh_count;
        }
        if (motor.warning_error != 0) ++bus.warning_count;
        if (motor.fatal_error != 0) ++bus.fault_count;
        if (motor.feedback_received && std::isfinite(motor.feedback_age_s)) {
            bus.max_feedback_age_s = std::max(bus.max_feedback_age_s,
                motor.feedback_age_s);
            bus.has_feedback_age = true;
            max_motor_age_s = std::max(max_motor_age_s, motor.feedback_age_s);
            has_motor_age = true;
        }
    }
    for (uint32_t i = 0; i < diagnostics.joint_count; ++i) {
        const auto &joint = diagnostics.joints[i];
        if (!joint.feedback_valid) continue;
        ++valid_joint_count;
        UpdateAbsoluteExtreme(&max_velocity, joint.name, joint.velocity);
        UpdateAbsoluteExtreme(&max_torque, joint.name, joint.torque);
        UpdateMaximum(&max_temperature, joint.name, joint.temperature);
    }
    for (uint32_t i = 0; i < diagnostics.coupling_count; ++i) {
        const auto &coupling = diagnostics.couplings[i];
        if (coupling.valid) ++valid_coupling_count;
    }
    for (double gyro : diagnostics.imu.gyro) {
        if (std::isfinite(gyro)) max_gyro = std::max(max_gyro, std::abs(gyro));
    }

    std::vector<std::string> alerts;
    if (diagnostics.health.state != WHOLE_BODY_HEALTH_READY &&
        diagnostics.health.state != WHOLE_BODY_HEALTH_READ_ONLY) {
        std::ostringstream alert;
        alert << "整机 " << HealthName(diagnostics.health.state)
            << "，错误码 " << diagnostics.health.last_error;
        alerts.push_back(alert.str());
    }
    if (!diagnostics.imu.feedback_received) {
        alerts.emplace_back("IMU 未收到反馈");
    } else if (!diagnostics.imu.feedback_fresh) {
        std::ostringstream alert;
        alert << "IMU 反馈过期 " << std::fixed << std::setprecision(1)
            << diagnostics.imu.feedback_age_s * 1000.0 << " ms";
        alerts.push_back(alert.str());
    }
    for (uint32_t i = 0; i < diagnostics.motor_count; ++i) {
        const auto &motor = diagnostics.motors[i];
        if (motor.fatal_error == 0) continue;
        alerts.push_back(MotorAlertTarget(motor) + " 致命故障 " +
            HexCode(motor.fatal_error));
    }
    for (uint32_t i = 0; i < diagnostics.coupling_count; ++i) {
        const auto &coupling = diagnostics.couplings[i];
        if (coupling.valid) continue;
        alerts.push_back("并联关节 " + CompactLabel(coupling.joint_names, 28) +
            " 解算无效");
    }
    for (uint32_t i = 0; i < diagnostics.motor_count; ++i) {
        const auto &motor = diagnostics.motors[i];
        if (!motor.feedback_received) {
            alerts.push_back(MotorAlertTarget(motor) + " 无反馈");
        } else if (!motor.feedback_fresh) {
            std::ostringstream alert;
            alert << MotorAlertTarget(motor) << " 反馈过期 " << std::fixed
                << std::setprecision(1) << motor.feedback_age_s * 1000.0 << " ms";
            alerts.push_back(alert.str());
        }
    }
    for (uint32_t i = 0; i < diagnostics.motor_count; ++i) {
        const auto &motor = diagnostics.motors[i];
        if (motor.warning_error == 0) continue;
        alerts.push_back(MotorAlertTarget(motor) + " 警告 " +
            HexCode(motor.warning_error));
    }

    std::ostringstream output;
    output << "\033[2J\033[H"
        << "================ 整机实时状态 ================\n"
        << "整机：" << HealthName(diagnostics.health.state) << "    控制频率："
        << std::fixed << std::setprecision(1)
        << (cycle_s > 0.0 ? 1.0 / cycle_s : 0.0) << " Hz\n"
        << "反馈：电机 " << fresh_motor_count << "/" << diagnostics.motor_count
        << "    关节 " << valid_joint_count << "/" << diagnostics.joint_count
        << "    IMU " << ImuState(diagnostics.imu) << "\n"
        << "通信：最慢电机 ";
    if (has_motor_age) {
        output << max_motor_age_s * 1000.0 << " ms";
    } else {
        output << "无数据";
    }
    output << "    反馈时间差 " << diagnostics.feedback_window_s * 1000.0
        << " ms    看门狗 " << diagnostics.health.watchdog_events << "\n";
    if (diagnostics.imu.feedback_received) {
        output << "姿态：左右倾 "
            << SignedValue(rpy[0] * kRadiansToDegrees, 2) << "°"
            << "    前后倾 " << SignedValue(rpy[1] * kRadiansToDegrees, 2) << "°"
            << "    水平转动 " << SignedValue(rpy[2] * kRadiansToDegrees, 2) << "°\n"
            << "动态：IMU 最大角速度 "
            << std::fixed << std::setprecision(2)
            << max_gyro * kRadiansToDegrees << "°/s"
            << "    数据延迟 " << diagnostics.imu.feedback_age_s * 1000.0
            << " ms\n";
    } else {
        output << "姿态：暂无有效 IMU 数据\n"
            << "动态：暂无有效 IMU 数据\n";
    }

    output << "---------------- 总线反馈 ----------------\n";
    if (buses.empty()) output << "  未配置电机总线\n";
    for (const auto &bus : buses) {
        const std::string location = CompactLabel(bus.name + "/" + bus.device, 24);
        output << "  " << std::left << std::setw(24) << location
            << std::right << BusState(bus)
            << "    电机 " << bus.fresh_count << "/" << bus.motor_count
            << "    延迟 ";
        if (bus.has_feedback_age) {
            output << std::fixed << std::setprecision(1)
                << bus.max_feedback_age_s * 1000.0 << " ms";
        } else {
            output << "无数据";
        }
        output << "\n";
    }

    output << "---------------- 关节概况 ----------------\n";
    if (max_velocity.valid && max_torque.valid && max_temperature.valid) {
        output << "  最大速度  " << std::left << std::setw(22) << max_velocity.name
            << std::right
            << SignedValue(max_velocity.value * kRadiansToDegrees, 2) << "°/s\n"
            << "  最大力矩  " << std::left << std::setw(22) << max_torque.name
            << std::right << SignedValue(max_torque.value, 2) << " Nm\n"
            << "  最高温度  " << std::left << std::setw(22) << max_temperature.name
            << std::right << std::fixed << std::setprecision(1)
            << max_temperature.value << " C\n";
    } else {
        output << "  暂无有效关节反馈\n";
    }
    if (diagnostics.coupling_count > 0) {
        output << "  并联关节  "
            << (valid_coupling_count == diagnostics.coupling_count ? "正常" : "异常")
            << "    " << valid_coupling_count << "/"
            << diagnostics.coupling_count << "\n";
    }

    output << "---------------- 当前异常 ----------------\n";
    if (alerts.empty()) {
        output << "  无\n";
    } else {
        const size_t shown = std::min(alerts.size(), kMaxTerminalAlerts);
        output << "  共 " << alerts.size() << " 项";
        if (shown < alerts.size()) {
            output << "，仅显示前 " << shown << " 项；完整信息见 driver CSV 日志";
        }
        output << "\n";
        for (size_t i = 0; i < shown; ++i) {
            output << "  " << i + 1 << ". " << alerts[i] << "\n";
        }
    }
    std::cout << output.str() << std::flush;
}

void RecordWholeBodyDiagnostics(const whole_body_diagnostics_v2 &diagnostics,
    const whole_body_motor_command_diagnostics_v2 &command_diagnostics) {
    const double wall_time = WallTimeSeconds();

    static bool inventory_recorded = false;
    if (!inventory_recorded) {
        std::ostringstream inventory;
        for (uint32_t i = 0; i < diagnostics.motor_count; ++i) {
            const auto &motor = diagnostics.motors[i];
            inventory << motor.name << "," << motor.joint_names << ","
                << motor.driver << "," << motor.model << "," << motor.bus << ","
                << motor.device << "," << motor.command_id << "," << motor.feedback_id;
            if (i + 1 < diagnostics.motor_count) inventory << "\n";
        }
        runtime_logging::RecordCsv("driver_inventory",
            "motor,joints,driver,model,bus,device,command_id,feedback_id",
            inventory.str());
        inventory_recorded = true;
    }

    static std::string motor_header;
    if (motor_header.empty()) {
        std::ostringstream header;
        header << "wall_time_s,device_time_s,feedback_window_s";
        for (uint32_t i = 0; i < diagnostics.motor_count; ++i) {
            const std::string prefix = ColumnPrefix(diagnostics.motors[i].name);
            for (const char *field : {"feedback_timestamp_s", "timestamp_source", "received",
                    "fresh", "age_s", "raw_pos", "raw_vel", "raw_torque", "calibrated_pos",
                    "calibrated_vel", "calibrated_torque", "temp_c", "error_raw",
                    "error_warning", "error_fatal", "command_valid", "command_age_s",
                    "command_mode", "command_pos", "command_vel", "command_torque",
                    "command_kp", "command_kd", "estimated_torque", "position_rate",
                    "velocity_rate", "torque_rate"}) {
                header << "," << prefix << "." << field;
            }
        }
        motor_header = header.str();
    }
    std::ostringstream motors;
    motors << std::fixed << std::setprecision(9)
        << wall_time << "," << diagnostics.timestamp_s << ","
        << diagnostics.feedback_window_s;
    for (uint32_t i = 0; i < diagnostics.motor_count; ++i) {
        const auto &motor = diagnostics.motors[i];
        const bool has_command = i < command_diagnostics.motor_count;
        const auto *command = has_command ? &command_diagnostics.motors[i] : nullptr;
        motors << "," << motor.feedback_timestamp_s
            << "," << static_cast<int>(motor.feedback_timestamp_source)
            << "," << motor.feedback_received << "," << motor.feedback_fresh
            << "," << motor.feedback_age_s << "," << motor.raw_position
            << "," << motor.raw_velocity << "," << motor.raw_torque
            << "," << motor.calibrated_position << "," << motor.calibrated_velocity
            << "," << motor.calibrated_torque << "," << motor.temperature
            << "," << motor.error << "," << motor.warning_error
            << "," << motor.fatal_error << "," << (command && command->valid) << ","
            << (command ? command->age_s : 0.0) << ","
            << (command ? command->mode : 0) << ","
            << (command ? command->position : 0.0) << ","
            << (command ? command->velocity : 0.0) << ","
            << (command ? command->torque : 0.0) << ","
            << (command ? command->kp : 0.0) << ","
            << (command ? command->kd : 0.0) << ","
            << (command ? command->estimated_torque : 0.0) << ","
            << (command ? command->position_rate : 0.0) << ","
            << (command ? command->velocity_rate : 0.0) << ","
            << (command ? command->torque_rate : 0.0);
    }
    runtime_logging::RecordCsv("driver_motor", motor_header, motors.str());

    static std::string joint_header;
    if (joint_header.empty()) {
        std::ostringstream header;
        header << "wall_time_s,device_time_s";
        for (uint32_t i = 0; i < diagnostics.joint_count; ++i) {
            const std::string prefix = ColumnPrefix(diagnostics.joints[i].name);
            for (const char *field : {"valid", "position", "velocity", "torque",
                    "temp_c", "error"}) {
                header << "," << prefix << "." << field;
            }
        }
        joint_header = header.str();
    }
    std::ostringstream joints;
    joints << std::fixed << std::setprecision(9)
        << wall_time << "," << diagnostics.timestamp_s;
    for (uint32_t i = 0; i < diagnostics.joint_count; ++i) {
        const auto &joint = diagnostics.joints[i];
        joints << "," << joint.feedback_valid << "," << joint.position << "," << joint.velocity
            << "," << joint.torque << "," << joint.temperature << ","
            << joint.motor_error;
    }
    runtime_logging::RecordCsv("driver_joint", joint_header, joints.str());

    const auto rpy = ImuRpy(diagnostics.imu);
    std::ostringstream imu;
    imu << std::fixed << std::setprecision(9)
        << wall_time << "," << diagnostics.timestamp_s << ","
        << diagnostics.feedback_window_s << ","
        << diagnostics.imu.feedback_received << "," << diagnostics.imu.feedback_fresh << ","
        << diagnostics.imu.feedback_age_s << "," << diagnostics.imu.sample_timestamp_s << ","
        << diagnostics.imu.receive_timestamp_s << ","
        << diagnostics.imu.quaternion[0] << "," << diagnostics.imu.quaternion[1] << ","
        << diagnostics.imu.quaternion[2] << "," << diagnostics.imu.quaternion[3] << ","
        << rpy[0] << "," << rpy[1] << "," << rpy[2] << ","
        << diagnostics.imu.gyro[0] << "," << diagnostics.imu.gyro[1] << ","
        << diagnostics.imu.gyro[2] << "," << diagnostics.imu.acceleration[0] << ","
        << diagnostics.imu.acceleration[1] << "," << diagnostics.imu.acceleration[2] << ","
        << diagnostics.imu.valid_frames << "," << diagnostics.imu.crc_errors << ","
        << diagnostics.imu.decode_errors << "," << diagnostics.imu.superseded_frames << ","
        << diagnostics.imu.resync_discarded_bytes << ","
        << diagnostics.imu.overflow_discarded_bytes;
    runtime_logging::RecordCsv(
        "driver_imu",
        "wall_time_s,device_time_s,feedback_window_s,received,fresh,age_s,"
        "sample_timestamp_s,receive_timestamp_s,qw,qx,qy,qz,roll,pitch,yaw,"
        "gx,gy,gz,ax,ay,az,valid_frames,crc_errors,decode_errors,"
        "superseded_frames,resync_discarded_bytes,overflow_discarded_bytes",
        imu.str());

    static std::string coupling_header;
    if (coupling_header.empty()) {
        std::ostringstream header;
        header << "wall_time_s,device_time_s";
        for (uint32_t i = 0; i < diagnostics.coupling_count; ++i) {
            const std::string prefix = ColumnPrefix(diagnostics.couplings[i].joint_names);
            header << "," << prefix << ".valid"
                << "," << prefix << ".jacobian_condition"
                << "," << prefix << ".torque_amplification";
        }
        coupling_header = header.str();
    }
    if (diagnostics.coupling_count > 0) {
        std::ostringstream couplings;
        couplings << std::fixed << std::setprecision(9)
            << wall_time << "," << diagnostics.timestamp_s;
        for (uint32_t i = 0; i < diagnostics.coupling_count; ++i) {
            const auto &coupling = diagnostics.couplings[i];
            couplings << "," << coupling.valid << "," << coupling.jacobian_condition
                << "," << coupling.torque_amplification;
        }
        runtime_logging::RecordCsv(
            "driver_coupling", coupling_header, couplings.str());
    }
}

}  // namespace driver_runtime
