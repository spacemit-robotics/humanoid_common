/**
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *
 * @file test_rl_inference_logging.cpp
 * @brief Verify rejected inference diagnostics without publishing late actions
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>  // NOLINT(build/c++17)
#include <fstream>
#include <iostream>
#include <iterator>
#include <map>
#include <stdexcept>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "behavior_fsm.h"
#include "rl_test_model.h"
#include "runtime_logger.h"
#include "state_factory.h"

namespace {

namespace fs = std::filesystem;
using behavior_manager::RLRuntimeEvent;
using behavior_manager::RLRuntimeEventType;
using behavior_manager::StateName;
using behavior_manager::testing::kIdentityModel;

void Require(bool condition, const std::string &message) {
    if (!condition) throw std::runtime_error(message);
}

std::string ReadText(const fs::path &path) {
    std::ifstream input(path);
    Require(input.good(), "cannot read " + path.string());
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

std::vector<std::string> ParseCsvRow(const std::string &line) {
    std::vector<std::string> fields(1);
    bool quoted = false;
    for (std::size_t i = 0; i < line.size(); ++i) {
        const char c = line[i];
        if (c == '"') {
            if (quoted && i + 1 < line.size() && line[i + 1] == '"') {
                fields.back().push_back(c);
                ++i;
            } else {
                quoted = !quoted;
            }
        } else if (c == ',' && !quoted) {
            fields.emplace_back();
        } else {
            fields.back().push_back(c);
        }
    }
    Require(!quoted, "unterminated CSV quote");
    return fields;
}

class Fixture {
public:
    Fixture() {
        const auto nonce = std::chrono::steady_clock::now().time_since_epoch().count();
        directory_ = fs::temp_directory_path() / ("rl_inference_logging_" + std::to_string(nonce));
        Require(fs::create_directory(directory_), "cannot create test directory");
        std::ofstream model(directory_ / "identity.onnx", std::ios::binary);
        model.write(reinterpret_cast<const char *>(kIdentityModel), sizeof(kIdentityModel));
        Require(model.good(), "cannot write test model");
    }

    ~Fixture() {
        std::error_code error;
        fs::remove_all(directory_, error);
    }

    const fs::path &Directory() const { return directory_; }

    fs::path WriteConfig(bool debug) const {
        const auto path = directory_ / "config.yaml";
        std::ofstream yaml(path);
        yaml << "robot_base:\n"
            "  name: inference_logging_test\n"
            "logging:\n"
            "  level: " << (debug ? "debug" : "info") << "\n"
            "  directory: logs\n"
            "  console: {enabled: false}\n"
            "  file: {enabled: true}\n"
            "  telemetry: {enabled: " << (debug ? "true" : "false") << "}\n"
            "rl_policy:\n"
            "  type: onnx_infer\n"
            "  rl_dt: 0.02\n"
            "  onnx_infer:\n"
            "    policies:\n"
            "      probe:\n"
            "        model_path: identity.onnx\n"
            "        runtime: {provider: cpu}\n"
            "        model_io:\n"
            "          inputs: [{name: obs, source: observation}]\n"
            "          outputs: [{name: action, target: action}]\n"
            "        action_scale: [1.0]\n"
            "        rl_default_pos: [0.0, 0.0, 0.0]\n"
            "        kp: [1.0, 1.0, 1.0]\n"
            "        kd: [0.1, 0.1, 0.1]\n"
            "        observation:\n"
            "          segment_0_terms: [ang_vel]\n";
        Require(yaml.good(), "cannot write test config");
        return path;
    }

private:
    fs::path directory_;
};

class Observer final : public behavior_manager::RLRuntimeObserver {
public:
    void OnRuntimeInitialized(const rl_policy::InferenceRuntimeInfo &) override {}

    void OnRuntimeEvent(const RLRuntimeEvent &event) noexcept override {
        if (event.type == RLRuntimeEventType::ACTION_PUBLISHED) {
            published.fetch_add(1, std::memory_order_release);
        }
    }

    std::atomic<int> published{0};
};

void CheckCsv(const fs::path &path, bool timeout) {
    std::ifstream input(path);
    Require(input.good(), "policy CSV is missing");
    std::string header, row, extra;
    Require(static_cast<bool>(std::getline(input, header)), "CSV header is missing");
    Require(static_cast<bool>(std::getline(input, row)), "inference frame was not recorded");
    Require(!std::getline(input, extra), "unexpected duplicate inference frame");
    const auto names = ParseCsvRow(header);
    const auto values = ParseCsvRow(row);
    Require(names.size() == values.size(), "CSV column count mismatch");
    std::map<std::string, std::string> fields;
    for (std::size_t i = 0; i < names.size(); ++i) fields.emplace(names[i], values[i]);
    Require(fields.at("result") == (timeout ? "inference_timeout" : "published"),
        "incorrect inference result");
    Require(fields.at("release_sequence") == "1", "incorrect release sequence");
    Require(fields.at("request_sequence") == "17" && fields.at("request_operation") == "1" &&
        fields.at("request_action") == "gesture,\"test\"", "interaction context was corrupted");
    double stage_sum = 0.0;
    for (const auto *name : {"snapshot_ms", "prepare_inputs_ms", "assemble_obs_ms",
            "policy_infer_ms", "postprocess_ms"}) {
        const double value = std::stod(fields.at(name));
        Require(std::isfinite(value) && value >= 0.0, std::string("invalid stage timing: ") + name);
        stage_sum += value;
    }
    Require(std::abs(stage_sum - std::stod(fields.at("inference_ms"))) < 1.0e-5,
        "stage timings do not cover the inference interval");
    Require(std::abs(std::stod(fields.at("release_to_finish_ms")) - stage_sum -
        std::stod(fields.at("release_to_start_ms"))) < 1.0e-5, "total inference timing mismatch");
    Require(std::isfinite(std::stod(fields.at("inference_thread_cpu_ms"))),
        "thread CPU time is missing");
    for (const auto *name : {"action_published_time_s", "finish_to_publish_ms",
            "release_to_publish_ms"}) {
        Require(std::isnan(std::stod(fields.at(name))) == timeout,
            std::string("incorrect publication timing: ") + name);
    }
    if (timeout) {
        Require(std::stod(fields.at("release_to_finish_ms")) >
            std::stod(fields.at("inference_deadline_ms")), "deadline was not exceeded");
    }
    Require(std::abs(std::stod(fields.at("raw_action_0")) - 0.2) < 1.0e-6,
        "failed frame action payload is missing");
}

void TestLogging(bool timeout, bool debug) {
    const Fixture fixture;
    const auto yaml_path = fixture.WriteConfig(debug);
    auto config = behavior_manager::LoadRLStateConfig(
        yaml_path.string(), "probe", fixture.Directory().string());
    // A one-nanosecond budget deterministically rejects a completed inference.
    config.inference_deadline_s = timeout ? 1.0e-9 : 0.0;
    Observer observer;
    config.runtime_observer = &observer;
    fs::path log_directory;
    {
        const auto yaml = robot_base::YamlFile::Load(yaml_path.string());
        runtime_logging::Session logging(yaml, yaml_path.string(), "control", false);
        log_directory = runtime_logging::GetSessionDirectory();
        robot_base::RobotData sensor;
        sensor.num_dof = 3;
        sensor.joint_pos = config.policy.rl_default_pos;
        sensor.joint_vel.assign(3, 0.0);
        sensor.gyro = {0.2, -0.1, 0.3};
        sensor.base_quat = {1.0, 0.0, 0.0, 0.0};
        robot_base::Command command;
        behavior_manager::ControlOutput output;
        behavior_manager::FSM fsm;
        auto state = behavior_manager::CreateStateRl(config);
        auto *rl_state = state.get();
        fsm.AddState(StateName::POWER_OFF, behavior_manager::CreateStatePowerOff());
        fsm.AddState(StateName::RL, std::move(state));
        fsm.SetDataPointers(&sensor, &command, &output);
        fsm.Init();
        fsm.ForceSwitch(StateName::RL, "inference logging regression");
        command.interaction.sequence = 17;
        command.interaction.operation = robot_base::InteractionRequest::Operation::START;
        command.interaction.action = "gesture,\"test\"";
        rl_state->Run(0.02F, 0.02F);
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        while (!rl_state->CurrentFault().latched && observer.published.load() == 0 &&
                std::chrono::steady_clock::now() < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        const auto fault = rl_state->CurrentFault();
        if (timeout) {
            Require(fault.latched && fault.code == robot_base::FaultCode::INFERENCE_TIMEOUT,
                "expected inference timeout was not latched");
            Require(rl_state->CheckTransition() == StateName::SAFETY,
                "timeout no longer requests the safety state");
            Require(observer.published.load() == 0 && !output.enable,
                "late action was published or applied");
        } else {
            Require(!fault.latched && observer.published.load() == 1,
                "successful action was not published");
            rl_state->Run(0.001F, 0.02F);
            Require(output.enable, "successful action was not applied");
        }
        fsm.ForceSwitch(StateName::POWER_OFF, "inference logging regression complete");
    }
    if (debug) {
        CheckCsv(log_directory / "control_policy_probe.csv", timeout);
        Require(fs::exists(log_directory / "control_policy_apply_probe.csv") == !timeout,
            "incorrect applied-action trace for rejected inference");
    } else {
        Require(!fs::exists(log_directory / "control_policy_probe.csv"),
            "debug trace was emitted with telemetry disabled");
    }
    if (timeout) {
        const auto events = ReadText(log_directory / "events.log");
        for (const auto *field : {"RL inference missed its release-to-finish deadline",
                "policy=\"probe\"", "release_sequence=1", "total_ms=", "deadline_ms=",
                "wait_ms=", "snapshot_ms=", "prepare_inputs_ms=", "assemble_obs_ms=",
                "policy_infer_ms=", "postprocess_ms=", "inference_thread_cpu_ms=",
                "interaction_action=", "request_sequence=17"}) {
            Require(events.find(field) != std::string::npos,
                std::string("fault diagnostics missing: ") + field);
        }
    }
}

}  // namespace

int main() {
    try {
        TestLogging(false, true);
        TestLogging(true, true);
        TestLogging(true, false);
        std::cout << "StateRL inference logging regression passed\n";
        return 0;
    } catch (const std::exception &error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
