# behavior_manager — FSM 行为管理

有限状态机（FSM）的人形机器人行为控制模块。支持 5 种状态（POWER_OFF、DAMP、ZERO、RL、SAFETY），完全由 YAML 配置驱动，与机器人型号解耦。StateRL 集成 ONNX 推理，在独立线程中执行策略。

## 接口说明

### 枚举与数据结构

#### `StateName` — 状态枚举

| 状态 | 说明 |
|------|------|
| `POWER_OFF` | 完全失力（关节零力矩）|
| `DAMP` | 阻尼保持（kp=0, kd=配置值）|
| `ZERO` | 回零位（平滑到初始位置）|
| `RL` | RL 控制（异步 ONNX 推理）|
| `SAFETY` | 安全保护（IMU 超限触发）|

#### `robot_base::RobotData` — 传感器输入

| 成员 | 类型 | 说明 |
|------|------|------|
| `rpy` | `array<double, 3>` | IMU 欧拉角 (rad) |
| `gyro` | `array<double, 3>` | 角速度 (rad/s) |
| `joint_pos` | `vector<double>` | 关节位置 (rad) |
| `joint_vel` | `vector<double>` | 关节速度 (rad/s) |
| `time` | `double` | 时间戳 (s) |

#### `robot_base::Command` — 行为控制命令

| 成员 | 类型 | 说明 |
|------|------|------|
| `vx`, `vy`, `wz` | `float` | 速度指令（前进、横向、旋转）(m/s, rad/s) |
| `key` | `int` | 状态切换指令 |
| `switch_policy` | `string` | 策略切换请求（RL 状态），空字符串表示无切换 |

#### `ControlOutput` — 控制输出

| 成员 | 类型 | 说明 |
|------|------|------|
| `target_pos` | `vector<double>` | 目标关节位置 (rad) |
| `target_vel` | `vector<double>` | 目标关节速度 (rad/s) |
| `kp`, `kd` | `vector<double>` | PD 参数（每帧由当前 RL 策略提供）|
| `enable` | `bool` | 使能标志 |

### `BehaviorManagerClass` — 行为管理器

| 方法 | 说明 |
|------|------|
| `BehaviorManagerClass(config_path)` | 构造，加载 YAML 配置 |
| `Init()` | 初始化（加载配置、注册状态、初始化 FSM）|
| `Step(control_dt, rl_dt)` | 执行一步控制 |
| `SetSensorData(data)` | 设置传感器数据 |
| `SetCommand(cmd)` | 设置命令（状态切换、速度指令、策略切换）|
| `GetOutput()` | 获取控制输出 |
| `CurrentState()` | 获取当前状态 |
| `CurrentPolicyName()` | 获取当前 RL 策略名 |
| `GetRlFreq()` | 获取 RL 推理频率（Hz）|

**状态切换指令 (`Command.key`)：**

| key | 转移状态 | 说明 |
|-----|---------|------|
| `1` | POWER_OFF/ZERO/RL → DAMP | 进入/退回阻尼 |
| `2` | DAMP → ZERO | 开始回零 |
| `3` | ZERO → RL | 进入 RL（需插值完成）|
| `-1` | 任意 → POWER_OFF | 完全失力（ESC）|

## 依赖

- `robot_base`：共享数据结构（同仓库内）
- `rl`：ONNX 推理（来自 `components/model_zoo/rl`，从 `output/staging` 查找）
- 系统库：libyaml-cpp、libeigen3

## 快速开始

### 编译

```bash
source ~/spacemit_robot/build/envsetup.sh
cd application/native/humanoid_common
mm
```

### 运行测试

```bash
cd ~/spacemit_robot
./output/staging/bin/test_behavior application/native/humanoid_unitree_g1/config/g1.yaml
./output/staging/bin/test_behavior application/native/humanoid_qinglong/config/qinglong.yaml
```

### 使用示例

参考 [`../../example/behavior_manager/test_behavior.cpp`](../../example/behavior_manager/test_behavior.cpp) 了解完整的接口使用方法（演示状态切换、数据设置、输出获取）。

## 说明

### YAML 配置

模块读取 YAML 的 `behavior_manager` 和 `rl_policy` 节点。参考 `example/config_example.yaml` 了解配置结构和参数含义。

**关键参数：**
- `rl_policy.policies.<name>.kp/kd`：各策略训练时的 PD 增益（每帧随控制命令下发）
- `rl_policy.policies.<name>.prerequisite.policy / .duration`（可选）：前置策略链——切到本策略前先自动跑前置策略 `duration` 秒
- `behavior_manager.damp_kd`：阻尼状态 kd（≈ policy kd / 5）
- `behavior_manager.zero_pos`：回零位置（无 rl_policy 时的 fallback；完整 FSM 用 `rl_policy.policies.<name>.rl_default_pos`）
- `behavior_manager.zero_duration`：回零时间（秒）

**路径解析：** `robot_base.robot_dir` 相对于 YAML 文件；`model_path` 相对于 `robot_dir`。

### 状态机流程

```
POWER_OFF ──key=1──→ DAMP ──key=2──→ ZERO ──key=3(插值完成)──→ RL
    ↑                                                           │
    └─────────────────── key=-1(ESC) ─────────────────────────┘
                                                                │
                                          RL ──IMU超限──→ SAFETY
```

### 关键设计

- **异步推理：** StateRL 在独立线程执行 ONNX 推理，不阻塞主循环
- **自动模型检测：** 支持 MLP 和 LSTM（自动检测）
- **动态策略切换：** 仅在 `POWER_OFF` / `DAMP` 状态接受 `Command.switch_policy`；进入 `ZERO` 后策略锁定（ZERO 的目标位置/kp/kd 取自当前策略，切换会同步重建 ZERO 状态）
- **前置策略链调度：** 目标策略可在 yaml 配置 `prerequisite: { policy, duration }`，behavior_manager 收到切换请求后自动先切前置策略，在 RL 状态运行 `duration` 秒后再切目标策略；用户感知层面只发一次切换命令。典型场景：`dance` / `kungfu` 配 `prerequisite: stand`，先用 LocoMode 站稳并预热 LSTM，再进 dance/kungfu，避免直接从 PD 锁位的 ZERO 切动态动作时摔倒
- **安全保护：** IMU 倾角/关节限位自动触发安全状态
- **参数隔离：** damp_kd ≈ policy kd / 5（不同阶段刚度需求不同）
