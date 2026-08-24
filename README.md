# humanoid_common — 人形机器人通用控制层

## 项目简介

人形机器人通用控制层，提供三进程（driver / control / hmi）的可执行程序以及它们共用的三个基础库：

- `robot_base` — 统一数据结构与 YAML 配置解析
- `behavior_manager` — FSM 行为管理（POWER_OFF / DAMP / HOME / ZERO / RL / SAFETY）
- `transport` — 跨进程通信（UDP / SHM 可切换）

所有机型（g1、asimov、tinker、tiangong、qinglong、go1）共用本模块的二进制，机型差异由各机型仓库的 `config/<robot>.yaml` 描述。

## 功能特性

支持：
- FSM 完整控制流程（POWER_OFF / DAMP / HOME / ZERO / RL / SAFETY 状态切换）
- sim2sim 模式（跳过 FSM，直接 RL 推理，用于算法验证）
- SHM / UDP 两种通信后端（同机 / 跨机均可）
- `driver_runtime` 通过 YAML 选择 MuJoCo 或 `whole_body` backend
- x86_64 与 riscv64（K3 板卡）双平台编译

不支持：
- 在线训练或策略更新

## 快速开始

### 环境准备

```bash
sudo apt install -y libeigen3-dev libyaml-cpp-dev
```

本模块还依赖 SDK 内部组件 `model_zoo/rl`。`simulation/mujoco` 和
`control/whole_body` 分别提供可选 driver backend；至少找到其中一个时才构建
`driver_runtime`。

### 构建编译

**SDK 内编译（mm）**：

```bash
source build/envsetup.sh
mm
```

编译产物安装到 `output/staging/`：
- `lib/`：`librobot_base.so`、`libbehavior_manager.so`、`libtransport_executor.so`
- `bin/`：`driver_runtime`、`control_runtime`、`hmi_runtime`、`control_sim2sim_runtime`

**独立 cmake 编译**（需确保 rl 和所需 driver backend 已安装到 `output/staging/`）：

```bash
cd application/native/humanoid_common
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/spacemit_robot/output/staging
make
```

### 运行示例

启动脚本位于对应机型仓库（如 `humanoid_unitree_g1/scripts/`），编译安装后进入 PATH，以 g1 为例：

**FSM 完整仿真（3 个终端）：**

```bash
run_driver_g1.sh    # 终端1（PC，x86_64）
run_control_g1.sh   # 终端2（K3 板卡）
run_hmi_g1.sh       # 终端3（K3 板卡）
```

**sim2sim（2 个终端）：**

```bash
run_driver_g1.sh    # 终端1（PC）
run_sim2sim_g1.sh   # 终端2（K3 板卡）
```

### CI 测试

模块自带 `test.yaml`（CI 用例清单）+ `tests/`，经 SDK 根目录的 `robot-test` 运行：

```bash
scripts/test/robot-test list application/native/humanoid_common
scripts/test/robot-test run  application/native/humanoid_common --scope pr
```

PR 档含 `humanoid-common-functional`（前三个公共模块跑通离线流程，driver_runtime
验证后端解析、能力检测和 factory 分派）与
`humanoid-common-error-path`（坏/缺配置快速失败），均不依赖硬件。

## 详细使用

### 三进程架构

```
 ┌───────────┐   HMI cmd    ┌───────────────┐   control cmd   ┌──────────────┐
 │  hmi      │ ───────────▶ │  control      │ ──────────────▶ │  driver      │
 │  _runtime │ ◀─────────── │  _runtime     │ ◀────────────── │  _runtime    │
 └───────────┘ ControlStatus│  (FSM + RL)   │   robot state   │  (MuJoCo /   │
                            └───────────────┘                 │ whole_body)  │
                                                              └──────────────┘
```

### Driver backend

机型 YAML 通过以下字段选择 driver：

```yaml
driver:
  backend: mujoco       # 或 whole_body
```

旧配置缺少该字段时默认选择 `mujoco`。`driver_runtime` 只负责 transport、信号和
backend 生命周期；MuJoCo 与整机硬件细节分别封装在私有 adapter 中，不进入 common
公共 API。选择了当前构建未包含的 backend 时，进程会在启动阶段明确失败。

`RobotData` 在原位置/速度基础上携带关节力矩、温度、错误码和 IMU 加速度；
`ControlCmd` 携带执行器模式及位置、速度、力矩、KP/KD 命令。力矩向量为空时
按全零处理。SHM/UDP 使用同一版二进制协议，协议版本不匹配会拒绝数据包，
不会静默按旧布局解析。

### Runtime 日志

日志是 common 内部能力，不进入公开头文件，也不扩展 `RobotData` 或 SHM 协议。
机型 YAML 可按需启用：

```yaml
logging:
  level: debug
  directory: "../../../../log/humanoid"  # 相对当前机型 YAML
  console:
    enabled: true
  file:
    enabled: true
    max_size_mb: 32
    max_files: 3
  queue_capacity: 8192
  telemetry:
    enabled: true
    rate_hz: 20
  driver_monitor:
    enabled: true
    rate_hz: 2
```

未配置时保留原控制行为且不落盘。启用后，每个 runtime 建立独立 session 目录：
`events.log` 记录生命周期、FSM、策略和错误；driver 记录物理电机、虚拟关节和 IMU
CSV；control 记录控制状态以及关节目标/反馈 CSV。driver 终端以固定屏限频刷新，
不会按控制频率刷屏。落盘由有界后台队列完成，磁盘跟不上时丢弃日志并记录丢弃数，
不会阻塞控制循环。

### 执行器模式

`ControlMode` 和 `ActuationMode` 是两个正交的语义层：

- `ControlMode` 表示 `POWER_OFF/DAMP/HOME/ZERO/RL/SAFETY` 行为与安全状态。
- `ActuationMode` 表示整条命令中哪些字段是执行器的主目标。

| `ActuationMode` | 命令语义 |
| --- | --- |
| `HYBRID` | 阻抗控制：`kp * (target_pos - q) + kd * (target_vel - dq) + target_torque`，其中 `target_torque` 是前馈项 |
| `POSITION` | 位置闭环，`target_pos` 为主目标 |
| `VELOCITY` | 速度闭环，`target_vel` 为主目标 |
| `TORQUE` | 力矩控制，`target_torque` 是直接目标力矩 |

当前 FSM 和已有 RL 策略均输出 `HYBRID`，且前馈力矩默认为零，因此不改变
现有机型行为。whole_body backend 会将该模式映射到 motor 组件；MuJoCo backend
当前仅接受 `HYBRID/POSITION`，且不支持非零前馈力矩，传入时会明确报错。

### PD 增益数据流

kp/kd 在不同阶段由不同来源提供，配置上分散在两个 yaml 节点：

- **MuJoCo 启动期**：MuJoCo backend 读取 `robot_base.kp/kd/default_joint_pos`，作为仿真的初始 PD 增益与默认站立姿态。
- **HOME 阶段**：从当前实机关节位置先平滑建立增益，再恢复到 `robot_base.default_joint_pos`；可用 `behavior_manager.home` 单独配置时长和 kp/kd。
- **RL 控制期**：进入 RL 状态后，control 端每帧通过 `ControlCmd.kp/kd` 下发当前策略训练时的真实增益（来自 `rl_policy.policies.<name>.kp/kd`），driver backend 转发给实际执行组件。
- **ZERO/DAMP 阶段**：ZERO 优先用当前策略的可选 `zero_target_pos`，未配置时用 `rl_default_pos`；机型可配置独立 ZERO 安全增益，进入 RL 后立即使用当前策略增益。未配置 ZERO 增益的已有机型保持原有策略增益行为；DAMP 使用 `behavior_manager.damp_kd`。

### 策略链调度（prerequisite chain）

某些 RL 策略需要先经过另一策略热身才能稳定运行（典型：`dance` / `kungfu` 必须先用 LocoMode `stand` 站稳并预热 LSTM，否则从 PD 锁位的 ZERO 直接切动态动作会摔）。在 yaml 中给目标策略加可选 `prerequisite` 子节点即可：

```yaml
rl_policy:
  onnx_infer:
    policies:
      stand:  { ... }                          # 前置策略，正常配置
      dance:
        ...                                    # 已有字段保留
        prerequisite:
          policy: stand                        # 前置策略名
          duration: 2.0                        # 前置运行时长（秒）
```

调度流程：HMI 选择 `dance` → behavior_manager 命中 map → 内部先把 active 切到 `stand`，进 RL 跑满 2s 后自动切到 `dance`（StateRL 会重新 OnEnter 重置 LSTM）。HMI 单次确认，用户无感。详见 [`src/behavior_manager/README.md`](src/behavior_manager/README.md)。

### Motion tracking 策略

需要外置参考动作或特殊模型输入的策略，在机型 YAML 的策略段配置
`policy_adapter`。它是 `behavior_manager` 的内部子模块，不是与
`behavior_manager` 平级的新 common 模块：

1. `model_zoo/rl` 只负责通用 ONNX I/O、feedback tensor 和观测项组装。
2. [`policy_adapter`](src/behavior_manager/policy_adapter/) 负责参考动作读取、时间轴、朝向对齐，以及 HoloMotion / ProtoMotions / SONIC 的输入协议。
3. 应用层机型仓库只提供模型、参考动作和 YAML 参数，不放 tracker C++ 代码。

机器人自由度取自策略 `rl_default_pos`，模型关节顺序取自
`action_joint_index`，自定义观测维度取自 `custom_array_dims`；这些机型和
模型参数均由应用层 YAML 传入，不在 common 中固定。

```yaml
policy_adapter:
  type: mjlab                 # mjlab / holomotion / protomotions / sonic
  reference_file: policy/example/motion.npz
  motion_fps: 50
  playback_speed: 1.0
  loop: false
  loop_pause: 0.0
  anchor_body_index: 0
  anchor_waist_joint_indices: []
  anchor_yaw_align: true
  # 可选：与 MJLab 的 reference + residual action 定义保持一致。
  reference_action:
    joint_indices: [0, 1]  # 示例；按应用层的机器人关节顺序配置
    residual_scale: 0.0
    residual_clip: 1.0
```

各类 adapter 的参考输入和未来窗口参数分别为：

- MJLab：`reference_file` 指向单个 NPZ；
- HoloMotion：`reference_file` 指向单个预处理 CSV，使用
  `future_frames` 和 `context_length`；
- ProtoMotions：`reference_file` 指向单个预处理 CSV，使用
  `future_steps`；
- SONIC：`reference_file` 指向参考动作目录；目录内必须包含
  `joint_pos.csv`、`joint_vel.csv` 和 `body_quat.csv`，使用
  `future_frames` 和 `future_step`。

策略若配置
`zero_target_pos`，ZERO 阶段先过渡到参考动作起始姿态。非循环动作播放完成后
保持末帧，状态切换仍由 HMI/control 负责。

MJLab 的 `reference_action` 未配置时保持标准策略行为。配置后，仅列出的机器人
关节使用 `参考关节角 + residual_scale * clip(原始模型 action)`；其余关节仍按
通用 `rl_default_pos + action_scale * action` 映射。关节列表属于应用层机型参数，
common 不固定自由度或手臂索引。

### ControlMode 数据流（control → driver）

`ControlCmd.mode` 是 control 端发给 driver 的通用控制语义字段（`enum class ControlMode { POWER_OFF, DAMP, ZERO, RL, SAFETY, HOME }`），driver 据此自主决定后端行为：

- **三进程 FSM 模式**：`control_runtime` 把 `bm.CurrentState()` 透传到 `ctrl.mode`
- **sim2sim 模式**：`control_sim2sim_runtime` 始终发 `ControlMode::RL`
- **mujoco driver**：边沿检测 mode 变化—— `RL` → 自动取消悬挂保护，`POWER_OFF` → 自动启用悬挂；其余 mode 不主动覆盖（保留 mujoco 界面 F 键的手动权限）
- **whole_body driver**：转换为整机控制模式，并由组件本地安全门控与 watchdog 约束

设计原则：跨层接口字段必须用通用语义，禁止携带某一具体后端（mujoco 悬挂等）的私有概念。

### hmi_runtime 键盘操作

hmi_runtime 使用带颜色的 ANSI 全屏 TUI。FSM、当前策略、Control 实际采用速度和
RL 频率均来自 control 端回传，不在 HMI 本地预判。主界面用左右键切换相邻 FSM，
策略和速度分别使用独立子页面。

| 按键 | 动作 |
| --- | --- |
| `←/→` | 按真实 FSM 后退/前进；RL 按左键直接退到 DAMP |
| `f` | 全局请求 POWER_OFF（完全失力） |
| `p` | 打开策略选择页；`↑/↓` 或 `j/k` 移动，Enter 确认，Esc 返回 |
| `v` / `Enter` | 真实状态为 RL 且心跳正常时进入速度控制页 |
| `w/s` | 速度页内增减 vx |
| `a/d` | 速度页内增减 vy |
| `q/e` | 速度页内增减 wz |
| `空格` | 速度清零 |
| `Esc` / `v` | 退出速度页并清零 |

`o/h/z/r` 保留为兼容快捷键，但合法性仍由 control 端 FSM 校验。HMI 以配置频率发送
心跳；退出速度页、离开 RL、HMI 退出或心跳超时都会清零速度。按键步长和通信超时在
YAML 的 `hmi` 节点配置；速度范围由应用层在每个策略的
`command.limits.{max_vx,max_vy,max_wz}` 中声明。未声明范围的策略不接受速度命令。

### 通信配置

通过 YAML 中 `transport.type` 字段选择后端：

- `"shm"`：共享内存，同机高性能，默认值
- `"udp"`：跨机通信或同机隔离进程，需填写 `driver_ip` / `control_ip` /
  `hmi_ip`，状态回传使用独立 `status_port`

详见 [`src/transport/README.md`](src/transport/README.md)。

## 常见问题

| 现象 | 处理 |
| --- | --- |
| cmake 报 `rl not found` | 先编译安装 `components/model_zoo/rl`，确认 `--prefix` 路径一致 |
| `driver_runtime` 未生成 | 先编译安装 `components/simulation/mujoco` 或 `components/control/whole_body` |
| YAML 选择的 backend unavailable | 编译对应组件后重新编译 `humanoid_common` |
| control_runtime 启动后无数据 | 检查 YAML 中 `transport.type` 及 IP 配置，确认 driver/control 两端可互相 ping 通 |

## 版本与发布

| 版本 | 说明 |
| --- | --- |
| 0.1.0 | 初始版本，支持 FSM 完整流程与 sim2sim，SHM/UDP 双后端 |

## 贡献方式

贡献者与维护者名单见：`CONTRIBUTORS.md`

## License

本仓库源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
