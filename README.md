# humanoid_common — 人形机器人通用控制层

## 项目简介

人形机器人通用控制层，提供三进程（driver / control / hmi）的可执行程序以及它们共用的三个基础库：

- `robot_base` — 统一数据结构与 YAML 配置解析
- `behavior_manager` — FSM 行为管理（POWER_OFF / DAMP / ZERO / RL / SAFETY）
- `transport` — 跨进程通信（UDP / SHM 可切换）

所有机型（g1、asimov、tinker、tiangong、qinglong、go1）共用本模块的二进制，机型差异由各机型仓库的 `config/<robot>.yaml` 描述。

## 功能特性

支持：
- FSM 完整控制流程（POWER_OFF / DAMP / ZERO / RL / SAFETY 状态切换）
- sim2sim 模式（跳过 FSM，直接 RL 推理，用于算法验证）
- SHM / UDP 两种通信后端（同机 / 跨机均可）
- x86_64 与 riscv64（K3 板卡）双平台编译

不支持：
- 硬件驱动（driver_runtime 当前仅对接 MuJoCo 仿真器）
- 在线训练或策略更新

## 快速开始

### 环境准备

```bash
sudo apt install -y libeigen3-dev libyaml-cpp-dev
```

本模块还依赖 SDK 内部组件 `model_zoo/rl` 和 `simulation/mujoco`（x86_64），需先完成它们的编译安装。

### 构建编译

**SDK 内编译（mm）**：

```bash
source build/envsetup.sh
mm
```

编译产物安装到 `output/staging/`：
- `lib/`：`librobot_base.so`、`libbehavior_manager.so`、`libtransport_executor.so`
- `bin/`：`driver_runtime`（x86_64）、`control_runtime`、`hmi_runtime`、`control_sim2sim_runtime`

**独立 cmake 编译**（需确保 rl、mujoco 两个组件均经过 mm 编译、产物已安装到 `output/staging/`）：

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

PR 档含 `humanoid-common-functional`（robot_base / behavior_manager / transport_executor 从 example 配置跑通核心流程 + 维度断言）与 `humanoid-common-error-path`（坏/缺配置快速失败），均不依赖硬件。

## 详细使用

### 三进程架构

```
 ┌───────────┐   HMI cmd    ┌───────────────┐   control cmd   ┌──────────────┐
 │  hmi      │ ───────────▶ │  control      │ ──────────────▶ │  driver      │
 │  _runtime │              │  _runtime     │                 │  _runtime    │
 └───────────┘              │  (FSM + RL)   │ ◀────────────── │  (MuJoCo /   │
                            └───────────────┘   robot state   │   硬件驱动)  │
                                                              └──────────────┘
```

### PD 增益数据流

kp/kd 在不同阶段由不同来源提供，配置上分散在两个 yaml 节点：

- **driver 启动期**：`driver_runtime` 读取 `robot_base.kp/kd/default_joint_pos`，作为 MuJoCo 仿真的初始 PD 增益与默认站立姿态。
- **RL 控制期**：进入 RL 状态后，control 端每帧通过 `ControlCmd.kp/kd` 下发当前策略训练时的真实增益（来自 `rl_policy.policies.<name>.kp/kd`），driver 端原样转发给 MuJoCo，不再使用启动期的默认值。
- **ZERO/DAMP 阶段**：ZERO 用当前策略的 `rl_default_pos` + `kp/kd`（无策略时回退到 `behavior_manager.zero_pos`，kp/kd 由 robot_base 兜底）；DAMP 用 `behavior_manager.damp_kd`（≈ policy kd / 5）。

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

调度流程：HMI 切 `dance` → behavior_manager 命中 map → 内部先把 active 切到 `stand`，进 RL 跑满 2s 后自动切到 `dance`（StateRL 会重新 OnEnter 重置 LSTM）。HMI 单次按键，用户无感。详见 [`src/behavior_manager/README.md`](src/behavior_manager/README.md)。

### Motion tracking 策略

`tracking` 策略与 `dance` / `kungfu` 是不同的 motion mimic 范式：

| | dance/kungfu | tracking (unitree_rl_mjlab) |
|---|---|---|
| 训练来源 | RoboMimic_Deploy（1D phase 范式） | unitree_rl_mjlab（多维参考量范式） |
| obs 维度 | 380（1D phase + history × 4） | 160（多维参考 + 单帧） |
| motion 数据 | 烘进 actor 权重 | 外挂 npz 文件（cnpy 运行时加载） |
| anchor 计算 | 不需要 | 需要（torso 相对位姿） |

实现路径（**rl 层泛型扩展，零业务字眼**）：

1. **rl 层**：`ObsTermCalculator` 把 1D `SetCustomScalar` 扩到 N D `SetCustomArray`，未识别 term 名走 `custom_arrays_` 查表 memcpy（[components/model_zoo/rl/src/obs_term.h](../../components/model_zoo/rl/src/obs_term.h)）。yaml 通过 `custom_array_dims: {name: dim}` 声明 N 维 term 维度。
2. **common 层**：新增 `MotionTrackingHelper`（[src/behavior_manager/motion_tracking_helper.h](src/behavior_manager/motion_tracking_helper.h)）封装 cnpy npz 加载、yaw 对齐、anchor 计算，state_rl 在 OnEnter / InferStep 各插桩调用，把 `motion_command(58) / motion_anchor_pos_b(3) / motion_anchor_ori_b(6)` 通过 `policy.SetCustomArray(...)` 推给 rl。
3. **应用层**：g1.yaml 中 tracking 策略段加 `motion_file / motion_fps / anchor_body_name / anchor_yaw_align` 四个 tracking-specific 字段 + `custom_array_dims` 声明。

motion 播完后冻结在末帧（帧索引 clamp 到最后一帧）保持，不自动回 ZERO；需切换状态由 HMI/control 手动触发（POWER_OFF / DAMP 等）。第一次进 tracking 时会按机器人当前 yaw vs npz 第 0 帧 yaw 做一次性对齐。

### ControlMode 数据流（control → driver）

`ControlCmd.mode` 是 control 端发给 driver 的通用控制语义字段（`enum class ControlMode { POWER_OFF, DAMP, ZERO, RL, SAFETY }`），driver 据此自主决定后端行为：

- **三进程 FSM 模式**：`control_runtime` 把 `bm.CurrentState()` 透传到 `ctrl.mode`
- **sim2sim 模式**：`control_sim2sim_runtime` 始终发 `ControlMode::RL`
- **mujoco driver**：边沿检测 mode 变化—— `RL` → 自动取消悬挂保护，`POWER_OFF` → 自动启用悬挂；其余 mode 不主动覆盖（保留 mujoco 界面 F 键的手动权限）
- **实机 driver**：可忽略此字段，或用于状态阈值/恢复策略/遥测打点

设计原则：跨层接口字段必须用通用语义，禁止携带某一具体后端（mujoco 悬挂等）的私有概念。

### hmi_runtime 键盘操作

hmi_runtime 使用 ANSI 备用屏幕缓冲区实现全屏 TUI，策略列表自动分行显示。

| 按键 | 动作 |
| --- | --- |
| `f` | 切换到 POWER_OFF（完全失力） |
| `o` | 切换到 DAMP（阻尼保持） |
| `z` | 切换到 ZERO（回零位） |
| `r` | 切换到 RL（进入 RL 控制） |
| `1`~`9` | 切换到对应序号的策略 |
| `w/s` | 增减 vx |
| `a/d` | 增减 vy |
| `q/e` | 增减 wz |
| `空格` | 速度清零 |

### 通信配置

通过 YAML 中 `transport.type` 字段选择后端：

- `"shm"`：共享内存，同机高性能，默认值
- `"udp"`：跨机通信或同机隔离进程，需填写 `driver_ip` / `control_ip`

详见 [`src/transport/README.md`](src/transport/README.md)。

## 常见问题

| 现象 | 处理 |
| --- | --- |
| cmake 报 `rl not found` | 先编译安装 `components/model_zoo/rl`，确认 `--prefix` 路径一致 |
| cmake 报 `mujoco_sim not found` | 先编译安装 `components/simulation/mujoco`（仅 x86_64 需要） |
| control_runtime 启动后无数据 | 检查 YAML 中 `transport.type` 及 IP 配置，确认 driver/control 两端可互相 ping 通 |
| K3 板卡上 driver_runtime 不存在 | 正常现象，driver_runtime 仅在 x86_64 上编译 |

## 版本与发布

| 版本 | 说明 |
| --- | --- |
| 0.1.0 | 初始版本，支持 FSM 完整流程与 sim2sim，SHM/UDP 双后端 |

## 贡献方式

贡献者与维护者名单见：`CONTRIBUTORS.md`

## License

本仓库源码文件头声明为 Apache-2.0，最终以本目录 `LICENSE` 文件为准。
