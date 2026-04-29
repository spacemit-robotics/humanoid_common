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

### hmi_runtime 键盘操作

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
