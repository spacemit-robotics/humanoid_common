# transport — 统一传输模块

跨进程通信接口，负责机器人状态、控制命令、HMI 命令的收发。完全由 YAML 配置驱动，与具体通信方式无关。通过 `transport.type` 字段在 UDP 和 SHM 之间切换，应用代码无需修改。

## 接口说明

### 枚举

#### `Role` — 传输角色

| 角色 | 发送 | 接收 |
|------|------|------|
| `DRIVER` | 状态（RobotData） | 控制命令（ControlCmd） |
| `CONTROL` | 控制命令（ControlCmd） | 状态（RobotData）+ 命令（Command） |
| `HMI` | 命令（Command） | — |

### `TransportBase` — 传输接口

| 方法 | 说明 |
|------|------|
| `Create(yaml_path)` | 工厂函数，根据 YAML `transport.type` 创建实例 |
| `Init(yaml_path, role)` | 初始化，指定角色，返回是否成功 |
| `SendState(state)` | 发送机器人状态（DRIVER 使用）|
| `RecvState(state)` | 接收机器人状态（CONTROL 使用）|
| `SendControl(cmd)` | 发送控制命令（CONTROL 使用）|
| `RecvControl(cmd)` | 接收控制命令（DRIVER 使用）|
| `SendCommand(cmd)` | 发送行为命令（HMI 使用）|
| `RecvCommand(cmd)` | 接收行为命令（CONTROL 使用）|

## 依赖

- `robot_base`：RobotData、ControlCmd、Command 数据结构
- 系统库：librt（SHM 需要）

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
./output/staging/bin/test_transport_executor application/native/humanoid_unitree_g1/config/g1.yaml
```

### 使用示例

参考 [`../../example/transport_executor/test_transport_executor.cpp`](../../example/transport_executor/test_transport_executor.cpp) 了解完整的三角色收发演示。

## 说明

### YAML 配置

参考 [`../../example/transport_executor/config_example.yaml`](../../example/transport_executor/config_example.yaml) 了解完整配置。

```yaml
transport:
  type: udp   # 或 shm

  udp:
    driver_ip: 127.0.0.1
    control_ip: 127.0.0.1
    state_port: 8800
    control_port: 8801
    hmi_port: 8802

  shm:
    prefix: "robot1"
    capacity: 4
    slot_size: 4096
```

切换传输方式只需修改 `type` 字段，应用代码无需任何改动。

### 传输方式对比

| 特性 | UDP | SHM |
|------|-----|-----|
| 延迟 | ~50-100 μs | ~1-5 μs |
| 适用频率 | 50-500 Hz | 1000Hz+ |
| 适用场景 | 跨机器通信 | 单机多进程 |

### 架构

```
src/transport/
├── transport_executor/    # 对外唯一接口模块（libtransport_executor.so）
├── udp/                   # UDP 底层实现（编译进 so，不对外暴露）
└── shm/                   # SHM 底层实现（编译进 so，不对外暴露）
```

`udp` 和 `shm` 的源码直接编译进 `libtransport_executor.so`，不生成独立 `.so`，不对外暴露头文件。
