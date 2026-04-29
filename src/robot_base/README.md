# robot_base — 机器人基础状态模块

提供统一的机器人状态数据结构、YAML 配置解析引擎和工具函数，供所有模块共用。

## 1. 接口说明

本模块对外接口分为配置解析、状态管理和工具函数三大类。
各接口的详细使用范例与集成测试，请参考：**[../../example/robot_base/test_robot_base.cpp](../../example/robot_base/test_robot_base.cpp)**

### 1.1 `YamlFile` (配置解析引擎)

提供类型安全的配置读取能力，隔离第三方库依赖。

| 接口名称 | 参数类型 | 返回值 | 功能说明 |
| :--- | :--- | :--- | :--- |
| `Load` | `const std::string &yaml_path` | `YamlFile` | 静态方法，加载并解析指定路径的 YAML 配置文件 |
| `Read<T>` | `const std::string &path` | `std::optional<T>` | 模板方法，根据层级路径（如 `a.b[0].c`）读取配置。路径不存在时返回 nullopt |
| `ToAbsPath` | `const std::string &path` | `std::string` | 将相对路径转换为基于当前 YAML 文件所在目录的绝对路径 |

`Read<T>` 支持的模板类型：`int`、`double`、`bool`、`std::string`、`std::vector<int>`、`std::vector<double>`、`std::vector<std::string>`

### 1.2 `RobotData` (机器人核心状态)

各模块之间传递物理状态和传感器数据的核心契约。

| 接口名称 | 参数类型 | 返回值 | 功能说明 |
| :--- | :--- | :--- | :--- |
| `FromYaml` | `const std::string &yaml_path` | `RobotData` | 静态工厂方法，从 YAML 中读取自由度数量等信息并创建状态实例 |
| `Create` | `int num_dof` | `RobotData` | 静态工厂方法，通过指定自由度数量直接创建状态实例 |
| `UpdateImuData`| `void` | `void` | 根据当前填充的 `base_quat` (姿态) 和 `base_vel` (速度) 更新 rpy 和 gyro 数据 |
| `Reset` | `void` | `void` | 清零所有物理状态数据，但不改变 `num_dof` (自由度数量) |
| `IsValid` | `void` | `bool` | 校验内部向量容器（如位置、速度）的尺寸是否与 `num_dof` 严格匹配 |

### 1.3 `ThreadLoop` (线程管理)

统一的线程调度配置与生命周期管理，供所有模块使用。

配置字段为 public，`Start()` 前直接赋值；`Apply()` 可单独调用以配置调用线程自身（如 application 主线程）。

支持拷贝构造/赋值，**仅复制配置字段**（`name/cpu_id/sched/priority`），不复制线程运行状态，因此可作为配置载体嵌入其他可拷贝结构体。

| 接口 | 说明 |
| :--- | :--- |
| `name` | 线程名（最长 15 字符，用于 `ps`/`htop` 中识别） |
| `cpu_id` | CPU 亲和性，`-1` 不绑定 |
| `sched` | 调度策略：`"other"`（CFS 默认）\| `"fifo"` \| `"rr"` |
| `priority` | 优先级（`fifo`/`rr`: 1~99；`other` 忽略） |
| `FromYaml(yaml, name)` | 静态工厂：从 `robot_base.threads.{name}` 加载配置，返回未启动的 ThreadLoop |
| `Apply()` | 对调用线程应用当前配置（`fifo`/`rr` 需 root 权限）；`Start()` 内部自动调用 |
| `Start(func)` | 启动后台线程，循环执行 `func`，返回 `false` 时退出 |
| `Stop()` | 停止线程（阻塞等待） |
| `IsRunning()` | 查询线程是否运行中 |
| 拷贝构造/赋值 | 仅复制配置字段，新对象处于未启动状态 |

YAML 配置位于 `robot_base.threads`，按线程名独立配置，各模块按名读取对应条目：

```yaml
robot_base:
  threads:
    rl_infer:       # RL 推理线程
      cpu_id: 2
      sched: fifo   # other（默认）| fifo | rr（需 root）
      priority: 80
    control_main:   # control_demo 主控制循环
      cpu_id: 3
      sched: fifo
      priority: 90
    driver_main:    # driver_demo / hardware 主线程
      cpu_id: 4
      sched: fifo
      priority: 85
```

### 1.4 数据契约与工具函数

**`ControlCmd`** — 控制器下发到底层驱动/仿真的指令：

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `enable` | `bool` | 使能标志，`false` 时驱动器失力 |
| `target_pos` | `std::vector<double>` | 目标关节位置 (rad)，大小 = num_dof |
| `target_vel` | `std::vector<double>` | 目标关节速度 (rad/s)，大小 = num_dof |
| `kp` | `std::vector<double>` | PD 控制比例增益，大小 = num_dof |
| `kd` | `std::vector<double>` | PD 控制微分增益，大小 = num_dof |

**`Command`** — HMI 下发到控制层的上层指令：

| 字段 | 类型 | 说明 |
| :--- | :--- | :--- |
| `key` | `int` | 状态切换指令（1=DAMP, 2=ZERO, 3=RL, -1=POWER_OFF） |
| `vx` | `float` | 前进速度指令 (m/s) |
| `vy` | `float` | 横向速度指令 (m/s) |
| `wz` | `float` | 旋转角速度指令 (rad/s) |
| `switch_policy` | `std::string` | 策略切换请求，空字符串表示无切换 |

**工具函数：**

| 接口名称 | 参数类型 | 返回值 | 功能说明 |
| :--- | :--- | :--- | :--- |
| `QuatToRpy` | `const array<double,4>&, array<double,3>&` | `void` | 将四元数转换为欧拉角 (roll, pitch, yaw) |
| `RpyToQuat` | `const array<double,3>&, array<double,4>&` | `void` | 将欧拉角转换为四元数 |
| `NormalizeQuat`| `array<double,4>&` | `void` | 就地对四元数进行归一化处理 |

---

## 2. 依赖

- **标准库**: C++17 及以上
- **yaml-cpp**: 用于底层的 YAML 文件解析（已被 `YamlFile` 完全封装隔离，外部上层模块**无需**直接链接或引入其头文件）。

  安装命令：
  ```bash
  sudo apt install libyaml-cpp-dev
  ```

---

## 3. 快速开始

本模块提供了自包含的单元测试，可直接编译并运行，用以验证模块基础功能及接口正确性：

```bash
source ~/spacemit_robot/build/envsetup.sh
cd application/native/humanoid_common
mm

cd ~/spacemit_robot
./output/staging/bin/test_robot_base \
  application/native/humanoid_common/example/robot_base/config_example.yaml
```

---

## 4. 注意事项

1. **核心数据契约的维护**：
   本模块定义的 `RobotData`、`ControlCmd` 和 `Command` 是全 SDK 共通的数据基础。如需扩展或修改其中的字段，需同步排查并适配相关依赖模块（仿真、通信、FSM 等），以确保数据流的完整性。
2. **配置解析的最佳实践**：
   `YamlFile` 已经对底层的 yaml-cpp 进行了封装隔离。其他业务模块在读取配置时，优先使用 `YamlFile` 提供的类型安全接口。
