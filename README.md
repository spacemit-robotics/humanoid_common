# humanoid_common — 人形机器人通用控制层

## 项目简介

人形机器人通用控制层，提供三进程（driver / control / hmi）及以下共享模块：

- `robot_base` — 统一数据结构与 YAML 配置解析
- `behavior_manager` — FSM 行为管理（POWER_OFF / DAMP / HOME / ZERO / RL / SAFETY）
- `transport` — 跨进程通信（UDP / SHM 可切换）
- `operator_service` — 终端、App、网页和手柄的操作请求、控制权与结果确认

所有机型（g1、asimov、tinker、tiangong、qinglong、go1）共用本模块的二进制，机型差异由各机型仓库的 `config/<robot>.yaml` 描述。

## 功能特性

支持：

- FSM 完整控制流程（POWER_OFF / DAMP / HOME / ZERO / RL / SAFETY 状态切换）
- sim2sim 模式（跳过 FSM，直接 RL 推理，用于算法验证）
- SHM / UDP 两种通信后端（同机 / 跨机均可）
- `driver_runtime` 通过 YAML 选择 MuJoCo 或 `whole_body` backend
- HMI 常驻服务与独立终端客户端；实机支持网页、Android 和本机手柄接入
- 可选离线音频播报，以及运行日志和故障前后数据采集
- x86_64 与 riscv64（K3 板卡）双平台编译

不支持：

- 在线训练或策略更新

## 快速开始

### 环境准备

```bash
sudo apt install -y libeigen3-dev libyaml-cpp-dev libboost-dev nlohmann-json3-dev
```

本模块还依赖 SDK 内部组件 `model_zoo/rl`。`simulation/mujoco` 和
`control/whole_body` 分别提供可选 driver backend；至少找到其中一个时才构建
`driver_runtime`。

cnpy、qrcodegen 和 Lucide 由 `cmake/` 自动获取固定版本，统一缓存到
`~/.cache/thirdparty/`；缓存和离线配置见 [第三方依赖](thirdparty/README.md)。

### 构建编译

**SDK 内编译（mm）**：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
cd application/native/humanoid_common
mm
```

编译产物安装到 `output/staging/`：
- `lib/`：`librobot_base.so`、`libbehavior_manager.so`、`libtransport_executor.so`、`liboperator_client.so`
- `bin/`：`driver_runtime`、`control_runtime`、`hmi_runtime`、`hmi_tui`、`control_sim2sim_runtime`、`hmi_gamepad`、可选 `operator_audio`
- `include/`：`operator_client.h`、`operator_types.h` 等公共接口
- `share/humanoid_common/operator_web/`：桌面和手机网页；已构建的 Android APK 随 `mm` 安装到 `downloads/`
- benchmark：`benchmark_humanoid_policy`、`run_benchmark_humanoid_policy.sh`

单配置生成器未指定 `CMAKE_BUILD_TYPE` 时默认使用 `Release`。StateRL runtime
benchmark 会执行本包中的控制、adapter 和异步推理路径；若本包是 O0 而 RL 库
是 O3，结果不能代表部署性能。

**独立 cmake 编译**（需确保 rl 和所需 driver backend 已安装到 `output/staging/`）：

```bash
cd application/native/humanoid_common
mkdir build && cd build
cmake .. -DCMAKE_INSTALL_PREFIX=$HOME/spacemit_robot/output/staging
make
```

### 运行示例

启动脚本位于对应机型仓库（如 `humanoid_unitree_g1/scripts/`），编译安装后进入 PATH，以 g1 为例：

**FSM 完整仿真：**

```bash
run_driver_g1.sh    # 终端1（PC，x86_64）
run_control_g1.sh   # 终端2（K3 板卡）
hmi_runtime /absolute/path/g1.yaml --sim # 终端3，仅接受本机 TUI
hmi_tui --connection FILE          # 可选终端客户端，FILE 使用服务启动时显示的路径
```

**sim2sim（2 个终端）：**

```bash
run_driver_g1.sh    # 终端1（PC）
run_sim2sim_g1.sh   # 终端2（K3 板卡）
```

### StateRL 异步运行时 benchmark

`run_benchmark_humanoid_policy.sh` 使用机型 YAML 中真实的模型、参考动作、
`policy_adapter`、custom/external/feedback I/O，并直接运行生产
`FSM + StateRL + ThreadLoop`：

```text
控制周期写入传感器快照并 notify
  → 推理线程唤醒
  → PrepareInputs → AssembleObs → Infer → OnAction
  → 写入 action cache
  → 后续控制周期 MapActionToTargetPos 并应用新动作
```

benchmark 不复制上述调用顺序，而是通过 production `StateRL::Run()` 和
`StateRL::InferStep()` 执行真实的双线程、condition variable、传感器快照、动作
缓存和关节目标映射。传感器状态仍由工具生成，因此不包含 transport、CAN/关节
总线或执行器闭环。

它按 `behavior_manager.control_dt` 驱动控制周期，并按 YAML `rl_policy.rl_dt`
触发推理。默认即为 `periodic`；不支持 throughput，也不提供可配置的 overrun
策略。推理过载时完全沿用 StateRL 当前的 latest-snapshot 行为，并报告 release
合并、未应用动作、旧动作复用周期和最大 release lag。

```bash
cd ~/spacemit_robot/output/staging/bin
./run_benchmark_humanoid_policy.sh g1 sonic \
  --provider cpu --threads 2 --affinity 0,1 \
  --ort-spinning off \
  --hz 50 --rounds 1000 \
  --csv /tmp/g1_sonic_state_rl.csv
```

默认按 YAML 的参考动作播放规则运行。短动作做长尾采样时可加
`--reference-loop`，它只在 benchmark 内存配置中启用循环，不修改 YAML；输出和
CSV 会明确记录该覆盖。加 `--restart-smoke` 会在测量结束后复用同一个 StateRL
对象完成一次退出、重入和两帧新动作校验，用于检查线程、动作缓存及模型状态残留。

主要指标包括控制周期误差、RL release 间隔抖动、推理线程唤醒时间、推理 service
time、release-to-publish、publish-to-apply、action age、动作更新间隔、deadline
miss、coalesced release 和未应用动作。`action age` 从 StateRL 接收传感器快照的
release 时刻开始，到控制线程第一次使用对应动作结束；它仍不包含传感器进入
transport 前和关节命令离开 transport 后的时间。

`--ort-spinning on|off` 显式控制 ONNX Runtime intra-op worker 的 busy-spin，默认
`on` 保持原有行为。`off` 适合与 `on` 做 50 Hz 周期负载对照，但可能引入线程
唤醒延迟；它不控制 SpaceMIT EP 内部线程。stdout 与 CSV 会记录实际值。

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

终端、Android App 和网页是外部客户端，不占用内部 HMI 写端；它们统一调用
`hmi_runtime` 提供的操作服务。核心 driver/control/hmi 仍为三进程，sim2sim 仍为双进程。
接口、控制权和扫码说明见 [操作服务](src/operator_service/README.md)。

语音等外部智能体通过 [MCP 客户端](clients/mcp/README.md) 调用同一个操作接口；
网页语音面板和 HTTPS 入口见 [Web 语音接入](clients/web/voice/README.md)。
两者属于可选客户端，不向 common 引入 ASR、TTS、LLM 或 omni_agent 构建依赖。
机型的人设、动作白名单、声卡和部署服务放在对应机型仓库。

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

同一份机型 YAML 可用 `driver_runtime <config.yaml> --backend mujoco` 显式切换仿真，
或用 `--backend whole_body` 选择实机。覆盖只影响该进程，既不重写 YAML，
也不覆盖策略、FSM 或硬件安全配置；日志记录实际选择的后端。
重复、缺值或未知参数在设备初始化前拒绝。

whole_body 设备初始化失败（`WHOLE_BODY_ERR_DEVICE`）、尚未进入命令循环时，driver 返回退出码 `75`
（启动设备暂不可用）。机型启动器可在有限时间内重建本次启动的进程来重试初始化。
机型启用 whole_body 的 `communication_tolerance_s` 后，窗口内的收发迟滞返回
`WHOLE_BODY_RETRY`：driver 保持正常循环、不锁存故障、不要求 POWER_OFF，不进入
100 ms 退避。反馈等待时发布保留原时间戳的最后有效状态，whole_body 保持上一组
电机目标；恢复后继续接受最新命令。control 的状态推进超时仍独立生效，需与机型
容错窗口对齐；不能通过伪造状态时间戳规避该检查。
超出窗口或硬件错误会锁存故障、保持失能，driver/control/HMI 不因电机事务超时而退出。
失能发送仍失败时每 100 ms 重试；发送恢复后按正常周期读取新反馈。等待期间继续
上报故障、消费并丢弃旧运动命令，不把缓存状态标为新反馈。通信恢复后只接受
POWER_OFF，随后由 control 的人工故障确认流程解除锁存，不自动回到 RL。
无效配置、状态发布失败等不可恢复错误仍退出；机型服务可重启进程，但必须重新
进入 POWER_OFF，不恢复旧控制权或运动指令。回归项 `hardware_recovery` 使用假设备，
覆盖容错窗口内不失能/不锁存/不退避，以及反复/持续断流、
失能恢复后旧 RL 指令拒绝及正常退出。
control 确认设备时间连续推进后，记录一次 `driver state stream ready`。
启动器可通过 `HUMANOID_STARTUP_READY_FD` 传入继承的可写管道描述符（≥3）；
control 就绪时向该非阻塞管道写入一次 `ready\n` 并关闭描述符。
该通知不依赖日志等级、控制台开关或文件缓冲，不改变已有状态通信协议。
HMI 的 `online=1` 仅表示 HMI 与 control 连通，不能作为硬件就绪的依据。
后台日志线程逐条刷新控制台事件，确保输出重定向到文件时，就绪消息也立即可见；
刷新仍在后台执行，不把终端阻塞带回控制线程。

旧配置缺少该字段时默认选择 `mujoco`。`driver_runtime` 只负责 transport、信号和
backend 生命周期；MuJoCo 与整机硬件细节分别封装在私有 adapter 中，不进入 common
公共 API。选择了当前构建未包含的 backend 时，进程会在启动阶段明确失败。

`RobotData` 在原位置/速度基础上携带关节力矩、温度、错误码和 IMU 加速度；
`ControlCmd` 携带执行器模式及位置、速度、力矩、KP/KD 命令。力矩向量为空时
按全零处理。SHM/UDP 使用同一版二进制协议，协议版本不匹配会拒绝数据包，
不会静默按旧布局解析。

driver 通过 `TransportBaseV2` 在机器人状态旁路发送硬件或仿真 backend 故障。control 会锁存
driver、传输、安全监控和策略故障，阻止重新上电并通过 v2 状态旁路回传给 HMI；故障
条件消失后，仍需先回到 `POWER_OFF`，再在 HMI 按 `x` 确认。活动故障不能确认，同一
故障再次发生时会生成新的故障事件并重新进入 SAFETY。
电机、IMU、whole_body 或 transport 反馈失效时 SAFETY 立即保持失能，不使用冻结状态
继续渐退；只有反馈仍有效的策略故障和姿态/角速度超限才执行配置的受控卸力。
control 启动时先确认 driver 状态的设备时间在允许的包间隔内递增；在此之前保持
POWER_OFF，不转发 HMI 上电命令，也不启动运行期 `driver_state_timeout_s` 看门狗。
从首个无故障状态包起，确认过程最多等待 `behavior_manager.safety.driver_startup_timeout_s`
（默认 1.5 s，且不小于运行期阈值）；永久停流或时间戳冻结会报超时并保留锁存。
启动中的真实 driver 故障仍会传到 HMI；状态流就绪后按原运行期阈值保护。

### 实时线程配置

实时循环复用关节、观测、动作与驱动状态缓冲，控制线程以 try-lock 交接推理数据，
不会等待被抢占的推理线程释放锁；等待期间仍执行动作有效期检查。
线程可配置 `lock_memory: true`，启动时预触页并锁定已用内存，需服务设置
`LimitMEMLOCK=infinity`。该选项默认关闭，不改变其他机型的运行配置。

### Runtime 日志

日志落盘实现是 common 内部能力，不额外扩展对外数据接口；上节所述结构化故障由
`TransportBaseV2` 在 SHM/UDP v7 协议中与原有数据并行传递。
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
  artifacts:
    - "config/hardware.yaml"
    - "policy/policy.onnx"
  telemetry:
    enabled: true
    rate_hz: 20
    control_rate_hz: 50
    hardware_rate_hz: 100
    timing_window_s: 5
  driver_monitor:
    enabled: true
    rate_hz: 2
```

未配置时保留原控制行为且不落盘。启用后，每个 runtime 建立独立 session 目录：
`events.log` 记录生命周期、FSM、策略和错误；`driver_timing.csv` 和
`control_timing.csv` 记录周期、迟到和跳周期统计；`driver_motor.csv`、
`driver_joint.csv`、`driver_imu.csv`、`driver_coupling.csv` 分别记录物理电机命令与
反馈、统一关节反馈、IMU 接收质量和耦合机构数值状态；`control_trace.csv` 和
`control_joint.csv` 记录控制状态及最终关节目标/反馈。`control_rate_hz` 和
`hardware_rate_hz` 可分别提高控制与实机硬件诊断采样率，未配置时继承 `rate_hz`。
driver/control 的固定屏显示只在交互终端启用，由后台最新快照线程刷新；重定向到
文件时不再写入周期性 ANSI 屏幕内容。`runtime_logging::Log` 的控制台输出和落盘
都经有界后台队列，慢终端或文件写入不在调用线程执行；队列满时丢弃日志并记录数量。
结构化故障仍经 transport 传递，不依赖日志是否写出。

日志、资产校验、最新快照以及异步策略准备线程使用普通调度，并恢复进程启动时的
CPU 集合，不继承 control/driver 的实时优先级或单核绑定。`ThreadLoop::Apply()`
实际设置 `other`，并报告 requested/actual 调度、优先级和 CPU 集合；设置失败仍会
输出错误及实际状态。机型的 FIFO/绑核参数保存在机型 YAML，不在 common 硬编码。
初始模型、通信和辅助线程先在普通调度下初始化，完成后才对 control 主循环应用
实时调度，避免模型工作线程继承同一单核 FIFO 后被初始化线程的忙等待饿死。

whole_body 硬件后端的 CSV 格式化和终端显示也在后台执行；控制线程只复制一份
固定大小的诊断快照，保留采集时刻的 `wall_time_s`。快照队列只保留最新一份，
不能锁定时直接丢弃，绝不排队等待打印。`driver_timing.csv` 的
`telemetry_dropped` / `telemetry_failed` 记录累计快照丢弃数及后台异常状态，
与日志落盘队列的丢弃统计分开。退出时回收后台线程。
`driver_bus.csv` 按 `hardware_rate_hz` 每条总线一行，记录累计收发字节、有效帧、
CRC/解码/重同步错误、未知 ID、读写/poll 错误及超时；同时保留最近失败事务的
耗时、截止时间超出量、接收/发送情况、缺失反馈 ID 位图和 errno。控制线程仅在
采样时复制已有计数，CSV 格式化和落盘均复用上述后台线程，没有新增串口操作、
逐帧打印或等待磁盘。计数不参与控制决策，也不改变通信容错参数。
`test_bus_telemetry` 用生产快照类型对比新增计数前后的复制开销，并故意阻塞消费
线程，验证生产者仍可提交/丢弃快照；输出是快照交接成本，不是整个控制周期耗时。
`read_mean_ms/read_max_ms` 是外设读取加关节映射耗时，
`work_mean_ms/work_max_ms` 是循环主体耗时（不含主动休眠），用于定位周期超期。

`metadata.json` 记录启动配置、common 构建版本和工作区状态。配置中的
`logging.artifacts` 以及运行期加载的硬件配置、仿真模型、策略模型和参考动作会登记到
`artifacts.csv`；文件大小和 SHA256 由日志线程异步计算。CSV 写入前会检查表头、列数
和同一流的表头一致性，格式错误只写入 `events.log`，不会污染数据文件。资产哈希使用
独立后台队列，不会阻塞控制循环或 CSV 落盘线程。

启用 telemetry 后，`policy_enabled: true` 可独立开启策略诊断；未填写时仍沿用
`level: debug` 才开启的兼容行为。StateRL 按每次实际推理采样
`control_policy_<策略名>.csv`。每行包含组装观测、ONNX 原始 action、策略执行器完成
裁剪/平滑后的 action，以及 policy adapter 处理后最终用于关节映射的 action；同时记录
推理发布时序和动作被控制线程采用后的最终关节目标。采样不受 `telemetry.rate_hz`
降采样；下面的黑匣子配置决定全量样本留在内存还是持续写盘。

每次推理在 `result` 列标记为 `published` 或 `inference_timeout`。超时动作不会发布，
也不会产生对应的 apply 行；该行的 `action_published_time_s`、
`finish_to_publish_ms`、`release_to_publish_ms` 为 `nan`。统计已发布动作时应筛选
`result=published`。

CSV 还记录 `release_to_finish_ms`、`inference_deadline_ms`、传感器快照
`snapshot_ms`、参考输入准备 `prepare_inputs_ms`、观测组装 `assemble_obs_ms`、
执行器调用 `policy_infer_ms`、后处理 `postprocess_ms`，以及交互动作和请求状态。
阶段耗时按经过时间统计，包含线程等待；`policy_infer_ms` 包含 PolicyExecutor 的
输入输出处理，并非纯 ONNX Run。`inference_thread_cpu_ms` 仅统计推理线程自身的
CPU 时间，不包含 ONNX/EP 工作线程。超时原因也会写入 `events.log`，不依赖
debug 或 telemetry 开关。

可选的长时间运行配置（未启用时连续写入 CSV）：

```yaml
logging:
  level: info
  queue_capacity: 8192
  queue_memory_mb: 16
  storage: {max_total_mb: 4096, min_free_mb: 5120}
  blackbox:
    enabled: true
    pre_trigger_s: 30
    post_trigger_s: 10
    cooldown_s: 60
    max_memory_mb: 64
  telemetry:
    enabled: true
    policy_enabled: true
    idle_rate_hz: 1
    continuous_rate_hz: 2
    control_rate_hz: 50
    hardware_rate_hz: 100
```

- 正常期间按 `continuous_rate_hz` 写概览，各流的完整采样保留在后台内存环中。
  POWER_OFF 的 control、实机 driver 诊断采样降到 `idle_rate_hz`，不改变控制周期、
  电机总线频率或 IMU 读取。故障片段采集中即使掉电仍按原诊断频率采集。
- 错误事件及 whole_body 的通信重试触发前后片段，三个进程通过同一 boot ID、
  单调时钟和 incident ID 关联。`incident_*/trigger.json` 记录实际保留时长，
  `.complete` 表示片段关闭。连续故障合并，冷却时间内不重复生成海量片段。
  这是进程内存缓存；突然掉电/进程崩溃之前未触发落盘的高频数据不会自动恢复。
- control 和 StateRL 的大行 CSV 使用预分配快照队列，生产者只复制固定容量数据、
  `try_lock` 失败即丢弃日志，不等待后台。策略快照上限 8192 维观测/128 维动作；
  超出只禁用该策略 trace 并告警，不影响推理。control 关节快照上限 128 轴。
  `logger_health.json` 每 5 秒更新队列/格式化丢弃数、I/O 错误、存储抑制数和内存环用量；
  driver 原有诊断快照丢弃数仍在 `driver_timing.csv`。
- 队列同时限制条数与估算字节数；每进程缓存最多 `max_memory_mb` 的记录载荷，
  先达到时间或内存上限就淘汰最旧记录。因此高维模型可能不足完整 30 秒，实际时长
  以 trigger 元数据为准；容量不是整个进程的 RSS 上限。
- 总量按同一日志根目录和 `robot_base.name` 汇总。优先删除带本机型元数据、
  未持有 `.active.lock` 的旧 session，以及本进程已关闭的旧片段；不删除活跃
  session、不跟随符号链接。后台按秒/每 MiB 检查并预留余量，不是文件系统硬配额。
  活跃文件自身轮转；额度或剩余空间不足时停止大流写入，小事件使用有限保留余量。
  满盘、目录不可写、格式化异常只降级日志，不触发 FSM 故障或改变电机命令。
  文件打开/写入失败会在后台限速重试，空间恢复后可续写，不要求重启控制程序。
- 启动器可调用 `humanoid_logs.py prepare/sink`，为控制台输出设置轮转和写盘失败处理。
  `prepare` 无法创建磁盘日志时回退到 `/dev/shm/humanoid-logs-<uid>`
  （128 MiB 配额、16 MiB 空间余量，重启即丢失）；`sink` 持续排空输入，写盘失败时丢弃输出。
  此功能需要启动器调用，单独配置 logging YAML 不会启用。

故障黑匣子在触发后立即 fsync 前段数据，录制中每秒落盘一次；结束时所有文件和目录
同步成功才写入 `.complete`，其中 `durable=1` 表示已完成落盘。掉电前尚未结束的
片段没有完成标记，但已经同步的前段数据仍可读取。
`control_inference_progress_<policy>.csv` 单独记录唤醒、观测、模型、发布与应用阶段，
附线程 CPU 时间、缺页和主动/被动切换计数，未返回的推理也能留下开始记录。
实时线程只采集数值并尝试写预分配队列；格式化、文件写入和 fsync 全在后台。
普通快照竞争或队列满时丢弃并统计；故障事件另有 8 个预分配备用槽保存触发原因，
备用槽也满时仍保留黑匣子触发请求，不用磁盘写入反压控制循环。

主动保存接下来 20 秒高频数据、或者导出日志压缩包：

```bash
humanoid_logs.py capture --config robot.yaml --seconds 20 --reason manual-check
humanoid_logs.py export --config robot.yaml --output output/robot-diagnostics.tar.gz
```

手动片段仍遵循冷却/合并规则，最大后段 120 秒；命令仅请求日志、不发送运动命令。
导出使用 gzip，包含事件、CSV、版本/模型哈希和文件 SHA256 清单；不拷贝模型，
不读取设备、不打包 FIFO/符号链接、不强制停止服务。默认最大原始数据 4096 MiB，
优先取最近文件，未纳入的文件列在 `manifest.json.omitted`。输出须在日志目录外。
使用 RAM 回退目录时，给工具同样设置 `HUMANOID_LOG_DIRECTORY`。

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
- **HOME 阶段**：从当前实机关节位置先平滑建立增益，再恢复到 `robot_base.default_joint_pos`；可用 `behavior_manager.home` 配置时长和 kp/kd。
- **RL 控制期**：进入 RL 状态后，control 端每帧通过 `ControlCmd.kp/kd` 下发 `rl_policy.onnx_infer.policies.<name>.kp/kd`，driver backend 转发给执行组件。
- **ZERO/DAMP 阶段**：ZERO 优先用当前策略的可选 `zero_target_pos`，未配置时用 `rl_default_pos`；满足 `behavior_manager.zero` 的关节收敛条件后才允许进入 RL。DAMP 使用 `behavior_manager.damp_kd`。

HOME → ZERO 的位置插值从 HOME 最后一帧目标开始，保留受载状态下的位置误差，
不将目标重置成实际反馈角度。没有有效的已使能目标时，才从关节反馈开始插值。

### 策略切换结果

运行中切换由策略的 `switch_group` 显式启用：同组策略可在 RL 中请求切换。
控制器先将速度降至零，等待当前 adapter 返回站立且无动作占用，再接入后台
准备好的新策略；模型加载、旧推理线程回收不在控制循环中执行。
未填写分组的策略仍仅能在 POWER_OFF/DAMP 选择。`display_name` 用于 Web/TUI
显示，不改变内部策略键名；可选范围仍由 `policy_names` 限定。

HMI 的策略列表来自 YAML，不表示对应的模型和参考动作文件已经存在。
在 POWER_OFF / DAMP 选择策略时，`Command.policy_request_sequence` 标识一次请求；
Control 通过 `ControlStatus.policy_switch` 返回原请求序号、策略名及
`PREPARING / READY / FAILED / REJECTED`。序号为零保留原有 C++ 调用方式。
HMI 只接受匹配当前请求的结果，同名策略重试使用新序号。

加载失败保留原策略，立即显示错误并解除 HMI 的等待，不锁存为整机故障。
尚未完成加载时，HOME 仍由 behavior_manager 的准备就绪条件保护。
`f` 掉电请求不受策略等待阻挡；它清除 HMI 待确认请求，但不阻塞等待后台加载线程退出。
前置策略已准备时可以返回 READY，此时实际运行策略名仍由 `active_policy` 表示，
不把“已选择目标动作”混同于“目标动作已开始”。

固定布局协议版本为 v7，`Command / ControlStatus` 的 C++ 布局包含策略切换状态。
必须一起重新编译、重启 driver、control、HMI 及使用这些结构的程序；
不能混用旧二进制或旧版外部遥控端。RL 推理组件和模型格式没有改动。
回归测试 `policy_switch` 覆盖失败回滚、重试、过期结果和 FSM；
`hmi_policy_switch` 用真实 HMI 终端和隔离的 SHM 通道验证失败反馈、前置链和掉电优先。

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

### 策略协议适配

对于“选择模式后待命、按动作键才执行”的入口，使用
[`policy_actions` 动作目录](src/behavior_manager/README.md#模式内动作目录)，不要使用自动执行的前置链。
目录可调度不同模型，待命模型、动作名称和子策略均由机型 YAML 配置；
动作结束仍保留所选模式，返回该模式的待命模型。

需要外置参考动作或特殊模型输入的策略，在机型 YAML 的策略段配置
`policy_adapter`。它是 `behavior_manager` 的内部子模块，不是与
`behavior_manager` 平级的新 common 模块：

1. `model_zoo/rl` 负责推理后端、通用模型 I/O、feedback tensor 和观测项组装。
2. [`policy_adapter`](src/behavior_manager/policy_adapter/) 负责参考动作读取、时间轴、朝向对齐，以及 ProtoMotions / SONIC 的输入协议。
3. 应用层机型仓库只提供模型、参考动作和 YAML 参数，不放 tracker C++ 代码。

`stance_motion` 适配器支持同一个 RL 行为内部按命令选择预加载的站立/运动模型。
两模型共用观测历史和上一动作，切换不退出 RL；通用的整项策略切换规则保持不变。
模型路径、输入端口、历史帧数、命令逐帧增减量、站立和前向档位阈值、转向比例及
6D 姿态参考系由机型 YAML 指定。协议的观测为角速度、6D 姿态、绝对关节角、
关节速度、上一帧缩放动作；模型须共享 I/O、动作尺度、PD 与关节顺序。
`StateRL` 通过 `SelectExecutor` 选择同一次推理的执行器，默认实现始终返回原执行器；
adapter 自有执行器参与退出时的取消。内部选择记录在推理 CSV 的 `model_variant`，
不伪装成交互动作请求，也不修改 transport 协议。该适配器不依赖某一种推理后端。

`history_actor` adapter 提供本体观测历史和八维 gait 命令，关节映射、缩放、
步频和站立相位条件由机型配置；不改变 RL 推理组件的模型接口。

其 `command_type: latent` 分支支持共享解码器、动作目录和 CSV 时间裁切。
`reference_file` 是待命参考，动作的 `start_time`/`end_time` 按 `rl_dt` 取帧，
结束时间不含；播放结束或取消后回待命，动作占用结束才允许切模式。
带根部姿态的参考通过 IMU 构造旋转误差，`mask_root_position` 可按模型要求
屏蔽平移误差；这些输入约定由 YAML 指定。模型和参考数据在后台准备时加载，
不会在控制循环或动作按钮处理时读取文件。

`entry_target_transition_duration` 启用时，StateRL 保存进入时的关节目标作为固定
插值起点，按推理帧推进到当前策略目标；不会反复以已混合的上一帧结果作为起点。
最终目标仍执行策略限位，未配置此项的策略保持直接输出。

机器人自由度取自策略 `rl_default_pos`，模型关节顺序取自
`action_joint_index`，自定义观测维度取自 `custom_array_dims`；这些机型和
模型参数均由应用层 YAML 传入，不在 common 中固定。

```yaml
policy_adapter:
  type: mjlab                 # mjlab / protomotions / sonic
  reference_file: policy/example/motion.npz
  motion_fps: 50
  playback_speed: 1.0
  loop: false
  loop_pause: 0.0
  start_mode: auto           # auto / manual；manual 在 RL 中按 G 后开始
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
- ProtoMotions：`reference_file` 指向单个预处理 CSV，使用
  `future_steps`；
- SONIC：单动作配置用 `reference_file`；多动作配置用 `catalog`，目录中的
  每个 `file` 都指向包含 `joint_pos.csv`、`joint_vel.csv` 和
  `body_quat.csv` 的参考动作目录。两种配置均使用 `future_frames` 和
  `future_step`。

SONIC 多动作模式复用 HMI 的交互动作页，但与 `joint_trajectory` 的局部关节
接管不同：切换动作只更新策略的 `reference_obs`，ONNX 输出仍控制全部策略关节。
进入 RL 后先保持 `default_action` 的首帧，选择动作时按
`transition_duration` 平滑切换参考序列。

下例中 `sonic_actions` 位于 YAML 根节点，`policy_adapter` 位于对应策略节点：

```yaml
sonic_actions:
  action_names: [gesture_a, gesture_b]
  default_action: gesture_a
  transition_duration: 1.0
  actions:
    gesture_a: {file: policy/sonic/gesture_a, display_name: "动作 A"}
    gesture_b: {file: policy/sonic/gesture_b, display_name: "动作 B"}

rl_policy:
  onnx_infer:
    policies:
      sonic:
        policy_adapter:
          type: sonic
          catalog: sonic_actions
          future_frames: 10
          future_step: 5
```

策略若配置
`zero_target_pos`，ZERO 阶段先过渡到参考动作起始姿态。非循环动作播放完成后
保持末帧，状态切换仍由 HMI/control 负责。

MJLab 策略可配置 `start_mode: manual`。进入 RL 后策略继续按实时状态推理，但参考
输入保持第 0 帧且参考速度为零；HMI 按 `G` 后开始时间轴。开始命令复用现有按键
通道，使用现有 SHM/UDP 命令布局；未收到命令时保持等待，可再次按 `G`。

MJLab 的 `reference_action` 未配置时保持标准策略行为。配置后，仅列出的机器人
关节使用 `参考关节角 + residual_scale * clip(原始模型 action)`；其余关节仍按
通用 `rl_default_pos + action_scale * action` 映射。关节列表属于应用层机型参数，
common 不固定自由度或手臂索引。

`joint_trajectory` adapter 用于在 RL 保持运行时临时接管 YAML 声明的关节。
未接管的关节仍采用当前策略输出；接管后的最终 action 通过自定义观测项回写给
下一帧策略，避免策略历史与实际执行命令不一致。动作目录和关节范围均由机型配置：

```yaml
policy_adapter:
  type: joint_trajectory
  catalog: interaction_actions
  applied_action_term: applied_action

custom_array_dims:
  applied_action: 28
observation:
  segment_0_terms: [ang_vel, projected_gravity, command, dof_pos,
                    dof_vel, applied_action]

interaction_actions:
  action_names: [wave]
  blend_in_duration: 0.25
  blend_out_duration: 0.25
  actions:
    wave:
      display_name: "挥手"
      file: policy/stand/interactions/wave.npz
      joint_indices: [14, 15]
```

轨迹 NPZ 的 `joint_pos` 为 float32/float64 `[T,K]`，列顺序与对应动作的
`joint_indices` 一致；`display_name` 仅用于 HMI 展示，控制协议继续使用稳定的
动作 key。HMI 仅在当前 RL 策略注册了动作目录时显示 `A/C`：
按 `A` 打开动作页后可连续选择动作，执行进度和完成状态在页内持续显示，按
`C` 平滑取消。MJLab 手动参考策略则只显示 `G` 启动提示。

`stance_motion` 可组合相同的局部轨迹播放器，保留双模型站走切换：

```yaml
policy_adapter:
  type: stance_motion
  # 保留 motion_model_path、观测、命令缓变等原有字段。
  joint_trajectory:
    catalog: interaction_actions
```

该组合仅在零速度请求、已选中 stance 时接受动作。动作及收回期间固定使用
stance；动作中按住的速度不会在结束后自动生效，须先松开归零再给新指令。
双模型共享历史保存轨迹覆盖前的网络输出乘 `action_scale`，不把手势目标写回
模型的上一动作；轨迹播放器独立保存合成目标，用于接管/收回的连续过渡。
不额外注入 `applied_action` 观测项。
未配置 `joint_trajectory` 的策略行为不变。

`stance_motion.interaction_observation` 可显式声明 `joint_indices`、
`neutral_positions` 和 `release_frames`：接管权重大于零时以固定关节角替代该组
q 观测和模型自身目标，dq 仍实测；权重归零立即恢复真实 q 观测，第
`release_frames` 帧恢复模型目标。模型动作历史始终在任何目标覆盖之前保存。
配置缺省时不做此覆盖，参考姿态由 YAML 提供。

轨迹目录或 `joint_trajectory` 内可选 `measured_entry_max_velocity`（rad/s），
大于零时从实测关节角进入首帧，时长为最大角差/速度，入场即采用完整轨迹增益；
默认 0 保留原先从策略目标混合的方式。`linear_blend: true` 使用线性进入/收回，
默认 false 保留 smoothstep。二者可独立配置，不改变其他策略的默认行为。

目录或每个动作可选配置 `kp`、`kd`，长度与动作的 `joint_indices` 一致。
默认增益和关节目标一起平滑接管/收回，并作为同一帧缓存传给 control；未接管关节
保持策略增益。取消进入过渡中的动作从当时已应用的目标和增益权重收回。
`hold_last_frame: true` 时，可用动作的 `max_hold_duration` 设置末帧保持秒数，
超时自动收回；默认 0 保留原来的无限保持、人工取消行为。退出 RL 清除动作状态。
TUI、Web、Android 共用此目录，不需要为机型编写新界面。

### ControlMode 数据流（control → driver）

`ControlCmd.mode` 是 control 端发给 driver 的通用控制语义字段（`enum class ControlMode { POWER_OFF, DAMP, ZERO, RL, SAFETY, HOME }`），driver 据此自主决定后端行为：

- **三进程 FSM 模式**：`control_runtime` 把 `bm.CurrentState()` 透传到 `ctrl.mode`
- **sim2sim 模式**：`control_sim2sim_runtime` 始终发 `ControlMode::RL`
- **mujoco driver**：边沿检测 mode 变化—— `RL` → 自动取消悬挂保护，`POWER_OFF` / `DAMP` → 自动启用悬挂；其余 mode 不主动覆盖（保留 mujoco 界面 F 键的手动权限）
- **whole_body driver**：转换为整机控制模式，并由组件本地安全门控与 watchdog 约束

设计原则：跨层接口字段必须用通用语义，禁止携带某一具体后端（mujoco 悬挂等）的私有概念。

### 操作服务与客户端

`hmi_runtime` 提供统一操作服务，终端、网页、Android 和手柄通过客户端接口访问。
策略、动作列表、按键绑定和遥控速度范围由 YAML 配置。

- [操作服务](src/operator_service/README.md)：连接与配对、控制权、Web/App 遥控、离线音频、协议。
- [本机手柄](clients/gamepad/README.md)：设备布局、点按/长按、摇杆和震动配置。
- [Android](clients/android/README.md)：APK 构建、扫码和前后台生命周期。

`operator_audio` 为可选独立进程，播放 `audio_prompts` 配置的离线文件；
音频解码和播放不在 driver/control 的实时线程执行。

### hmi_tui 键盘操作

hmi_tui 复用原 HMI 的彩色 ANSI 全屏界面，经公开客户端接口连接 hmi_runtime。
FSM、当前策略、Control 实际采用速度和
RL 频率均来自 control 端回传，不在 HMI 本地预判。主界面用左右键切换相邻 FSM，
策略和速度分别使用独立子页面。

| 按键 | 动作 |
| --- | --- |
| `L/U` | 申请/释放控制权 |
| `←/→` | 按真实 FSM 后退/前进；RL 按左键直接退到 DAMP |
| `f` | 全局请求 POWER_OFF（完全失力） |
| `p` | 打开策略选择页；`↑/↓` 或 `j/k` 移动，Enter 确认，Esc 返回 |
| `v` / `Enter` | 真实状态为 RL 且心跳正常时进入速度控制页 |
| `w/s` | 速度页内增减 vx |
| `a/d` | 速度页内增减 vy |
| `q/e` | 速度页内增减 wz |
| `空格` | 速度清零 |
| `Esc` / `v` | 退出速度页并清零 |
| `A` / `G` / `C` | 主页面进入动作列表 / 开始手动参考动作 / 平滑取消动作 |
| `X` | 确认已经恢复的锁存故障 |
| `Ctrl+C` | 退出客户端并释放控制权，不自动掉电 |

`o/h/z/r` 保留为兼容快捷键，但合法性仍由 control 端 FSM 校验。HMI 以配置频率发送
心跳；退出速度页、离开 RL、HMI 退出或心跳超时都会清零速度。按键步长和通信超时在
YAML 的 `hmi` 节点配置；速度范围由应用层在每个策略的
`command.limits.{min_vx,max_vx,min_vy,max_vy,min_wz,max_wz}` 中声明；未配置
`min_*` 时默认为对应的 `-max_*`。未声明范围的策略不接受速度命令。

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
