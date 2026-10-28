# behavior_manager — FSM 行为管理

有限状态机（FSM）的人形机器人行为控制模块。支持 POWER_OFF、DAMP、HOME、ZERO、RL、SAFETY 和可选的 TRAJECTORY 状态，由 YAML 配置驱动，与机器人型号解耦。StateRL 在独立线程执行 ONNX 推理；固定底座轨迹控制不运行推理。

## 接口说明

### 枚举与数据结构

#### `StateName` — 状态枚举

| 状态 | 说明 |
|------|------|
| `POWER_OFF` | 完全失力（关节零力矩）|
| `DAMP` | 阻尼保持（kp=0, kd=配置值）|
| `HOME` | 平滑恢复 `robot_base.default_joint_pos` |
| `ZERO` | 平滑到当前控制方式的准备姿态 |
| `RL` | RL 控制（异步 ONNX 推理）|
| `TRAJECTORY` | 固定底座关节轨迹播放 |
| `SAFETY` | 安全保护（传感器、策略、传输或执行故障触发）|

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
| `actuation_mode` | `robot_base::ActuationMode` | 执行器命令语义，当前 FSM 默认使用 `HYBRID` |
| `target_pos` | `vector<double>` | 目标关节位置 (rad) |
| `target_vel` | `vector<double>` | 目标关节速度 (rad/s) |
| `target_torque` | `vector<double>` | `HYBRID` 下为前馈力矩，`TORQUE` 下为直接目标力矩 (Nm) |
| `kp`, `kd` | `vector<double>` | PD 参数（每帧由当前 RL 策略提供）|
| `enable` | `bool` | 使能标志 |

### `BehaviorManagerClass` — 行为管理器

| 方法 | 说明 |
|------|------|
| `BehaviorManagerClass(config_path)` | 构造，加载 YAML 配置 |
| `Init()` | 初始化（加载配置、注册状态、初始化 FSM）|
| `Step(control_dt, rl_dt)` | 执行一步控制 |
| `SetSensorData(data)` | 设置传感器数据 |
| `SetSensorFault(fault)` | 设置与当前传感器帧关联的 driver/backend 故障 |
| `SetCommand(cmd)` | 设置命令（状态切换、速度指令、策略切换）|
| `AcknowledgeFault(sequence)` | 在 POWER_OFF 下确认对应序列的已恢复锁存故障 |
| `GetOutput()` | 获取控制输出 |
| `CurrentState()` | 获取当前状态 |
| `IsZeroReady()` | 查询 ZERO 是否完成、是否可进入 RL |
| `CurrentPolicyName()` | 获取当前 RL 策略名 |
| `GetRlFreq()` | 获取 RL 推理频率（Hz）|
| `CurrentFault()` | 获取当前活动或锁存故障 |

**状态切换指令 (`Command.key`)：**

| key | 转移状态 | 说明 |
|-----|---------|------|
| `1` | POWER_OFF/HOME/ZERO/RL → DAMP | 进入/退回阻尼 |
| `4` | DAMP → HOME | 恢复机型默认姿态 |
| `2` | HOME → ZERO | 切到策略准备姿态（需 HOME 完成） |
| `3` | ZERO → RL | 进入 RL（需 ZERO 到位）|
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

模块读取 YAML 的 `behavior_manager` 和 `rl_policy` 节点。参考
[`example/behavior_manager/config_example.yaml`](../../example/behavior_manager/config_example.yaml)
了解配置结构和参数含义。

**关键参数：**
- `rl_policy.onnx_infer.policies.<name>.kp/kd`：各策略的 PD 增益
- `rl_policy.onnx_infer.policies.<name>.entry_target_transition_duration`（可选）：进入 RL 后的目标位置过渡时间，默认 `0`
- `rl_policy.onnx_infer.policies.<name>.switch_group`（可选）：允许 RL 内交接的兼容组；来源与目标须属于同一非空组
- `rl_policy.onnx_infer.policies.<name>.policy_switch.transition_duration`（可选）：RL 内交接时，目标位置与 KP/KD 同步过渡的时长，默认 `0`；不改变普通 ZERO→RL 的配置
- `rl_policy.onnx_infer.policies.<name>.return_policy`（可选）：有限动作完成后的返回策略，须在同一兼容组中；省略时保持当前策略
- `behavior_manager.policy_switch.timeout_s`（可选）：后台准备与交接等待上限，默认 30 秒；超时取消本次请求，当前策略继续运行
- `rl_policy.onnx_infer.policies.<name>.target_position_lower / target_position_upper`（可选）：按机器人关节顺序给出目标位置上下限（rad），须同时配置且长度与 `rl_default_pos` 一致；同时省略时禁用
- `rl_policy.onnx_infer.policies.<name>.target_limit_margin`（可选）：目标位置限位的内缩余量（rad），默认 `0`；`[lower + margin, upper - margin]` 必须为有效区间
- `rl_policy.onnx_infer.policies.<name>.prerequisite.policy / .duration`（可选）：进入目标策略前运行的策略及持续时间
- `rl_policy.onnx_infer.policies.<name>.policy_adapter`（可选）：参考动作、特殊模型输入或 RL 内关节轨迹接管；支持 `mjlab`、`protomotions`、`sonic`、`joint_trajectory`
- `rl_policy.onnx_infer.policies.<name>.policy_adapter.start_mode`（MJLab 可选）：`auto`（默认）或 `manual`；手动模式进入 RL 后保持参考首帧，按 `G` 开始
- `rl_policy.onnx_infer.policies.<name>.policy_adapter.reference_action`（MJLab 可选）：指定采用参考动作与残差合成的关节
- `rl_policy.onnx_infer.policies.<name>.policy_adapter.catalog`（SONIC 可选）：为同一全身策略注册多个参考动作；HMI 选择动作后仅切换 `reference_obs`，不覆盖模型 action
- `rl_policy.onnx_infer.policies.<name>.command.limits`（可选）：应用层声明该策略接受的 `min_vx/max_vx/min_vy/max_vy/min_wz/max_wz`；未配置 `min_*` 时默认为对应的 `-max_*`，整个 limits 未配置时 HMI 和 Control 均拒绝速度命令
- `behavior_manager.damp_kd`：阻尼状态 kd（≈ policy kd / 5）
- `behavior_manager.home.gain_ramp_duration / move_duration`：HOME 建立增益和移动时长，`move_duration` 默认 3 秒
- `behavior_manager.home.kp/kd`（可选）：HOME 增益；未配置时使用 `robot_base.kp/kd`
- `behavior_manager.home.target_position_lower / target_position_upper`（可选）：HOME 目标位置范围，按关节顺序、单位 rad，须成对配置。初始反馈越界时，增益渐入阶段从最近的有效目标开始，再平滑移向默认姿态；反馈不修改。默认姿态本身越界则拒绝配置。未配置时保持原行为。
- `behavior_manager.zero_pos`：未配置 RL 策略时的 ZERO 目标位置；配置策略后使用 `rl_policy.onnx_infer.policies.<name>.zero_target_pos` 或 `rl_default_pos`
- `behavior_manager.zero.move_duration / kp / kd`：ZERO 移动时长和独立安全增益，`move_duration` 默认 3 秒；kp/kd 必须同时配置
- `behavior_manager.zero.position_tolerance / velocity_tolerance / settle_duration`：ZERO 到位条件，默认分别为 0.15 rad、0.10 rad/s、0.20 秒
- `behavior_manager.safety.driver_state_timeout_s`：driver 状态最大允许年龄，超时后锁存传输故障
- `behavior_manager.safety.max_roll / max_pitch / max_angular_velocity`：control 侧姿态和角速度安全门限，`0` 表示不启用对应门限
- `behavior_manager.safety.release_duration_s`：反馈仍有效时 SAFETY 逐步卸力的时长，默认 1 秒；反馈无效时立即失能
- `behavior_manager.rl_safety.first_action_timeout_s / max_action_age_s / inference_deadline_s`：RL 首帧等待、已发布 action 年龄和单次推理时限；也可在单个策略的 `runtime_safety` 下覆盖，`0` 表示不启用
- `discard_late_inference: true` 丢弃超过推理期限的输出并等待最新观测，不立即锁故障；首帧及 action 年龄仍限制恢复总时长，默认关闭。

**路径解析：** `robot_base.robot_dir` 相对于 YAML 文件；`model_path` 相对于 `robot_dir`。

### 状态机流程

```
POWER_OFF ─key=1→ DAMP ─key=4→ HOME ─key=2(完成)→ ZERO ─key=3(完成)→ RL
    ↑                                                                  │
    └────────────────────── key=-1(ESC) ───────────────────────────────┘
                                                                       │
                              传感器/策略/传输/执行故障 ───────→ SAFETY
```

故障分为“仍在发生”的 active 状态和“等待人工确认”的 latched 状态。SAFETY 完成安全
退回后仍保留锁存，阻止再次上电；只有故障条件已经消失且 FSM 位于 `POWER_OFF` 时，
HMI 的 `x` 才能清除锁存。重复收到同一条已确认的恢复状态不会重新锁存，但该故障再次
变为 active 时会按新事件重新触发 SAFETY。

### 关键设计

- **策略准备与异步推理：** 默认策略在 BehaviorManager 初始化时创建 ONNX runtime；退出 RL 后或在 POWER_OFF/DAMP 选择策略时，后台准备全新的策略状态，准备完成前不进入 HOME。进入 RL 后只重置时序状态并启动独立推理线程，不在控制循环内加载模型
- **策略入场过渡：** 普通进入 RL 使用 `entry_target_transition_duration`，增益立即生效；RL 内交接使用 `policy_switch.transition_duration`，位置及增益同步过渡。推理和历史状态正常更新，不暂停动作时间；定时动作应按模型要求选择过渡时长
- **目标位置限位：** `StateRL` 在关节映射和入场插值后按可选的逐关节上下限裁剪最终目标，配置错误会阻止策略加载。此限位不修改模型的 `last_action` 或动作平滑历史；`clip_actions` 另用于模型输出裁剪
- **通用模型 I/O：** `PolicyExecutorConfig` 整体透传给 RL 组件；多输入输出、feedback、external 等拓扑由策略 YAML `model_io` 声明，common 无需复制底层字段
- **策略协议适配：** `policy_adapter` 是 behavior_manager 的私有子模块；机器人自由度、关节映射和自定义观测维度由机型 YAML 透传，common 不固定具体机型维度
- **交互动作接管：** `joint_trajectory` adapter 在 RL 状态内部合并选择性关节轨迹，最终 applied action 回写策略观测；不会创建与 RL 并列的 FSM 状态
- **SONIC 多动作：** 多个参考目录在进入 RL 前统一校验并加载；进入 RL 后保持默认参考首帧，HMI 选择动作时平滑切换参考输入，全身 action 始终由同一个 SONIC 模型输出
- **动态策略切换：** POWER_OFF / DAMP 允许选择策略；HOME / ZERO 锁定当前策略。RL 内同组切换时先清零速度、后台准备目标，旧策略持续控制；准备成功后由当前适配器完成退出并交接。加载失败保留旧策略，状态切换或故障取消待处理交接
- **前置策略链调度：** 目标可配置 `prerequisite: { policy, duration }`。前置策略实际生效、进入 RL 后才计时，模型加载和旧策略运行时间不计入。前置和目标须同组，不支持嵌套前置链；用户只发一次选择请求
- **安全保护：** driver 故障、状态超时、无效数据、姿态超限和 RL 推理/action 超时统一触发安全状态并跨进程锁存
- **状态回传：** `CurrentState()`、`IsZeroReady()`、当前策略和 RL 频率由 control_runtime 组装为 `ControlStatus` 发给 HMI
- **交互取消：** `history_actor` 在待命时确认取消并保持 IDLE；重复取消不重置动作退出进度，尚未执行的动作也可取消。
- **交互状态校验：** 交互元数据无效时保留同一策略最近一次有效交互信息，继续发送当前 FSM、故障和控制状态；切换策略时清除缓存。其他状态字段仍按原规则校验。
- **参数隔离：** damp_kd ≈ policy kd / 5（不同阶段刚度需求不同）

### 机型接入动态策略

已有模型输入协议无需改写。机型保留自己的 `model_io`、观测、关节映射、增益、
`policy_adapter`，只在兼容策略上声明组名、过渡时间和可选前置/返回策略。
组名由机型定义，common 不注册固定模式数，也不识别机型或策略名称。

在各策略既有配置中增加以下字段即可；示例策略名由机型自行替换：

```yaml
stand:
  switch_group: whole_body
  policy_switch: {transition_duration: 0.5}
walk:
  switch_group: whole_body
  policy_switch: {transition_duration: 0.5}
motion:
  switch_group: whole_body
  policy_switch: {transition_duration: 0.0}
  prerequisite: {policy: stand, duration: 2.0}
  return_policy: stand
```

示例省略模型和观测字段；具体时长须匹配各策略的入场要求。

| 当前策略类型 | 收到切换请求后的交接点 |
| --- | --- |
| 普通速度策略，无适配器 | 收到零速度后的有效推理帧 |
| 内置 `ref_motion_phase` 的有限动作 | `motion_length` 结束 |
| `joint_trajectory` | 当前轨迹收回完成 |
| `mjlab` / `protomotions` 参考动作 | 待播放首帧，或当前动作末帧；循环动作等到本轮末帧 |
| 带动作目录的 `sonic` | 当前动作收回到待命参考 |
| `stance_motion` / `history_actor` | 退出交互、返回站立或待命分支 |

有限动作可通过 `return_policy` 自动返回另一策略；返回模型准备失败时保留当前控制，
不无限重试加载。手动启动的参考动作在尚未播放时不会触发自动返回。
需要等待较长动作结束时，同时设置 `behavior_manager.policy_switch.timeout_s` 和
`hmi.request_timeout`，后者应更长。超时会报告失败，不强制从动作中途切走。

组名只表示机型允许尝试交接，不转换不同策略的训练观测协议。对于全新输入协议，
仍需实现对应适配器；对于已有协议，不需要在 common 增加机型分支。

MJLab 参考使用世界坐标时，可设置 `policy_adapter.anchor_xy_align: true`：
每次进入策略，把参考首帧的水平位置对齐到机器人当前位置，随后保持该参考原点。
它与 `anchor_yaw_align` 分别控制平移、朝向对齐，不改变参考高度；默认关闭，
继续支持固定场地原点的任务。

## Fixed-Base Trajectory Behavior

`behavior_manager.trajectory.enabled: true` registers `TRAJECTORY` after ZERO.
It requires `robot_base.fixed_base: true` and no `rl_policy`. HOME and ZERO keep
their existing feedback gates and configured gains. TRAJECTORY holds
`behavior_manager.zero_pos` until an interaction request, plays joint-space NPZ
clips, and returns to that pose on completion or cancellation. It runs on the
control loop without policy inference. The RL joint-trajectory adapter shares
the same player and retains its applied-action observation feedback.

`trajectory.catalog` selects the action catalog. Optional `catalog_file` is
relative to the runtime YAML and reuses another configuration's catalog;
source `joint_indices` are mapped through that file's `robot_base.joint_names`
to the active profile's joint names. Missing joints are errors, not skipped.
Motion files resolve against the catalog configuration's `robot_base.robot_dir`.
`trajectory.kp/kd` are optional and otherwise use the ZERO gains.

`trajectory.prepare_on_start: true` opts a fixed-base profile into one-time
POWER_OFF -> HOME -> ZERO -> TRAJECTORY preparation after feedback arrives.
It keeps the HOME timing and ZERO readiness gate, then waits without playing an
action. A fault or explicit state request cancels automatic preparation;
acknowledgement does not re-enable it. The default is false; full-body RL is unchanged.

`trajectory.return_max_velocity` (rad/s, default 0) limits the nominal return
trajectory to the fixed ready pose. A positive value uses quintic interpolation
and extends `blend_out_duration` as needed. Cancellation retains motor enable
and gains; repeated cancellation does not restart the return. This is a target
speed limit, not a guarantee of measured motor speed or torque; hardware
protections remain active. Keep it at 0 for adapters returning to a moving RL target.

Fixed-base mode disables body-attitude safety checks, not joint or transport
faults. Configure optional sensors explicitly in the hardware backend; a
mathematical default base pose is not an IMU measurement. Operator clients use
the same interaction/cancel requests in RL and TRAJECTORY; locomotion velocity
and policy selection are unavailable in the standalone trajectory profile.
