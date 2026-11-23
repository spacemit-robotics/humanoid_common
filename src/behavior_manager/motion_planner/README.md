# motion_planner — 跟踪策略的参考动作规划器

跟踪类策略（SONIC、SuperMimic 等）按参考动作输出全身控制。规划器负责产生参考：
根据遥控指令实时规划，或按任务描述一次生成一段动作。本目录提供规划器接口、类型
注册表、运行宿主以及与跟踪策略的衔接。使用哪些规划器由机型 YAML 登记，进入 RL 后
由操作端启动和停止，也可配置为随策略自动启动。未启动规划器时，跟踪策略的待命参考
和目录动作与未配置规划器时相同。

实时规划器可在 `command.presets` 列出操作端可选的 `key`、`display_name` 和可选的 `heights_cm`（首项为默认高度）。网页和手柄复用同一个目录；更新运行中规划器的参数不重载模型。参数为 `key`，带高度时为 `key:厘米数`。未配置预设时沿用原有的参数接口。

## 组成

| 文件 | 内容 |
| --- | --- |
| `motion_planner.h/.cpp` | 接口 `RealtimePlanner`、`OfflinePlanner`，发布内容 `PlannerOutput`，按 `type` 注册的工厂 |
| `planner_host.h/.cpp` | `PlannerHost`：读取机型登记，在后台线程加载、启动、生成、停止规划器，上报阶段、进度和统计 |
| `planner_binding.h/.cpp` | `PlannerBinding`、`ReferenceSink`：把规划器的发布转换成跟踪策略的参考切换 |
| `../policy_adapter/sonic_motion_planner.cpp` | 已注册的类型 `sonic_planner` |
| `../../runtime/common/planner_catalog.h` | control 与操作服务共用的登记读取 |

```text
操作端 planner / planner_stop
  → hmi_runtime → Command.planner → control_runtime → StateRL → 跟踪策略 adapter
  → PlannerBinding → PlannerHost → 规划器
规划器发布 → PlannerBinding → ReferenceSink → 跟踪策略的参考
PlannerStatus → ControlStatus.planner → 操作服务 status.planner → 终端 / 网页 / 手柄 / 语音
```

## 规划器类别

| 类别 | `category` | 接口 | 发布内容 | 示例 |
| --- | --- | --- | --- | --- |
| 实时，模型驱动 | `realtime` | `RealtimePlanner`，`Kind::MODEL` | 显式轨迹，沿 RL 时间轴持续更新 | sonic-planner、motion-bricks |
| 实时，决策 | `realtime` | `RealtimePlanner`，`Kind::BEHAVIOR_TREE` | 跟踪策略动作目录中的动作名，也可以是轨迹 | 行为树在起步、慢走、转向、停步之间选择 |
| 离线 | `offline` | `OfflinePlanner` | 一段有限长度的轨迹或一个动作名，播放一遍 | kimodo、textop 等按任务描述生成 |

实时规划器从启动运行到停止，运行期间速度指令交给它。离线规划器每次请求生成一段
动作，播放结束后跟踪策略回到待命参考。`Kind` 只写入日志，不影响调度。

## 机型配置

规划器在 YAML 根节点 `motion_planners` 下登记，跟踪策略在 `policy_adapter.planners`
中按名字引用。同一个规划器可以被多个策略引用。

```yaml
motion_planners:
  sonic_planner:
    display_name: SONIC 规划行走
    type: sonic_planner            # 已注册的规划器类型
    category: realtime
    command:
      limits: {min_vx: -0.3, max_vx: 0.8, max_vy: 0.5, max_wz: 0.8}
    prepare_time: 20
    model_path: policy/sonic/planner_sonic.onnx   # 其余字段由该类型读取

rl_policy:
  onnx_infer:
    policies:
      sonic:
        policy_adapter:
          type: sonic
          catalog: sonic_actions
          planners: [sonic_planner]
          # auto_planner: sonic_planner   # 可选：进入 RL 后自动启动
```

| `motion_planners.<名字>` 字段 | 默认 | 含义 |
| --- | --- | --- |
| `type` | 必填 | 规划器类型；未注册时配置加载失败，错误信息列出已注册的类型 |
| `category` | `realtime` | `realtime` 或 `offline`，须与该类型注册的类别一致 |
| `display_name` | 规划器名 | 操作端显示名 |
| `enabled_in` | 全部环境 | `[sim]`、`[real]` 或两者。当前环境未启用的规划器不出现在操作端目录，control 拒绝启动 |
| `command.limits` | 无 | 规划器运行期间速度指令的范围，格式与策略的 `command.limits` 相同；未配置时不接受速度指令 |
| `command.teleop`、`command.smoothing` | 同策略 | 规划器运行期间的遥控档位与服务端平滑，含义与策略的同名字段相同 |
| `preload` | `false` | `true` 时在策略准备阶段加载，加载失败则策略准备失败；一个策略至多一个规划器可设 |
| `keep_loaded` | `true` | 停止后保留已加载的实例，同一次 RL 内再次启动不重新加载 |
| `stop_timeout` | `10` | 停止时等待规划器回到静止的上限 (s)，超时后直接交还参考；`0` 表示一直等待 |
| `prepare_time` | `0` | 加载并完成首次启动的预计耗时 (s)，只用于估计准备进度；`0` 表示未知 |
| `thread` | `motion_planner` | `robot_base.threads` 下的线程配置名，加载规划器前应用到后台线程，规划器在加载时创建的线程继承其核心范围和调度 |

规划器名不超过 63 字节。`policy_adapter.planners` 只用于 `sonic` 和 `supermimic`
adapter，并要求该策略配置了 `catalog`。`auto_planner` 须是 `planners` 中列出的实时
规划器。旧的 `policy_adapter.planner` 写法不再支持，加载时报错并提示迁移方式。

## 运行过程

| 阶段 | 含义 |
| --- | --- |
| `IDLE` | 未加载规划器 |
| `LOADING` / `WARMING` | 正在加载 / 模型已加载、正在预热或首次启动；跟踪策略保持待命参考 |
| `GENERATING` | 离线规划器正在生成；跟踪策略保持待命参考 |
| `RUNNING` | 跟踪策略正在跟随规划器的发布 |
| `STOPPING` | 正在回到静止并交还参考，或正在卸载 |
| `READY` | 已加载、未运行，再次启动不需要加载 |
| `FAILED` | 上一次启动、规划或生成失败，原因在 `detail` |

- 启停请求只在 RL 状态、当前策略列出了规划器时处理，其余情况由 control 拒绝并给出原因。
- 启动被拒绝的情形：规划器未被当前策略列出，当前环境未启用，另一个规划器正在占用
  参考，跟踪策略正在执行目录动作或准备切换策略。对正在运行的实时规划器再次发送启动
  请求只更新其 `argument`。
- 加载、预热和生成在宿主的后台线程执行，推理线程不等待。
- 规划器占用参考期间（准备、运行、交还）不接受目录动作请求；目录动作播放期间不接受
  规划器启动请求。
- 停止实时规划器时，宿主持续提交零速度指令，直到规划器的 `Idle()` 返回 true，再过渡
  回待命参考；超过 `stop_timeout` 时直接过渡并记录警告。准备阶段的停止直接取消。
- RL 内切换策略前先请求规划器交还参考，回到待命后再交接。
- 离开 RL 时规划器被卸载，下次进入 RL 后重新加载；`preload: true` 的规划器随下一次
  策略准备加载。
- 加载、启动、规划或生成抛出异常，发布内容不合法（关节数、帧数、非有限值、零四元数），
  或跟踪策略无法采用发布内容时，规划器进入 `FAILED`，参考回到待命，原因写入状态的
  `detail` 和 `events.log`。不产生故障，RL 继续运行。停止请求清除 `FAILED`。

状态中的数值：

- `progress`：`LOADING` / `WARMING` 时按 `prepare_time` 估计，同一进程内完成过一次加载后
  改用实测耗时，估计值上限 0.95；没有估计依据时为 0，界面显示不定进度。`GENERATING`
  时取离线规划器通过 `report_progress` 上报的值。`RUNNING` 时为目录动作或有限轨迹的
  播放进度，跟随连续轨迹时为 0。
- `plan_count`、`last_plan_ms`：实时规划器 `Stats()` 给出的规划次数和最近一次耗时。
- `plan_rate_hz`：按 2 s 窗口统计的规划频率。
- `reference_remaining_s`：当前轨迹末帧相对 RL 时间的余量。

## 新增规划器类型

1. 在 humanoid_common 新增源文件，实现 `RealtimePlanner` 或 `OfflinePlanner`，并在文件
   作用域定义一个 `PlannerTypeRegistration`。把源文件加入 `CMakeLists.txt` 中
   `behavior_manager` 的源文件列表；该库为动态库，加载时完成注册。
2. 在机型 YAML 的 `motion_planners` 登记，并写入跟踪策略的 `policy_adapter.planners`。
3. 操作服务、终端、网页和手柄从 YAML 读取目录，不需要修改。

工厂收到的 `PlannerContext` 包含规划器名、YAML 路径、配置节点 `config_base`、机型资源
目录、机器人关节数和跟踪策略的参考帧率；类型自有的参数从 `config_base` 下读取。

```cpp
// 决策类实时规划器：按速度指令在跟踪策略动作目录的 walk、stop 两个动作之间选择。
class GaitSelector final : public motion_planner::RealtimePlanner {
public:
    explicit GaitSelector(const motion_planner::PlannerContext &) {}

    void Start(const robot_base::RobotData &, double) override {
        current_.clear();
        output_.reset();
        settled_ = true;
    }

    void Update(const motion_planner::PlannerInput &input) override {
        const bool moving = input.command.vx != 0.0F || input.command.wz != 0.0F;
        const bool finished = input.playback.active && input.playback.finished;
        if (moving && (current_ != "walk" || finished)) {
            Select("walk");  // 播完后指令仍在，再播一遍
        } else if (!moving && current_ == "walk") {
            Select("stop");
        }
        // 停步动作播完后才允许交还待命参考。
        settled_ = !moving && current_ != "walk" && (current_.empty() || finished);
    }

    std::shared_ptr<const motion_planner::PlannerOutput> Output() override { return output_; }
    bool Idle() const override { return settled_; }

private:
    void Select(const std::string &action) {
        current_ = action;
        auto output = std::make_shared<motion_planner::PlannerOutput>();
        output->action = action;
        output_ = std::move(output);
    }

    std::string current_;
    std::shared_ptr<const motion_planner::PlannerOutput> output_;
    bool settled_ = true;
};

const motion_planner::PlannerTypeRegistration kGaitSelector("gait_selector", {
    motion_planner::Category::REALTIME,
    motion_planner::Kind::BEHAVIOR_TREE,
    [](const motion_planner::PlannerContext &context) -> std::unique_ptr<motion_planner::Planner> {
        return std::make_unique<GaitSelector>(context);
    },
});
```

接口约定：

- 工厂、`Start`、`Stop`、`Generate` 在宿主的后台线程调用，可以阻塞。
- `Update`、`Output`、`Idle`、`Stats` 在跟踪策略的推理线程每步调用，不得阻塞。模型推理
  等耗时计算放在规划器自己的工作线程，推理线程只交换最新指令和已完成的结果。决策
  逻辑的单步计算在微秒级时可以直接写在 `Update` 中。
- `Output()` 在内容不变时返回同一个对象。每个新对象是一次新的指令：给出动作名时，
  跟踪策略从该动作的首帧起播，重复选择同一个动作也会重新起播。
- 轨迹的关节顺序与 `RobotData::joint_pos` 相同，每帧关节数为 `context.num_dof`，至少两帧。
  实时规划器的 `start_time_s` 是第 0 帧对应的 RL 时间，与 `Start`、`Update` 收到的
  `time_s` 同一时间轴；跟踪策略按该时间轴取帧，超出末帧后保持末帧。
- `frame: WORLD` 的根位姿与 `RobotData::base_pos / base_quat` 同一坐标系；`LOCAL` 的轨迹
  自带坐标系，起播时由跟踪策略把首帧朝向对齐到机器人。
- 动作名取自跟踪策略的动作目录，过渡时长和插值由跟踪策略按目录配置执行。目录中
  没有该动作时规划器进入 `FAILED`。需要自行拼接动作片段的规划器发布轨迹。
- `PlannerInput::playback` 给出跟踪策略对最近一次发布的播放进度和是否播完，决策类
  规划器据此衔接下一个动作。`PlannerInput::command` 已按该规划器的 `command.limits` 限幅。
- 离线规划器的 `Generate` 收到 `argument`（操作端随启动请求传入的文本，最长 255
  字节）和当时的机器人状态，返回的轨迹末帧姿态应接近待命姿态。生成期间检查
  `cancelled`，置位后尽快返回；有进度信息时调用 `report_progress`。访问云端生成服务的
  规划器在 `Generate` 内完成请求和结果转换。

`sonic_motion_planner.cpp` 是模型驱动实时规划器的实现示例。
[`example/behavior_manager/fake_planners.h`](../../../example/behavior_manager/fake_planners.h)
包含三类规划器各一个最小实现。

## 跟踪策略接入规划器

跟踪策略的 adapter 实现 `ReferenceSink` 的四个操作，并持有一个 `PlannerBinding`：

| `ReferenceSink` | 跟踪策略的处理 |
| --- | --- |
| `FollowAction(action, t)` | 从当前参考过渡到目录动作并从首帧起播；目录中没有时返回原因 |
| `FollowTrajectory(trajectory, install, finite, replace, t)` | `replace` 为 true 时原位替换上一段连续轨迹；否则从当前参考过渡到该轨迹。`finite` 为 true 时从首帧播放一遍，否则按 `start_time_s` 的时间轴取帧 |
| `FollowStandby(t)` | 从当前参考过渡回待命参考 |
| `Playback(t)` | 目录动作或有限轨迹的播放进度和是否播完 |

`FollowAction` / `FollowTrajectory` 返回非空字符串表示无法采用这次发布，此时须保持原有
参考不变。整段轨迹在宿主后台校验；需要转换帧格式的策略配置 `prepare_trajectory`，返回一个只交换缓冲区的 `install` 操作。跟踪线程在保留过渡起点后调用它，旧缓冲区由后台回收。

adapter 中的调用点：

- 构造：`LoadHostConfig()` 返回的配置非空时创建 `PlannerBinding`。
- `HandlePlannerRequest()`：`SetRequest()`；`PlannerStatus()`：返回 `Status()`。
- 每个推理步在组装参考输入之前调用 `Step()`，传入限幅后的速度指令、RL 时间，以及
  跟踪策略是否正在执行目录动作或准备切换策略。
- `Engaged()` 为 true 时拒绝目录动作请求，`ReadyForPolicySwitch()` 返回 false；
  `PreparePolicySwitch()` 调用 `RequestRelease()`。
- 进入 RL 调用 `Reset()`，推理线程停止后调用 `Suspend()`。

SONIC 与 SuperMimic adapter 均按此方式接入。SuperMimic 的目标输入只使用与航向无关的
量，根部速度由轨迹的根位姿差分得到。

## 测试

```bash
ctest -R 'motion_planner|policy_adapter|supermimic|policy_switch' --output-on-failure
```

- `test_motion_planner`：类型注册、发布校验、宿主状态机、准备进度，以及
  `PlannerBinding` 与 `ReferenceSink` 的约定。
- `test_policy_adapter`、`test_supermimic`：SONIC、SuperMimic adapter 跟随三类规划器。
- `test_policy_switch`：未配置规划器的策略对启停请求的处理。
