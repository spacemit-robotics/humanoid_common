# 操作服务

`hmi_runtime` 常驻运行，负责外部操作者接入。它调用 `operator_service`，再把经过
身份验证和控制权检查的请求交给内部 transport。状态机、策略推理和最终安全判断仍由
`control_runtime` 负责；driver 与 sim2sim 不依赖本模块。

```text
终端 / 本机手柄 / Android App / 浏览器
            ↓ WebSocket JSON v1
       hmi_runtime
            ↓ operator_service
       内部 SHM / UDP
            ↓
       control_runtime
            ↓
       driver_runtime
```

## 目录与接口

- `include/operator_client.h`：C++ 客户端接口，安装为 `liboperator_client.so`。
- `include/operator_types.h`：机型无关的请求结果、状态、策略和动作类型。
- `src/operator_service/service/`：控制权、超时、确认逻辑和 HTTP/WebSocket 服务。
- `src/operator_service/protocol/`：内部 JSON 编解码；不暴露 robot_base 类型。
- `clients/terminal/`：独立终端程序 `hmi_tui`，`terminal_ui.cpp` 复用原 ANSI 四页界面。
- `clients/web/`：服务端提供的静态网页，包含手机布局和扫码配对。
- `clients/gamepad/`：evdev 输入、可配置按键和摇杆、可选震动。
- `clients/android/`：Android 扫码、连接和生命周期外壳，使用同一份网页操作台。

Android App 不是第二套控制实现：页面仍通过 WebSocket 调用操作服务。
浏览器/Android 不直接读写 SHM，C++ 客户端也不链接 transport、RL 或电机库。

机型可用 `operator_service.camera_preview: {policy: ..., port: ...}` 指定可预览的策略和本地 WebRTC 网关。
HMI 将有权限的网页会话的 SDP 协商请求转给回环地址上的 `/preview/whep`，不转发视频数据。
实机和仿真使用同一页面；切离该策略、故障、会话断开或退出管理时关闭对应视频会话。
观看无需接管控制权。

## 启动与配对

```bash
hmi_runtime /absolute/path/robot.yaml --sim
hmi_tui --connection "$HOME/.local/state/humanoid-operator/robot/connection.json"
```

仿真与实机使用同一套 TUI、网页、App 和手柄入口，默认监听回环地址。
手机连接仿真时同样需要显式配置 `--listen` 和可达的 `--public-url`；
`--sim`、`--real` 不改变客户端能力，driver 后端仍由启动方式选择。

`--terminal-only` 或 `operator_service.terminal_only: true` 可让实机仅使用本机客户端，
绑定回环地址且不提供网页和二维码，不影响 driver 后端。

服务输出本地 connection 文件路径。该文件和实例锁位于当前用户专属目录，目录
`0700`、凭据文件 `0600`。默认使用 `$HOME/.local/state/humanoid-operator/<robot>/`，
支持 `XDG_STATE_HOME`；不依赖登录终端的 `XDG_RUNTIME_DIR`。客户端与服务应使用同一
普通用户，不必为了接入 HMI 提权。connection 文件退出后移除，实机认证凭据与二维码保留。

实机默认只监听 `127.0.0.1`。手机接入时，显式开放机器人内网并配置固定入口：

```bash
hmi_runtime /absolute/path/robot.yaml --real --listen 0.0.0.0 --public-url http://192.168.1.247:8765
hmi_runtime /absolute/path/robot.yaml --real --export-pairing --public-url http://192.168.1.247:8765
```

手机先连接机器人 Wi-Fi，再扫描 `access.svg` 配对，不需要手填凭据。
固定二维码包含地址和访问凭据，可打印供受信任操作者使用，不依赖
终端显示。网页右上角也可查看、下载二维码。地址和凭据不变时重启无需重印。
`public_url` 应使用固定 IP 或可解析的固定主机名；写入 YAML 后，无界面自启动执行
同一条 `hmi_runtime robot.yaml` 即可。

凭据首次随机生成，私有保存为 `credential`，跨重启保留。未配置管理员密码时，二维码等同
操作钥匙，不应公开发布；扫描后仍须显式申请控制权，扫码本身不触发上电。浏览器只在当前标签页的 sessionStorage
保留连接凭据，自动移除地址栏中的凭据，不写 localStorage。本机自动连接接口和
`/pairing.svg` 只允许回环地址、正确 Host 和同源请求；局域网必须通过已有二维码配对。

实机网页可从同一服务下载 `/downloads/SpacemiT-Operator.apk`，支持 GET/HEAD，文件位于
`operator_web/downloads/`。这是不含凭据的安装包，不要求操作权限；下载不授予控制权。
APK 缺失时返回 404，网页隐藏下载按钮；`terminal-only` 不提供下载入口。
YAML `operator_service.voice_chat_port` 大于 0 时，扫码首页默认语音，底部保留管理员入口。
手机浏览器只在 HTTPS 页面里允许录音，所以语音由机器人上的语音服务提供：它给本控制台套一层 HTTPS
（除 `/voice/` 外的请求原样转发到本服务，含 WebSocket），并提供语音面板 `/voice/panel.js`。网页在 HTTPS
下加载面板；HTTP 入口提供语音链接，经该端口跳到 HTTPS 版（令牌在 `#` 片段里，不发给服务器），
不用重新配对。语音动作由语音服务自己申请控制权；状态里的控制者 `voice#<会话号>` 显示为「语音」。
动作尚未收回时管理员页面不允许退出管理；取消并收回后再退出。App 保留管理员控制，语音使用浏览器扫码。
本版本 HTTP/WS 没有链路加密，只用于受信任局域网，不应暴露公网。网页按钮显示的是
control 实际回报的状态，不把“已受理”当作“已完成”。

配置 `operator_service.admin_password_hash` 可限制公开扫码凭据只能查看状态、访问语音，
管理员登录后才能申请控制权或发送命令。格式为 `pbkdf2-sha256:100000:<32位盐hex>:<64位摘要hex>`；
使用随机盐生成，不能填写明文密码。密码验证在后台执行，每 30 秒最多尝试 5 次。
手机浏览器使用语音 HTTPS 入口输入密码；App 也可扫描 HTTPS 二维码，旧 HTTP 入口仅可用于受信任内网。
服务另存私有的 `credential.control`；本机 `connection.json.token` 供 TUI/MCP 使用，
`public_token` 供语音网页和二维码使用。公开二维码不变，私有控制令牌不得进入网页或二维码。
管理员授权只在当前连接有效，断线后重新登录；登录不自动获取控制权。

## 操作与安全边界

- 登录可查看状态，只有一个客户端能持有控制权；接管是显式操作，不抢占其他操作者。
- 网页控制权栏显示已确认的持有者，如“手柄已接管控制权”；横屏移动页也显示，
  释放后恢复“无人持有”，状态过期显示“等待状态确认”。只翻译显示，不改变控制权逻辑。
- 默认控制权有效期 1000 ms，客户端存活期间续租。速度另有有效期，默认 300 ms。
- 松手、速度有效期结束、断连、后台或控制权过期时，服务端将目标归零并按配置减速，取消待处理请求与交互动作。
- 明确的 `stop`、POWER_OFF/DAMP、离开 RL、控制反馈过期或故障立即清零；不为普通遥控断连请求掉电。
- 清零不会自动 DAMP 或掉电，避免站立机器人因遥控断连突然卸力。原 control 安全保护仍有效。
- 控制客户端可执行停止、DAMP 或 POWER_OFF；启用管理员密码后，公开扫码会话不能发送这些命令。
- HOME/ZERO/RL 不跳过前置状态；RL 仍要求 ZERO 已到位，不改变原闭环阈值。
- 策略在 POWER_OFF/DAMP 可选；RL 中仅允许切换到同一非空 `switch_group` 的策略。
  control 等待零速度、动作结束和新策略准备完成后执行切换。策略与动作列表及速度范围均读取 YAML。
- 策略、动作和规划器的目录按 control 回传的运行环境（仿真 / 实机）过滤，当前环境未启用的
  条目不列出，对应请求返回 `unavailable`；环境变化后目录版本更新，客户端重新读取。
- 当前策略登记了规划器时，控制客户端可在 RL 中启动和停止规划器。规划器占用参考期间
  （加载、预热、生成、运行、停止）动作请求返回 `not_ready`，动作播放期间规划器启动请求
  返回 `not_ready`。实时规划器运行时速度范围、档位和平滑取该规划器的配置。释放控制权或
  断连时速度归零，实时规划器继续运行并保持站立；离线规划器正在生成或播放的动作被停止。
- 请求超时只说明未获确认，不自动重试；操作者应查看真实状态。重新连接不恢复旧命令。
- HMI 心跳与客户端控制权分开，HMI 进程存活不能替客户端续租。

Web/WebView 的连接恢复与运动输入分开：速度回执延迟或缺失不暂停输入、不关闭连接，
请求记录到期后丢弃；服务明确拒绝时仍清除被拒绝的输入。短暂网页刷新停顿和窗口
尺寸变化保留当前按键。连续 1 秒没有状态时暂停输入，保留页面和配对；
状态连续 5 秒不更新才替换连接。
建立连接超时或连接关闭后按 0.5、1、2、4、5 秒间隔自动重试，之后维持每 5 秒重试。
恢复连接只重新验证身份、读取状态，不自动接管或重发速度、状态切换、动作请求。
页面进入后台会撤销输入、释放控制权，回到前台恢复连接；主动点“断开”后不自动
重连，凭据失效则显示重新配对入口。正常网络恢复不要求再次扫码。

### Web 与 Android 遥控

默认使用方向按键，支持双指同时按住；快慢档填入速度预设，也可在输入框中调整。
桌面 W/S/A/D 控制平移，J/L 转向。双摇杆布局适合横屏：左杆平移、右杆转向，
推幅线性映射当前档位限额；两侧独立回零，不使用按键页的手填速度。

两种布局共用 50 ms 速度发送周期。计算和发送不等待速度回执；未确认请求到期丢弃，
服务拒绝时清除相关输入。发送缓冲未清空时不追加中间速度，恢复后发送最新目标，
包括松手的零目标。摇杆仅增加空间死区与范围裁剪，不额外增加时间滤波。

`operator_service.teleop` 配置档位和服务端加减速度：

```yaml
operator_service:
  teleop:
    slow_limits: [0.3, 0.15, 0.5]  # vx/vy: m/s，wz: rad/s
    fast_limits: [0.8, 0.3, 1.0]
    acceleration: [1.0, 1.0, 2.0]  # m/s²、m/s²、rad/s²
    deceleration: [2.0, 2.0, 4.0]
```

上述数值为配置示例。档位为双向幅值上限，与策略的各方向 `command.limits` 取交集。
未配置档位时使用策略最大幅值的 40%/100%；未配置加减速度时采用上例值。
零限速方向不可操作，切换策略后重新计算范围。

各策略可在 `command.teleop` 中配置 `slow_limits`、`fast_limits`，分别覆盖全局同名档位。
数值仍与该策略的 `command.limits` 取交集，网页、WebView、手柄和 TUI 共用这些配置。
两档实际范围相同时，网页隐藏快慢档按钮；机型也应省略该策略的手柄档位切换绑定。
切换策略时，网页按新策略的档位重新填写速度，不沿用上一策略的较低数值。

客户端发送方向/推幅，服务端按当前策略和档位换算目标，在自己的时钟上按斜坡加减速，反向先降到零再加速。
客户端不再计算时间平滑，也不等待上一条速度回执。目标速度、发给 control 的速度与 control 实际采用速度分别回报。
策略配置 `command.smoothing: false` 时直接传递目标，由策略处理缓变；兼容原 `command.client_smoothing` 配置名。
默认启用服务端平滑。目录中的 `service_smoothing` 表示此配置，旧 `client_smoothing` 始终为 false，避免旧网页重复平滑。

当前策略登记了规划器时，网页在 RL 中显示规划器面板：选择规划器后点“启动”，离线
规划器另有一个任务描述输入框。加载、预热和生成期间显示进度条，control 没有进度估计
时显示不定进度；运行期间显示规划次数、单次耗时、每秒规划次数和参考余量。规划器
运行时方向键和摇杆的速度范围、快慢档切换为该规划器的配置，点“停止”后回到待命并恢复
策略自身的范围。规划器失败时面板显示原因，点“停止”清除。旁观者只看到状态。

切换页面、失焦、进入后台、释放控制权或故障会清除旧输入；重新连接不恢复旧动作。
普通松手不释放控制权。桌面空格和协议 `stop` 清零并释放控制权，网页不显示全局停止按钮。
页面刷新异常清除输入并提示错误，但不关闭正常连接。诊断 `client_event` 记录方向按下、
页面离开、脚本异常和速度拒绝，不授予控制权或发送运动命令。

### 终端

终端按 `L` 接管，右箭头依次前进，左箭头退回；`p` 选策略，`A` 选动作，`N` 选规划器，`G` 开始
手动参考动作，`v` 进入速度页。速度页保留 `Q/W/E`、`A/S/D` 六键，按当前档位持续移动；`B` 切换快慢档，空格将目标归零，
`U` 释放控制权，`X` 确认故障，`Ctrl+C` 退出。速度页 `Esc` 清零后返回。
终端没有松键事件，松开按键不会停，须按空格；退出速度页清除方向。终端不承担扫码入口显示。
网页方块按钮与终端 `C` 都发送 `cancel`，取消当前动作并按配置收回，不掉电；网页不再显示全局停止按钮。
规划器准备、运行或失败时，终端 `C` 改为发送 `planner_stop`。主页面显示规划器的阶段、
准备进度、单次规划耗时和每秒规划次数；速度页的范围随运行中的规划器切换。
终端不提供离线规划器的任务描述输入。

服务在现有 HMI 日志会话的 `events.log` 记录接管、释放、状态/策略/动作请求及确认结果。
不记录 token、二维码内容或每次速度心跳。完整控制与硬件轨迹仍由原 control/driver 日志负责。
在线状态要求 control 状态新鲜且已收到 HMI 心跳，不把单向状态接收当成完整控制链路。

### 离线音频

可选 `operator_audio` 在独立普通优先级进程观察已确认状态，播放 `audio_prompts`
配置的离线文件。支持 `state.*`、`policy.*`、`planner.*`、`action.*`、`preset.*`、`fault`、
`control.acquired/released`、`gamepad.connected/disconnected` 和服务事件；未配置的键不播。
服务事件由启动器调用 `--once`；其他事件订阅本机操作服务。

```bash
operator_audio robot.yaml --connection connection.json --dry-run
operator_audio robot.yaml --once service.ready --dry-run
```

`policy.*` 在成功进入 RL 或 RL 内完成策略切换时触发，不在发送请求时触发。
`planner.<名字>` 在该规划器进入运行、开始接管参考时触发，加载和预热期间不触发。
`audio_prompts.fault_repeat_interval_s` 设置锁存故障的重复间隔，默认 0 关闭重复；
锁存清除或状态失联时取消待播及正在播放的故障提示。

动作配乐可配置 `{file: clip.wav, music: true, max_play_seconds: 30}`；动作完成、
取消、故障、退出 RL 或状态失联时停止。`prefix: intro.wav` 表示先播前奏再播 `file`，
前奏失败或配乐被取消时不续播。机器人动作不等待音频完成。
普通提示使用短播放期限，播放子进程失败不影响控制进程。

配置 `operator_gamepad` 时，音频进程只读观察设备路径和 VID/PID，连接/挂断分别触发
手柄提示；不读取按键，不把静止输入或 HMI 重连当作设备断开，启动时未连接不播断开。

可选的 `operator_audio` 只读观察已确认的全局 `owner`：从空变为持有者产生
`control.acquired`，变为空产生 `control.released`；直接换持有者仅产生新的接管提示。
可在 `audio_prompts.clips` 绑定音频。页面连接、接管请求被拒绝、重复状态和续租
均不触发；音频观察器初次连接或重连只记录当前值，不重播旧事件。
这与 `gamepad.connected/disconnected` 的输入设备连接提示分开，不改变控制权或超时处理。

## 外部协议

WebSocket 路径 `/api/v1/ws`；JSON UTF-8 文本，版本 `v: 1`。建立连接后先验证身份：

```json
{"v":1,"id":1,"op":"hello","args":{"token":"OPERATOR_CREDENTIAL","name":"app"}}
```

`id` 是当前连接内严格递增的正整数，最大为 JavaScript 安全整数；不可重放。回复沿用同一
`id`，包含 `ok/code/message/data`；每 100 ms 主动推送 `event: "status"`。

| op | args | 作用 |
| --- | --- | --- |
| acquire / renew / release | `{}` | 申请、续租、释放控制权 |
| status / catalog / pairing | `{}` | 当前状态、能力目录、固定入口二维码 |
| subscribe_robot_feedback | `{"enabled":true}` | 订阅只读关节与 IMU 反馈；false 取消订阅，不申请控制权 |
| admin_login / admin_logout | `{"password":"..."}` / `{}` | 验证管理员密码、退出管理并释放本会话控制权 |
| state | `{"state":"ZERO"}` | 请求状态切换，返回请求序号 |
| policy | `{"policy":"name"}` | 请求切换策略 |
| velocity | `{"vx":0.1,"vy":0,"wz":0,"valid_for_ms":300}` | 短期速度指令，服务按策略范围限幅 |
| teleop | `{"vx":1,"vy":0,"wz":0,"valid_for_ms":300}` | 归一化方向/推幅，范围 -1..1，由服务端换算档位和平滑 |
| preset | `{"name":"fast"}` | 持有者选择 slow/fast，成功后在状态和语音中确认 |
| stop | `{}` | 清零、撤销控制权和未完成请求；不是硬件急停 |
| interaction / cancel | `{"action":"key"}` / `{}` | 开始、取消交互动作 |
| planner / planner_stop | `{"name":"key","argument":"..."}` / `{}` | 启动、停止当前策略登记的规划器；`argument` 可省略，离线规划器用它传任务描述，最长 255 字节 |
| reference_start | `{}` | 开始手动参考动作 |
| ack | `{}` | POWER_OFF 下确认已解除的锁存故障 |

状态包含 `online/state/zero_ready/policy/velocity/fault/owner/owns_control/interaction/request`，
以及 `environment`（`sim`、`real` 或 `unknown`）和 `planner`。
`planner` 的字段为 `name`、`phase`（`IDLE/LOADING/WARMING/READY/GENERATING/RUNNING/STOPPING/FAILED`）、
`progress`（0–1，准备进度或有限参考的播放进度，0 表示没有估计）、`plan_count`、`last_plan_ms`、
`plan_rate_hz`、`reference_remaining_s` 和 `detail`。`planner` 请求的 `completed` 表示 control
已接受启动，规划器随后经加载、预热进入 `RUNNING`，以 `planner.phase` 为准。
没有规划器在准备或运行时，`planner_stop` 直接返回成功，不向 control 发送请求。
目录中每个策略带有 `planners`（`name/display_name/category/type` 及与策略相同的速度范围字段）
和 `auto_planner`；目录的 `environment` 与状态一致。旧客户端忽略这些字段。
`target_velocity` 是限幅后的目标，`command_velocity` 是服务端平滑输出，`velocity` 是 control 的实际回报。
`velocity` 操作使用 m/s、rad/s，不套用遥控档位；`teleop` 操作使用归一化输入，两者共用服务端平滑。
感知超时回退后，`motion_rearm_required` 为 true。此时非零速度请求返回 `neutral_required`，不执行或排队。
持有控制权的客户端显式发送三轴全零，再等此状态解除后才能发送新的移动指令。网页用“恢复移动”确认，TUI 按空格，手柄回中。
网页状态额外包含 `admin/admin_login_pending/admin_login_error`；`admin_login` 返回 `pending` 后
通过这些字段确认验证结果。C++ 客户端结构和接口保持不变。
`request.phase` 区分 `accepted/completed/rejected/expired/cancelled`。`completed` 表示
control 确认状态或接受动作请求，不代表整个动作播放结束；播放状态见 `interaction`。
`reference_start` 现有内部协议无独立开始确认，返回 `sent`，不伪造完成回执。
一次只能有一个待确认的状态、策略或动作请求；速度、续租和安全停止不被它阻塞。

### 只读机器人反馈

导航或监视客户端认证后可订阅 `event: "robot_feedback"`，无需申请控制权。配置
`robot_base.joint_names` 时，名称和顺序须与 driver 的关节反馈一致，不可重复。
使用 UDP 时还需设置 [反馈端口](../transport/README.md)。

连接后调用一次 `client.SubscribeRobotFeedback()`，检查返回的 `ok`。首帧异步到达，
在客户端定时回调中读取快照，例如：

```cpp
std::pair<uint64_t, uint64_t> last_frame{};
auto poll_feedback = [&] {
    const auto feedback = client.LatestRobotFeedback();
    const auto frame = std::make_pair(feedback.stream_id, feedback.sequence);
    if (!feedback.available || !feedback.fresh || frame == last_frame) return;
    PublishJointState(feedback);  // 由调用方提供。
    last_frame = frame;
};
```

重连后重新订阅，并清空客户端的去重记录。JSON 事件的 `data` 字段：

| 字段 | 含义 |
| --- | --- |
| `available` / `fresh` | 是否有反馈 / 是否仍可作为当前测量使用 |
| `stream_id` / `sequence` | driver 发布会话编号 / 会话内的样本序号；用两者一起去重 |
| `joint_names` / `joint_positions` / `joint_velocities` / `joint_efforts` | 同序的关节名、位置（rad）、速度（rad/s）、力矩（N·m）；未提供力矩时为空数组 |
| `orientation` / `angular_velocity` / `acceleration` | 基座姿态四元数（wxyz）、角速度（rad/s）、加速度（m/s²）；遵循 driver 的坐标约定。缺失值为 JSON `null`，C++ 用 `has_base_imu` 和 `has_acceleration` 判断 |
| `frame_id` | `operator_service.feedback.frame_id` 所配置的基座坐标系，默认留空 |
| `source_time_s` / `received_at_unix_s` | driver 的源时间 / driver 首次收到该样本的系统时间，均以秒计。源时间可能是仿真时间或设备时间 |
| `age_ms` / `stale_after_ms` | 反馈年龄 / 过期阈值（取 `hmi.status_timeout`，换算为毫秒） |

跨机使用时须同步 driver、HMI 和客户端的系统时钟。只用 `available` 和 `fresh` 均为真的
样本更新 TF；基座世界位置和 `map/odom` 由定位模块提供。

## 验证与兼容

CTest 包含纯服务规则和真实 WebSocket/C++ SDK 往返测试，网络测试使用独立 UDP 的
模拟控制端，不驱动机器人。PC 完整仿真按 [测试说明](../../example/operator_service/README.md) 运行。

旧 `hmi_runtime` 不再自带终端界面。其他机型若仍用旧 `run_hmi_*.sh` 调用它，会启动服务
而非 TUI，需要另开 `hmi_tui`。灵龙脚本已拆为服务启动和终端启动两项。
直接使用内部 transport 的 ROS 2 HMI 节点不能与本服务同时担任 HMI 写端，
需迁移到客户端接口后才能同时接入。

现有 SHM 写端退出会删除通道，单独重启核心进程不保证旧读端自动重新映射。更新或重启
核心服务时应停止三个核心进程后按 driver、control、HMI 顺序启动；外部网页、App 和
TUI 客户端可独立退出、重连，无需重启核心进程。完整重启不改变固定二维码。

网页离线回归：`node example/operator_service/test_web.js`。
浏览器触屏回归：`node example/operator_service/test_web_browser.js`，需要 Playwright；
只连接回环测试页面和模拟状态。`OPERATOR_BROWSER` 指定浏览器，
`OPERATOR_WEB_TEST_OUTPUT` 指定截图目录。
