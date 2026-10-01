# 操作服务

`hmi_runtime` 常驻运行，负责外部操作者接入。它调用 `operator_service`，再把经过
身份验证和控制权检查的请求交给内部 transport。状态机、策略推理和最终安全判断仍由
`control_runtime` 负责；driver 与 sim2sim 不依赖本模块。

```text
终端（仿真与实机）/ Android App / 浏览器（仅实机）
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
- `clients/android/`：Android 扫码、连接和生命周期外壳，使用同一份网页操作台。

Android App 不是第二套控制实现：页面仍通过 WebSocket 调用操作服务。
浏览器/Android 不直接读写 SHM，C++ 客户端也不链接 transport、RL 或电机库。

## 启动与配对

```bash
hmi_runtime /absolute/path/robot.yaml --sim
hmi_tui --connection "$HOME/.local/state/humanoid-operator/robot/connection.json"
```

`--sim` 只接受本机 TUI，不提供网页、App 配对或二维码，绑定回环地址，连接凭据仅在
本次运行有效。`--real` 启用实机客户端入口；不传模式时按 YAML 的 `driver.backend`
选择，只有 `whole_body` 启用实机入口。driver 与 HMI 的模式须一致，一键启动脚本统一传入。

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

凭据首次随机生成，私有保存为 `credential`，跨重启保留。二维码等同操作钥匙，不应
公开发布；扫描后仍须显式申请控制权，不自动上电。浏览器只在当前标签页的 sessionStorage
保留连接凭据，自动移除地址栏中的凭据，不写 localStorage。本机自动连接接口和
`/pairing.svg` 只允许回环地址、正确 Host 和同源请求；局域网必须通过已有二维码配对。

实机网页可从同一服务下载 `/downloads/SpacemiT-Operator.apk`，支持 GET/HEAD，文件位于
`operator_web/downloads/`。这是不含凭据的安装包，不要求操作权限；下载不授予控制权。
APK 缺失时返回 404，网页隐藏下载按钮；仿真仍禁用网页、App 和 APK 下载入口。
本版本 HTTP/WS 没有链路加密，只用于受信任局域网，不应暴露公网。网页按钮显示的是
control 实际回报的状态，不把“已受理”当作“已完成”。

## 操作与安全边界

- 登录可查看状态，只有一个客户端能持有控制权；接管是显式操作，不抢占其他操作者。
- 默认控制权有效期 1000 ms，客户端存活期间续租。速度另有有效期，默认 300 ms。
- 断连、后台、控制权过期或 control 反馈超时会清零速度、取消待处理请求与交互动作。
- 清零不会自动 DAMP 或掉电，避免站立机器人因遥控断连突然卸力。原 control 安全保护仍有效。
- 任意已验证客户端可执行停止、DAMP 或 POWER_OFF。DAMP/POWER_OFF 仍按原 FSM 判断。
- HOME/ZERO/RL 不跳过前置状态；RL 仍要求 ZERO 已到位，不改变原闭环阈值。
- 策略只允许在 POWER_OFF/DAMP 选择，策略与动作列表、速度范围均从当前机型 YAML 读取。
- 请求超时只说明未获确认，不自动重试；操作者应查看真实状态。重新连接不恢复旧命令。
- HMI 心跳与客户端控制权分开，HMI 进程存活不能替客户端续租。

终端按 `L` 接管，右箭头依次前进，左箭头退回；`p` 选策略，`A` 选动作，`G` 开始
手动参考动作，`v` 进入速度页。速度页保留 `Q/W/E`、`A/S/D` 六键，空格只清零速度，
`U` 释放控制权，`X` 确认故障，`Ctrl+C` 退出。速度页 `Esc` 清零后返回。
按键步长继续读取 `hmi.velocity.step_*`，默认 0.1。终端不承担扫码入口显示。

服务在现有 HMI 日志会话的 `events.log` 记录接管、释放、状态/策略/动作请求及确认结果。
不记录 token、二维码内容或每次速度心跳。完整控制与硬件轨迹仍由原 control/driver 日志负责。
在线状态要求 control 状态新鲜且已收到 HMI 心跳，不把单向状态接收当成完整控制链路。

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
| state | `{"state":"ZERO"}` | 请求状态切换，返回请求序号 |
| policy | `{"policy":"name"}` | 请求切换策略 |
| velocity | `{"vx":0.1,"vy":0,"wz":0,"valid_for_ms":300}` | 短期速度指令，服务按策略范围限幅 |
| stop | `{}` | 清零、撤销控制权和未完成请求；不是硬件急停 |
| interaction / cancel | `{"action":"key"}` / `{}` | 开始、取消交互动作 |
| reference_start | `{}` | 开始手动参考动作 |
| ack | `{}` | POWER_OFF 下确认已解除的锁存故障 |

状态包含 `online/state/zero_ready/policy/velocity/fault/owner/owns_control/interaction/request`。
`request.phase` 区分 `accepted/completed/rejected/expired/cancelled`。`completed` 表示
control 确认状态或接受动作请求，不代表整个动作播放结束；播放状态见 `interaction`。
`reference_start` 现有内部协议无独立开始确认，返回 `sent`，不伪造完成回执。
一次只能有一个待确认的状态、策略或动作请求；速度、续租和安全停止不被它阻塞。

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
