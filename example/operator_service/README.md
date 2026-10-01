# 操作服务验证

PC 验证使用真实 driver、control、HMI 服务与机型策略，但 driver 连接 MuJoCo，
不代替 K3 编译或实机验收。正常使用无需生成临时 YAML 或 export 配置变量。

## 编译

```bash
cd ~/spacemit_robot
source build/envsetup.sh
cd application/native/humanoid_common
mm
cd ../humanoid_linglong
mm
```

## PC 启动

每个新终端先执行：

```bash
cd ~/spacemit_robot
source build/envsetup.sh
```

一条命令启动三个核心进程及原 TUI：

```bash
run_linglong.sh --sim
```

仿真仅支持 TUI，不提供网页、App、二维码或局域网监听。Ctrl+C 退出本次启动的全部
进程。要分别观察启动日志，也可依次执行：

| 终端 | 命令 |
| --- | --- |
| 1 | `run_driver_linglong.sh --sim` |
| 2 | `run_control_linglong.sh` |
| 3 | `run_hmi_linglong.sh --sim` |

等待 driver 初始化完成再启动 control。driver 与 HMI 均传入 `--sim`，同一机型 YAML 的
策略、增益、安全阈值和内部通信配置不变。

使用分开的三条命令时，另开终端执行：

```bash
run_hmi_tui_linglong.sh
```

`L` 申请控制权，右箭头依次 DAMP/HOME/ZERO，等待 ZERO 到位后再次右箭头进入 RL。
`p` 在掉电或阻尼状态选择策略，`A` 选择交互动作，`G` 开始手动参考动作。
速度页 `v` 保留 `Q/W/E`、`A/S/D`；空格清零速度，`U` 释放控制权，`Ctrl+C` 退出。

## 实机固定二维码与手机

以下仅用于 K3 实机。手机连接机器人自带路由器的 Wi-Fi；入口使用该路由器为 K3
保留的固定内网地址，不使用开发 PC/虚拟机地址。先停止原有三个核心进程，再启动：

```bash
run_linglong.sh --real --listen 0.0.0.0 --public-url http://192.168.1.247:8765
```

展示部署时一次性写入 YAML，之后保持普通的三条启动命令即可：

```yaml
operator_service:
  bind_address: "0.0.0.0"
  port: 8765
  public_url: "http://192.168.1.247:8765"
  lease_ms: 1000
```

地址应是固定 IP 或手机可解析的固定主机名。不能用 `0.0.0.0` 或 `127.0.0.1` 作为手机入口。
服务自动保存以下文件，默认不受登录终端和系统服务的 `XDG_RUNTIME_DIR` 差异影响：

- `~/.local/state/humanoid-operator/linglong/access.svg`：包含连接凭据的固定二维码，仅供受信任操作者。
- `~/.local/state/humanoid-operator/linglong/credential`：私有操作凭据，重启不变。
- `~/.local/state/humanoid-operator/linglong/connection.json`：本机客户端连接文件，服务退出后删除。

也可在没有 driver/control、没有运行中的 HMI 服务时导出二维码：

```bash
run_hmi_linglong.sh --real --export-pairing --public-url http://192.168.1.247:8765
```

手机浏览器或 App 扫码自动连接，再手动申请控制权，不用输入密码或凭据。网页右上角
可查看、下载同一二维码，也可直接打印 `access.svg`；不依赖终端按键。
二维码等同操作钥匙，不应放到公开宣传材料中。服务重启后原二维码仍可使用，
客户端不会自动恢复控制权或运动。
现有 SHM 不保证核心进程单独热重启；重启服务时停止三个核心进程，再按上面的顺序启动。
网页、App 或 TUI 自身可独立重连，不受这个限制影响。
修改访问地址需重新打印二维码。手机浏览器兼容性仍需设备验收；App 当前只支持 Android。

终端与手机可同时查看，只有一个能持有控制权。方向按钮按住运动、松开或失焦清零；
网页关闭、App 后台或客户端失联会释放控制权。不会自动 DAMP 卸力。HTTP/WS 仅用于
受信任局域网，不应暴露公网。

## Android APK

需要 JDK 17+、SDK platform 35、build-tools 35.0.0。构建脚本默认查找 `~/Android/Sdk`，
也支持已有 `ANDROID_HOME`，不要求手动 export：

```bash
cd ~/spacemit_robot/application/native/humanoid_common/clients/android
./build_apk.sh
```

APK 输出为 `app/build/outputs/apk/debug/app-debug.apk`。手机连接到电脑后可安装：

```bash
"$HOME/Android/Sdk/platform-tools/adb" install -r app/build/outputs/apk/debug/app-debug.apk
```

App 扫码或粘贴完整连接链接后操作。切到后台释放控制权；回到前台只恢复查看，
不会重放动作或自动接管。实际手机相机、系统生命周期和 K3 网络仍需设备验收。

## 离线与隔离测试

```bash
cd ~/spacemit_robot
ctest --test-dir output/build/cmake/pkgs/application_native_humanoid_common --output-on-failure
```

`operator_service` 验证请求规则；`operator_network` 使用真实 HMI 服务、C++ SDK 和
独立模拟 UDP 控制端验证认证、控制权、有效期、断连、动作、持久二维码，以及本机
配对接口的 Host/Origin 检查。它不驱动电机。

`operator_terminal_only` 复用同一测试，验证仿真 TUI 可以完成状态切换，但网页、
配对和二维码接口不可用。实机网页服务的离线协议回归不代表 PC 仿真提供手机入口。

`prepare_simulation.py` 只供自动化测试隔离 SHM/日志，日常启动不需要它。

## 实机

完成板端同步编译后，依次运行 driver `--real`、control、HMI；使用任意客户端操作。
必须先可靠吊装，确认没有旧 HMI/ROS 2 节点同时占用内部 HMI 写端。该操作服务不改变
原有实机安全阈值。开机自启动的安装与管理参见机型仓库 README；服务
运行时不再手动启动另一套核心进程，可单独连接 TUI、网页或 App。
