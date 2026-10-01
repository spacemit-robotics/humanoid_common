# Android 操作客户端

本 App 仅用于实机，提供配对二维码扫描和前后台断连管理。控制页面由机器人上的 HMI
服务提供，与手机浏览器共用 WebSocket v1 接口、权限检查和控制权租约。PC 仿真只使用 TUI。

## 构建

需要 JDK 17 或更新版本，以及 Android SDK platform 35 / build-tools 35.0.0。
Gradle wrapper 首次运行需要下载 Gradle、Android 插件及扫码依赖。脚本默认使用
`~/Android/Sdk`，也支持已有 `ANDROID_HOME`。

```bash
cd ~/spacemit_robot/application/native/humanoid_common/clients/android
./build_apk.sh
"$HOME/Android/Sdk/platform-tools/adb" install -r app/build/outputs/apk/debug/app-debug.apk
```

APK 独立使用 Gradle 构建，不是 `mm` 的编译目标。构建后在 `humanoid_common` 下运行
`mm`，会把已有 APK 安装到 `output/staging/share/humanoid_common/operator_web/downloads/`
并命名为 `SpacemiT-Operator.apk`。板端不需要 Android SDK；也可将 PC 编好的同名 APK
部署到该目录。实机 HMI 在同一端口提供 `/downloads/SpacemiT-Operator.apk`，扫码网页
出现“下载 Android App”，不需要 Python 下载服务器。未部署 APK 时不显示下载按钮。

手机先连接机器人 Wi-Fi。实机 HMI 用 `--real`，配置 `--listen 0.0.0.0` 和机器人固定
内网地址对应的 `--public-url`，App 扫描服务生成的 `access.svg` 或网页上的二维码即可连接，
不用手填凭据。也可粘贴完整连接链接。重启 HMI 不要求重新打印二维码。
二维码包含操作访问凭据，只交给受信任操作者；连接后手动申请控制权，不会自动上电。

切到后台、退出页面或失焦会停止持续速度输入，后台会释放控制权并断开连接。回到前台
使用当前 WebView 会话中的凭据重新验证身份，不恢复控制权或旧运动命令。配对信息
不保存到偏好设置、localStorage 或系统备份；网页用 sessionStorage 支持当前页重载。
App 使用 HTTP/WS 开发局域网连接，没有 TLS 或公网远控能力。

安装名称为“SpacemiT 控制台”；应用图标与连接页使用官方 SpacemiT logo，控制页沿用
同一品牌标识。来源见 [品牌资源](../web/assets/README.md)。

本工程只支持 Android；其他手机可直接扫描二维码使用浏览器。
