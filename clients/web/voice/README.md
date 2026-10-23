# Web 语音接入

`panel.js` / `panel.css` 是操作网页的语音面板，复用网页的配对状态和机器人状态。
`ptt_server.py` 提供 HTTPS、录音接口和 `ptt_chat` 子进程管理；只有 Python 标准库依赖。
它不直接写 transport，不替代 HMI 服务，也不自动改变机器人状态。

```text
手机录音 -> HTTPS /voice/api/* -> ptt_chat (omni_agent)
                                     -> ASR / LLM / TTS
                                     -> MCP -> operator_client -> HMI
手机控制 -> HTTPS / WebSocket --------> HMI
```

文件随 common 安装到 `<PREFIX>/share/humanoid_common/operator_web/voice/`。
构建 common 不需要安装语音模型、omni_agent 或音频依赖；使用语音时再单独构建 omni_agent。
人设、工具配置、动作清单、声卡、证书和开机服务由机型包管理，不写在本目录中。

## 启动

使用机型包的语音脚本最方便。通用入口示例：

```bash
python3 <PREFIX>/share/humanoid_common/operator_web/voice/ptt_server.py \
  --listen '[::]:8443' --tls-cert fullchain.pem --tls-key key.pem \
  --console 127.0.0.1:8765 --operator-connection <connection.json> \
  --redirect-listen '[::]:8442' --redirect-to https://<robot-host>:8443/ \
  --chat-log <chat.log> --actions 'wave=挥手,heart=比心' -- \
  <PREFIX>/bin/ptt_chat --mcp-config <mcp.json> <voice-engine-options>
```

手机录音需要浏览器信任的 HTTPS 证书，以及能解析到机器人地址的域名。
控制页面和 WebSocket 原样转发到 HMI，不再生成或替换旧版控制页面。
`/api/v1/local-session`、`/pairing.svg` 仅供本机使用，代理始终拦截这两个入口。
`/voice/api/*` 校验连接文件的 `public_token`；旧格式没有该字段时兼容 `token`。
管理员控制授权仍由 HMI 检查，不因使用 HTTPS 或语音而绕过。

一次只有一个客户端录音；最近 30 轮对话共享显示，录音只保留最近 20 段。
`--idle-reset` 设置空闲后清空对话的秒数。`ptt_chat` 退出后默认每 15 秒重试，
期间控制页面继续转发。语音动作由 [MCP 客户端](../../mcp/README.md) 申请、续期和释放控制权。

## 验证

```bash
python3 example/operator_service/test_voice_web.py
```

测试使用假的控制服务和语音引擎，覆盖 HTTPS/WebSocket 转发、配对隔离、录音互斥、
流式回复、子进程恢复及资源回收；不加载模型、不访问机器人。
