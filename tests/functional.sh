#!/usr/bin/env bash
# humanoid_common functional 用例：三个内部模块从 example 配置实例化并跑通核心流程。
# 二进制内部已对维度/字段做断言（不一致即非 0 退出），此处再校验退出码 + 成功标志。
# 由 robot-test 调用，CWD = 模块根，二进制经 staging/bin 在 PATH 中。
set -uo pipefail

EX=example
fail=0

check() {  # $1=binary  $2=config  $3=success-sentinel
  local bin="$1" cfg="$2" sentinel="$3" out
  if [[ ! -f "$cfg" ]]; then echo "[FAIL] 配置缺失: $cfg"; fail=1; return; fi
  if ! out=$(timeout 30 "$bin" "$cfg" 2>&1); then
    echo "[FAIL] $bin 退出非 0（核心流程或断言失败）"; echo "$out" | tail -8; fail=1; return
  fi
  if ! grep -q "$sentinel" <<<"$out"; then
    echo "[FAIL] $bin 未见成功标志「$sentinel」"; echo "$out" | tail -8; fail=1; return
  fi
  echo "[OK] $bin"
}

check test_robot_base         "$EX/robot_base/config_example.yaml"         "测试完成"
check test_behavior           "$EX/behavior_manager/config_example.yaml"   "测试完成"
check test_transport_executor "$EX/transport_executor/config_example.yaml" "全部测试完成"

if [[ "$fail" -ne 0 ]]; then echo "humanoid-common functional: FAILED"; exit 1; fi
echo "humanoid-common functional: PASS"
