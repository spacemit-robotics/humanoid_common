#!/usr/bin/env bash
# humanoid_common error-path 用例：坏/缺配置时各内部模块必须快速失败。
# 断言：非 0 退出 + 不 hang（timeout 包裹，超时即视为 hang -> FAIL）。
set -uo pipefail

fail=0

expect_fail() {  # $@ = 待执行命令
  if timeout 15 "$@" >/dev/null 2>&1; then
    echo "[FAIL] 期望非 0 却成功: $*"; fail=1
  else
    local rc=$?
    if [[ "$rc" -eq 124 ]]; then echo "[FAIL] 命令 hang（15s 超时）: $*"; fail=1
    else echo "[OK] 正确拒绝(rc=$rc): $*"; fi
  fi
}

expect_fail test_robot_base /nonexistent/path.yaml      # 坏配置路径
expect_fail test_behavior                               # 缺参
expect_fail test_transport_executor                     # 缺参

if [[ "$fail" -ne 0 ]]; then echo "humanoid-common error-path: FAILED"; exit 1; fi
echo "humanoid-common error-path: PASS"
