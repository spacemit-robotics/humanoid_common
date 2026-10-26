# Operator MCP Client

`humanoid_operator_mcp` exposes the public `operator_client` action interface to
MCP hosts. It runs as a child process, reading one JSON-RPC message per line from
stdin and writing replies to stdout. Diagnostics go to stderr.

The executable is built and installed with `humanoid_common`. Its direct build
dependencies are `operator_client`, `nlohmann_json` and `Threads`; it does not
compile or link speech recognition, speech synthesis, LLM or voice-agent code.

## Connection

```bash
humanoid_operator_mcp --connection /path/to/connection.json --name voice
```

`--robot <name>` selects the connection file written by `hmi_runtime` under
`${XDG_STATE_HOME:-$HOME/.local/state}/humanoid-operator/<name>/connection.json`.
The connection file contains credentials and must remain private.

`--actions wave,heart=Heart` restricts actions to the supplied keys and optionally
sets display names. Without this option, tools use the current action catalog
reported by the operator service. `--renew-ms` defaults to 300 ms, accepts
50-900 ms, and must remain below the service's control lease duration.

## Tools And Safety

- `perform_action(action)` acquires control, requests an available upper-body
  action, and returns after the control endpoint confirms it has started. The
  client renews its lease while the action runs and releases control afterward.
- `stop_action()` requests the existing operator-service stop operation, which
  clears velocity, cancels the action and revokes control. As before, this can
  also stop an action owned by another authenticated client.

No tool switches robot state or policy, sends velocity commands, or preempts
another owner's control lease. Execution requires an online control endpoint
and an action present in both the current catalog and the configured allow-list
(when supplied). A second action is rejected while one is running. Existing
start timeouts, maximum-action cancellation and exit cleanup are unchanged.
The MCP client does not bypass operator-service or control-layer safety checks.

Supported methods are `initialize`, `ping`, `tools/list` and `tools/call`.
Incoming notifications are ignored; catalog changes can emit
`notifications/tools/list_changed` after processing an input message. Protocol
versions remain `2024-11-05`, `2025-03-26` and `2025-06-18`.

## Tests

With `BUILD_TESTS=ON`, common builds and installs:

- `test_operator_mcp_protocol`: in-process JSON-RPC and stdio checks, with no
  service or sockets; registered as CTest `operator_mcp_protocol`.
- `test_operator_mcp`: launches `hmi_runtime` and `humanoid_operator_mcp` with a
  synthetic control endpoint over local UDP/WebSocket; registered as CTest
  `operator_mcp`. It does not start a hardware driver or inference model.

Both tests are also registered in common's `test.yaml`; no separate shell
compilation scripts are needed.
