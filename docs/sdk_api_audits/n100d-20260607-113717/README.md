# n100d ARX5 SDK API Audit - 2026-06-07 11:37 CST

This audit records the actual Python binding and dynamic-library surface exposed
by the ARX5 SDK installed on the n100d lab machine during real-arm bring-up.

## Scope

- Host: `skyswordx` (`172.19.122.197`)
- Remote repo: `/home/circlemoon/Roboclaw/references/projects/armctrl-clean-codex-20260607-102002`
- Audit directory on remote: `/home/circlemoon/Roboclaw/references/projects/armctrl-clean-codex-20260607-102002/runs/sdk-api-audit-n100d-20260607-113717`
- Python package: `arx5-interface==0.1.2`
- Python: CPython 3.12 via `uv run`
- Hardware connection used for instance-level introspection: `X5` on `can0`

## Dynamic Library Identity

- Shared object: `.venv/lib/python3.12/site-packages/arx5_interface/python/arx5_interface.cpython-312-x86_64-linux-gnu.so`
- Size on n100d: 1.1 MB
- SHA256: `f3dc1c039c184c397f302d5e0391a35bd4d59a54f18a9aecc131d7735e522f9b`
- ELF: 64-bit x86-64, stripped

See `raw/so_file_info.txt`, `raw/ldd.txt`, `raw/nm_dynamic_all.txt`,
`raw/readelf_symbols_all.txt`, and `raw/strings_all.txt`.

## Controller API Finding

The installed Python binding does not expose a hold-mode API on
`arx5_interface.Arx5JointController`.

Explicit candidate probe results from `raw/controller_instance_api_audit.txt`:

- `set_to_hold`: missing
- `set_hold`: missing
- `hold`: missing
- `set_mode`: missing
- `get_mode`: missing
- `set_to_servo`: missing
- `set_to_zero_force`: missing
- `set_to_brake`: missing
- `brake`: missing
- `stop`: missing
- `emergency_stop`: missing
- `enable`: missing
- `disable`: missing

Exposed motion/control-relevant methods include:

- `get_joint_state`
- `get_joint_cmd`
- `set_joint_cmd`
- `set_joint_traj`
- `set_to_damping`
- `reset_to_home`
- `send_recv_once`
- `recv_once`
- `set_gain`
- `get_gain`
- `get_timestamp`

The SDK package stub also confirms `set_to_damping` but no `set_to_hold`; see
`raw/package_text_grep.txt`.

## Safety Interpretation

The previously observed `sdk-hold-damping-check` failure is a backend API
contract mismatch, not a CAN or power failure:

- `set_to_damping` is callable.
- `set_to_hold` is not available in this installed Python SDK.
- No joint command was sent by the audit.
- The instance-level audit called `set_to_damping` only as cleanup.

The current `armctrl` gate correctly blocked tiny motion because it required a
proved `hold -> damping` sequence. For this SDK version, that gate cannot pass
as written.

## Required Follow-Up

Before any tiny motion, Agent real smoke, or SysID real smoke, update the ARX5
backend readiness contract to reflect the actual SDK surface:

- Do not require `set_to_hold` for `arx5-interface==0.1.2`, because it is not
  exposed by the installed Python binding.
- Represent this backend as `damping_only_landing` unless a newer SDK exposes a
  real hold/brake/stop mode.
- Keep the gate blocking real motion until the revised damping-only safety
  contract is explicit, tested, and documented.
- Add runtime SDK API discovery to avoid relying on vendor source or fake
  controller assumptions.

## Raw Files

- `raw/environment.txt`: host, PATH, repo, git, and uv context
- `raw/uv_pip_list.txt`: installed Python packages
- `raw/python_module_api_audit.txt`: module-level class and method inventory
- `raw/controller_instance_api_audit.txt`: live controller instance method
  inventory and read-only state probe
- `raw/so_file_info.txt`: shared-object identity and hash
- `raw/ldd.txt`: dynamic dependencies
- `raw/nm_dynamic_all.txt`: complete dynamic symbol dump from `nm -D -C`
- `raw/nm_dynamic_filtered.txt`: filtered dynamic symbols
- `raw/readelf_symbols_all.txt`: complete symbol table from `readelf -Ws`
- `raw/readelf_symbols_filtered.txt`: filtered symbol table
- `raw/strings_all.txt`: complete `strings` dump
- `raw/strings_filtered.txt`: filtered `strings` dump
- `raw/package_file_tree.txt`: installed package file list
- `raw/package_text_grep.txt`: grep over SDK text/stub files
- `raw/repo_vendor_grep.txt`: grep over the synced repo and venv context
