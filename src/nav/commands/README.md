# Native Navigation Command Boundary

This directory is the client side of the native navigation command boundary.
It converts operator intent into typed DDS requests and waits for the
authoritative endpoint acknowledgement. It does not contain global planning,
local planning, path following, or motor control.

```text
Gateway / GoalService / Explore
  -> Python navigation or inspection interface
  -> stable C ABI (`liblingtu_nav_client.so`)
  -> one process-wide C++ Client / CycloneDDS participant
  -> typed request + typed business ACK
  -> `nav_native_endpoint`
  -> authority, safety, planner, follower, driver
```

## Ownership

- Wire enums and validation belong to `src/message/cpp` because producers and
  consumers must share the same stable numeric values.
- `cpp/Client` owns one DDS session and exposes two narrow views:
  `navigation()` and `inspection()`.
- `client_c.h` is the stable cross-language ABI. Version and capability checks
  fail early when Python and the deployed shared library do not match.
- `nav/adapters/native/abi.py` owns the process-wide ctypes
  handle. `navigation.py` and `inspection_commands.py` are small Python-facing
  interfaces over that same handle; they never create Python DDS writers.
- The endpoint is the authority. A successful DDS write is not admission; only
  a matching accepted ACK completes command submission. For goals this means
  `planning_started`, not that planning succeeded or the robot arrived.
- A `task_id` identifies one navigation task from admission through its single
  terminal state. Goal, cancel, status, and audit records keep that identity.
- A `request_id` identifies one immutable command attempt against that task.
  Replaying the same attempt requires the same kind and payload; a distinct
  cancel or replacement attempt uses a new request ID. The native client may
  expose a suffixed `-clock-retry-N` attempt after a rejected clock-sync probe,
  while the logical request ID remains unchanged.
- An accepted command receipt proves endpoint admission only. It does not prove
  that a safe velocity was published, the motors moved, or the task reached its

## Compatibility

The endpoint's old goal, path, cancel, and teleop topic readers are explicit
legacy adapters. They are created only when
`--allow-legacy-motion-inputs=true`; field product profiles keep this disabled.

## Tests

- `src/nav/tests/test_command_client.py`: navigation Python/C ABI contract.
- `src/nav/tests/test_inspection_command_client.py`: inspection Python/C ABI
  contract.
- `cpp/test_client.cpp`: real CycloneDDS request/ACK behavior for both command
  interfaces.
