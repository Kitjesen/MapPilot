# Native Navigation Command Boundary

This directory is the client side of the native navigation command boundary.
It converts operator intent into typed DDS requests and waits for the
authoritative endpoint acknowledgement. It does not contain global planning,
local planning, path following, or motor control.

```text
Gateway / GoalService / Explore
  -> Python command or inspection interface
  -> stable C ABI (`liblingtu_nav_client.so`)
  -> one process-wide C++ Client / CycloneDDS participant
  -> typed request + typed business ACK
  -> `navd`
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
  handle. `commands.py` and `inspection_commands.py` are small Python-facing
  interfaces over that same handle; they never create Python DDS writers.
- The endpoint is the authority. A successful DDS write is not admission; only
  a matching accepted ACK completes command submission. For goals this means
  `planning_started`, not that planning succeeded or the robot arrived.
- A `request_id` identifies one immutable logical command. Retries may reuse it
  only with the same kind and payload because the endpoint caches ACKs for
  idempotency.

## Typed ingress

This directory is the Host client path for typed command ingress. `navd` accepts
the declared typed request paths; the Host does not publish paths or motion
commands as an alternative control route.

## Tests

- `src/nav/tests/test_command_client.py`: navigation Python/C ABI contract.
- `src/nav/tests/test_inspection_command_client.py`: inspection Python/C ABI
  contract.
- `cpp/test_client.cpp`: real CycloneDDS request/ACK behavior for both command
  interfaces.
