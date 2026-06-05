# Known LingTu Production Gaps

These three gaps block production use for certain workflows. They are
documented here so anyone picking up the codebase knows the current
limitations without rediscovering them.

---

## 1. Windows is unusable (numpy segfault on import)

**Severity**: P0 -- LingTu cannot import on Windows.

**Root cause**: `pyproject.toml` (line 12) pins `numpy>=1.24,<2.0`.
The Windows pip resolver installs numpy 1.26.4, which is a MINGW-W64
build. Importing numpy triggers:
```
Warning: Numpy built with MINGW-W64 on Windows 64 bits is experimental,
CRASHES ARE TO BE EXPECTED
```
`from core.module import Module` immediately segfaults (exit 139).
`pytest src\core\tests` collects zero tests.

**Impact**: Developers on Windows cannot run any framework tests
locally. They must use WSL, a Linux VM, or SSH to the S100P robot.

**Fix**: Either:
- (Quick) Add a `pyproject.toml` note that Windows is unsupported
  and have the import path detect `sys.platform == "win32"` and raise
  a clear `RuntimeError` instead of segfaulting.
- (Proper) Switch to a numpy build that works on Windows natively,
  or document WSL as the only supported Windows workflow.

**File**: `pyproject.toml` line 12

---

## 2. SDK excluded from `pip install -e .`

**Severity**: P1 -- Customer following QuickStart hits `ModuleNotFoundError`.

**Root cause**: `pyproject.toml` line 43:
```toml
[tool.setuptools.packages.find]
where = ["."]
include = ["src*", "cli*"]
```
The `include` pattern only matches `src/` and `cli/` directories.
`lingtu_sdk/` lives at the repo root and is excluded. `pip install -e .`
installs the `lingtu` framework but not the `lingtu_sdk` package.

Customers running `from lingtu_sdk import LingTuClient` get
`ModuleNotFoundError`.

**Impact**: SDK is a user-facing product (the "LingTu Python SDK") but
cannot be installed with the standard `pip install` command. Users must
discover this manually and install with `pip install -e ./lingtu_sdk`.

**Fix**: Change `include` to `["src*", "cli*", "lingtu_sdk*"]` in
`pyproject.toml`.

**File**: `pyproject.toml` line 43

---

## 3. SDK `health()` crashes with raw exception on offline robot

**Severity**: P1 -- First user action crashes with unhelpful traceback.

**Root cause**: `lingtu_sdk/client.py` `_get()` (line 1063) and `_post()`
(line 1078) both catch `urllib.error.HTTPError` but NOT
`urllib.error.URLError`. When the robot is offline, the first API call
(e.g. `robot.health()`) raises:

```
urllib.error.URLError: <urlopen error [Errno 111] Connection refused>
```

This is an unhandled exception with a full stack trace. There is no
friendly "robot not reachable" message.

**Impact**: SDK looks broken on first launch when the robot is offline.
A customer evaluating the SDK sees a crash dump instead of a helpful
message.

**Fix**: Add `except URLError` in both `_get` and `_post` to return
`{"error": "robot not reachable", "detail": str(e)}` or re-raise as a
typed `LingTuConnectionError`.

**File**: `lingtu_sdk/client.py` lines 1063, 1078

---

## Priority

| Gap | Severity | Effort | Impact |
|-----|----------|--------|--------|
| 1. Windows segfault | P0 | 1 hour | All Windows devs blocked |
| 2. SDK not installed | P1 | 1 line | All SDK users confused |
| 3. health() crashes | P1 | 5 lines | All offline users see traceback |
