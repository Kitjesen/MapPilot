#!/usr/bin/env bash
# install_hooks.sh - install L1 pre-commit + L2 pre-push hooks into .git/hooks/
#
# Idempotent. Re-running overwrites previous versions.

set -euo pipefail
cd "$(dirname "$0")/../.."

REPO_ROOT="$(pwd)"
HOOK_DIR="$REPO_ROOT/.git/hooks"

if [[ ! -d "$HOOK_DIR" ]]; then
  echo "ERROR: $HOOK_DIR not found - are you inside a git repo?"
  exit 1
fi

# pre-commit: pytest must be green
cat > "$HOOK_DIR/pre-commit" <<'HOOK'
#!/usr/bin/env bash
# LingTu L1 - block commit if any framework test fails.
set -euo pipefail
echo "[L1 pre-commit] running pytest src/runtime/tests/ ..."
cd "$(git rev-parse --show-toplevel)"
PYTHONIOENCODING=utf-8 python -m pytest src/runtime/tests/ -q --tb=no 2>&1 | tail -6
echo "[L1 pre-commit] OK"
HOOK

# pre-push: pytest + stub blueprint smoke
cat > "$HOOK_DIR/pre-push" <<'HOOK'
#!/usr/bin/env bash
# LingTu L2 - block push if L1 or stub smoke fails.
set -euo pipefail
cd "$(git rev-parse --show-toplevel)"
if [ "${SKIP_HEAVY_TESTS:-}" != "true" ]; then
  echo "[L2 pre-push] running pytest src/runtime/tests/ ..."
  PYTHONIOENCODING=utf-8 python -m pytest src/runtime/tests/ -q --tb=no 2>&1 | tail -6
else
  echo "[L2 pre-push] SKIPPED pytest src/runtime/tests/ (SKIP_HEAVY_TESTS=true)"
fi
echo "[L2 pre-push] running stub blueprint smoke ..."
PYTHONIOENCODING=utf-8 python -c "
import sys
sys.path.insert(0, 'src')
from runtime.blueprints.profile_builder import build_system_for_profile
# Build a real lightweight stub: stub driver, no SLAM, no native C++ nodes,
# no semantic stack - purely the framework wire-up smoke check.
system = build_system_for_profile('stub', overrides={
    'robot': 'stub',
    'slam_profile': 'none',
    'enable_native': False,
    'enable_semantic': False,
})
print('[L2] stub profile build OK - %d modules' % len(system._modules))

# Start a tighter offline runtime graph as the lifecycle smoke.  Gateway and
# map services are covered by the build above; keeping them out of this start
# avoids local port conflicts and filesystem side effects in a push hook.
runtime = build_system_for_profile('stub', overrides={
    'robot': 'stub',
    'slam_profile': 'none',
    'enable_native': False,
    'enable_semantic': False,
    'enable_gateway': False,
    'enable_map_modules': False,
    'run_startup_checks': False,
})
try:
    runtime.start()
    failed = runtime.health().get('failed_modules') or {}
    if failed:
        raise SystemExit('[L2] stub profile start failed modules: %s' % failed)
    print('[L2] stub profile start OK - %d modules' % len(runtime._modules))
finally:
    runtime.stop()
"
echo "[L2 pre-push] OK"
HOOK

chmod +x "$HOOK_DIR/pre-commit" "$HOOK_DIR/pre-push"
echo "Installed L1 pre-commit + L2 pre-push hooks at $HOOK_DIR"
echo "Bypass (emergency only): git commit --no-verify / git push --no-verify"
