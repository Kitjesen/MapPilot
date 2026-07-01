# LingTu Commit and Push Policy

This policy is the project-facing checklist for every commit and push.
`AGENTS.md` remains the authoritative agent contract and contains the required
Lore Commit Protocol. This document explains how that protocol is applied in
day-to-day engineering work.

## Where the Rules Live

| Surface | Purpose |
| --- | --- |
| `AGENTS.md` | Required agent behavior, repository boundaries, and Lore commit message format. |
| `docs/07-testing/COMMIT_PUSH_POLICY.md` | Human-readable commit/push acceptance checklist. |
| `docs/07-testing/README.md` | Regression suite overview and L1/L2/L3 testing layers. |
| `docs/07-testing/install_hooks.sh` | Local hook installer for the L1 pre-commit and L2 pre-push gates. |

## Commit Gate

A commit is acceptable only when all of these are true:

- The diff is scoped to one coherent reason for change.
- The commit does not include unrelated local edits, generated junk, secrets, or machine-local files.
- Existing behavior is preserved unless the commit intentionally changes it.
- New or changed behavior has a focused test, or the commit explains why it cannot.
- Gateway/App/Web API changes update response schemas, manifest coverage, and client types when relevant.
- Robot-control changes identify the affected safety/control path and include the narrowest useful regression test.
- The commit message follows the Lore Commit Protocol from `AGENTS.md`: the first line explains why, and useful trailers record constraints, rejected alternatives, confidence, scope risk, tested evidence, and known gaps.

Minimum local check before committing:

```bash
python -m pytest src/runtime/tests/ -q
```

Use a narrower focused suite first while iterating, then run the broader suite
before creating the commit.

## Push Gate

A push is acceptable only when all commit-gate rules are satisfied and:

- The branch is synchronized with its upstream, or any merge/rebase conflict has been resolved and retested.
- The working tree has no accidental tracked changes.
- L1 and L2 gates pass, either through installed hooks or manually.
- Web changes pass `npm run build` from `web/`.
- Gateway/API changes pass the Gateway contract tests listed below.
- Simulation-backed navigation, planning, localization, tracking, exploration,
  or Gateway command-safety claims include a strict `server_sim_closure.py`
  summary for the affected gates.
- Hardware-facing behavior is not claimed as verified unless an S100P L3 script or field test actually ran.

Recommended Gateway/App/Web push check:

```bash
python -m pytest \
  src/runtime/tests/test_gateway_app_bootstrap.py \
  src/runtime/tests/test_gateway_route_split.py \
  src/runtime/tests/test_gateway_telemetry_contract.py \
  src/runtime/tests/test_gateway_state_snapshot.py \
  src/runtime/tests/test_gateway_runtime_status.py \
  src/runtime/tests/test_gateway_readiness.py \
  -q
```

Recommended frontend check when `web/` changes:

```bash
cd web
npm run build
```

Recommended server simulation closure check when navigation behavior is part of
the claim:

```bash
bash docs/07-testing/l25_fresh_closure.sh
```

Closure summaries include `report_age_s` per gate. The wrapper enforces
`--max-report-age-s`, defaulting to 21600 seconds, so older passing artifacts do
not silently support new simulation-backed claims.

For setup-only validation, `scripts/deploy/setup_server_ros_pct.sh` writes the
setup-safe subset summary to `artifacts/server_sim_closure_summary_setup.json`.
When Gateway is running, `/api/v1/diagnostics/routecheck/latest` also exposes
the latest routecheck `report_age_s`, no-motion flags, and `published` counters
for operator review.

## Bypass Rule

`git commit --no-verify` or `git push --no-verify` is allowed only for an
emergency or a broken local toolchain. The commit message, PR description, or
final engineering note must include:

- why the bypass was necessary,
- which checks were skipped,
- how the skipped checks will be recovered,
- whether the change is safe to deploy to S100P.

## Reporting Standard

Every final report after a commit or push should include:

- branch and commit hash,
- changed files or change areas,
- verification commands and pass/fail result,
- known untested areas,
- whether the remote branch is synchronized.
