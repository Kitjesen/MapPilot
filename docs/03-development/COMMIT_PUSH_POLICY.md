# LingTu Commit and Push Policy

Status: current repository validation policy

This policy is the project-facing checklist for every commit and push.
`AGENTS.md` remains the authoritative agent contract. This document defines
the repository's day-to-day commit and push checks.

## Where the Rules Live

| Surface | Purpose |
| --- | --- |
| `AGENTS.md` | Required agent behavior and repository boundaries. |
| `docs/03-development/COMMIT_PUSH_POLICY.md` | Human-readable commit/push acceptance checklist. |
| `docs/07-testing/README.md` | Regression suite overview and L1/L2/L3 testing layers. |

## Commit Gate

A commit is acceptable only when all of these are true:

- The diff is scoped to one coherent reason for change.
- The commit does not include unrelated local edits, generated junk, secrets, or machine-local files.
- Existing behavior is preserved unless the commit intentionally changes it.
- New or changed behavior has a focused test, or the commit explains why it cannot.
- Gateway/App/Web API changes update response schemas, manifest coverage, and client types when relevant.
- Robot-control changes identify the affected safety/control path and include the narrowest useful regression test.
- The commit subject explains the coherent reason for the change. Add a body or
  trailers when constraints, safety impact, test evidence, or known gaps are not
  already clear from the diff and final engineering report.

Minimum local check before committing:

```bash
python -m pytest tests/runtime/ -q
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
  or Gateway command-safety claims include a strict `python -m sim.diagnostics`
  summary for the affected gates.
- Hardware-facing behavior is not claimed as verified unless an S100P L3 script or field test actually ran.

Recommended Gateway/App/Web push check:

```bash
python -m pytest \
  tests/runtime/test_gateway_app_bootstrap.py \
  tests/runtime/test_gateway_route_split.py \
  tests/runtime/test_gateway_telemetry_contract.py \
  tests/runtime/test_gateway_state_snapshot.py \
  tests/runtime/test_gateway_runtime_status.py \
  tests/runtime/test_gateway_readiness.py \
  -q
```

Recommended frontend check when `web/` changes:

```bash
cd web
npm run build
```

Recommended simulation diagnostics check when navigation behavior is part of
the claim:

```bash
python -m sim.diagnostics \
  --required-only \
  --strict \
  --max-report-age-s 21600
```

Closure summaries include `report_age_s` per gate. `--max-report-age-s` keeps
older passing artifacts from supporting new simulation-backed claims.

For setup-only validation, `scripts/sim/setup_linux_validation_host.sh` writes the
setup-safe subset summary to `artifacts/sim_diagnostics_summary_setup.json`.
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
