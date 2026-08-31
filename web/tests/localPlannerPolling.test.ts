import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

import {
  LOCAL_PLANNER_DIAGNOSTICS_POLL_MS,
  localPlannerSampleWarning,
  shouldPollLocalPlannerDiagnostics,
} from '../src/services/localPlannerDiagnostics.ts'

const sceneViewSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)

test('ordinary scene pages do not poll native planner diagnostics until the layer is enabled', () => {
  assert.equal(shouldPollLocalPlannerDiagnostics(false, false), false)
  assert.equal(shouldPollLocalPlannerDiagnostics(false, true), true)
  assert.equal(shouldPollLocalPlannerDiagnostics(true, false), true)
})

test('enabled planner diagnostics stay in the requested low-rate 2-5 Hz band', () => {
  const hz = 1000 / LOCAL_PLANNER_DIAGNOSTICS_POLL_MS
  assert.ok(hz >= 2 && hz <= 5, `expected 2-5 Hz, got ${hz} Hz`)
})

test('scene view exposes the opt-in layer and feeds the snapshot to Scene3D', () => {
  assert.match(sceneViewSource, /k="localPlanner"/)
  assert.match(sceneViewSource, /api\.fetchNavigationDdsSnapshot\(\)/)
  assert.match(sceneViewSource, /localPlannerSnapshot=\{localPlannerSnapshot\}/)
  assert.match(sceneViewSource, /局部安全诊断（采样）/)
})

test('bounded local-planner output is explicitly disclosed as an incomplete sample', () => {
  const warning = localPlannerSampleWarning({
    nav_endpoint: {
      local_map: {
        traversability: {
          complete: false,
          truncated: true,
          risk_cells_total: 120,
          risk_cells_returned: 32,
          unreported_cells: 'not_serialized',
        },
      },
    },
  })
  assert.match(warning ?? '', /仅显示采样风险点，不代表完整风险栅格/)
  assert.match(warning ?? '', /32\/120/)
})
