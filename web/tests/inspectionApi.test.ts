import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const appSource = readFileSync(
  new URL('../src/App.tsx', import.meta.url),
  'utf8',
)
const apiSource = readFileSync(
  new URL('../src/services/api.ts', import.meta.url),
  'utf8',
)
const typesSource = readFileSync(
  new URL('../src/types/index.ts', import.meta.url),
  'utf8',
)

test('dashboard emergency stop always reports command acceptance or failure', () => {
  assert.match(appSource, /const response = await api\.sendStop\(\)/)
  assert.match(appSource, /api\.formatCommandAck\(response, '紧急停止'\)/)
  assert.match(appSource, /api\.formatCommandError\(error/)
  assert.doesNotMatch(appSource, /best-effort/)
})

test('inspection API surface exposes one task-addressed lifecycle', () => {
  assert.match(apiSource, /fetchInspectionRoutes/)
  assert.match(apiSource, /fetchInspectionRoute/)
  assert.match(apiSource, /saveInspectionRoute/)
  assert.match(apiSource, /fetchInspectionTasks/)
  assert.match(apiSource, /fetchInspectionTask/)
  assert.match(apiSource, /startInspectionTask/)
  assert.match(apiSource, /pauseInspectionTask/)
  assert.match(apiSource, /resumeInspectionTask/)
  assert.match(apiSource, /cancelInspectionTask/)
  assert.match(apiSource, /inspectionTaskActionPath/)
  assert.match(apiSource, /inspection_task_pause/)
  assert.match(apiSource, /inspection_task_resume/)
  assert.match(apiSource, /inspection_task_cancel/)
  assert.match(apiSource, /fetchInspectionStatus/)
  assert.match(apiSource, /fetchInspectionEvidence/)
  assert.match(apiSource, /inspectionEvidenceArtifactUrl/)
  assert.doesNotMatch(apiSource, /startInspectionRoute/)
  assert.doesNotMatch(apiSource, /pauseInspectionRun/)
  assert.doesNotMatch(apiSource, /resumeInspectionRun/)
  assert.doesNotMatch(apiSource, /cancelInspectionRun/)
})

test('public evidence worker status type does not expose robot filesystem paths', () => {
  const workerType = typesSource
    .split('export interface InspectionEvidenceWorkerStatus {', 2)[1]
    .split('\n}', 1)[0]
  assert.doesNotMatch(workerType, /\bevidence_root\??:/)
  assert.doesNotMatch(workerType, /\bpath\??:/)
})
