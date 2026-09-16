import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

function source(relativePath: string): string {
  return readFileSync(new URL(`../src/${relativePath}`, import.meta.url), 'utf8')
}

test('dashboard derives start and estop-reset permissions from one motion truth gate', () => {
  const app = source('App.tsx')
  assert.match(app, /evaluateMotionAction\(\s*MotionAction\.START/)
  assert.match(app, /evaluateMotionAction\(\s*MotionAction\.RESET_EMERGENCY_STOP/)
})

test('all web motion-start surfaces consume the shared gate', () => {
  const scene = source('components/SceneView.tsx')
  const map = source('components/MapView.tsx')
  const chat = source('components/ChatPanel.tsx')

  assert.match(scene, /motionStartAllowed/)
  assert.match(scene, /disabled=\{[^}]*!motionStartAllowed/)
  assert.match(map, /if \(!motionStartAllowed\)/)
  assert.ok(chat.indexOf('if (!motionStartAllowed)') < chat.indexOf('api.sendInstruction(text)'))
})

test('estop reset UI warns that an old task never resumes implicitly', () => {
  const camera = source('components/CameraFeed.tsx')
  const app = source('App.tsx')
  assert.match(camera, /onClick=\{onResetEstop\}/)
  assert.match(camera, /disabled=\{resetBusy \|\| !resetAllowed\}/)
  assert.match(camera, /estop && <div className=\{styles\.estopOverlay\}/)
  assert.match(camera, /旧任务不会自动恢复/)
  assert.match(app, /<Topbar[^>]*onStop=\{handleStop\}/)
  assert.doesNotMatch(camera, /onStop|btnStop/)
  assert.match(app, /if \(!estopResetGate\.allowed\)/)
  assert.ok(app.indexOf('if (!confirmed) return') < app.indexOf('await api.resetEstop()'))
})

test('dashboard compares received state with the current render clock, not the preceding UI tick', () => {
  const app = source('App.tsx')
  assert.match(app, /nowMs:\s*Math\.max\(nowMs, sseState\.lastTruthAt \?\? 0\)/)
  assert.doesNotMatch(app, /freshness = \{ nowMs, /)
})

test('scene preview is read-only while dispatch retains the shared motion gate', () => {
  const scene = source('components/SceneView.tsx')
  const preview = scene.slice(scene.indexOf('const handlePendingGoal ='), scene.indexOf('const handleSceneRelocalize ='))
  assert.match(preview, /if \(previewDisabledReason\)/)
  assert.doesNotMatch(preview, /canSendGoal|api\.sendGoal|showToast/)
  const dispatch = scene.slice(scene.indexOf('const handleConfirmGoal ='), scene.indexOf('const handleDirectedExploration ='))
  assert.ok(dispatch.indexOf('if (!canSendGoal)') < dispatch.indexOf('api.sendGoal'))
  assert.match(dispatch, /if \(!res\.ok\)/)
})
