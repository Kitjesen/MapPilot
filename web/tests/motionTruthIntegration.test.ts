import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

function source(relativePath: string): string {
  return readFileSync(new URL(`../src/${relativePath}`, import.meta.url), 'utf8')
}

const mapViewSource = source('components/MapView.tsx')

test('dashboard derives start and estop-reset permissions from one motion truth gate', () => {
  const app = source('App.tsx')
  assert.match(app, /evaluateMotionAction\(\s*MotionAction\.START/)
  assert.match(app, /evaluateMotionAction\(\s*MotionAction\.RESET_EMERGENCY_STOP/)
})

test('direct motion surfaces keep the frontend gate while dialogue goes to the owner', () => {
  const scene = source('components/SceneView.tsx')
  const map = mapViewSource
  const chat = source('components/ChatPanel.tsx')

  assert.match(scene, /motionStartAllowed/)
  assert.match(scene, /disabled=\{[^}]*!motionStartAllowed/)
  // The saved-map library opens Scene, where preview and motion admission live.
  assert.match(map, /onClick=\{onReturnLive\}><Navigation size=\{16\} \/>返回\{returnLabel\}/)
  assert.doesNotMatch(map, /api\.(?:navigateClick|sendGoal|sendWaypoints)|onPick=/)
  const instructionController = source('hooks/useInstructionTask.ts')
  assert.match(chat, /controller\.submit\(text\)/)
  assert.match(instructionController, /api\.sendInstruction\(text, current\?\.runId, controlOnly\)/)
  assert.match(instructionController, /const controlOnly = !motionStartAllowed \|\| Boolean\(current && !instructionTaskCanDismiss\(current\)\)/)
  assert.doesNotMatch(instructionController.slice(instructionController.indexOf('const rejection ='), instructionController.indexOf('if (rejection)')), /motionStartAllowed/)
  assert.match(source('components/InstructionTaskBar.tsx'), /!motionStartAllowed/)
})

test('manual map dispatch respects the active instruction owner while read-only preview stays available', () => {
  const app = source('App.tsx')
  const scene = app.slice(app.indexOf('<SceneView'), app.indexOf('/>', app.indexOf('<SceneView')))
  assert.match(app, /instructionTask\.task && !instructionTaskCanDismiss\(instructionTask\.task\)/)
  assert.match(scene, /motionStartAllowed=\{motionStartGate\.allowed && !instructionActive\}/)
  assert.match(scene, /请先结束当前指令任务，再开始地图导航/)
})

test('active task controls survive route editing and workspace changes', () => {
  const app = source('App.tsx')
  const scene = source('components/SceneView.tsx')
  assert.match(app, /const sceneWorkspace = activeTab === 'scene' \|\| activeTab === 'mapping'/)
  assert.match(app, /sceneWorkspace \? taskPanelTarget : globalTaskPanelTarget/)
  assert.match(app, /currentTaskPanelTarget && createPortal/)
  assert.match(app, /container=\{currentTaskPanelTarget\}/)
  assert.doesNotMatch(scene, /ref=\{onTaskPanelMount\}[\s\S]{0,150}hidden=/)
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

test('scene preview is read-only and single-goal admission belongs to navigation', () => {
  const scene = source('components/SceneView.tsx')
  const goalGate = scene.slice(scene.indexOf('const goalDisabledReason ='), scene.indexOf('const canSendGoal ='))
  assert.match(goalGate, /observe/)
  assert.match(goalGate, /instructionActive/)
  assert.doesNotMatch(goalGate, /canAcceptGoal|motionStartAllowed|resumeRequired/)
  const preview = scene.slice(scene.indexOf('const handlePendingGoal ='), scene.indexOf('const handleSceneRelocalize ='))
  assert.match(preview, /if \(previewDisabledReason\)/)
  assert.doesNotMatch(preview, /canSendGoal|api\.sendGoal|showToast/)
  assert.match(preview, /window\.setTimeout\(\(\) => \{\s*void handlePendingGoal\(pendingGoal\.x, pendingGoal\.y, pendingGoal\.z\)\s*\}, 220\)/)
  assert.match(preview, /rawReason === 'empty_path'[\s\S]*?'未找到可行路线'/)
  assert.match(scene, /event\.defaultPrevented \|\| goalSendPending \|\| document\.fullscreenElement\) return/)
  assert.doesNotMatch(scene, /event\.defaultPrevented \|\| goalSendPending \|\| goalPreviewPending/)
  const dispatch = scene.slice(scene.indexOf('const handleConfirmGoal ='), scene.indexOf('const handleDirectedExploration ='))
  assert.ok(dispatch.indexOf('if (!canSendGoal)') < dispatch.indexOf('api.sendGoal'))
  assert.match(dispatch, /if \(!res\.ok\)/)
  const disabled = scene.match(/onClick=\{handleConfirmGoal\}\s+disabled=\{([^}]+)\}/)?.[1]
  assert.ok(disabled)
  for (const gate of ['goalSendPending', 'goalHeadingEditing', 'goalYaw === undefined', 'goalPreviewPending', '!previewCurrent', '!plannedGoal', '!canSendGoal']) {
    assert.ok(disabled.includes(gate), `goal confirmation must retain ${gate}`)
  }
  assert.match(scene, /hasGoal \? '切换到此目标' : '开始导航'/)
  assert.doesNotMatch(scene, />重新预览<|>预览路径</)
})
