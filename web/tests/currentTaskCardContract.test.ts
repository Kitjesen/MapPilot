import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

function taskGuidance(state: string, changes: Record<string, unknown> = {}): string {
  const source = readFileSync(new URL('../src/components/CurrentTaskCard.tsx', import.meta.url), 'utf8')
  const file = ts.createSourceFile('CurrentTaskCard.tsx', source, ts.ScriptTarget.Latest, true, ts.ScriptKind.TSX)
  const declaration = file.statements.find(node => ts.isFunctionDeclaration(node) && node.name?.text === 'nextActionLabel')
  assert.ok(declaration)
  const code = ts.transpileModule(`${declaration.getText(file)}; nextActionLabel(state, 'zh', options)`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText
  return runInNewContext(code, {
    state,
    options: { terminal: false, stale: false, snapshotFound: true, waitingForRoute: false,
      pendingAction: null, pendingState: 'idle', cancelState: 'idle', resumeAllowed: true,
      resumeBlockedReason: '', ...changes },
    text: (_locale: string, _en: string, zh: string) => zh,
  }) as string
}

test('confirmed cancellation replaces pending stop guidance only after the terminal result', () => {
  assert.equal(taskGuidance('EXECUTING', { cancelState: 'requested' }), '等待机器人确认停车')
  assert.equal(taskGuidance('CANCELLED', { terminal: true, cancelState: 'requested' }), '任务已取消')
})

test('stale active status never presents a pending request as current robot feedback', () => {
  assert.equal(taskGuidance('EXECUTING', { stale: true, pendingAction: 'pause', pendingState: 'requested' }), '等待任务状态恢复连接')
  assert.equal(taskGuidance('EXECUTING', { stale: true, cancelState: 'requested' }), '等待任务状态恢复连接')
})

test('unconfirmed tasks and blocked continuation give the operator the actual next condition', () => {
  assert.equal(taskGuidance('UNKNOWN', { snapshotFound: false }), '等待机器人确认任务')
  assert.equal(taskGuidance('PAUSED', { resumeAllowed: false, resumeBlockedReason: '定位尚未恢复' }), '定位尚未恢复')
})

test('current task cancel remains requested until native terminal status arrives', () => {
  const card = readFileSync(
    new URL('../src/components/CurrentTaskCard.tsx', import.meta.url),
    'utf8',
  )

  assert.match(card, /cancelNavigationTask\(identity\.task_id\)/)
  assert.match(card, /status !== 'cancel_requested'/)
  assert.match(card, /Cancel requested; waiting for stop confirmation/)
  assert.doesNotMatch(card, /cancelNavigationTask[\s\S]{0,400}dismiss\(\)/)
})

test('current task pause and resume are task-specific and keep cancel available', () => {
  const card = readFileSync(
    new URL('../src/components/CurrentTaskCard.tsx', import.meta.url),
    'utf8',
  )

  assert.match(card, /pauseNavigationTask\(identity\.task_id\)/)
  assert.match(card, /resumeNavigationTask\(identity\.task_id\)/)
  assert.match(card, /status !== 'pause_requested'/)
  assert.match(card, /status !== 'resume_requested'/)
  assert.match(card, /Task resume failed/)
  assert.match(card, /task\.canResume/)
  assert.match(card, /resumeAllowed/)
  assert.doesNotMatch(card, /api\.resumeNavigation\(/)
  assert.doesNotMatch(card, /disabled=\{pendingAction !== null\}[\s\S]{0,220}cancelNavigationTask\(identity\.task_id\)/)
  assert.match(card, /isTaskControlAwaitingConfirmation/)
  assert.match(card, /state: 'requested'/)
  assert.doesNotMatch(card, /finally\s*\{[\s\S]{0,100}setPendingAction\(null\)/)
})

test('task card never projects an ambiguous bare numeric state as a lifecycle result', () => {
  const card = readFileSync(
    new URL('../src/components/CurrentTaskCard.tsx', import.meta.url),
    'utf8',
  )

  assert.doesNotMatch(card, /STATE_NAMES/)
  assert.doesNotMatch(card, /numericState/)
  assert.match(card, /STATE_LABELS\[task\.state\]/)
})

test('task card shares the operator task vocabulary while keeping task-specific polling', () => {
  const card = readFileSync(
    new URL('../src/components/CurrentTaskCard.tsx', import.meta.url),
    'utf8',
  )

  assert.match(card, /useCurrentNavigationTask\(\)/)
  assert.match(card, /navigationTaskLabel\(task\.state, locale, rawReason\)/)
})

test('non-terminal task card cannot be dismissed from the operator screen', () => {
  const card = readFileSync(
    new URL('../src/components/CurrentTaskCard.tsx', import.meta.url),
    'utf8',
  )

  assert.match(card, /\{terminal && \(/)
  assert.match(card, /Dismiss completed task/)
})

test('task bar presents one current task without inventing a destination', () => {
  const card = readFileSync(
    new URL('../src/components/CurrentTaskCard.tsx', import.meta.url),
    'utf8',
  )

  assert.match(card, /taskName\?: string/)
  assert.match(card, /cleanText\(taskName\) \|\| text\(locale, 'Navigation task', '导航任务'\)/)
  assert.match(card, /className=\{styles\.primaryControl\}/)
  assert.match(card, /className=\{styles\.moreButton\}/)
  assert.doesNotMatch(card, /前往会议室/)
  assert.doesNotMatch(card, /status legend|state legend/i)
})

test('feedback interruption keeps task-specific cancellation discoverable in details', () => {
  const card = readFileSync(
    new URL('../src/components/CurrentTaskCard.tsx', import.meta.url),
    'utf8',
  )

  assert.match(card, /<details key=\{identity\.task_id\}/)
  assert.match(card, /!terminal && task\.canCancel/)
  assert.match(card, /cancelNavigationTask\(identity\.task_id\)/)
})

test('the dashboard adopts the authoritative active task instead of relying on local storage', () => {
  const app = readFileSync(
    new URL('../src/App.tsx', import.meta.url),
    'utf8',
  )

  assert.match(app, /const liveNavigation = liveNavigationStatus\(sseState, nowMs \/ 1000\)/)
  assert.match(app, /const activeTaskId = liveNavigation\?\.task\.task_id/)
  assert.doesNotMatch(app, /activeRequestId/)
  assert.doesNotMatch(app, /mission\.raw|active_task_id|active_request_id/)
  assert.match(app, /currentNavigationTaskStore\.adoptAuthoritative/)
  assert.match(app, /MotionAction\.RESUME/)
  assert.match(app, /resumeAllowed=\{motionResumeGate\.allowed\}/)
})
