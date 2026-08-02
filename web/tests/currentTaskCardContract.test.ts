import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

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

test('non-terminal task card cannot be dismissed from the operator screen', () => {
  const card = readFileSync(
    new URL('../src/components/CurrentTaskCard.tsx', import.meta.url),
    'utf8',
  )

  assert.match(card, /\{terminal && \(/)
  assert.match(card, /Dismiss completed task/)
})

test('the dashboard adopts the authoritative active task instead of relying on local storage', () => {
  const app = readFileSync(
    new URL('../src/App.tsx', import.meta.url),
    'utf8',
  )

  assert.match(app, /navigationStatus\?\.mission\.raw/)
  assert.match(app, /active_task_id/)
  assert.match(app, /active_request_id/)
  assert.match(app, /currentNavigationTaskStore\.adoptAuthoritative/)
  assert.match(app, /MotionAction\.RESUME/)
  assert.match(app, /resumeAllowed=\{motionResumeGate\.allowed\}/)
})