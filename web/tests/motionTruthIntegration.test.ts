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
  assert.match(camera, /onResetEstop/)
  assert.match(camera, /旧任务不会自动恢复/)
})
