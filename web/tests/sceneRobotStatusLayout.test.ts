import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const sceneSource = readFileSync(
  new URL('../src/components/SceneView.tsx', import.meta.url),
  'utf8',
)
const sceneStyles = readFileSync(
  new URL('../src/components/SceneView.module.css', import.meta.url),
  'utf8',
)

test('robot status normalizes negative zero before rendering telemetry', () => {
  assert.match(sceneSource, /normalizeDisplayZero/)
  assert.doesNotMatch(sceneSource, /displayRobot[XY]\s*=.*?\.toFixed\(2\)/)
})

test('robot status value slots reserve stable widths', () => {
  assert.match(sceneSource, /styles\.robotPositionValue/)
  assert.match(sceneSource, /styles\.robotYawValue/)
  assert.match(sceneSource, /styles\.robotSpeedValue/)
  assert.match(sceneStyles, /\.robotPositionValue[\s\S]*?inline-size:/)
  assert.match(sceneStyles, /\.robotYawValue[\s\S]*?inline-size:/)
  assert.match(sceneStyles, /\.robotSpeedValue[\s\S]*?inline-size:/)
})
