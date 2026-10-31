import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const source = readFileSync(new URL('../src/components/SlamStatusPanel.tsx', import.meta.url), 'utf8')

test('SLAM panel distinguishes unavailable degeneracy diagnostics', () => {
  assert.match(source, /degeneracyAvailable === false/)
  assert.match(source, /退化诊断不可用/)
  assert.match(source, /awaiting_lidar_geometry/)
})

test('SLAM panel renders weak IMU modes and does not claim unapplied updates', () => {
  assert.match(source, /principal_directions/)
  assert.match(source, /PARTIAL.*UNOBSERVABLE/)
  assert.match(source, /Rx Ry Rz \/ Tx Ty Tz/)
  assert.match(source, /仅诊断，未抑制/)
  assert.match(source, /无弱方向/)
})
