import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const app = readFileSync(new URL('../src/App.tsx', import.meta.url), 'utf8')
const panel = readFileSync(new URL('../src/components/RobotStatusPanel.tsx', import.meta.url), 'utf8')
const dataflow = readFileSync(new URL('../src/components/RuntimeDataflowView.tsx', import.meta.url), 'utf8')
const api = readFileSync(new URL('../src/services/api.ts', import.meta.url), 'utf8')
const types = readFileSync(new URL('../src/types/index.ts', import.meta.url), 'utf8')
const sse = readFileSync(new URL('../src/hooks/useSSE.ts', import.meta.url), 'utf8')

test('console shows current robot state without Product switch preview', () => {
  assert.match(app, /<RobotStatusPanel/)
  assert.doesNotMatch(app, /ProductModePanel/)
  assert.match(panel, /fetchSession\(\)/)
  assert.match(panel, /globalRelocalize\(activeMap\)/)
  assert.doesNotMatch(panel, /target_product|ProductControl|copy.*command/i)
})

test('web contracts keep runtime switching out of the browser', () => {
  assert.doesNotMatch(api, /runRuntimeSwitch|prepareProductSwitch|copyProductSwitchCommand|runtime_switch/)
  assert.doesNotMatch(types, /RuntimeSwitchRequest|RuntimeSwitchResponse|runtime_switch\?:/)
  assert.doesNotMatch(dataflow, /Runtime Switch|runRuntimeSwitch|dry-run preflight/)
})

test('initial state snapshot restores visual follow availability', () => {
  assert.match(types, /visual_servo\?: VisualServoStatus/)
  assert.match(sse, /if \(d\.visual_servo\) next\.visualServoStatus = d\.visual_servo/)
})

test('tracking task shows the selected person state', () => {
  assert.match(panel, /session\?\.product === 'tracking'/)
  assert.match(types, /person: VisualServoPersonStatus \| null/)
  assert.match(panel, /visualServo\?\.person\?\.id/)
  assert.match(panel, /visualServo\?\.target_visible/)
})

test('operator can follow an explicit person track from the current scene', () => {
  assert.match(types, /target_id\?: string \| null/)
  assert.match(panel, /sceneGraph\?\.objects/)
  assert.match(panel, /sendVisualServo\(mode, target \|\| null, selectedPersonId\)/)
})
