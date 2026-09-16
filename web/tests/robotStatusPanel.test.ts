import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const app = readFileSync(new URL('../src/App.tsx', import.meta.url), 'utf8')
const panel = readFileSync(new URL('../src/components/RobotStatusPanel.tsx', import.meta.url), 'utf8')
const dataflow = readFileSync(new URL('../src/components/RuntimeDataflowView.tsx', import.meta.url), 'utf8')
const api = readFileSync(new URL('../src/services/api.ts', import.meta.url), 'utf8')
const types = readFileSync(new URL('../src/types/index.ts', import.meta.url), 'utf8')
const sse = readFileSync(new URL('../src/hooks/useSSE.ts', import.meta.url), 'utf8')
const planner = readFileSync(new URL('../src/components/PlannerTuning.tsx', import.meta.url), 'utf8')
const localization = readFileSync(new URL('../src/components/LocalizationCard.tsx', import.meta.url), 'utf8')

test('console shows current robot state without Product switch preview', () => {
  assert.match(app, /<RobotStatusPanel/)
  assert.doesNotMatch(app, /ProductModePanel/)
  assert.match(panel, /fetchSession\(\)/)
  assert.match(panel, /globalRelocalize\(activeMap\)/)
  assert.doesNotMatch(panel, /target_product|ProductControl|copy.*command/i)
})

test('compact console labels Product and localization evidence without claiming motion readiness', () => {
  assert.match(panel, /'Operating mode', '运行模式'/)
  assert.match(panel, /'Localization ready', '定位就绪'/)
  assert.doesNotMatch(panel, /'Task', '任务'|'State', '状态'/)
  assert.match(localization, /sseState\.session\?\.pose_fresh/)
  assert.match(localization, /数据已过期/)
  assert.match(localization, /已收到里程计/)
  assert.doesNotMatch(localization, /已锁定|未锁定/)
})

test('web contracts keep runtime switching out of the browser', () => {
  assert.doesNotMatch(api, /runRuntimeSwitch|prepareProductSwitch|copyProductSwitchCommand|runtime_switch/)
  assert.doesNotMatch(types, /RuntimeSwitchRequest|RuntimeSwitchResponse|runtime_switch\?:/)
  assert.doesNotMatch(dataflow, /Runtime Switch|runRuntimeSwitch|dry-run preflight/)
})

test('navigation status has one public contract', () => {
  assert.match(types, /interface NavigationStatusResponse[\s\S]*?schema_version: 3/)
  assert.doesNotMatch(types, /NavigationOperatorState|operator_state|NavigationTargetSummary/)
  assert.doesNotMatch(types, /interface Navigation(?:Readiness|Progress|Path|Frame|Diagnostics|Speed|Motion|Control|Localization)/)
  assert.match(planner, /fetchAppBootstrap\(\)/)
  assert.doesNotMatch(planner, /fetchNavigationStatus|fetchSession|里程计|<span>TF<\/span>/)
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
