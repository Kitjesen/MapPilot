import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const statusSource = readFileSync(
  new URL('../src/components/StatusBar.tsx', import.meta.url),
  'utf8',
)
const statusStyles = readFileSync(
  new URL('../src/components/StatusBar.module.css', import.meta.url),
  'utf8',
)

test('status metrics normalize negative zero before rendering', () => {
  assert.match(statusSource, /normalizeDisplayZero/)
  assert.match(statusSource, /Math\.abs\(value\) < threshold/)
})

test('pose yaw and speed reserve stable character widths', () => {
  assert.match(statusSource, /styles\.poseValue/)
  assert.match(statusSource, /styles\.yawValue/)
  assert.match(statusSource, /styles\.speedValue/)
  assert.match(statusStyles, /\.poseValue[\s\S]*?inline-size:/)
  assert.match(statusStyles, /\.yawValue[\s\S]*?inline-size:/)
  assert.match(statusStyles, /\.speedValue[\s\S]*?inline-size:/)
  assert.match(statusStyles, /font-variant-numeric:\s*tabular-nums/)
})

test('operator status axes are rendered from the live navigation status projection', () => {
  assert.match(statusSource, /presentNavigationOperatorStatus\(navigation/)
  assert.match(statusSource, /operatorView\.task\.label/)
  assert.match(statusSource, /operatorView\.goalAdmission\.label/)
  assert.match(statusSource, /operatorView\.control\.label/)
  assert.match(statusSource, /operatorView\.motion\.permission\.label/)
  assert.match(statusSource, /operatorView\.motion\.stopConfirmation\.label/)
  assert.match(statusSource, /operatorView\.summary\.nextAction/)
  assert.doesNotMatch(statusSource, /missionStatus/)
})
