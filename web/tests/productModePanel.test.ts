import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const panelSource = readFileSync(
  new URL('../src/components/ProductModePanel.tsx', import.meta.url),
  'utf8',
)
const panelStyles = readFileSync(
  new URL('../src/components/ProductModePanel.module.css', import.meta.url),
  'utf8',
)

test('compact mode control uses the themed listbox instead of a native select', () => {
  assert.match(panelSource, /function ModePicker/)
  assert.match(panelSource, /aria-haspopup="listbox"/)
  assert.match(panelSource, /role="listbox"/)
  assert.match(panelSource, /createPortal/)
  assert.doesNotMatch(panelSource, /const modeSelect = \(\s*<label[\s\S]*?<select/)
})

test('mode picker exposes descriptions, lifecycle badges, and selected state', () => {
  assert.match(panelSource, /summaryForOption\(selected, locale\)/)
  assert.match(panelSource, /policyLabel\(item\.policy, locale\)/)
  assert.match(panelSource, /aria-selected=\{isSelected\}/)
  assert.match(panelStyles, /\.modeMenuOptionSelected/)
  assert.match(panelStyles, /\.modeMenuCheckVisible/)
})

test('mode picker supports keyboard navigation and escapes card clipping', () => {
  assert.match(panelSource, /event\.key === 'ArrowDown'/)
  assert.match(panelSource, /event\.key === 'Escape'/)
  assert.match(panelStyles, /\.modeMenu\s*\{[\s\S]*?position: fixed/)
  assert.match(panelStyles, /z-index: 5000/)
})

test('strategy and map controls use the same themed floating picker', () => {
  assert.match(panelSource, /function SimplePicker/)
  assert.match(panelSource, /label=\{text\(locale, 'Strategy'/)
  assert.match(panelSource, /label=\{text\(locale, 'Map'/)
  assert.doesNotMatch(panelSource, /<span>\{text\(locale, 'Strategy'[\s\S]*?<select/)
  assert.match(panelStyles, /\.simplePickerTrigger/)
  assert.match(panelStyles, /\.simplePickerOption/)
})

test('product mode renders only the console card surface', () => {
  assert.match(panelSource, /styles\.compactCard/)
  assert.doesNotMatch(panelSource, /variant\?: 'page' \| 'card'/)
  assert.doesNotMatch(panelSource, /panel-mode/)
  assert.doesNotMatch(panelSource, /sendVisualServo/)
  assert.doesNotMatch(panelStyles, /\.page\s*\{/)
  assert.doesNotMatch(panelStyles, /\.modeGrid\s*\{/)
})
