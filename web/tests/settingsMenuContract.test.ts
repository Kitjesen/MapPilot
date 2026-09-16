import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { resetAllLayouts } from '../src/components/floatingWidgetLayout.ts'

const settings = readFileSync(new URL('../src/components/SettingsMenu.tsx', import.meta.url), 'utf8')

test('primary navigation and More retain every page without exposing controls in observation mode', () => {
  const topbar = readFileSync(new URL('../src/components/Topbar.tsx', import.meta.url), 'utf8')
  assert.match(topbar, /const observe = isObservationMode\(\)/)
  assert.match(topbar, /observe \? <span[^>]*className=\{styles.viewLabel\}/)
  assert.match(topbar, /name="lingtu-menu"/)
  assert.match(topbar, /onTabChange\(page.key\)/)
  for (const tab of ['scene', 'console', 'map', 'slam', 'inspection', 'dataflow', 'planner']) {
    assert.match(topbar, new RegExp(`key: '${tab}'`))
  }
  assert.doesNotMatch(topbar, /debug_nav|role="tablist"|>LingTu</)
  assert.doesNotMatch(settings, /WORKSPACE_PAGES|onNavigateTab|authLogin|authCheck|Sign Out|OtaModal|alert\(|localStorage\.clear\(/)
})

test('system settings read real health, report failures and offer the actual GET download', () => {
  assert.match(settings, /api\.fetchHealth\(\)/)
  assert.match(settings, /error && <p[^>]+role="alert"/)
  assert.match(settings, /snapshot\?\.data\.modules/)
  assert.match(settings, /snapshot\.receivedAt\.toLocaleTimeString/)
  assert.match(settings, /<a[^>]+href="\/api\/v1\/diagnostic_pack" download>/)
  assert.match(settings, /section === 'system' && <SystemSettings/)
  assert.doesNotMatch(settings, /setInterval|Already up to date|currentVersion|currentCommit/)
})

test('keyboard focus stays inside settings including the download link', () => {
  assert.match(settings, /button:not\(:disabled\), a\[href\]/)
  assert.match(settings, /previousFocus\?\.focus\(\)/)
  assert.match(settings, /document\.removeEventListener\('keydown', onKey\)/)
})

test('reset layout refreshes the page without erasing theme or language preferences', () => {
  const previousStorage = Object.getOwnPropertyDescriptor(globalThis, 'localStorage')
  const previousWindow = Object.getOwnPropertyDescriptor(globalThis, 'window')
  const saved = new Map([
    ['lingtu-widget-layouts-v4', '{}'], ['lingtu-widget-layouts-v3', '{}'],
    ['lingtu-theme', 'dark'], ['lingtu-locale', 'zh'],
  ])
  let reloads = 0
  Object.defineProperty(globalThis, 'localStorage', { configurable: true, value: {
    removeItem: (key: string) => saved.delete(key),
  } })
  Object.defineProperty(globalThis, 'window', { configurable: true, value: {
    location: { reload: () => { reloads += 1 } },
  } })
  try {
    resetAllLayouts()
    assert.deepEqual([...saved], [['lingtu-theme', 'dark'], ['lingtu-locale', 'zh']])
    assert.equal(reloads, 1)
  } finally {
    if (previousStorage) Object.defineProperty(globalThis, 'localStorage', previousStorage)
    else Reflect.deleteProperty(globalThis, 'localStorage')
    if (previousWindow) Object.defineProperty(globalThis, 'window', previousWindow)
    else Reflect.deleteProperty(globalThis, 'window')
  }
})
