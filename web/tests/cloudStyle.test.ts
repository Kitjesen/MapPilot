import assert from 'node:assert/strict'
import test from 'node:test'
import { CLOUD_PALETTES, DEFAULT_CLOUD_STYLE, readCloudStyle, saveCloudStyle } from '../src/services/cloudStyle.ts'
import { PCD_POINT_SIZE_CSS } from '../src/services/pcdPicking.ts'

function withStorage(check: (values: Map<string, string>) => void) {
  const previous = Object.getOwnPropertyDescriptor(globalThis, 'localStorage')
  const values = new Map<string, string>()
  Object.defineProperty(globalThis, 'localStorage', { configurable: true, value: {
    getItem: (key: string) => values.get(key) ?? null,
    setItem: (key: string, value: string) => values.set(key, value),
  } })
  try { check(values) }
  finally {
    if (previous) Object.defineProperty(globalThis, 'localStorage', previous)
    else Reflect.deleteProperty(globalThis, 'localStorage')
  }
}

test('missing or unreadable cloud preferences use the selected display preset', () => {
  withStorage(values => {
    for (const stored of [null, '{invalid']) {
      values.clear()
      if (stored !== null) values.set('lingtu.map.cloudStyle', stored)
      const style = readCloudStyle()
      assert.equal(style.palette, 'height')
      assert.equal(CLOUD_PALETTES.find(palette => palette.id === style.palette)?.label, '多色高度')
      assert.equal(style.pointSize, 1.1)
      assert.equal(style.opacity, .29)
      assert.equal(style.background, 'theme')
      assert.equal(style.grid, true)
      assert.deepEqual(style, DEFAULT_CLOUD_STYLE)
    }
  })
  assert.equal(PCD_POINT_SIZE_CSS, 2.2, 'the saved-map display preset does not change the picking default')
})

test('saved cloud choices survive read and save while older partial preferences inherit defaults', () => {
  withStorage(values => {
    const chosen = { ...DEFAULT_CLOUD_STYLE, palette: 'custom' as const, lowColor: '#123456', highColor: '#fedcba',
      pointSize: 4.2, opacity: .8, background: 'custom' as const, backgroundColor: '#334455', grid: false }
    saveCloudStyle(chosen)
    assert.deepEqual(readCloudStyle(), chosen)
    values.set('lingtu.map.cloudStyle', JSON.stringify({ palette: 'blue', pointSize: 3.5, opacity: .6 }))
    assert.deepEqual(readCloudStyle(), { ...DEFAULT_CLOUD_STYLE, palette: 'blue', pointSize: 3.5, opacity: .6 })
  })
})

test('previous monochrome preferences return to height colors without resetting display choices', () => {
  withStorage(values => {
    values.set('lingtu.map.cloudStyle', JSON.stringify({ palette: 'mono', monoColor: '#123456',
      pointSize: 1.8, opacity: .4, background: 'dark', grid: false }))
    const style = readCloudStyle()
    assert.deepEqual(style, { ...DEFAULT_CLOUD_STYLE, palette: 'height',
      pointSize: 1.8, opacity: .4, background: 'dark', grid: false })
    saveCloudStyle(style)
    assert.deepEqual(readCloudStyle(), style)
    assert.equal(values.get('lingtu.map.cloudStyle')?.includes('mono'), false)
    assert.ok(CLOUD_PALETTES.every(palette => String(palette.id) !== 'mono'))
  })
})
