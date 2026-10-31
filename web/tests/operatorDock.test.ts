import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const source = readFileSync(new URL('../src/components/Topbar.tsx', import.meta.url), 'utf8')
const css = readFileSync(new URL('../src/components/Topbar.module.css', import.meta.url), 'utf8')

test('operator dock opens explicitly from a draggable capsule and retains an independent stop', () => {
  assert.match(source, /data-dock-drag/)
  assert.match(source, /onPointerMove=\{moveDrag\}/)
  assert.match(source, /setCollapsed\(true\)/)
  assert.match(source, /const \[collapsed, setCollapsed\] = useState\(true\)/)
  assert.match(source, /className=\{styles\.emergencyStop\}/)
  assert.match(source, /loadLayouts\(\)/)
  assert.match(source, /saveLayouts\(/)
  assert.match(source, /setCollapsed\(false\)/)
  assert.doesNotMatch(source, /onPointerEnter|setHoverPreview/)
  assert.match(source, /handle\.setPointerCapture\?\.\(event\.pointerId\)/)
  assert.doesNotMatch(source, /event\.currentTarget\.setPointerCapture/)
  assert.match(source, /id="scene-actions-slot"/)
  assert.match(css, /\.collapsed \{[^}]*border-radius: var\(--r-full\)/)
  assert.match(css, /background: var\(--bg-layer-1\)/)
})

test('operator dock keeps the LingTu mark and gives the logo a visible scene action', () => {
  assert.match(source, /<Navigation size=\{24\} strokeWidth=\{1\.8\} aria-hidden="true" \/>/)
  assert.match(source, /else if \(activeTab === 'scene'\) setCollapsed\(true\)/)
})
