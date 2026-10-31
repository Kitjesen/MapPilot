import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'

const html = readFileSync(new URL('../index.html', import.meta.url), 'utf8')
const sceneSource = readFileSync(new URL('../src/components/SceneView.tsx', import.meta.url), 'utf8')
const topbarSource = readFileSync(new URL('../src/components/Topbar.tsx', import.meta.url), 'utf8')

test('operator page uses the short LingTu title', () => {
  assert.match(html, /<title>LingTu<\/title>/)
})

test('technical control values stay available without crowding the main status view', () => {
  assert.match(sceneSource, /<div className=\{styles\.metricRow\}><span>导航<\/span>/)
  assert.match(sceneSource, /<summary>安全细节<\/summary>/)
  assert.match(sceneSource, /!sseState\.connected \? <div className=\{styles\.statusWaiting\}/)
  assert.match(sceneSource, /sseState\.connected \? missionStateLabel : '等待连接'/)
  assert.match(sceneSource, /className=\{styles\.statusValue\} data-state=\{navigationView\.motion\.observation\.state\}/)
  assert.match(sceneSource, /navigationView\.control\.label/)
  assert.match(sceneSource, /navigationView\.motion\.permission\.label/)
  assert.match(sceneSource, /navigationView\.motion\.stopConfirmation\.label/)
  assert.match(sceneSource, /sensorDiagnosticsVisible && <section className=\{styles\.statCard\} aria-label="雷达与定位">/)
  assert.match(sceneSource, /label="雷达诊断"/)
  assert.match(sceneSource, /仅供查看，不参与导航决策。/)
  assert.doesNotMatch(sceneSource, /styles\.controlSummary/)
})

test('scene actions live in the floating dock instead of duplicating in the canvas header', () => {
  assert.match(sceneSource, /const sceneDockControls = <>/)
  assert.match(sceneSource, /createPortal\(sceneDockControls, sceneDockTarget\)/)
  assert.match(topbarSource, /id="scene-actions-slot"/)
  assert.equal([...sceneSource.matchAll(/<span>当前扫描<\/span>/g)].length, 1)
  assert.equal([...sceneSource.matchAll(/<span>开始建图<\/span>/g)].length, 1)
  assert.match(sceneSource, /pendingGoal\.z\.toFixed\(2\)/)
})

test('canvas header does not show generic scene and environment labels', () => {
  assert.doesNotMatch(sceneSource, /现场地图/)
  assert.doesNotMatch(sceneSource, /当前环境/)
})

test('layers use a grouped settings surface with explicit point-size feedback', () => {
  assert.match(sceneSource, /className=\{styles\.layerSection\}/)
  assert.match(sceneSource, /只影响当前三维视图/)
  assert.match(sceneSource, /<output>\{pointSize\.toFixed\(2\)\}<\/output>/)
  assert.match(sceneSource, /aria-label="点云大小"/)
})
