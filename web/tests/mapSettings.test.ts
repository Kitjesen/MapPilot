import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { createRequire } from 'node:module'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import * as React from 'react'
import { renderToStaticMarkup } from 'react-dom/server'
import ts from 'typescript'
import type { MapDraftResult } from '../src/services/mapDraft.ts'
import type { MapEditScene } from '../src/types/mapEdit.ts'

const require = createRequire(import.meta.url)
function component(file: string, mocks: Record<string, unknown> = {}) {
  const source = readFileSync(new URL(`../src/components/${file}.tsx`, import.meta.url), 'utf8')
  const exports: Record<string, unknown> = {}
  runInNewContext(ts.transpileModule(source, { compilerOptions: {
    target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX,
  } }).outputText, { exports, document: { body: {} }, window: { addEventListener() {}, removeEventListener() {} }, require: (name: string) => {
    if (name in mocks) return mocks[name]
    if (name.endsWith('.css')) return { default: {} }
    return require(name)
  } })
  return exports[file] as React.ComponentType<Record<string, unknown>>
}

test('processing shows a real stage and elapsed time without inventing a completion percentage', () => {
  const Progress = component('MapProgress')
  const html = renderToStaticMarkup(React.createElement(Progress, { stage: '重建导航地图 · 第 2 / 2 步', completed: '点云已保存' }))
  assert.match(html, /重建导航地图 · 第 2 \/ 2 步/)
  assert.match(html, /已用 0 秒/)
  assert.match(html, /点云已保存/)
  assert.match(html, /暂未提供完成百分比/)
  assert.match(html, /<progress aria-label=/)
  assert.doesNotMatch(html, /<progress[^>]*value=/)
})

function renderEditor(initialTool?: 'crop' | 'clean', storedDraft: MapDraftResult | null = null, target: object | null = null) {
  const effects: React.EffectCallback[] = []
  const settingsOpened: string[] = []
  const scenes: MapEditScene[] = []
  const portals: { node: React.ReactNode; target: object }[] = []
  const open = (focus: string) => settingsOpened.push(focus)
  const Editor = component('MapEditor', {
    react: { ...React, useEffect: (effect: React.EffectCallback) => { effects.push(effect) } },
    '../hooks/useMapSettings.ts': { useMapSettings: () => ({ target, setEditing() {}, open }) },
    'react-dom': { createPortal: (node: React.ReactNode, target: object) => { portals.push({ node, target }); return null } },
    './MapProgress.tsx': { MapProgress: () => null },
    '../services/api.ts': { makeRequestId: (prefix: string) => `${prefix}-test` },
    '../services/noGo.ts': {},
    '../services/mapDraft.ts': { readMapDraft: () => storedDraft },
    '../services/mapEdit.ts': { mapEditCloudBounds: () => null, suggestedEditedMapName: () => 'copy',
      applyMapEditOperations: (_points: number[]) => _points, mapEditLabel: (edit: { operation: string }) => edit.operation },
  })
  const html = renderToStaticMarkup(React.createElement(Editor, { source: { name: 'map', contentEpoch: 1, frameId: 'map' },
    initialTool, points: [], anchor: null, footerTarget: null, onSceneChange: (scene: MapEditScene) => scenes.push(scene),
    onView() {}, onClose() {}, onOpenMap() {}, onMapsChanged() {} }))
  for (const effect of effects) effect()
  return { html, settingsOpened, scenes, portals }
}

test('map cleaning has a direct action without a floating parameter card', () => {
  const { html } = renderEditor()
  assert.match(html, /aria-label="清洗点云"[^>]*>.*?>清洗<\/span>/)
  assert.doesNotMatch(html, /aria-label="点云清洗参数"/)
})

test('cleaning parameters render into the global settings target', () => {
  const target = {}
  const result = renderEditor(undefined, null, target)
  assert.equal(result.portals.length, 1)
  assert.equal(result.portals[0].target, target)
  assert.match(renderToStaticMarkup(result.portals[0].node), /aria-label="点云清洗参数"/)
  assert.doesNotMatch(result.html, /aria-label="点云清洗参数"/)
})

test('the crop tool starts scene selection on entry', () => {
  const result = renderEditor('crop')
  assert.equal(result.scenes.at(-1)?.selecting, true)
  assert.deepEqual(result.settingsOpened, [])
})

test('the cleaning tool opens cloud settings without starting scene selection', () => {
  const result = renderEditor('clean')
  assert.deepEqual(result.settingsOpened, ['cloud'])
  assert.equal(result.scenes.at(-1)?.selecting, false)
})

test('the existing editor entry waits for an explicit tool choice', () => {
  const result = renderEditor()
  assert.equal(result.scenes.at(-1)?.selecting, false)
  assert.deepEqual(result.settingsOpened, [])
})

test('a recoverable draft keeps crop entry from starting scene selection', () => {
  const result = renderEditor('crop', { kind: 'ready', draft: {
    source: { name: 'map', contentEpoch: 1, frameId: 'map' }, name: 'copy', status: 'editing',
    history: { edits: [{ operation: 'voxel', voxel_size: .1 }], cursor: 1 },
  } })
  assert.equal(result.scenes.at(-1)?.selecting, false)
  assert.match(result.html, /aria-label="框选范围"[^>]*disabled/)
  assert.match(result.html, /恢复草稿/)
})

test('map settings expose a separate no-go section while editing only renders its cleaning target', () => {
  const Settings = component('SettingsMenu', {
    '../hooks/useMapSettings.ts': { useMapSettings: () => ({ request: 1, editing: true,
      setTarget() {}, map: { name: 'office', contentEpoch: 42 } }) },
    './SurfaceSettings.tsx': { SurfaceSettings: () => React.createElement('section', { 'aria-label': '补面预览设置' }) },
    './NoGoSettings.tsx': { NoGoSettings: () => React.createElement('section', { 'aria-label': '独立禁行区设置' }) },
    '../services/api': {}, './floatingWidgetLayout': {}, '../i18n': { text: (_locale: string, _en: string, zh: string) => zh },
    'react-dom': { createPortal: (node: React.ReactNode) => node },
  })
  const html = renderToStaticMarkup(React.createElement(Settings, { open: true, onClose() {}, theme: 'light',
    resolvedTheme: 'light', onThemeChange() {}, locale: 'zh', onLocaleChange() {}, sseState: {}, localNowS: 123, showToast() {} }))
  assert.match(html, /设置分类/)
  assert.match(html, /aria-current="page"[^>]*>.*?地图/)
  assert.match(html, /点云清洗/)
  assert.match(html, /独立禁行区设置/)
  assert.match(html, /补面预览设置/)
})

test('no-go controls use the current map identity and block writes during active navigation or point editing', () => {
  const NoGo = component('NoGoSettings', {
    '../hooks/useNoGo.ts': { useNoGo: (name: string, epoch: number) => {
      assert.equal(name, 'office'); assert.equal(epoch, 42)
      return { map: { content_epoch: 42, regions: [{ id: 'door', min: [1, 2, 0], max: [2, 3, 2] }] } }
    } }, '../services/noGo.ts': {}, './MapProgress.tsx': { MapProgress: () => null },
    '../services/api.ts': { makeRequestId: (prefix: string) => `${prefix}-test` },
  })
  for (const [active, editing, expected] of [[false, false, 'enabled'], [true, false, 'active'], [false, true, 'editing']] as const) {
    const html = renderToStaticMarkup(React.createElement(NoGo, { map: { name: 'office', contentEpoch: 42,
      active, readOnly: false, onChanged() {} }, editing }))
    assert.match(html, /door/)
    assert.match(html, /X 下限 m/)
    assert.match(html, /Z 上限 m/)
    assert.equal([...html.matchAll(/<input type="number"/g)].length, 6)
    assert.match(html, /maxLength="128"/)
    assert.match(html, /显示名称/)
    assert.match(html, /玻璃障碍/)
    assert.match(html, /新增区域/)
    assert.equal(/<button[^>]*disabled[^>]*>保存禁行区/.test(html), expected !== 'enabled')
    if (expected === 'active') assert.match(html, /退出导航后编辑/)
    if (expected === 'editing') assert.match(html, /先退出点云整理/)
  }
})


test('surface parameters live in settings and explicitly describe preview geometry', () => {
  const Surface = component('SurfaceSettings', {
    '../hooks/useMapSettings.ts': { useMapSettings: () => ({ surfaceOptions: {
      headroom_m: .6, radius_m: .2, max_hole_area_m2: .16, max_slope: .7 }, setSurfaceOptions() {} }) },
  })
  const html = renderToStaticMarkup(React.createElement(Surface))
  assert.match(html, /补面预览设置/)
  assert.match(html, /最大洞面积<small>m²<\/small>/)
  assert.match(html, /最大坡度<small>m\/m<\/small>/)
  assert.match(html, /title="拟合坡度为高差 \/ 水平距离/)
  assert.match(html, /仅用于三维补面预览，不修改原始点云或导航配置。/)
  assert.match(html, /<details[^>]*><summary>预览说明<\/summary>/)
  assert.match(html, /实测支撑为青色，推断为橙色/)
  assert.match(html, /正式导航按所选机器人净空计算/)
  const inputs = [...html.matchAll(/<input type="number"[^>]*>/g)].map(match => match[0])
  assert.equal(inputs.length, 4)
  for (const [index, value] of [.2, .16, .7, .6].entries()) {
    assert.ok(inputs[index].includes(`value="${value}"`))
    assert.match(inputs[index], /required="" step="0.01"/)
    assert.ok(inputs[index].includes(`min="${index === 3 ? .01 : 0}"`))
  }
  assert.match(html, /应用预览参数/)
})
