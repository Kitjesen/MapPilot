import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { createRequire } from 'node:module'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import * as React from 'react'
import * as jsx from 'react/jsx-runtime'
import { renderToStaticMarkup } from 'react-dom/server'
import ts from 'typescript'
import * as presentation from '../src/services/mapSavePresentation.ts'
import * as thumbnails from '../src/services/mapThumbnail.ts'
import type { MapEditScene } from '../src/types/mapEdit.ts'

const require = createRequire(import.meta.url)
const source = readFileSync(new URL('../src/components/MapGallery.tsx', import.meta.url), 'utf8')
const code = ts.transpileModule(source, { compilerOptions: {
  target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX,
} }).outputText
const cloud = { positions: new Float32Array([1, 2, 3, 4, 5, 6]),
  colorRange: { min: 3, max: 6 }, bounds: { min: [1, 2, 3], max: [4, 5, 6] }, sourceCount: 2 }
const loaded = { status: 'ready', cloud, frameId: 'floor' }

interface Element {
  type: string
  key: string | null
  props: Record<string, unknown> & { children?: unknown }
}
function walk(value: unknown): Element[] {
  if (Array.isArray(value)) return value.flatMap(walk)
  if (!value || typeof value !== 'object' || !('props' in value)) return []
  const element = value as Element
  return [element, ...walk(element.props.children)]
}

function gallery() {
  const exports: Record<string, (props: object) => unknown> = {}
  const state: unknown[] = []
  const refs: Array<{ current: unknown }> = []
  let stateIndex = 0
  let refIndex = 0
  let full: typeof loaded | { status: string; message?: string } = loaded
  const cloudReads: unknown[][] = []
  const guards: unknown[] = []
  const opened: string[] = []
  let closed = 0
  let changed = 0
  const editing = { tool: 'crop' as 'crop' | 'clean' | 'point' | 'surface', onClose: () => { closed++; props.editing = null },
    onOpenMap: (name: string) => opened.push(name), onMapsChanged: () => changed++,
    onExitGuardChange: (guard: unknown) => guards.push(guard) }
  const props = { open: true, entries: [{ name: 'inactive-map', operations: [], map: {
    name: 'inactive-map', content_epoch: 42, has_pcd: true, has_octomap: true, can_activate: true, is_active: false } }],
    selectedMap: 'inactive-map', saves: { error: '' }, loading: false, error: '', readOnly: false,
    onSelect() {}, onRename() {}, onDelete() {}, onRefresh() {}, editing: null as typeof editing | null }
  runInNewContext(code, { exports, require: (name: string) => {
    if (name === 'react') return {
      useEffect() {}, useMemo: (read: () => unknown) => read(), useCallback: (callback: unknown) => callback,
      useState: (initial: unknown) => {
        const index = stateIndex++
        if (!(index in state)) state[index] = typeof initial === 'function' ? initial() : initial
        return [state[index], (value: unknown) => { state[index] = typeof value === 'function' ? value(state[index]) : value }]
      },
      useRef: (current: unknown) => { const index = refIndex++; return refs[index] ??= { current } },
    }
    if (name === 'react/jsx-runtime') return jsx
    if (name.includes('mapSavePresentation')) return presentation
    if (name.includes('mapThumbnail.ts')) return thumbnails
    if (name.includes('useMapSettings.ts')) return { useMapSettings: () => ({ agentPreview: null }) }
    if (name.includes('useMapThumbnails')) return { useMapThumbnails: () => ({ thumbnails: {}, load() {} }) }
    if (name.includes('useSavedCloud')) return { useSavedCloud: (...args: unknown[]) => {
      cloudReads.push(args)
      return args[2] ? full : { status: 'idle' }
    } }
    if (name.includes('useNoGo')) return { useNoGo: () => ({ map: { regions: [{ id: 'stairs' }] } }) }
    if (name.includes('useOctomap')) return { useOctomap: () => ({ status: 'ready', map: { cells: new Float32Array(),
      resolution: .1, bounds: cloud.bounds } }) }
    if (name.includes('MapGalleryScene')) return { MapGalleryScene: 'map-scene' }
    if (name.includes('MapEditor.tsx')) return { MapEditor: 'map-editor' }
    if (name.includes('MapInspectionPoints')) return { MapInspectionPoints: 'inspection-points' }
    if (name.includes('MapLibrary')) return { MapStatusIcon: () => null }
    if (name.includes('MapSaveHistory')) return { MapSaveHistory: () => null }
    if (name === 'lucide-react') return {}
    if (name.endsWith('.module.css')) return { __esModule: true, default: {} }
    throw new Error(`Unexpected module ${name}`)
  } })
  return { props, editing, cloudReads, guards, opened,
    setCloud(value: typeof full) { full = value },
    render() { stateIndex = 0; refIndex = 0; return walk(exports.MapGallery(props)) },
    counts: () => ({ closed, changed }) }
}

test('editing reuses the displayed full cloud, map epoch and frame in the same scene', () => {
  const h = gallery()
  const before = h.render().find(item => item.type === 'map-scene')!
  h.props.editing = h.editing
  const tree = h.render()
  const scene = tree.find(item => item.type === 'map-scene')!
  const editor = tree.find(item => item.type === 'map-editor')!
  assert.equal(scene.type, before.type)
  assert.equal(scene.key, before.key)
  assert.equal(scene.props.cloud, before.props.cloud)
  assert.equal(scene.props.cloud, cloud)
  assert.equal(editor.props.points, cloud.positions)
  assert.equal(editor.props.initialTool, 'crop')
  assert.deepEqual({ ...editor.props.source as object }, { name: 'inactive-map', contentEpoch: 42, frameId: 'floor' })
  assert.equal(h.cloudReads.length, 2)
  assert.deepEqual(h.cloudReads[0], h.cloudReads[1], 'entering editing keeps the same full-cloud cache key and enabled state')
})

test('inspection point editing uses the current map and pick callbacks without starting a crop draft', () => {
  const h = gallery()
  h.editing.tool = 'point'; h.props.editing = h.editing
  let tree = h.render()
  const scene = tree.find(item => item.type === 'map-scene')!
  assert.equal(scene.props.pointPicking, true)
  assert.equal(tree.some(item => item.type === 'map-editor'), false)
  ;(scene.props.onPointPick as (point: object) => void)({ x: 1, y: 2, z: .3 })
  tree = h.render()
  const panel = tree.find(item => item.type === 'inspection-points')!
  assert.equal(panel.props.mapName, 'inactive-map')
  assert.equal(panel.props.epoch, 42)
  assert.deepEqual(panel.props.picked, { x: 1, y: 2, z: .3 })
  ;(panel.props.onMarkers as (points: unknown[]) => void)([{ name: '柜子', x: 1, y: 2, z: 1.3 }])
  assert.deepEqual(h.render().find(item => item.type === 'map-scene')!.props.inspectionPoints,
    [{ name: '柜子', x: 1, y: 2, z: 1.3 }])
})

test('editor scene, selection, views, guards and lifecycle callbacks route through the gallery', () => {
  const h = gallery()
  h.props.editing = h.editing
  let tree = h.render()
  let scene = tree.find(item => item.type === 'map-scene')!
  let editor = tree.find(item => item.type === 'map-editor')!
  const originalSceneKey = scene.key
  const selected = { min: { x: 1, y: 2, z: 3 }, max: { x: 4, y: 5, z: 6 } }
  let receivedBounds: unknown
  let receivedView: unknown
  ;(editor.props.ref as { current: unknown }).current = { setBounds: (value: unknown) => { receivedBounds = value } }
  ;(scene.props.onReady as (value: unknown) => void)({ mapEditView: (value: unknown) => { receivedView = value } })
  ;(scene.props.onMapEditBounds as (value: unknown) => void)(selected)
  assert.equal(receivedBounds, selected)
  ;(editor.props.onView as (value: string) => void)('front')
  assert.equal(receivedView, 'front')
  const anchor = { x: 100, y: 120 }
  const edit: MapEditScene = { selecting: true, selection: selected, operations: [], view: 'draft', showDeleted: false }
  ;(scene.props.onMapEditAnchor as (value: unknown) => void)(anchor)
  ;(editor.props.onSceneChange as (value: unknown) => void)(edit)
  tree = h.render()
  scene = tree.find(item => item.type === 'map-scene')!
  editor = tree.find(item => item.type === 'map-editor')!
  assert.equal(scene.props.mapEdit, edit)
  assert.equal(editor.props.anchor, anchor)
  assert.equal(editor.props.onExitGuardChange, h.editing.onExitGuardChange)
  const guard = (proceed: () => void) => proceed()
  ;(editor.props.onExitGuardChange as (value: unknown) => void)(guard)
  assert.equal(h.guards[0], guard)
  ;(editor.props.onOpenMap as (name: string) => void)('copy')
  ;(editor.props.onMapsChanged as () => void)()
  assert.deepEqual(h.opened, ['copy'])
  assert.equal(h.counts().changed, 1)
  ;(editor.props.onClose as () => void)()
  tree = h.render()
  scene = tree.find(item => item.type === 'map-scene')!
  assert.equal(h.counts().closed, 1)
  assert.equal(scene.key, originalSceneKey)
  assert.equal(scene.props.cloud, cloud)
  assert.equal(scene.props.mapEdit, null)
  assert.ok(!tree.some(item => item.type === 'map-editor'))
})

test('only a ready full cloud can mount the editor and cleaning uses the same source', () => {
  const h = gallery()
  h.props.editing = h.editing
  for (const status of ['loading', 'error', 'idle']) {
    h.setCloud({ status, message: status === 'error' ? '地图内容已更新，请刷新地图库后重试' : undefined })
    const tree = h.render()
    assert.ok(!tree.some(item => item.type === 'map-editor'), status)
    const message = tree.find(item => item.props.role === (status === 'error' ? 'alert' : 'status'))!
    assert.ok(message, status)
    assert.ok(walk(message).some(item => item.props.children === '退出编辑'), status)
  }
  h.setCloud(loaded)
  h.editing.tool = 'clean'
  const editor = h.render().find(item => item.type === 'map-editor')!
  assert.equal(editor.props.initialTool, 'clean')
  assert.equal(editor.props.points, cloud.positions)
})

test('editing a voxel view switches the same scene to full source points', () => {
  const h = gallery()
  const tree = h.render()
  ;(tree.find(item => item.type === 'button' && item.props.children === '体素地图')!.props.onClick as () => void)()
  const voxel = h.render().find(item => item.type === 'map-scene')!
  assert.ok(voxel.props.octomap)
  h.props.editing = h.editing
  const point = h.render().find(item => item.type === 'map-scene')!
  assert.equal(point.key, voxel.key)
  assert.equal(point.props.cloud, cloud)
  assert.equal(point.props.octomap, undefined)
})

test('worker-decoded source frame stays with the cached cloud and is reused without fetching', () => {
  const hookSource = readFileSync(new URL('../src/hooks/useSavedCloud.ts', import.meta.url), 'utf8')
  const ast = ts.createSourceFile('useSavedCloud.ts', hookSource, ts.ScriptTarget.Latest, true)
  let workerHandler: ts.Expression | undefined
  const find = (node: ts.Node) => {
    if (ts.isBinaryExpression(node) && node.left.getText(ast) === 'worker.onmessage') workerHandler = node.right
    ts.forEachChild(node, find)
  }
  find(ast)
  assert.ok(workerHandler)
  let cache = new Map<string, unknown>()
  let delivered: unknown
  const handleDecoded = runInNewContext(ts.transpileModule(`(${workerHandler.getText(ast)})`, {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText, { abort: { signal: { aborted: false } }, preview: { frameId: 'floor' }, key: 'inactive-map:42', Map,
    setCache: (update: (current: typeof cache) => typeof cache) => { cache = update(cache) },
    update: (value: unknown) => { delivered = value }, worker: { terminate() {} } }) as (event: unknown) => void
  handleDecoded({ data: { cloud } })
  assert.equal((delivered as { frameId: string }).frameId, 'floor')
  const exports: Record<string, (name: string, epoch: number, enabled: boolean) => typeof loaded> = {}
  let index = 0
  let fetches = 0
  runInNewContext(ts.transpileModule(hookSource.replaceAll('import.meta.url', JSON.stringify('https://local.test/useSavedCloud.ts')), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS },
  }).outputText, { exports, require: (name: string) => name === 'react' ? {
    useState: () => [index++ === 0 ? cache : undefined, () => {}], useEffect: (effect: () => void) => effect(),
  } : { fetchMapPreview: () => { fetches++; throw new Error('Cached cloud must not be downloaded again') } } })
  const ready = exports.useSavedCloud('inactive-map', 42, true)
  assert.equal(ready.status, 'ready')
  assert.equal(ready.cloud, cloud)
  assert.equal(ready.frameId, 'floor')
  assert.equal(fetches, 0)
})

test('saved-map crop and cleaning actions respect management and observation mode', () => {
  const mapView = ts.transpileModule(readFileSync(new URL('../src/components/MapView.tsx', import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
  for (const [management, observe, expected] of [[true, false, 'enabled'], [false, false, 'disabled'], [true, true, 'hidden']] as const) {
    const exports: Record<string, unknown> = {}
    let state = 0
    runInNewContext(mapView, { exports, require: (name: string) => {
      if (name === 'react') return { ...React, useEffect: () => {}, useState: (initial: unknown) => {
        const index = state++
        return [index === 0 ? [{ name: 'inactive-map', has_pcd: true, has_octomap: true, content_epoch: 42,
          can_activate: true, is_active: false }] : index === 1 ? false : index === 6 ? management : initial, () => {}]
      } }
      if (name.endsWith('/observationMode.ts')) return { isObservationMode: () => observe }
      if (name.endsWith('/api')) return {}
      if (name.endsWith('/i18n')) return { text: (_locale: string, _en: string, zh: string) => zh }
      if (name.endsWith('/mapReadiness')) return { mapSaveBlockedReason: () => '', navigationGoalQueueReady: () => false,
        mapIsActivationReady: () => true, mapUseStatus: () => ({ label: '导航数据齐全', detail: '' }) }
      if (name.includes('/mapSavePresentation')) return { mappingCapacityNotice: () => null,
        mapLibraryEntries: () => [], mapLibraryStatus: () => ({ label: '导航数据齐全', detail: '' }) }
      if (name.includes('/localizationInitialPose')) return {}
      if (name.includes('/mapEdit')) return {}
      if (name.endsWith('/useMapSettings.ts')) return { useMapSettings: () => ({ open() {} }) }
      if (name.endsWith('/useMapSaves.ts')) return { useMapSaves: () => ({ history: { operations: [] }, error: '' }) }
      if (name.endsWith('.css')) return { default: {} }
      if (name.startsWith('./')) return new Proxy({}, { get: () => () => null })
      return require(name)
    } })
    const component = exports.MapView as React.ComponentType<Record<string, unknown>>
    const html = renderToStaticMarkup(React.createElement(component, { initialSelectedMap: 'inactive-map',
      productSwitchAllowed: false, productSwitchReason: '', productSwitchMessage: '',
      onUseMap() {}, onReturnLive() {}, session: null, slamDiag: null, showToast() {}, locale: 'zh' }))
    for (const label of ['裁剪点云', '清洗点云']) {
      const button = html.match(new RegExp(`<button[^>]*aria-label="${label}"[^>]*>.*?<\\/button>`))?.[0]
      if (expected === 'hidden') assert.equal(button, undefined, label)
      else {
        assert.ok(button, label)
        assert.equal(button.includes('disabled'), expected === 'disabled', label)
      }
    }
  }
})


test('the empty map library renders before a map or Agent preview exists', () => {
  const h = gallery()
  h.props.entries = []
  h.props.selectedMap = ''
  assert.doesNotThrow(() => h.render())
  assert.equal(h.render().some(element => element.type === 'map-scene'), false)
})
