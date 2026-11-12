import assert from 'node:assert/strict'
import test from 'node:test'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import * as jsx from 'react/jsx-runtime'
import ts from 'typescript'
import * as presentation from '../src/services/mapSavePresentation.ts'
import type { MapInfo } from '../src/types/index.ts'

const ready: MapInfo = { name: 'garage', content_epoch: 1, has_pcd: true, has_octomap: true, can_activate: true, is_active: false }

type Element = { type: string; props: Record<string, unknown> & { children?: unknown } }
const walk = (value: unknown): Element[] => {
  if (Array.isArray(value)) return value.flatMap(walk)
  if (!value || typeof value !== 'object' || !('props' in value)) return []
  const element = value as Element
  return [element, ...walk(element.props.children)]
}

function mapView(initialSelectedMap: string | null, settingsAvailable = true) {
  const source = readFileSync(new URL('../src/components/MapView.tsx', import.meta.url), 'utf8')
  const code = ts.transpileModule(source, { compilerOptions: {
    target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX,
  } }).outputText
  const state: unknown[] = []
  let cursor = 0
  const settingsOpened: string[] = []
  const exports: Record<string, (props: object) => unknown> = {}
  runInNewContext(code, { exports, require: (name: string) => {
    if (name === 'react') return {
      useEffect() {}, useRef: (current: unknown) => ({ current }), useCallback: (fn: unknown) => fn,
      useState: (initial: unknown) => {
        const slot = cursor++
        if (!(slot in state)) state[slot] = slot === 0 ? [ready] : slot === 6 ? true : initial
        return [state[slot], (value: unknown) => { state[slot] = typeof value === 'function' ? value(state[slot]) : value }]
      },
    }
    if (name === 'react/jsx-runtime') return jsx
    if (name.includes('mapSavePresentation')) return presentation
    if (name.includes('useMapSettings')) return { useMapSettings: () => settingsAvailable
      ? { open: (focus: string) => settingsOpened.push(focus) } : null }
    if (name.includes('useMapSaves')) return { useMapSaves: () => ({ history: { operations: [] }, error: '' }) }
    if (name.includes('observationMode')) return { isObservationMode: () => false }
    if (name.includes('mapReadiness')) return { mapSaveBlockedReason: () => '', mapUseStatus: () => ({}), navigationGoalQueueReady: () => false, mapIsActivationReady: () => false }
    if (name.includes('MapGallery')) return { MapGallery: 'map-gallery' }
    return { default: {} }
  } })
  const render = () => { cursor = 0; return walk(exports.MapView({ initialSelectedMap, session: null })) }
  return { render, state, settingsOpened }
}

test('all map entry points select maps inside the integrated viewer', () => {
  for (const initialSelectedMap of [null, 'garage', 'missing']) {
    const { render, state } = mapView(initialSelectedMap)
    let tree = render()
    let gallery = tree.find(e => e.type === 'map-gallery')!
    assert.equal(gallery.props.open, true)
    assert.equal(gallery.props.onOpen, undefined, 'no separate full-map page remains')
    ;(gallery.props.onSelect as (name: string) => void)('garage')
    tree = render()
    gallery = tree.find(e => e.type === 'map-gallery')!
    assert.equal(gallery.props.selectedMap, 'garage')
    assert.equal(gallery.props.open, true)
    state[0] = []
    assert.equal(render().find(e => e.type === 'map-gallery')!.props.open, true)
  }
})

test('map tools expose direct actions with surface editing unavailable', () => {
  const { render } = mapView('garage')
  const tools = render().find(e => e.props.role === 'group' && e.props['aria-label'] === '地图工具')!
  const actions = walk(tools)
  assert.deepEqual(actions.filter(e => e.type === 'button').map(e => e.props['aria-label']),
    ['裁剪点云', '清洗点云', '设置禁行区', '人工补面（待接入）'])
  assert.equal(actions.some(e => e.type === 'details'), false)
  assert.equal(actions.find(e => e.props['aria-label'] === '人工补面（待接入）')!.props.disabled, true)
})

test('no-go action opens its settings directly', () => {
  const { render, settingsOpened } = mapView('garage')
  const noGo = render().find(e => e.props['aria-label'] === '设置禁行区')!
  ;(noGo.props.onClick as () => void)()
  assert.deepEqual(settingsOpened, ['no-go'])
})

test('cleaning stays unavailable without the global settings provider', () => {
  const { render } = mapView('garage', false)
  const tree = render()
  assert.equal(tree.find(e => e.props['aria-label'] === '清洗点云')!.props.disabled, true)
  assert.equal(tree.find(e => e.props['aria-label'] === '裁剪点云')!.props.disabled, false)
})

for (const [tool, label] of [['crop', '裁剪点云'], ['clean', '清洗点云']] as const) {
  test(`${tool} action keeps the selected gallery open and closes its editing tool`, () => {
    const { render } = mapView('garage')
    const edit = render().find(e => e.props['aria-label'] === label)!
    assert.equal(edit.props.disabled, false)
    ;(edit.props.onClick as () => void)()
    const tree = render()
    const gallery = tree.find(e => e.type === 'map-gallery')!
    assert.equal(gallery.props.open, true)
    assert.equal(gallery.props.selectedMap, 'garage')
    const editing = gallery.props.editing as { tool: string; onClose(): void }
    assert.equal(editing.tool, tool)
    editing.onClose()
    const closed = render().find(e => e.type === 'map-gallery')!
    assert.equal(closed.props.open, true)
    assert.equal(closed.props.editing, null)
  })
}

test('one row per map includes unfinished saves and orders each history newest first', () => {
  const old = { map_id: 'garage', operation_id: 'old', state: 'SUCCEEDED', created_at_ns: 10 }
  const recent = { map_id: 'garage', operation_id: 'new', state: 'RUNNING', created_at_ns: 20 }
  const pending = { name: 'new-map', operation_id: 'pending', state: 'CAPTURED', source_saved: true }
  const entries = presentation.mapLibraryEntries([ready], [old, pending, recent])
  assert.deepEqual(entries.map(entry => entry.name), ['garage', 'new-map'])
  assert.deepEqual(entries[0].operations.map(operation => operation.operation_id), ['new', 'old'])
  assert.equal(entries[1].map, undefined)
  assert.equal(entries[1].operations[0], pending)
})

test('current processing and failures take precedence over a previously ready map', () => {
  assert.equal(presentation.mapLibraryStatus(ready).kind, 'ready')
  const running = presentation.mapLibraryStatus(ready, { state: 'RUNNING', phase: 'BUILD_ARTIFACTS', source_saved: true })
  assert.equal(running.kind, 'busy')
  assert.equal(running.label, '生成导航地图')
  const failed = presentation.mapLibraryStatus(ready, { state: 'FAILED', source_saved: true })
  assert.equal(failed.kind, 'attention')
  assert.match(failed.detail, /已有导航地图仍保留/)
  assert.equal(presentation.mapLibraryStatus(ready, { state: 'RUNNING', cancel_requested: true }).kind, 'busy')
  assert.equal(presentation.mapLibraryStatus(ready, { state: 'RUNNING', cancel_requested: true }).label, '取消待确认')
})

test('a saved source or old successful job cannot advertise a missing map as ready', () => {
  for (const state of ['CAPTURED', 'FAILED', 'CANCELLED', 'CANCELED', 'SUCCEEDED']) {
    assert.equal(presentation.mapLibraryStatus(undefined, { state, source_saved: true }).kind, 'attention', state)
  }
  assert.equal(presentation.mapLibraryStatus({ ...ready, can_activate: false }, { state: 'SUCCEEDED' }).kind, 'attention')
  assert.equal(presentation.mapLibraryStatus(ready, undefined, true).kind, 'attention')
})

test('map rows keep text details closed, separate selection from status, and open an unpublished save', () => {
  let opened = false
  let preview = ''
  const exports: Record<string, (props: Record<string, unknown>) => unknown> = {}
  const source = readFileSync(new URL('../src/components/MapLibrary.tsx', import.meta.url), 'utf8')
  runInNewContext(ts.transpileModule(`${source}\nexport { MapRow }`, { compilerOptions: {
    module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022, jsx: ts.JsxEmit.ReactJSX,
  } }).outputText, { exports, require: (name: string) => {
    if (name === 'react') return {
      useId: () => 'map-details', useState: () => [opened, (value: boolean | ((old: boolean) => boolean)) => { opened = typeof value === 'function' ? value(opened) : value }],
    }
    if (name === 'react/jsx-runtime') return jsx
    if (name.includes('mapSavePresentation')) return presentation
    if (name.includes('MapSaveHistory')) return { MapSaveHistory: () => null }
    if (name.includes('lucide-react')) return {}
    return { default: {} }
  } })
  type Element = { type: string; props: Record<string, unknown> & { children?: unknown; onClick?: () => void } }
  const elements = (value: unknown): Element[] => {
    if (Array.isArray(value)) return value.flatMap(elements)
    if (!value || typeof value !== 'object' || !('props' in value)) return []
    const node = value as Element
    return [node, ...elements(node.props.children)]
  }
  const entry: presentation.MapLibraryEntry = { name: 'garage', map: ready, operations: [] }
  const render = () => elements(exports.MapRow({ entry, saves: { error: '' }, selected: true, readOnly: false,
    onPreview: (name: string) => { preview = name }, onRename() {}, onDelete() {} }))
  const initial = render()
  assert.equal(initial.find(element => element.props.id === 'map-details')?.props.hidden, true)
  const status = initial.find(element => element.props['data-status'] === 'ready')!
  assert.equal(status.props['aria-pressed'], undefined)
  assert.match(String(status.props['aria-label']), /地图就绪/)
  status.props.onClick!()
  assert.equal(render().find(element => element.props.id === 'map-details')?.props.hidden, false)
  render().find(element => element.props['aria-pressed'] === true)!.props.onClick!()
  assert.equal(preview, 'garage')
  entry.map = { ...ready, can_activate: false, activation_blockers: ['metadata missing', 'no saved rays'] }
  opened = false
  const blocked = render()
  assert.ok(blocked.some(element => element.props['data-status'] === 'attention'))
  const details = blocked.find(element => element.props.id === 'map-details')!
  assert.equal(details.props.hidden, true)
  const reasons = elements(details).filter(element => element.type === 'p').map(element => element.props.children)
  assert.ok(reasons.includes('metadata missing'))
  assert.ok(reasons.includes('no saved rays'))
  opened = false
  preview = ''
  entry.map = undefined
  entry.operations = [{ state: 'CAPTURED', source_saved: true }]
  render().find(element => element.props['aria-pressed'] === true)!.props.onClick!()
  assert.equal(opened, true)
  assert.equal(preview, '', 'an unpublished save must open its details instead of an unavailable point cloud')
})
