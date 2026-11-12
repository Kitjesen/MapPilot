import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import * as jsx from 'react/jsx-runtime'
import ts from 'typescript'
import * as presentation from '../src/services/mapSavePresentation.ts'
import * as thumbnails from '../src/services/mapThumbnail.ts'

test('hover, focus and touch select one matching terrain and information without opening the map', () => {
  const state: unknown[] = []
  let cursor = 0
  let effects: (() => void)[] = []
  let complete = false
  let cloudError = ''
  let voxelStatus: 'ready' | 'loading' | 'error' = 'ready'
  let refreshes = 0
  const selections: string[] = []
  const renamed: string[] = []
  const cloud = { positions: new Float32Array([0, 0, 0, 1, 1, 2]), colorRange: { min: 0, max: 2 }, bounds: { min: [0, 0, 0], max: [1, 1, 2] }, sourceCount: 2 }
  const exports: Record<string, (props: object) => unknown> = {}
  const source = readFileSync(new URL('../src/components/MapGallery.tsx', import.meta.url), 'utf8')
  const images = Object.fromEntries(['first', 'second'].map((name, i) => [`${name}:1`, {
    state: 'ready', image: `data:image/png,${name}`, data: { name, content_epoch: 1, total_points: i + 100,
      built_at_ms: null, cleanup_state: i ? 'cleaned' : 'unknown', points: [[i, 0, 0], [i, 1, 2]] },
  }]))
  runInNewContext(ts.transpileModule(`${source}\nexport { MapChoice }`, { compilerOptions: {
    module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022, jsx: ts.JsxEmit.ReactJSX,
  } }).outputText, { exports, require: (name: string) => {
    if (name === 'react') return {
      useEffect: (effect: () => void) => effects.push(effect), useRef: (current: unknown) => ({ current }), useMemo: (read: () => unknown) => read(),
      useCallback: (callback: unknown) => callback,
      useState: (initial: unknown) => {
        const i = cursor++
        if (!(i in state)) state[i] = typeof initial === 'function' ? initial() : initial
        return [state[i], (value: unknown) => { state[i] = typeof value === 'function' ? value(state[i]) : value }]
      },
    }
    if (name === 'react/jsx-runtime') return jsx
    if (name.includes('mapSavePresentation')) return presentation
    if (name.includes('mapThumbnail.ts')) return thumbnails
    if (name.includes('useMapSettings.ts')) return { useMapSettings: () => ({ agentPreview: null }) }
    if (name.includes('useMapThumbnails')) return { useMapThumbnails: () => ({ thumbnails: images, load() {} }) }
    if (name.includes('useSavedCloud')) return { useSavedCloud: (_name: unknown, _epoch: unknown, enabled: boolean) => !enabled ? { status: 'idle' }
      : cloudError ? { status: 'error', message: cloudError } : complete ? { status: 'ready', cloud, frameId: 'floor' } : { status: 'loading' } }
    if (name.includes('useNoGo')) return { useNoGo: () => ({ map: { regions: [] } }) }
    if (name.includes('useOctomap')) return { useOctomap: (_name: unknown, _epoch: unknown, _frame: unknown, enabled: boolean) => !enabled ? { status: 'idle' }
      : voxelStatus !== 'ready' ? { status: voxelStatus, message: voxelStatus === 'error' ? '体素解析失败' : undefined } : ({ status: 'ready', map: {
      cells: new Float32Array([0, 0, 0, .1]), resolution: .1, bounds: { min: [0, 0, 0], max: [2, 2, 2] },
    } }) }
    if (name.includes('MapGalleryScene')) return { MapGalleryScene: 'map-scene' }
    if (name.includes('MapEditor.tsx')) return { MapEditor: 'map-editor' }
    if (name.includes('MapInspectionPoints')) return { MapInspectionPoints: 'inspection-points' }
    if (name.includes('MapLibrary')) return { MapStatusIcon: () => null }
    if (name.includes('MapSaveHistory')) return { MapSaveHistory: () => null }
    if (name === 'lucide-react') return {}
    if (name.endsWith('.module.css')) return { __esModule: true, default: { spinner: 'spinner' } }
    return { default: {} }
  } })
  type Element = { type: string | ((props: object) => unknown); props: Record<string, unknown> & { children?: unknown } }
  const walk = (value: unknown): Element[] => {
    if (Array.isArray(value)) return value.flatMap(walk)
    if (!value || typeof value !== 'object' || !('props' in value)) return []
    const element = value as Element
    return [element, ...walk(element.props.children)]
  }
  const props = { open: true, entries: ['first', 'second'].map(name => ({ name, operations: [],
    map: { name, content_epoch: 1, has_pcd: true, has_octomap: true, can_activate: true, is_active: false } })),
    selectedMap: 'first', saves: { error: '' }, loading: false, error: '', readOnly: false,
    onSelect: (name: string) => { selections.push(name); props.selectedMap = name }, onRename: (name: string) => renamed.push(name), onDelete() {}, onRefresh() { refreshes++ } }
  const render = () => {
    cursor = 0
    effects = []
    const tree = walk(exports.MapGallery(props))
    effects.forEach(effect => effect())
    return tree
  }
  let tree = render()
  const menus = tree.filter(e => e.type === 'details' && 'data-pinned' in e.props)
  assert.equal(menus.length, 2)
  assert.ok(menus.every(menu => !menu.props.open), 'library and settings start as compact entries')
  assert.ok(menus.every(menu => !menu.props.name && menu.props['data-hover-menu'] === true), 'both sides reveal independently')
  assert.ok(tree.some(e => e.type === 'figure' && e.props['aria-label'] === 'first 地图'))
  const search = tree.find(e => e.props['aria-label'] === '搜索地图')!
  for (const query of ['second', 'missing']) {
    ;(search.props.onChange as (event: object) => void)({ target: { value: query } })
    tree = render()
    assert.equal(props.selectedMap, 'first', 'searching only filters the library')
    assert.equal(tree.find(e => e.type === 'map-scene')!.props.name, 'first')
    const choices = tree.filter(e => typeof e.type === 'function' && e.type.name === 'MapChoice')
    assert.deepEqual(choices.map(e => (e.props.entry as { name: string }).name), query === 'second' ? ['second'] : [])
  }
  assert.deepEqual(selections, [], 'searching must not trigger selection effects')
  props.selectedMap = 'removed'
  render()
  assert.equal(props.selectedMap, 'first', 'an unavailable selection falls back to the first complete entry')
  assert.deepEqual(selections, ['first'])
  ;(search.props.onChange as (event: object) => void)({ target: { value: '' } })
  tree = render()
  assert.ok(!tree.some(e => e.type === 'figcaption' || e.props['aria-label'] === '预览视角'))
  const rename = tree.find(e => e.props['aria-label'] === '重命名 second')!
  let focusRestored = false
  ;(rename.props.onClick as (event: object) => void)({ currentTarget: { closest: () => ({ querySelector: () => ({ focus() { focusRestored = true } }) }) } })
  assert.deepEqual(renamed, ['second'], 'renaming a row targets that row, independently of the displayed map')
  assert.equal(props.selectedMap, 'first')
  assert.equal(focusRestored, true)
  props.entries[1].map.is_active = true
  assert.equal(render().find(e => e.props['aria-label'] === '重命名 second')!.props.disabled, true)
  props.entries[1].map.is_active = false
  props.readOnly = true
  assert.ok(!render().some(e => String(e.props['aria-label']).startsWith('重命名 ')))
  props.readOnly = false
  const pin = tree.find(e => e.props['aria-label'] === '固定地图库到左侧')!
  ;(pin.props.onClick as () => void)()
  tree = render()
  const library = tree.find(e => e.type === 'details' && e.props['data-pinned'] === true)!
  assert.equal(library.props.name, undefined, 'pinned library is independent of transient menus')
  assert.equal(library.props['data-hover-menu'], undefined, 'pinned library does not auto-hide')
  const unpin = tree.find(e => e.props['aria-label'] === '取消固定地图库')!
  assert.equal(unpin.props['aria-pressed'], true)
  ;(unpin.props.onClick as () => void)()
  tree = render()
  assert.ok(tree.some(e => e.type === 'details' && e.props['data-pinned'] === false
    && e.props['aria-label'] === '左侧地图库' && e.props['data-hover-menu'] === true))
  const pinSettings = tree.find(e => e.props['aria-label'] === '固定地图设置到右侧')!
  ;(pinSettings.props.onClick as () => void)()
  tree = render()
  const settings = tree.find(e => e.type === 'details' && e.props['aria-label'] === '右侧地图设置')!
  assert.equal(settings.props['data-pinned'], true)
  assert.equal(settings.props['data-hover-menu'], undefined)
  assert.equal(tree.find(e => e.props['aria-label'] === '取消固定地图设置')!.props['aria-pressed'], true)
  assert.ok(tree.some(e => e.props['aria-label'] === '固定地图库到左侧'), 'settings pin does not pin the library')
  ;(tree.find(e => e.props['aria-label'] === '取消固定地图设置')!.props.onClick as () => void)()
  tree = render()
  assert.ok(tree.some(e => e.type === 'details' && e.props['aria-label'] === '右侧地图设置'
    && e.props['data-pinned'] === false && e.props['data-hover-menu'] === true))
  const choice = tree.find(e => typeof e.type === 'function' && e.type.name === 'MapChoice'
    && (e.props.entry as { name: string }).name === 'second')!
  const button = exports.MapChoice(choice.props) as Element
  for (const event of ['onPointerEnter', 'onFocus', 'onClick']) {
    props.selectedMap = 'first'
    ;(button.props[event] as () => void)()
    tree = render()
    assert.ok(tree.some(e => e.type === 'figure' && e.props['aria-label'] === 'second 地图'), event)
    const preview = tree.find(e => e.type === 'map-scene')!
    assert.equal(preview.props.name, 'second', event)
    assert.deepEqual(preview.props.points, [1, 0, 0, 1, 1, 2], event)
    assert.ok(walk(button).some(e => e.props.role === 'img' && e.props['aria-label'] === '已清洗'), event)
    assert.ok(!walk(button).some(e => e.props.children === '已清洗'), 'cleaning status is an icon, not repeated text')
    assert.equal(props.selectedMap, 'second')
  }
  const opacity = tree.find(e => e.type === 'input' && e.props['aria-label'] === '地图不透明度')!
  ;(opacity.props.onChange as (event: object) => void)({ target: { value: '40' } })
  tree = render()
  assert.equal(tree.find(e => e.type === 'map-scene')!.props.opacity, .4)
  const height = tree.find(e => e.type === 'input' && e.props['aria-label'] === '地图显示高度')!
  ;(height.props.onChange as (event: object) => void)({ target: { value: '1' } })
  assert.equal(render().find(e => e.type === 'map-scene')!.props.ceiling, 1)
  ;(height.props.onChange as (event: object) => void)({ target: { value: String(height.props.max) } })
  tree = render()
  assert.equal(tree.find(e => e.type === 'map-scene')!.props.ceiling, null, 'the slider maximum shows the whole map')
  const voxelButton = tree.find(e => e.type === 'button' && e.props.children === '体素地图')!
  ;(voxelButton.props.onClick as () => void)()
  tree = render()
  assert.equal(tree.find(e => e.type === 'map-scene')!.props.points, undefined)
  assert.ok(tree.find(e => e.type === 'map-scene')!.props.octomap)
  voxelStatus = 'loading'
  tree = render()
  const voxelLoading = tree.find(e => e.props.role === 'status')!
  assert.ok(walk(voxelLoading).some(e => e.props.children === '正在读取体素地图…'))
  assert.ok(walk(voxelLoading).some(e => e.props.className === 'spinner'))
  assert.ok(!tree.some(e => e.props.children === '正在加载完整点云…'))
  voxelStatus = 'error'
  tree = render()
  const voxelError = tree.find(e => e.props.role === 'alert')!
  assert.ok(walk(voxelError).some(e => e.props.children === '体素解析失败'))
  assert.ok(!walk(voxelError).some(e => e.props.className === 'spinner'))
  ;(walk(voxelError).find(e => e.props.children === '重新读取')!.props.onClick as () => void)()
  assert.equal(refreshes, 1, 'voxel failures retry through the existing refresh action')
  voxelStatus = 'ready'
  tree = render()
  const scale = tree.find(e => e.type === 'select' && e.props['aria-label'] === '体素显示大小')!
  ;(scale.props.onChange as (event: object) => void)({ target: { value: '4' } })
  tree = render()
  assert.equal(tree.find(e => e.type === 'map-scene')!.props.voxelScale, 4)
  props.entries[1].map.has_octomap = false
  tree = render()
  assert.equal(tree.find(e => e.type === 'button' && e.props.children === '体素地图')!.props.disabled, true)
  assert.equal(tree.find(e => e.type === 'map-scene')!.props.octomap, undefined)
  cloudError = '点云解析失败'
  tree = render()
  const pointError = tree.find(e => e.props.role === 'alert')!
  assert.ok(walk(pointError).some(e => e.props.children === '点云解析失败'))
  ;(walk(pointError).find(e => e.props.children === '重新读取')!.props.onClick as () => void)()
  assert.equal(refreshes, 2, 'point-cloud failures retain the same retry action')
  cloudError = ''
  complete = true
  tree = render()
  assert.equal(tree.find(e => e.type === 'map-scene')!.props.cloud, cloud)
  assert.equal(tree.find(e => e.type === 'map-scene')!.props.points, undefined)
  assert.equal(tree.find(e => e.type === 'map-scene')!.props.grid, true)
  assert.ok(!tree.some(e => e.props.children === '打开完整地图'), 'the current workspace displays the full cloud')
  assert.ok(walk(button).some(e => e.type === 'time'), 'update time is visible on each row')
  props.entries[1].map.has_pcd = false
  assert.ok(render().some(e => e.props.children === '点云尚未保存'), 'an unpublished source is not reported as loading')
  props.entries = []
  props.error = '无法获取地图列表：服务未连接'
  tree = render()
  assert.ok(tree.some(e => e.props.role === 'alert'), 'a closed library menu must not hide a load failure')
  assert.ok(!tree.some(e => e.props.children === '还没有保存的地图'))
})
