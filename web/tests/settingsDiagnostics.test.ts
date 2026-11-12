import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import { createRequire } from 'node:module'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import ts from 'typescript'

const require = createRequire(import.meta.url)
interface View {
  type: unknown
  props: {
    [key: string]: unknown
    children?: unknown
    className: string
    onClick: () => void
  }
}

function descendants(view: unknown): View[] {
  if (Array.isArray(view)) return view.flatMap(descendants)
  if (!view || typeof view !== 'object' || !('props' in view)) return []
  const node = view as View
  return [node, ...descendants(node.props.children)]
}

function label(view: unknown): string {
  if (Array.isArray(view)) return view.map(label).join('')
  if (typeof view === 'string') return view
  return view && typeof view === 'object' && 'props' in view ? label((view as View).props.children) : ''
}

function settings() {
  const states: unknown[] = []
  const refs: Array<{ current: unknown }> = []
  const effects: Array<() => void | (() => void)> = []
  const handlers = new Map<string, (event: Record<string, unknown>) => void>()
  let stateIndex = 0, refIndex = 0, lazyIndex = 0, restored = 0
  const document = { body: {}, activeElement: { focus() { restored++ } },
    addEventListener: (name: string, handler: (event: Record<string, unknown>) => void) => handlers.set(name, handler),
    removeEventListener: (name: string) => handlers.delete(name) }
  const source = readFileSync(new URL('../src/components/SettingsMenu.tsx', import.meta.url), 'utf8')
  const exports: { SettingsMenu?: (props: Record<string, unknown>) => View | null } = {}
  runInNewContext(ts.transpileModule(source, { compilerOptions: {
    target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX,
  } }).outputText, { exports, document,
    require: (name: string) => {
      if (name === 'react') return {
        lazy: () => ['SlamStatusPanel', 'RuntimeDataflowView', 'PlannerTuning', 'LocalizationCard'][lazyIndex++],
        Suspense: 'Suspense', useId: () => 'settings',
        useEffect: (effect: () => void) => effects.push(effect),
        useRef: (current: unknown) => refs[refIndex++] ?? (refs[refIndex - 1] = { current }),
        useState: (initial: unknown) => {
          const index = stateIndex++
          if (!(index in states)) states[index] = initial
          return [states[index], (next: unknown) => { states[index] = next }]
        },
      }
      if (name === 'react-dom') return { createPortal: (view: unknown) => view }
      if (name === '../hooks/useMapSettings.ts') return { useMapSettings: () => null }
      if (name === '../i18n') return { text: (_locale: string, _en: string, zh: string) => zh }
      if (name === './SurfaceSettings.tsx' || name === './NoGoSettings.tsx' || name === '../services/api' || name === './floatingWidgetLayout') return {}
      if (name.endsWith('.css')) return { expandedPanel: 'expanded' }
      return require(name)
    },
  })
  const sseState = { session: null, slamDiag: null }
  const showToast = () => {}
  let closes = 0
  return { refs, effects, handlers, document, sseState, showToast, restored: () => restored, closes: () => closes,
    render(open = true) {
      stateIndex = 0; refIndex = 0; effects.length = 0
      return exports.SettingsMenu!({ open, onClose: () => { closes++ }, theme: 'light', resolvedTheme: 'light',
        onThemeChange() {}, locale: 'zh', onLocaleChange() {}, sseState, localNowS: 123, showToast })
    },
  }
}

function click(view: View | null, name: string) {
  const button = descendants(view).find(node => node.type === 'button' && label(node) === name)
  assert.ok(button, `missing button ${name}`)
  button.props.onClick()
}

test('diagnostics opens all three existing views inside settings and returns to its list', () => {
  const ui = settings()
  click(ui.render(), '诊断')
  const list = ui.render()
  assert.deepEqual(descendants(list).filter(node => node.type === 'button').map(label).slice(-3), ['定位诊断查看 SLAM 状态与定位质量。', '数据诊断检查运行时数据链路与话题采样。', '规划参数检查参数并验证地图规划。'])
  for (const [title, component] of [['定位诊断', 'SlamStatusPanel'], ['数据诊断', 'RuntimeDataflowView'], ['规划参数', 'PlannerTuning']]) {
    const button = descendants(ui.render()).find(node => node.type === 'button' && label(node).startsWith(title))!
    button.props.onClick()
    const nodes = descendants(ui.render())
    const embedded = nodes.find(node => node.type === component)
    assert.ok(embedded)
    assert.ok(nodes.some(node => node.props.role === 'dialog' && node.props.className.includes('expanded')))
    assert.equal(nodes.filter(node => ['SlamStatusPanel', 'RuntimeDataflowView', 'PlannerTuning'].includes(String(node.type))).length, 1)
    if (component !== 'PlannerTuning') assert.equal(embedded.props.sseState, ui.sseState)
    if (component !== 'RuntimeDataflowView') assert.equal(embedded.props.showToast, ui.showToast)
    if (component === 'SlamStatusPanel') {
      assert.equal(embedded.props.locale, 'zh')
      const details = nodes.find(node => node.type === 'details')!
      assert.equal(details.props.open, undefined)
      assert.ok(descendants(details).some(node => node.type === 'summary' && label(node) === '卫星与融合'))
      const card = descendants(details).find(node => node.type === 'LocalizationCard')!
      assert.equal(card.props.sseState, ui.sseState)
      assert.equal(card.props.localNowS, 123)
      assert.equal(card.props.locale, 'zh')
    }
    click(ui.render(), '返回诊断列表')
    assert.equal(descendants(ui.render()).some(node => node.type === component), false)
  }
  assert.equal(ui.closes(), 0)
  assert.equal(ui.render(false), null)
})

test('diagnostics keeps keyboard focus trapped, Escape dismissal and opener restoration', () => {
  const ui = settings()
  click(ui.render(), '诊断')
  ui.render()
  let focused = 0, wrapped = 0, prevented = 0
  const first = { focus() { focused++ }, getClientRects: () => [1], matches: () => false }
  const last = { focus() { wrapped++ }, getClientRects: () => [1], matches: () => false }
  ui.refs[0].current = { querySelector: () => first, querySelectorAll: () => [first, last] }
  const cleanup = ui.effects[0]()
  ui.effects[1]()
  assert.equal(focused, 2)
  ui.document.activeElement = first
  ui.handlers.get('keydown')!({ key: 'Tab', shiftKey: true, preventDefault() { prevented++ } })
  assert.equal(wrapped, 1)
  ui.document.activeElement = last
  ui.handlers.get('keydown')!({ key: 'Tab', shiftKey: false, preventDefault() { prevented++ } })
  assert.equal(focused, 3)
  ui.handlers.get('keydown')!({ key: 'Escape', preventDefault() { prevented++ } })
  assert.equal(ui.closes(), 1)
  assert.equal(prevented, 3)
  if (cleanup) cleanup()
  assert.equal(ui.handlers.has('keydown'), false)
  assert.equal(ui.restored(), 1)
})
