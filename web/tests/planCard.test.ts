import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'
import type { ConversationPlan, ConversationSnapshot } from '../src/services/agentConversation.ts'

const require = createRequire(import.meta.url)
const { renderToStaticMarkup } = require('react-dom/server')
function compile(path: string) {
  return ts.transpileModule(readFileSync(new URL(path, import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
}
const source = compile('../src/components/agentConversation/PlanCard.tsx')
type Element = { type?: string; props: Record<string, unknown> }
function elements(node: unknown): Element[] {
  if (Array.isArray(node)) return node.flatMap(elements)
  if (!node || typeof node !== 'object') return []
  const element = node as Element
  return [element, ...elements(element.props?.children)]
}

const plan: ConversationPlan = { title: '检查点云间隙', summary: '先确认数据来源，再检查定位与碰撞信息。', steps: [
  { title: '核对地图版本', detail: '检查当前地图与定位使用的版本是否一致。' },
  { title: '查看导航证据' },
] }
const markdown = '# 检查点云间隙\n\n1. 核对地图版本\n2. 查看导航证据\n'

function harness(options: { copy?: (value: string) => Promise<void>; downloadFailure?: boolean } = {}) {
  const slots: Array<{ value?: unknown; deps?: unknown[] }> = []
  let cursor = 0
  let effects: Array<() => void> = []
  const copied: string[] = []
  const blobs: Blob[] = []
  const downloads: Array<{ href: string; filename: string }> = []
  const revoked: string[] = []
  const timers: Array<() => void> = []
  const selection = { focus: 0, select: 0 }
  const exports: Record<string, unknown> = {}
  const react = {
    useRef: () => ({ current: { focus: () => selection.focus++, select: () => selection.select++ } }),
    useState(initial: unknown) {
      const index = cursor++
      slots[index] ??= { value: initial }
      return [slots[index].value, (value: unknown) => { slots[index].value = value }]
    },
    useEffect(effect: () => void, deps: unknown[]) {
      const index = cursor++
      const previous = slots[index]
      if (!previous?.deps || deps.some((value, position) => !Object.is(value, previous.deps?.[position]))) {
        effects.push(() => { slots[index] = { deps }; effect() })
      }
    },
  }
  runInNewContext(source, { exports, Blob, navigator: { clipboard: { writeText: async (text: string) => {
    copied.push(text); await options.copy?.(text)
  } } }, URL: {
    createObjectURL(blob: Blob) { blobs.push(blob); return 'blob:plan-1' },
    revokeObjectURL: (url: string) => revoked.push(url),
  }, document: { createElement: () => {
    const anchor = { href: '', download: '', click() {
      if (options.downloadFailure) throw new Error('blocked')
      downloads.push({ href: anchor.href, filename: anchor.download })
    } }
    return anchor
  } }, setTimeout: (callback: () => void) => { timers.push(callback) }, require: (name: string) => {
    if (name === 'react') return react
    if (name.endsWith('.css')) return { default: new Proxy({}, { get: (_target, key) => key }) }
    return require(name)
  } })
  const props = { locale: 'zh', plan, text: markdown }
  const render = () => {
    cursor = 0; effects = []
    const result = (exports.PlanCard as (props: unknown) => Element)(props)
    effects.forEach(effect => effect())
    return result
  }
  return { props, copied, blobs, downloads, revoked, timers, selection, render,
    html: () => renderToStaticMarkup(render()) as string,
    button: (label: string) => elements(render()).find(element => element.type === 'button' && element.props['aria-label'] === label)!,
  }
}

test('a plan is a collapsed draft with numbered steps and no execution controls', () => {
  const h = harness()
  const tree = h.render()
  const nodes = elements(tree)
  const details = nodes.find(element => element.type === 'details')!
  assert.ok(details)
  assert.equal(details.props.open, undefined)
  assert.equal(elements(details)[1].type, 'summary')
  assert.equal(nodes.filter(element => element.type === 'li').length, 2)
  assert.equal(nodes.filter(element => element.type === 'button').length, 2)
  const markup = h.html()
  assert.match(markup, /方案草稿 · 未执行/)
  assert.match(markup, /2 个步骤/)
  assert.match(markup, /核对地图版本/)
  assert.doesNotMatch(markup, /checkbox|已完成|开始执行|批准|<input/)
  h.props.plan = { ...plan, title: '<img src=x onerror=alert(1)>', steps: [{ title: '<script>fake</script>', detail: '**未执行**' }] }
  assert.match(h.html(), /&lt;img src=x onerror=alert\(1\)&gt;/)
  assert.match(h.html(), /&lt;script&gt;fake&lt;\/script&gt;/)
  assert.doesNotMatch(h.html(), /<img|<script/)
})

test('copy success waits for the clipboard and keeps the exact supplied Markdown', async () => {
  let finish!: () => void
  const h = harness({ copy: () => new Promise(resolve => { finish = resolve }) })
  const copying = (h.button('复制方案 Markdown').props.onClick as () => void)()
  assert.equal(copying, undefined)
  assert.equal(h.button('复制方案 Markdown').props.disabled, true)
  assert.match(h.html(), /正在复制/)
  assert.doesNotMatch(h.html(), /已复制/)
  finish()
  for (let i = 0; i < 5; i++) await Promise.resolve()
  assert.deepEqual(h.copied, [markdown])
  assert.match(h.html(), /已复制/)
})

test('clipboard failure exposes and selects the original Markdown for manual copying', async () => {
  const h = harness({ copy: async () => { throw new Error('denied') } })
  ;(h.button('复制方案 Markdown').props.onClick as () => void)()
  for (let i = 0; i < 5; i++) await Promise.resolve()
  const original = elements(h.render()).find(element => element.type === 'textarea')!
  assert.equal(original.props.readOnly, true)
  assert.equal(original.props.value, markdown)
  assert.match(h.html(), /复制失败，请选中下方原文复制/)
  assert.ok(h.selection.focus >= 1 && h.selection.select >= 1)
  assert.doesNotMatch(h.html(), /已复制/)
})

test('download exports the exact UTF-8 Markdown and releases its object URL', async () => {
  const h = harness()
  ;(h.button('下载方案 Markdown').props.onClick as () => void)()
  assert.deepEqual(h.downloads, [{ href: 'blob:plan-1', filename: 'lingtu-plan.md' }])
  assert.equal(h.blobs[0].type, 'text/markdown;charset=utf-8')
  assert.equal(await h.blobs[0].text(), markdown)
  assert.deepEqual(h.revoked, [])
  h.timers.forEach(callback => callback())
  assert.deepEqual(h.revoked, ['blob:plan-1'])
  const failed = harness({ downloadFailure: true })
  ;(failed.button('下载方案 Markdown').props.onClick as () => void)()
  assert.match(failed.html(), /下载失败，可复制方案后保存为 Markdown/)
  assert.equal(failed.downloads.length, 0)
  failed.timers.forEach(callback => callback())
  assert.deepEqual(failed.revoked, ['blob:plan-1'])
})

test('HTTP and SSE reject malformed plan data while old messages remain compatible', async () => {
  const exports: Record<string, unknown> = {}
  const message = { id: 'plan', turn_id: 'turn', seq: 1, role: 'assistant', text: markdown, kind: 'plan', revision: 1, metadata: { plan } }
  let snapshot: { conversation_id: string; messages: unknown[]; turns: unknown[] } = { conversation_id: 'one', messages: [message], turns: [] }
  runInNewContext(compile('../src/services/agentConversation.ts'), { exports, Error, Map, AbortSignal,
    require: () => ({ dashboardFetch: async () => new Response(JSON.stringify(snapshot), { status: 200 }) }),
  })
  const read = exports.fetchConversation as (id: string) => Promise<ConversationSnapshot>
  const parse = exports.parseConversationMessageEvent as (value: unknown) => unknown
  const valid = exports.isConversationPlan as (value: unknown) => boolean
  assert.equal((await read('one')).messages[0].metadata?.plan?.steps.length, 2)
  const event = { ...message, type: 'agent_message', conversation_id: 'one' }
  assert.ok(parse(event))
  for (const malformed of [null, { title: 1, steps: plan.steps }, { title: 'bad', steps: [] },
    { ...plan, summary: null }, { ...plan, summary: ' ' }, { ...plan, title: 'a'.repeat(121) },
    { title: 'bad', steps: Array.from({ length: 13 }, () => ({ title: 'step' })) },
    { title: 'bad', steps: [{ title: 'step', detail: { status: 'completed' } }] }]) {
    assert.equal(valid(malformed), false)
    assert.equal(parse({ ...event, metadata: { plan: malformed } }), null)
    snapshot = { ...snapshot, messages: [{ ...message, metadata: { plan: malformed } }] }
    await assert.rejects(read('one'), /快照格式无效/)
  }
  snapshot = { ...snapshot, messages: [{ ...message, metadata: undefined }] }
  assert.equal((await read('one')).messages.length, 1)
})
