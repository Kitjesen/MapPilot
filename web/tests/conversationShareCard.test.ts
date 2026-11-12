import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'
import type { ConversationSnapshot, SharedConversationSnapshot } from '../src/services/agentConversation.ts'

const require = createRequire(import.meta.url)
const React = require('react')
const { renderToStaticMarkup } = require('react-dom/server')
function compile(path: string) {
  return ts.transpileModule(readFileSync(new URL(path, import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
}
const source = compile('../src/components/ConversationShareCard.tsx')
const success: Record<string, unknown> = {}
runInNewContext(compile('../src/components/SuccessCheck.tsx'), { exports: success, require: (name: string) =>
  name.endsWith('.css') ? { default: new Proxy({}, { get: (_target, key) => key }) } : require(name),
})
const markdown: Record<string, unknown> = {}
runInNewContext(compile('../src/guide/markdown.ts'), { exports: markdown })

type Element = { type?: string | ((props: Record<string, unknown>) => unknown); props: Record<string, unknown> }
function elements(node: unknown): Element[] {
  if (Array.isArray(node)) return node.flatMap(elements)
  if (!node || typeof node !== 'object') return []
  const element = node as Element
  return [element, ...elements(element.props?.children)]
}

function harness(options: { copy?: (url: string) => Promise<void>; fetch?: (token: string) => Promise<SharedConversationSnapshot>; reducedMotion?: boolean } = {}) {
  const slots: Array<{ value?: unknown; deps?: unknown[]; cleanup?: () => void }> = []
  let cursor = 0
  let effects: Array<() => void> = []
  const selection = { focus: 0, select: 0 }
  const input = { current: { focus: () => selection.focus++, select: () => selection.select++ } }
  const exports: Record<string, unknown> = {}
  const react = {
    useRef: () => input,
    useState(initial: unknown) {
      const index = cursor++
      slots[index] ??= { value: initial }
      return [slots[index].value, (next: unknown) => { slots[index].value = typeof next === 'function' ? next(slots[index].value) : next }]
    },
    useEffect(effect: () => (() => void) | undefined, deps: unknown[]) {
      const index = cursor++
      const previous = slots[index]
      if (!previous?.deps || deps.some((value, i) => !Object.is(value, previous.deps?.[i]))) {
        effects.push(() => { previous?.cleanup?.(); slots[index] = { deps, cleanup: effect() } })
      }
    },
  }
  runInNewContext(source, { exports, Error, window: { matchMedia: () => ({ matches: options.reducedMotion ?? false }) },
    navigator: { clipboard: { writeText: options.copy ?? (async () => {}) } }, require: (name: string) => {
    if (name === 'react') return react
    if (name.endsWith('/guide/markdown')) return markdown
    if (name.endsWith('/SuccessCheck')) return success
    if (name.endsWith('/services/agentConversation')) return { fetchSharedConversation: options.fetch ?? (async () => { throw new Error('Unexpected request') }) }
    if (name.endsWith('/Modal')) return { ModalShell: (props: Record<string, unknown>) => React.createElement('dialog', {
      open: true, 'aria-labelledby': 'share-title', onPointerMove: props.onPointerMove, onPointerLeave: props.onPointerLeave },
      React.createElement('header', null, React.createElement('h2', { id: 'share-title' }, props.title), props.headerAction), props.children, React.createElement('footer', null, props.footer)) }
    if (name.endsWith('.css')) return { default: new Proxy({}, { get: (_target, key) => key }) }
    return require(name)
  } })
  const snapshot: ConversationSnapshot = { conversation_id: 'one', turns: [{ turn_id: 'done', state: 'completed' }], messages: [
    { id: 'u', turn_id: 'done', role: 'user', seq: 1, text: '用户原文 <img src=x>' },
    { id: 'a', turn_id: 'done', role: 'assistant', seq: 2, text: '**检查完成**' },
  ] }
  const props = { locale: 'zh', title: '点云间隙会影响导航吗？', snapshot, url: '', busy: false, local: true,
    onCreate: async () => {}, onRevoke: async () => {}, onClose: () => {},
  }
  const render = () => {
    cursor = 0; effects = []
    const view = (exports.ConversationShareCard as (props: unknown) => Element)(props)
    const tree = (view.type as (props: Record<string, unknown>) => unknown)(view.props)
    effects.forEach(effect => effect())
    return tree
  }
  return { props, selection, render, html: () => renderToStaticMarkup(render()) as string,
    button: (label: string) => elements(render()).find(element => element.type === 'button' && renderToStaticMarkup(element).includes(label))!,
    flush: async () => { for (let i = 0; i < 8; i++) await Promise.resolve() },
  }
}

test('foil follows the mouse within the dialog and respects touch and reduced motion', () => {
  const h = harness()
  const dialog = h.render() as Element
  const styles = new Map<string, string>()
  const attributes = new Map<string, string>()
  const card = {
    getBoundingClientRect: () => ({ left: 100, top: 50, width: 520, height: 400 }),
    style: {
      setProperty: (name: string, value: string) => styles.set(name, value),
      removeProperty: (name: string) => styles.delete(name),
    },
    setAttribute: (name: string, value: string) => attributes.set(name, value),
    removeAttribute: (name: string) => attributes.delete(name),
  }
  const event = { pointerType: 'mouse', clientX: 490, clientY: 150, currentTarget: card }
  const move = dialog.props.onPointerMove as (event: unknown) => void
  move(event)
  assert.equal(styles.get('--foil-x'), '75%')
  assert.equal(styles.get('--foil-y'), '25%')
  assert.equal(attributes.has('data-foil-active'), true)
  ;(dialog.props.onPointerLeave as (event: unknown) => void)(event)
  assert.equal(attributes.has('data-foil-active'), false)
  assert.equal(styles.size, 0)
  move(event)
  move({ ...event, clientX: 10 })
  assert.equal(attributes.has('data-foil-active'), false)
  assert.equal(styles.size, 0)
  move(event)
  move({ ...event, pointerType: 'touch' })
  assert.equal(styles.size, 0)
  move(event)
  const reduced = harness({ reducedMotion: true }).render() as Element
  ;(reduced.props.onPointerMove as (event: unknown) => void)(event)
  assert.equal(styles.size, 0)
  assert.equal(attributes.has('data-foil-active'), false)
})

test('share dialog previews safe terminal public messages and keeps creation explicit', async () => {
  const h = harness()
  h.props.snapshot.turns.push({ turn_id: 'active', state: 'running' })
  h.props.snapshot.turns.push({ turn_id: 'queued', state: 'queued' })
  h.props.snapshot.messages.push(
    { id: 'tool', turn_id: 'done', role: 'tool', seq: 3, text: 'PRIVATE_TOOL' },
    { id: 'pending', turn_id: 'active', role: 'user', seq: 4, text: 'UNFINISHED' },
    { id: 'queued-input', turn_id: 'queued', role: 'user', seq: 5, text: 'QUEUED_INPUT' },
  )
  let created = 0
  h.props.onCreate = async () => { created++ }
  const markup = h.html()
  assert.match(markup, /<dialog[^>]+aria-labelledby="share-title"/)
  assert.match(markup, /<h2 id="share-title">点云间隙会影响导航吗？<\/h2>/)
  assert.match(markup, /aria-label="分享内容预览: 点云间隙会影响导航吗/)
  assert.match(markup, /分享后新增的消息不会加入这份快照/)
  assert.doesNotMatch(markup, /shareWordmark/)
  assert.match(markup, /用户原文 &lt;img src=x&gt;/)
  assert.match(markup, /<strong>检查完成<\/strong>/)
  assert.doesNotMatch(markup, /PRIVATE_TOOL|UNFINISHED|QUEUED_INPUT|<img|保存一份|接收者可|文字对话|交互演示/)
  assert.match(markup, /本机只读.*仅此设备可打开/)
  assert.equal(created, 0)
  const generate = h.button('生成链接')
  assert.equal(generate.props['aria-label'], '生成链接')
  assert.equal(generate.props.title, '生成链接')
  assert.match(markup, /data-share-glyph/)
  await (generate.props.onClick as () => Promise<void>)()
  assert.equal(created, 1)
  h.props.busy = true
  assert.equal(h.button('正在生成').props.disabled, true)
  h.props.local = false
  assert.match(h.html(), /持链接且能访问服务者可查看/)
  h.props.locale = 'en'
  assert.match(h.html(), /Generating/)
})

test('first generate action copies the newly returned link and shows success', async () => {
  const copied: string[] = []
  const h = harness({ copy: async url => { copied.push(url) } })
  const createdUrl = 'http://localhost/?conversation-share=created'
  h.props.onCreate = async () => createdUrl
  await (h.button('生成链接').props.onClick as () => Promise<void>)()
  assert.deepEqual(copied, [createdUrl])
  assert.match(h.html(), /data-copy-success="1"/)
})

test('a generated link opens the complete read-only conversation in a separate tab', () => {
  const h = harness()
  assert.doesNotMatch(h.html(), /查看会话/)
  h.props.url = 'http://localhost/?conversation-share=frozen'
  const link = elements(h.render()).find(element => element.type === 'a' && element.props.href === h.props.url)!
  assert.ok(link)
  assert.equal(link.props.target, '_blank')
  assert.equal(link.props.rel, 'noopener noreferrer')
  assert.match(renderToStaticMarkup(link), /查看会话/)
  h.props.url = ''
  assert.doesNotMatch(h.html(), /查看会话/)
})

test('a paginated conversation makes the full-share scope clear before generation', () => {
  const h = harness()
  h.props.snapshot.history_cursor = 25
  assert.match(h.html(), /此处预览最近的消息，链接将包含全部已结束的文字对话/)
  h.props.snapshot.history_cursor = undefined
  assert.doesNotMatch(h.html(), /此处预览最近的消息/)
})

test('share title removes encoded spaces and markdown heading remnants', () => {
  const h = harness()
  h.props.title = '&#x20; ## 一起打磨 LingTu 分享体验 &#x20;'
  const markup = h.html()
  assert.match(markup, /<h2 id="share-title">一起打磨 LingTu 分享体验<\/h2>/)
  assert.doesNotMatch(markup, /&#x20;|##/)
})

test('an existing link previews its frozen payload rather than the latest conversation', async () => {
  const requests: string[] = []
  const h = harness({ fetch: async token => { requests.push(token); return {
    title: 'Frozen title', created_at: 1, messages: [{ role: 'assistant', seq: 1, text: 'Frozen answer' }],
  } } })
  h.props.url = 'http://localhost/?conversation-share=frozen'
  h.props.snapshot.share = { token: 'frozen', created_at: 1, message_count: 1 }
  assert.match(h.html(), /正在读取分享内容/)
  await h.flush()
  const markup = h.html()
  assert.match(markup, /Frozen title/)
  assert.match(markup, /Frozen answer/)
  assert.doesNotMatch(markup, /用户原文|检查完成/)
  assert.deepEqual(requests, ['frozen'])
})

test('copy writes the link and clipboard failure exposes and selects it for manual copying', async () => {
  const copied: string[] = []
  const h = harness({ copy: async url => { copied.push(url) } })
  h.props.url = 'http://localhost/?conversation-share=real-token'
  await (h.button('复制链接').props.onClick as () => Promise<void>)()
  assert.deepEqual(copied, [h.props.url])
  assert.match(h.html(), /已复制/)
  assert.equal(h.button('已复制').props['aria-label'], '已复制')
  assert.match(h.html(), /data-copy-success="1"/)
  assert.equal([...h.html().matchAll(/data-icon="pixel:check"/g)].length, 2)
  await (h.button('已复制').props.onClick as () => Promise<void>)()
  assert.match(h.html(), /data-copy-success="2"/)
  assert.deepEqual(copied, [h.props.url, h.props.url])
  const blocked = harness({ copy: async () => { throw new Error('clipboard unavailable') } })
  blocked.props.url = h.props.url
  await (blocked.button('复制链接').props.onClick as () => Promise<void>)()
  assert.match(blocked.html(), /请复制已选中的链接/)
  assert.doesNotMatch(blocked.html(), /data-copy-success/)
  assert.deepEqual(blocked.selection, { focus: 1, select: 1 })
  await (blocked.button('复制链接').props.onClick as () => Promise<void>)()
  assert.deepEqual(blocked.selection, { focus: 2, select: 2 })
})

test('creating stays creating when the receipt arrives and copying waits for the frozen snapshot', async () => {
  let finishCreate!: () => void
  let finishRead!: (snapshot: SharedConversationSnapshot) => void
  const h = harness({ fetch: () => new Promise(resolve => { finishRead = resolve }) })
  h.props.onCreate = () => new Promise(resolve => { finishCreate = resolve })
  const creating = (h.button('生成链接').props.onClick as () => Promise<void>)()
  h.props.busy = true
  h.props.url = 'http://localhost/?conversation-share=new'
  h.props.snapshot.share = { token: 'new', created_at: 1, message_count: 1 }
  assert.equal(h.button('正在生成').props.disabled, true)
  assert.doesNotMatch(h.html(), /正在撤销|data-copy-success/)
  finishCreate()
  await creating
  h.props.busy = false
  assert.equal(h.button('读取中').props.disabled, true)
  assert.doesNotMatch(h.html(), /data-copy-success/)
  finishRead({ title: 'Shared', created_at: 1, messages: [{ role: 'assistant', seq: 1, text: 'Saved answer' }] })
  await h.flush()
  assert.equal(h.button('复制链接').props.disabled, false)
  assert.match(h.html(), /Saved answer/)
})

test('clipboard latency shows pending feedback and only completed copies replay success', async () => {
  let finishCopy!: () => void
  const h = harness({ copy: () => new Promise(resolve => { finishCopy = resolve }) })
  h.props.url = 'http://localhost/?conversation-share=ready'
  const first = (h.button('复制链接').props.onClick as () => Promise<void>)()
  assert.equal(h.button('正在复制').props.disabled, true)
  assert.doesNotMatch(h.html(), /data-copy-success|已复制/)
  finishCopy()
  await first
  assert.match(h.html(), /data-copy-success="1"/)
  const second = (h.button('已复制').props.onClick as () => Promise<void>)()
  assert.equal(h.button('正在复制').props.disabled, true)
  assert.doesNotMatch(h.html(), /data-copy-success/)
  finishCopy()
  await second
  assert.match(h.html(), /data-copy-success="2"/)
})

test('failed requests retain the share and report the failure inside the dialog', async () => {
  const h = harness()
  h.props.onCreate = async () => { throw new Error('create failed') }
  await (h.button('生成链接').props.onClick as () => Promise<void>)()
  assert.match(h.html(), /create failed/)
  h.props.url = 'http://localhost/?conversation-share=retained'
  await (h.button('复制链接').props.onClick as () => Promise<void>)()
  assert.match(h.html(), /data-copy-success/)
  h.props.onRevoke = async () => { throw new Error('revoke failed') }
  await (h.button('撤销链接').props.onClick as () => Promise<void>)()
  assert.match(h.html(), /revoke failed/)
  assert.doesNotMatch(h.html(), /data-copy-success/)
  assert.equal(h.props.url, 'http://localhost/?conversation-share=retained')
})

test('a failed frozen preview blocks copying and retries the same snapshot', async () => {
  let requests = 0
  const h = harness({ fetch: async () => {
    requests++
    if (requests === 1) throw new Error('unavailable')
    return { title: 'Saved', created_at: 1, messages: [{ role: 'assistant', seq: 1, text: 'Saved answer' }] }
  } })
  h.props.url = 'http://localhost/?conversation-share=saved'
  h.props.snapshot.share = { token: 'saved', created_at: 1, message_count: 1 }
  h.render()
  await h.flush()
  assert.match(h.html(), /分享快照读取失败/)
  assert.equal(h.button('复制链接').props.disabled, true)
  ;(h.button('重试').props.onClick as () => void)()
  h.render()
  await h.flush()
  assert.match(h.html(), /Saved answer/)
  assert.equal(h.button('复制链接').props.disabled, false)
  assert.equal(requests, 2)
})

test('changing a share token discards the previous preview response', async () => {
  const pending = new Map<string, (value: SharedConversationSnapshot) => void>()
  const h = harness({ fetch: token => new Promise(resolve => pending.set(token, resolve)) })
  h.props.url = 'http://localhost/?conversation-share=old'
  h.props.snapshot.share = { token: 'old', created_at: 1, message_count: 1 }
  h.render()
  h.props.url = 'http://localhost/?conversation-share=new'
  h.props.snapshot.share = { token: 'new', created_at: 2, message_count: 1 }
  h.render()
  pending.get('old')!({ title: 'Old', created_at: 1, messages: [{ role: 'assistant', seq: 1, text: 'Stale answer' }] })
  await h.flush()
  assert.doesNotMatch(h.html(), /Stale answer/)
  pending.get('new')!({ title: 'New', created_at: 2, messages: [{ role: 'assistant', seq: 1, text: 'Current answer' }] })
  await h.flush()
  assert.match(h.html(), /Current answer/)
})

test('success artwork uses the original Pixel Icon check and appears whole with square fireworks', () => {
  const markup = renderToStaticMarkup(React.createElement(React.Fragment, null,
    React.createElement(success.SuccessCheck), React.createElement(success.SuccessCheck))) as string
  assert.match(markup, /color="#22c55e"/)
  assert.match(markup, /width="168" height="168" viewBox="0 0 24 24" fill="currentColor"/)
  assert.equal([...markup.matchAll(/data-icon="pixel:check"/g)].length, 2)
  assert.equal([...markup.matchAll(/data-pixel-check=""/g)].length, 2)
  assert.equal([...markup.matchAll(/data-firework=""/g)].length, 6)
  assert.equal([...markup.matchAll(/width="4" height="4"/g)].length, 96)
  assert.match(markup, /shape-rendering="crispEdges"/)
  assert.doesNotMatch(markup, /<defs|Gradient|<circle|<filter|<line |stroke-linecap|url\(/)
  assert.match(markup, /data-success-check/)
  const iconSource = readFileSync(new URL('../src/components/SuccessCheck.tsx', import.meta.url), 'utf8')
  assert.match(iconSource, /CC BY 4\.0/)
  assert.match(iconSource, /HackerNoon/)
  const notice = readFileSync(new URL('../public/pixel-icon-NOTICE.txt', import.meta.url), 'utf8')
  assert.match(notice, /https:\/\/creativecommons.org\/licenses\/by\/4.0\//)
  const css = readFileSync(new URL('../src/components/SuccessCheck.module.css', import.meta.url), 'utf8')
  assert.match(css, /prefers-reduced-motion:\s*reduce/)
  assert.match(css, /0%, 78% \{ visibility: visible; opacity: 1;/)
  assert.doesNotMatch(css, /clip-path|@keyframes draw|@keyframes settle/)
  assert.doesNotMatch(css, /infinite|gradient|filter|shadow/)
})
