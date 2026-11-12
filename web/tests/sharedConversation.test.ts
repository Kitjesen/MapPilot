import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'

const require = createRequire(import.meta.url)
const { renderToStaticMarkup } = require('react-dom/server')

function compile(path: string) {
  return ts.transpileModule(readFileSync(new URL(path, import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
}

const markdown: Record<string, unknown> = {}
runInNewContext(compile('../src/guide/markdown.ts'), { exports: markdown })

class ConversationError extends Error {
  status: number
  constructor(status: number) { super(`HTTP ${status}`); this.status = status }
}

interface Snapshot {
  title: string
  created_at: number
  messages: Array<{ role: 'user' | 'assistant'; text: string; seq: number }>
}

type Element = { type?: string; props?: { children?: unknown; onClick?: () => void; disabled?: boolean } }

function elements(node: unknown): Element[] {
  if (Array.isArray(node)) return node.flatMap(elements)
  if (!node || typeof node !== 'object') return []
  const element = node as Element
  return [element, ...elements(element.props?.children)]
}

function harness(fetch: (token: string) => Promise<Snapshot>, images: Array<{ loading: string; decode: () => Promise<void> }> = []) {
  const slots: Array<{ value?: unknown; deps?: unknown[]; cleanup?: () => void }> = []
  let cursor = 0
  let token = 'share-a'
  let effects: Array<() => void> = []
  const requests: string[] = []
  const document = { title: 'LingTu' }
  let prints = 0
  const react = {
    useRef: () => ({ current: { querySelectorAll: () => images } }),
    useState(initial: unknown) {
      const index = cursor++
      slots[index] ??= { value: initial }
      return [slots[index].value, (value: unknown) => {
        slots[index].value = typeof value === 'function' ? value(slots[index].value) : value
      }]
    },
    useEffect(effect: () => (() => void) | undefined, deps: unknown[]) {
      const index = cursor++
      const previous = slots[index]
      if (!previous?.deps || deps.some((value, i) => !Object.is(value, previous.deps?.[i]))) {
        effects.push(() => {
          previous?.cleanup?.()
          slots[index] = { deps, cleanup: effect() }
        })
      }
    },
  }
  const exports: Record<string, unknown> = {}
  runInNewContext(compile('../src/components/SharedConversation.tsx'), { exports, document, window: { print: () => { prints++ } }, require: (name: string) => {
    if (name === 'react') return react
    if (name.endsWith('/guide/markdown')) return markdown
    if (name.endsWith('/services/agentConversation')) return {
      ConversationError,
      fetchSharedConversation: (id: string) => { requests.push(id); return fetch(id) },
    }
    if (name.endsWith('.css')) return { default: new Proxy({}, { get: (_target, key) => key }) }
    return require(name)
  } })
  const component = exports.SharedConversation as (props: { token: string; locale: string }) => unknown
  const render = () => {
    cursor = 0
    effects = []
    const tree = component({ token, locale: 'zh' })
    effects.forEach(effect => effect())
    return tree
  }
  return {
    render,
    html: () => renderToStaticMarkup(render()) as string,
    flush: async () => { for (let i = 0; i < 8; i++) await Promise.resolve() },
    setToken: (next: string) => { token = next },
    requests,
    prints: () => prints,
    document,
    button: (label: string) => elements(render()).find(element => element.type === 'button' && renderToStaticMarkup(element).includes(label)),
  }
}

test('shared conversations render safe read-only messages from one share request', async () => {
  const h = harness(async () => ({
    title: '玻璃通道检查', created_at: 1_799_999_999, messages: [
      { role: 'user', seq: 1, text: '**用户原文** <img src=x>' },
      { role: 'assistant', seq: 2, text: '**检查完成**\n\n<script>alert(1)</script>\n\n[bad](javascript:alert)' },
    ],
  }))
  assert.match(h.html(), /正在读取分享/)
  await h.flush()
  const html = h.html()
  assert.deepEqual(h.requests, ['share-a'])
  assert.match(html, /只读分享/)
  assert.match(html, /玻璃通道检查/)
  assert.equal(h.document.title, '玻璃通道检查 · LingTu')
  assert.match(html, /<strong>检查完成<\/strong>/)
  assert.match(html, /\*\*用户原文\*\* &lt;img src=x&gt;/)
  assert.match(html, /&lt;script&gt;alert\(1\)&lt;\/script&gt;/)
  assert.match(html, /保存为 PDF/)
  assert.match(html, /这份对话停留在分享时刻/)
  assert.doesNotMatch(html, /<script|<img|href="javascript:|<textarea|<input|启用控制|允许执行|继续执行/)
})

test('revoked and unavailable shares show their state and retry only the same share request', async () => {
  let status = 404
  const h = harness(async () => { throw new ConversationError(status) })
  h.render()
  await h.flush()
  assert.match(h.html(), /分享已撤销或不存在/)
  assert.equal(h.button('保存为 PDF'), undefined)
  status = 503
  h.button('重试')?.props?.onClick?.()
  assert.match(h.html(), /正在读取分享/)
  assert.equal(h.document.title, 'LingTu')
  await h.flush()
  assert.match(h.html(), /分享服务暂不可用/)
  assert.deepEqual(h.requests, ['share-a', 'share-a'])
})

test('switching share tokens ignores the previous request result', async () => {
  const pending = new Map<string, (snapshot: Snapshot) => void>()
  const h = harness(token => new Promise(resolve => { pending.set(token, resolve) }))
  h.render()
  h.setToken('share-b')
  h.render()
  pending.get('share-a')!({ title: '旧会话', created_at: 1_799_999_999, messages: [] })
  await h.flush()
  assert.doesNotMatch(h.html(), /旧会话/)
  assert.match(h.html(), /正在读取分享/)
  pending.get('share-b')!({ title: '新会话', created_at: 1_799_999_999, messages: [] })
  await h.flush()
  assert.match(h.html(), /新会话/)
  assert.deepEqual(h.requests, ['share-a', 'share-b'])
})

test('PDF opens the print dialog only after the conversation and its images have loaded', async () => {
  let finishImage!: () => void
  const image = { loading: 'lazy', decode: () => new Promise<void>(resolve => { finishImage = resolve }) }
  const h = harness(async () => ({ title: '分享标题', created_at: 1_799_999_999, messages: [] }), [image])
  assert.equal(h.button('保存为 PDF'), undefined)
  await h.flush()
  h.button('保存为 PDF')?.props?.onClick?.()
  assert.equal(image.loading, 'eager')
  assert.equal(h.prints(), 0)
  assert.equal(h.button('正在准备')?.props?.disabled, true)
  finishImage()
  await h.flush()
  assert.equal(h.prints(), 1)
  assert.equal(h.button('保存为 PDF')?.props?.disabled, false)
})

test('failed images require an explicit choice before printing an incomplete PDF', async () => {
  const h = harness(async () => ({ title: '分享标题', created_at: 1_799_999_999, messages: [] }), [
    { loading: 'lazy', decode: async () => { throw new Error('image unavailable') } },
  ])
  h.render()
  await h.flush()
  h.button('保存为 PDF')?.props?.onClick?.()
  await h.flush()
  assert.equal(h.prints(), 0)
  assert.match(h.html(), /role="alert"/)
  assert.match(h.html(), /部分图片加载失败/)
  assert.equal(h.button('保存为 PDF')?.props?.disabled, false)
  h.button('仍然保存')?.props?.onClick?.()
  assert.equal(h.prints(), 1)
})

test('text-only snapshots can be saved without image preparation', async () => {
  const h = harness(async () => ({ title: '分享标题', created_at: 1_799_999_999, messages: [] }))
  h.render()
  await h.flush()
  h.button('保存为 PDF')?.props?.onClick?.()
  await h.flush()
  assert.equal(h.prints(), 1)
  assert.doesNotMatch(h.html(), /部分图片加载失败/)
})
