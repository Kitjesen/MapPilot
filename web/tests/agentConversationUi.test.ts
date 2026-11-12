import { conversationDependency } from './agentConversationFixtures.ts'
import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'
import { renderMarkdown } from '../src/guide/markdown.ts'
import * as markdownBlocks from '../src/components/agentConversation/markdownStreamBlocks.ts'

const require = createRequire(import.meta.url)
const source = ts.transpileModule(readFileSync(new URL('../src/components/AgentConversation.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText
const shareSource = ts.transpileModule(readFileSync(new URL('../src/components/ConversationShareCard.tsx', import.meta.url), 'utf8'), {
  compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
}).outputText

function component(react = require('react'), dependencies: Record<string, unknown> = {}) {
  const exports: Record<string, unknown> = {}
  runInNewContext(source, { exports, window: { matchMedia: () => ({ matches: false }) }, document: { addEventListener() {}, removeEventListener() {} }, require: (name: string) => {
    if (name in dependencies) return dependencies[name]
    const dependency = conversationDependency(name)
    if (dependency) return dependency
    if (name === 'react') return { ...require('react'), ...react }
    if (name.endsWith('/services/agentConversation.ts')) return { conversationToolNames: {} }
    if (name.endsWith('/useMapSettings.ts')) return { useMapSettings: () => null }
    if (name.endsWith('/agentConversation/ToolActivity.tsx')) return { toolNames: {}, ToolActivity: ({ message }: { message: { text: string } }) => createRequire(import.meta.url)('react').createElement('p', null, message.text) }
    if (name.endsWith('/guide/markdown')) return { renderMarkdown }
    if (name.endsWith('/agentConversation/markdownStreamBlocks.ts')) return markdownBlocks
    if (name.endsWith('/menuEvents')) return { useMenuDismiss() {} }
    if (name.endsWith('/agentConversation/EffortGlyph')) return { EffortGlyph: () => null }
    if (name.endsWith('/ConversationShareCard')) {
      const shareExports: Record<string, unknown> = {}
      runInNewContext(shareSource, { exports: shareExports, require: (dependency: string) => {
        if (dependency === 'react') return react
        if (dependency.endsWith('/Modal')) return { ModalShell: () => null }
        if (dependency.endsWith('/SuccessCheck')) return { SuccessCheck: () => null, PixelCheck: () => null }
        if (dependency.endsWith('/guide/markdown')) return { renderMarkdown: (text: string) => text }
        if (dependency.endsWith('/services/agentConversation')) return {}
        if (dependency.endsWith('.css')) return { default: new Proxy({}, { get: (_target, key) => key }) }
        return require(dependency)
      } })
      return shareExports
    }
    if (name.endsWith('.css')) return { default: new Proxy({}, { get: (_target, key) => key }) }
    return require(name)
  } })
  return exports.AgentConversation as (props: unknown) => unknown
}

function controller(overrides: Record<string, unknown> = {}) {
  return {
    open: true, draft: '', replyTo: '', setReplyTo() {}, editable: true, attachments: [], model: 'test', reasoning: 'default',
    permissionMode: 'default', approvalBusy: null, options: {
      models: [{ id: 'test', label: 'Test model', reasoning: [{ id: 'default', label: '默认' }] }],
      attachments: { accept: '.txt', max_files: 4, max_file_bytes: 1000 },
    },
    snapshot: { messages: [], turns: [], approvals: [] }, ...overrides,
  }
}

function html(overrides: Record<string, unknown> = {}) {
  return require('react-dom/server').renderToStaticMarkup(require('react').createElement(component(), {
    locale: 'zh', controller: controller(overrides),
  })) as string
}

function avatarState(overrides: Record<string, unknown> = {}) {
  const render = component({ useRef: () => ({ current: null }), useEffect() {}, useLayoutEffect() {},
    useState: (initial: unknown) => [typeof initial === 'function' ? initial() : initial, () => {}] })
  type Element = { props?: { children?: unknown; size?: number; state?: string } }
  function elements(node: unknown): Element[] {
    if (Array.isArray(node)) return node.flatMap(elements)
    if (!node || typeof node !== 'object') return []
    const element = node as Element
    return [element, ...elements(element.props?.children)]
  }
  const avatar = elements(render({ locale: 'zh', controller: controller(overrides) }))
    .find(element => typeof element.props?.shape === 'string' && typeof element.props.state === 'string')
  assert.ok(avatar)
  return avatar.props!.state
}

test('queued messages stay above the composer and collapse after three rows', () => {
  const result = html({ active: true, snapshot: {
    conversation_id: 'queue', messages: Array.from({ length: 4 }, (_, index) => ({
      id: `message-${index}`, turn_id: `turn-${index}`, seq: index + 1, role: 'user', text: `queued-${index}`,
    })), turns: Array.from({ length: 4 }, (_, index) => ({
      turn_id: `turn-${index}`, message_id: `message-${index}`, state: 'queued',
    })),
  } })
  assert.match(result, /待执行 4/)
  assert.match(result, /还有 1 条/)
  assert.match(result, /加入待执行/)
  assert.equal((result.match(/queued-0/g) ?? []).length, 2)
  assert.doesNotMatch(result, /queued-3/)
})

test('unfinished turn feedback stays beside its request instead of below newer messages', () => {
  const markup = html({ snapshot: { conversation_id: 'feedback', messages: [
    { id: 'first', turn_id: 'one', seq: 1, role: 'user', text: '检查地图' },
    { id: 'second', turn_id: 'two', seq: 2, role: 'user', text: '检查定位' },
  ], turns: [{ turn_id: 'one', state: 'cancelled', text: '这次检查已取消' }, { turn_id: 'two', state: 'running' }] } })
  const articles = markup.match(/<article\b[\s\S]*?<\/article>/g) ?? []
  assert.match(articles[0], /这次检查已取消/)
  assert.doesNotMatch(articles[1], /这次检查已取消/)
  assert.equal((markup.match(/这次检查已取消/g) ?? []).length, 1)
})

test('an empty history stays compact through background refresh', () => {
  const idle = html()
  const refreshing = html({ loading: true })
  assert.equal(refreshing, idle)
  assert.doesNotMatch(idle, /role="log"/)
  assert.match(idle, /rows="1"/)
  assert.doesNotMatch(idle, /可以提问|读取中/)
  assert.match(html({ loading: true, snapshot: null }), /读取中/)
})

test('a quote is rendered beside a clean question draft and linked from sent history', () => {
  const answer = { id: 'answer', seq: 1, role: 'assistant', turn_id: 'one', text: '**地图查看完成。**\n\n[查看使用指南](https://example.com/guide)' }
  const markup = html({ draft: '补面具体怎么做？', replyTo: 'answer', snapshot: { messages: [answer], turns: [] } })
  assert.match(markup, /aria-label="引用的回复"/)
  assert.match(markup, /aria-label="取消引用"/)
  assert.match(markup, /title="展开或收起引用"/)
  assert.match(markup, /<strong>地图查看完成。<\/strong>/)
  assert.match(markup, /<a class="link-capsule external-link" href="https:\/\/example.com\/guide"/)
  const draft = markup.match(/<textarea[^>]*>([\s\S]*?)<\/textarea>/)![1]
  assert.equal(draft, '补面具体怎么做？')
  assert.doesNotMatch(markup, /关于这条回复|&gt; \*\*/)
  const sent = html({ snapshot: { messages: [answer, { id: 'question', seq: 2, role: 'user', turn_id: 'two', text: '补面具体怎么做？' }],
    turns: [{ turn_id: 'two', state: 'completed', reply_to_message_id: 'answer' }] } })
  assert.match(sent, /aria-label="引用的回复"/)
  assert.doesNotMatch(sent, /aria-label="取消引用"/)
})

test('conversation stream renders inline cards, collaboration and reactions', () => {
  const markup = html({ snapshot: { conversation_id: 'conversation-1', messages: [{
    id: 'card', seq: 1, role: 'assistant', turn_id: 'one', text: '已找到相关内容。',
    metadata: {
      card: { title: '巡检草稿', subtitle: '车库 A 区', price: '待确认', actions: [{ label: '打开设置', href: '/?tab=scene' }] },
      collaboration: { name: '地图助手', status: '已完成', summary: '返回了 3 个候选点', href: '/?conversation=assistant-map' },
    },
  }], turns: [] } })
  assert.match(markup, /aria-label="巡检草稿"/)
  assert.match(markup, /巡检草稿/)
  assert.match(markup, /地图助手/)
  assert.match(markup, /aria-label="消息回应"/)
  assert.match(markup, /打开设置/)
})

test('a structured plan appears once as an unexecuted draft with export actions', () => {
  const markup = html({ snapshot: { conversation_id: 'plans', turns: [{ turn_id: 'plan', state: 'completed' }], messages: [{
    id: 'plan-result', seq: 1, role: 'assistant', turn_id: 'plan', kind: 'plan',
    text: '# 巡检方案\n\n1. 检查地图版本\n\n尚未执行。', metadata: { plan: { title: '巡检方案', summary: '先核对现场条件。',
      steps: [{ title: '检查地图版本', detail: '确认地图与当前现场一致。' }] } },
  }] } })
  assert.match(markup, /方案草稿 · 未执行/)
  assert.match(markup, /<details[^>]*><summary[^>]*aria-label="方案：巡检方案"/)
  assert.equal((markup.match(/检查地图版本/g) ?? []).length, 1)
  assert.doesNotMatch(markup, /# 巡检方案|立即执行|执行方案|已完成方案/)
  assert.match(markup, /aria-label="复制方案 Markdown"/)
  assert.match(markup, /aria-label="下载方案 Markdown"/)
  assert.match(html(), /制定方案/)
})

test('selected text stays literal in the composer and sent quote', () => {
  const answer = { id: 'answer', seq: 1, role: 'assistant', turn_id: 'one', text: '完整回复，不应整条出现在引用块中。' }
  const quote = '路线会显示在地图上，方便你\n检查起点、终点。 <b>原文</b>'
  const composer = html({ draft: '这是什么意思？', replyTo: 'answer', replyQuote: quote,
    snapshot: { messages: [answer], turns: [] } })
  assert.match(composer, /<p[^>]*>路线会显示在地图上，方便你\n检查起点、终点。 &lt;b&gt;原文&lt;\/b&gt;<\/p>/)
  assert.doesNotMatch(composer, /<b>原文<\/b>/)
  assert.equal((composer.match(/完整回复，不应整条出现在引用块中。/g) ?? []).length, 1)
  const sent = html({ snapshot: { messages: [answer, { id: 'question', seq: 2, role: 'user', turn_id: 'two', text: '这是什么意思？' }],
    turns: [{ turn_id: 'two', state: 'completed', reply_to_message_id: 'answer', reply_quote: quote }] } })
  assert.match(sent, /<p[^>]*>路线会显示在地图上/)
  assert.equal((sent.match(/完整回复，不应整条出现在引用块中。/g) ?? []).length, 1)
})

test('an unavailable Agent has one stable notice without empty history panels', () => {
  const markup = html({ options: null, snapshot: null, optionsError: 'Agent 未启用', error: 'signal timed out' })
  assert.equal((markup.match(/对话暂不可用/g) ?? []).length, 1)
  assert.doesNotMatch(markup, /signal timed out|读取中|class="history"/)
  assert.match(markup, /title="Agent 未启用"/)
  assert.match(markup, /aria-label="刷新模型连接"/)
})

test('the bot distinguishes processing, failed and terminal conversation states', () => {
  assert.equal(avatarState({ active: true }), 'processing')
  assert.equal(avatarState({ snapshot: { messages: [], turns: [{ turn_id: 'turn', state: 'completed' }] } }), 'done')
  assert.equal(avatarState({ snapshot: { messages: [], turns: [{ turn_id: 'turn', state: 'failed' }] } }), 'error')
  for (const state of ['cancelled', 'interrupted']) assert.equal(avatarState({ snapshot: { messages: [], turns: [{ turn_id: 'turn', state }] } }), 'idle')
  assert.equal(avatarState({ options: null }), 'unavailable')
  assert.equal(avatarState({ options: null, optionsLoading: true }), 'processing')
})

test('pending approval has priority over connection, upload and sending feedback', () => {
  const waiting = { active: true, error: 'network error', optionsLoading: true, uploading: true, sending: true,
    snapshot: { messages: [], turns: [], approvals: [
    { approval_id: 'one', turn_id: 'turn', tool_name: 'navigate_to', arguments: {}, state: 'pending' },
  ] } }
  assert.equal(avatarState(waiting), 'waiting')
  const markup = html(waiting)
  assert.match(markup, /等待确认/)
  assert.match(markup, /data-tone="waiting"/)
  assert.doesNotMatch(markup, /data-working="true"|正在上传附件/)
})

test('the bot follows current-turn tools and text instead of old history', () => {
  const oldTool = { id: 'old-tool', seq: 1, role: 'tool', turn_id: 'old', kind: 'inspect_saved_map', text: 'old', metadata: { status: 'running' } }
  const oldReply = { id: 'old-reply', seq: 2, role: 'assistant', turn_id: 'old', text: 'old', metadata: { status: 'streaming' } }
  const streaming = { id: 'reply', seq: 3, role: 'assistant', turn_id: 'current', text: '正在生成的真实文本', metadata: { status: 'streaming' } }
  const tool = { ...oldTool, id: 'current-tool', seq: 4, turn_id: 'current' }
  for (const message of [{ ...oldTool, turn_id: undefined }, { ...oldReply, turn_id: undefined }]) {
    assert.equal(avatarState({ active: true, snapshot: { turns: [], messages: [message] } }), 'processing')
  }
  const snapshot = { turns: [{ turn_id: 'old', state: 'completed' }, { turn_id: 'current', state: 'running' }], messages: [oldTool, oldReply] }
  assert.equal(avatarState({ snapshot }), 'processing')
  assert.equal(avatarState({ snapshot: { ...snapshot, messages: [...snapshot.messages, streaming] } }), 'writing')
  assert.equal(avatarState({ snapshot: { ...snapshot, messages: [...snapshot.messages, streaming, tool] } }), 'working')
  const completedTool = { ...tool, id: 'other-tool', seq: 5, metadata: { status: 'completed' } }
  assert.equal(avatarState({ snapshot: { ...snapshot, messages: [tool, completedTool] } }), 'working')
  assert.equal(avatarState({ snapshot: { ...snapshot, messages: [completedTool] } }), 'processing')
  const laterOtherReply = { ...oldReply, id: 'later-other', seq: 6, metadata: { status: 'completed' } }
  const writing = { snapshot: { ...snapshot, messages: [streaming, laterOtherReply] } }
  assert.equal(avatarState(writing), 'writing')
  assert.match(html(writing), /正在回复/)
  assert.match(html({ active: true, snapshot: { ...snapshot, messages: [completedTool] } }), /正在处理工具结果/)
})

test('sending and uploading are processing while connection failures stop activity feedback', () => {
  for (const busy of [{ sending: true }, { uploading: true }, { optionsLoading: true }, { sessionBusy: 'fork' }]) {
    assert.equal(avatarState(busy), 'processing')
  }
  assert.match(html({ sending: true }), /正在发送/)
  assert.match(html({ uploading: true }), /正在上传附件/)
  const broken = { active: true, error: 'offline', snapshot: { messages: [{
    id: 'tool', seq: 1, role: 'tool', turn_id: 'one', kind: 'inspect_saved_map', text: 'running', metadata: { status: 'running' },
  }], turns: [{ turn_id: 'one', state: 'running' }] } }
  assert.equal(avatarState(broken), 'error')
  assert.match(html(broken), /连接异常，请重试/)
  assert.doesNotMatch(html(broken), /data-working="true"/)
  assert.equal(avatarState({ options: null, error: 'offline' }), 'unavailable')
})

test('suggestions fill the draft without submitting a message', () => {
  const drafts: string[] = []
  let sends = 0
  const render = component({ useRef: () => ({ current: null }), useState: () => [false, () => {}], useEffect() {}, useLayoutEffect() {} })
  const tree = render({ locale: 'zh', controller: controller({ setDraft: (value: string) => drafts.push(value), send: () => { sends++ } }) })
  type Element = { type?: string; props?: { children?: unknown; onClick?: () => void } }
  function elements(node: unknown): Element[] {
    if (Array.isArray(node)) return node.flatMap(elements)
    if (!node || typeof node !== 'object') return []
    const element = node as Element
    return [element, ...elements(element.props?.children)]
  }
  const suggestion = elements(tree).find(element => element.type === 'button' && element.props?.children === '查看机器人状态')
  assert.ok(suggestion)
  suggestion.props?.onClick?.()
  assert.deepEqual(drafts, ['查看机器人状态'])
  assert.equal(sends, 0)
})

test('reply times and completed work duration use stored timestamps', () => {
  const markup = html({ snapshot: { messages: [
    { id: 'reply', seq: 1, role: 'assistant', turn_id: 'turn', text: 'done', created_at: 1700000008 },
  ], turns: [{ turn_id: 'turn', state: 'completed', started_at: 1700000000, finished_at: 1700000008.5 }] } })
  assert.match(markup, /<time dateTime="2023-11-14T22:13:28.000Z"/)
  assert.match(markup, /用时 8.5 秒/)
  assert.match(markup, /aria-label="复制回复"/)
  const old = html({ snapshot: { messages: [
    { id: 'old', seq: 1, role: 'assistant', turn_id: 'old-turn', text: 'old reply' },
  ], turns: [{ turn_id: 'old-turn', state: 'completed' }] } })
  assert.doesNotMatch(old, /<time|用时/)
  assert.match(old, /old reply/)
})

test('stored SDK errors stay failed and expose raw content only as literal diagnostics', () => {
  const reason = "Unknown content: {'annotations': [], 'text': '| 地图 |\\n<script>bad</script>', 'type': 'output_text'}"
  const markup = html({ snapshot: { messages: [
    { id: 'broken', seq: 1, role: 'assistant', kind: 'failed', turn_id: 'turn', text: reason },
  ], turns: [{ turn_id: 'turn', state: 'failed', failure_reason: reason }] } })
  assert.match(markup, /对话内容解析失败，本轮未完成。请重试。/)
  assert.match(markup, /<details><summary>错误详情<\/summary><pre>/)
  assert.doesNotMatch(markup, /<details open|<script>|<table|data-quote-source="broken"/)
  assert.match(markup, /&lt;script&gt;bad&lt;\/script&gt;/)
  assert.match(markup, /data-kind="failed"/)
  const literal = html({ snapshot: { messages: [
    { id: 'normal', seq: 1, role: 'assistant', kind: 'reply', turn_id: 'turn', text: 'Unknown content: 是一个错误示例。' },
  ], turns: [{ turn_id: 'turn', state: 'completed' }] } })
  assert.doesNotMatch(literal, /错误详情/)
  assert.match(literal, /Unknown content: 是一个错误示例。/)
})

test('only the latest answer offers regeneration and busy drafts keep it disabled', () => {
  const snapshot = { messages: [
    { id: 'first', seq: 1, role: 'assistant', turn_id: 'one', text: 'first' },
    { id: 'last', seq: 2, role: 'assistant', turn_id: 'two', text: 'last' },
  ], turns: [{ turn_id: 'one', state: 'completed' }, { turn_id: 'two', state: 'completed' }] }
  const markup = html({ snapshot })
  assert.equal((markup.match(/aria-label="重新生成回答"/g) ?? []).length, 1)
  assert.match(markup, /aria-label="引用追问"/)
  assert.doesNotMatch(markup, /更多回复操作|简短一点|展开说明/)
  assert.match(html({ snapshot, draft: 'draft' }), /disabled=""[^>]*aria-label="重新生成回答"/)
})

test('work text follows actual tool events and terminal failure stops the tool animation', () => {
  const start = { id: 'start', seq: 1, role: 'tool', turn_id: 'one', kind: 'tool_start', text: '{"tool":"get_agent_status"}' }
  const running = html({ active: true, snapshot: { messages: [start], turns: [{ turn_id: 'one', state: 'running' }] } })
  assert.match(running, /正在调用/)
  assert.match(running, /data-running="true"/)
  const waiting = html({ active: true, snapshot: { messages: [start], turns: [{ turn_id: 'one', state: 'running' }], approvals: [
    { approval_id: 'approval', turn_id: 'one', tool_name: 'get_agent_status', arguments: {}, state: 'pending' },
  ] } })
  assert.match(waiting, /待确认/)
  assert.doesNotMatch(waiting, /data-working="true"|data-running="true"/)
  const result = { id: 'result', seq: 2, role: 'tool', turn_id: 'one', kind: 'get_agent_status', text: '{"ok":false,"reason":"定位未就绪"}' }
  const processing = html({ active: true, snapshot: { messages: [start, result], turns: [{ turn_id: 'one', state: 'running' }] } })
  assert.match(processing, /正在处理工具结果/)
  assert.match(processing, /定位未就绪/)
  assert.doesNotMatch(processing, /<pre/)
  const failure = html({ snapshot: { messages: [start], turns: [{ turn_id: 'one', state: 'failed' }] } })
  assert.match(failure, /已发起/)
  assert.match(failure, /本轮未完成/)
  assert.doesNotMatch(failure, /data-working="true"|data-running="true"/)
})

test('legacy starts fold into their result and multiple actual tools retain the process disclosure', () => {
  const start = { id: 'start', seq: 1, role: 'tool', turn_id: 'one', kind: 'tool_start', text: '{"tool":"list_saved_maps"}' }
  const output = { id: 'output', seq: 2, role: 'tool', turn_id: 'one', kind: 'list_saved_maps', text: '{"maps":[]}' }
  const render = (messages: unknown[], state = 'completed') => html({ snapshot: { messages, turns: [{ turn_id: 'one', state }] } })
  const complete = render([start, output])
  assert.equal((complete.match(/aria-label="展开工具过程"/g) ?? []).length, 0)
  assert.doesNotMatch(complete, /已发起/)
  assert.match(complete, /已返回/)
  assert.doesNotMatch(render([start, output], 'running'), /aria-label="展开工具过程"/)
  const failed = render([start, { ...output, text: '{"error":"地图未加载"}' }])
  assert.doesNotMatch(failed, /aria-label="展开工具过程"/)
  assert.match(failed, /地图未加载/)
  const preview = render([start, { ...output, text: '{"preview_request":{"map_id":"map","expected_source_content_epoch":1,"operations":[]}}' }])
  assert.doesNotMatch(preview, /aria-label="展开工具过程"/)
  assert.match(preview, /查看三维/)
  const separated = render([start, output, { id: 'answer', seq: 3, role: 'assistant', turn_id: 'one', text: '继续' },
    { ...start, id: 'second-start', seq: 4, turn_id: 'two' }, { ...output, id: 'second-output', seq: 5, turn_id: 'two' }])
  assert.equal((separated.match(/已返回/g) ?? []).length, 2)
  const short = [output, { ...output, id: 'another', seq: 3, kind: 'get_agent_status' }]
  assert.doesNotMatch(render(short), /aria-label="展开工具过程"/)
  assert.match(render([...short, { ...output, id: 'third', seq: 4 }, { ...output, id: 'fourth', seq: 5 }]), /aria-label="展开工具过程"/)
})

test('step states follow tool events and a failed call never appears completed', () => {
  const tool = { id: 'tool', seq: 1, role: 'tool', turn_id: 'one', kind: 'inspect_saved_map', text: '执行中',
    metadata: { status: 'running', started_at: 1000 } }
  const render = (message: unknown, active = true, approvals: unknown[] = []) => html({ active,
    snapshot: { messages: [message], approvals, turns: [{ turn_id: 'one', state: active ? 'running' : 'failed' }] } })
  const running = render(tool)
  assert.match(running, /<ol[^>]*aria-label="执行步骤"/)
  assert.match(running, /data-state="running"/)
  assert.doesNotMatch(running, /data-state="completed"/)
  assert.match(render(tool, true, [{ approval_id: 'approval', turn_id: 'one', tool_name: 'inspect_saved_map', arguments: {}, state: 'pending' }]), /data-state="waiting"/)
  assert.match(render(tool, false), /data-state="pending"/)
  assert.match(render({ ...tool, text: '{"success":true}', metadata: { status: 'completed' } }), /data-state="completed"/)
  const rejected = render({ ...tool, text: '{"success":false,"reason":"地图不存在"}', metadata: { status: 'completed' } })
  assert.match(rejected, /data-state="failed"/)
  assert.doesNotMatch(rejected, /data-state="completed"/)
  const failed = render({ ...tool, metadata: { status: 'failed' } })
  assert.match(failed, /data-state="failed"/)
  assert.doesNotMatch(failed, /data-running="true"|data-state="completed"/)
})

test('a nonzero exit stays visible even when the turn completed', () => {
  const markup = html({ snapshot: { messages: [
    { id: 'command', seq: 1, role: 'tool', turn_id: 'one', kind: 'exec_command', text: '{"command":"check-map","exit_code":2,"stderr":"missing map"}' },
    { id: 'status', seq: 2, role: 'tool', turn_id: 'one', kind: 'get_agent_status', text: '{"ok":true}' },
  ], turns: [{ turn_id: 'one', state: 'completed' }] } })
  assert.doesNotMatch(markup, /aria-label="展开工具过程"/)
  assert.match(markup, /data-failed="true"/)
  assert.match(markup, /退出 2/)
  assert.doesNotMatch(markup, /<pre/)
})

test('running work uses server time and only the last reply shows the final duration', () => {
  const running = html({ active: true, snapshot: { messages: [], server_time: 1012, turns: [
    { turn_id: 'turn', state: 'running', started_at: 1000 },
  ] } })
  assert.match(running, /已用时 12 秒/)
  const completed = html({ snapshot: { messages: [
    { id: 'first', seq: 1, role: 'assistant', turn_id: 'turn', text: 'first reply' },
    { id: 'last', seq: 2, role: 'assistant', turn_id: 'turn', text: 'last reply' },
  ], turns: [{ turn_id: 'turn', state: 'completed', started_at: 1000, finished_at: 1009 }] } })
  assert.equal((completed.match(/用时 9.0 秒/g) ?? []).length, 1)
})

test('the permission menu shows three modes and locks with an unconfirmed message', () => {
  const markup = html()
  assert.match(markup, /aria-label="权限"/)
  assert.match(markup, /<span>只读<\/span>/)
  assert.match(markup, /<span>执行前确认<\/span>/)
  assert.match(markup, /<span>Full access<\/span>/)
  assert.match(html({ editable: false, pending: { message_id: 'original' } }),
    /aria-label="权限"[^>]*aria-disabled="true"/)
})

test('permission stays to the left of model and reasoning controls', () => {
  const markup = html()
  assert.ok(markup.indexOf('aria-label="添加内容"') < markup.indexOf('aria-label="权限"'))
  assert.ok(markup.indexOf('aria-label="权限"') < markup.indexOf('aria-label="模型"'))
  assert.ok(markup.indexOf('aria-label="模型"') < markup.indexOf('aria-label="思考深度"'))
})

test('composer offers an add menu and keeps history in the header', () => {
  const markup = html({ snapshot: { conversation_id: 'one', turns: [], messages: [
    { id: 'reply', seq: 1, role: 'assistant', turn_id: 'turn', text: '回复' },
  ] } })
  assert.match(markup, /aria-label="添加内容"[^>]*aria-haspopup="menu"/)
  assert.match(markup, /role="menu" aria-label="机器人功能与插件"/)
  assert.match(markup, /role="menuitem"[^>]*>[\s\S]*?文件与图片/)
  assert.match(markup, /机器人状态/)
  assert.match(markup, /前往目标/)
  assert.match(markup, /安全状态/)
  assert.match(markup, /插件与组件/)
  const addMenu = markup.match(/<div[^>]*role="menu" aria-label="机器人功能与插件"[\s\S]*?<\/div><\/details>/)?.[0] ?? ''
  assert.ok(addMenu)
  assert.doesNotMatch(addMenu, /Bot 形象与角色|更多机器人设置/)
  const header = markup.match(/<header[^>]*>([\s\S]*?)<\/header>/)![1]
  assert.match(header, /aria-label="会话历史"/)
  assert.match(header, /aria-label="打开 Bot 形象与角色"/)
  assert.doesNotMatch(markup, /aria-label="会话记录"/)
})

test('default thinking is icon-only and permissions keep their selected mode', () => {
  const markup = html()
  const thinking = markup.match(/<summary[^>]*aria-label="思考深度"[\s\S]*?<\/summary>/)![0]
  assert.doesNotMatch(thinking, /<span>默认<\/span>|lucide-sparkles/)
  assert.match(thinking, /lucide-sliders-horizontal/)
  assert.match(markup, /data-mode="default"/)
  assert.match(markup, /执行前确认/)
  assert.match(markup, /role="menuitemradio"[^>]*aria-checked="true"[^>]*>[\s\S]*?默认/)
})

test('conversation management exposes explicit actions and does not create a share on open', () => {
  let shares = 0
  const markup = html({ sessionListOpen: true, archivedView: false,
    shareCurrentConversation: () => { shares++ },
    conversations: [{ conversation_id: 'one', title: '办公室任务', archived: false }],
    snapshot: { conversation_id: 'one', messages: [], turns: [], metadata: { title: '办公室任务', archived: false } },
  })
  assert.match(markup, /历史会话/)
  assert.doesNotMatch(markup, /class="history"/)
  assert.match(markup, /aria-label="新建会话"/)
  assert.match(markup, /aria-label="查看归档会话"/)
  assert.match(markup, /办公室任务/)
  assert.match(markup, /重命名/)
  assert.match(markup, /归档/)
  assert.match(markup, /分享/)
  assert.doesNotMatch(markup, /创建分享链接/)
  assert.equal(shares, 0)
})

test('archived conversations are read only and session switching disables draft actions', () => {
  const archived = html({ draft: '保留的草稿', snapshot: {
    conversation_id: 'one', messages: [], turns: [], metadata: { title: '历史任务', archived: true },
  } })
  assert.match(archived, /已归档/)
  assert.match(archived, /<textarea[^>]*readOnly=""/)
  assert.match(archived, /<button[^>]*disabled=""[^>]*aria-label="发送消息"/)
  assert.match(archived, /保留的草稿/)
  const switching = html({ sessionBusy: 'two', draft: '草稿', sessionListOpen: true,
    conversations: [{ conversation_id: 'one', title: '原会话', archived: false }],
  })
  assert.match(switching, /<textarea[^>]*disabled=""/)
  assert.match(switching, /aria-label="权限"[^>]*aria-disabled="true"/)
  assert.match(switching, /处理中/)
})

for (const entry of ['header', 'menu']) test(`opening sharing from the ${entry} previews the current conversation without creating a link`, () => {
  const state: unknown[] = []
  let cursor = 0
  const react = {
    useRef: () => ({ current: null }), useEffect() {}, useLayoutEffect() {},
    useState(initial: unknown) {
      const index = cursor++
      if (!(index in state)) state[index] = typeof initial === 'function' ? initial() : initial
      return [state[index], (value: unknown) => { state[index] = value }]
    },
  }
  let created = 0
  const current = controller({ sessionListOpen: entry === 'menu', shareUrl: 'http://localhost/?conversation-share=current', shareBusy: false,
    shareCurrentConversation: async () => { created++ }, revokeShare: async () => {},
    conversations: [{ conversation_id: 'one', title: '当前会话', archived: false }],
    snapshot: { conversation_id: 'one', metadata: { title: '当前会话' }, messages: [
      { id: 'first', seq: 1, turn_id: 'turn', role: 'user', text: '当前问题' },
    ], turns: [], approvals: [] },
  })
  type Element = { type?: string | ((props: unknown) => unknown); props?: Record<string, unknown> }
  function elements(node: unknown): Element[] {
    if (Array.isArray(node)) return node.flatMap(elements)
    if (!node || typeof node !== 'object') return []
    const element = node as Element
    return [element, ...elements(element.props?.children)]
  }
  const renderComponent = component(react)
  const render = () => { cursor = 0; return renderComponent({ locale: 'zh', controller: current }) }
  let root = render()
  let actions = root
  if (entry === 'menu') {
    const sessions = elements(root).find(element => typeof element.type === 'function' && element.type.name === 'ConversationSessions')!
    actions = (sessions.type as (props: unknown) => unknown)(sessions.props)
  }
  const share = elements(actions).find(element => element.type === 'button' && (element.props?.['aria-label'] === '分享对话'
    || Array.isArray(element.props?.children) && element.props.children.includes('分享')))!
  ;(share.props!.onClick as (event: unknown) => void)({ currentTarget: { closest: () => null } })
  root = render()
  const cards = elements(root).filter(element => typeof element.type === 'function' && element.type.name === 'ConversationShareCard')
  assert.equal(cards.length, 1)
  const card = cards[0]
  assert.equal(card.props!.title, '当前会话')
  assert.equal(card.props!.url, current.shareUrl)
  assert.equal(card.props!.onCreate, current.shareCurrentConversation)
  assert.equal(card.props!.onRevoke, current.revokeShare)
  assert.equal(created, 0)
  ;(card.props!.onClose as () => void)()
  assert.equal(elements(render()).some(element => typeof element.type === 'function' && element.type.name === 'ConversationShareCard'), false)
  ;(share.props!.onClick as (event: unknown) => void)({ currentTarget: { closest: () => null } })
  current.snapshot.conversation_id = 'another'
  assert.equal(elements(render()).some(element => typeof element.type === 'function' && element.type.name === 'ConversationShareCard'), false)
})

test('the header keeps Bot beside an icon-only share entry and waits for session operations', () => {
  const snapshot = { conversation_id: 'one', metadata: { title: '地图讨论' }, messages: [], turns: [], approvals: [], archived: false }
  const ready = html({ snapshot: { ...snapshot, metadata: { ...snapshot.metadata, archived: true } } })
  assert.match(ready, /aria-label="分享对话"/)
  assert.match(ready, /预览并分享当前会话/)
  const header = ready.match(/<header\b[^>]*>([\s\S]*?)<\/header>/)![1]
  const share = header.match(/<button[^>]*aria-label="分享对话"[^>]*>([\s\S]*?)<\/button>/)![1]
  assert.match(share, /<svg/)
  assert.equal(share.replace(/<[^>]*>/g, '').trim(), '')
  assert.match(header, /aria-label="打开 Bot 形象与角色"/)
  assert.ok(header.indexOf('打开 Bot 形象与角色') < header.indexOf('aria-label="分享对话"'))
  const waiting = html({ sessionBusy: 'two', snapshot: { ...snapshot, metadata: { ...snapshot.metadata, archived: true } } })
  assert.match(waiting, /aria-label="分享对话"[^>]*disabled=""/)
})

test('branching is offered on terminal final replies and does not send a message', async () => {
  const calls: string[] = []
  let sends = 0
  const snapshot = { conversation_id: 'original', metadata: { title: '原会话', archived: true }, turns: [
    { turn_id: 'done', state: 'completed' }, { turn_id: 'active', state: 'running' },
  ], messages: [
    { id: 'earlier', seq: 1, role: 'assistant', turn_id: 'done', text: '中间说明' },
    { id: 'final', seq: 2, role: 'assistant', turn_id: 'done', text: '最终结果' },
    { id: 'partial', seq: 3, role: 'assistant', turn_id: 'active', text: '正在生成', metadata: { status: 'streaming' } },
  ] }
  const overrides = { snapshot, forkConversation: async (id: string) => { calls.push(id) }, send() { sends++ } }
  const ready = html(overrides)
  assert.equal((ready.match(/aria-label="从这里开分支"/g) ?? []).length, 1)
  assert.match(ready, /保留至这条回复的对话/)
  assert.doesNotMatch(ready, /disabled=""[^>]*aria-label="从这里开分支"/)
  assert.match(html({ ...overrides, sessionBusy: 'fork' }), /disabled=""[^>]*aria-label="从这里开分支"/)
  const render = component({ useRef: () => ({ current: null }), useEffect() {}, useLayoutEffect() {},
    useState: (initial: unknown) => [typeof initial === 'function' ? initial() : initial, () => {}] })
  type Node = { type?: string | ((props: unknown) => unknown); props?: Record<string, unknown> }
  function nodes(node: unknown): Node[] {
    if (Array.isArray(node)) return node.flatMap(nodes)
    if (!node || typeof node !== 'object') return []
    return [node as Node, ...nodes((node as Node).props?.children)]
  }
  const tree = render({ locale: 'zh', controller: controller(overrides) })
  const footer = nodes(tree).find(node => typeof node.type === 'function' && node.type.name === 'ReplyMeta' && node.props?.fork)!
  const buttons = (footer.type as (props: unknown) => unknown)(footer.props)
  const fork = nodes(buttons).find(node => node.props?.['aria-label'] === '从这里开分支')!
  await (fork.props!.onClick as () => Promise<void>)()
  assert.deepEqual(calls, ['final'])
  assert.equal(sends, 0)
})

test('a branch shows its source and returns to the original reply without submitting', async () => {
  const origin = { conversation_id: 'source', message_id: 'anchor', title: '地图检查', seq: 18 }
  const opened: unknown[] = []
  const current = controller({ snapshot: { conversation_id: 'branch', metadata: { title: '地图检查 · 分支', forked_from: origin },
    messages: [{ id: 'copied', seq: 1, turn_id: 'copy', role: 'assistant', text: '历史结果' }], turns: [{ turn_id: 'copy', state: 'completed' }] },
    openSearchResult: async (value: unknown) => { opened.push(value) },
  })
  const markup = html(current)
  assert.match(markup, /分支自/)
  assert.match(markup, /回到原会话/)
  const render = component({ useRef: () => ({ current: null }), useEffect() {}, useLayoutEffect() {},
    useState: (initial: unknown) => [typeof initial === 'function' ? initial() : initial, () => {}] })
  type Node = { type?: unknown; props?: Record<string, unknown> }
  function nodes(node: unknown): Node[] {
    if (Array.isArray(node)) return node.flatMap(nodes)
    if (!node || typeof node !== 'object') return []
    return [node as Node, ...nodes((node as Node).props?.children)]
  }
  const button = nodes(render({ locale: 'zh', controller: current })).find(node => node.type === 'button'
    && Array.isArray(node.props?.children) && node.props.children.includes('回到原会话'))!
  await (button.props!.onClick as () => Promise<void>)()
  assert.equal(JSON.stringify(opened), JSON.stringify([{ ...origin, role: 'assistant', snippet: '' }]))
})

test('an explicit historical quote remains visible without a copied reference message', () => {
  const markup = html({ snapshot: { conversation_id: 'branch', messages: [
    { id: 'question', seq: 1, turn_id: 'copy', role: 'user', text: '这句话是什么意思？' },
  ], turns: [{ turn_id: 'copy', state: 'completed', reply_quote: '保留下来的原文摘录' }] } })
  assert.match(markup, /aria-label="引用的回复"/)
  assert.match(markup, /保留下来的原文摘录/)
})

test('history search shows message snippets and opens a result without submitting a message', async () => {
  let searched = ''
  let opened: unknown = null
  let sent = 0
  const hit = { conversation_id: 'old', message_id: 'early', seq: 2, title: '旧地图讨论', role: 'assistant', snippet: '此前检查过点云间隙。' }
  const render = component({ useRef: () => ({ current: null }), useEffect() {}, useLayoutEffect() {},
    useState: (initial: unknown) => [typeof initial === 'function' ? initial() : initial, () => {}] })
  const tree = render({ locale: 'zh', controller: controller({ sessionListOpen: true, searchQuery: '间隙', searchResults: [hit],
    setSearchQuery: (value: string) => { searched = value }, openSearchResult: async (value: unknown) => { opened = value }, send() { sent++ },
  }) })
  type Node = { type?: string | ((props: unknown) => unknown); props?: Record<string, unknown> }
  function nodes(node: unknown): Node[] {
    if (Array.isArray(node)) return node.flatMap(nodes)
    if (!node || typeof node !== 'object') return []
    return [node as Node, ...nodes((node as Node).props?.children)]
  }
  const sessions = nodes(tree).find(node => typeof node.type === 'function' && node.type.name === 'ConversationSessions')!
  const contents = (sessions.type as (props: unknown) => unknown)(sessions.props)
  const input = nodes(contents).find(node => node.type === 'input' && node.props?.type === 'search')!
  ;(input.props!.onChange as (event: unknown) => void)({ target: { value: '点云' } })
  assert.equal(searched, '点云')
  const button = nodes(contents).find(node => node.type === 'button' && node.props?.className !== 'iconButton' && typeof node.props?.onClick === 'function'
    && nodes(node).some(child => child.type === 'strong' && child.props?.children === hit.title))!
  assert.ok(button)
  await (button.props!.onClick as () => Promise<void>)()
  assert.equal(opened, hit)
  assert.equal(sent, 0)
  const markup = html({ sessionListOpen: true, searchQuery: '间隙', searchResults: [hit] })
  assert.match(markup, /此前检查过点云间隙/)
  assert.match(markup, /搜索会话正文/)
  assert.match(html({ sessionListOpen: true, searchQuery: '无结果', searchResults: [] }), /没有找到匹配的消息/)
  assert.match(html({ sessionListOpen: true, searchQuery: '点云', searchError: '搜索服务不可用' }), /role="alert"[^>]*>搜索服务不可用/)
})

test('map context displays its version and selection and can be removed or attached again', () => {
  const context = { map_id: '办公室地图', content_epoch: 7, selection: { min: [1, 2, 3], max: [4, 5, 6] } }
  const markup = html({ context, includeMapContext: true, includeSelection: true })
  assert.match(markup, /本次消息的上下文/)
  assert.match(markup, /办公室地图/)
  assert.match(markup, /内容版本.*7/)
  assert.match(markup, /Min · 1.00, 2.00, 3.00/)
  assert.match(markup, /aria-label="移除地图上下文"/)
  assert.match(markup, /aria-label="移除框选上下文"/)
  assert.match(html({ context, includeMapContext: false, includeSelection: true }), /附加当前地图/)
  assert.match(html({ context, includeMapContext: true, includeSelection: false }), /附加框选范围/)
  const pending = html({ context, includeMapContext: true, includeSelection: true, pending: { message_id: 'pending' } })
  assert.match(pending, /disabled="" aria-label="移除地图上下文"/)
  assert.match(pending, /disabled="" aria-label="移除框选上下文"/)
})

test('a loaded search target receives focus and does not resume following new messages', () => {
  let refIndex = 0
  let focused = 0
  let scrolled = 0
  const effects: Array<() => void> = []
  const target = { dataset: { messageId: 'early' }, scrollIntoView() { scrolled++ }, focus() { focused++ } }
  const render = component({
    useState: (initial: unknown) => [typeof initial === 'function' ? initial() : initial, () => {}],
    useRef: (initial: unknown) => ({ current: refIndex++ === 1 ? { querySelectorAll: () => [target], addEventListener() {}, removeEventListener() {} } : initial }),
    useLayoutEffect() {}, useEffect: (effect: () => void) => effects.push(effect),
  })
  render({ locale: 'zh', controller: controller({ searchTarget: { conversationId: 'old', messageId: 'early', query: '地图' },
    snapshot: { conversation_id: 'old', messages: [{ id: 'early', seq: 1, role: 'assistant', turn_id: 'done', text: '地图结果' }], turns: [] },
  }) })
  effects.forEach(effect => effect())
  assert.equal(scrolled, 1)
  assert.equal(focused, 1)
})

test('only pending approvals show actions and decisions retain the approval identity', () => {
  const effects: Array<() => void> = []
  const decisions: unknown[][] = []
  const render = component({ useRef: () => ({ current: null }), useState: () => [false, () => {}], useLayoutEffect() {}, useEffect: (effect: () => void) => effects.push(effect) })
  const tree = render({ locale: 'zh', controller: controller({
    decideApproval: (...args: unknown[]) => decisions.push(args),
    snapshot: { messages: [], turns: [], approvals: [
      { approval_id: 'pending', tool_name: 'navigate_to', arguments: { x: 1, y: 2 }, state: 'pending' },
      { approval_id: 'old', tool_name: 'navigate_to', arguments: { x: 0 }, state: 'approved' },
    ] },
  }) })
  type Element = { type?: string; props?: { children?: unknown; onClick?: () => void; 'data-approval-id'?: string } }
  function elements(node: unknown): Element[] {
    if (Array.isArray(node)) return node.flatMap(elements)
    if (!node || typeof node !== 'object') return []
    const element = node as Element
    return [element, ...elements(element.props?.children)]
  }
  const cards = elements(tree).filter(element => element.props?.['data-approval-id'])
  assert.equal(cards.length, 2)
  assert.equal(elements(cards[1]).filter(element => element.type === 'button').length, 0)
  assert.equal(cards[0].props?.['data-approval-id'], 'pending')
  const buttons = elements(cards[0]).filter(element => element.type === 'button')
  buttons[0].props?.onClick?.()
  buttons[1].props?.onClick?.()
  assert.deepEqual(decisions, [['pending', false], ['pending', true]])
  effects.forEach(effect => effect())
  assert.deepEqual(decisions, [['pending', false], ['pending', true]])
  const waiting = html({ approvalBusy: 'pending', snapshot: { messages: [], turns: [], approvals: [
    { approval_id: 'pending', tool_name: 'navigate_to', arguments: { x: 1 }, state: 'pending' },
  ] } })
  assert.match(waiting, /前往目标位置/)
  assert.match(waiting, /x 1/)
  assert.match(waiting, /button[^>]*disabled=""[^>]*>拒绝/)
  assert.match(waiting, /button[^>]*disabled=""[^>]*>允许执行/)
})


test('unconfigured providers are visible but cannot be selected', () => {
  const markup = html({ options: { models: [
    { id: 'test', label: 'Test model', reasoning: [{ id: 'default', label: '默认' }] },
    { id: 'gpt-6.1-sol', label: 'GPT-6.1 Sol', provider: 'openai', configured: false,
      key_env: 'OPENAI_API_KEY', reasoning: [{ id: 'default', label: '默认' }] },
  ], attachments: { accept: '.txt', max_files: 4, max_file_bytes: 1000 } } })
  assert.match(markup, /GPT-6.1 Sol/)
  assert.match(markup, /alt="openai"/)
  assert.match(markup, /未配置/)
  assert.doesNotMatch(markup, /openai · 未配置/)
  assert.match(markup, /disabled="" title="服务端需配置 OPENAI_API_KEY"/)
})


test('clarification presents lettered choices and Other while retaining the question', () => {
  const markup = html({ snapshot: { messages: [{ id: 'q', seq: 1, turn_id: 't', role: 'assistant', kind: 'clarification',
    text: '补地面还是墙面？', metadata: { choices: ['地面', '墙面'] } }], turns: [] } })
  assert.match(markup, /补地面还是墙面？/)
  assert.match(markup, />A<\/span>/)
  assert.match(markup, />B<\/span>/)
  assert.match(markup, /其他，自己说/)
})

test('conversation rail links to user turns and tools display recorded duration', () => {
  const markup = html({ snapshot: { messages: [
    { id: 'u1', seq: 1, turn_id: 'one', role: 'user', text: '查询地图' },
    { id: 'tool', seq: 2, turn_id: 'one', role: 'tool', kind: 'list_saved_maps', text: '{"ok":true}', metadata: { status: 'completed', duration_ms: 1234 } },
    { id: 'u2', seq: 3, turn_id: 'two', role: 'user', text: '继续查看' },
  ], turns: [] } })
  assert.match(markup, /对话定位/)
  assert.match(markup, /跳到第 1 次提问：查询地图/)
  assert.match(markup, /跳到第 2 次提问：继续查看/)
  assert.match(markup, /1.2.*秒/)
})


test('connection cards distinguish configured models and missing server keys', () => {
  const markup = html({ snapshot: { messages: [{ id: 'connections', seq: 1, turn_id: 'one', role: 'tool', kind: 'get_model_connections',
    text: JSON.stringify({ models: [
      { id: 'kimi', label: 'Kimi', configured: true },
      { id: 'gpt', label: 'GPT', configured: false, key_env: 'OPENAI_API_KEY' },
    ] }), metadata: { status: 'completed', duration_ms: 23 } }], turns: [] }, setModel() {} })
  assert.match(markup, /已配置/)
  assert.match(markup, /未配置 · OPENAI_API_KEY/)
  assert.match(markup, /23 ms/)
  assert.equal((markup.match(/<summary>选择模型<\/summary>/g) ?? []).length, 1)
})


test('jumping to a question scrolls there without submitting or following the bottom again', () => {
  let refIndex = 0
  let jumps = 0
  let bottomScrolls = 0
  let sends = 0
  const effects: Array<() => void> = []
  const target = { dataset: { messageId: 'u1' }, scrollIntoView() { jumps++ } }
  const render = component({
    useState: (initial: unknown) => [typeof initial === 'function' ? initial() : initial, () => {}],
    useRef: (initial: unknown) => ({ current: refIndex++ === 1 ? { querySelectorAll: () => [target], addEventListener() {}, removeEventListener() {} }
      : refIndex === 3 ? { scrollIntoView() { bottomScrolls++ } } : initial }),
    useLayoutEffect() {}, useEffect: (effect: () => void) => effects.push(effect),
  })
  const tree = render({ locale: 'zh', controller: controller({ send() { sends++ }, snapshot: {
    conversation_id: 'jump-conversation', turns: [], messages: [
      { id: 'u1', seq: 1, turn_id: 'one', role: 'user', text: '第一问' },
      { id: 'u2', seq: 2, turn_id: 'two', role: 'user', text: '第二问' },
    ],
  } }) })
  type Element = { props?: { children?: unknown; 'aria-label'?: string; onClick?: () => void } }
  const walk = (node: unknown): Element[] => Array.isArray(node) ? node.flatMap(walk)
    : node && typeof node === 'object' ? [node as Element, ...walk((node as Element).props?.children)] : []
  const point = walk(tree).find(element => element.props?.['aria-label'] === '跳到第 1 次提问：第一问')
  assert.ok(point)
  effects.forEach(effect => effect())
  bottomScrolls = 0
  point.props?.onClick?.()
  effects.forEach(effect => effect())
  assert.equal(jumps, 1)
  assert.equal(sends, 0)
  assert.equal(bottomScrolls, 0)
})

test('map results expose real readiness without turning missing data into success', () => {
  const markup = html({ snapshot: { messages: [{ id: 'maps', seq: 1, role: 'tool', kind: 'list_saved_maps', turn_id: 't', text: JSON.stringify({ maps: [
    { name: 'map-ready', size_mb: 12.25, can_activate: true, record: { state: 'READY' } },
    { name: 'map-stale', can_activate: false, activation_blockers: ['missing octomap'], record: { state: 'STALE' } },
    { name: '<script>unknown</script>' },
  ] }) }], turns: [] } })
  assert.match(markup, /地图查询结果/)
  assert.match(markup, /12.3 MB/)
  assert.match(markup, /已过期/)
  assert.match(markup, /激活条件未知/)
  assert.match(markup, /missing octomap/)
  assert.doesNotMatch(markup, /<script>/)
})

test('approval receipt is permission only and save request shows source and destination', () => {
  const markup = html({ snapshot: { messages: [], turns: [{ turn_id: 't', state: 'running' }], approvals: [
    { approval_id: 'save', turn_id: 't', tool_name: 'save_map_copy', arguments: { map_id: 'original', name: 'copy', operations: [{ type: 'voxel' }] }, state: 'approved' },
  ] }, active: true })
  assert.match(markup, /原地图/)
  assert.match(markup, /另存为/)
  assert.match(markup, /已允许/)
  assert.doesNotMatch(markup, /允许执行|保存成功/)
})


test('robot observations show source units, partial fields and actual joint count', () => {
  const markup = html({ snapshot: { messages: [{ id: 'sensor', seq: 1, role: 'tool', kind: 'read_robot_sensor', turn_id: 'turn', text: JSON.stringify({
    success: true, sensor: 'joints', robot: 'go2', fresh: false, values: { joints: [
      { name: 'front-left', position_rad: .2, velocity_rad_s: .3 },
      { name: 'front-right', position_rad: .4, velocity_rad_s: .5 },
    ] },
  }) }], turns: [] } })
  assert.match(markup, /front-left/)
  assert.match(markup, /位置 rad/)
  assert.match(markup, /过期/)
  assert.match(markup, /—/)
  assert.doesNotMatch(markup, /joint-15/)
})


test('Python approval shows the code and execution environment before allowing it', () => {
  const markup = html({ snapshot: { messages: [], turns: [{ turn_id: 't', state: 'running' }], approvals: [
    { approval_id: 'python', turn_id: 't', tool_name: 'run_python', arguments: { code: 'print(6 * 7)' }, state: 'pending' },
  ] }, active: true })
  assert.match(markup, /print\(6 \* 7\)/)
  assert.match(markup, /不是隔离沙箱/)
  assert.match(markup, /允许执行/)
})

test('pending action strip lists only actual pending approvals without turning receipts into queued tasks', () => {
  const snapshot = { messages: [], turns: [{ turn_id: 't', state: 'running' }], approvals: [
    { approval_id: 'pending-save', turn_id: 't', tool_name: 'save_map_copy', arguments: {}, state: 'pending' },
    { approval_id: 'past', turn_id: 't', tool_name: 'navigate_to', arguments: {}, state: 'approved' },
  ] }
  const markup = html({ snapshot })
  assert.match(markup, /aria-label="待确认动作"/)
  const strip = markup.split('aria-label="待确认动作"')[1].split('</nav>')[0]
  assert.match(strip, /1 项待确认/)
  assert.match(strip, /另存地图/)
  assert.doesNotMatch(strip, /前往目标位置/)
  assert.match(markup, /aria-label="动作流程"/)
  assert.doesNotMatch(html({ snapshot: { ...snapshot, approvals: [snapshot.approvals[1]] } }), /aria-label="待确认动作"/)
})

test('specialist reports render returned evidence and do not mark cancelled reviews completed', () => {
  const markup = html({ active: true, snapshot: { messages: [
    { id: 'maps', seq: 1, role: 'tool', turn_id: 't', kind: 'specialist_review', metadata: { status: 'completed', duration_ms: 30 }, text: JSON.stringify({ specialist: '地图助手', task: '检查地图', summary: '发现一张过期地图', evidence: [{ tool: 'list_saved_maps', call_id: 'one' }] }) },
    { id: 'obs', seq: 2, role: 'tool', turn_id: 't', kind: 'specialist_review', metadata: { status: 'cancelled' }, text: JSON.stringify({ specialist: '观测助手', task: '检查观测', summary: '协作已中断' }) },
  ], turns: [{ turn_id: 't', state: 'running' }] } })
  assert.match(markup, /地图助手/)
  assert.match(markup, /发现一张过期地图/)
  assert.match(markup, /1 条工具回执/)
  assert.match(markup, /data-state="cancelled"/)
  assert.match(markup, /已中断/)
})


test('new sensor monitor tool results open once without replaying history or failures', () => {
  const seen = { current: null }
  const opened: string[] = []
  let refIndex = 0
  let effects: Array<() => void> = []
  const render = component({
    useRef: (initial: unknown) => refIndex++ === 0 ? seen : { current: initial },
    useState: () => [false, () => {}], useLayoutEffect() {},
    useEffect: (effect: () => void) => effects.push(effect),
  }, {
    '../hooks/useSensorMonitor.ts': { useSensorMonitor: () => (view: string) => opened.push(view) },
    '../hooks/useImuMonitor.ts': { useImuMonitor: () => () => opened.push('imu') },
  })
  const message = (id: string, monitor: string, success = true, status = 'completed') => ({ id, seq: 1, role: 'tool', kind: 'open_sensor_monitor', turn_id: 'turn', metadata: { status }, text: JSON.stringify({ success, monitor }) })
  function update(messages: ReturnType<typeof message>[]) {
    refIndex = 0; effects = []
    render({ locale: 'zh', controller: controller({ snapshot: { messages, turns: [], approvals: [] } }) })
    effects[0]()
  }
  const old = message('old', 'camera')
  update([old]); assert.deepEqual(opened, [])
  update([old, message('new', 'joints', true, 'running')]); assert.deepEqual(opened, [])
  update([old, message('new', 'joints')]); assert.deepEqual(opened, ['joints'])
  update([old, message('new', 'joints'), message('bad', 'camera', false), message('unknown', 'missing')]); assert.deepEqual(opened, ['joints'])
  update([old, message('new', 'joints'), message('imu', 'imu'), message('all', 'catalog')]); assert.deepEqual(opened, ['joints','imu','catalog'])
})
