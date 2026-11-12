import { conversationDependency } from './agentConversationFixtures.ts'
import assert from 'node:assert/strict'
import { readFileSync } from 'node:fs'
import test from 'node:test'
import { runInNewContext } from 'node:vm'
import { createRequire } from 'node:module'
import ts from 'typescript'
import type { ConversationAttachment, ConversationMessage, ConversationMetadata, ConversationOptions, ConversationSearchResult, ConversationSnapshot, ConversationSubmission } from '../src/services/agentConversation.ts'
import type { AgentMessageEvent } from '../src/types/index.ts'
import type { useAgentConversation } from '../src/hooks/useAgentConversation.ts'

const capabilities: ConversationOptions = {
  available: true, provider: 'configured', default_model: 'model-a', default_reasoning: 'default',
  default_permission_mode: 'default', permission_modes: ['read_only', 'default', 'full_access'],
  models: [
    { id: 'model-a', label: 'Model A', reasoning: [{ id: 'default', label: '默认' }, { id: 'high', label: '深入' }], image_input: true },
    { id: 'model-b', label: 'Model B', reasoning: [{ id: 'default', label: '默认' }], image_input: false },
  ], attachments: { accept: '.txt,.pdf,image/*', max_files: 4, max_file_bytes: 10 * 1024 * 1024 },
}

function compile(path: string) {
  return ts.transpileModule(readFileSync(new URL(path, import.meta.url), 'utf8'), {
    compilerOptions: { target: ts.ScriptTarget.ES2022, module: ts.ModuleKind.CommonJS, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
}

function componentDependency(name: string, fallback: NodeRequire) {
  if (name.endsWith('/services/agentConversation.ts')) return { conversationToolNames: {} }
  if (name.endsWith('/useMapSettings.ts')) return { useMapSettings: () => null }
  if (name.endsWith('/agentConversation/ToolActivity.tsx')) return {
    toolNames: {}, ToolActivity: ({ message }: { message: { text: string } }) => fallback('react').createElement('p', null, message.text),
  }
  return fallback(name)
}

test('older history cannot overwrite a revised live tool result', () => {
  const live = { id: 'tool-1', seq: 5, role: 'tool' as const, turn_id: 'turn-1', text: 'done', revision: 2 }
  const older = { ...live, text: 'running', revision: 1 }
  assert.equal(merge([live], [older])[0].text, 'done')
  assert.equal(merge([older], [live])[0].text, 'done')
})

const glyphExports: Record<string, unknown> = {}
runInNewContext(compile('../src/components/agentConversation/EffortGlyph.tsx'), { exports: glyphExports, require: createRequire(import.meta.url) })
const shareCardExports: Record<string, unknown> = {}
const shareRequire = createRequire(import.meta.url)
runInNewContext(compile('../src/components/ConversationShareCard.tsx'), { exports: shareCardExports,
  require: (name: string) => {
    const dependency = conversationDependency(name)
    if (dependency) return dependency
    if (name.endsWith('/Modal')) return { ModalShell: () => null }
    if (name.endsWith('/SuccessCheck')) return { SuccessCheck: () => null, PixelCheck: () => null }
    if (name.endsWith('/guide/markdown')) return { renderMarkdown: (text: string) => text }
    if (name.endsWith('/services/agentConversation')) return {}
    return name.endsWith('.css') ? { default: {} } : shareRequire(name)
  },
})

const markdownExports: Record<string, unknown> = {}
runInNewContext(compile('../src/guide/markdown.ts'), { exports: markdownExports })

class ConversationError extends Error {
  status: number
  constructor(message: string, status: number) { super(message); this.status = status }
}

function message(id: string, seq: number, role: ConversationMessage['role'] = 'user', run_id?: string): ConversationMessage {
  return { id, seq, role, text: id, turn_id: `turn-${id}`, run_id }
}

function forkSnapshots(archived = false) {
  const metadata: ConversationMetadata = { conversation_id: 'conversation-1', title: '原会话', created_at: 1, updated_at: 2, archived }
  const history = [
    { ...message('question', 1), turn_id: 'done', attachments: [{ id: 'history-file', name: '历史.txt', mime_type: 'text/plain', size: 5 }] },
    { ...message('answer', 2, 'assistant'), turn_id: 'done' },
  ]
  const source: ConversationSnapshot = { conversation_id: metadata.conversation_id, metadata,
    messages: [...history, { ...message('later', 3), turn_id: 'active' }, { ...message('tool', 4, 'tool'), turn_id: 'active' }],
    turns: [{ turn_id: 'done', state: 'completed' }, { turn_id: 'active', state: 'running', run_id: 'source-run' }],
    active_run: { run_id: 'source-run', instruction: '原任务继续执行' },
  }
  const branch: ConversationSnapshot = { conversation_id: 'branch-1', messages: history, turns: [{ turn_id: 'done', state: 'completed' }],
    metadata: { conversation_id: 'branch-1', title: '原会话 · 分支', created_at: 3, updated_at: 3, archived: false,
      forked_from: { conversation_id: metadata.conversation_id, message_id: 'answer', title: metadata.title, seq: 2 } },
  }
  return { source, branch }
}

const serviceExports: Record<string, unknown> = {}
runInNewContext(compile('../src/services/agentConversation.ts'), {
  exports: serviceExports, require: () => ({ dashboardFetch: () => { throw new Error('Unexpected request') } }),
  Error, Map, AbortSignal,
})
const merge = serviceExports.mergeConversationMessages as (previous: ConversationMessage[], incoming: ConversationMessage[]) => ConversationMessage[]

function harness(options: { storage?: Map<string, string>; snapshot?: ConversationSnapshot; failSend?: boolean; failFetch?: number; rejectSend?: number; failOptions?: boolean; mapSettings?: unknown; forkSnapshot?: ConversationSnapshot; forkError?: Error } = {}) {
  const storage = options.storage ?? new Map<string, string>()
  const slots: Array<{ value?: unknown; deps?: unknown[]; cleanup?: () => void }> = []
  const timers = new Map<number, () => Promise<void>>()
  const timerDelays = new Map<number, number>()
  let liveMessage: AgentMessageEvent | null = null
  let mapSettings = options.mapSettings ?? null
  let cursor = 0
  let dirty = false
  let renderingEnabled = true
  let timerId = 0
  let effects: Array<() => void> = []
  let nextId = 0
  let createCount = 0
  let fetchCount = 0
  let optionsCount = 0
  let uploadCount = 0
  let fetchGate: Promise<void> | null = null
  let forkGate: Promise<void> | null = null
  let sessionsGate: Promise<void> | null = null
  const forks: Array<{ id: string; messageId: string }> = []
  const sessionReads: boolean[] = []
  const approvals: Array<{ id: string; approvalId: string; approved: boolean }> = []
  let currentSnapshot = options.snapshot ?? { conversation_id: 'conversation-1', messages: [], turns: [] }
  const snapshots = new Map<string, ConversationSnapshot>([[currentSnapshot.conversation_id, currentSnapshot]])
  const historyPages = new Map<string, ConversationSnapshot>()
  const historyReads: Array<{ id: string; beforeSeq: number }> = []
  const searches: Array<{ query: string; archived: boolean }> = []
  let searchHandler: (query: string, archived: boolean) => Promise<ConversationSearchResult[]> = async () => []
  const setSnapshot = (snapshot: ConversationSnapshot) => {
    currentSnapshot = snapshot
    snapshots.set(snapshot.conversation_id, snapshot)
  }
  const metadata = (snapshot: ConversationSnapshot) => snapshot.metadata ?? {
    conversation_id: snapshot.conversation_id, title: '新对话', created_at: 1, updated_at: 1, archived: false,
  }
  let failSend = options.failSend ?? false
  let failFetch = options.failFetch ?? 0
  const sends: Array<{ id: string; message: ConversationSubmission }> = []
  const observed: string[] = []
  const observer = (id: string) => { observed.push(id) }
  const react = {
    useState(initial: unknown) {
      const slot = cursor++
      slots[slot] ??= { value: typeof initial === 'function' ? initial() : initial }
      return [slots[slot].value, (value: unknown) => { slots[slot].value = value; dirty = true }]
    },
    useRef(initial: unknown) {
      const slot = cursor++
      slots[slot] ??= { value: { current: initial } }
      return slots[slot].value
    },
    useCallback(callback: unknown, deps: unknown[]) {
      const slot = cursor++
      const previous = slots[slot]
      if (!previous?.deps || deps.some((value, index) => !Object.is(value, previous.deps?.[index]))) slots[slot] = { value: callback, deps }
      return slots[slot].value
    },
    useMemo(factory: () => unknown, deps: unknown[]) {
      const slot = cursor++
      const previous = slots[slot]
      if (!previous?.deps || deps.some((value, index) => !Object.is(value, previous.deps?.[index]))) slots[slot] = { value: factory(), deps }
      return slots[slot].value
    },
    useEffect(effect: () => (() => void) | undefined, deps: unknown[]) {
      const slot = cursor++
      const previous = slots[slot]
      if (!previous?.deps || deps.some((value, index) => !Object.is(value, previous.deps?.[index]))) effects.push(() => {
        previous?.cleanup?.()
        slots[slot] = { deps, cleanup: effect() }
      })
    },
  }
  const service = {
    ConversationError,
    isConversationAttachment: serviceExports.isConversationAttachment,
    isConversationPermission: serviceExports.isConversationPermission,
    fetchConversationOptions: async () => {
      optionsCount++
      if (options.failOptions) throw new ConversationError('Agent 未启用，当前无法对话。', 503)
      return capabilities
    },
    uploadConversationAttachment: async (_id: string, file: File, id: string): Promise<ConversationAttachment> => {
      uploadCount++
      return { id, name: file.name, mime_type: file.type, size: file.size }
    },
    mergeConversationMessages: merge,
    parseConversationMessageEvent: serviceExports.parseConversationMessageEvent,
    searchConversations: async (query: string, archived: boolean) => {
      searches.push({ query, archived })
      return searchHandler(query, archived)
    },
    createConversation: async () => {
      const id = `conversation-${++createCount}`
      if (!snapshots.has(id)) snapshots.set(id, { conversation_id: id, messages: [], turns: [] })
      return id
    },
    forkConversation: async (id: string, messageId: string) => {
      forks.push({ id, messageId })
      if (forkGate) await forkGate
      if (options.forkError) throw options.forkError
      const branch = options.forkSnapshot ?? { conversation_id: 'branch-1', messages: [], turns: [] }
      snapshots.set(branch.conversation_id, branch)
      return metadata(branch)
    },
    listConversations: async (archived: boolean) => {
      sessionReads.push(archived)
      const items = [...snapshots.values()].map(metadata).filter(item => item.archived === archived)
      if (sessionsGate) await sessionsGate
      return items
    },
    updateConversation: async (id: string, changes: object) => {
      const snapshot = snapshots.get(id)!
      const updated = { ...metadata(snapshot), ...changes }
      setSnapshot({ ...snapshot, metadata: updated })
      return updated
    },
    createConversationShare: async (id: string) => {
      const snapshot = snapshots.get(id)!
      const share = { token: `share-${id}`, created_at: 1, message_count: snapshot.messages.length }
      setSnapshot({ ...snapshot, share })
      return share
    },
    deleteConversationShare: async (id: string) => { setSnapshot({ ...snapshots.get(id)!, share: null }) },
    fetchConversation: async (id: string, beforeSeq?: number) => {
      fetchCount++
      if (beforeSeq != null) historyReads.push({ id, beforeSeq })
      const snapshot = beforeSeq == null ? snapshots.get(id)! : historyPages.get(`${id}:${beforeSeq}`)!
      if (fetchGate) await fetchGate
      if (failFetch) { const status = failFetch; failFetch = 0; throw new ConversationError('Agent 未启用', status) }
      return snapshot
    },
    resolveConversationApproval: async (id: string, approvalId: string, approved: boolean) => {
      approvals.push({ id, approvalId, approved })
      const snapshot = snapshots.get(id)!
      const approval = snapshot.approvals!.find(item => item.approval_id === approvalId)!
      const updated = { ...approval, state: approved ? 'approved' as const : 'denied' as const }
      setSnapshot({ ...snapshot, approvals: [updated] })
      return updated
    },
    sendConversationMessage: async (id: string, pending: ConversationSubmission) => {
      sends.push({ id, message: { ...pending } })
      if (options.rejectSend) throw new ConversationError('turn_active', options.rejectSend)
      if (failSend) throw new Error('HTTP timeout')
      const turn = { turn_id: `turn-${pending.message_id}`, state: 'running' }
      const snapshot = snapshots.get(id)!
      setSnapshot({ ...snapshot,
        messages: [...snapshot.messages, { ...message(pending.message_id, snapshot.messages.length + 1), text: pending.text }],
        turns: [turn as ConversationSnapshot['turns'][number]],
      })
      return turn
    },
  }
  const exports: Record<string, unknown> = {}
  const profileExports: Record<string, unknown> = {}
  const browserStorage = { getItem: (key: string) => storage.get(key) ?? null, setItem: (key: string, value: string) => storage.set(key, value) }
  runInNewContext(compile('../src/services/botProfile.ts'), { exports: profileExports, localStorage: browserStorage })
  runInNewContext(compile('../src/hooks/useAgentConversation.ts'), {
    exports, require: (name: string) => name.endsWith('/services/botProfile.ts') ? profileExports : name === 'react' ? react : name.endsWith('/useMapSettings.ts') ? { useMapSettings: () => mapSettings } : name.endsWith('/api')
      ? { makeRequestId: () => `message-${++nextId}` } : service,
    localStorage: { getItem: (key: string) => storage.get(key) ?? null, setItem: (key: string, value: string) => storage.set(key, value) },
    location: { origin: 'http://robot.test' }, Error, Set, JSON,
    setTimeout: (callback: () => Promise<void>, delay: number) => { timers.set(++timerId, callback); timerDelays.set(timerId, delay); return timerId },
    clearTimeout: (id: number) => { timers.delete(id); timerDelays.delete(id) },
  })
  const runHook = exports.useAgentConversation as (observer: (id: string) => void, enabled: boolean, liveMessage: AgentMessageEvent | null) => ReturnType<typeof useAgentConversation>
  const render = (enabled = true, event = liveMessage) => {
    liveMessage = event
    renderingEnabled = enabled
    cursor = 0
    effects = []
    const result = runHook(observer, enabled, liveMessage)
    for (const effect of effects) effect()
    return result
  }
  const flush = async () => {
    for (let i = 0; i < 32; i++) {
      await Promise.resolve()
      if (dirty) { dirty = false; render(renderingEnabled) }
    }
  }
  return { render, flush, storage, sends, observed, timers, approvals, searches, historyReads, forks, sessionReads,
    setFetchGate: (gate: Promise<void> | null) => { fetchGate = gate },
    setForkGate: (gate: Promise<void> | null) => { forkGate = gate },
    setSessionsGate: (gate: Promise<void> | null) => { sessionsGate = gate },
    setFailFetch: (status: number) => { failFetch = status },
    setSnapshot,
    setMapSettings: (value: unknown) => { mapSettings = value },
    setHistoryPage: (id: string, cursor: number, page: ConversationSnapshot) => { historyPages.set(`${id}:${cursor}`, page) },
    setSearchHandler: (handler: typeof searchHandler) => { searchHandler = handler },
    setFailSend: (fail: boolean) => { failSend = fail },
    counts: () => ({ createCount, fetchCount, optionsCount, uploadCount }),
    tick: async () => { const next = timers.entries().next().value; assert.ok(next); timers.delete(next[0]); await next[1]() },
    tickSearch: async () => {
      const timer = [...timers].find(([id]) => timerDelays.get(id) === 250)
      assert.ok(timer); timers.delete(timer[0]); timerDelays.delete(timer[0]); await timer[1]()
    },
  }
}

test('messages merge by id and sort by server sequence without timestamp deduplication', () => {
  const merged = merge([message('b', 2), message('a', 1)], [message('b', 2, 'assistant'), message('c', 3)])
  assert.deepEqual(Array.from(merged, item => item.id), ['a', 'b', 'c'])
  assert.equal(merged[1].role, 'assistant')
})

test('HTTP service sends the original id and validates conversation identity and message roles', async () => {
  const calls: Array<{ path: string; body?: string }> = []
  let responseBody: object = { conversation_id: 'conversation/1', message_id: 'request-1', turn_id: 'turn-1', state: 'running' }
  let responseStatus = 202
  const exports: Record<string, unknown> = {}
  runInNewContext(compile('../src/services/agentConversation.ts'), {
    exports, require: () => ({ dashboardFetch: async (path: string, init: RequestInit) => {
      calls.push({ path, body: init.body as string | undefined })
      return new Response(JSON.stringify(responseBody), { status: responseStatus })
    } }), Error, Map, AbortSignal,
  })
  const send = exports.sendConversationMessage as (id: string, pending: ConversationSubmission) => Promise<unknown>
  const fetchSnapshot = exports.fetchConversation as (id: string) => Promise<unknown>
  await send('conversation/1', { message_id: 'request-1', text: '你好' })
  assert.equal(calls[0].path, '/api/v1/agent/conversations/conversation%2F1/messages')
  assert.deepEqual(JSON.parse(calls[0].body!), { message_id: 'request-1', text: '你好' })
  responseStatus = 200
  responseBody = { conversation_id: 'other', messages: [], turns: [] }
  await assert.rejects(fetchSnapshot('conversation/1'), /快照格式无效/)
  responseBody = { conversation_id: 'conversation/1', messages: [{ ...message('bad', 1), role: 'invented' }], turns: [] }
  await assert.rejects(fetchSnapshot('conversation/1'), /快照格式无效/)
  responseStatus = 503
  responseBody = { detail: { reason: 'agent_unavailable' } }
  await assert.rejects(fetchSnapshot('conversation/1'), /Agent 未启用/)
  responseStatus = 403
  responseBody = { error: 'map_edit_only', message: 'Only saved-map editing is available here' }
  await assert.rejects(fetchSnapshot('conversation/1'), /当前工作区仅支持地图编辑，未启用对话/)
})

test('fork service posts the selected message and validates new metadata and its origin', async () => {
  const calls: Array<{ path: string; body: string }> = []
  const valid: ConversationMetadata = { conversation_id: 'new', title: '新分支', created_at: 2, updated_at: 2, archived: false,
    forked_from: { conversation_id: 'source/1', message_id: 'answer/2', title: '原会话', seq: 2 } }
  let response: object = valid
  const exports: Record<string, unknown> = {}
  runInNewContext(compile('../src/services/agentConversation.ts'), {
    exports, require: () => ({ dashboardFetch: async (path: string, init: RequestInit) => {
      calls.push({ path, body: init.body as string }); return new Response(JSON.stringify(response), { status: 201 })
    } }), Error, Map, AbortSignal,
  })
  const fork = exports.forkConversation as (id: string, messageId: string) => Promise<ConversationMetadata>
  assert.equal((await fork('source/1', 'answer/2')).conversation_id, 'new')
  assert.equal(calls[0].path, '/api/v1/agent/conversations/source%2F1/fork')
  assert.deepEqual(JSON.parse(calls[0].body), { message_id: 'answer/2' })
  for (const invalid of [{ ...valid, conversation_id: 'source/1' }, { ...valid, archived: true },
    { ...valid, forked_from: { ...valid.forked_from, conversation_id: 'other' } },
    { ...valid, forked_from: { ...valid.forked_from, message_id: 'other' } },
    { ...valid, forked_from: { ...valid.forked_from, seq: 0 } }]) {
    response = invalid
    await assert.rejects(fork('source/1', 'answer/2'), /创建结果未确认/)
  }
  response = { ...valid, forked_from: undefined }
  await assert.rejects(fork('source/1', 'answer/2'), /创建结果未确认/)
})

test('fork keeps the source draft and active work while opening an empty draft with inherited model settings', async () => {
  const { source, branch } = forkSnapshots()
  const h = harness({ snapshot: source, forkSnapshot: branch })
  h.render().setOpen(true); await h.flush()
  h.render().setDraft('原会话未发草稿')
  h.render().setReplyTo('answer', '保留引用')
  h.render().setReasoning('high')
  h.render().setPermissionMode('full_access')
  h.render().setIncludeMapContext(false)
  h.render().setIncludeSelection(false)
  await h.render().addAttachments([{ name: 'draft.txt', size: 7, type: 'text/plain' } as File])
  await h.render().forkConversation('answer')
  assert.equal(h.render().snapshot?.conversation_id, 'branch-1')
  assert.equal(h.render().snapshot?.metadata?.forked_from?.message_id, 'answer')
  assert.equal(h.render().draft, '')
  assert.equal(h.render().pending, null)
  assert.equal(h.render().replyTo, '')
  assert.equal(h.render().attachments.length, 0)
  assert.equal(h.render().snapshot?.messages[0].attachments?.[0].id, 'history-file')
  assert.equal(h.render().model, 'model-a')
  assert.equal(h.render().reasoning, 'high')
  assert.equal(h.render().permissionMode, 'default')
  assert.equal(h.sends.length, 0)
  assert.equal(h.approvals.length, 0)
  await h.render().switchConversation('conversation-1')
  assert.equal(h.render().draft, '原会话未发草稿')
  assert.equal(h.render().replyQuote, '保留引用')
  assert.equal(h.render().attachments[0].name, 'draft.txt')
  assert.equal(h.render().permissionMode, 'full_access')
  assert.equal(h.render().includeMapContext, false)
  assert.equal(h.render().includeSelection, false)
  assert.equal(h.render().snapshot?.active_run?.run_id, 'source-run')
  assert.equal(h.render().snapshot?.turns.find(turn => turn.turn_id === 'active')?.state, 'running')
})

test('fork blocks duplicate creation and isolates the branch from old source snapshots and live messages', async () => {
  const { source, branch } = forkSnapshots()
  const h = harness({ snapshot: source, forkSnapshot: branch })
  h.render().setOpen(true); await h.flush()
  let finishRead!: () => void
  h.setFetchGate(new Promise(resolve => { finishRead = resolve }))
  const oldRead = h.render().refresh()
  h.setFetchGate(null)
  let finishFork!: () => void
  h.setForkGate(new Promise(resolve => { finishFork = resolve }))
  const first = h.render().forkConversation('answer')
  assert.equal(h.render().sessionBusy, 'fork')
  await assert.rejects(h.render().forkConversation('answer'), /等待当前操作/)
  finishFork(); await first
  finishRead(); await oldRead
  h.render(true, { type: 'agent_message', ts: 5, conversation_id: 'conversation-1', id: 'late', turn_id: 'active', seq: 99, revision: 10, role: 'assistant', text: '原会话迟到输出', metadata: { status: 'streaming' } })
  assert.equal(h.forks.length, 1)
  assert.equal(h.render().snapshot?.conversation_id, 'branch-1')
  assert.equal(h.render().snapshot?.messages.some(message => message.id === 'late'), false)
  assert.equal(h.render().error, '')
  assert.equal(h.sends.length, 0)
})

test('fork rejects non-final targets and an unresolved submitted message without making a request', async () => {
  const { source } = forkSnapshots()
  const h = harness({ snapshot: { ...source, messages: [
    { ...message('earlier', 1, 'assistant'), turn_id: 'done' },
    { ...message('answer', 2, 'assistant'), turn_id: 'done' },
    { ...message('pending-answer', 3, 'assistant'), turn_id: 'active' },
  ] }, failSend: true })
  h.render().setOpen(true); await h.flush()
  await h.render().forkConversation('earlier')
  await h.render().forkConversation('pending-answer')
  await h.render().forkConversation('missing')
  assert.equal(h.forks.length, 0)
  h.render().setDraft('尚未确认的提交'); await h.render().send()
  const pending = h.render().pending
  await h.render().forkConversation('answer')
  assert.equal(h.render().pending?.message_id, pending?.message_id)
  assert.equal(h.forks.length, 0)
  assert.match(h.render().error, /待重试消息/)
})

test('fork waits for attachment uploads and approval submissions before changing sessions', async () => {
  for (const operation of ['upload', 'approval']) {
    const { source, branch } = forkSnapshots()
    const h = harness({ snapshot: { ...source, approvals: [
      { approval_id: 'approval', turn_id: 'active', tool_name: 'save_map_copy', arguments: {}, state: 'pending' },
    ] }, forkSnapshot: branch })
    h.render().setOpen(true); await h.flush()
    const pending = operation === 'upload'
      ? h.render().addAttachments([{ name: 'draft.txt', size: 7, type: 'text/plain' } as File])
      : h.render().decideApproval('approval', true)
    await assert.rejects(h.render().forkConversation('answer'), /等待当前操作/)
    await pending
    assert.equal(h.forks.length, 0)
    assert.equal(h.render().snapshot?.conversation_id, 'conversation-1')
  }
})

test('failed fork creation or initial reading keeps the source draft and never retries creation', async () => {
  const { source, branch } = forkSnapshots()
  for (const forkError of [new Error('HTTP timeout'), undefined]) {
    const h = harness({ snapshot: source, forkSnapshot: branch, forkError })
    h.render().setOpen(true); await h.flush()
    h.render().setDraft('保留原稿')
    if (!forkError) h.setFailFetch(503)
    await assert.rejects(h.render().forkConversation('answer'), forkError ? /HTTP timeout/ : /分支已创建.*原会话已保留/)
    await h.flush()
    assert.equal(h.render().snapshot?.conversation_id, 'conversation-1')
    assert.equal(h.render().draft, '保留原稿')
    assert.equal(h.forks.length, 1)
    assert.ok(h.render().error)
    assert.equal(h.render().sessionBusy, null)
  }
})

test('fork from an archived source loads the active list and discards the old archive response', async () => {
  const { source, branch } = forkSnapshots(true)
  const h = harness({ snapshot: source, forkSnapshot: branch })
  h.render().setOpen(true); await h.flush()
  h.render().setDraft('归档会话的保留草稿')
  let finishArchive!: () => void
  h.setSessionsGate(new Promise(resolve => { finishArchive = resolve }))
  h.render().setArchivedView(true)
  h.render().setSessionListOpen(true)
  await h.flush()
  h.setSessionsGate(null)
  await h.render().forkConversation('answer')
  finishArchive(); await h.flush()
  assert.equal(h.render().archivedView, false)
  assert.equal(h.sessionReads.at(-1), false)
  assert.equal(h.render().conversations.length, 1)
  assert.equal(h.render().conversations[0].conversation_id, 'branch-1')
  assert.equal(h.render().snapshot?.metadata?.archived, false)
  await h.render().switchConversation('conversation-1')
  assert.equal(h.render().snapshot?.metadata?.archived, true)
  assert.equal(h.render().draft, '归档会话的保留草稿')
})

test('search service encodes literal queries and validates public message hits', async () => {
  const calls: string[] = []
  let result: unknown = { results: [{ conversation_id: 'one', title: '旧讨论', message_id: 'answer', seq: 9, role: 'assistant', snippet: '点云 50%_ 间隙' }] }
  const exports: Record<string, unknown> = {}
  runInNewContext(compile('../src/services/agentConversation.ts'), {
    exports, require: () => ({ dashboardFetch: async (path: string) => {
      calls.push(path); return new Response(JSON.stringify(result), { status: 200 })
    } }), Error, Map, AbortSignal,
  })
  const search = exports.searchConversations as (query: string, archived?: boolean) => Promise<ConversationSearchResult[]>
  assert.equal((await search(' 点云 50%_ ', true))[0].message_id, 'answer')
  assert.equal(calls[0], `/api/v1/agent/conversations/search?q=${encodeURIComponent('点云 50%_')}&archived=true`)
  assert.equal((await search('   ')).length, 0)
  await assert.rejects(search('a'.repeat(201)), /200/)
  assert.equal(calls.length, 1)
  result = { results: [{ conversation_id: 'one', title: '旧讨论', message_id: 'tool', seq: 10, role: 'tool', snippet: 'internal' }] }
  await assert.rejects(search('internal'), /搜索结果格式无效/)
})

test('live messages retain the newest revision and cannot cross sessions or invent a finished turn', async () => {
  const original = { ...message('answer', 2, 'assistant'), turn_id: 'running', revision: 1, text: '初始', metadata: { status: 'streaming' } }
  const snapshot: ConversationSnapshot = { conversation_id: 'conversation-1', messages: [original], turns: [{ turn_id: 'running', state: 'running' }] }
  const h = harness({ snapshot })
  h.render().setOpen(true); await h.flush()
  const event: AgentMessageEvent = { ...original, type: 'agent_message', ts: 1, conversation_id: 'conversation-1', revision: 2, text: '真实增量内容' }
  h.render(true, event)
  assert.equal(h.render().snapshot?.messages[0].text, '真实增量内容')
  h.render(true, { ...event, revision: 1, text: '迟到' })
  await h.render().refresh()
  assert.equal(h.render().snapshot?.messages[0].text, '真实增量内容')
  h.render(true, { ...event, revision: 3, text: '完整回答', metadata: { status: 'completed' }, run_id: 'unconfirmed-run' })
  assert.equal(h.render().snapshot?.messages[0].text, '完整回答')
  assert.equal(h.render().active, true)
  assert.deepEqual(h.observed, [])
  h.setSnapshot({ ...snapshot, messages: [{ ...original, revision: 3, text: '完整回答', metadata: { status: 'completed' } }], turns: [{ turn_id: 'running', state: 'completed' }] })
  await h.render().refresh()
  h.render(true, { ...event, revision: 4, text: '迟到的生成片段' })
  assert.equal(h.render().snapshot?.messages[0].text, '完整回答')
  await h.render().createNewConversation()
  h.render(true, { ...event, revision: 99 })
  assert.equal(h.render().snapshot?.messages.length, 0)
  h.render(false, { ...event, conversation_id: 'conversation-2', revision: 100 })
  assert.equal(h.render(false).snapshot?.messages.length, 0)
})

test('malformed and legacy live events are ignored at the message boundary', () => {
  const parse = serviceExports.parseConversationMessageEvent as (value: unknown) => ConversationMessage | null
  const valid = { type: 'agent_message', conversation_id: 'one', ...message('answer', 2, 'assistant'), revision: 1 }
  assert.equal(parse(valid)?.id, 'answer')
  assert.equal(parse({ ...valid, revision: 0 }), null)
  assert.equal(parse({ ...valid, metadata: [] }), null)
  assert.equal(parse({ ...valid, role: 'thinking' }), null)
  assert.equal(parse({ type: 'agent_message', role: 'assistant', text: 'legacy' }), null)
})

test('search discards an old response and reloads the selected archive view', async () => {
  const h = harness()
  h.render().setSessionListOpen(true); await h.flush()
  let finishOld!: (results: ConversationSearchResult[]) => void
  h.setSearchHandler(async query => query === '旧' ? new Promise(resolve => { finishOld = resolve })
    : [{ conversation_id: 'conversation-1', title: '当前', message_id: 'new', seq: 2, role: 'assistant', snippet: query }])
  h.render().setSearchQuery('旧'); await h.flush()
  const old = h.tickSearch()
  h.render().setSearchQuery('点云'); await h.flush(); await h.tickSearch()
  finishOld([{ conversation_id: 'conversation-1', title: '迟到', message_id: 'old', seq: 1, role: 'user', snippet: '旧' }])
  await old
  assert.equal(h.render().searchResults[0].message_id, 'new')
  h.render().setArchivedView(true); await h.flush(); await h.tickSearch()
  assert.equal(h.searches.at(-1)?.archived, true)
  h.render().setSearchQuery(''); await h.flush()
  assert.equal(h.render().searchResults.length, 0)
  assert.equal(h.render().searchBusy, false)
})

test('opening an old search hit loads history pages and preserves the draft without sending', async () => {
  const h = harness({ snapshot: { conversation_id: 'conversation-1', messages: [message('latest', 501)], turns: [], history_cursor: 501 } })
  h.setHistoryPage('conversation-1', 501, { conversation_id: 'conversation-1', messages: [message('middle', 301)], turns: [], history_cursor: 301 })
  h.setHistoryPage('conversation-1', 301, { conversation_id: 'conversation-1', messages: [message('older', 20, 'assistant')], turns: [], history_cursor: null })
  h.render().setOpen(true); await h.flush()
  h.render().setDraft('尚未发送的草稿')
  h.render().setSearchQuery('旧结论'); await h.flush()
  await h.render().openSearchResult({ conversation_id: 'conversation-1', title: '旧讨论', message_id: 'older', seq: 20, role: 'assistant', snippet: '旧结论' })
  assert.equal(h.render().searchTarget?.messageId, 'older')
  assert.equal(h.render().searchTarget?.query, '旧结论')
  assert.deepEqual(h.historyReads.map(item => item.beforeSeq), [501, 301])
  assert.equal(h.render().draft, '尚未发送的草稿')
  assert.equal(h.sends.length, 0)
})

test('search navigation waits for an in-flight history page instead of abandoning its target', async () => {
  const h = harness({ snapshot: { conversation_id: 'conversation-1', messages: [message('latest', 501)], turns: [], history_cursor: 501 } })
  h.setHistoryPage('conversation-1', 501, { conversation_id: 'conversation-1', messages: [message('older', 20, 'assistant')], turns: [], history_cursor: null })
  h.render().setOpen(true); await h.flush()
  let finish!: () => void
  h.setFetchGate(new Promise(resolve => { finish = resolve }))
  const reading = h.render().loadHistory()
  const navigating = h.render().openSearchResult({ conversation_id: 'conversation-1', title: '旧讨论', message_id: 'older', seq: 20, role: 'assistant', snippet: '旧结论' })
  await h.flush()
  assert.equal(h.render().searchTarget, null)
  finish(); await Promise.all([reading, navigating])
  assert.equal(h.render().searchTarget?.messageId, 'older')
  assert.deepEqual(h.historyReads.map(item => item.beforeSeq), [501])
})

test('context inclusion belongs to each draft and uncertain retries keep the original map snapshot', async () => {
  const mapSettings = { map: { name: 'map-a', contentEpoch: 3 }, selection: { min: { x: 0, y: 1, z: 2 }, max: { x: 3, y: 4, z: 5 } } }
  const h = harness({ mapSettings, failSend: true })
  h.render().setOpen(true); await h.flush()
  h.render().setIncludeSelection(false)
  h.render().setDraft('检查当前地图'); await h.render().send()
  assert.equal(h.sends[0].message.map_context?.map_id, 'map-a')
  assert.equal(h.sends[0].message.map_context?.selection, null)
  h.setMapSettings({ ...mapSettings, map: { name: 'map-b', contentEpoch: 4 } })
  h.render().setIncludeMapContext(false)
  assert.equal(h.render().includeMapContext, true)
  await h.render().send()
  assert.deepEqual(h.sends[1].message.map_context, h.sends[0].message.map_context)
  await h.render().createNewConversation()
  assert.equal(h.render().includeSelection, true)
  h.render().setIncludeMapContext(false)
  h.render().setDraft('仅解释文字'); await h.render().send()
  assert.equal(h.sends[2].message.map_context, null)
  await h.render().switchConversation('conversation-1')
  assert.equal(h.render().includeSelection, false)
  assert.equal(h.render().includeMapContext, true)
  const restored = harness({ storage: h.storage, mapSettings })
  restored.render(); await restored.flush()
  assert.equal(restored.render().includeSelection, false)
  assert.equal(restored.render().pending?.map_context?.content_epoch, 3)
})

test('multiple turns share one conversation and refresh restores server history', async () => {
  const h = harness()
  h.render().setDraft('你好')
  await h.flush()
  await h.render().send()
  h.setSnapshot({ conversation_id: 'conversation-1', messages: [message('message-1', 1), message('answer', 2, 'assistant')],
    turns: [{ turn_id: 'turn-message-1', state: 'completed', kind: 'reply' }] })
  await h.render().refresh()
  h.render().setDraft('接着说')
  await h.render().send()
  assert.deepEqual(h.sends.map(item => item.id), ['conversation-1', 'conversation-1'])
  assert.equal(h.counts().createCount, 1)
  const restored = harness({ storage: h.storage, snapshot: h.render().snapshot! })
  restored.render()
  await restored.flush()
  assert.equal(restored.counts().createCount, 0)
  assert.equal(restored.render().snapshot?.messages.length, 3)
})

test('failed sends preserve draft and request id across retry and page refresh', async () => {
  const h = harness({ failSend: true })
  h.render().setDraft('去门口')
  await h.flush()
  await h.render().send()
  const pending = h.render().pending!
  assert.equal(h.render().draft, '去门口')
  assert.match(h.render().error, /HTTP timeout/)
  await h.render().refresh()
  assert.match(h.render().error, /HTTP timeout/)
  h.render().setDraft('修改内容')
  assert.equal(h.render().draft, '去门口')
  const restored = harness({ storage: h.storage, failSend: true })
  restored.render()
  await restored.flush()
  await restored.render().send()
  assert.equal(restored.sends[0].message.message_id, pending.message_id)
  assert.equal(restored.render().draft, '去门口')
  h.setFailSend(false)
  await h.render().send()
  assert.equal(h.sends[0].message.message_id, h.sends[1].message.message_id)
  assert.equal(h.render().draft, '')
  assert.equal(h.render().pending, null)
})

test('quoted follow-ups retain their target through refresh and retry without copying the reply into the draft', async () => {
  const snapshot: ConversationSnapshot = { conversation_id: 'conversation-1', messages: [message('answer', 1, 'assistant')], turns: [] }
  const h = harness({ snapshot, failSend: true })
  h.render().setOpen(true)
  await h.flush()
  h.render().setDraft('补面具体怎么做？')
  h.render().setReplyTo('answer', '只引用这句话')
  assert.equal(h.render().draft, '补面具体怎么做？')
  const restored = harness({ storage: h.storage, snapshot, failSend: true })
  restored.render()
  await restored.flush()
  assert.equal(restored.render().replyTo, 'answer')
  assert.equal(restored.render().replyQuote, '只引用这句话')
  await restored.render().send()
  const pending = restored.render().pending!
  assert.equal(pending.reply_to_message_id, 'answer')
  assert.equal(pending.reply_quote, '只引用这句话')
  assert.equal(pending.text, '补面具体怎么做？')
  restored.render().setReplyTo('different')
  assert.equal(restored.render().replyTo, 'answer')
  const retry = harness({ storage: restored.storage, snapshot })
  retry.render()
  await retry.flush()
  assert.equal(retry.render().pending?.reply_quote, '只引用这句话')
  await retry.render().send()
  assert.equal(retry.sends[0].message.message_id, pending.message_id)
  assert.equal(retry.sends[0].message.reply_to_message_id, 'answer')
  assert.equal(retry.sends[0].message.reply_quote, '只引用这句话')
  assert.equal(retry.render().replyTo, '')
  assert.equal(retry.render().replyQuote, '')
  assert.equal(retry.render().draft, '')
})

test('removing a quote keeps the question and sends without a reply target', async () => {
  const h = harness()
  h.render().setDraft('继续解释一下')
  await h.flush()
  h.render().setReplyTo('answer', '选中的段落')
  h.render().setReplyTo('')
  assert.equal(h.render().replyQuote, '')
  assert.equal(h.render().draft, '继续解释一下')
  await h.render().send()
  assert.equal(h.sends[0].message.reply_to_message_id, undefined)
  assert.equal(h.sends[0].message.reply_quote, undefined)
})

test('regeneration creates a read-only follow-up and retries the same receipt without changing the selected permission', async () => {
  const answer = message('answer', 2, 'assistant')
  const snapshot: ConversationSnapshot = { conversation_id: 'conversation-1', messages: [message('question', 1), answer],
    turns: [{ turn_id: answer.turn_id, state: 'completed' }] }
  const h = harness({ snapshot, failSend: true })
  h.render(); await h.flush()
  await h.render().refresh()
  h.render().setPermissionMode('full_access')
  await h.render().send('answer')
  assert.equal(h.sends.length, 1)
  const request = h.sends[0].message
  assert.equal(request.reply_to_message_id, 'answer')
  assert.equal(request.permission_mode, 'read_only')
  assert.match(request.text, /重新生成上一条回答/)
  assert.equal(h.render().permissionMode, 'full_access')
  assert(h.render().snapshot?.messages.some(item => item.id === 'answer'))
  const restored = harness({ storage: h.storage, snapshot, failSend: true })
  restored.render(); await restored.flush()
  await restored.render().send()
  assert.equal(restored.sends[0].message.message_id, request.message_id)
  assert.equal(restored.sends[0].message.permission_mode, 'read_only')
  assert.equal(restored.sends[0].message.reply_to_message_id, 'answer')
})

test('regeneration preserves drafts and refuses an old reply or a running task', async () => {
  const snapshot: ConversationSnapshot = { conversation_id: 'conversation-1', messages: [message('old', 1, 'assistant'), message('answer', 2, 'assistant')], turns: [] }
  const h = harness({ snapshot })
  h.render(); await h.flush(); await h.render().refresh()
  await h.render().send('old')
  h.render().setDraft('尚未发送的草稿')
  await h.render().send('answer')
  assert.equal(h.render().draft, '尚未发送的草稿')
  h.render().setDraft('')
  h.setSnapshot({ ...snapshot, active_run: { run_id: 'navigation' } })
  await h.render().refresh()
  await h.render().send('answer')
  assert.equal(h.sends.length, 0)
})

test('role preferences reach new messages and pending retries keep their original persona', async () => {
  const h = harness({ failSend: true })
  const profile = { name: '巡检伙伴', shape: 'orb', color: 'sage', instructions: '简洁中文回复' }
  h.storage.set('lingtu.bot-profile.v1', JSON.stringify(profile))
  h.render().setDraft('查看地图')
  await h.flush(); await h.render().send()
  const pending = h.render().pending!
  assert.match(pending.persona!, /巡检伙伴.*\n.*简洁中文回复/)
  h.storage.set('lingtu.bot-profile.v1', JSON.stringify({ ...profile, instructions: '新的角色' }))
  const restored = harness({ storage: h.storage, failSend: true })
  restored.render(); await restored.flush(); await restored.render().send()
  assert.equal(restored.sends[0].message.persona, pending.persona)
  assert.equal(restored.sends[0].message.message_id, pending.message_id)
})

test('standby suspends conversation requests and preserves the draft for start', async () => {
  const h = harness()
  h.render(false)
  await h.flush()
  assert.equal(h.counts().optionsCount, 0)
  assert.equal(h.counts().fetchCount, 0)
  h.render().setOpen(true)
  h.render()
  await h.flush()
  h.render().setDraft('还没发')
  await h.flush()
  assert.equal(h.timers.size, 1)
  h.render(false)
  assert.equal(h.timers.size, 0)
  assert.equal(h.render(false).draft, '还没发')
  const reads = h.counts().fetchCount
  h.render()
  await h.flush()
  assert.equal(h.counts().fetchCount, reads + 1)
  assert.equal(h.sends.length, 0)
})

test('closing keeps the draft and history and stops idle polling until reopened', async () => {
  const h = harness({ snapshot: { conversation_id: 'conversation-1', messages: [message('a', 1)], turns: [] } })
  h.render().setOpen(true)
  h.render()
  await h.flush()
  h.render().setDraft('还没发')
  h.render().setOpen(false)
  h.render()
  await h.flush()
  assert.equal(h.render().draft, '还没发')
  assert.equal(h.render().snapshot?.messages.length, 1)
  assert.equal(h.timers.size, 0)
  const reads = h.counts().fetchCount
  h.render().setOpen(true)
  h.render()
  await h.flush()
  assert.ok(h.counts().fetchCount > reads)
})

test('background history refresh keeps the resolved empty state instead of flashing loading', async () => {
  const h = harness()
  h.render().setOpen(true)
  h.render()
  assert.equal(h.render().loading, false)
  assert.equal(h.render().optionsLoading, true)
  await h.flush()
  assert.equal(h.render().loading, false)
  let release!: () => void
  h.setFetchGate(new Promise<void>(resolve => { release = resolve }))
  const refresh = h.render().refresh()
  assert.equal(h.render().loading, false)
  assert.equal(h.render().snapshot?.messages.length, 0)
  release()
  await refresh
  assert.equal(h.render().loading, false)
})

test('permission modes persist and uncertain sends freeze the original permission', async () => {
  const h = harness({ failSend: true })
  h.render()
  await h.flush()
  assert.equal(h.render().permissionMode, 'default')
  h.render().setPermissionMode('read_only')
  h.render().setDraft('说明一下')
  await h.render().send()
  assert.equal(h.sends[0].message.permission_mode, 'read_only')
  h.render().setPermissionMode('full_access')
  assert.equal(h.render().permissionMode, 'read_only')
  const restored = harness({ storage: h.storage, failSend: true })
  restored.render()
  await restored.flush()
  assert.equal(restored.render().permissionMode, 'read_only')
  await restored.render().send()
  assert.equal(restored.sends[0].message.permission_mode, 'read_only')
  assert.equal(restored.sends[0].message.message_id, h.sends[0].message.message_id)
})

test('approval decisions are scoped and repeated clicks do not submit twice', async () => {
  const h = harness({ snapshot: { conversation_id: 'conversation-1', messages: [], turns: [], approvals: [
    { approval_id: 'approval-1', turn_id: 'turn-1', tool_name: 'navigate_to', arguments: { x: 1, y: 2 }, state: 'pending' },
  ] } })
  h.render().setOpen(true)
  h.render()
  await h.flush()
  const first = h.render().decideApproval('approval-1', false)
  assert.equal(h.render().approvalBusy, 'approval-1')
  await h.render().decideApproval('approval-1', true)
  await first
  assert.deepEqual(h.approvals, [{ id: 'conversation-1', approvalId: 'approval-1', approved: false }])
  assert.equal(h.render().snapshot?.approvals?.[0].state, 'denied')
  assert.equal(h.render().approvalBusy, null)
  await h.render().decideApproval('approval-1', true)
  assert.equal(h.approvals.length, 1)
})

test('switching sessions restores their own drafts preferences and pending request without replaying', async () => {
  const h = harness({ failSend: true })
  h.render().setOpen(true)
  h.render()
  await h.flush()
  h.render().setDraft('第一份草稿')
  h.render().setPermissionMode('read_only')
  await h.render().send()
  const original = h.render().pending
  await h.render().createNewConversation()
  assert.equal(h.render().snapshot?.conversation_id, 'conversation-2')
  assert.equal(h.render().draft, '')
  assert.equal(h.render().permissionMode, 'default')
  h.render().setDraft('第二份草稿')
  await h.render().switchConversation('conversation-1')
  assert.equal(h.render().draft, '第一份草稿')
  assert.equal(h.render().permissionMode, 'read_only')
  assert.equal(h.render().pending?.message_id, original?.message_id)
  assert.equal(h.sends.length, 1)
  await h.render().switchConversation('conversation-2')
  assert.equal(h.render().draft, '第二份草稿')
  assert.equal(h.render().pending, null)
})

test('queue edits survive session switches without overwriting the other draft', async () => {
  const h = harness()
  h.render().setOpen(true)
  h.render()
  await h.flush()
  h.render().setQueuedEdit({ id: 'queued-1', text: '保留队列修改' })
  const finishOldEdit = h.render().setQueuedEdit
  await h.render().createNewConversation()
  h.render().setDraft('另一个会话的草稿')
  h.render().setQueuedEdit({ id: 'queued-2', text: '另一份队列修改' })
  finishOldEdit(null)
  assert.equal(h.render().queuedEdit?.id, 'queued-2')
  await h.render().switchConversation('conversation-1')
  assert.equal(h.render().queuedEdit?.text, '保留队列修改')
  await h.render().switchConversation('conversation-2')
  assert.equal(h.render().draft, '另一个会话的草稿')
  assert.equal(h.sends.length, 0)
})

test('a slow old snapshot cannot replace a newly selected conversation or its draft', async () => {
  const h = harness()
  h.render().setOpen(true)
  h.render()
  await h.flush()
  await h.render().createNewConversation()
  h.render().setDraft('保留第二份')
  let release!: () => void
  h.setFetchGate(new Promise<void>(resolve => { release = resolve }))
  const oldRead = h.render().refresh()
  h.setFetchGate(null)
  await h.render().switchConversation('conversation-1')
  h.render().setDraft('第一份正在编辑')
  release()
  await oldRead
  assert.equal(h.render().snapshot?.conversation_id, 'conversation-1')
  assert.equal(h.render().draft, '第一份正在编辑')
  await h.render().switchConversation('conversation-2')
  assert.equal(h.render().draft, '保留第二份')
})

test('session metadata and sharing update without submitting messages or changing a task', async () => {
  const h = harness()
  h.render().setOpen(true)
  h.render()
  await h.flush()
  await h.render().renameConversation('conversation-1', '玻璃区域检查')
  assert.equal(h.render().snapshot?.metadata?.title, '玻璃区域检查')
  await h.render().archiveConversation('conversation-1', true)
  h.render().setDraft('尚未发送')
  await h.render().send()
  assert.match(h.render().error, /已归档/)
  await h.render().archiveConversation('conversation-1', false)
  await h.render().shareCurrentConversation()
  assert.match(h.render().shareUrl, /conversation-share=share-conversation-1$/)
  await h.render().revokeShare()
  assert.equal(h.render().shareUrl, '')
  assert.equal(h.sends.length, 0)
  assert.deepEqual(h.observed, [])
})

test('a definite rejected message remains retryable and can be edited into a new request', async () => {
  const h = harness({ rejectSend: 409 })
  h.render().setDraft('去门口')
  await h.flush()
  await h.render().send()
  const originalId = h.render().pending?.message_id
  assert.ok(originalId)
  assert.equal(h.render().editable, true)
  h.render().setDraft('先查询状态')
  assert.equal(h.render().pending, null)
  await h.render().send()
  assert.notEqual(h.render().pending?.message_id, originalId)
  assert.equal(h.sends[1].message.text, '先查询状态')
})

test('active turns poll while closed and repeated snapshots adopt a run only once', async () => {
  const h = harness({ snapshot: { conversation_id: 'conversation-1', messages: [message('u', 1), message('tool', 2, 'tool', 'run-1')],
    turns: [{ turn_id: 'turn-u', state: 'running' }], active_run: { run_id: 'run-1', instruction: '任务' } } })
  h.render().setOpen(true)
  h.render()
  await h.flush()
  h.render().setOpen(false)
  h.render()
  await h.flush()
  await h.tick()
  assert.equal(h.render().snapshot?.messages.length, 2)
  assert.deepEqual(h.observed, ['run-1'])
  assert.ok(h.timers.size)
})

test('a stale saved conversation creates a new one and explicitly reports unrecovered history', async () => {
  const storage = new Map([['lingtu.agent.conversation:http://robot.test', JSON.stringify({ conversation_id: 'stale', draft: '草稿', pending: null })]])
  const h = harness({ storage, failFetch: 404 })
  h.render()
  await h.flush()
  assert.equal(h.counts().createCount, 1)
  assert.match(h.render().notice, /历史未恢复/)
  assert.equal(h.render().draft, '草稿')
  assert.equal(h.sends.length, 0)
})

test('an unavailable Agent stays idle until manual retry and preserves the draft', async () => {
  const options = { failOptions: true }
  const h = harness(options)
  h.render().setOpen(true)
  h.render()
  await h.flush()
  h.render().setDraft('保留这句话')
  const reads = h.counts()
  await h.flush()
  await h.flush()
  assert.deepEqual(h.counts(), reads)
  assert.equal(h.counts().createCount, 0)
  assert.equal(h.timers.size, 0)
  assert.equal(h.render().loading, false)
  options.failOptions = false
  await h.render().reconnect()
  await h.flush()
  assert.ok(h.render().options)
  assert.equal(h.render().draft, '保留这句话')
  assert.equal(h.sends.length, 0)
})

test('service unavailable is shown without fabricating messages or tasks', async () => {
  const h = harness({ failFetch: 503 })
  h.render().setOpen(true)
  h.render()
  await h.flush()
  assert.match(h.render().error, /Agent 未启用/)
  assert.equal(h.render().snapshot, null)
  assert.deepEqual(h.observed, [])
})

test('conversation markup labels replies clarification and tool progress and exposes a close button', () => {
  const require = createRequire(import.meta.url)
  const react = require('react')
  const { renderToStaticMarkup } = require('react-dom/server')
  const exports: Record<string, unknown> = {}
  const messages = [message('one', 1), { ...message('two', 2, 'assistant'), kind: 'clarification' }, message('three', 3, 'tool')]
  runInNewContext(compile('../src/components/AgentConversation.tsx'), { exports, require: (name: string) => {
    const dependency = conversationDependency(name)
    if (dependency) return dependency
    if (name.endsWith('/guide/markdown')) return markdownExports
    if (name.endsWith('/menuEvents')) return { useMenuDismiss() {} }
    if (name.endsWith('/agentConversation/EffortGlyph')) return glyphExports
    if (name.endsWith('/ConversationShareCard')) return shareCardExports
    if (name.endsWith('.css')) return { default: {} }
    return require(name)
  } })
  const html = renderToStaticMarkup(react.createElement(exports.AgentConversation, { locale: 'zh', controller: {
    open: true, draft: '草稿', editable: true, attachments: [], options: capabilities, model: 'model-a', reasoning: 'default',
    snapshot: { messages, turns: [] }, setOpen() {}, setDraft() {}, send() {},
  } }))
  assert.match(html, /需要补充信息/)
  assert.match(html, /工具进展/)
  assert.match(html, /收起对话/)
  assert.match(html, /与机器人对话…/)
  assert.match(html, /textarea/)
  assert.match(html, /文件与图片/)
  assert.match(html, /思考深度/)
  assert.match(html, /Model A/)
  assert.doesNotMatch(html, /结束任务|跟随我|一瓶水/)
})

test('model capabilities load once while history polls and unavailable capabilities prevent sending', async () => {
  const h = harness()
  h.render().setOpen(true)
  h.render()
  await h.flush()
  await h.tick()
  assert.equal(h.counts().optionsCount, 1)
  assert.ok(h.counts().fetchCount >= 2)
  h.render().setReasoning('high')
  assert.equal(h.render().reasoning, 'high')
  h.render().setModel('model-b')
  assert.equal(h.render().reasoning, 'default')
  await h.render().reconnect()
  assert.equal(h.counts().optionsCount, 2)

  const disconnected = harness({ failOptions: true })
  disconnected.render().setDraft('你好')
  await disconnected.flush()
  await disconnected.render().send()
  assert.match(disconnected.render().optionsError, /Agent 未启用/)
  assert.equal(disconnected.render().options, null)
  assert.equal(disconnected.sends.length, 0)
})

test('attachment-only drafts persist metadata and timeout retries freeze model reasoning and attachment ids', async () => {
  const h = harness({ failSend: true })
  h.render()
  await h.flush()
  h.render().setReasoning('high')
  await h.render().addAttachments([{ name: '现场.txt', type: 'text/plain', size: 14 } as File])
  assert.equal(h.render().attachments[0].name, '现场.txt')
  assert.doesNotMatch([...h.storage.values()][0], /data_base64|data:/)
  await h.render().send()
  const original = h.render().pending!
  assert.equal(original.text, '')
  assert.equal(original.model, 'model-a')
  assert.equal(original.reasoning, 'high')
  assert.equal(original.attachment_ids?.length, 1)
  h.render().setModel('model-b')
  h.render().setReasoning('default')
  h.render().removeAttachment(original.attachment_ids![0])
  assert.equal(h.render().model, 'model-a')
  assert.equal(h.render().reasoning, 'high')
  assert.equal(h.render().attachments.length, 1)

  const restored = harness({ storage: h.storage, failSend: true })
  restored.render()
  await restored.flush()
  assert.equal(restored.render().attachments[0].name, '现场.txt')
  await restored.render().send()
  assert.deepEqual(JSON.parse(JSON.stringify(restored.sends[0].message)), JSON.parse(JSON.stringify(original)))
  h.setFailSend(false)
  await h.render().send()
  assert.deepEqual(h.sends[0].message, h.sends[1].message)
  assert.equal(h.render().attachments.length, 0)
  assert.equal(h.render().model, 'model-a')
  assert.equal(h.render().reasoning, 'high')
})

test('attachment limits preserve the draft and attachment metadata appears in history', async () => {
  const h = harness()
  h.render()
  await h.flush()
  await h.render().addAttachments(Array.from({ length: 5 }, (_, index) => ({ name: `${index}.txt`, type: 'text/plain', size: 1 } as File)))
  assert.match(h.render().error, /最多添加 4/)
  assert.equal(h.counts().uploadCount, 0)
  await h.render().refresh()
  assert.match(h.render().error, /最多添加 4/)
  await h.render().addAttachments([{ name: 'notes.txt', type: 'text/plain', size: 1024 } as File])
  const require = createRequire(import.meta.url)
  const exports: Record<string, unknown> = {}
  runInNewContext(compile('../src/components/AgentConversation.tsx'), { exports, require: (name: string) => {
    const dependency = conversationDependency(name)
    if (dependency) return dependency
    if (name.endsWith('/guide/markdown')) return markdownExports
    if (name.endsWith('/menuEvents')) return { useMenuDismiss() {} }
    if (name.endsWith('/agentConversation/EffortGlyph')) return glyphExports
    if (name.endsWith('/ConversationShareCard')) return shareCardExports
    return name.endsWith('.css') ? { default: {} } : componentDependency(name, require)
  } })
  const html = require('react-dom/server').renderToStaticMarkup(require('react').createElement(exports.AgentConversation, {
    locale: 'zh', controller: { ...h.render(), open: true, snapshot: {
      messages: [{ ...message('attached', 1), text: '', attachments: h.render().attachments }], turns: [],
    } },
  }))
  assert.match(html, /notes.txt/)
  assert.match(html, /1 KB/)
  assert.match(html, /移除 notes.txt/)
})

test('images require a capable model and remain available after changing the model', async () => {
  const h = harness()
  h.render()
  await h.flush()
  h.render().setModel('model-b')
  await h.render().addAttachments([{ name: 'scene.png', type: 'image/png', size: 20 } as File])
  await h.render().send()
  assert.match(h.render().error, /当前模型不支持图片/)
  assert.equal(h.sends.length, 0)
  assert.equal(h.render().attachments.length, 1)
  h.render().setModel('model-a')
  await h.render().send()
  assert.equal(h.sends.length, 1)
  assert.equal(h.sends[0].message.model, 'model-a')
})

test('the multiline composer sends Enter and preserves Shift Enter and IME composition', () => {
  const require = createRequire(import.meta.url)
  const exports: Record<string, unknown> = {}
  const react = { useRef: () => ({ current: null }), useState: () => [false, () => {}], useEffect() {}, useLayoutEffect() {} }
  runInNewContext(compile('../src/components/AgentConversation.tsx'), { exports, require: (name: string) => {
    const dependency = conversationDependency(name)
    if (dependency) return dependency
    if (name === 'react') return { ...require('react'), ...react }
    if (name.endsWith('/useMapSettings.ts')) return { useMapSettings: () => null }
    if (name.endsWith('/agentConversation/ToolActivity.tsx')) return { toolNames: {}, ToolActivity: ({ message }: { message: { text: string } }) => createRequire(import.meta.url)('react').createElement('p', null, message.text) }
    if (name.endsWith('/guide/markdown')) return markdownExports
    if (name.endsWith('/menuEvents')) return { useMenuDismiss() {} }
    if (name.endsWith('/agentConversation/EffortGlyph')) return glyphExports
    if (name.endsWith('/ConversationShareCard')) return shareCardExports
    if (name.endsWith('.css')) return { default: {} }
    return require(name)
  } })
  let sends = 0
  let prevented = 0
  const render = exports.AgentConversation as (props: unknown) => unknown
  const tree = render({ locale: 'zh', controller: {
    open: false, draft: '第一行\n第二行', editable: true, attachments: [], options: capabilities,
    model: 'model-a', reasoning: 'default', send() { sends++ },
  } })
  type Element = { type?: string; props?: { children?: unknown; onKeyDown?: (event: unknown) => void } }
  const findTextarea = (node: unknown): Element | undefined => {
    if (!node || typeof node !== 'object') return undefined
    const element = node as Element
    if (element.type === 'textarea') return element
    const children = element.props?.children
    return (Array.isArray(children) ? children : [children]).map(findTextarea).find(Boolean)
  }
  const onKeyDown = findTextarea(tree)?.props?.onKeyDown
  assert.ok(onKeyDown)
  const enter = { key: 'Enter', shiftKey: false, nativeEvent: { isComposing: false }, keyCode: 13,
    preventDefault() { prevented++ } }
  onKeyDown({ ...enter, shiftKey: true })
  assert.equal(sends, 0)
  assert.equal(prevented, 0)
  onKeyDown({ ...enter, nativeEvent: { isComposing: true } })
  onKeyDown({ ...enter, keyCode: 229 })
  assert.equal(sends, 0)
  onKeyDown(enter)
  assert.equal(sends, 1)
  assert.equal(prevented, 3)
})

test('assistant markdown escapes HTML, user messages stay plain and tool details start collapsed', () => {
  const require = createRequire(import.meta.url)
  const exports: Record<string, unknown> = {}
  runInNewContext(compile('../src/components/AgentConversation.tsx'), { exports, require: (name: string) => {
    const dependency = conversationDependency(name)
    if (dependency) return dependency
    if (name.endsWith('/guide/markdown')) return markdownExports
    if (name.endsWith('/menuEvents')) return { useMenuDismiss() {} }
    if (name.endsWith('/agentConversation/EffortGlyph')) return glyphExports
    if (name.endsWith('/ConversationShareCard')) return shareCardExports
    return name.endsWith('.css') ? { default: {} } : componentDependency(name, require)
  } })
  const assistant = '**TURTLE-42** 和 **红色**\n\n<script>alert(1)</script>\n\n[unsafe](javascript:alert)\n\n## 标题\n\n```html\n<img src=x onerror=alert(1)>\n```'
  const html = require('react-dom/server').renderToStaticMarkup(require('react').createElement(exports.AgentConversation, {
    locale: 'zh', controller: { open: true, draft: '', editable: true, attachments: [], options: capabilities,
      model: 'model-a', reasoning: 'default', snapshot: { messages: [
        { ...message('a', 1, 'assistant'), text: assistant },
        { ...message('u', 2), text: '**用户原文** <img src=x>' },
        { ...message('t', 3, 'tool'), text: '**工具原文** <script>' },
      ], turns: [] },
    },
  }))
  assert.match(html, /<strong>TURTLE-42<\/strong> 和 <strong>红色<\/strong>/)
  assert.match(html, /&lt;script&gt;alert\(1\)&lt;\/script&gt;/)
  assert.match(html, /&lt;img src=x onerror=alert\(1\)&gt;/)
  assert.match(html, /href="#"[^>]*>[\s\S]*unsafe[\s\S]*<\/a>/)
  assert.match(html, /\*\*用户原文\*\* &lt;img src=x&gt;/)
  assert.doesNotMatch(html, /工具原文/)
  assert.doesNotMatch(html, /<script|<img|href="javascript:/)
})

test('composer menus show the selected model and reasoning without changing the submission', () => {
  const require = createRequire(import.meta.url)
  const exports: Record<string, unknown> = {}
  runInNewContext(compile('../src/components/AgentConversation.tsx'), { exports, require: (name: string) => {
    const dependency = conversationDependency(name)
    if (dependency) return dependency
    if (name.endsWith('/guide/markdown')) return markdownExports
    if (name.endsWith('/menuEvents')) return { useMenuDismiss() {} }
    if (name.endsWith('/agentConversation/EffortGlyph')) return glyphExports
    if (name.endsWith('/ConversationShareCard')) return shareCardExports
    return name.endsWith('.css') ? { default: {} } : componentDependency(name, require)
  } })
  const render = (pending: unknown) => require('react-dom/server').renderToStaticMarkup(require('react').createElement(exports.AgentConversation, {
    locale: 'zh', controller: { open: false, draft: '去门口', editable: !pending, attachments: [], options: capabilities,
      model: 'model-a', reasoning: 'high', pending },
  }))
  const html = render(null)
  assert.match(html, /<summary[^>]*aria-label="模型"[^>]*aria-haspopup="menu"/)
  assert.match(html, /role="menuitemradio" aria-checked="true"[^>]*><span>Model A<\/span>/)
  assert.match(html, /aria-label="思考深度"/)
  assert.match(html, /<span>深入<\/span>/)
  assert.doesNotMatch(html, /<select/)
  const pending = render({ message_id: 'original', text: '去门口' })
  assert.match(pending, /aria-label="模型"[^>]*aria-disabled="true"/)
  assert.match(pending, /aria-label="重试原消息"/)
})

test('migrated reasoning glyph renders ascending pixel-aligned bars and the selected intensity', () => {
  const require = createRequire(import.meta.url)
  const html = require('react-dom/server').renderToStaticMarkup(require('react').createElement(glyphExports.EffortGlyph, { count: 3, filled: 1 }))
  assert.equal((html.match(/<rect /g) ?? []).length, 3)
  assert.equal((html.match(/opacity="1"/g) ?? []).length, 1)
  assert.equal((html.match(/opacity="0.4"/g) ?? []).length, 2)
  assert.deepEqual([...html.matchAll(/<rect[^>]*height="(\d+)"/g)].map(match => Number(match[1])), [6, 9, 12])
  assert.match(html, /aria-hidden="true"/)
})

test('options and attachment HTTP contracts validate responses and upload only permitted files', async () => {
  const calls: Array<{ path: string; body?: string; timeout?: number }> = []
  let responseBody: unknown = capabilities
  let reads = 0
  let timeout = 0
  const exports: Record<string, unknown> = {}
  class Reader {
    result = 'data:text/plain;base64,aGk='
    onload?: () => void
    readAsDataURL() { reads++; this.onload?.() }
  }
  runInNewContext(compile('../src/services/agentConversation.ts'), {
    exports, require: () => ({ dashboardFetch: async (path: string, init: RequestInit) => {
      calls.push({ path, body: init.body as string | undefined, timeout })
      return new Response(JSON.stringify(responseBody), { status: 200 })
    } }), Error, Map, FileReader: Reader, AbortSignal: { timeout: (value: number) => { timeout = value; return undefined } },
  })
  const getOptions = exports.fetchConversationOptions as () => Promise<ConversationOptions>
  assert.equal((await getOptions()).models.length, 2)
  assert.equal(calls[0].path, '/api/v1/agent/options')
  responseBody = { ...capabilities, permission_modes: undefined, default_permission_mode: undefined }
  await assert.rejects(getOptions(), /模型能力格式无效/)
  responseBody = { ...capabilities, models: [] }
  await assert.rejects(getOptions(), /模型能力格式无效/)
  const upload = exports.uploadConversationAttachment as (id: string, file: File, attachmentId: string, limits: ConversationOptions['attachments']) => Promise<ConversationAttachment>
  const file = { name: 'notes.txt', type: 'text/plain', size: 2 } as File
  responseBody = { id: 'file-1', name: file.name, mime_type: file.type, size: 2 }
  await upload('conversation/1', file, 'file-1', capabilities.attachments)
  const call = calls.at(-1)!
  assert.equal(call.path, '/api/v1/agent/conversations/conversation%2F1/attachments')
  assert.equal(call.timeout, 60_000)
  assert.deepEqual(JSON.parse(call.body!), { attachment_id: 'file-1', name: 'notes.txt', mime_type: 'text/plain', data_base64: 'aGk=' })
  await assert.rejects(upload('conversation/1', { ...file, name: 'run.exe' } as File, 'file-2', capabilities.attachments), /不支持/)
  await assert.rejects(upload('conversation/1', { ...file, size: 20 * 1024 * 1024 } as File, 'file-2', capabilities.attachments), /超过/)
  assert.equal(reads, 1)
  const fetchSnapshot = exports.fetchConversation as (id: string) => Promise<unknown>
  responseBody = { conversation_id: 'conversation/1', messages: [{ ...message('file', 1), attachments: [{ id: 'bad' }] }], turns: [] }
  await assert.rejects(fetchSnapshot('conversation/1'), /快照格式无效/)
})


test('choice reply submits its question identity once and preserves an existing draft', async () => {
  const h = harness()
  h.render()
  h.render()
  await h.flush()
  await Promise.all([h.render().answerQuestion('question-1', '地面'), h.render().answerQuestion('question-1', '地面')])
  assert.equal(h.sends.length, 1)
  assert.equal(h.sends[0].message.text, '地面')
  assert.equal(h.sends[0].message.reply_to_message_id, 'question-1')
  h.render().setDraft('未发送的草稿')
  await h.render().answerQuestion('question-2', '墙面')
  assert.equal(h.sends.length, 1)
  assert.equal(h.render().draft, '未发送的草稿')
})
