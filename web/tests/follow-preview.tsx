import { useEffect, useRef, useState } from 'react'
import { createRoot } from 'react-dom/client'
import { Play, RotateCcw } from 'lucide-react'
import { AgentConversation } from '../src/components/AgentConversation'
import { InstructionTaskBar } from '../src/components/InstructionTaskBar'
import { useAgentConversation } from '../src/hooks/useAgentConversation'
import type { ConversationSnapshot } from '../src/services/agentConversation'
import type { AgentTaskStatusResponse } from '../src/types'
import '../src/App.css'
import './follow-preview.css'

// Preview transport is deliberately closed: no request reaches a Host or model.
const cid = 'follow-preview'
const runId = 'demo-follow-run'
const instruction = '跟随穿红色上衣的人'
const steps = ['选人', '跟随', '目标丢失', '取消中', '已取消']
let snapshot: ConversationSnapshot
let changeFromChat: (step: number) => void = () => {}
let step = 1
let sequence = 0

function status(index: number): AgentTaskStatusResponse {
  return {
    found: true, run_id: runId, instruction,
    state: index >= 3 ? 'cancelled' : 'running',
    motion_attempted: true, motion_success: false, stop_confirmed: index === 4,
    navigation: {
      follow_task_id: 'demo-follow-1', product_session_id: 'demo-session',
      state: index === 4 ? 'cancelled' : index === 3 ? 'cancelling' : 'running',
      phase: ['selecting', 'following', 'lost', 'cancelling', 'cancelled'][index],
      terminal: index === 4, success: false, reason: index === 2 ? 'target_lost' : '',
    },
  }
}

function message(turnId: string, role: 'user' | 'assistant' | 'tool', text: string, kind = 'reply') {
  const seq = ++sequence
  snapshot.messages.push({ id: `demo-${seq}`, seq, turn_id: turnId, role, text, kind,
    run_id: runId, created_at: Date.now() / 1000 - 12 + seq / 10,
    ...(role === 'tool' ? { metadata: { status: 'completed', activity_state: 'completed', duration_ms: 180 } } : {}),
  })
}

function build(index: number) {
  step = index
  sequence = 0
  snapshot = { conversation_id: cid, metadata: { conversation_id: cid, title: '人员跟随', archived: false,
    created_at: Date.now() / 1000, updated_at: Date.now() / 1000 }, messages: [], turns: [], approvals: [],
    history_cursor: null, active_run: { run_id: runId, instruction } }
  message('start', 'user', instruction)
  message('start', 'tool', JSON.stringify({ accepted: true, phase: 'selecting', execution_confirmed: false,
    follow_task_id: 'demo-follow-1', run_id: runId }), 'follow_person')
  message('start', 'assistant', '请求已受理，正在选择符合描述的人员。')
  if (index >= 1) {
    message('query', 'user', '跟随状态')
    message('query', 'tool', JSON.stringify(status(index === 2 ? 2 : 1)), 'get_agent_status')
    message('query', 'assistant', index === 2
      ? '跟随目标暂时丢失，正在尝试重新识别。'
      : '正在跟随穿红色上衣的人。你可以继续聊天，任务会保持运行。')
  }
  if (index >= 3) {
    message('stop', 'user', '停止跟随')
    message('stop', 'tool', JSON.stringify({ cancel_requested: true, stop_confirmed: index === 4 }), 'cancel_agent_task')
    message('stop', 'assistant', '已请求取消当前任务。停止尚未确认。')
  }
  for (const turnId of [...new Set(snapshot.messages.map(item => item.turn_id))]) {
    const rows = snapshot.messages.filter(item => item.turn_id === turnId)
    snapshot.turns.push({ turn_id: turnId, message_id: rows[0].id, run_id: runId, state: 'completed',
      kind: 'reply', text: rows.at(-1)!.text, started_at: rows[0].created_at ?? undefined,
      finished_at: rows.at(-1)!.created_at ?? undefined })
  }
}
build(1)

window.fetch = async (input, init) => {
  const url = new URL(typeof input === 'string' ? input : input instanceof URL ? input.href : input.url, location.origin)
  const path = url.pathname
  const method = init?.method ?? (input instanceof Request ? input.method : 'GET')
  const reply = (value: unknown, code = 200) => new Response(JSON.stringify(value),
    { status: code, headers: { 'Content-Type': 'application/json' } })
  if (path === '/api/v1/agent/options') return reply({ available: true, provider: 'preview', default_model: 'kimi-k3',
    default_reasoning: 'default', default_permission_mode: 'full_access', permission_modes: ['read_only', 'default', 'full_access'],
    models: [{ id: 'kimi-k3', label: 'Kimi K3', configured: true, provider: 'moonshot',
      reasoning: [{ id: 'default', label: '默认' }], image_input: false }],
    attachments: { accept: '.txt', max_files: 1, max_file_bytes: 1000 } })
  if (path === '/api/v1/agent/conversations' && method === 'POST') return reply({ conversation_id: cid }, 201)
  if (path === '/api/v1/agent/conversations') return reply({ conversations: [] })
  if (path.endsWith('/messages') && method === 'POST') {
    const body = JSON.parse(String(init?.body || '{}'))
    if (/停|取消/.test(body.text) || /跟随/.test(body.text) && !/状态/.test(body.text)) {
      changeFromChat(/停|取消/.test(body.text) ? 3 : 0)
      const turn = snapshot.turns.at(-1)!
      const user = snapshot.messages.find(item => item.turn_id === turn.turn_id && item.role === 'user')!
      user.id = turn.message_id = body.message_id
      user.text = body.text
      return reply({ ...turn, conversation_id: cid }, 202)
    }
    const turnId = `input-${Date.now()}`
    message(turnId, 'user', body.text)
    snapshot.messages.at(-1)!.id = body.message_id
    message(turnId, 'assistant', /停|取消/.test(body.text)
      ? '已请求取消，等待任务结束确认。' : `当前演示阶段：${steps[step]}。`)
    const turn = { turn_id: turnId, message_id: body.message_id, run_id: runId, state: 'completed' as const,
      kind: 'reply', started_at: Date.now() / 1000, finished_at: Date.now() / 1000 }
    snapshot.turns.push(turn)
    return reply({ ...turn, conversation_id: cid }, 202)
  }
  if (path.includes('/turns/')) return reply(snapshot.turns.find(item => path.endsWith(item.turn_id)))
  if (path.startsWith('/api/v1/agent/conversations/')) return reply({ ...snapshot, server_time: Date.now() / 1000 })
  return reply({ detail: { reason: 'preview_only' } }, 503)
}

function FollowPreview() {
  const [current, setCurrent] = useState(1)
  const [playing, setPlaying] = useState(false)
  const [dismissed, setDismissed] = useState(false)
  const timers = useRef<ReturnType<typeof setTimeout>[]>([])
  const chat = useAgentConversation(undefined, true, null, 'follow-preview')
  const refresh = useRef(chat.refresh)
  refresh.current = chat.refresh
  const clear = () => { timers.current.forEach(clearTimeout); timers.current = []; setPlaying(false) }
  const show = (index: number) => {
    build(index); setCurrent(index); setDismissed(false)
    void refresh.current()
  }
  const stop = () => {
    clear(); show(3)
    timers.current.push(setTimeout(() => show(4), 2400))
  }
  const replay = () => {
    clear(); setPlaying(true); show(0)
    ;[3200, 7600, 11800, 15000].forEach((delay, index) => {
      timers.current.push(setTimeout(() => { show(index + 1); if (index === 3) setPlaying(false) }, delay))
    })
  }
  useEffect(() => { chat.setOpen(true) }, [chat.setOpen])
  useEffect(() => {
    changeFromChat = index => index === 3 ? stop() : (clear(), show(index))
    if (new URLSearchParams(location.search).has('play')) replay()
    return () => { timers.current.forEach(clearTimeout) }
  }, [])
  return <main className="follow-demo">
    <header className="follow-demo-heading"><div><strong>LingTu</strong><span>对话跟随</span></div><span className="follow-demo-badge">模拟演示 · 未连接机器人</span></header>
    <nav className="follow-demo-steps" aria-label="跟随演示阶段">
      <div>{steps.map((label, index) => <button type="button" key={label} aria-pressed={current === index}
        onClick={() => { clear(); show(index) }}><span>{index + 1}</span>{label}</button>)}</div>
      <button type="button" className="follow-demo-play" onClick={playing ? clear : replay}>
        {playing ? <RotateCcw size={14} /> : <Play size={14} />} {playing ? '暂停演示' : '播放流程'}
      </button>
    </nav>
    <div className="follow-demo-stage" data-stage={current}>
      <InstructionTaskBar locale="zh" showComposer={false} motionStartAllowed motionStartBlockedReason="" hasNativeTask={false}
        controller={{ task: dismissed ? null : { runId, instruction, executor: 'agent', generation: 1,
          status: status(current), statusSource: 'poll', cancelRequested: current === 3, cancelError: '', pollError: '' },
          receipt: null, sending: false, error: '', adopt: () => {}, submit: async () => {},
          cancel: async () => stop(), dismiss: () => setDismissed(true) }} />
      <AgentConversation locale="zh" controller={chat} />
    </div>
    <footer>正式聊天组件与任务条 · 回执为演示数据</footer>
  </main>
}
createRoot(document.getElementById('root')!).render(<FollowPreview />)
