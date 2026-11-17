import { useEffect, useRef, useState } from 'react'
import { createRoot } from 'react-dom/client'
import { AgentConversation } from '../src/components/AgentConversation'
import { useAgentConversation } from '../src/hooks/useAgentConversation'
import type { ConversationSnapshot } from '../src/services/agentConversation'
import type { AgentTaskStatusResponse } from '../src/types'
import peopleImage from './fixtures/follow-people.jpg'
import reviewedPeople from './fixtures/follow-review.json'
import '../src/App.css'
import './follow-preview.css'

// Preview transport is deliberately closed: no request reaches a Host or model.
const cid = 'follow-preview'
const runId = 'demo-follow-run'
const instruction = '选择画面中的人员跟随'
const steps = ['选人确认', '跟随', '目标丢失', '取消中', '已取消']
let snapshot: ConversationSnapshot
let changeFromChat: (step: number, input?: { text: string; message_id: string }) => void = () => {}
let step = 0
let confirmedTarget = ''
let sequence = 0
let taskNumber = 0
let title = '人员跟随'
let cancellationTurn = ''

function status(index: number): AgentTaskStatusResponse {
  return {
    found: true, run_id: runId, instruction,
    state: index >= 3 ? 'cancelled' : 'running',
    motion_attempted: true, motion_success: false, stop_confirmed: index === 4,
    navigation: {
      follow_task_id: `demo-follow-${taskNumber}`, product_session_id: 'demo-session',
      state: index === 4 ? 'cancelled' : index === 3 ? 'cancelling' : 'running',
      phase: ['awaiting_confirmation', 'following', 'lost', 'cancelling', 'cancelled'][index],
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

function build(index: number, input?: { text: string; message_id: string }) {
  step = index
  if (index === 0) { taskNumber++; confirmedTarget = '' }
  snapshot ??= { conversation_id: cid, metadata: { conversation_id: cid, title, archived: false,
    created_at: Date.now() / 1000, updated_at: Date.now() / 1000 }, messages: [], turns: [], approvals: [],
    history_cursor: null, active_run: { run_id: runId, instruction } }
  if (index === 1) return
  const turnId = index === 4 && cancellationTurn ? cancellationTurn : `demo-turn-${sequence + 1}`
  if (index !== 4 || !cancellationTurn) {
    message(turnId, 'user', input?.text ?? (index === 4 ? '取消选人' : instruction))
    if (input) snapshot.messages.at(-1)!.id = input.message_id
  }
  if (index === 0) {
    cancellationTurn = ''
    message(turnId, 'tool', JSON.stringify({ accepted: true, phase: 'selecting', execution_confirmed: false,
      confirmation_required: true, product_session_id: 'demo-session',
      follow_task_id: `demo-follow-${taskNumber}`, run_id: runId }), 'follow_person')
    message(turnId, 'assistant', '8 个候选中已排除 1 个广告人物。可切换前后对比，再选人确认。')
  } else {
    if (index === 3) cancellationTurn = turnId
    message(turnId, 'tool', JSON.stringify(status(index)), index === 3 ? 'cancel_agent_task' : 'get_agent_status')
    message(turnId, 'assistant', index === 4 ? '跟随已取消，已确认停止。' : index === 3
      ? '已请求取消当前任务，等待停止确认。' : '跟随目标暂时丢失，正在尝试重新识别。')
  }
  const rows = snapshot.messages.filter(item => item.turn_id === turnId)
  const turn = { turn_id: turnId, message_id: rows[0].id, run_id: runId, state: 'completed' as const,
    kind: 'reply', text: rows.at(-1)!.text, started_at: rows[0].created_at ?? undefined,
    finished_at: rows.at(-1)!.created_at ?? undefined }
  const previous = snapshot.turns.find(item => item.turn_id === turnId)
  if (previous) Object.assign(previous, turn)
  else snapshot.turns.push(turn)
}
build(0)

window.fetch = async (input, init) => {
  const url = new URL(typeof input === 'string' ? input : input instanceof URL ? input.href : input.url, location.origin)
  const path = url.pathname
  const method = init?.method ?? (input instanceof Request ? input.method : 'GET')
  const reply = (value: unknown, code = 200) => new Response(JSON.stringify(value),
    { status: code, headers: { 'Content-Type': 'application/json' } })
  if (path.startsWith('/api/v1/follow/')) {
    if (path.split('/')[4] !== `demo-follow-${taskNumber}`) return reply({ found: true, terminal: true,
      confirmation_required: false, state: 'cancelled', accepted: false, reason: 'follow_task_not_active' })
    const task = { found: true, follow_task_id: `demo-follow-${taskNumber}`, product_session_id: 'demo-session',
      confirmation_required: step === 0, confirmed_target_id: confirmedTarget || (step > 0 ? '6' : ''),
      state: step === 4 ? 'cancelled' : step === 3 ? 'cancelling' : 'running', terminal: step === 4 }
    if (path.endsWith('/selection')) return reply(step === 0 ? { ...task,
      observation_sequence: 1, width: 1920, height: 1080, image: peopleImage, review_state: 'ready',
      candidates: reviewedPeople.candidates.map(candidate => ({ ...candidate,
        ...reviewedPeople.review[candidate.target_id as keyof typeof reviewedPeople.review],
        // Chinese display translation; the saved model response remains in the fixture.
        ...(candidate.target_id === '8' ? { reason: '广告屏中的人脸，位于带边框的平面画面内，不是真实行人。' } : {}),
      })),
    } : task)
    if (path.endsWith('/confirm')) {
      const body = JSON.parse(String(init?.body || '{}'))
      const verdict = reviewedPeople.review[body.target_id as keyof typeof reviewedPeople.review]
      const accepted = step === 0 && verdict?.kind === 'person' && body.observation_sequence === 1
      if (accepted) { confirmedTarget = body.target_id; changeFromChat(1) }
      return reply({ ...task, ok: accepted, accepted, status: accepted ? 'accepted' : 'rejected',
        confirmation_required: !accepted, reason: accepted ? 'target_confirmed' : 'person_not_verified' })
    }
    if (path.endsWith('/cancel')) {
      changeFromChat(4)
      return reply({ ...task, ok: true, accepted: true, status: 'accepted', terminal: true, state: 'cancelled' })
    }
    return reply(task)
  }
  if (path === '/api/v1/agent/options') return reply({ available: true, provider: 'preview', default_model: 'kimi-k3',
    default_reasoning: 'default', default_permission_mode: 'full_access', permission_modes: ['read_only', 'default', 'full_access'],
    models: [{ id: 'kimi-k3', label: 'Kimi K3', configured: true, provider: 'moonshot',
      reasoning: [{ id: 'default', label: '默认' }], image_input: false }],
    attachments: { accept: '.txt', max_files: 1, max_file_bytes: 1000 } })
  if (path === '/api/v1/agent/conversations' && method === 'POST') return reply({ conversation_id: cid }, 201)
  if (path === '/api/v1/agent/conversations') return reply({ conversations: [snapshot.metadata] })
  if (path === `/api/v1/agent/conversations/${cid}` && method === 'PATCH') {
    const body = JSON.parse(String(init?.body || '{}'))
    if (typeof body.title === 'string') title = body.title
    snapshot.metadata = { ...snapshot.metadata!, title, updated_at: Date.now() / 1000 }
    return reply(snapshot.metadata)
  }
  if (path.endsWith('/messages') && method === 'POST') {
    const body = JSON.parse(String(init?.body || '{}'))
    if (/停|取消|丢失|选人/.test(body.text) || /跟随/.test(body.text) && !/状态/.test(body.text)) {
      changeFromChat(/停|取消/.test(body.text) ? 3 : /丢失/.test(body.text) ? 2 : 0, body)
      const turn = snapshot.turns.at(-1)!
      return reply({ ...turn, conversation_id: cid }, 202)
    }
    const turnId = `input-${Date.now()}`
    message(turnId, 'user', body.text)
    snapshot.messages.at(-1)!.id = body.message_id
    message(turnId, 'tool', JSON.stringify(status(step)), 'get_agent_status')
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
  const [botId, setBotId] = useState('follow-preview')
  const timer = useRef<ReturnType<typeof setTimeout> | undefined>(undefined)
  const chat = useAgentConversation(undefined, true, null, botId)
  const refresh = useRef(chat.refresh)
  refresh.current = chat.refresh
  const show = (index: number, input?: { text: string; message_id: string }) => {
    build(index, input)
    void refresh.current()
  }
  useEffect(() => { chat.setOpen(true) }, [chat.setOpen])
  useEffect(() => {
    changeFromChat = (index, input) => {
      clearTimeout(timer.current); show(index, input)
      if (index === 3) timer.current = setTimeout(() => show(4), 2400)
    }
    return () => { clearTimeout(timer.current) }
  }, [])
  return <main className="follow-demo">
    <header className="follow-demo-heading"><span>MOT17 · Kimi 复核回放</span><span className="follow-demo-badge">模拟演示 · 未连接机器人</span></header>
    <AgentConversation locale="zh" controller={chat} onBotChange={setBotId} />
    <footer>对话内可输入：跟随状态 · 模拟目标丢失 · 停止跟随 · 重新选人</footer>
  </main>
}
createRoot(document.getElementById('root')!).render(<FollowPreview />)
