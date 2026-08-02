import { useState, useRef, useEffect, useMemo, useCallback } from 'react'
import { Send } from 'lucide-react'
import type { SSEState } from '../types'
import * as api from '../services/api'
import {
  isSlashCommand,
  executeSlashCommand,
  matchSlashCommands,
  type SlashCommandSpec,
} from '../utils/slashCommands'
import { ThinkingBubble } from './ThinkingBubble'
import styles from './ChatPanel.module.css'

interface Message {
  id: number
  role: 'user' | 'system' | 'assistant'
  text: string
  ts: number
  phase?: string
}

let _id = 0
function nextId() { return ++_id }

function formatTime(ts: number) {
  return new Date(ts).toLocaleTimeString('zh-CN', {
    hour12: false, hour: '2-digit', minute: '2-digit', second: '2-digit',
  })
}

const NAV_STATE_ZH: Record<string, string> = {
  PLANNING:  '规划路线中',
  EXECUTING: '导航中',
  ARRIVED:   '已到达目的地。',
  IDLE:      '导航空闲。',
  FAILED:    '导航失败。',
  CANCELLED: '导航已取消。',
}

interface ChatPanelProps {
  sseState: SSEState
  motionStartAllowed: boolean
  motionStartBlockedReason: string
}

export function ChatPanel({
  sseState,
  motionStartAllowed,
  motionStartBlockedReason,
}: ChatPanelProps) {
  const [messages, setMessages] = useState<Message[]>([
    {
      id: nextId(),
      role: 'system',
      text: '灵途导航系统已上线。发送自然语言指令，或输入 / 查看命令。',
      ts: Date.now(),
    },
  ])
  const [input, setInput] = useState('')
  const [sending, setSending] = useState(false)
  const [suggestionIdx, setSuggestionIdx] = useState(0)
  const [thinking, setThinking] = useState<{ hint: string; startedAt: number } | null>(null)
  const listRef = useRef<HTMLDivElement>(null)
  const inputRef = useRef<HTMLInputElement>(null)
  const prevMissionRef = useRef<string | null>(null)
  const lastAgentTsRef = useRef<number>(0)

  // ── Autocomplete candidates ──
  const suggestions: SlashCommandSpec[] = useMemo(() => {
    if (!isSlashCommand(input)) return []
    // Only show suggestions while user is still typing the command name
    // (i.e. no space typed yet). Once space is typed the user is entering args.
    if (input.includes(' ')) return []
    return matchSlashCommands(input)
  }, [input])

  const showDropdown = suggestions.length > 0

  // Reset selected index when suggestions change
  useEffect(() => {
    setSuggestionIdx(0)
  }, [suggestions.length])

  // Scroll to bottom on new messages or thinking state change
  useEffect(() => {
    if (listRef.current) {
      listRef.current.scrollTop = listRef.current.scrollHeight
    }
  }, [messages, thinking])

  // Safety: clear orphaned thinking after 60s (agent hung or crashed).
  // Inline setMessages to avoid forward-reference to addSystem.
  useEffect(() => {
    if (!thinking) return
    const t = setTimeout(() => {
      setThinking(null)
      setMessages(prev => [...prev, {
        id: nextId(), role: 'system', text: '⚠ 智能体超时未响应', ts: Date.now(),
      }])
    }, 60_000)
    return () => clearTimeout(t)
  }, [thinking])

  // React to mission_status changes from SSE
  useEffect(() => {
    const ms = sseState.missionStatus
    if (!ms) return
    const key = `${ms.state}:${ms.goal ?? ''}:${ms.progress}`
    if (key === prevMissionRef.current) return
    prevMissionRef.current = key

    const label = NAV_STATE_ZH[ms.state] ?? ms.state
    const text = ms.goal
      ? `[${ms.state}] ${label} 目标：${ms.goal} (${Math.round(ms.progress * 100)}%)`
      : `[${ms.state}] ${label}`

    setMessages(prev => [...prev, { id: nextId(), role: 'system', text, ts: Date.now() }])
  }, [sseState.missionStatus])

  const addSystem = useCallback((text: string) => {
    setMessages(prev => [...prev, { id: nextId(), role: 'system', text, ts: Date.now() }])
  }, [])

  // ── Agent chat stream ──
  // SemanticPlanner publishes messages with role='thinking' (progress hint)
  // or 'assistant' (concrete result). Thinking updates the bubble's hint;
  // assistant appends to the message list and dismisses the bubble.
  useEffect(() => {
    const m = sseState.agentMessage
    if (!m) return
    if (m.ts <= lastAgentTsRef.current) return  // dedup
    lastAgentTsRef.current = m.ts

    if (m.role === 'thinking') {
      setThinking(prev => ({
        hint: m.text,
        startedAt: prev?.startedAt ?? Date.now(),
      }))
    } else if (m.role === 'assistant') {
      setMessages(prev => [...prev, {
        id: nextId(),
        role: 'assistant',
        text: m.text,
        ts: Math.floor(m.ts * 1000),
        phase: m.phase,
      }])
      setThinking(null)   // agent produced a reply — stop thinking
    }
  }, [sseState.agentMessage])

  const sendInstruction = useCallback(async () => {
    const text = input.trim()
    if (!text || sending) return
    setInput('')
    setMessages(prev => [...prev, { id: nextId(), role: 'user', text, ts: Date.now() }])

    if (isSlashCommand(text)) {
      const response = await executeSlashCommand(text, sseState)
      addSystem(response)
      return
    }

    if (!motionStartAllowed) {
      addSystem(`为避免依据旧状态触发运动，智能指令暂未发送：${motionStartBlockedReason}`)
      return
    }

    setSending(true)
    setThinking({ hint: '发送指令…', startedAt: Date.now() })
    try {
      const res = await api.sendInstruction(text)
      if (!res.ok) {
        addSystem(api.formatCommandAck(res, '指令'))
        setThinking(null)
      }
      // else: thinking bubble stays visible until agent_message arrives
    } catch (e: unknown) {
      addSystem(api.formatCommandError(e, '指令发送失败'))
      setThinking(null)
    } finally {
      setSending(false)
    }
  }, [input, sending, sseState, addSystem, motionStartAllowed, motionStartBlockedReason])

  // Accept the currently highlighted suggestion — inserts command
  // name + trailing space, so the user can immediately type args.
  const acceptSuggestion = useCallback((spec: SlashCommandSpec) => {
    // If the command has an argument template, add a space so typing
    // arguments feels natural. Commands with no args close the dropdown.
    const hasArgs = spec.usage.includes('<')
    setInput(spec.name + (hasArgs ? ' ' : ''))
    setSuggestionIdx(0)
    // Refocus for keyboard flow
    requestAnimationFrame(() => inputRef.current?.focus())
  }, [])

  function handleKeyDown(e: React.KeyboardEvent<HTMLInputElement>) {
    if (showDropdown) {
      if (e.key === 'ArrowDown') {
        e.preventDefault()
        setSuggestionIdx(i => (i + 1) % suggestions.length)
        return
      }
      if (e.key === 'ArrowUp') {
        e.preventDefault()
        setSuggestionIdx(i => (i - 1 + suggestions.length) % suggestions.length)
        return
      }
      if (e.key === 'Tab' || (e.key === 'Enter' && !e.shiftKey)) {
        e.preventDefault()
        acceptSuggestion(suggestions[suggestionIdx])
        return
      }
      if (e.key === 'Escape') {
        e.preventDefault()
        setInput('')
        return
      }
    }

    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault()
      sendInstruction()
    }
  }

  return (
    <div className={styles.chatPanel}>
      <div className={styles.header}>
        <span className={styles.title}>智能体</span>
        <span
          className={sseState.connected ? styles.sseDotOn : styles.sseDotOff}
          title={sseState.connected ? 'SSE 已连接' : 'SSE 已断开'}
        />
      </div>

      {/* Quick-command chips — Raycast suggestion style */}
      <div className={styles.quickBar}>
        {(['停止', '状态', '去充电站', '回原点', '报告位置'] as const).map(cmd => (
          <button
            key={cmd}
            className={styles.quickChip}
            onClick={() => {
              setInput(cmd)
              requestAnimationFrame(() => inputRef.current?.focus())
            }}
          >
            {cmd}
          </button>
        ))}
      </div>

      <div className={styles.messages} ref={listRef}>
        {messages.map(msg => {
          const cls =
            msg.role === 'user'      ? styles.bubbleUser      :
            msg.role === 'assistant' ? styles.bubbleAssistant :
                                        styles.bubbleSystem
          return (
            <div key={msg.id} className={cls}>
              <div className={styles.bubbleText}>{msg.text}</div>
              <div className={styles.bubbleTime}>{formatTime(msg.ts)}</div>
            </div>
          )
        })}
        {thinking && (
          <div className={styles.bubbleThinking}>
            <ThinkingBubble hint={thinking.hint} startedAt={thinking.startedAt} />
          </div>
        )}
      </div>

      <div className={styles.inputRow}>
        {showDropdown && (
          <div className={styles.suggestBox} role="listbox">
            <div className={styles.suggestHeader}>
              <span>命令</span>
              <span className={styles.suggestHint}>↑↓ 选择 · Tab 补全 · Esc 关闭</span>
            </div>
            <div className={styles.suggestList}>
              {suggestions.map((c, i) => (
                <button
                  key={c.name}
                  type="button"
                  role="option"
                  aria-selected={i === suggestionIdx}
                  className={i === suggestionIdx ? styles.suggestItemActive : styles.suggestItem}
                  onMouseEnter={() => setSuggestionIdx(i)}
                  onClick={() => acceptSuggestion(c)}
                >
                  <span className={styles.suggestName}>{c.usage}</span>
                  <span className={styles.suggestDesc}>{c.description}</span>
                  {c.group && <span className={styles.suggestGroup}>{c.group}</span>}
                </button>
              ))}
            </div>
          </div>
        )}
        <input
          ref={inputRef}
          className={styles.input}
          type="text"
          placeholder="输入指令或 / 查看命令…"
          value={input}
          onChange={e => setInput(e.target.value)}
          onKeyDown={handleKeyDown}
          disabled={sending}
          aria-label="导航指令"
          autoComplete="off"
          spellCheck={false}
        />
        <button
          className={styles.btnSend}
          onClick={sendInstruction}
          disabled={sending || !input.trim()}
          aria-label="发送"
        >
          <Send size={16} />
        </button>
      </div>
    </div>
  )
}
