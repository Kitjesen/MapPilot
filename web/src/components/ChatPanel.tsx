import { useState, useRef, useEffect, useMemo, useCallback } from 'react'
import { Send, X } from 'lucide-react'
import type { SSEState } from '../types'
import type { InstructionTaskController } from '../hooks/useInstructionTask'
import { presentInstructionTask } from '../services/instructionTask'
import {
  isSlashCommand,
  executeSlashCommand,
  matchSlashCommands,
  type SlashCommandSpec,
} from '../utils/slashCommands'
import { liveNavigationStatus, presentNavigationStatus } from '../services/navigationStatus'
import { ThinkingBubble } from './ThinkingBubble'
import { shouldDisplayAgentMessage } from '../services/agentTask'
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

interface ChatPanelProps {
  sseState: SSEState
  controller: InstructionTaskController
  motionStartAllowed: boolean
  motionStartBlockedReason: string
}

export function ChatPanel({
  sseState,
  controller,
}: ChatPanelProps) {
  const [messages, setMessages] = useState<Message[]>([])
  const liveNavigation = liveNavigationStatus(sseState, Date.now() / 1000)
  const [input, setInput] = useState('')
  const { task: activeTask, receipt, sending, cancel: cancelActiveTask } = controller
  const [suggestionIdx, setSuggestionIdx] = useState(0)
  const [thinking, setThinking] = useState<{ hint: string; startedAt: number } | null>(null)
  const listRef = useRef<HTMLDivElement>(null)
  const inputRef = useRef<HTMLInputElement>(null)
  const prevNavigationRef = useRef<string | null>(null)
  const lastAgentTsRef = useRef<number>(0)

  const taskView = useMemo(
    () => activeTask ? presentInstructionTask(activeTask) : null,
    [activeTask],
  )

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
  }, [messages, thinking, receipt])

  // Report task changes from the same navigation status used by the rest of the UI.
  useEffect(() => {
    const task = liveNavigation?.task
    if (!task) return
    const view = presentNavigationStatus(liveNavigation, 'zh')
    const key = `${task.task_id}:${view.task.state}`
    if (key === prevNavigationRef.current) return
    prevNavigationRef.current = key

    const text = `[${view.task.state}] ${view.task.label}`

    setMessages(prev => [...prev, { id: nextId(), role: 'system', text, ts: Date.now() }])
  }, [liveNavigation])

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
    if (!shouldDisplayAgentMessage(m, activeTask?.runId ?? '')) return

    if (m.role === 'thinking') {
      setThinking(prev => ({
        hint: m.text,
        startedAt: prev?.startedAt ?? Date.now(),
      }))
    } else if (m.role === 'assistant') {
      setMessages(prev => [...prev, {
        id: nextId(),
        role: 'assistant',
        text: `助手说明：${m.text}`,
        ts: Math.floor(m.ts * 1000),
        phase: m.phase,
      }])
      setThinking(null)
    }
  }, [sseState.agentMessage, activeTask])

  const sendInstruction = useCallback(async () => {
    const text = input.trim()
    if (!text || sending) return
    if (isSlashCommand(text)) {
      setInput('')
      setMessages(prev => [...prev, { id: nextId(), role: 'user', text, ts: Date.now() }])
      const response = await executeSlashCommand(text, sseState)
      addSystem(response)
      return
    }
    setThinking({ hint: '发送指令…', startedAt: Date.now() })
    try {
      await controller.submit(text)
      setInput('')
      setMessages(prev => [...prev, { id: nextId(), role: 'user', text, ts: Date.now() }])
    } catch (failure: unknown) {
      addSystem(failure instanceof Error ? failure.message : String(failure))
    } finally {
      setThinking(null)
    }
  }, [input, sending, sseState, addSystem, controller])

  const fillExample = useCallback((example: string) => {
    setInput(example)
    requestAnimationFrame(() => inputRef.current?.focus())
  }, [])

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
    if (e.nativeEvent.isComposing || e.keyCode === 229) return
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
        <div className={styles.titleGroup}>
          <span className={styles.title}>任务</span>
          <span className={styles.subtitle}>用一句话告诉机器人要做什么</span>
        </div>
        <div className={styles.connectionStatus}>
          <span
            className={sseState.connected ? styles.sseDotOn : styles.sseDotOff}
            title={sseState.connected ? '任务反馈已连接' : '任务反馈未连接'}
          />
          <span>{sseState.connected ? '反馈已连接' : '反馈未连接'}</span>
        </div>
      </div>

      <div className={styles.messages} ref={listRef}>
        {messages.length === 0 && !activeTask && !receipt && !thinking && (
          <section className={styles.emptyState} aria-label="任务示例">
            <div className={styles.emptyEyebrow}>新任务</div>
            <h2 className={styles.emptyTitle}>想让机器人做什么？</h2>
            <p className={styles.emptyDescription}>
              描述目标即可。任务受理后，这里会显示真实的执行状态和结果。
            </p>
            <div className={styles.exampleList}>
              <button type="button" onClick={() => fillExample('帮我找一瓶水')}>
                <span>寻找物品</span>
                帮我找一瓶水
              </button>
              <button type="button" onClick={() => fillExample('跟随我')}>
                <span>持续任务</span>
                跟随我
              </button>
            </div>
          </section>
        )}
        {messages.map(msg => {
          const cls =
            msg.role === 'user'      ? styles.bubbleUser      :
            msg.role === 'assistant' ? styles.bubbleAssistant :
                                        styles.bubbleSystem
          return (
            <div key={msg.id} className={cls}>
              <div className={styles.bubbleText}>{msg.text}</div>
              {msg.role !== 'system' && (
                <div className={styles.bubbleTime}>{formatTime(msg.ts)}</div>
              )}
            </div>
          )
        })}
        {receipt && !activeTask && (
          <div className={styles.bubbleAssistant} role="status" aria-live="polite">
            <div className={styles.bubbleText}>{receipt.message}</div>
          </div>
        )}
        {activeTask && taskView && (
          <section className={`${styles.taskCard} ${styles[`taskCard_${taskView.tone}`]}`} aria-live="polite">
            <div className={styles.taskCardHeader}>
              <div>
                <div className={styles.taskEyebrow}>
                  <span className={styles.taskToneDot} aria-hidden="true" />
                  当前任务
                </div>
                <h3 className={styles.taskTitle}>{activeTask.instruction}</h3>
              </div>
              {taskView.canCancel && (
                <button
                  type="button"
                  className={styles.taskCancel}
                  onClick={cancelActiveTask}
                  disabled={activeTask.cancelRequested}
                  aria-label="取消当前任务"
                >
                  <X size={14} />
                  {activeTask.cancelRequested ? '取消中' : '取消'}
                </button>
              )}
            </div>
            <div className={styles.taskState}>
              <div className={styles.taskStateLabel}>
                {activeTask.cancelRequested ? '正在取消任务' : taskView.label}
              </div>
              <div className={styles.taskDetail}>
                {activeTask.cancelRequested
                  ? activeTask.status.motion_attempted
                    ? '取消请求已发送，正在等待运动任务结束。'
                    : '取消请求已发送，正在等待任务结束；当前尚未派发运动。'
                  : taskView.detail}
              </div>
            </div>
            {activeTask.pollError && (
              <div className={styles.taskWarning}>状态暂时无法刷新：{activeTask.pollError}</div>
            )}
            {activeTask.cancelError && (
              <div className={styles.taskWarning}>结束请求未成功：{activeTask.cancelError}</div>
            )}
            <details className={styles.taskMeta}>
              <summary>技术信息</summary>
              <div>任务 ID：{activeTask.runId}</div>
              <div>执行器：{activeTask.executor}</div>
              <div>状态：{activeTask.status.state ?? 'unknown'}</div>
              {(activeTask.status.failure_reason || activeTask.status.navigation?.reason) && (
                <div>原因：{activeTask.status.failure_reason ?? activeTask.status.navigation?.reason}</div>
              )}
            </details>
          </section>
        )}
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
