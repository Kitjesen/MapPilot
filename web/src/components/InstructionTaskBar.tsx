import { useRef, useState } from 'react'
import { ArrowUp, Ellipsis, Navigation, Plus, Square, X } from 'lucide-react'
import type { Locale } from '../i18n'
import type { InstructionTaskController } from '../hooks/useInstructionTask'
import { instructionTaskCanDismiss, presentInstructionTask } from '../services/instructionTask'
import styles from './InstructionTaskBar.module.css'

interface InstructionTaskBarProps {
  controller: InstructionTaskController
  locale: Locale
  motionStartAllowed: boolean
  motionStartBlockedReason: string
  hasNativeTask: boolean
}

const ENGLISH_STAGES: Record<string, string> = {
  '指令已受理': 'Instruction accepted', '任务状态暂时无法刷新': 'Task feedback unavailable',
  '无法确认任务状态': 'Task status unconfirmed', '正在等待任务停止确认': 'Waiting for stop confirmation',
  '该指令已不是当前任务': 'This instruction is no longer the current task',
  '正在结束任务': 'Ending task', '任务已完成': 'Task completed', '任务已取消': 'Task cancelled',
  '任务未完成': 'Task did not complete', '执行结果已确认，正在结束任务': 'Result confirmed; finishing task',
  '正在分析搜索位置': 'Evaluating search positions', '正在搜索目标': 'Searching for the target',
  '正在规划观察路线': 'Planning an observation route', '正在重新观察目标': 'Observing the target again',
  '正在确认目标': 'Verifying the target', '正在选择跟随目标': 'Selecting a person to follow',
  '正在持续跟随': 'Following', '正在重新识别跟随目标': 'Reacquiring the person',
  '跟随目标暂时丢失': 'Follow target lost', '正在前往目标': 'Navigating to the target',
  '正在规划路线': 'Planning a route', '正在理解指令': 'Understanding the instruction',
}

export function InstructionTaskBar({ controller, locale, motionStartAllowed,
  motionStartBlockedReason, hasNativeTask }: InstructionTaskBarProps) {
  const [input, setInput] = useState('')
  const [composerOpen, setComposerOpen] = useState(false)
  const inputRef = useRef<HTMLInputElement>(null)
  const { task, receipt, sending, error } = controller
  const view = task ? presentInstructionTask(task) : null
  const text = (en: string, zh: string) => locale === 'zh' ? zh : en

  const send = async () => {
    if (!input.trim() || sending) return
    try {
      await controller.submit(input)
      setInput('')
      setComposerOpen(false)
    } catch {
      // The shared controller keeps the error; preserve the draft for correction.
    }
  }

  const fillExample = (example: string) => {
    setInput(example)
    inputRef.current?.focus()
  }

  const composer = (
    <div className={task ? styles.activeComposer : undefined}>
      <div className={styles.inputRow}>
        <input ref={inputRef} value={input} onChange={event => setInput(event.target.value)}
          placeholder={task ? text('Ask about this task or ask it to stop', '询问进展，或结束当前任务') : text('What would you like the robot to do?', '想让机器人做什么？')}
          aria-label={text('Task instruction', '任务指令')} disabled={sending} autoComplete="off"
          onKeyDown={event => {
            if (event.nativeEvent.isComposing || event.keyCode === 229) return
            if (event.key === 'Enter') { event.preventDefault(); void send() }
            if (event.key === 'Escape' && hasNativeTask) setComposerOpen(false)
          }} />
        <button type="button" className={styles.sendButton} onClick={() => { void send() }}
          disabled={!input.trim() || sending}
          aria-label={text('Send task', '发送任务')}><ArrowUp size={21} /></button>
      </div>
      {sending ? <p className={styles.stage} role="status">{text('Sending instruction…', '正在发送指令…')}</p> : !task && (
        <div className={styles.examples} aria-label={text('Fill an example instruction', '填入示例指令')}>
          <button type="button" onClick={() => fillExample(text('Follow me', '跟随我'))}>{text('Follow me', '跟随我')}</button>
          <button type="button" onClick={() => fillExample(text('Help me find a bottle of water', '帮我找一瓶水'))}>{text('Find a bottle of water', '帮我找一瓶水')}</button>
        </div>
      )}
      {!motionStartAllowed && motionStartBlockedReason && <p className={styles.notice}>{motionStartBlockedReason}</p>}
      {error && <p className={styles.notice} role="alert">{error}</p>}
    </div>
  )

  if (task && view) {
    const canDismiss = instructionTaskCanDismiss(task)
    const stage = locale === 'zh' ? view.label : ENGLISH_STAGES[view.label] ?? view.label
    return (
      <section className={`${styles.bar} ${view.tone === 'error' ? styles.error : view.tone === 'warning' ? styles.warning : ''}`}
        data-instruction-bar data-state={canDismiss ? 'terminal' : task.cancelRequested ? 'stopping' : 'active'}
        aria-label={text('Current instruction task', '当前指令任务')}>
        <div className={styles.taskRow}>
          <Navigation className={styles.taskIcon} size={23} aria-hidden="true" />
          <div className={styles.taskCopy} aria-live="polite">
            <div className={styles.taskTitle} title={task.instruction}>{task.instruction}</div>
            <div className={styles.stage}>{stage}</div>
          </div>
          {view.canCancel && (
            <button type="button" className={styles.endButton} onClick={() => { void controller.cancel() }}
              disabled={task.cancelRequested}>
              <Square size={14} aria-hidden="true" />{text('End task', '结束任务')}
            </button>
          )}
          {canDismiss && (
            <button type="button" className={styles.iconButton} onClick={controller.dismiss}
              aria-label={text('Dismiss task result', '关闭任务结果')}><X size={18} /></button>
          )}
        </div>
        {task.cancelError && <p className={styles.notice} role="alert">{text('End request failed: ', '结束请求未成功：')}{task.cancelError}</p>}
        {(task.cancelRequested || (!view.terminal && task.status.motion_attempted === true
          && ['failed', 'cancelled'].includes(task.status.state ?? ''))) && (
          <p className={styles.notice}>{text('The motion task has not yet confirmed its stop.', '运动任务停止尚未确认。')}</p>
        )}
        <details className={styles.details} name="lingtu-menu" onKeyDown={event => {
          if (event.key !== 'Escape') return
          event.currentTarget.open = false
          event.currentTarget.querySelector('summary')?.focus()
          event.stopPropagation()
        }}>
          <summary aria-label={text('Task details', '任务详情')} title={text('Task details', '任务详情')}><Ellipsis size={21} aria-hidden="true" /></summary>
          <div className={styles.detailBody}>
            <p className={styles.fullInstruction}>{task.instruction}</p>
            <p>{locale === 'zh' ? view.detail : task.status.summary ?? stage}</p>
            {locale === 'zh' && task.status.summary && <p>{task.status.summary}</p>}
            <dl>
              {task.pollError && <div><dt>{text('Connection', '连接信息')}</dt><dd>{task.pollError}</dd></div>}
              <div><dt>{text('Feedback', '反馈来源')}</dt><dd>{task.statusSource === 'ack'
                ? text('Request acceptance only', '仅请求受理') : text('Task status query', '任务状态查询')}</dd></div>
              <div><dt>Agent run_id</dt><dd>{task.runId}</dd></div>
              <div><dt>{text('Executor', '执行器')}</dt><dd>{task.executor}</dd></div>
              <div><dt>{text('State', '状态')}</dt><dd>{task.status.state ?? text('Unknown', '未知')}</dd></div>
              {task.status.navigation?.state && <div><dt>{text('Domain state', '领域状态')}</dt><dd>{task.status.navigation.state}</dd></div>}
              {task.status.navigation?.task_id && <div><dt>task_id</dt><dd>{task.status.navigation.task_id}</dd></div>}
              {task.status.navigation?.follow_task_id && <div><dt>follow_task_id</dt><dd>{task.status.navigation.follow_task_id}</dd></div>}
              {(task.status.failure_reason || task.status.reason || task.status.navigation?.reason) && (
                <div><dt>{text('Reason', '原因')}</dt><dd>{task.status.failure_reason ?? task.status.reason ?? task.status.navigation?.reason}</dd></div>
              )}
            </dl>
          </div>
        </details>
        {composer}
      </section>
    )
  }

  if (receipt) {
    return (
      <section className={styles.bar} data-instruction-bar data-state="receipt" aria-label={text('Instruction receipt', '指令受理结果')}>
        <div className={styles.taskRow}>
          <div className={styles.taskCopy} aria-live="polite">
            <div className={styles.taskTitle} title={receipt.instruction}>{receipt.instruction}</div>
            <p className={styles.stage}>{text(receipt.operation === 'start' ? 'Accepted; execution result is not confirmed.' : receipt.message, receipt.message)}</p>
          </div>
          <button type="button" className={styles.iconButton} onClick={controller.dismiss}
            aria-label={text('Dismiss receipt', '关闭受理结果')}><X size={18} /></button>
        </div>
        {composer}
      </section>
    )
  }

  const compact = hasNativeTask && !composerOpen && !sending
  if (compact) {
    return <button type="button" className={styles.newTask} data-instruction-bar data-state="idle" onClick={() => setComposerOpen(true)}>
      <Plus size={16} aria-hidden="true" />{text('New task', '新任务')}
    </button>
  }

  return (
    <section className={`${styles.bar} ${styles.composer}`} data-instruction-bar data-state={sending ? 'sending' : 'idle'} aria-label={text('New instruction task', '新指令任务')}>
      <div className={styles.composerHeading}>
        <Navigation size={19} aria-hidden="true" /><span>{text('New task', '新任务')}</span>
        {hasNativeTask && <button type="button" className={styles.iconButton}
          onClick={() => setComposerOpen(false)} aria-label={text('Close new task', '收起新任务')}><X size={17} /></button>}
      </div>
      {composer}
    </section>
  )
}
