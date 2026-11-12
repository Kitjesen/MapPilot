import { Ban, Ellipsis, LoaderCircle, Route, Pause, Play, X } from 'lucide-react'
import { useEffect, useState } from 'react'
import { createPortal } from 'react-dom'
import { useCurrentNavigationTask } from '../hooks/useCurrentNavigationTask'
import { text, type Locale } from '../i18n'
import * as api from '../services/api'
import {
  describeNavigationTask,
  isTaskControlAwaitingConfirmation,
  type NavigationTaskControlRequest,
} from '../services/navigationTaskLifecycle'
import { navigationTaskLabel, navigationTaskReasonMessage, operatorReasonMessage } from '../services/navigationStatus'
import type { ToastKind } from '../types'
import { ConfirmModal } from './Modal'
import styles from './CurrentTaskCard.module.css'

interface CurrentTaskCardProps {
  container?: HTMLElement | null
  locale: Locale
  showToast: (message: string, kind?: ToastKind) => void
  taskName?: string
  resumeAllowed?: boolean
  resumeBlockedReason?: string
}

type StateTone = 'active' | 'success' | 'danger' | 'muted' | 'waiting'

const STATE_LABELS: Record<string, { tone: StateTone }> = {
  IDLE: { tone: 'muted' },
  PLANNING: { tone: 'active' },
  EXECUTING: { tone: 'active' },
  PAUSED: { tone: 'waiting' },
  RECOVERING: { tone: 'waiting' },
  SUCCESS: { tone: 'success' },
  FAILED: { tone: 'danger' },
  CANCELLED: { tone: 'muted' },
  UNKNOWN: { tone: 'muted' },
}

function cleanText(value: unknown): string {
  return typeof value === 'string' ? value.trim() : ''
}

function formatAge(timestamp: number, now: number, locale: Locale): string {
  if (!Number.isFinite(timestamp) || timestamp <= 0) return text(locale, 'unknown', '未知')
  const seconds = Math.max(0, Math.floor(now - timestamp))
  if (seconds < 5) return text(locale, 'just now', '刚刚')
  if (seconds < 60) return text(locale, `${seconds}s ago`, `${seconds} 秒前`)
  const minutes = Math.floor(seconds / 60)
  if (minutes < 60) return text(locale, `${minutes}m ago`, `${minutes} 分钟前`)
  const hours = Math.floor(minutes / 60)
  return text(locale, `${hours}h ago`, `${hours} 小时前`)
}

function nextActionLabel(
  state: string,
  locale: Locale,
  options: {
    terminal: boolean
    stale: boolean
    snapshotFound: boolean
    waitingForRoute: boolean
    pendingAction: 'pause' | 'resume' | null
    pendingState: 'idle' | 'requesting' | 'requested'
    cancelState: 'idle' | 'requesting' | 'requested'
    resumeAllowed: boolean
    resumeBlockedReason: string
  },
): string {
  if (options.terminal) {
    switch (state) {
      case 'SUCCESS':
        return text(locale, 'Task complete', '任务已完成')
      case 'FAILED':
        return text(locale, 'Review the reason and choose another target', '查看原因并重新选择目标')
      case 'CANCELLED':
        return text(locale, 'Task cancelled', '任务已取消')
      default:
        return text(locale, 'Task ended', '任务已结束')
    }
  }
  if (options.stale) return text(locale, 'Waiting for task status to reconnect', '等待任务状态恢复连接')
  if (options.cancelState !== 'idle') {
    return options.cancelState === 'requested'
      ? text(locale, 'Waiting for the robot to confirm it has stopped', '等待机器人确认停车')
      : text(locale, 'Sending the cancellation request', '正在发送取消请求')
  }
  if (options.pendingAction === 'pause') {
    return options.pendingState === 'requested'
      ? text(locale, 'Waiting for the robot to confirm it has stopped', '等待机器人确认停稳')
      : text(locale, 'Sending the pause request', '正在发送暂停请求')
  }
  if (options.pendingAction === 'resume') {
    return options.pendingState === 'requested'
      ? text(locale, 'Waiting for the robot to confirm continuation', '等待机器人确认继续导航')
      : text(locale, 'Sending the continuation request', '正在发送继续请求')
  }
  if (!options.snapshotFound) return text(locale, 'Waiting for the robot to confirm the task', '等待机器人确认任务')
  if (options.waitingForRoute) return text(locale, 'Waiting for a new executable route', '等待新的可执行路线')

  switch (state) {
    case 'PLANNING':
      return text(locale, 'Waiting for route planning to finish', '等待路线规划完成')
    case 'EXECUTING':
      return text(locale, 'Waiting for the robot to reach the target', '等待机器人到达目标')
    case 'PAUSED':
      return options.resumeAllowed
        ? text(locale, 'Continue navigation or cancel this task', '继续导航或取消任务')
        : options.resumeBlockedReason || text(locale, 'Waiting until navigation can continue', '等待导航恢复到可继续状态')
    case 'RECOVERING':
      return text(locale, 'Waiting for the recovery result', '等待恢复结果')
    default:
      return text(locale, 'Waiting for a valid navigation state', '等待有效的导航状态')
  }
}

export function CurrentTaskCard({
  container,
  locale,
  showToast,
  taskName = '',
  resumeAllowed = true,
  resumeBlockedReason = '',
}: CurrentTaskCardProps) {
  const { identity, snapshot, loading, stale, error, dismiss } = useCurrentNavigationTask()
  const [now, setNow] = useState(() => Date.now() / 1000)
  const [controlRequest, setControlRequest] = useState<NavigationTaskControlRequest | null>(null)
  const [confirmation, setConfirmation] = useState<{ taskId: string; action: 'resume' | 'cancel' } | null>(null)
  const [cancelRequest, setCancelRequest] = useState<{
    taskId: string
    state: 'requesting' | 'requested'
  } | null>(null)
  const status = snapshot?.found ? snapshot.status ?? null : null
  const reportedStateName = cleanText(status?.state_name).toUpperCase()
  const task = describeNavigationTask({
    lifecycle_state_name: status?.lifecycle_state_name,
    state_name: reportedStateName || undefined,
    state: status?.state,
    terminal: status?.terminal,
  })
  const currentTaskId = identity?.task_id ?? ''
  const controlAwaiting = controlRequest !== null
    && isTaskControlAwaitingConfirmation(controlRequest, currentTaskId, task)
  const pendingAction = controlAwaiting ? controlRequest.action : null
  const pendingState = controlAwaiting ? controlRequest.state : 'idle'
  const cancelState = identity && cancelRequest?.taskId === identity.task_id
    ? cancelRequest.state
    : 'idle'
  useEffect(() => {
    if (!identity) return undefined
    const timer = window.setInterval(() => setNow(Date.now() / 1000), 1_000)
    return () => window.clearInterval(timer)
  }, [identity])

  useEffect(() => {
    if (!controlRequest || controlAwaiting) return undefined
    const timer = window.setTimeout(() => {
      setControlRequest(current => current === controlRequest ? null : current)
    }, 0)
    return () => window.clearTimeout(timer)
  }, [controlAwaiting, controlRequest])

  useEffect(() => {
    if (stale || !task.terminal || !['SUCCESS', 'CANCELLED'].includes(task.state)) return
    const timer = window.setTimeout(dismiss, 8000)
    return () => window.clearTimeout(timer)
  }, [currentTaskId, dismiss, stale, task.state, task.terminal])

  if (!identity) return null
  const knownState = STATE_LABELS[task.state]

  const terminal = task.terminal
  const rawReason = cleanText(status?.reason)
    || cleanText(snapshot?.reason)
    || (loading ? text(locale, 'Waiting for task feedback', '等待任务状态更新') : '-')
  const waitingForRoute = !terminal && ['EXECUTING', 'RECOVERING'].includes(task.state)
    && rawReason === 'replan_waiting'
  const state = stale
    ? { label: text(locale, 'Task feedback interrupted', '任务反馈中断'), tone: 'waiting' as StateTone }
    : !snapshot
      ? { label: text(locale, 'Awaiting robot confirmation', '等待机器人确认'), tone: 'waiting' as StateTone }
      : !snapshot.found || !status
        ? { label: text(locale, 'Awaiting robot confirmation', '等待机器人确认'), tone: 'waiting' as StateTone }
        : knownState
          ? { label: navigationTaskLabel(task.state, locale, rawReason), tone: waitingForRoute ? 'waiting' as StateTone : knownState.tone }
          : { label: text(locale, 'Unknown', '未知'), tone: 'muted' as StateTone }
  const reason = operatorReasonMessage(navigationTaskReasonMessage(rawReason), locale)
  const showReason = !waitingForRoute && reason !== '-' && Boolean(reason) && (
    stale || !snapshot?.found || ['PAUSED', 'RECOVERING', 'FAILED', 'UNKNOWN'].includes(task.state)
  )
  const visibleReason = !stale && snapshot?.found && (
    task.state === 'FAILED'
    || (task.state === 'RECOVERING' && !waitingForRoute)
    || (task.state === 'PAUSED' && !resumeAllowed)
  )
    ? (task.state === 'PAUSED' && resumeBlockedReason) || (showReason ? reason : '')
    : ''
  const updatedAt = Number(status?.ts ?? snapshot?.ts ?? identity.submitted_at)
  const freshness = formatAge(updatedAt, now, locale)
  const snapshotSource = cleanText((snapshot as { source?: unknown } | null)?.source)
  const evidenceStatus = cleanText((snapshot as { evidence_status?: unknown } | null)?.evidence_status)
  const nextAction = nextActionLabel(task.state, locale, {
    terminal,
    stale,
    snapshotFound: Boolean(snapshot?.found && status),
    waitingForRoute,
    pendingAction,
    pendingState,
    cancelState,
    resumeAllowed,
    resumeBlockedReason,
  })

  const requestPause = async () => {
    if (!task.canPause || pendingAction !== null || cancelState !== 'idle') return

    setControlRequest({
      taskId: identity.task_id,
      action: 'pause',
      state: 'requesting',
    })
    try {
      const response = await api.pauseNavigationTask(identity.task_id)
      if (!response.ok || response.status !== 'pause_requested') {
        setControlRequest(null)
        showToast(api.formatCommandAck(response, text(locale, 'Task pause', '暂停任务')), 'error')
        return
      }
      setControlRequest({
        taskId: identity.task_id,
        action: 'pause',
        state: 'requested',
      })
      showToast(
        text(
          locale,
          'Pause requested; waiting for the robot to confirm it has stopped',
          '暂停请求已受理，等待机器人确认停稳',
        ),
        'info',
      )
    } catch (pauseError: unknown) {
      setControlRequest(null)
      showToast(
        api.formatCommandError(pauseError, text(locale, 'Task pause failed', '暂停任务失败')),
        'error',
      )
    }
  }
  const requestResume = async () => {
    if (stale || !snapshot?.found || !task.canResume || pendingAction !== null || cancelState !== 'idle') return
    if (!resumeAllowed) {
      showToast(
        resumeBlockedReason || text(locale, 'Robot state cannot safely resume motion', '当前状态暂不能继续导航'),
        'error',
      )
      return
    }
    setControlRequest({
      taskId: identity.task_id,
      action: 'resume',
      state: 'requesting',
    })
    try {
      const response = await api.resumeNavigationTask(identity.task_id)
      if (!response.ok || response.status !== 'resume_requested') {
        setControlRequest(null)
        showToast(api.formatCommandAck(response, text(locale, 'Task resume', '继续导航')), 'error')
        return
      }
      setControlRequest({
        taskId: identity.task_id,
        action: 'resume',
        state: 'requested',
      })
      showToast(
        text(
          locale,
          'Continuation requested; waiting for the robot to confirm',
          '继续请求已受理，等待机器人确认',
        ),
        'info',
      )
    } catch (resumeError: unknown) {
      setControlRequest(null)
      showToast(
        api.formatCommandError(resumeError, text(locale, 'Task resume failed', '继续导航失败')),
        'error',
      )
    }
  }
  const requestCancel = async () => {
    if (terminal || cancelState !== 'idle') return
    setCancelRequest({ taskId: identity.task_id, state: 'requesting' })
    try {
      const response = await api.cancelNavigationTask(identity.task_id)
      if (!response.ok || response.status !== 'cancel_requested') {
        setCancelRequest(null)
        showToast(api.formatCommandAck(response, text(locale, 'Task cancel', '取消任务')), 'error')
        return
      }
      setCancelRequest({ taskId: identity.task_id, state: 'requested' })
      showToast(
        text(
          locale,
          'Cancel requested; waiting for stop confirmation',
          '已请求取消，正在等待停车确认',
        ),
        'info',
      )
    } catch (cancelError: unknown) {
      setCancelRequest(null)
      showToast(
        api.formatCommandError(cancelError, text(locale, 'Task cancel failed', '取消任务失败')),
        'error',
      )
    }
  }

  const confirmAction = () => {
    if (!confirmation || confirmation.taskId !== identity.task_id) return
    setConfirmation(null)
    if (confirmation.action === 'resume') void requestResume()
    else void requestCancel()
  }

  const displayLabel = !stale && !terminal && cancelState !== 'idle'
    ? cancelState === 'requesting'
      ? text(locale, 'Requesting cancellation', '正在请求取消')
      : text(locale, 'Awaiting stop confirmation', '等待停车确认')
    : !stale && !terminal && pendingAction === 'pause'
      ? text(locale, 'Awaiting pause confirmation', '等待停稳确认')
      : !stale && !terminal && pendingAction === 'resume'
        ? text(locale, 'Awaiting continuation', '等待继续确认')
        : state.label
  const displayTaskName = cleanText(taskName) || text(locale, 'Navigation task', '导航任务')
  const controlsPending = pendingAction !== null || cancelState !== 'idle'
  const visualState = stale || !snapshot?.found || !status ? 'unknown'
    : task.state === 'FAILED' ? 'failed'
      : terminal ? 'idle'
        : pendingAction !== null || cancelState !== 'idle' || waitingForRoute ? 'waiting'
          : task.state === 'EXECUTING' ? 'executing'
            : task.state === 'PAUSED' ? 'paused' : 'idle'

  const card = (
    <aside
      className={`${styles.card} ${container ? styles.embedded : styles.floating}`}
      aria-label={text(locale, 'Current navigation task', '当前导航任务')}
      data-visual-state={visualState}
    >
      <span className={styles.flow} aria-hidden="true" />
      <div className={styles.taskIdentity}>
        <span className={styles.taskIcon} aria-hidden="true">
          <Route size={21} strokeWidth={1.7} className={styles[state.tone]} aria-hidden="true" />
        </span>
        <span className={styles.taskCopy}>
          <span className={styles.taskHeading}>
            <strong className={styles.taskName} title={displayTaskName}>{displayTaskName}</strong>
            <span className={styles.stateLabel} aria-live="polite">{displayLabel}</span>
          </span>
          {visibleReason && <span className={styles.visibleReason} title={visibleReason} aria-live="polite">{visibleReason}</span>}
        </span>
      </div>

      {!terminal && task.canPause && (
        <button
          type="button"
          className={styles.primaryControl}
          onClick={() => void requestPause()}
          disabled={controlsPending}
          aria-busy={controlsPending}
          aria-label={text(locale, 'Pause task', '暂停任务')}
          title={text(locale, 'Pause task', '暂停任务')}
        >
          {controlsPending ? <LoaderCircle size={18} className={styles.pendingIcon} aria-hidden="true" /> : <Pause size={19} aria-hidden="true" />}
        </button>
      )}
      {!terminal && task.canResume && (
        <button
          type="button"
          className={styles.primaryControl}
          onClick={() => setConfirmation({ taskId: identity.task_id, action: 'resume' })}
          disabled={controlsPending || !resumeAllowed}
          aria-busy={controlsPending}
          aria-label={text(locale, 'Continue navigation', '继续导航')}
          title={resumeAllowed
            ? text(locale, 'Continue navigation', '继续导航')
            : resumeBlockedReason || text(locale, 'Resume is blocked by robot state', '机器人状态暂不支持继续导航')}
        >
          {controlsPending ? <LoaderCircle size={18} className={styles.pendingIcon} aria-hidden="true" /> : <Play size={19} fill="currentColor" aria-hidden="true" />}
        </button>
      )}

      {!terminal && task.canCancel && (
        <button
          type="button"
          className={styles.cancelButton}
          onClick={() => setConfirmation({ taskId: identity.task_id, action: 'cancel' })}
          disabled={cancelState !== 'idle'}
          aria-busy={cancelState !== 'idle'}
          aria-label={text(locale, 'Cancel task', '取消任务')}
          title={text(locale, 'Cancel task', '取消任务')}
        >
          {cancelState !== 'idle'
            ? <LoaderCircle size={17} className={styles.pendingIcon} aria-hidden="true" />
            : <Ban size={17} aria-hidden="true" />}
        </button>
      )}

      <details key={identity.task_id} className={styles.disclosure}>
        <summary className={styles.moreButton} title={text(locale, 'Task details', '查看任务详情')}>
          <Ellipsis size={21} aria-hidden="true" />
          <span className={styles.srOnly}>{text(locale, 'Task details', '查看任务详情')}</span>
        </summary>
        <section className={styles.panel} aria-label={text(locale, 'Task details', '任务详情')}>
          <p className={styles.guidance}>{nextAction}</p>
          {showReason && !stale && snapshot?.found && reason !== nextAction && (
            <p className={styles.reason}>{reason}</p>
          )}
          {!terminal && task.canResume && !resumeAllowed && resumeBlockedReason && nextAction !== resumeBlockedReason && (
            <p className={styles.reason}>{resumeBlockedReason}</p>
          )}
          <dl className={styles.meta}>
            <div><dt>{text(locale, 'Updated', '最近更新')}</dt><dd>{freshness}</dd></div>
            <div><dt>{text(locale, 'Task ID', '任务 ID')}</dt><dd><code>{identity.task_id}</code></dd></div>
            {identity.request_id && (
              <div><dt>{text(locale, 'Request ID', '请求 ID')}</dt><dd><code>{identity.request_id}</code></dd></div>
            )}
            {snapshotSource && (
              <div><dt>{text(locale, 'Status source', '状态来源')}</dt><dd><code>{snapshotSource}</code></dd></div>
            )}
            {evidenceStatus && (
              <div><dt>{text(locale, 'Evidence', '状态证据')}</dt><dd><code>{evidenceStatus}</code></dd></div>
            )}
            <div><dt>{text(locale, 'Raw reason', '原始原因')}</dt><dd><code>{rawReason}</code></dd></div>
            {error && <div><dt>{text(locale, 'Connection', '连接信息')}</dt><dd>{error}</dd></div>}
          </dl>

        </section>
      </details>
      {terminal && (
        <button
          type="button"
          className={styles.moreButton}
          onClick={dismiss}
          aria-label={text(locale, 'Dismiss completed task', '关闭已结束任务')}
          title={text(locale, 'Dismiss', '关闭')}
        >
          <X size={16} aria-hidden="true" />
        </button>
      )}
      <ConfirmModal
        className={styles.confirmation}
        open={confirmation?.taskId === identity.task_id && !terminal}
        title={confirmation?.action === 'resume'
          ? text(locale, 'Continue navigation?', '继续导航？')
          : text(locale, 'Cancel this task?', '取消当前任务？')}
        icon={confirmation?.action === 'resume' ? <Play size={20} /> : <Ban size={20} />}
        message={confirmation?.action === 'resume'
          ? (!resumeAllowed && resumeBlockedReason) || text(locale, 'The robot may start moving after confirmation.', '确认后，机器人可能开始移动。')
          : text(locale, 'The task ends once the robot confirms it has stopped.', '机器人确认停车后，任务才会结束。')}
        confirmLabel={confirmation?.action === 'resume'
          ? text(locale, 'Continue navigation', '继续导航')
          : text(locale, 'Cancel task', '取消任务')}
        cancelLabel={text(locale, 'Back', '返回')}
        confirmDisabled={confirmation?.action === 'resume'
          ? stale || !snapshot?.found || !task.canResume || controlsPending || !resumeAllowed
          : cancelState !== 'idle'}
        onCancel={() => setConfirmation(null)}
        onConfirm={confirmAction}
      />
    </aside>
  )
  return createPortal(card, container ?? document.body)
}
