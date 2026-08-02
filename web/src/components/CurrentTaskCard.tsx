import { Ban, Clock3, Navigation2, Pause, Play, X } from 'lucide-react'
import { useEffect, useState } from 'react'
import { useCurrentNavigationTask } from '../hooks/useCurrentNavigationTask'
import { text, type Locale } from '../i18n'
import * as api from '../services/api'
import {
  describeNavigationTask,
  isTaskControlAwaitingConfirmation,
  type NavigationTaskControlRequest,
} from '../services/navigationTaskLifecycle'
import type { ToastKind } from '../types'
import styles from './CurrentTaskCard.module.css'

interface CurrentTaskCardProps {
  locale: Locale
  showToast: (message: string, kind?: ToastKind) => void
  resumeAllowed?: boolean
  resumeBlockedReason?: string
}

type StateTone = 'active' | 'success' | 'danger' | 'muted' | 'waiting'

const STATE_LABELS: Record<string, { en: string; zh: string; tone: StateTone }> = {
  PLANNING: { en: 'Planning', zh: '规划中', tone: 'active' },
  EXECUTING: { en: 'Navigating', zh: '导航中', tone: 'active' },
  PAUSED: { en: 'Paused', zh: '已暂停', tone: 'waiting' },
  RECOVERING: { en: 'Recovering', zh: '恢复中', tone: 'waiting' },
  SUCCESS: { en: 'Reached', zh: '已到达', tone: 'success' },
  FAILED: { en: 'Failed', zh: '失败', tone: 'danger' },
  CANCELLED: { en: 'Cancelled', zh: '已取消', tone: 'muted' },
}

function shortTaskId(taskId: string): string {
  return taskId.length > 10 ? `${taskId.slice(0, 8)}...` : taskId
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

export function CurrentTaskCard({
  locale,
  showToast,
  resumeAllowed = true,
  resumeBlockedReason = '',
}: CurrentTaskCardProps) {
  const { identity, snapshot, loading, stale, error, dismiss } = useCurrentNavigationTask()
  const [now, setNow] = useState(() => Date.now() / 1000)
  const [controlRequest, setControlRequest] = useState<NavigationTaskControlRequest | null>(null)
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

  if (!identity) return null
  const knownState = STATE_LABELS[task.state]

  const terminal = task.terminal
  const state = !snapshot && stale
    ? { label: text(locale, 'Status unavailable', '状态不可用'), tone: 'danger' as StateTone }
    : !snapshot
      ? { label: text(locale, 'Submitted', '已提交'), tone: 'waiting' as StateTone }
      : !snapshot.found || !status
        ? { label: text(locale, 'Waiting for native status', '等待原生状态'), tone: 'waiting' as StateTone }
        : knownState
          ? { label: text(locale, knownState.en, knownState.zh), tone: knownState.tone }
          : { label: text(locale, 'Unknown', '未知'), tone: 'muted' as StateTone }
  const reason = cleanText(status?.reason)
    || cleanText(snapshot?.reason)
    || (loading ? text(locale, 'Awaiting the first lifecycle update', '等待首个生命周期更新') : '-')
  const updatedAt = Number(status?.ts ?? snapshot?.ts ?? identity.submitted_at)
  const freshness = formatAge(updatedAt, now, locale)

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
    if (!task.canResume || pendingAction !== null || cancelState !== 'idle') return
    if (!resumeAllowed) {
      showToast(
        resumeBlockedReason || text(locale, 'Robot state cannot safely resume motion', '当前状态不能安全恢复运动'),
        'error',
      )
      return
    }
    if (!window.confirm(text(
      locale,
      'Resume this paused task? The robot may begin moving after native validation succeeds.',
      '恢复这个已暂停任务？原生校验通过后，机器人可能开始移动。',
    ))) return

    setControlRequest({
      taskId: identity.task_id,
      action: 'resume',
      state: 'requesting',
    })
    try {
      const response = await api.resumeNavigationTask(identity.task_id)
      if (!response.ok || response.status !== 'resume_requested') {
        setControlRequest(null)
        showToast(api.formatCommandAck(response, text(locale, 'Task resume', '恢复任务')), 'error')
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
          'Resume accepted; rechecking localization, map, and safety',
          '恢复请求已受理，正在重新检查定位、地图和安全状态',
        ),
        'info',
      )
    } catch (resumeError: unknown) {
      setControlRequest(null)
      showToast(
        api.formatCommandError(resumeError, text(locale, 'Task resume failed', '恢复任务失败')),
        'error',
      )
    }
  }
  const requestCancel = async () => {
    if (terminal || cancelState !== 'idle') return
    if (!window.confirm(text(
      locale,
      'Request cancellation of this task? The task is not cancelled until the robot confirms it has stopped.',
      '请求取消这个任务？只有机器人确认停车后，任务才会变为“已取消”。',
    ))) return

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

  return (
    <aside
      className={`${styles.card} ${stale ? styles.cardStale : ''}`}
      aria-label={text(locale, 'Current navigation task', '当前导航任务')}
    >
      <header className={styles.header}>
        <div className={styles.heading}>
          <span className={styles.icon} aria-hidden="true">
            <Navigation2 size={15} strokeWidth={2} />
          </span>
          <span>{text(locale, 'Current navigation task', '当前导航任务')}</span>
        </div>
        {terminal && (
          <button
            type="button"
            className={styles.dismiss}
            onClick={dismiss}
            aria-label={text(locale, 'Dismiss completed task', '关闭已结束任务')}
            title={text(locale, 'Dismiss', '关闭')}
          >
            <X size={15} />
          </button>
        )}
      </header>

      <div className={styles.stateRow} aria-live="polite">
        <span className={`${styles.stateDot} ${styles[state.tone]}`} aria-hidden="true" />
        <span className={styles.stateLabel}>{state.label}</span>
        {stale && (
          <span
            className={styles.staleBadge}
            title={error || text(locale, 'The latest refresh failed', '最近一次刷新失败')}
          >
            {text(locale, 'STALE', '已过期')}
          </span>
        )}
      </div>

      <div className={styles.reasonRow}>
        <span className={styles.label}>{text(locale, 'Reason', '原因')}</span>
        <span className={styles.reason} title={reason}>{reason}</span>
      </div>

      {!terminal && (
        <div className={styles.actions}>
          {task.canPause && (
            <button
              type="button"
              className={styles.pauseButton}
              onClick={() => void requestPause()}
              disabled={pendingAction !== null || cancelState !== 'idle'}
              title={text(locale, 'Request a confirmed stop while keeping this task resumable', '请求确认停车，并保留任务以便恢复')}
            >
              <Pause size={13} aria-hidden="true" />
              {pendingAction !== 'pause'
                ? text(locale, 'Pause', '暂停')
                : pendingState === 'requested'
                  ? text(locale, 'Waiting for stop confirmation...', '等待停稳确认...')
                  : text(locale, 'Requesting pause...', '正在请求暂停...')}
            </button>
          )}
          {task.canResume && (
            <button
              type="button"
              className={styles.resumeButton}
              onClick={() => void requestResume()}
              disabled={pendingAction !== null || cancelState !== 'idle' || !resumeAllowed}
              title={resumeAllowed
                ? text(locale, 'Resume the same task after native validation', '通过原生校验后恢复同一任务')
                : resumeBlockedReason || text(locale, 'Resume is blocked by robot state', '机器人状态阻止恢复')}
            >
              <Play size={13} aria-hidden="true" />
              {pendingAction !== 'resume'
                ? text(locale, 'Resume', '恢复')
                : pendingState === 'requested'
                  ? text(locale, 'Rechecking safety...', '正在重新检查安全状态...')
                  : text(locale, 'Requesting resume...', '正在请求恢复...')}
            </button>
          )}
          <button
            type="button"
            className={styles.cancelButton}
            onClick={() => void requestCancel()}
            disabled={cancelState !== 'idle'}
          >
            <Ban size={13} aria-hidden="true" />
            {cancelState === 'requested'
              ? text(locale, 'Waiting for stop...', '等待停车确认...')
              : cancelState === 'requesting'
                ? text(locale, 'Requesting...', '请求中...')
                : text(locale, 'Cancel task', '取消任务')}
          </button>
        </div>
      )}

      <footer className={styles.meta}>
        <span title={identity.task_id}>
          {text(locale, 'Task', '任务')} <code>#{shortTaskId(identity.task_id)}</code>
        </span>
        <span className={styles.freshness}>
          <Clock3 size={11} aria-hidden="true" />
          {text(locale, 'Updated', '更新于')} {freshness}
        </span>
      </footer>
    </aside>
  )
}