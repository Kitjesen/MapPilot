import { CheckCircle2, Clock3, LockKeyhole, Monitor, Pause, Play, ShieldAlert, XCircle } from 'lucide-react'
import styles from './ComputerProgressCard.module.css'

export type ComputerProgressState = 'running' | 'awaiting_permission' | 'paused' | 'completed' | 'failed'

export interface ComputerProgressCardProps {
  state: ComputerProgressState
  action: string
  detail?: string
  elapsedMs?: number
  permissionLabel?: string
  error?: string
  onPause?: () => void
  onResume?: () => void
  onRequestPermission?: () => void
}

const stateLabels: Record<ComputerProgressState, string> = {
  running: '正在操作屏幕',
  awaiting_permission: '等待电脑权限',
  paused: '已暂停',
  completed: '操作完成',
  failed: '操作未完成',
}

function formatElapsed(elapsedMs: number | undefined) {
  if (elapsedMs == null || !Number.isFinite(elapsedMs) || elapsedMs < 0) return null
  const seconds = Math.floor(elapsedMs / 1000)
  if (seconds < 60) return `${seconds} 秒`
  const minutes = Math.floor(seconds / 60)
  return `${minutes} 分 ${String(seconds % 60).padStart(2, '0')} 秒`
}

export function ComputerProgressCard({ state, action, detail, elapsedMs, permissionLabel = '屏幕操作', error,
  onPause, onResume, onRequestPermission }: ComputerProgressCardProps) {
  const elapsed = formatElapsed(elapsedMs)
  const Icon = state === 'awaiting_permission' ? LockKeyhole
    : state === 'completed' ? CheckCircle2
      : state === 'failed' ? XCircle : state === 'paused' ? Pause : Monitor
  return <section className={styles.card} data-state={state} aria-label="电脑操作进度">
    <header className={styles.header}>
      <span className={styles.icon} aria-hidden="true"><Icon size={16} /></span>
      <div className={styles.heading}><strong>电脑操作</strong><span>{stateLabels[state]}</span></div>
      {elapsed && <span className={styles.elapsed}><Clock3 size={12} aria-hidden="true" />{elapsed}</span>}
    </header>
    <div className={styles.body}>
      <p className={styles.action}>{action}</p>
      {detail && <p className={styles.detail}>{detail}</p>}
      {state === 'awaiting_permission' && <p className={styles.permission}><ShieldAlert size={14} aria-hidden="true" />需要允许“{permissionLabel}”后继续</p>}
      {state === 'failed' && error && <p className={styles.error}>{error}</p>}
    </div>
    {(state === 'running' && onPause || state === 'paused' && onResume || state === 'awaiting_permission' && onRequestPermission) && <footer className={styles.footer}>
      {state === 'running' && onPause && <button type="button" onClick={onPause}><Pause size={14} aria-hidden="true" />暂停</button>}
      {state === 'paused' && onResume && <button type="button" onClick={onResume}><Play size={14} aria-hidden="true" />继续</button>}
      {state === 'awaiting_permission' && onRequestPermission && <button type="button" className={styles.primary} onClick={onRequestPermission}><LockKeyhole size={14} aria-hidden="true" />请求权限</button>}
    </footer>}
  </section>
}

