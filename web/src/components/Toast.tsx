import { AlertTriangle, CheckCircle2, Info, X } from 'lucide-react'
import { createPortal } from 'react-dom'
import type { Toast, ToastKind } from '../types'
import styles from './Toast.module.css'

interface ToastContainerProps {
  toasts: Toast[]
  dismiss: (id: number) => void
}

function kindClass(kind: ToastKind) {
  if (kind === 'success') return styles.toastSuccess
  if (kind === 'error') return styles.toastError
  return styles.toastInfo
}

function kindTitle(kind: ToastKind) {
  if (kind === 'success') return '已完成'
  if (kind === 'error') return '需要处理'
  return '提示'
}

function kindIcon(kind: ToastKind) {
  if (kind === 'success') return <CheckCircle2 size={16} aria-hidden="true" />
  if (kind === 'error') return <AlertTriangle size={16} aria-hidden="true" />
  return <Info size={16} aria-hidden="true" />
}

export function ToastContainer({ toasts, dismiss }: ToastContainerProps) {
  if (toasts.length === 0) return null

  const content = (
    <div className={styles.container} aria-live="polite" aria-atomic="false">
      {toasts.map(t => (
        <div key={t.id} className={kindClass(t.kind)} role={t.kind === 'error' ? 'alert' : 'status'}>
          <span className={styles.icon}>{kindIcon(t.kind)}</span>
          <span className={styles.content}>
            <strong className={styles.title}>{kindTitle(t.kind)}</strong>
            <span className={styles.msg}>{t.message}</span>
          </span>
          <button
            className={styles.close}
            onClick={() => dismiss(t.id)}
            aria-label="Close notification"
          >
            <X size={12} />
          </button>
        </div>
      ))}
    </div>
  )

  return createPortal(content, document.body)
}
