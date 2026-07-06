import { X } from 'lucide-react'
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

export function ToastContainer({ toasts, dismiss }: ToastContainerProps) {
  if (toasts.length === 0) return null

  const content = (
    <div className={styles.container} aria-live="polite" aria-atomic="false">
      {toasts.map(t => (
        <div key={t.id} className={kindClass(t.kind)} role="status">
          <span className={styles.msg}>{t.message}</span>
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
