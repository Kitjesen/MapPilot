import { useEffect, useRef, useState } from 'react'
import { createPortal } from 'react-dom'
import { X, Save, Trash2, AlertTriangle, Pencil } from 'lucide-react'
import styles from './Modal.module.css'

// ── Base modal shell ────────────────────────────────────────────

interface ModalProps {
  open: boolean
  title: string
  icon?: React.ReactNode
  danger?: boolean
  onClose: () => void
  children: React.ReactNode
  footer?: React.ReactNode
}

function ModalShell({ open, title, icon, danger, onClose, children, footer }: ModalProps) {
  // Close on Escape
  useEffect(() => {
    if (!open) return
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose()
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [open, onClose])

  if (!open) return null

  const content = (
    <div
      className={styles.backdrop}
      onClick={onClose}
      role="dialog"
      aria-modal="true"
    >
      <div
        className={styles.dialog}
        onClick={(e) => e.stopPropagation()}
      >
        <div className={styles.header}>
          <div className={danger ? styles.iconWrapDanger : styles.iconWrap}>
            {icon}
          </div>
          <h2 className={styles.title}>{title}</h2>
          <button
            className={styles.closeBtn}
            onClick={onClose}
            aria-label="关闭"
          >
            <X size={16} />
          </button>
        </div>
        <div className={styles.body}>{children}</div>
        {footer && <div className={styles.actions}>{footer}</div>}
      </div>
    </div>
  )

  return createPortal(content, document.body)
}

// ── Prompt modal (text input) ───────────────────────────────────

interface PromptModalProps {
  open: boolean
  title: string
  message?: string
  placeholder?: string
  initialValue?: string
  confirmLabel?: string
  icon?: React.ReactNode
  validate?: (value: string) => string | null
  onConfirm: (value: string) => void
  onCancel: () => void
}

export function PromptModal({
  open,
  title,
  message,
  placeholder,
  initialValue = '',
  confirmLabel = '确认',
  icon = <Save size={18} />,
  validate,
  onConfirm,
  onCancel,
}: PromptModalProps) {
  const [value, setValue] = useState(initialValue)
  const [error, setError] = useState<string | null>(null)
  const inputRef = useRef<HTMLInputElement>(null)

  // Reset state when reopened
  useEffect(() => {
    if (open) {
      setValue(initialValue)
      setError(null)
      // Autofocus after entrance animation
      const t = setTimeout(() => inputRef.current?.focus(), 80)
      return () => clearTimeout(t)
    }
  }, [open, initialValue])

  const handleConfirm = () => {
    const trimmed = value.trim()
    if (!trimmed) {
      setError('不能为空')
      return
    }
    const validationError = validate?.(trimmed)
    if (validationError) {
      setError(validationError)
      return
    }
    onConfirm(trimmed)
  }

  const handleKey = (e: React.KeyboardEvent<HTMLInputElement>) => {
    if (e.key === 'Enter') {
      e.preventDefault()
      handleConfirm()
    }
  }

  return (
    <ModalShell
      open={open}
      title={title}
      icon={icon}
      onClose={onCancel}
      footer={
        <>
          <button className={styles.btnCancel} onClick={onCancel}>
            取消
          </button>
          <button
            className={styles.btnPrimary}
            onClick={handleConfirm}
            disabled={!value.trim()}
          >
            {confirmLabel}
          </button>
        </>
      }
    >
      {message && <p>{message}</p>}
      <input
        ref={inputRef}
        type="text"
        className={styles.input}
        placeholder={placeholder}
        value={value}
        onChange={(e) => { setValue(e.target.value); setError(null) }}
        onKeyDown={handleKey}
        autoComplete="off"
        spellCheck={false}
      />
      {error && <div className={styles.error}>{error}</div>}
    </ModalShell>
  )
}

// ── Confirm modal (yes/no) ──────────────────────────────────────

interface ConfirmModalProps {
  open: boolean
  title: string
  message: string
  confirmLabel?: string
  danger?: boolean
  busy?: boolean
  onConfirm: () => void
  onCancel: () => void
}

export function ConfirmModal({
  open,
  title,
  message,
  confirmLabel = '确认',
  danger = false,
  busy = false,
  onConfirm,
  onCancel,
}: ConfirmModalProps) {
  return (
    <ModalShell
      open={open}
      title={title}
      icon={danger ? <AlertTriangle size={18} /> : <Save size={18} />}
      danger={danger}
      onClose={busy ? () => undefined : onCancel}
      footer={
        <>
          <button className={styles.btnCancel} onClick={onCancel} disabled={busy}>
            取消
          </button>
          <button
            className={danger ? styles.btnDanger : styles.btnPrimary}
            onClick={onConfirm}
            disabled={busy}
          >
            {busy ? '处理中…' : confirmLabel}
          </button>
        </>
      }
    >
      {message}
    </ModalShell>
  )
}

// ── Icon helpers (re-exported for convenience) ─────────────────
export { Save, Trash2, Pencil, AlertTriangle }
