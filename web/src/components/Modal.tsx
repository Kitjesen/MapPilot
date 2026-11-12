import { useEffect, useId, useRef, useState } from 'react'
import { createPortal } from 'react-dom'
import { Save, Trash2, AlertTriangle, Pencil } from 'lucide-react'
import styles from './Modal.module.css'

// ── Base modal shell ────────────────────────────────────────────

interface ModalProps {
  open: boolean
  title: React.ReactNode
  icon?: React.ReactNode
  danger?: boolean
  onClose: () => void
  children: React.ReactNode
  footer?: React.ReactNode
  initialFocus?: React.RefObject<HTMLElement | null>
  className?: string
  headerAction?: React.ReactNode
  onPointerMove?: React.PointerEventHandler<HTMLDialogElement>
  onPointerLeave?: React.PointerEventHandler<HTMLDialogElement>
}

export function ModalShell({
  open, title, icon, danger, onClose, children, footer, initialFocus,
  className, headerAction, onPointerMove, onPointerLeave,
}: ModalProps) {
  const dialogRef = useRef<HTMLDialogElement>(null)
  const titleId = useId()
  useEffect(() => {
    if (!open) return
    const dialog = dialogRef.current
    if (!dialog) return
    const previousFocus = document.activeElement instanceof HTMLElement ? document.activeElement : null
    dialog.showModal()
    const focusTarget = initialFocus?.current ?? dialog.querySelector<HTMLButtonElement>('[data-modal-cancel]')
    focusTarget?.focus()
    return () => {
      dialog.close()
      if (previousFocus?.isConnected) previousFocus.focus()
    }
  }, [open, initialFocus])

  if (!open) return null

  const content = (
    <dialog
      ref={dialogRef}
      className={`${styles.dialog} ${className ?? ''}`}
      data-has-icon={Boolean(icon)}
      aria-labelledby={titleId}
      onPointerMove={onPointerMove}
      onPointerLeave={onPointerLeave}
      onCancel={event => { event.preventDefault(); event.stopPropagation(); onClose() }}
      onKeyDown={event => { if (event.key === 'Escape') event.stopPropagation() }}
      onClick={event => {
        if (event.target !== event.currentTarget) return
        const bounds = event.currentTarget.getBoundingClientRect()
        if (event.clientX < bounds.left || event.clientX > bounds.right
          || event.clientY < bounds.top || event.clientY > bounds.bottom) onClose()
      }}
    >
      <div className={styles.header}>
        {icon && <div className={danger ? styles.iconWrapDanger : styles.iconWrap}>
          {icon}
        </div>}
        <h2 id={titleId} className={styles.title}>{title}</h2>
        {headerAction}
      </div>
      <div className={styles.body}>{children}</div>
      {footer && <div className={styles.actions}>{footer}</div>}
    </dialog>
  )

  return createPortal(content, document.body)
}

// ── Prompt modal (text input) ───────────────────────────────────

interface PromptModalProps {
  open: boolean
  title: string
  message?: string
  children?: React.ReactNode
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
  children,
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
      initialFocus={inputRef}
      footer={
        <>
          <button type="button" data-modal-cancel className={styles.btnCancel} onClick={onCancel}>
            取消
          </button>
          <button
            type="button"
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
      {children}
    </ModalShell>
  )
}

// ── Confirm modal (yes/no) ──────────────────────────────────────

interface ConfirmModalProps {
  open: boolean
  title: string
  message: React.ReactNode
  confirmLabel?: string
  cancelLabel?: string
  icon?: React.ReactNode
  danger?: boolean
  busy?: boolean
  confirmDisabled?: boolean
  className?: string
  onConfirm: () => void
  onCancel: () => void
}

export function ConfirmModal({
  open,
  title,
  message,
  confirmLabel = '确认',
  cancelLabel = '取消',
  icon,
  danger = false,
  busy = false,
  confirmDisabled = false,
  className,
  onConfirm,
  onCancel,
}: ConfirmModalProps) {
  return (
    <ModalShell
      open={open}
      title={title}
      className={className}
      icon={icon ?? (danger ? <AlertTriangle size={20} /> : <Save size={20} />)}
      danger={danger}
      onClose={busy ? () => undefined : onCancel}
      footer={
        <>
          <button type="button" data-modal-cancel className={styles.btnCancel} onClick={onCancel} disabled={busy}>
            {cancelLabel}
          </button>
          <button
            type="button"
            className={danger ? styles.btnDanger : styles.btnPrimary}
            onClick={onConfirm}
            disabled={busy || confirmDisabled}
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
