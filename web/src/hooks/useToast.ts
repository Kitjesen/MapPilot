import { useState, useCallback, useRef } from 'react'
import type { Toast, ToastKind } from '../types'
import { operatorReasonMessage } from '../services/navigationStatus'

// Re-export types for backward compatibility
export type { ToastKind, Toast } from '../types'

let _id = 0

function operatorToastMessage(message: string): string {
  const locale = /[\u3400-\u9fff]/.test(message) ? 'zh' : 'en'
  const clean = message.replace(/\b(local_no_path|local_blocked|local_near_field_stop|execution_stalled|local_recovery_exhausted|empty_path)(?::[^\s，。；;)]*)?/g, code => {
    const base = code.split(':', 1)[0]
    return operatorReasonMessage(base, locale)
  })
  return clean
    // Request IDs and planner counters help developers, but do not help an operator recover.
    .replace(/\s*[（(]\s*#?[a-f0-9]{6,}\s*[）)]\s*$/i, '')
    .replace(/\s*[（(]planner=[^）)]*points=[^）)]*[）)]\s*$/i, '')
    .replace(/\s{2,}/g, ' ')
    .trim()
}

export function useToast() {
  const [toasts, setToasts] = useState<Toast[]>([])
  const toastsRef = useRef<Toast[]>([])
  const timers = useRef<Map<number, ReturnType<typeof setTimeout>>>(new Map())

  const dismiss = useCallback((id: number) => {
    setToasts(prev => {
      const next = prev.filter(t => t.id !== id)
      toastsRef.current = next
      return next
    })
    const t = timers.current.get(id)
    if (t) { clearTimeout(t); timers.current.delete(id) }
  }, [])

  const show = useCallback((message: string, kind: ToastKind = 'info') => {
    const cleanMessage = operatorToastMessage(message)
    if (!cleanMessage) return
    if (toastsRef.current.some(t => t.message === cleanMessage && t.kind === kind)) return
    const id = ++_id
    setToasts(prev => {
      const next = [...prev, { id, message: cleanMessage, kind }].slice(-3)
      toastsRef.current = next
      return next
    })
    const t = setTimeout(() => dismiss(id), kind === 'error' ? 5_000 : 3_200)
    timers.current.set(id, t)
  }, [dismiss])

  return { toasts, show, dismiss }
}
