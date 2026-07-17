import { useState, useCallback, useRef } from 'react'
import type { Toast, ToastKind } from '../types'

// Re-export types for backward compatibility
export type { ToastKind, Toast } from '../types'

let _id = 0

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
    if (toastsRef.current.some(t => t.message === message && t.kind === kind)) return
    const id = ++_id
    setToasts(prev => {
      const next = [...prev, { id, message, kind }]
      toastsRef.current = next
      return next
    })
    const t = setTimeout(() => dismiss(id), 2500)
    timers.current.set(id, t)
  }, [dismiss])

  return { toasts, show, dismiss }
}
