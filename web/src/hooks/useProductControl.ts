import { useCallback, useEffect, useRef, useState } from 'react'
import {
  fetchProductControl, fetchProductOperation, submitProductSwitch,
  ProductControlError, switchIsTerminal,
  type ProductControlSnapshot, type ProductSwitchOperation,
} from '../services/productControl.ts'
import { isObservationMode } from '../services/observationMode.ts'
import { makeRequestId } from '../services/api.ts'
import type { LocalizationInitialPose } from '../services/localizationInitialPose.ts'

const PENDING_KEY = 'lingtu.product-control.pending'

function readPending(): string | null {
  try { return localStorage.getItem(PENDING_KEY) } catch { return null }
}

function storePending(id: string | null) {
  try {
    if (id) localStorage.setItem(PENDING_KEY, id)
    else localStorage.removeItem(PENDING_KEY)
  } catch { /* The in-memory request still survives a Host restart. */ }
}

export function useProductControl(onComplete: (product: 'map' | 'nav') => void) {
  const observe = isObservationMode()
  const [snapshot, setSnapshot] = useState<ProductControlSnapshot | null>(null)
  const [pendingId, setPendingId] = useState(readPending)
  const [operation, setOperation] = useState<ProductSwitchOperation | null>(null)
  const [error, setError] = useState('')
  const [available, setAvailable] = useState(false)
  const [submitting, setSubmitting] = useState(false)
  const submittingRef = useRef(false)

  useEffect(() => {
    if (observe) return
    let disposed = false
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      try {
        if (submittingRef.current) return
        if (pendingId) {
          const next = await fetchProductOperation(pendingId)
          if (disposed) return
          setAvailable(true)
          setError('')
          setOperation(next)
          if (switchIsTerminal(next)) {
            storePending(null)
            setPendingId(null)
            setSnapshot(null)
            setAvailable(false)
            if (next.state === 'succeeded') onComplete(next.request.product)
          }
        } else {
          const next = await fetchProductControl()
          if (disposed) return
          setSnapshot(next)
          setAvailable(next.available)
          setError('')
        }
      } catch (cause) {
        if (disposed) return
        setAvailable(false)
        if (pendingId && cause instanceof ProductControlError && cause.status === 404) {
          storePending(null)
          setPendingId(null)
          setOperation(null)
          setError('未找到切换请求，请核对当前模式后重试')
        } else {
          setError(pendingId ? '切换结果尚未确认，正在等待服务恢复…' : '模式切换服务未连接')
        }
      } finally {
        if (!disposed) timer = setTimeout(() => void poll(), pendingId ? 1_000 : 5_000)
      }
    }
    void poll()
    return () => { disposed = true; clearTimeout(timer) }
  }, [onComplete, pendingId, observe])

  const busy = submitting || Boolean(pendingId) || snapshot?.operation?.state === 'running'
  const switchProduct = useCallback(async (product: 'map' | 'nav', mapName: string | null = null, initialPose?: LocalizationInitialPose) => {
    if (observe || submittingRef.current || busy || !available || !snapshot) return
    submittingRef.current = true
    setSubmitting(true)
    const request = {
      request_id: makeRequestId('product'), product, map_name: mapName,
      expected_product_session_id: snapshot.current.product_session_id ?? '',
      ...(initialPose ? { initial_pose: initialPose } : {}),
    }
    storePending(request.request_id)
    setError('')
    setOperation({ request_id: request.request_id, request, state: 'running', message: '正在提交切换…', result: null })
    let acceptedOrUnknown = true
    try {
      setOperation(await submitProductSwitch(request))
    } catch (cause) {
      if (cause instanceof ProductControlError && cause.status >= 400 && cause.status < 500) {
        acceptedOrUnknown = false
        storePending(null)
        setOperation({ request_id: request.request_id, request, state: 'failed', message: cause.message, result: null })
      } else {
        setError('请求结果尚未确认，正在重连查询…')
      }
    } finally {
      submittingRef.current = false
      setSubmitting(false)
      if (acceptedOrUnknown) setPendingId(request.request_id)
    }
  }, [available, busy, observe, snapshot])

  return {
    switchProduct, busy, operation, error,
    message: operation ? error || operation.message
      : pendingId ? error || '正在查询上次切换结果…'
      : snapshot?.operation?.state === 'running' ? snapshot.operation.message : '',
    allowed: available && Boolean(snapshot) && !busy && !observe,
    reason: observe ? '只读监控' : busy ? '模式正在切换' : available ? '' : error || '正在连接模式切换服务',
    dismiss: () => { if (!busy) { setOperation(null); setError('') } },
  }
}
