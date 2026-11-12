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

export function useProductControl(onProductChanged: (product: 'map' | 'nav' | 'inspection', restoring: boolean) => void) {
  const observe = isObservationMode()
  const [snapshot, setSnapshot] = useState<ProductControlSnapshot | null>(null)
  const [pendingId, setPendingId] = useState(readPending)
  const [operation, setOperation] = useState<ProductSwitchOperation | null>(null)
  const [error, setError] = useState('')
  const [notice, setNotice] = useState('')
  const [available, setAvailable] = useState(false)
  const [submitting, setSubmitting] = useState(false)
  const submittingRef = useRef(false)
  const sessionRef = useRef<string | null>(null)

  useEffect(() => {
    if (observe) return
    let disposed = false
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      try {
        if (submittingRef.current) return
        let missingOperation = false
        if (pendingId) {
          let next: ProductSwitchOperation | undefined
          try {
            next = await fetchProductOperation(pendingId)
          } catch (cause) {
            if (!(cause instanceof ProductControlError && cause.status === 404)) throw cause
            missingOperation = true
          }
          if (disposed) return
          if (next) {
            setOperation(next)
            if (!switchIsTerminal(next)) {
              setAvailable(true)
              setError('')
              return
            }
          } else {
            setOperation(null)
          }
          setAvailable(false)
        }
        const next = await fetchProductControl()
        if (disposed) return
        setSnapshot(next)
        setAvailable(next.available)
        setError('')
        if (missingOperation) setNotice('上次请求结果不可用，已读取机器人当前状态')
        if (pendingId) {
          storePending(null)
          setPendingId(null)
        }
        const session = next.current.product_session_id ?? null
        if (next.current.status === 'active' && session !== sessionRef.current) {
          const restoring = sessionRef.current === null
          sessionRef.current = session
          const product = next.current.product
          if (product === 'map' || product === 'nav' || product === 'inspection') {
            onProductChanged(product, restoring)
          }
        }
      } catch {
        if (disposed) return
        setAvailable(false)
        setError(pendingId ? '切换结果尚未确认，正在等待服务恢复…' : '模式切换服务未连接')
      } finally {
        if (!disposed) timer = setTimeout(() => void poll(), pendingId ? 1_000 : 5_000)
      }
    }
    void poll()
    return () => { disposed = true; clearTimeout(timer) }
  }, [onProductChanged, pendingId, observe])

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
    setNotice('')
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
    switchProduct, busy, operation, error, available,
    current: snapshot?.current ?? null,
    message: ((pendingId || operation) ? error : '') || notice || (operation ? operation.message
      : pendingId ? '正在查询上次切换结果…'
      : snapshot?.operation?.state === 'running' ? snapshot.operation.message : ''),
    allowed: available && Boolean(snapshot) && !busy && !observe,
    reason: observe ? '只读监控' : busy ? '模式正在切换' : available ? '' : error || '正在连接模式切换服务',
    dismiss: () => { if (!busy) { setOperation(null); setError(''); setNotice('') } },
  }
}
