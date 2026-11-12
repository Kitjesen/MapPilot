import { useCallback, useEffect, useRef, useState } from 'react'
import * as api from '../services/api.ts'
import { mapSaveReason, mapSaveStateLabel } from '../services/mapSavePresentation.ts'

interface Options {
  refreshKey: string
  currentOperationId?: string
  onContinue: (result: api.SaveMapResult & { operation_id: string }) => void
  onComplete: (result: api.SaveMapResult & { operation_id: string }) => void
}

export function useMapSaves({ refreshKey, currentOperationId, onContinue, onComplete }: Options) {
  const [history, setHistory] = useState<api.MapSaveOperationList>({ operations: [], ts: 0 })
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')
  const [feedback, setFeedback] = useState<{ id: string; message: string; error?: boolean } | null>(null)
  const [busy, setBusy] = useState<string | null>(null)
  const previousStates = useRef(new Map<string, string | null | undefined>())
  const optimizeRequests = useRef(new Map<string, string>())

  const load = useCallback(async (signal?: AbortSignal) => {
    setLoading(true)
    setError('')
    try {
      const result = await api.fetchMapSaveOperations(signal)
      if (!signal?.aborted) setHistory(result)
    } catch (cause) {
      if (!signal?.aborted) setError(`无法获取保存记录：${cause instanceof Error ? cause.message : String(cause)}`)
    } finally {
      if (!signal?.aborted) setLoading(false)
    }
  }, [])

  useEffect(() => {
    const controller = new AbortController()
    void load(controller.signal)
    return () => controller.abort()
  }, [load, refreshKey])

  const active = history.operations.some(operation => mapSaveStateLabel(operation.state) === '进行中')
  useEffect(() => {
    if (!active) return
    const controller = new AbortController()
    let timer: ReturnType<typeof setTimeout>
    const poll = async () => {
      await load(controller.signal)
      if (!controller.signal.aborted) timer = setTimeout(() => void poll(), 2000)
    }
    timer = setTimeout(() => void poll(), 2000)
    return () => { controller.abort(); clearTimeout(timer) }
  }, [active, load])

  useEffect(() => {
    const previous = previousStates.current
    previousStates.current = new Map(history.operations.map(operation => [operation.operation_id!, operation.state]))
    for (const operation of history.operations) {
      const id = operation.operation_id
      if (id && operation.state === 'SUCCEEDED' && previous.get(id) !== 'SUCCEEDED'
        && (previous.has(id) || id === currentOperationId)) {
        onComplete({ schema_version: 1, ok: true, success: true,
          name: operation.map_id || operation.name || '', operation_id: id, operation, ts: history.ts })
      }
    }
  }, [history, currentOperationId, onComplete])

  const act = async (operation: api.MapSaveOperationStatus, action: 'retry' | 'process' | 'reoptimize' | 'cancel') => {
    const id = operation.operation_id!
    setBusy(id)
    setFeedback(null)
    try {
      if (action !== 'cancel') {
        let result: api.SaveMapResult
        if (action === 'reoptimize') {
          const requestId = optimizeRequests.current.get(id) ?? api.makeRequestId('map-reoptimize')
          optimizeRequests.current.set(id, requestId)
          result = await api.reoptimizeMapSaveOperation(id, requestId)
        } else {
          result = action === 'process' ? await api.processMapSaveOperation(id) : await api.retryMapSaveOperation(id)
        }
        const nextId = result.operation_id ?? result.operation?.operation_id
        if (!nextId) throw new Error('响应缺少任务编号，请刷新记录确认结果。')
        if (action === 'reoptimize') optimizeRequests.current.delete(id)
        onContinue({ ...result, name: result.name || operation.map_id || operation.name || '', operation_id: nextId })
        setFeedback({ id, message: action === 'reoptimize' ? '正在重新对齐扫描并生成导航地图。'
          : action === 'process' ? '已提交后台处理。' : '已提交重试，继续查询该次保存。' })
      } else {
        await api.cancelMapSaveOperation(id)
        setFeedback({ id, message: '已请求取消，等待机载任务确认。' })
      }
      await load()
    } catch (cause) {
      const result = cause instanceof api.MapSaveError ? cause.result : undefined
      const message = result ? mapSaveReason(result.operation ?? result)
        : cause instanceof Error ? cause.message : String(cause)
      await load()
      setFeedback({ id, message, error: true })
    } finally {
      setBusy(null)
    }
  }

  return { history, loading, error, feedback, busy, load, act, onContinue }
}

export type MapSaves = ReturnType<typeof useMapSaves>
