import { useCallback, useEffect, useRef, useState } from 'react'
import { RefreshCw } from 'lucide-react'
import * as api from '../services/api.ts'
import { mapSavePhaseLabel, mapSaveReason, mapSaveStateLabel } from '../services/mapSavePresentation.ts'
import styles from './MapSaveHistory.module.css'

interface MapSaveHistoryProps {
  refreshKey: string
  readOnly: boolean
  currentOperationId?: string
  onContinue: (result: api.SaveMapResult & { operation_id: string }) => void
  onComplete: (result: api.SaveMapResult & { operation_id: string }) => void
}

export function MapSaveHistory({ refreshKey, readOnly, currentOperationId, onContinue, onComplete }: MapSaveHistoryProps) {
  const [history, setHistory] = useState<api.MapSaveOperationList>({ operations: [], ts: 0 })
  const [filter, setFilter] = useState('全部')
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState('')
  const [notice, setNotice] = useState('')
  const [busy, setBusy] = useState<string | null>(null)
  const previousStates = useRef(new Map<string, string | null | undefined>())

  const load = useCallback(async (signal?: AbortSignal) => {
    setLoading(true)
    setError('')
    try {
      const result = await api.fetchMapSaveOperations(signal)
      if (!signal?.aborted) setHistory(result)
    } catch (cause) {
      if (!signal?.aborted) setError(cause instanceof Error ? cause.message : String(cause))
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

  const act = async (operation: api.MapSaveOperationStatus, action: 'retry' | 'process' | 'cancel') => {
    const id = operation.operation_id!
    setBusy(id)
    setError('')
    setNotice('')
    try {
      if (action !== 'cancel') {
        const result = action === 'process' ? await api.processMapSaveOperation(id) : await api.retryMapSaveOperation(id)
        const nextId = result.operation_id ?? result.operation?.operation_id
        if (!nextId) throw new Error('重试响应缺少保存任务编号，请刷新记录确认结果。')
        onContinue({ ...result, name: result.name || operation.map_id || operation.name || '', operation_id: nextId })
        setNotice(action === 'process' ? '已提交后台处理。' : '已提交重试，继续查询该次保存。')
      } else {
        await api.cancelMapSaveOperation(id)
        setNotice('已请求取消，等待机载任务确认。')
      }
      await load()
    } catch (cause) {
      const result = cause instanceof api.MapSaveError ? cause.result : undefined
      const message = result ? mapSaveReason(result.operation ?? result)
        : cause instanceof Error ? cause.message : String(cause)
      await load()
      setError(message)
    } finally {
      setBusy(null)
    }
  }

  const visible = history.operations.filter(operation => filter === '全部' || mapSaveStateLabel(operation.state) === filter)
  return <details className={styles.history}>
    <summary><span className={styles.summaryTitle}>保存记录 <span>最近 100 条</span></span></summary>
    <div className={styles.toolbar}>
      <div className={styles.filters} role="group" aria-label="筛选保存记录">
        {['全部', '等待处理', '进行中', '失败', '已取消', '成功'].map(value =>
          <button key={value} type="button" aria-pressed={filter === value}
            onClick={() => setFilter(value)}>{value}</button>)}
      </div>
      <button className={styles.refresh} type="button" onClick={() => void load()} disabled={loading}
        aria-label="刷新保存记录" title="刷新保存记录"><RefreshCw size={15} /></button>
    </div>
    {loading && <p className={styles.message} role="status">正在读取保存记录…</p>}
    {error && <p className={`${styles.message} ${styles.error}`} role="alert">无法获取保存记录：{error}</p>}
    {error && history.operations.length > 0 && <p className={styles.message}>以下为最近读取的保存记录。</p>}
    {notice && <p className={styles.message} role="status">{notice}</p>}
    {!loading && !error && visible.length === 0 && <p className={styles.message}>没有符合条件的保存记录。</p>}
    <ul className={styles.list}>
      {visible.map(operation => {
        const id = operation.operation_id
        const name = operation.map_id || operation.name || '未命名地图'
        const state = mapSaveStateLabel(operation.state)
        const running = state === '进行中'
        return <li key={id} className={styles.item}>
          <div className={styles.heading}><strong>{name}</strong><span data-state={operation.state}>{state}</span></div>
          <p>{operation.created_at_ns ? new Date(operation.created_at_ns / 1e6).toLocaleString('zh-CN', { hour12: false }) : '时间未提供'}</p>
          <p>{mapSavePhaseLabel(operation.phase)}</p>
          {operation.source_saved && <p>PCD 已保存</p>}
          <p>{mapSaveReason(operation)}</p>
          <p className={styles.id}>{id}</p>
          <div className={styles.actions}>
            {running && id && <button type="button" onClick={() => onContinue({
              schema_version: 1, ok: true, success: null, name, operation_id: id, operation, ts: history.ts,
            })}>继续查询</button>}
            {(operation.state === 'CAPTURED' || operation.state === 'FAILED' || state === '已取消') && id && <button type="button"
              disabled={readOnly || busy !== null} onClick={() => void act(operation, operation.source_saved ? 'process' : 'retry')}>
              {operation.source_saved ? operation.state === 'CAPTURED' ? '开始处理' : '重新处理' : '重试保存'}</button>}
            {operation.source_saved && id && <a href={api.mapSaveSourceUrl(id)} download={`${name}.pcd`}>下载存档 PCD</a>}
            {running && id && <button type="button" disabled={readOnly || busy !== null || operation.cancel_requested === true}
              onClick={() => void act(operation, 'cancel')}>{operation.cancel_requested ? '取消待确认' : '取消保存'}</button>}
          </div>
        </li>
      })}
    </ul>
  </details>
}
