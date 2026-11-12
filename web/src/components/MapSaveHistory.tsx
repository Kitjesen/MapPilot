import * as api from '../services/api.ts'
import type { MapSaves } from '../hooks/useMapSaves.ts'
import { mapSavePhaseLabel, mapSaveReason, mapSaveStateLabel } from '../services/mapSavePresentation.ts'
import styles from './MapSaveHistory.module.css'

interface MapSaveHistoryProps {
  operations: api.MapSaveOperationStatus[]
  saves: MapSaves
  readOnly: boolean
}

export function MapSaveHistory({ operations, saves, readOnly }: MapSaveHistoryProps) {
  const { history, busy, act, onContinue } = saves
  const feedback = saves.feedback && operations.some(operation => operation.operation_id === saves.feedback?.id)
    ? saves.feedback : null
  const items = operations.map(operation => {
    const id = operation.operation_id
    const name = operation.map_id || operation.name || '未命名地图'
    const state = mapSaveStateLabel(operation.state)
    const running = state === '进行中'
    const canOptimize = operation.state === 'SUCCEEDED' && operation.source_saved === true && !!id
    const mapBusy = operations.some(item => (item.map_id || item.name) === (operation.map_id || operation.name)
      && mapSaveStateLabel(item.state) === '进行中')
    return <li key={id} className={styles.item}>
      <div className={styles.heading}><strong>{running ? mapSavePhaseLabel(operation.phase) : state}</strong></div>
      <p className={styles.time}>{operation.created_at_ns ? new Date(operation.created_at_ns / 1e6).toLocaleString('zh-CN', { hour12: false }) : '时间未提供'}</p>
      <div className={styles.details}>
        {!running && <p>{operation.source_operation_id && '重新优化 · '}{mapSavePhaseLabel(operation.phase)}</p>}
        {operation.source_saved && <p>原始点云已保存</p>}
      </div>
      {(operation.reason_code || (operation.state === 'FAILED' && operation.message))
        && <p className={styles.reason}>{mapSaveReason(operation)}</p>}
      {id && <p className={styles.id} title={id}>任务 {id}</p>}
      {canOptimize && <p className={styles.reason}>按这次存档重新生成，完成后替换同名地图。</p>}
      <div className={styles.actions}>
        {running && id && <button type="button" onClick={() => onContinue({
          schema_version: 1, ok: true, success: null, name, operation_id: id, operation, ts: history.ts,
        })}>继续查询</button>}
        {(operation.state === 'CAPTURED' || operation.state === 'FAILED' || state === '已取消') && id && <button type="button"
          disabled={readOnly || busy !== null} onClick={() => void act(operation, operation.source_saved ? 'process' : 'retry')}>
          {operation.source_saved ? operation.state === 'CAPTURED' ? '开始处理' : '重新处理' : '重试保存'}</button>}
        {canOptimize && <button type="button"
          disabled={readOnly || busy !== null || mapBusy} onClick={() => void act(operation, 'reoptimize')}
          title="重新对齐存档扫描并生成新版导航地图">重新优化</button>}
        {operation.source_saved && id && <a href={api.mapSaveSourceUrl(id)} download={`${name}.pcd`}>下载存档 PCD</a>}
        {running && id && <button type="button" disabled={readOnly || busy !== null || operation.cancel_requested === true}
          onClick={() => void act(operation, 'cancel')}>{operation.cancel_requested ? '取消待确认' : '取消保存'}</button>}
      </div>
    </li>
  })
  return <div className={styles.history}>
    {saves.error && <p className={`${styles.message} ${styles.error}`} role="alert">{saves.error}</p>}
    {feedback && <p className={`${styles.message} ${feedback.error ? styles.error : ''}`}
      role={feedback.error ? 'alert' : 'status'}>{feedback.message}</p>}
    <ul className={styles.list}>{items[0]}</ul>
    {items.length > 1 && <details>
      <summary>早期保存 · {items.length - 1}</summary>
      <ul className={styles.list}>{items.slice(1)}</ul>
    </details>}
  </div>
}
