import { Map, ChevronDown } from 'lucide-react'
import { SavedMapCard } from './SavedMapCard'
import styles from '../AgentConversation.module.css'

export function SavedMapsResult({ maps }: { maps: unknown[] }) {
  const rows = maps.filter((value): value is Record<string, unknown> => Boolean(value) && typeof value === 'object'
    && typeof (value as Record<string, unknown>).name === 'string')
  if (rows.length !== maps.length) return null
  return <section className={styles.mapResult} aria-label="地图查询结果">
    <header><Map size={15} /><strong>地图库</strong><span>{rows.length} 张 · 本次查询</span></header>
    {!rows.length && <p>暂无已保存地图</p>}
    {rows.map((map, index) => {
      const record = map.record as Record<string, unknown> | null | undefined
      const state = typeof record?.state === 'string' ? record.state : 'UNKNOWN'
      const label = ({ READY: '就绪', STALE: '已过期', BUILDING: '构建中', FAILED: '失败', UNKNOWN: '未知' } as Record<string, string>)[state] ?? state
      const blockers = Array.isArray(map.activation_blockers) ? map.activation_blockers.filter((item): item is string => typeof item === 'string') : []
      return <details key={`${map.name}:${index}`} className={styles.mapResultRow}>
        <summary><Map size={14} /><strong>{String(map.name)}</strong><span data-state={state}>{label}</span>
          <small>{typeof map.size_mb === 'number' && Number.isFinite(map.size_mb) ? `${map.size_mb.toFixed(1)} MB` : '—'}</small><ChevronDown size={13} /></summary>
        <div><span>{map.is_active === true ? '当前激活地图' : map.can_activate === true ? '允许激活' : map.can_activate === false ? '暂不可激活' : '激活条件未知'}</span>
          {typeof record?.content_epoch === 'number' && <span>版本 {record.content_epoch}</span>}
          {blockers.length > 0 && <p>{blockers.join('；')}</p>}
          {Number.isSafeInteger(record?.content_epoch) && Number(record?.content_epoch) > 0
            && <SavedMapCard key={`${map.name}:${record!.content_epoch}`} name={String(map.name)} epoch={Number(record!.content_epoch)} autoOpen={false} />}
        </div>
      </details>
    })}
  </section>
}
