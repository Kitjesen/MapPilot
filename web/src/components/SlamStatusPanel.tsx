import { useCallback, useState } from 'react'
import { Activity, LocateFixed, Map as MapIcon, RefreshCw } from 'lucide-react'
import { text, type Locale } from '../i18n'
import * as api from '../services/api'
import type { SSEState, ToastKind } from '../types'
import styles from './SlamStatusPanel.module.css'

interface SlamStatusPanelProps {
  sseState: SSEState
  showToast: (message: string, kind?: ToastKind) => void
  locale: Locale
}

function metric(data: Record<string, unknown> | undefined, key: string): number | null {
  const value = data?.[key]
  const parsed = typeof value === 'number' ? value : Number(value)
  return Number.isFinite(parsed) ? parsed : null
}

export function SlamStatusPanel({ sseState, showToast, locale }: SlamStatusPanelProps) {
  const [refreshedSession, setRefreshedSession] = useState<typeof sseState.session>(null)
  const [refreshing, setRefreshing] = useState(false)
  const [relocalizing, setRelocalizing] = useState(false)
  const session = sseState.session ?? refreshedSession

  const refresh = useCallback(async () => {
    setRefreshing(true)
    try {
      setRefreshedSession(await api.fetchSession())
    } catch (error) {
      showToast(
        `${text(locale, 'SLAM status refresh failed', 'SLAM 状态刷新失败')}: ${error instanceof Error ? error.message : String(error)}`,
        'error',
      )
    } finally {
      setRefreshing(false)
    }
  }, [locale, showToast])

  const activeMap = session?.active_map ?? null
  const relocalizationSupported = Boolean(
    activeMap
      && (session?.saved_map_relocalization_supported ?? session?.relocalization_supported),
  )

  const relocalize = useCallback(async () => {
    if (!activeMap) return
    setRelocalizing(true)
    try {
      const result = await api.globalRelocalize(activeMap)
      showToast(
        result.ok
          ? text(locale, 'Relocalization accepted', '重定位已接受')
          : result.message || text(locale, 'Relocalization rejected', '重定位被拒绝'),
        result.ok ? 'success' : 'error',
      )
      await refresh()
    } catch (error) {
      showToast(
        `${text(locale, 'Relocalization failed', '重定位失败')}: ${error instanceof Error ? error.message : String(error)}`,
        'error',
      )
    } finally {
      setRelocalizing(false)
    }
  }, [activeMap, locale, refresh, showToast])

  const diagnostics = sseState.slamDiag?.data
  const slamHz = metric(diagnostics, 'processed_scan_hz') ?? sseState.slamStatus?.slam_hz ?? null
  const mapPoints = metric(diagnostics, 'map_points') ?? sseState.slamStatus?.map_points ?? null
  const backend = session?.localization_backend ?? session?.slam_profile ?? sseState.slamStatus?.mode ?? '--'

  return (
    <section className={styles.panel} role="tabpanel" id="panel-slam">
      <header className={styles.header}>
        <div>
          <span className={styles.eyebrow}>SLAM</span>
          <h1>{text(locale, 'Localization status', '定位状态')}</h1>
        </div>
        <button onClick={() => void refresh()} disabled={refreshing} title={text(locale, 'Refresh', '刷新')}>
          <RefreshCw size={16} className={refreshing ? styles.spin : undefined} />
        </button>
      </header>

      <div className={styles.cards}>
        <div><Activity size={16} /><span>{text(locale, 'Mode', '模式')}</span><strong>{session?.mode ?? 'idle'}</strong></div>
        <div><MapIcon size={16} /><span>{text(locale, 'Active map', '当前地图')}</span><strong>{activeMap ?? '--'}</strong></div>
        <div><span>{text(locale, 'Backend', '后端')}</span><strong>{backend}</strong></div>
        <div><span>SLAM Hz</span><strong>{slamHz === null ? '--' : slamHz.toFixed(1)}</strong></div>
        <div><span>{text(locale, 'Map points', '地图点数')}</span><strong>{mapPoints === null ? '--' : Math.round(mapPoints).toLocaleString()}</strong></div>
        <div><span>ICP</span><strong>{session?.icp_quality?.toFixed(3) ?? '--'}</strong></div>
      </div>

      <button
        className={styles.action}
        onClick={() => void relocalize()}
        disabled={!relocalizationSupported || relocalizing}
      >
        <LocateFixed size={16} />
        {relocalizing
          ? text(locale, 'Relocalizing…', '重定位中…')
          : text(locale, 'Relocalize on active map', '在当前地图上重新定位')}
      </button>

      {!activeMap && (
        <p className={styles.hint}>{text(locale, 'No active saved map.', '当前没有激活的保存地图。')}</p>
      )}
    </section>
  )
}
