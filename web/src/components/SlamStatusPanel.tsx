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
  const ltlioDegeneracy = diagnostics?.ltlio_degeneracy as Record<string, unknown> | undefined
  const degeneracyAvailable = ltlioDegeneracy?.available
  const categories = Array.isArray(ltlioDegeneracy?.categories) ? ltlioDegeneracy.categories : []
  const principalDirections = Array.isArray(ltlioDegeneracy?.principal_directions)
    ? ltlioDegeneracy.principal_directions
    : []
  const weakModes = categories
    .map((category, index) => ({ category: String(category), index }))
    .filter(({ category }) => category === 'PARTIAL' || category === 'UNOBSERVABLE')
    .map(({ category, index }) => {
      const row = Array.isArray(principalDirections[index]) ? principalDirections[index] : []
      const values = row.slice(0, 6).map((value) => typeof value === 'number' && Number.isFinite(value) ? value : null)
      return { category, index, values }
    })
  const guidance = ltlioDegeneracy?.guidance
  const guidanceText = guidance === 'turn_or_retrace_for_features'
    ? text(locale, 'Turn or retrace for features', '转向或回退以获取特征')
    : guidance === 'awaiting_lidar_geometry'
      ? text(locale, 'Awaiting LiDAR geometry', '等待 LiDAR 几何')
      : text(locale, 'No guidance', '无提示')

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
      {degeneracyAvailable === false && (
        <p className={styles.hint}>{text(locale, 'Degeneracy diagnostics unavailable', '退化诊断不可用')} · {guidanceText}</p>
      )}
      {degeneracyAvailable === true && (
        <div className={styles.hint}>
          {weakModes.length === 0
            ? <div>{text(locale, 'No weak modes', '无弱方向')}</div>
            : <>
              <div>{text(locale, 'Weak modes (IMU frame)', '弱方向（IMU系）')}</div>
              {weakModes.map(({ category, index, values }) => (
                <div key={`${category}-${index}`}>
                  {category} · [{values.slice(0, 3).map((value) => value === null ? '--' : value.toFixed(2)).join(' ')}]
                  {' '}[{values.slice(3).map((value) => value === null ? '--' : value.toFixed(2)).join(' ')}]
                  <small> Rx Ry Rz / Tx Ty Tz</small>
                </div>
              ))}
            </>}
          <div>{guidanceText} · {ltlioDegeneracy?.update_applied === true
            ? text(locale, 'update applied', '已应用更新')
            : text(locale, 'diagnostic only', '仅诊断，未抑制')}</div>
        </div>
      )}
    </section>
  )
}
