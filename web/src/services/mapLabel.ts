import { text, type Locale } from '../i18n.ts'
import type { SSEState } from '../types/index.ts'
import { estimateSceneTime, freshSource } from './sceneTelemetry.ts'

/** Keep the real map identity readable without inventing a place name. */
export function shortMapLabel(mapId: string | null | undefined): string | null {
  const name = mapId?.trim().split(/[\\/]/).at(-1)?.replace(/\.pcd$/i, '')
  if (!name) return null
  return name.replace(/_+/g, ' · ')
}

export type MapLocalizationState = 'disconnected' | 'waiting' | 'stale' | 'tracking'
  | 'initializing' | 'relocalizing' | 'degraded' | 'lost' | 'no_odometry' | 'unknown'

/** The connection and the latest localization evidence remain separate facts. */
export function mapLocalizationState(
  state: Pick<SSEState, 'connected' | 'stateSnapshot' | 'stateSnapshotReceivedAt'>,
  localNowS: number,
): MapLocalizationState {
  if (!state.connected) return 'disconnected'
  const snapshot = state.stateSnapshot
  if (!snapshot?.localization) return 'waiting'
  const nowS = estimateSceneTime(localNowS, snapshot.ts, state.stateSnapshotReceivedAt)
  if (!freshSource(snapshot.ts, nowS, 7) || snapshot.localization.pose_fresh === false) return 'stale'
  const localization = snapshot.localization.state
  if (localization === 'ready' || localization === 'tracking') {
    return snapshot.localization.pose_fresh === true ? 'tracking' : 'unknown'
  }
  switch (localization) {
    case 'initializing':
    case 'relocalizing':
    case 'degraded':
    case 'lost':
    case 'no_odometry': return localization
    default: return 'unknown'
  }
}

export function mapLocalizationLabel(state: MapLocalizationState, locale: Locale): string {
  return {
    disconnected: text(locale, 'Disconnected', '连接断开'),
    waiting: text(locale, 'Waiting for data', '等待数据'),
    stale: text(locale, 'Data stale', '数据已过期'),
    tracking: text(locale, 'Tracking', '定位跟踪中'),
    initializing: text(locale, 'Initializing', '初始化中'),
    relocalizing: text(locale, 'Relocalizing', '重定位中'),
    degraded: text(locale, 'Degraded', '定位退化'),
    lost: text(locale, 'Lost', '定位丢失'),
    no_odometry: text(locale, 'Waiting for odometry', '等待里程计'),
    unknown: text(locale, 'Unknown', '未知'),
  }[state]
}
