/**
 * LocalizationCard — 合并 SLAM / GNSS / 融合 的极简信息卡.
 *
 * 原则:
 *   - 沿用 design tokens(var(--glass), var(--border), var(--accent))
 *   - 不做装饰性动画(没有旋转球、卫星天空图)— 信息密度优先
 *   - 一个 accent 只用在:活动指示点 + 链接高亮,其余全部 neutral
 */
import { useEffect, useState } from 'react'
import { sensorRequest } from '../services/sensors.ts'
import { ChevronDown } from 'lucide-react'
import type { Locale } from '../i18n'
import { mapLocalizationLabel, mapLocalizationState } from '../services/mapLabel.ts'
import type { SSEState } from '../types'
import styles from './LocalizationCard.module.css'

type TabKey = 'slam' | 'gnss' | 'fusion'

interface LocalizationCardProps {
  embedded?: boolean
  initialTab?: TabKey
  sseState: SSEState
  localNowS: number
  locale: Locale
}

function fmtInt(n: number | undefined): string {
  return typeof n === 'number' && Number.isFinite(n) ? Math.round(n).toLocaleString() : '--'
}

function num(data: Record<string, unknown>, key: string): number | undefined {
  const value = data[key]
  if (typeof value === 'number' && Number.isFinite(value)) return value
  if (typeof value === 'string') {
    const parsed = Number(value)
    return Number.isFinite(parsed) ? parsed : undefined
  }
  return undefined
}

function text(data: Record<string, unknown>, key: string): string | undefined {
  const value = data[key]
  return typeof value === 'string' && value.length > 0 ? value : undefined
}

function bool(data: Record<string, unknown>, key: string): boolean | undefined {
  const value = data[key]
  if (typeof value === 'boolean') return value
  if (typeof value === 'number') return value !== 0
  if (typeof value === 'string') {
    if (value === 'true' || value === '1') return true
    if (value === 'false' || value === '0') return false
  }
  return undefined
}

function object(data: Record<string, unknown>, key: string): Record<string, unknown> {
  const value = data[key]
  return value != null && typeof value === 'object' && !Array.isArray(value)
    ? value as Record<string, unknown>
    : {}
}

export function LocalizationCard({ sseState, localNowS, locale, initialTab = 'slam', embedded = false }: LocalizationCardProps) {
  const [tab, setTab] = useState<TabKey>(initialTab)

  return (
    <div className={embedded ? styles.embedded : styles.card}>
      <div className={styles.tabs} role="tablist">
        {(['slam', 'gnss', 'fusion'] as TabKey[]).map((t) => (
          <button
            key={t}
            role="tab"
            aria-selected={tab === t}
            className={tab === t ? styles.tabActive : styles.tab}
            onClick={() => setTab(t)}
          >
            {labelFor(t)}
          </button>
        ))}
      </div>
      <div className={styles.body}>
        {tab === 'slam' && <SlamPanel sseState={sseState} localNowS={localNowS} locale={locale} />}
        {tab === 'gnss' && <GnssPanel localNowS={localNowS} />}
        {tab === 'fusion' && <FusionPanel sseState={sseState} />}
      </div>
    </div>
  )
}

function labelFor(t: TabKey): string {
  switch (t) {
    case 'slam':   return '定位'
    case 'gnss':   return '卫星'
    case 'fusion': return '融合'
  }
}

function modeLabel(mode: string): string {
  if (mode === 'native_dds' || mode === 'cpp_slam_status') return '原生定位'
  if (mode === 'ltlio') return 'LTLIO'
  if (mode === 'mapping') return '建图'
  if (mode === 'localization') return '地图定位'
  if (mode.includes('ros2')) return '兼容桥'
  return mode
}

/* ─── SLAM panel ───────────────────────────────────────────────── */

function SlamPanel({ sseState, localNowS, locale }: LocalizationCardProps) {
  const odom = sseState.odometry
  const slam = sseState.slamStatus
  const diag = sseState.slamDiag?.data ?? {}
  const connected = sseState.connected

  const posX = fmt(odom?.x, 2)
  const posY = fmt(odom?.y, 2)
  const yawDeg =
    typeof odom?.yaw === 'number' ? ((odom.yaw * 180) / Math.PI).toFixed(1) : '—'
  const vx = fmt(odom?.vx, 2)
  const mode = slam?.mode ?? '离线'
  const hasOdometry = connected && odom != null
  const localizationState = mapLocalizationState(sseState, localNowS)
  const poseFresh = sseState.stateSnapshot?.localization?.pose_fresh
  const odometryStatus = mapLocalizationLabel(localizationState, locale)
  const stale = localizationState === 'stale'
  const processedHz = num(diag, 'processed_scan_hz') ?? slam?.slam_hz
  const lidarHz = num(diag, 'lidar_input_hz')
  const imuHz = num(diag, 'imu_input_hz')
  const tickHz = num(diag, 'slam_tick_hz')
  const targetHz = num(diag, 'status_target_hz')
  const registeredPoints = num(diag, 'registered_points')
  const mapPoints = num(diag, 'map_points') ?? slam?.map_points
  const imuBuffer = num(diag, 'imu_buffer')
  const lidarBuffer = num(diag, 'lidar_buffer')
  const droppedLidar = num(diag, 'dropped_lidar_frames')
  const droppedImu = num(diag, 'dropped_imu_frames')
  const syncWait = num(diag, 'sync_wait_count')
  const sceneMode = text(diag, 'scene_mode') ?? 'unknown'
  const mapLoaded = bool(diag, 'map_loaded')
  const mapFrameJump = bool(diag, 'map_frame_jump')
  const tf = object(diag, 'map_odom_tf')
  const hasTf = bool(diag, 'has_map_odom_tf') ?? bool(tf, 'valid') ?? false
  const icpQuality = sseState.session?.icp_quality
  const icpText = typeof icpQuality === 'number' && Number.isFinite(icpQuality)
    ? icpQuality.toFixed(3)
    : '--'
  const icpGood = typeof icpQuality === 'number' && icpQuality > 0 && icpQuality < 0.3

  return (
    <>
      <Header title="定位里程计" live={localizationState === 'tracking'} status={odometryStatus} />

      <div className={styles.grid}>
        <Field label="X · Y" value={`${posX}, ${posY}`} unit="m" />
        <Field label="航向" value={yawDeg} unit="°" />
        <Field label="线速度" value={vx} unit="m/s" />
        <Field label="数据时效" value={!connected ? '连接断开' : !hasOdometry ? '未收到' : stale ? '已过期' : poseFresh === true ? '最新' : '未报告'} dim={poseFresh !== true || !connected || stale} />
      </div>
      <details className={styles.details}>
        <summary>定位详情<ChevronDown size={14} /></summary>
        <div className={styles.grid}>
          <Field label="链路" value={modeLabel(mode)} dim title={mode} />
          <Field label="处理频率" value={fmt(processedHz, 1)} unit="Hz" title="定位输出频率；和雷达原始输入不是同一个计数窗口。" />
          <Field label="目标频率" value={fmt(targetHz, 1)} unit="Hz" dim title="配置目标频率，不是实测雷达吞吐。" />
          <Field label="雷达输入" value={fmt(lidarHz, 1)} unit="Hz" title="原生运行时报告的雷达输入频率。" />
          <Field label="IMU 输入" value={fmt(imuHz, 1)} unit="Hz" />
          <Field label="匹配质量" value={icpText} dim={!icpGood} title="越低越好；大于 0.3 需要重定位或重新建图。" />
          <Field label="内部时钟" value={fmt(tickHz, 1)} unit="Hz" dim />
          <Field label="地图变换" value={hasTf ? '正常' : '缺失'} dim={!hasTf} />
          <Field label="匹配点数" value={fmtInt(registeredPoints)} />
          <Field label="地图点数" value={fmtInt(mapPoints)} />
          <Field label="缓存" value={`${fmtInt(imuBuffer)} / ${fmtInt(lidarBuffer)}`} unit="I/L" dim />
          <Field label="丢帧" value={`${fmtInt(droppedImu)} / ${fmtInt(droppedLidar)}`} unit="I/L" dim />
          <Field label="同步等待" value={fmtInt(syncWait)} dim />
          <Field label="场景" value={sceneMode} dim />
          <Field label="地图加载" value={mapLoaded ? '是' : '否'} dim={!mapLoaded} />
          <Field label="坐标跳变" value={mapFrameJump ? '是' : '否'} dim={!mapFrameJump} />
        </div>
      </details>
    </>
  )
}

/* ─── GNSS panel ───────────────────────────────────────────────── */

function GnssPanel({ localNowS }: { localNowS: number }) {
  const [snapshot, setSnapshot] = useState<{ data: Record<string, unknown>; receivedAt: number } | null>(null)
  const [error, setError] = useState('')
  useEffect(() => {
    const abort = new AbortController()
    let timer: ReturnType<typeof setTimeout>
    async function poll() {
      try {
        const started = Date.now() / 1000
        const data = await sensorRequest('/api/v1/geography/location', abort.signal)
        if (abort.signal.aborted) return
        setSnapshot({ data, receivedAt: started }); setError('')
      } catch (cause) {
        if (abort.signal.aborted) return
        setSnapshot(null); setError(String(cause))
      } finally { if (!abort.signal.aborted) timer = setTimeout(() => void poll(), 1000) }
    }
    void poll()
    return () => { abort.abort(); clearTimeout(timer) }
  }, [])
  const data = snapshot?.data
  const age = typeof data?.age_s === 'number' ? data.age_s + Math.max(0, localNowS - snapshot!.receivedAt) : undefined
  const ready = data?.state === 'ready' && age !== undefined && age <= 3
  const location = ready ? object(data!, 'location') : {}
  const states: Record<string, string> = { unavailable: '来源未连接', invalid: '来源数据无效', no_fix: '尚未定位', stale: '定位已过期' }
  return <>
    <Header title="GNSS 定位" live={ready} status={ready ? '已定位' : error ? '读取失败' : data?.state === 'ready' ? '定位已过期' : states[String(data?.state)] ?? '正在读取'} />
    {ready ? <div className={styles.grid}>
      <Field label="纬度" value={fmt(num(location, 'latitude'), 7)} unit="°" />
      <Field label="经度" value={fmt(num(location, 'longitude'), 7)} unit="°" />
      <Field label="精度" value={fmt(num(location, 'accuracy_m'), 3)} unit="m" />
      <Field label="定位类型" value={({ 1: 'SINGLE', 2: 'DGPS', 4: 'RTK FIX', 5: 'RTK FLOAT' } as Record<number, string>)[num(location, 'fix_type') ?? 0] ?? String(location.fix_type ?? '—')} />
      <Field label="观测年龄" value={fmt(age, 1)} unit="s" />
    </div> : <Empty text={error || '暂无有效地理位置'} hint="读取原生 GNSS 保存的 WGS84 位置，不打开或重新配置接收机。" />}
  </>
}

/* ─── Fusion panel ─────────────────────────────────────────────── */

interface FusionStatusData {
  aligned?: boolean
  fused_count?: number
  residual_m?: number
  alpha?: number
  rtk_available?: boolean
  last_fuse_age_s?: number
}

function FusionPanel({ sseState }: { sseState: SSEState }) {
  const events = sseState.events as unknown as Array<{ type?: string; data?: FusionStatusData }>
  const evt = events.slice().reverse().find((e) => e?.type === 'gnss_fusion')
  const d = evt?.data
  const hasData = d != null
  const aligned = d?.aligned ?? false

  if (!hasData) {
    return (
      <>
        <Header title="定位 ↔ 卫星" live={false} status="关闭" />
        <Empty
          text="融合未启用"
          hint={<>设置 <code>slam.gnss_fusion: true</code></>}
        />
      </>
    )
  }

  return (
    <>
      <Header
        title="定位 ↔ 卫星"
        live={aligned}
        status={aligned ? '已对齐' : '等待对齐'}
      />

      <div className={styles.grid}>
        <Field label="融合次数" value={String(d!.fused_count ?? 0)} />
        <Field label="α 权重" value={fmt(d!.alpha, 2)} />
        <Field label="残差" value={fmt(d!.residual_m, 3)} unit="m" />
        <Field label="上次更新" value={fmt(d!.last_fuse_age_s, 1)} unit="s" dim />
        <Field label="RTK" value={d!.rtk_available ? '可用' : '不可用'} dim />
      </div>
    </>
  )
}

/* ─── Shared atoms ─────────────────────────────────────────────── */

function Header({ title, live, status }: {
  title: string
  live: boolean
  status: string
}) {
  return (
    <div className={styles.header}>
      <span className={styles.title}>
        <span className={live ? styles.dotLive : styles.dot} />
        {title}
      </span>
      <span className={styles.status}>{status}</span>
    </div>
  )
}

function Field({
  label, value, unit, dim, title,
}: {
  label: string
  value: string
  unit?: string
  dim?: boolean
  title?: string
}) {
  return (
    <div className={styles.field} title={title}>
      <span className={styles.fieldLabel}>{label}</span>
      <span className={dim ? styles.fieldValueDim : styles.fieldValue}>
        {value}
        {unit && <span className={styles.fieldUnit}> {unit}</span>}
      </span>
    </div>
  )
}

function Empty({ text, hint }: { text: string; hint?: React.ReactNode }) {
  return (
    <div className={styles.empty}>
      <div className={styles.emptyText}>{text}</div>
      {hint && <div className={styles.emptyHint}>{hint}</div>}
    </div>
  )
}

function fmt(n: number | undefined, digits = 2): string {
  return typeof n === 'number' && Number.isFinite(n) ? n.toFixed(digits) : '—'
}
