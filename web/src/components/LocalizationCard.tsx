/**
 * LocalizationCard — 合并 SLAM / GNSS / 融合 的极简信息卡.
 *
 * 原则:
 *   - 沿用 design tokens(var(--glass), var(--border), var(--accent))
 *   - 不做装饰性动画(没有旋转球、卫星天空图)— 信息密度优先
 *   - 一个 accent 只用在:活动指示点 + 链接高亮,其余全部 neutral
 */
import { useState } from 'react'
import type { SSEState } from '../types'
import styles from './LocalizationCard.module.css'

type TabKey = 'slam' | 'gnss' | 'fusion'

interface LocalizationCardProps {
  sseState: SSEState
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

export function LocalizationCard({ sseState }: LocalizationCardProps) {
  const [tab, setTab] = useState<TabKey>('slam')

  return (
    <div className={styles.card}>
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
        {tab === 'slam' && <SlamPanel sseState={sseState} />}
        {tab === 'gnss' && <GnssPanel sseState={sseState} />}
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
  if (mode === 'fastlio2') return 'Fast-LIO2'
  if (mode === 'mapping') return '建图'
  if (mode === 'localization') return '地图定位'
  if (mode.includes('ros2')) return '兼容桥'
  return mode
}

/* ─── SLAM panel ───────────────────────────────────────────────── */

function SlamPanel({ sseState }: { sseState: SSEState }) {
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
  const hasFix = connected && odom != null
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
      <Header title="定位里程计" live={hasFix} status={hasFix ? '已锁定' : '未锁定'} />

      <div className={styles.grid}>
        <Field label="X · Y" value={`${posX}, ${posY}`} unit="m" />
        <Field label="航向" value={yawDeg} unit="°" />
        <Field label="线速度" value={vx} unit="m/s" />
        <Field label="链路" value={modeLabel(mode)} dim title={mode} />
      </div>
      <div className={styles.grid}>
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
    </>
  )
}

/* ─── GNSS panel ───────────────────────────────────────────────── */

type FixType =
  | 'NO_FIX' | 'SINGLE' | 'DGPS' | 'PPS'
  | 'RTK_FIXED' | 'RTK_FLOAT' | 'ESTIMATED' | 'MANUAL' | 'SIMULATION'

interface GnssStatusData {
  fix_type?: FixType
  num_sat?: number
  num_sat_used?: number
  hdop?: number
  age_s?: number
  rtcm_age_s?: number
  horizontal_std_m?: number
  link_ok?: boolean
  east?: number
  north?: number
}

function fixLabel(fix: FixType | undefined): string {
  switch (fix) {
    case 'RTK_FIXED':  return 'RTK FIX'
    case 'RTK_FLOAT':  return 'RTK FLOAT'
    case 'DGPS':       return 'DGPS'
    case 'SINGLE':     return 'SINGLE'
    case 'ESTIMATED':  return 'DR'
    case 'SIMULATION': return 'SIM'
    default:           return '无定位'
  }
}

function GnssPanel({ sseState }: { sseState: SSEState }) {
  const events = sseState.events as unknown as Array<{ type?: string } & GnssStatusData>
  const evt = events.slice().reverse().find((e) => e?.type === 'gnss_status')
  const hasData = evt != null && !!evt.link_ok

  if (!hasData) {
    return (
      <>
        <Header title="GNSS 卫星" live={false} status="离线" />
        <Empty
          text="接收机未连接"
          hint={<>启用 <code>gnss.enabled</code> in robot_config.yaml</>}
        />
      </>
    )
  }

  const fix = evt.fix_type ?? 'NO_FIX'
  const isLive = fix === 'RTK_FIXED' || fix === 'RTK_FLOAT'

  return (
    <>
      <Header title="GNSS 卫星" live={isLive} status={fixLabel(fix)} />

      <div className={styles.grid}>
        <Field label="卫星" value={`${evt.num_sat_used ?? 0}/${evt.num_sat ?? 0}`} />
        <Field label="HDOP" value={fmt(evt.hdop, 2)} />
        <Field label="水平精度" value={fmt(evt.horizontal_std_m, 3)} unit="m" />
        <Field label="差分龄" value={fmt(evt.rtcm_age_s, 1)} unit="s" />
        <Field label="E" value={fmt(evt.east, 2)} unit="m" />
        <Field label="N" value={fmt(evt.north, 2)} unit="m" />
      </div>
    </>
  )
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
