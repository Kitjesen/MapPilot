import { useState, useEffect, useCallback } from 'react'
import { Radio, Navigation, OctagonX, Activity, Map as MapIcon, AlertTriangle, Compass, LocateFixed } from 'lucide-react'
import type { SSEState, ToastKind, MapInfo } from '../types'
import * as api from '../services/api'
import { mapIsNavigationReady } from '../services/mapReadiness'
import { ConfirmModal } from './Modal'
import { text, type Locale } from '../i18n'
import styles from './SlamPanel.module.css'

type SessionMode = 'idle' | 'mapping' | 'navigating' | 'exploring'

interface SlamPanelProps {
  sseState: SSEState
  showToast: (msg: string, kind?: ToastKind) => void
  locale: Locale
}

const MODE_LABEL: Record<SessionMode, string> = {
  idle:        '空闲',
  mapping:     'SLAM 建图',
  navigating:  '导航巡航',
  exploring:   '自主探索',
}

interface PendingAction {
  kind: 'start-mapping' | 'start-navigating' | 'start-exploring' | 'end'
  mapName?: string
  title: string
  message: string
  confirmLabel: string
}

function numericMetric(data: Record<string, unknown> | undefined, key: string): number | undefined {
  const value = data?.[key]
  if (typeof value === 'number' && Number.isFinite(value)) return value
  if (typeof value === 'string') {
    const parsed = Number(value)
    return Number.isFinite(parsed) ? parsed : undefined
  }
  return undefined
}

/**
 * Session-driven SLAM control. Single source of truth is backend /api/v1/session.
 * All transitions require explicit user confirmation. Buttons disable themselves
 * when transitions aren't allowed (no active map, wrong mode, transition pending).
 */
export function SlamPanel({ sseState, showToast, locale }: SlamPanelProps) {
  const [busy, setBusy] = useState(false)
  const [pending, setPending] = useState<PendingAction | null>(null)
  const [maps, setMaps] = useState<MapInfo[]>([])
  const [selectedMap, setSelectedMap] = useState<string>('')
  const [lastSession, setLastSession] = useState(sseState.session)

  // Keep latest session from SSE, fallback to REST once if SSE hasn't delivered.
  useEffect(() => {
    if (sseState.session) setLastSession(sseState.session)
  }, [sseState.session])
  useEffect(() => {
    if (!lastSession) {
      api.fetchSession().then(s => setLastSession(s)).catch(() => {})
    }
  }, [lastSession])

  const session = lastSession
  const mode: SessionMode = session?.mode ?? 'idle'

  // Navigable maps = saved point cloud plus OctoPlanner3D map artifact.
  const loadMaps = useCallback(async () => {
    try {
      const all = await api.fetchMaps()
      const navigable = all.filter(mapIsNavigationReady)
      setMaps(navigable)
      if (!selectedMap && session?.active_map && navigable.some(m => m.name === session.active_map)) {
        setSelectedMap(session.active_map)
      } else if (!selectedMap && navigable.length > 0) {
        setSelectedMap(navigable[0].name)
      }
    } catch { /* noop */ }
  }, [selectedMap, session?.active_map])
  useEffect(() => { loadMaps() }, [loadMaps])

  const requestStartMapping = () => {
    setPending({
      kind: 'start-mapping',
      title: '进入 SLAM 建图模式',
      message: '机器狗会开始建立新地图。如果当前有定位任务会被终止。请确认后开始。',
      confirmLabel: '开始建图',
    })
  }

  const requestStartNavigating = () => {
    if (!selectedMap) {
      showToast('请先选择一张可导航地图', 'error')
      return
    }
    setPending({
      kind: 'start-navigating',
      mapName: selectedMap,
      title: `进入导航巡航：${selectedMap}`,
      message: `将激活地图「${selectedMap}」并启动地图定位进行 ICP 对齐。确认后进入巡航模式。`,
      confirmLabel: '开始巡航',
    })
  }

  const requestStartExploring = () => {
    setPending({
      kind: 'start-exploring',
      title: '启动自主探索',
      message: '机器狗会开启 SLAM 建图，同时 FrontierExplorer 自动挑选未知区域边界作为目标。确认后开始。',
      confirmLabel: '开始探索',
    })
  }

  const requestEnd = () => {
    setPending({
      kind: 'end',
      title: `结束当前 ${MODE_LABEL[mode]}?`,
      message: '会停止 SLAM 和地图定位，机器狗返回空闲。未保存的建图数据将丢失。',
      confirmLabel: '结束会话',
    })
  }

  const confirmPending = async () => {
    if (!pending) return
    const action = pending
    setPending(null)
    setBusy(true)
    try {
      const currentSession = action.kind === 'end' ? session : await api.fetchSession()
      if (action.kind === 'end') {
        if (!currentSession?.env) {
          throw new Error('Runtime Env is unknown')
        }
        const command = await api.copyProductControlStopCommand(currentSession.env)
        showToast(`${text(locale, 'ProductControl stop command copied', 'ProductControl 停止命令已复制')}: ${command}`, 'info')
      } else {
        const targetProduct = action.kind === 'start-mapping'
          ? 'map'
          : action.kind === 'start-navigating'
            ? 'nav'
            : 'explore'
        const handoff = await api.copyProductSwitchCommand(targetProduct, {
          currentProduct: currentSession?.product,
          mapName: action.kind === 'start-navigating' ? action.mapName : null,
        })
        showToast(`${text(locale, 'ProductControl command copied', 'ProductControl 命令已复制')}: ${handoff.command}`, 'info')
      }
    } catch (e: unknown) {
      showToast(`切换失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setBusy(false)
    }
  }

  const pendingTx = session?.pending ?? false
  const canStartMapping = (session?.can_start_mapping ?? false) && !busy && !pendingTx
  const canStartNavigating = (session?.can_start_navigating ?? false) && !busy && !pendingTx && !!selectedMap
  const canStartExploring = (session?.can_start_exploring ?? false) && !busy && !pendingTx
  const explorerAvailable = session?.explorer_available ?? false
  const canEnd = (session?.can_end ?? false) && !busy && !pendingTx
  const slamStatus = sseState.slamStatus
  const slamDiag = sseState.slamDiag?.data
  const displaySlamHz = numericMetric(slamDiag, 'processed_scan_hz') ?? slamStatus?.slam_hz
  const displayMapPoints = numericMetric(slamDiag, 'map_points') ?? slamStatus?.map_points
  const localizationBackend = session?.localization_backend ?? session?.slam_profile ?? slamStatus?.mode ?? 'unknown'
  const savedMapRelocalizeSupported =
    session?.saved_map_relocalization_supported ??
    session?.relocalization_supported ??
    localizationBackend === 'localizer'
  const recoveryMethod = session?.recovery_method ?? '--'
  const relocalizeTitle = savedMapRelocalizeSupported
    ? `后端 ${localizationBackend} 支持保存地图重定位`
    : `后端 ${localizationBackend} 不支持保存地图重定位；恢复方式：${recoveryMethod}`
  const mapSaveSource = session?.map_save_source ?? (session?.map_save_supported ? 'available' : '--')
  const recoverySignal = session?.recovery_signal && session.recovery_signal !== 'none'
    ? session.recovery_signal
    : recoveryMethod

  // 3D-BBS auto-relocalize requires a saved-map localizer backend.
  const [autoRelocBusy, setAutoRelocBusy] = useState(false)
  const [restartBusy, setRestartBusy] = useState(false)
  const canAutoReloc = mode === 'navigating' && savedMapRelocalizeSupported && !autoRelocBusy && !pendingTx
  const handleAutoReloc = useCallback(async () => {
    setAutoRelocBusy(true)
    // BBS3D 有随机性 + 个别帧 ICP refine 可能偏离,自动重试 3 次直到
    // 稳定锁定 (fitness < 0.1). 每次 HTTP 调用本身 async,总窗口 ≤25s.
    const MAX_ATTEMPTS = 3
    const LOCK_THRESHOLD = 0.1
    const PER_ATTEMPT_MS = 8_000
    try {
      for (let attempt = 1; attempt <= MAX_ATTEMPTS; attempt++) {
        showToast(
          attempt === 1
            ? '正在全图搜索位姿… (2-3 秒)'
            : `第 ${attempt}/${MAX_ATTEMPTS} 次尝试 (上次 fitness 未达标)`,
          'info',
        )
        const r = await api.autoRelocalize()
        if (!r.success) {
          showToast(`重定位请求失败: ${r.message}`, 'error')
          return
        }
        const startT = Date.now()
        let bestICP = Infinity
        while (Date.now() - startT < PER_ATTEMPT_MS) {
          await new Promise(r => setTimeout(r, 400))
          try {
            const s = await api.fetchSession()
            if (s.icp_quality > 0 && s.icp_quality < bestICP) bestICP = s.icp_quality
            if (s.localizer_ready && s.icp_quality > 0 && s.icp_quality < LOCK_THRESHOLD) {
              showToast(`对齐成功 (ICP=${s.icp_quality.toFixed(3)}) · 尝试 ${attempt} 次`, 'success')
              return
            }
          } catch { /* ignore */ }
        }
        // attempt didn't lock within 8s; if best ICP is "good enough" (< 0.3)
        // accept it with a warning instead of retrying to avoid thrashing.
        if (bestICP < 0.3) {
          showToast(`对齐质量一般 (ICP=${bestICP.toFixed(3)},建议推狗走一下让 SLAM 稳定)`, 'info')
          return
        }
        // else: loop, retry bbs3d
      }
      showToast('全图搜索 3 次均未锁定 — 地图与当前环境可能差异过大,建议重新建图', 'error')
    } catch (e) {
      showToast(`请求失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setAutoRelocBusy(false)
    }
  }, [showToast])

  const canRestartSlam = Boolean(session?.product) && !restartBusy && !busy && !pendingTx
  const handleRestartSlam = useCallback(async () => {
    setRestartBusy(true)
    try {
      if (!session?.env) {
        throw new Error('Runtime Env is unknown')
      }
      const command = await api.copyProductControlRestartCommand(session.env, 'slam')
      showToast(`已复制 ProductControl 重启命令：${command}`, 'info')
    } catch (e) {
      showToast(`复制重启命令失败: ${e instanceof Error ? e.message : String(e)}`, 'error')
    } finally {
      setRestartBusy(false)
    }
  }, [session?.env, showToast])

  const quality = session?.icp_quality ?? 0
  const qualityClass = quality <= 0 ? '' : quality < 0.15 ? styles.qualityGood : quality < 0.3 ? styles.qualityWarn : styles.qualityBad

  const noNavigableMap = maps.length === 0

  return (
    <div className={styles.slamPanel}>
      <div className={styles.panelHeader}>
        <span className={styles.panelTitle}>
          <Radio size={15} />
          {text(locale, 'Localization Mode', '定位模式')}
        </span>
        <span className={`${styles.modePill} ${mode === 'idle' ? styles.modePillIdle : mode === 'mapping' ? styles.modePillMapping : mode === 'exploring' ? styles.modePillExploring : styles.modePillNavigating}`}>
          当前：{MODE_LABEL[mode]}
        </span>
      </div>

      {/* Start mapping */}
      <div className={styles.section}>
        <p className={styles.sectionLabel}>建图</p>
        <button
          className={canStartMapping ? styles.primaryBtn : styles.primaryBtnDisabled}
          onClick={canStartMapping ? requestStartMapping : undefined}
          disabled={!canStartMapping}
        >
          <Navigation size={14} />
          <span>开始 SLAM 建图</span>
          {pendingTx && mode === 'idle' && <span className={styles.spinnerInline} />}
        </button>
        {mode !== 'idle' && <p className={styles.hint}>当前非空闲，请先结束会话</p>}
      </div>

      {/* Exploring (autonomous) */}
      <div className={styles.section}>
        <p className={styles.sectionLabel}>自主探索</p>
        {!explorerAvailable ? (
          <p className={styles.warnHint}>
            <AlertTriangle size={12} /> FrontierExplorer 模块未加载。用 <code>lingtu.py explore</code> profile 启动才能用。
          </p>
        ) : (
          <button
            className={canStartExploring ? styles.primaryBtn : styles.primaryBtnDisabled}
            onClick={canStartExploring ? requestStartExploring : undefined}
            disabled={!canStartExploring}
          >
            <Compass size={14} />
            <span>开始自主探索</span>
            {pendingTx && mode === 'idle' && <span className={styles.spinnerInline} />}
          </button>
        )}
      </div>

      {/* Navigating with map selector */}
      <div className={styles.section}>
        <p className={styles.sectionLabel}>导航巡航</p>
        {noNavigableMap ? (
          <p className={styles.warnHint}>
            <AlertTriangle size={12} /> 没有可导航的地图。先建图并构建 OctoMap，再来这里。
          </p>
        ) : (
          <>
            <label className={styles.mapSelectLabel}>
              <MapIcon size={12} />
              <span>选择地图</span>
            </label>
            <select
              className={styles.mapSelect}
              value={selectedMap}
              onChange={e => setSelectedMap(e.target.value)}
              disabled={mode !== 'idle'}
            >
              {maps.map(m => (
                <option key={m.name} value={m.name}>
                  {m.name}{m.is_active ? ' (激活)' : ''}
                </option>
              ))}
            </select>
            <button
              className={canStartNavigating ? styles.primaryBtn : styles.primaryBtnDisabled}
              onClick={canStartNavigating ? requestStartNavigating : undefined}
              disabled={!canStartNavigating}
            >
              <Activity size={14} />
              <span>开始巡航</span>
              {pendingTx && mode === 'idle' && <span className={styles.spinnerInline} />}
            </button>
          </>
        )}
      </div>

      {/* Auto-relocalize (BBS3D, no initial guess) */}
      <div className={styles.section}>
        <p className={styles.sectionLabel}>自动重定位</p>
        <button
          className={canAutoReloc ? styles.primaryBtn : styles.primaryBtnDisabled}
          onClick={canAutoReloc ? handleAutoReloc : undefined}
          disabled={!canAutoReloc}
          title={relocalizeTitle}
        >
          <LocateFixed size={14} />
          <span>自动重定位 (3D-BBS)</span>
          {autoRelocBusy && <span className={styles.spinnerInline} />}
        </button>
        {mode !== 'navigating' && (
          <p className={styles.hint}>仅巡航模式可用（地图定位器需运行）</p>
        )}
        {mode === 'navigating' && !savedMapRelocalizeSupported && (
          <p className={styles.hint}>保存地图重定位不可用；恢复方式：{recoveryMethod}</p>
        )}
      </div>

      <div className={styles.section}>
        <p className={styles.sectionLabel}>底层定位</p>
        <button
          className={canRestartSlam ? styles.primaryBtn : styles.primaryBtnDisabled}
          onClick={canRestartSlam ? handleRestartSlam : undefined}
          disabled={!canRestartSlam}
          title="复制由 ProductControl 执行的定位进程重启命令"
        >
          <LocateFixed size={14} />
          <span>复制重启命令</span>
          {restartBusy && <span className={styles.spinnerInline} />}
        </button>
        <p className={styles.hint}>命令由现场操作员执行，Web 不直接控制 systemd。</p>
      </div>

      {/* Stats */}
      <div className={styles.section}>
        <p className={styles.sectionLabel}>运行状态</p>
        <div className={styles.statsGrid}>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>处理输出</span>
            <span className={styles.statValue}>
              {typeof displaySlamHz === 'number' ? `${displaySlamHz.toFixed(1)} Hz` : '--'}
            </span>
          </div>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>地图点云</span>
            <span className={styles.statValue}>
              {typeof displayMapPoints === 'number' ? displayMapPoints.toLocaleString() : '--'}
            </span>
          </div>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>ICP 质量</span>
            <span className={`${styles.statValue} ${qualityClass}`}>
              {quality > 0 ? quality.toFixed(3) : '--'}
            </span>
          </div>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>地图定位</span>
            <span className={`${styles.statValue} ${session?.localizer_ready ? styles.statOk : styles.statDim}`}>
              {mode !== 'navigating' ? '停止' : session?.localizer_ready ? '就绪' : '对齐中…'}
            </span>
          </div>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>后端</span>
            <span className={`${styles.statValue} ${styles.statValueCompact}`} title={localizationBackend}>
              {localizationBackend}
            </span>
          </div>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>存图来源</span>
            <span className={`${styles.statValue} ${styles.statValueCompact}`} title={mapSaveSource}>
              {mapSaveSource}
            </span>
          </div>
          <div className={styles.statCard}>
            <span className={styles.statLabel}>恢复</span>
            <span
              className={`${styles.statValue} ${styles.statValueCompact} ${
                session?.restart_recovery_supported ? styles.statOk : styles.statDim
              }`}
              title={recoverySignal}
            >
              {recoverySignal}
            </span>
          </div>
        </div>
      </div>

      {/* End / Stop */}
      <div className={styles.sectionEstop}>
        <button
          className={canEnd ? styles.estopBtn : styles.estopBtnDisabled}
          onClick={canEnd ? requestEnd : undefined}
          disabled={!canEnd}
          aria-label="结束当前会话"
        >
          <OctagonX size={16} />
          结束会话
        </button>
        <p className={styles.estopHint}>停止 SLAM / 地图定位，回到空闲</p>
      </div>

      <ConfirmModal
        open={pending !== null}
        title={pending?.title ?? ''}
        message={pending?.message ?? ''}
        confirmLabel={pending?.confirmLabel ?? '确认'}
        onConfirm={confirmPending}
        onCancel={() => setPending(null)}
      />
    </div>
  )
}
