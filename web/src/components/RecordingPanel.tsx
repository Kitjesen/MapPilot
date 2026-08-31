import { useCallback, useEffect, useRef, useState } from 'react'
import {
  AlertTriangle,
  Camera,
  ChevronDown,
  Database,
  Download,
  HardDrive,
  Play,
  RefreshCw,
  Settings2,
  Square,
  Trash2,
  X,
} from 'lucide-react'
import type {
  RecordingDetailResponse,
  RecordingListResponse,
  RecordingStatusResponse,
  ToastKind,
} from '../types'
import * as api from '../services/api'
import { text, type Locale } from '../i18n'
import {
  recordingArtifactDownloadBlocked,
  recordingNeedsRecovery,
  recordingStatusIsActive,
} from '../services/recordingStatus'
import styles from './RecordingPanel.module.css'

interface RecordingPanelProps {
  showToast: (message: string, kind?: ToastKind) => void
  locale: Locale
  status: RecordingStatusResponse | null
  statusError: string | null
  refreshStatus: () => Promise<void>
  embedded?: boolean
  onClose?: () => void
}

function formatBytes(value: number | undefined): string {
  if (value === undefined || !Number.isFinite(value) || value < 0) return '—'
  const units = ['B', 'KiB', 'MiB', 'GiB', 'TiB']
  let current = value
  let unit = 0
  while (current >= 1024 && unit < units.length - 1) {
    current /= 1024
    unit += 1
  }
  return `${current.toFixed(unit === 0 ? 0 : 1)} ${units[unit]}`
}

function formatDuration(seconds: number | undefined): string {
  if (!Number.isFinite(seconds) || seconds === undefined || seconds < 0) return '—'
  const total = Math.floor(seconds)
  const hours = Math.floor(total / 3600)
  const minutes = Math.floor((total % 3600) / 60)
  const remainder = total % 60
  return hours > 0
    ? `${hours}h ${String(minutes).padStart(2, '0')}m`
    : `${minutes}m ${String(remainder).padStart(2, '0')}s`
}

function stateLabel(state: string | undefined, locale: Locale): string {
  const labels: Record<string, [string, string]> = {
    idle: ['Idle', '空闲'],
    preparing: ['Preparing', '准备中'],
    recording: ['Recording', '录制中'],
    stopping: ['Stopping', '停止中'],
    completed: ['Completed', '已完成'],
    failed: ['Failed', '失败'],
  }
  const label = labels[state || '']
  return label ? text(locale, label[0], label[1]) : state || text(locale, 'Unknown', '未知')
}

function recordingErrorMessage(message: string, locale: Locale): string {
  if (/HTTP 502|HTTP 503|Failed to fetch|NetworkError/i.test(message)) {
    return text(
      locale,
      'Gateway is unavailable. Start the Gateway and native recorder before managing sessions.',
      '网关当前不可用。请先启动 Gateway 和原生录制器，再管理录制会话。',
    )
  }
  return message
}

export function RecordingPanel({
  showToast,
  locale,
  status,
  statusError,
  refreshStatus,
  embedded = false,
  onClose,
}: RecordingPanelProps) {
  const [catalog, setCatalog] = useState<RecordingListResponse | null>(null)
  const [detail, setDetail] = useState<RecordingDetailResponse | null>(null)
  const [selectedId, setSelectedId] = useState<string | null>(null)
  const [duration, setDuration] = useState('600')
  const [prefix, setPrefix] = useState('web')
  const [camera, setCamera] = useState(false)
  const [minimumFreeGiB, setMinimumFreeGiB] = useState('5')
  const [busy, setBusy] = useState<string | null>(null)
  const [catalogError, setCatalogError] = useState<string | null>(null)
  const selectedIdRef = useRef<string | null>(null)
  const detailRequestIdRef = useRef(0)
  const statusSessionId = status?.session_id ?? null

  const refreshCatalog = useCallback(async (loadDetail = true) => {
    try {
      const nextCatalog = await api.fetchRecordingList()
      setCatalog(nextCatalog)
      setCatalogError(null)
      const currentSelectedId = selectedIdRef.current
      const currentStatusId = statusSessionId
        && nextCatalog.sessions.some(item => item.session_id === statusSessionId)
        ? statusSessionId
        : null
      const nextSelected = currentSelectedId
        && nextCatalog.sessions.some(item => item.session_id === currentSelectedId)
        ? currentSelectedId
        : currentStatusId || nextCatalog.sessions[0]?.session_id || null
      selectedIdRef.current = nextSelected
      setSelectedId(nextSelected)
      if (loadDetail && nextSelected) {
        const requestId = ++detailRequestIdRef.current
        try {
          const nextDetail = await api.fetchRecordingDetail(nextSelected)
          if (
            requestId === detailRequestIdRef.current
            && selectedIdRef.current === nextSelected
          ) {
            setDetail(nextDetail)
          }
        } catch {
          if (
            requestId === detailRequestIdRef.current
            && selectedIdRef.current === nextSelected
          ) {
            setDetail(null)
          }
        }
      } else if (!nextSelected) {
        detailRequestIdRef.current += 1
        setDetail(null)
      }
    } catch (cause: unknown) {
      const message = cause instanceof Error ? cause.message : String(cause)
      setCatalogError(message)
    }
  }, [statusSessionId])

  useEffect(() => {
    void refreshCatalog()
    const timer = window.setInterval(() => void refreshCatalog(), 2500)
    return () => window.clearInterval(timer)
  }, [refreshCatalog])

  const active = recordingStatusIsActive(status)
  const recoveryRequired = recordingNeedsRecovery(status)
  const recorderReady = statusError === null
    && status?.available === true
    && status.healthy !== false
  const controlsDisabled = busy !== null || active || !recorderReady
  const diskLabel = status?.disk_total
    ? `${formatBytes(status.disk_free)} / ${formatBytes(status.disk_total)}`
    : formatBytes(status?.disk_free)
  const visibleState = statusError
    ? text(locale, 'Status refresh failed', '状态刷新失败')
    : recoveryRequired
      ? text(locale, 'Recovery required', '需要恢复')
      : status?.state === 'failed'
        ? stateLabel(status.state, locale)
        : status?.healthy === false
          ? text(locale, 'Recorder unhealthy', '录制器异常')
          : stateLabel(status?.state, locale)
  const visibleError = statusError ?? catalogError
  const detailDownloadsBlocked = recordingArtifactDownloadBlocked(detail?.session.state)
  const artifactDownloadExplanation = text(
    locale,
    'Downloads are enabled only after the session stops.',
    '仅在录制会话停止后才能下载文件。',
  )

  const refreshAll = async () => {
    await Promise.all([refreshStatus(), refreshCatalog()])
  }

  const start = async () => {
    if (!recorderReady) {
      showToast(
        text(
          locale,
          'Native recorder is not ready. Start it before recording.',
          '原生录制器尚未就绪，请先启动它再录制。',
        ),
        'error',
      )
      return
    }
    const seconds = Number(duration)
    if (!Number.isInteger(seconds) || seconds < 1 || seconds > 86400) {
      showToast(text(locale, 'Duration must be 1–86400 seconds', '时长必须为 1–86400 秒'), 'error')
      return
    }
    const minimumFree = Number(minimumFreeGiB)
    if (!Number.isInteger(minimumFree) || minimumFree < 1 || minimumFree > 100) {
      showToast(text(locale, 'Minimum free space must be 1–100 GiB', '最小可用空间必须为 1–100 GiB'), 'error')
      return
    }
    setBusy('start')
    try {
      await api.startRecording({
        duration: seconds,
        prefix: prefix.trim() || 'web',
        capture_profile: 'sensors',
        camera,
        minimum_free_gib: minimumFree,
      })
      showToast(
        text(
          locale,
          'Recording started. Live state comes from the native recorder.',
          '录制已开始，实时状态以原生录制器为准。',
        ),
        'success',
      )
      await refreshAll()
    } catch (cause: unknown) {
      showToast(api.formatCommandError(cause, text(locale, 'Start recording failed', '开始录制失败')), 'error')
    } finally {
      setBusy(null)
    }
  }

  const stop = async () => {
    setBusy('stop')
    try {
      await api.stopRecording()
      showToast(text(locale, 'Recording stopped', '录制已停止'), 'success')
      await refreshAll()
    } catch (cause: unknown) {
      showToast(api.formatCommandError(cause, text(locale, 'Stop recording failed', '停止录制失败')), 'error')
    } finally {
      setBusy(null)
    }
  }

  const selectSession = async (id: string) => {
    selectedIdRef.current = id
    setSelectedId(id)
    const requestId = ++detailRequestIdRef.current
    try {
      const nextDetail = await api.fetchRecordingDetail(id)
      if (requestId === detailRequestIdRef.current && selectedIdRef.current === id) {
        setDetail(nextDetail)
      }
    } catch (cause: unknown) {
      if (requestId !== detailRequestIdRef.current || selectedIdRef.current !== id) return
      showToast(api.formatCommandError(cause, text(locale, 'Load session failed', '读取会话失败')), 'error')
      setDetail(null)
    }
  }

  const remove = async () => {
    const sessionId = detail?.session.session_id
    if (!sessionId || !window.confirm(text(locale, 'Delete this completed recording session?', '删除这条已完成的录制会话？'))) return
    setBusy('delete')
    try {
      await api.deleteRecording(sessionId)
      showToast(text(locale, 'Recording deleted', '录制已删除'), 'success')
      detailRequestIdRef.current += 1
      selectedIdRef.current = null
      setDetail(null)
      setSelectedId(null)
      await refreshCatalog(false)
    } catch (cause: unknown) {
      showToast(api.formatCommandError(cause, text(locale, 'Delete recording failed', '删除录制失败')), 'error')
    } finally {
      setBusy(null)
    }
  }

  return (
    <div
      className={[styles.panel, embedded ? styles.panelEmbedded : ''].filter(Boolean).join(' ')}
      role="region"
      aria-label={text(locale, 'Recording controls', '录制控制')}
    >
      <header className={styles.header}>
        <div>
          <div className={styles.eyebrow}>{text(locale, 'Native data capture', '原生数据录制')}</div>
          <h1><Database size={20} /> {text(locale, 'Recording', '录制')}</h1>
          <p>{text(locale, 'DDS/MCAP sessions managed by the native recorder.', '由原生录制器管理 DDS/MCAP 会话。')}</p>
        </div>
        <div className={styles.headerActions}>
          <button className={styles.iconButton} onClick={() => void refreshAll()} disabled={busy !== null} title={text(locale, 'Refresh', '刷新')}>
            <RefreshCw size={16} />
          </button>
          {onClose && (
            <button className={styles.iconButton} onClick={onClose} title={text(locale, 'Close recording controls', '关闭录制控制')}>
              <X size={16} />
            </button>
          )}
        </div>
      </header>

      <section className={styles.card} aria-label={text(locale, 'Recorder controls', '录制控制')}>
        <div className={styles.statusRow}>
          <span className={recoveryRequired ? styles.dotRecovery : active ? styles.dotActive : styles.dot} />
          <strong>{visibleState}</strong>
          {status?.session_id && <code>{status.session_id}</code>}
        </div>
        {recoveryRequired && (
          <div className={styles.recoveryNotice} role="alert">
            <AlertTriangle size={15} />
            <span>
              {text(
                locale,
                'The active session is unhealthy. Stop it to recover before starting another recording.',
                '当前活动会话异常。请先停止并恢复，再开始新的录制。',
              )}
              {status?.error && <small>{status.error}</small>}
            </span>
          </div>
        )}
        <div className={styles.metrics}>
          <div><span>{text(locale, 'Duration', '时长')}</span><strong>{formatDuration(status?.duration_s)}</strong></div>
          <div><span>{text(locale, 'Session size', '会话大小')}</span><strong>{formatBytes(status?.size_bytes)}</strong></div>
          <div><span>{text(locale, 'Disk free / total', '磁盘可用 / 总量')}</span><strong>{diskLabel}</strong></div>
        </div>
        <div className={styles.formRow}>
          <label>{text(locale, 'Seconds', '秒')}<input value={duration} onChange={event => setDuration(event.target.value)} inputMode="numeric" disabled={controlsDisabled} /></label>
          <label>{text(locale, 'Prefix', '前缀')}<input value={prefix} onChange={event => setPrefix(event.target.value)} maxLength={40} disabled={controlsDisabled} /></label>
          <button className={styles.primary} onClick={() => void start()} disabled={controlsDisabled}><Play size={15} /> {text(locale, 'Start', '开始')}</button>
          <button className={styles.danger} onClick={() => void stop()} disabled={busy !== null || !active}><Square size={14} /> {text(locale, 'Stop', '停止')}</button>
        </div>
        <details className={styles.config} open={embedded}>
          <summary>
            <Settings2 size={14} />
            {text(locale, 'Capture configuration', '采集配置')}
            <ChevronDown size={14} className={styles.configChevron} />
          </summary>
          <div className={styles.configGrid}>
            <label>
              <span>{text(locale, 'Minimum free space', '最小可用空间')}</span>
              <input value={minimumFreeGiB} onChange={event => setMinimumFreeGiB(event.target.value)} inputMode="numeric" disabled={controlsDisabled} />
              <small>GiB</small>
            </label>
            <label className={styles.cameraOption}>
              <input type="checkbox" checked={camera} onChange={event => setCamera(event.target.checked)} disabled={controlsDisabled} />
              <Camera size={14} />
              <span>{text(locale, 'Include camera video', '包含相机视频')}</span>
            </label>
          </div>
          <div className={styles.note}>
            <HardDrive size={14} /> {text(locale, 'Scene recording captures the sensor preset. Inspection evidence recording is owned by the active inspection task.', '场景录制只采集传感器预设；巡检证据录制由当前巡检任务负责。')}
          </div>
        </details>
      </section>

      <section className={styles.card} aria-label={text(locale, 'Recording files', '录制文件')}>
        <div className={styles.sectionHeader}>
          <h2>{text(locale, 'Saved sessions', '已保存会话')}</h2>
          <span>{catalog?.truncated ? text(locale, 'More sessions exist', '还有更多会话') : `${catalog?.sessions.length ?? 0}`}</span>
        </div>
        {visibleError && <div className={styles.error} role="alert">{recordingErrorMessage(visibleError, locale)}</div>}
        <div className={styles.sessionGrid}>
          <div className={styles.sessionList}>
            {!catalog?.sessions.length && <div className={styles.empty}>{text(locale, 'No recording sessions', '暂无录制会话')}</div>}
            {catalog?.sessions.map(item => (
              <button key={item.session_id} className={item.session_id === selectedId ? styles.sessionActive : styles.session} onClick={() => void selectSession(item.session_id)}>
                <span>{item.session_id}</span><small>{stateLabel(item.state, locale)}</small>
              </button>
            ))}
          </div>
          <div className={styles.detail}>
            {detail?.session ? (
              <>
                <div className={styles.detailHeader}><div><strong>{detail.session.session_id}</strong><small>{stateLabel(detail.session.state, locale)}</small></div><button className={styles.deleteButton} onClick={() => void remove()} disabled={busy !== null || detailDownloadsBlocked}><Trash2 size={14} /> {text(locale, 'Delete', '删除')}</button></div>
                <div className={styles.artifacts}>
                  {detail.session.artifacts.map(artifact => detailDownloadsBlocked ? (
                    <span
                      key={artifact.path}
                      className={[styles.artifact, styles.artifactDisabled].join(' ')}
                      aria-disabled="true"
                      title={artifactDownloadExplanation}
                    >
                      <span>{artifact.path}</span><Download size={14} />
                    </span>
                  ) : (
                    <a key={artifact.path} href={api.recordingArtifactUrl(detail.session.session_id, artifact.path)} target="_blank" rel="noreferrer" className={styles.artifact}>
                      <span>{artifact.path}</span><Download size={14} />
                    </a>
                  ))}
                  {detailDownloadsBlocked && (
                    <div className={styles.artifactNotice} role="note">
                      <AlertTriangle size={14} /> {artifactDownloadExplanation}
                    </div>
                  )}
                  {!detail.session.artifacts.length && <div className={styles.empty}>{text(locale, 'No verified artifacts', '没有已验证的文件')}</div>}
                </div>
              </>
            ) : <div className={styles.empty}>{text(locale, 'Select a session to inspect files', '选择一个会话查看文件')}</div>}
          </div>
        </div>
      </section>
    </div>
  )
}
