import { useCallback, useEffect, useRef, useState } from 'react'
import {
  AlertTriangle,
  Activity,
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
  RecordingCatalogResponse,
  RecordingListResponse,
  RecordingStatusResponse,
  RecordingTopic,
  RecordingSummaryResponse,
  RecordingValidationResponse,
  ToastKind,
} from '../types'
import * as api from '../services/api'
import { RecordingHistory } from './RecordingHistory'
import { text, type Locale } from '../i18n'
import {
  recordingArtifactDownloadBlocked,
  recordingNeedsRecovery,
  recordingStatusIsActive,
  recordingStatusCanStart,
  formatRecordingDuration as formatDuration,
  recordingStopHasIncompleteData,
} from '../services/recordingStatus'
import styles from './RecordingPanel.module.css'
import { RecordingStreams } from './RecordingStreams'
import { RecordingLiveCamera } from './RecordingLiveCamera'
import { readRecordingPreferences, writeRecordingPreferences } from '../services/recordingPreferences'

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

const TOPIC_GROUP_LABELS: Record<string, [string, string]> = {
  sensors: ['Sensors', '传感器'],
  localization: ['Localization', '定位与建图'],
  robot: ['Robot', '机器人状态'],
  navigation: ['Navigation', '导航与控制'],
  inspection: ['Inspection', '巡检'],
}

export function RecordingTopicPicker({ topics, selected, disabled, locale, onChange }: {
  topics: RecordingTopic[]
  selected: string[]
  disabled: boolean
  locale: Locale
  onChange: (topics: string[]) => void
}) {
  const groups = [...new Set(topics.map(topic => topic.group))]
  const toggle = (names: string[], enable: boolean) => {
    const next = new Set(selected)
    for (const name of names) {
      if (enable) next.add(name)
      else next.delete(name)
    }
    onChange(topics.filter(topic => next.has(topic.topic)).map(topic => topic.topic))
  }
  return (
    <div className={styles.topicPicker}>
      <div className={styles.selectionHeader}>
        <span>{text(locale, 'Data streams', '录制内容')} <small>{selected.length} / {topics.length}</small></span>
        <div>
          <button disabled={disabled} onClick={() => onChange(topics.filter(topic => topic.default_selected).map(topic => topic.topic))}>{text(locale, 'Default', '默认')}</button>
          <button disabled={disabled} onClick={() => onChange(topics.map(topic => topic.topic))}>{text(locale, 'All', '全选')}</button>
          <button disabled={disabled} onClick={() => onChange([])}>{text(locale, 'Clear', '清空')}</button>
        </div>
      </div>
      {groups.map(group => {
        const items = topics.filter(topic => topic.group === group)
        const count = items.filter(topic => selected.includes(topic.topic)).length
        const label = TOPIC_GROUP_LABELS[group]
        return (
          <details key={group} className={styles.topicGroup} open={group === 'sensors' || group === 'robot'}>
            <summary><span>{label ? text(locale, label[0], label[1]) : group}</span><small>{count} / {items.length}</small><ChevronDown size={14} /></summary>
            <div className={styles.topicGroupAction}>
              <button disabled={disabled} onClick={() => toggle(items.map(topic => topic.topic), count !== items.length)}>{count === items.length ? text(locale, 'Deselect group', '取消本组') : text(locale, 'Select group', '选择本组')}</button>
            </div>
            {items.map(topic => (
              <label key={topic.topic} className={styles.topicOption}>
                <input type="checkbox" checked={selected.includes(topic.topic)} disabled={disabled} onChange={event => toggle([topic.topic], event.target.checked)} />
                <span><strong>{topic.label}</strong><code>{topic.topic}</code></span>
                {!topic.replayable && <small>{text(locale, 'Analysis only', '仅分析')}</small>}
              </label>
            ))}
          </details>
        )
      })}
    </div>
  )
}

export function RecordingReceivedStreams({ summary, topics, locale }: {
  summary: RecordingSummaryResponse['summary']
  topics: RecordingTopic[]
  locale: Locale
}) {
  return <>
    {summary.topics.map(topic => <div key={topic.topic} className={styles.streamCount}>
      <span title={topic.dds_topic}>{topics.find(item => item.topic === topic.topic)?.label ?? topic.topic}</span>
      <strong>{topic.message_count.toLocaleString()}</strong>
    </div>)}
    {summary.missing_topics.map(topic => <div key={topic} className={styles.missingStream}><span>{topics.find(item => item.topic === topic)?.label ?? topic}</span><span>{text(locale, 'No samples', '未收到')}</span></div>)}
  </>
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
  const [view, setView] = useState<'live' | 'capture' | 'sessions'>('live')
  const [catalog, setCatalog] = useState<RecordingListResponse | null>(null)
  const [topicCatalog, setTopicCatalog] = useState<RecordingCatalogResponse | null>(null)
  const [topicCatalogError, setTopicCatalogError] = useState<string | null>(null)
  const [selectedTopics, setSelectedTopics] = useState<string[] | null>(() => readRecordingPreferences()?.topics ?? null)
  const [detail, setDetail] = useState<RecordingDetailResponse | null>(null)
  const [summary, setSummary] = useState<RecordingSummaryResponse | null>(null)
  const [summaryError, setSummaryError] = useState<string | null>(null)
  const [validation, setValidation] = useState<RecordingValidationResponse | null>(null)
  const [selectedId, setSelectedId] = useState<string | null>(null)
  const [duration, setDuration] = useState('600')
  const [prefix, setPrefix] = useState('web')
  const [camera, setCamera] = useState(() => readRecordingPreferences()?.camera ?? false)
  const [minimumFreeGiB, setMinimumFreeGiB] = useState('5')
  const [busy, setBusy] = useState<string | null>(null)
  const [catalogError, setCatalogError] = useState<string | null>(null)
  const selectedIdRef = useRef<string | null>(null)
  const detailRequestIdRef = useRef(0)
  const statusSessionId = status?.session_id ?? null
  const updateSelectedTopics = (topics: string[]) => {
    setSelectedTopics(topics)
    writeRecordingPreferences({ topics, camera })
  }

  const refreshTopicCatalog = useCallback(async () => {
    try {
      const next = await api.fetchRecordingCatalog()
      setTopicCatalog(next)
      setTopicCatalogError(null)
      setSelectedTopics(current => current === null
        ? next.topics.filter(topic => topic.default_selected).map(topic => topic.topic)
        : current.filter(name => next.topics.some(topic => topic.topic === name)))
    } catch (cause: unknown) {
      setTopicCatalogError(cause instanceof Error ? cause.message : String(cause))
    }
  }, [])

  useEffect(() => { void refreshTopicCatalog() }, [refreshTopicCatalog])

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
    && recordingStatusCanStart(status)
  const controlsDisabled = busy !== null || active || !recorderReady
  const includeCamera = camera && topicCatalog?.camera?.supported !== false
  const startDisabled = controlsDisabled || topicCatalog === null || (!selectedTopics?.length && !includeCamera)
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
  const detailSessionId = detail?.session.session_id
  const detailSessionState = detail?.session.state
  const detailHasDds = detail?.session.children.some(child => child.name === 'dds') ?? false
  const artifactDownloadExplanation = text(
    locale,
    'Downloads are enabled only after the session stops.',
    '仅在录制会话停止后才能下载文件。',
  )

  useEffect(() => {
    let current = true
    setSummary(null)
    setSummaryError(null)
    setValidation(null)
    if (detailSessionId && detailHasDds && !recordingArtifactDownloadBlocked(detailSessionState)) {
      void api.fetchRecordingSummary(detailSessionId).then(next => {
        if (current) setSummary(next)
      }).catch((cause: unknown) => {
        if (current) setSummaryError(cause instanceof Error ? cause.message : String(cause))
      })
    }
    return () => { current = false }
  }, [detailSessionId, detailSessionState, detailHasDds])

  const refreshAll = async () => {
    await Promise.all([refreshStatus(), refreshCatalog(), refreshTopicCatalog()])
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
    if (!topicCatalog || (!selectedTopics?.length && !includeCamera)) {
      showToast(text(locale, 'Select data streams or camera video', '请选择数据流或相机视频'), 'error')
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
        camera: includeCamera,
        minimum_free_gib: minimumFree,
        topics: selectedTopics ?? [],
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
      if (api.isGatewayApiError(cause) && recordingStopHasIncompleteData(cause.errorCode, cause.detail)) {
        showToast(text(locale, 'Recording stopped, but data is incomplete. Check missing streams and logs.', '录制已停止，但数据不完整；请查看缺失数据与日志。'), 'info')
      } else {
        showToast(api.formatCommandError(cause, text(locale, 'Stop recording failed', '停止录制失败')), 'error')
      }
      await refreshAll()
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

  const validate = async () => {
    const sessionId = detail?.session.session_id
    if (!sessionId) return
    setBusy('validate')
    try {
      const result = await api.validateRecording(sessionId)
      if (selectedIdRef.current === sessionId) setValidation(result)
      showToast(text(locale, 'Offline file and message-envelope validation completed; no data was published.', '离线文件与消息封装校验完成，未向机器人发布数据。'), 'success')
    } catch (cause: unknown) {
      showToast(api.formatCommandError(cause, text(locale, 'Recording validation failed', '录制数据校验失败')), 'error')
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
          <h1><Database size={16} /> {text(locale, 'Data', '数据')}</h1>
          <p>{text(locale, 'Live video, data streams and recording history', '实时视频、数据流与录制回看')}</p>
        </div>
        <div className={styles.headerActions}>
          {(active || recoveryRequired) && <span className={styles.headerState}><i className={recoveryRequired ? styles.dotRecovery : styles.dotActive} />{visibleState}</span>}
          {active && view !== 'capture' && <button className={styles.danger} onClick={() => void stop()} disabled={busy !== null || !active}><Square size={12} />{text(locale, 'Stop', '停止')}</button>}
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

      <div className={styles.tabs} role="tablist" aria-label={text(locale, 'Data workspace', '数据工作区')} onKeyDown={event => {
        if (!['ArrowLeft', 'ArrowRight', 'Home', 'End'].includes(event.key)) return
        event.preventDefault()
        const tabs = [...event.currentTarget.querySelectorAll<HTMLButtonElement>('[role="tab"]')]
        const index = tabs.indexOf(document.activeElement as HTMLButtonElement)
        const next = event.key === 'Home' ? 0 : event.key === 'End' ? tabs.length - 1
          : (index + (event.key === 'ArrowRight' ? 1 : -1) + tabs.length) % tabs.length
        tabs[next]?.click()
        tabs[next]?.focus()
      }}>
        {([
          ['live', Activity, text(locale, 'Live', '实时')],
          ['capture', Camera, text(locale, 'Record', '录制')],
          ['sessions', Database, text(locale, 'History', '历史回看')],
        ] as const).map(([value, Icon, label]) => <button key={value} id={`data-tab-${value}`} role="tab" aria-selected={view === value} aria-controls={`data-panel-${value}`} tabIndex={view === value ? 0 : -1} onClick={() => setView(value)}><Icon size={14} />{label}{value === 'sessions' && !!catalog?.sessions.length && <small>{catalog.sessions.length}</small>}</button>)}
      </div>

      {view === 'live' && <section id="data-panel-live" role="tabpanel" aria-labelledby="data-tab-live" className={styles.livePanel}>
        <RecordingLiveCamera locale={locale} />
        {topicCatalog ? <RecordingStreams topics={topicCatalog.topics} selected={selectedTopics ?? []} disabled={controlsDisabled} locale={locale} onChange={updateSelectedTopics} /> : <div className={styles.empty}>{text(locale, 'Loading data streams…', '正在读取数据流…')}</div>}
        {topicCatalogError && <p className={styles.error} role="alert">{recordingErrorMessage(topicCatalogError, locale)}</p>}
        <footer className={styles.liveFooter}><span>{text(locale, 'Choose streams here, then configure recording.', '选择要保留的数据，再设置录制。')}</span><button className={styles.secondary} onClick={() => setView('capture')}><Settings2 size={13} />{text(locale, 'Recording options', '录制选项')}</button></footer>
      </section>}

      {view === 'capture' && <section id="data-panel-capture" role="tabpanel" aria-labelledby="data-tab-capture" className={styles.card} aria-label={text(locale, 'Recorder controls', '录制控制')}>
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
          <label>{text(locale, 'Duration (seconds)', '时长（秒）')}<input value={duration} onChange={event => setDuration(event.target.value)} inputMode="numeric" disabled={controlsDisabled} /></label>
          <label>{text(locale, 'Name', '名称')}<input value={prefix} onChange={event => setPrefix(event.target.value)} maxLength={40} disabled={controlsDisabled} /></label>
          <button className={styles.primary} onClick={() => void start()} disabled={startDisabled}><Play size={15} /> {text(locale, 'Start', '开始录制')}</button>
          <button className={styles.danger} onClick={() => void stop()} disabled={busy !== null || !active}><Square size={14} /> {text(locale, 'Stop', '停止')}</button>
        </div>
        {topicCatalog ? (
          <RecordingTopicPicker topics={topicCatalog.topics} selected={selectedTopics ?? []} disabled={controlsDisabled} locale={locale} onChange={updateSelectedTopics} />
        ) : <div className={styles.empty}>{text(locale, 'Loading available recording streams…', '正在读取可录制的数据…')}</div>}
        {topicCatalogError && <div className={styles.error} role="alert">{recordingErrorMessage(topicCatalogError, locale)}</div>}
        <p className={styles.streamNote}>{text(locale, 'The catalog lists supported streams. A selected stream may have no publisher in this mode; verify received samples after recording.', '目录表示支持录制的数据；当前模式未发布的数据可能为零，录后请核对实际样本。')}</p>
        <details className={styles.config}>
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
              <input type="checkbox" checked={includeCamera} onChange={event => { setCamera(event.target.checked); writeRecordingPreferences({ topics: selectedTopics ?? [], camera: event.target.checked }) }} disabled={controlsDisabled || !topicCatalog || topicCatalog.camera?.supported === false} />
              <Camera size={14} />
              <span>{text(locale, 'Include color camera video', '包含彩色相机视频')}</span>
            </label>
          </div>
          <div className={styles.note}>
            <HardDrive size={14} /> {text(locale, 'Replay can provide recorded sensor inputs. Commands are retained for analysis and never replayed to the robot. Inspection evidence sessions remain task-owned.', '回放可重新提供传感器输入；控制指令仅供分析，不向机器人重放。巡检证据会话仍由任务管理。')}
          </div>
        </details>
      </section>}

      {view === 'sessions' && <section id="data-panel-sessions" role="tabpanel" aria-labelledby="data-tab-sessions" className={styles.card} aria-label={text(locale, 'Recording files', '录制文件')}>
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
                <a className={[styles.archiveButton, detailDownloadsBlocked ? styles.artifactDisabled : ''].filter(Boolean).join(' ')} href={detailDownloadsBlocked ? undefined : api.recordingArchiveUrl(detail.session.session_id)} aria-disabled={detailDownloadsBlocked} title={detailDownloadsBlocked ? artifactDownloadExplanation : undefined}>
                  <Download size={15} /> {text(locale, 'Download complete session', '下载完整会话')}
                </a>
                <details className={styles.sessionDetails}>
                  <summary>{text(locale, 'Recording details', '记录详情')}</summary>
                <details className={styles.recordedTopics}>
                  <summary>{text(locale, 'Requested streams', '本次选择的数据流')} · {detail.session.children.reduce((count, child) => count + child.selected_topics.length, 0)}</summary>
                  {detail.session.children.flatMap(child => child.selected_topics).map(topic => <code key={topic}>{topic}</code>)}
                </details>
                {!detailHasDds && <p className={styles.streamNote}>{text(locale, 'This session enabled camera recording only. DDS sample statistics and message validation do not apply.', '此会话仅启用相机录制；DDS 样本统计与消息校验不适用。')}</p>}
                {summary && <div className={styles.streamSummary}>
                  <div className={styles.sectionHeader}><h3>{text(locale, 'Received data', '实际收到的数据')}</h3><span>{summary.summary.message_count.toLocaleString()} {text(locale, 'messages', '条')}</span></div>
                  <div className={styles.streamSummaryDuration}>{text(locale, 'Time span', '时间覆盖')} · {formatDuration(summary.summary.duration_ns / 1e9)}</div>
                  <RecordingReceivedStreams summary={summary.summary} topics={topicCatalog?.topics ?? []} locale={locale} />
                  <button className={styles.validateButton} onClick={() => void validate()} disabled={busy !== null || detailDownloadsBlocked}><Play size={13} /> {busy === 'validate' ? text(locale, 'Validating…', '正在校验…') : text(locale, 'Validate replay data', '校验回放数据')}</button>
                  <p className={styles.streamNote}>{text(locale, 'Checks files, types and message envelopes without publishing data. Message field semantics are not verified.', '检查文件、类型与消息封装，不发布数据；不验证消息字段含义。')}</p>
                  <div className={styles.playCommand}><code>ltplay {detail.session.state === 'completed' ? detail.session.session_id : `${detail.session.session_id}/${detail.session.children.find(child => child.name === 'dds')?.artifacts.find(path => path.endsWith('.mcap') || path.endsWith('.mcap.tmp')) ?? 'dds/sensors.mcap.tmp'}`}</code><span>{detail.session.state === 'completed' ? text(locale, 'Run on the host containing this recording', '在保留这份录制的主机上运行') : text(locale, 'Partial data: use the file path under the recording root', '部分数据：在录制根目录下使用文件路径')}</span></div>
                  {validation && <p className={styles.validationResult}>{text(locale, 'Validated', '已校验')} {validation.validation.validated_messages.toLocaleString()} · {text(locale, 'Record-only messages', '仅记录消息')} {validation.validation.skipped_record_only.toLocaleString()}</p>}
                </div>}
                {summaryError && <p className={styles.streamNote}>{text(locale, 'Data summary unavailable', '暂时无法读取样本统计')} · {summaryError}</p>}
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
                </details>
              </>
            ) : <div className={styles.empty}>{text(locale, 'Select a session to inspect files', '选择一个会话查看文件')}</div>}
          </div>
        </div>
        {detail?.session && !detailDownloadsBlocked && <RecordingHistory key={detail.session.session_id} sessionId={detail.session.session_id} sessions={catalog?.sessions ?? []} summary={summary?.summary} topics={topicCatalog?.topics ?? []} locale={locale} />}
      </section>}
    </div>
  )
}
