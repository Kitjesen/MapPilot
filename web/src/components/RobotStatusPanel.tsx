import { useCallback, useEffect, useState } from 'react'
import { ChevronDown, LocateFixed, RefreshCw, Route } from 'lucide-react'
import { text, type Locale } from '../i18n'
import * as api from '../services/api'
import { selectDisplayedFollowTask } from '../services/followTask'
import { liveNavigationStatus } from '../services/navigationStatus'
import { presentVisualFollowStatus } from '../services/visualFollowStatus'
import type { FollowTaskRef, SSEState, ToastKind } from '../types'
import styles from './RobotStatusPanel.module.css'

interface RobotStatusPanelProps {
  sseState: SSEState
  showToast: (message: string, kind?: ToastKind) => void
  locale: Locale
  motionStartAllowed: boolean
  motionStartBlockedReason: string
}

function value(value: string | null | undefined, fallback: string): string {
  return value?.trim() || fallback
}

export function RobotStatusPanel({
  sseState,
  showToast,
  locale,
  motionStartAllowed,
  motionStartBlockedReason,
}: RobotStatusPanelProps) {
  const [refreshedSession, setRefreshedSession] = useState<typeof sseState.session>(null)
  const [refreshing, setRefreshing] = useState(false)
  const [relocalizing, setRelocalizing] = useState(false)
  const [visualTarget, setVisualTarget] = useState('')
  const [selectedPersonId, setSelectedPersonId] = useState<string | null>(null)
  const [visualCommand, setVisualCommand] = useState<'find' | 'follow' | 'stop' | null>(null)
  const [acceptedFollowTask, setAcceptedFollowTask] = useState<FollowTaskRef | null>(null)
  const session = sseState.session ?? refreshedSession
  const visualServo = sseState.visualServoStatus
  const followStatus = visualServo
    ? presentVisualFollowStatus(
        visualServo,
        liveNavigationStatus(sseState, Date.now() / 1000),
        locale,
      )
    : null
  const followAvailable = visualServo?.follow_available === true
  const displayedFollowTask = selectDisplayedFollowTask(
    visualServo,
    acceptedFollowTask,
    session?.product_session_id,
  )
  const displayedFollowState = visualServo?.task_state
    ?? (acceptedFollowTask ? 'accepted' : null)
  const trackedPersonId = visualServo?.native_track_id ?? visualServo?.person?.id ?? null
  const trackedIdentityId = visualServo?.target_identity_id ?? visualServo?.person?.identity_id
  const trackedPersonPosition = visualServo?.person?.position
  const visiblePeople = (sseState.sceneGraph?.objects ?? []).filter(
    person => person.id && ['person', 'people', 'human', 'pedestrian'].includes(person.label.toLowerCase()),
  )

  useEffect(() => {
    if (
      acceptedFollowTask
      && visualServo?.follow_task_id === acceptedFollowTask.follow_task_id
      && visualServo.product_session_id === acceptedFollowTask.product_session_id
    ) {
      setAcceptedFollowTask(null)
    }
  }, [acceptedFollowTask, visualServo?.follow_task_id, visualServo?.product_session_id])

  const refresh = useCallback(async () => {
    setRefreshing(true)
    try {
      setRefreshedSession(await api.fetchSession())
    } catch (error) {
      showToast(
        `${text(locale, 'Status refresh failed', '状态刷新失败')}: ${error instanceof Error ? error.message : String(error)}`,
        'error',
      )
    } finally {
      setRefreshing(false)
    }
  }, [locale, showToast])

  const activeMap = session?.active_map ?? null
  const canRelocalize = Boolean(
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
          : value(result.message, text(locale, 'Relocalization rejected', '重定位被拒绝')),
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

  const product = value(
    session?.product,
    text(locale, 'Not active', '未启用'),
  )
  const backend = value(
    session?.localization_backend ?? session?.slam_profile,
    text(locale, 'Not active', '未启用'),
  )
  const map = value(activeMap, text(locale, 'No map', '无地图'))
  const cloudPoints = sseState.mapCloud?.count ?? 0
  const ready = session?.localizer_ready === true

  const runVisualServo = useCallback(async (mode: 'find' | 'follow' | 'stop') => {
    const target = visualTarget.trim() || visualServo?.target?.trim() || ''
    const followTaskAtClick = displayedFollowTask
      ? { ...displayedFollowTask }
      : null
    if (mode !== 'stop' && !motionStartAllowed) {
      showToast(motionStartBlockedReason, 'error')
      return
    }
    if (mode !== 'stop' && !target && !(mode === 'follow' && selectedPersonId)) {
      showToast(text(locale, 'Enter the person or object to track', '请输入要寻找或跟随的人或物体'), 'error')
      return
    }
    if (mode === 'stop' && visualServo?.mode === 'follow' && !followTaskAtClick) {
      showToast(text(locale, 'Follow task reference is not ready', '跟随任务引用尚未就绪'), 'error')
      return
    }

    setVisualCommand(mode)
    try {
      const response = mode === 'stop' && followTaskAtClick
        ? await api.cancelFollowTask(followTaskAtClick)
        : await api.sendVisualServo(mode, target || null, selectedPersonId)
      if (
        mode === 'follow'
        && response.ok
        && typeof response.follow_task_id === 'string'
        && typeof response.product_session_id === 'string'
      ) {
        setAcceptedFollowTask({
          follow_task_id: response.follow_task_id,
          product_session_id: response.product_session_id,
        })
      } else if (mode === 'find' && response.ok) {
        setAcceptedFollowTask(null)
      }
      showToast(
        response.ok
          ? text(locale, `${mode} command submitted`, `${mode === 'find' ? '寻找' : mode === 'follow' ? '跟随' : '停止'}命令已提交`)
          : api.formatCommandAck(response, text(locale, 'Visual task rejected', '视觉任务被拒绝')),
        response.ok ? 'success' : 'error',
      )
    } catch (error) {
      showToast(
        api.formatCommandError(error, text(locale, 'Visual task failed', '视觉任务失败')),
        'error',
      )
    } finally {
      setVisualCommand(null)
    }
  }, [displayedFollowTask, locale, motionStartAllowed, motionStartBlockedReason, selectedPersonId, showToast, visualServo?.mode, visualServo?.target, visualTarget])

  return (
    <section className={styles.panel} aria-label={text(locale, 'Robot status', '机器人状态')}>
      <header className={styles.header}>
        <div className={styles.title}>
          <Route size={17} />
          <h2>{text(locale, 'Robot status', '机器人状态')}</h2>
        </div>
        <button
          className={styles.iconButton}
          onClick={() => void refresh()}
          disabled={refreshing}
          title={text(locale, 'Refresh', '刷新')}
        >
          <RefreshCw size={15} className={refreshing ? styles.spin : undefined} />
        </button>
      </header>

      <div className={styles.grid}>
        <div><span>{text(locale, 'Operating mode', '运行模式')}</span><strong>{product}</strong></div>
        <div>
          <span>{text(locale, 'Localization state', '定位状态')}</span>
          <strong className={ready ? styles.ready : styles.waiting}>
            {ready ? text(locale, 'Localization ready', '定位就绪') : text(locale, 'Awaiting localization', '等待定位')}
          </strong>
        </div>
        <div className={styles.mapField}><span>{text(locale, 'Map', '地图')}</span><strong>{map}</strong></div>
      </div>

      <details className={styles.details}>
        <summary>{text(locale, 'Runtime details', '运行详情')}<ChevronDown size={14} /></summary>
        <div className={styles.grid}>
          <div><span>{text(locale, 'Environment', '运行环境')}</span><strong>{session?.env ?? '--'}</strong></div>
          <div><span>{text(locale, 'Localization backend', '定位后端')}</span><strong>{backend}</strong></div>
          <div><span>{text(locale, 'Map cloud points', '地图点云数量')}</span><strong>{cloudPoints.toLocaleString()}</strong></div>
        </div>
      </details>

      <button
        className={styles.action}
        onClick={() => void relocalize()}
        disabled={!canRelocalize || relocalizing}
        title={!activeMap
          ? text(locale, 'No active map', '没有激活地图')
          : text(locale, 'Relocalize on the active map', '在当前地图上重新定位')}
      >
        <LocateFixed size={15} />
        {relocalizing
          ? text(locale, 'Relocalizing…', '重定位中…')
          : text(locale, 'Relocalize', '重新定位')}
      </button>

      {(session?.product === 'inspection' || session?.product === 'tracking') && (
        <div className={styles.visualServo}>
          <div className={styles.visualHeader}>
            <strong>{text(locale, 'Visual task', '视觉任务')}</strong>
            <span>
              {visualServo?.mode ?? 'idle'}
              {trackedIdentityId
                ? ` · ${text(locale, 'Target', '目标')} ${trackedIdentityId.slice(0, 8)}`
                : visualServo?.target_id
                  ? ` · ${text(locale, 'Track', '轨迹')} ${visualServo.target_id}`
                  : visualServo?.target
                    ? ` · ${visualServo.target}`
                    : ''}
            </span>
          </div>
          <input
            className={styles.targetInput}
            value={visualTarget}
            onChange={event => {
              setVisualTarget(event.target.value)
              setSelectedPersonId(null)
            }}
            placeholder={visualServo?.target || text(locale, 'person or object', '人员或物体')}
            disabled={visualCommand !== null}
            aria-label={text(locale, 'Visual target', '视觉目标')}
          />
          {visiblePeople.length > 0 && (
            <div className={styles.personCandidates} aria-label={text(locale, 'Visible people', '可见人员')}>
              {visiblePeople.map(person => (
                <button
                  key={person.id}
                  type="button"
                  data-selected={selectedPersonId === person.id}
                  aria-pressed={selectedPersonId === person.id}
                  onClick={() => setSelectedPersonId(current => current === person.id ? null : person.id)}
                  disabled={visualCommand !== null}
                >
                  ID {person.id}
                  {person.x != null && person.y != null
                    ? ` · (${person.x.toFixed(1)}, ${person.y.toFixed(1)})`
                    : ''}
                </button>
              ))}
            </div>
          )}
          <div className={styles.visualActions}>
            <button
              onClick={() => void runVisualServo('find')}
              disabled={visualCommand !== null || !motionStartAllowed}
              title={motionStartAllowed ? undefined : motionStartBlockedReason}
            >
              {text(locale, 'Find', '寻找')}
            </button>
            <button
              onClick={() => void runVisualServo('follow')}
              disabled={visualCommand !== null || !motionStartAllowed || (!followAvailable && !selectedPersonId)}
              title={!motionStartAllowed
                ? motionStartBlockedReason
                : followAvailable || selectedPersonId
                  ? undefined
                  : text(locale, 'Person selection is not configured', '未配置人员选择能力')}
            >
              {text(locale, 'Follow', '跟随')}
            </button>
            <button
              onClick={() => void runVisualServo('stop')}
              disabled={visualCommand !== null || (visualServo?.mode === 'follow' && !displayedFollowTask)}
              title={visualServo?.mode === 'follow' && !displayedFollowTask
                ? text(locale, 'Follow task reference is not ready', '跟随任务引用尚未就绪')
                : undefined}
            >
              {text(locale, 'Stop tracking', '停止跟随')}
            </button>
          </div>
          {displayedFollowTask && (
            <small className={styles.hotSwitch}>
              {text(locale, 'Follow task', '跟随任务')} {displayedFollowTask.follow_task_id}
              {displayedFollowState && ` · ${displayedFollowState}`}
            </small>
          )}
          {visualServo && (
            <>
              {followStatus && (
                <div className={styles.followStatus}>
                  <div>
                    <span>{text(locale, 'Visual tracking', '视觉跟踪')}</span>
                    <strong>{followStatus.visual.label}</strong>
                  </div>
                  {followStatus.visual.message && <small>{followStatus.visual.message}</small>}
                  {followStatus.navigation && (
                    <div>
                      <span>{text(locale, 'Follow navigation', '跟随导航')}</span>
                      <strong>{followStatus.navigation.label}</strong>
                      {followStatus.navigation.message && <small>{followStatus.navigation.message}</small>}
                    </div>
                  )}
                  <div>
                    <span>{text(locale, 'Robot motion', '机器人运动')}</span>
                    <strong>{followStatus.motion.observation}</strong>
                    <small>
                      {followStatus.motion.permission} · {followStatus.motion.stopConfirmation}
                    </small>
                    {followStatus.motion.reasonMessage && <small>{followStatus.motion.reasonMessage}</small>}
                  </div>
                </div>
              )}
              {trackedPersonId && trackedPersonPosition && (
                <small className={styles.hotSwitch}>
                  {visualServo?.target_visible
                    ? text(locale, 'Person locked', '人员已锁定')
                    : text(locale, 'Person temporarily lost', '人员暂时丢失')}
                  {trackedIdentityId && (
                    <> · {text(locale, 'Target', '目标')} {trackedIdentityId.slice(0, 8)}</>
                  )}
                  {' · '}{text(locale, 'Track', '轨迹')} {trackedPersonId}
                  {' · '}{visualServo.target_visible
                    ? text(locale, 'Map position', '地图位置')
                    : text(locale, 'Last observed map position', '最后观测地图位置')}
                  {' '}({trackedPersonPosition[0]?.toFixed(1)}, {trackedPersonPosition[1]?.toFixed(1)})
                  {typeof visualServo.distance_m === 'number' && (
                    <> · {visualServo.distance_m.toFixed(1)} m</>
                  )}
                </small>
              )}
              {visualServo.target_id && !visualServo.person && (
                <small className={styles.hotSwitch}>
                  {text(locale, 'Waiting for person', '等待人员')} · ID {visualServo.target_id}
                </small>
              )}
            </>
          )}
        </div>
      )}
    </section>
  )
}
