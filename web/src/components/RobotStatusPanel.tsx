import { useCallback, useState } from 'react'
import { LocateFixed, RefreshCw, Route } from 'lucide-react'
import { text, type Locale } from '../i18n'
import * as api from '../services/api'
import type { SSEState, ToastKind } from '../types'
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
  const session = sseState.session ?? refreshedSession
  const visualServo = sseState.visualServoStatus
  const followAvailable = visualServo?.follow_available === true
  const trackedPersonId = visualServo?.person?.id ?? null
  const trackedPersonPosition = visualServo?.person?.position
  const visiblePeople = (sseState.sceneGraph?.objects ?? []).filter(
    person => person.id && ['person', 'people', 'human', 'pedestrian'].includes(person.label.toLowerCase()),
  )

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
    text(locale, 'No active task', '当前无任务'),
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
    if (mode !== 'stop' && !motionStartAllowed) {
      showToast(motionStartBlockedReason, 'error')
      return
    }
    if (mode !== 'stop' && !target && !(mode === 'follow' && selectedPersonId)) {
      showToast(text(locale, 'Enter the person or object to track', '请输入要寻找或跟随的人或物体'), 'error')
      return
    }

    setVisualCommand(mode)
    try {
      const response = await api.sendVisualServo(mode, target || null, selectedPersonId)
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
  }, [locale, motionStartAllowed, motionStartBlockedReason, selectedPersonId, showToast, visualServo?.target, visualTarget])

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
        <div><span>{text(locale, 'Task', '任务')}</span><strong>{product}</strong></div>
        <div><span>Env</span><strong>{session?.env ?? '--'}</strong></div>
        <div><span>{text(locale, 'Map', '地图')}</span><strong>{map}</strong></div>
        <div><span>{text(locale, 'Localization', '定位')}</span><strong>{backend}</strong></div>
        <div><span>{text(locale, 'Cloud', '点云')}</span><strong>{cloudPoints.toLocaleString()}</strong></div>
        <div>
          <span>{text(locale, 'State', '状态')}</span>
          <strong className={ready ? styles.ready : styles.waiting}>
            {ready ? text(locale, 'Ready', '就绪') : text(locale, 'Waiting', '等待')}
          </strong>
        </div>
      </div>

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
              {visualServo?.target_id
                ? ` · ID ${visualServo.target_id}`
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
              disabled={visualCommand !== null}
            >
              {text(locale, 'Stop', '停止')}
            </button>
          </div>
          {visualServo && (
            <>
              <small className={styles.hotSwitch}>
                {visualServo.state === 'stopping'
                  ? text(locale, 'Stopping', '正在停止')
                  : visualServo.state === 'following'
                    ? text(locale, 'Following person', '正在跟人')
                    : visualServo.state === 'lost'
                      ? text(locale, 'Target lost', '目标丢失')
                      : visualServo.state === 'failed'
                        ? text(locale, 'Navigation failed', '导航失败')
                  : !visualServo.follow_available
                    ? text(locale, 'Person selection unavailable', '人员选择不可用')
                    : visualServo.mode === 'idle'
                      ? text(locale, 'Ready to switch task', '可直接切换任务')
                      : visualServo.select === 'unavailable'
                        ? text(locale, 'Target selection unavailable', '目标选择不可用')
                        : text(locale, 'Waiting for target lock', '等待锁定目标')}
              </small>
              {trackedPersonId && trackedPersonPosition && (
                <small className={styles.hotSwitch}>
                  {visualServo?.target_visible
                    ? text(locale, 'Person locked', '人员已锁定')
                    : text(locale, 'Person temporarily lost', '人员暂时丢失')}
                  {' · '}ID {trackedPersonId}
                  {' · '}map ({trackedPersonPosition[0]?.toFixed(1)}, {trackedPersonPosition[1]?.toFixed(1)})
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
