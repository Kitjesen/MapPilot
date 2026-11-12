import { useCallback, useEffect, useMemo, useRef, useState, type ReactNode } from 'react'
import { ChevronLeft, ChevronRight, Clock3, Pencil, Trash2 } from 'lucide-react'
import type { InspectionRoute } from '../types'
import { text, type Locale } from '../i18n'
import { isObservationMode } from '../services/observationMode'
import {
  deleteInspectionSchedule, fetchInspectionSchedules, inspectionMonthCells, inspectionScheduleInput,
  inspectionScheduleMatchesDate, inspectionToday, inspectionWeekday, saveInspectionSchedule, shiftInspectionDate,
  type InspectionSchedule, type InspectionScheduleFrequency, type InspectionScheduleInput, type InspectionScheduleOccurrence,
  type InspectionScheduleSnapshot,
} from '../services/inspectionSchedules'
import styles from './InspectionCalendar.module.css'

interface InspectionCalendarProps {
  routes: InspectionRoute[]
  locale: Locale
  onOpenTask: (taskId: string) => void
  routeSettings?: ReactNode
  routeUnavailable?: boolean
}

interface ScheduleDraft {
  name: string
  routeKey: string
  revision: number
  start_date: string
  end_date: string
  local_time: string
  frequency: InspectionScheduleFrequency
  weekdays: number[]
  enabled: boolean
}

const WEEKDAYS = ['一', '二', '三', '四', '五', '六', '日']
const DAY = ['Mon', 'Tue', 'Wed', 'Thu', 'Fri', 'Sat', 'Sun']
const routeKey = (route: InspectionRoute) => `${route.map_id}/${route.id}`

function newDraft(date: string): ScheduleDraft {
  return { name: '', routeKey: '', revision: 0, start_date: date, end_date: date, local_time: '09:00',
    frequency: 'once', weekdays: [inspectionWeekday(date)], enabled: false }
}

function displayDate(date: string, locale: Locale): string {
  if (!date) return text(locale, 'Choose date', '选择日期')
  return new Intl.DateTimeFormat(locale === 'zh' ? 'zh-CN' : 'en-US', {
    month: 'short', day: 'numeric', timeZone: 'UTC',
  }).format(new Date(`${date}T00:00:00Z`))
}

function occurrenceTime(value: string, locale: Locale): string {
  return new Intl.DateTimeFormat(locale === 'zh' ? 'zh-CN' : 'en-US', {
    timeZone: 'Asia/Shanghai', hour: '2-digit', minute: '2-digit', hour12: false,
  }).format(new Date(value))
}

function reasonLabel(reason: string, locale: Locale): string {
  const reasons: Record<string, [string, string]> = {
    inspection_active_map_mismatch: ['Active map differs', '当前地图不匹配'],
    inspection_route_revision_mismatch: ['Route has changed', '路线版本已更新'],
    inspection_native_unavailable: ['Robot service unavailable', '机器人服务不可用'],
    inspection_recording_start_failed: ['Evidence recording unavailable', '取证录制未就绪'],
    scheduler_offline: ['Scheduler was offline', '定时服务离线'],
    start_window_missed: ['Scheduled time passed', '已错过触发时间'],
    schedule_changed: ['Plan changed or disabled', '计划已修改或停用'],
    inspection_schedule_task_active: ['Another task is active', '已有任务正在执行'],
    inspection_schedule_control_unavailable: ['Inspection control unavailable', '巡检控制不可用'],
    inspection_schedule_admission_blocked: ['Start conditions are not met', '启动条件不满足'],
    inspection_schedule_motion_held: ['Motion is held', '机器人处于运动保持'],
    scheduler_stopped: ['Scheduler stopped; verify task state', '定时服务已停止，请核对任务'],
    inspection_dispatch_unconfirmed: ['Submission outcome is unconfirmed', '提交结果待核对'],
    missed: ['Scheduled time passed', '已错过计划时间'],
    host_restarted: ['Service restarted; verify task state', '服务已重启，请核对任务状态'],
  }
  return reasons[reason] ? text(locale, ...reasons[reason]) : reason
}

export function InspectionCalendar({ routes, locale, onOpenTask, routeSettings, routeUnavailable }: InspectionCalendarProps) {
  const [today, setToday] = useState(() => inspectionToday())
  const [month, setMonth] = useState(today.slice(0, 7))
  const [selectedDate, setSelectedDate] = useState(today)
  const [draft, setDraft] = useState(() => newDraft(today))
  const [selectingEnd, setSelectingEnd] = useState(false)
  const [editingId, setEditingId] = useState<string | null>(null)
  const [snapshot, setSnapshot] = useState<InspectionScheduleSnapshot | null>(null)
  const [loading, setLoading] = useState(false)
  const [busyId, setBusyId] = useState<string | null>(null)
  const [error, setError] = useState('')
  const [readError, setReadError] = useState('')
  const [saved, setSaved] = useState('')
  const readGeneration = useRef(0)
  const actionBusy = useRef(false)
  const observe = isObservationMode()
  const cells = useMemo(() => inspectionMonthCells(month), [month])
  const start = cells[0].date
  const end = cells[cells.length - 1].date
  const choices = routes.filter(route => Boolean(route.map_id) && Number.isInteger(route.revision) && Number(route.revision) > 0)
  const selectedRoute = choices.find(route => routeKey(route) === draft.routeKey)
    ?? (!editingId && !draft.routeKey ? choices[0] : undefined)
  const routeRevision = draft.revision || Number(selectedRoute?.revision ?? 0)
  const revisionChanged = Boolean(selectedRoute && routeRevision !== Number(selectedRoute.revision))

  const refresh = useCallback(async () => {
    const generation = ++readGeneration.current
    setToday(inspectionToday())
    setLoading(true)
    try {
      const next = await fetchInspectionSchedules(start, end)
      if (generation !== readGeneration.current) return
      setSnapshot(next)
      setReadError('')
    } catch (failure) {
      if (generation === readGeneration.current) setReadError(failure instanceof Error ? failure.message : String(failure))
    } finally { if (generation === readGeneration.current) setLoading(false) }
  }, [start, end])

  useEffect(() => {
    const requests = readGeneration
    void refresh()
    const timer = setInterval(() => { void refresh() }, 10_000)
    return () => { requests.current++; clearInterval(timer) }
  }, [refresh])

  const schedules = snapshot?.schedules ?? []
  const daySchedules = schedules.filter(schedule => inspectionScheduleMatchesDate(schedule, selectedDate))
    .sort((left, right) => left.local_time.localeCompare(right.local_time))
  const dayOccurrences = (snapshot?.occurrences ?? []).filter(item => inspectionToday(new Date(item.scheduled_for)) === selectedDate)
    .sort((left, right) => right.scheduled_for.localeCompare(left.scheduled_for))

  const chooseDate = (date: string) => {
    setSelectedDate(date)
    setMonth(date.slice(0, 7))
    setSaved('')
    if (draft.frequency === 'once') {
      setDraft(current => ({ ...current, start_date: date, end_date: date }))
      return
    }
    if (selectingEnd && draft.start_date) {
      setDraft(current => ({ ...current, start_date: date < current.start_date ? date : current.start_date,
        end_date: date < current.start_date ? current.start_date : date }))
    } else setDraft(current => ({ ...current, start_date: date, end_date: date }))
    setSelectingEnd(!selectingEnd)
  }

  const moveMonth = (direction: number) => {
    const date = new Date(`${month}-01T00:00:00Z`)
    date.setUTCMonth(date.getUTCMonth() + direction)
    const next = date.toISOString().slice(0, 10)
    setMonth(next.slice(0, 7))
    setSelectedDate(next)
  }

  const edit = (schedule: InspectionSchedule) => {
    setEditingId(schedule.id)
    setDraft({ name: schedule.name, routeKey: `${schedule.map_id}/${schedule.route_id}`, revision: schedule.revision,
      start_date: schedule.start_date, end_date: schedule.end_date, local_time: schedule.local_time,
      frequency: schedule.frequency, weekdays: schedule.weekdays, enabled: schedule.enabled })
    setSelectedDate(schedule.start_date)
    setMonth(schedule.start_date.slice(0, 7))
    setSelectingEnd(false)
    setSaved('')
  }

  const mutate = async (id: string, operation: () => Promise<unknown>) => {
    if (actionBusy.current || observe) return
    actionBusy.current = true
    setBusyId(id)
    setError('')
    setSaved('')
    try {
      await operation()
      await refresh()
    } catch (failure) {
      setError(failure instanceof Error ? failure.message : String(failure))
    } finally { actionBusy.current = false; setBusyId(null) }
  }

  const submit = async () => {
    if (!selectedRoute || revisionChanged || !draft.start_date || !draft.end_date || (draft.frequency === 'weekly' && !draft.weekdays.length)) return
    const input: InspectionScheduleInput = {
      name: draft.name.trim() || selectedRoute.name || selectedRoute.id, route_id: selectedRoute.id,
      map_id: selectedRoute.map_id!, revision: routeRevision, start_date: draft.start_date,
      end_date: draft.frequency === 'once' ? draft.start_date : draft.end_date, local_time: draft.local_time,
      frequency: draft.frequency, weekdays: draft.weekdays, timezone: 'Asia/Shanghai', enabled: draft.enabled,
    }
    await mutate('save', async () => {
      const result = await saveInspectionSchedule(input, editingId ?? undefined)
      setEditingId(result.id)
      setDraft(current => ({ ...current, routeKey: `${result.map_id}/${result.route_id}`, revision: result.revision }))
      setSaved(text(locale, result.enabled ? 'Plan saved and enabled' : 'Plan saved', result.enabled ? '计划已保存并启用' : '计划已保存'))
    })
  }

  const occurrenceLabel = (state: InspectionScheduleOccurrence['state']) => ({
    dispatching: text(locale, 'Submitting', '正在提交'), submitted: text(locale, 'Submitted', '已提交'),
    rejected: text(locale, 'Not started', '未启动'), missed: text(locale, 'Missed', '已错过'),
    unknown: text(locale, 'Verify task', '待核对任务'),
  })[state]

  return <section className={styles.root} aria-label={text(locale, 'Inspection calendar', '巡检日历')}>
    {(error || readError) && <p className={styles.error} role="alert">{error || readError}</p>}
    <div className={styles.layout}>
      <div className={styles.calendarColumn}>
        <section className={styles.picker} aria-label={text(locale, 'Schedule dates', '计划日期')}>
          {draft.frequency !== 'once' && <div className={styles.rangeFields}>
            <button type="button" data-active={!selectingEnd} disabled={busyId !== null} onClick={() => setSelectingEnd(false)}>
              <span>{text(locale, 'Start date', '开始日期')}</span><strong>{displayDate(draft.start_date, locale)}</strong></button>
            <button type="button" data-active={selectingEnd} disabled={busyId !== null} onClick={() => setSelectingEnd(true)}>
              <span>{text(locale, 'End date', '结束日期')}</span><strong>{displayDate(draft.end_date, locale)}</strong></button>
          </div>}
          <div className={styles.calendarPaper}>
            <div className={styles.monthHeading}><strong>{new Intl.DateTimeFormat(locale === 'zh' ? 'zh-CN' : 'en-US', { year: 'numeric', month: 'long', timeZone: 'UTC' }).format(new Date(`${month}-01T00:00:00Z`))}</strong>
              <div><button type="button" className={styles.iconButton} disabled={busyId !== null} onClick={() => moveMonth(-1)} aria-label={text(locale, 'Previous month', '上个月')}><ChevronLeft size={17} /></button>
                <button type="button" className={styles.iconButton} disabled={busyId !== null} onClick={() => moveMonth(1)} aria-label={text(locale, 'Next month', '下个月')}><ChevronRight size={17} /></button></div></div>
            <div className={styles.weekHeading} aria-hidden="true">{WEEKDAYS.map((day, index) => <span key={day}>{locale === 'zh' ? day : DAY[index]}</span>)}</div>
            <div className={styles.days} role="group" aria-label={text(locale, 'Choose a date', '选择日期')}>
              {cells.map(cell => {
                const inRange = cell.date >= draft.start_date && cell.date <= draft.end_date
                const endpoint = cell.date === draft.start_date || cell.date === draft.end_date
                const hasPlan = schedules.some(schedule => inspectionScheduleMatchesDate(schedule, cell.date))
                return <button type="button" key={cell.date} className={styles.day} disabled={busyId !== null}
                  data-muted={!cell.currentMonth} data-range={inRange} data-endpoint={endpoint}
                  data-start={cell.date === draft.start_date} data-end={cell.date === draft.end_date}
                  data-week-start={inspectionWeekday(cell.date) === 0} data-week-end={inspectionWeekday(cell.date) === 6}
                  aria-label={cell.date} aria-pressed={cell.date === selectedDate} aria-current={cell.date === today ? 'date' : undefined}
                  onClick={() => chooseDate(cell.date)}><span>{cell.day}</span>{hasPlan && <i aria-label={text(locale, 'Has plans', '有计划')} />}</button>
              })}
            </div>
          </div>
          <div className={styles.quickDates}>
            <button type="button" disabled={busyId !== null} onClick={() => chooseDate(today)}>{text(locale, 'Today', '今天')}</button>
            <button type="button" disabled={busyId !== null} onClick={() => chooseDate(shiftInspectionDate(today, 1))}>{text(locale, 'Tomorrow', '明天')}</button>
            <button type="button" disabled={busyId !== null} onClick={() => {
              const first = shiftInspectionDate(today, -inspectionWeekday(today))
              setDraft(current => ({ ...current, start_date: first, end_date: shiftInspectionDate(first, 6), frequency: 'daily' }))
              setSelectedDate(today); setMonth(today.slice(0, 7)); setSelectingEnd(false)
            }}>{text(locale, 'This week', '本周')}</button>
            <button type="button" disabled={busyId !== null} onClick={() => { setDraft(current => ({ ...current, start_date: '', end_date: '' })); setSelectingEnd(false); setSaved('') }}>{text(locale, 'Clear', '清除')}</button>
          </div>
        </section>
        <form className={styles.form} onSubmit={event => { event.preventDefault(); void submit() }}>
          <header><strong>{displayDate(selectedDate, locale)}<span className={styles.headingHint}>{text(locale, editingId ? 'Edit plan' : 'New plan', editingId ? '编辑计划' : '新建计划')}</span></strong>
            {editingId && <button type="button" className={styles.textButton} disabled={busyId !== null} onClick={() => { setEditingId(null); setDraft(newDraft(selectedDate)); setSaved('') }}>{text(locale, 'New plan', '新建')}</button>}</header>
          <label className={styles.fullField}><span>{text(locale, 'Name', '计划名称')}</span><input value={draft.name} maxLength={80} placeholder={selectedRoute?.name || text(locale, 'Inspection plan', '巡检计划')} disabled={observe || busyId !== null} onChange={event => setDraft(current => ({ ...current, name: event.target.value }))} /></label>
          <label className={styles.fullField}><span>{text(locale, 'Route', '巡检路线')}</span>
            <select aria-label={text(locale, 'Scheduled route', '计划使用的路线')} value={selectedRoute ? routeKey(selectedRoute) : ''} disabled={busyId !== null || observe}
              onChange={event => { const route = choices.find(item => routeKey(item) === event.target.value); setDraft(current => ({ ...current, routeKey: event.target.value, revision: Number(route?.revision ?? 0) })) }}>
              {!selectedRoute && <option value="">{routeUnavailable ? text(locale, 'Routes unavailable', '路线暂不可用') : text(locale, 'Choose a saved route', '选择已保存路线')}</option>}
              {choices.map(route => <option key={routeKey(route)} value={routeKey(route)}>{route.name || route.id} · v{route.revision}</option>)}</select></label>
          {routeSettings && <div className={styles.routeSettings}>{routeSettings}</div>}
          {revisionChanged && <button type="button" className={styles.textButton} onClick={() => setDraft(current => ({ ...current, revision: Number(selectedRoute!.revision) }))}>{text(locale, `Use saved route v${selectedRoute!.revision}`, `更新到已保存路线 v${selectedRoute!.revision}`)}</button>}
          <div className={styles.formRow}><label><span>{text(locale, 'Time', '执行时间')}</span><input type="time" required aria-label={text(locale, 'Execution time', '执行时间')} value={draft.local_time} disabled={observe || busyId !== null} onChange={event => setDraft(current => ({ ...current, local_time: event.target.value }))} /></label>
            <label><span>{text(locale, 'Repeat', '重复')}</span><select value={draft.frequency} aria-label={text(locale, 'Repeat schedule', '重复规则')} disabled={observe || busyId !== null} onChange={event => {
              const frequency = event.target.value as InspectionScheduleFrequency
              setDraft(current => ({ ...current, frequency, end_date: frequency === 'once' ? current.start_date : current.end_date }))
              setSelectingEnd(frequency !== 'once')
            }}><option value="once">{text(locale, 'Once', '单次')}</option><option value="daily">{text(locale, 'Daily', '每天')}</option><option value="weekly">{text(locale, 'Weekly', '每周')}</option></select></label></div>
          {draft.frequency === 'weekly' && <div className={styles.weekdays} role="group" aria-label={text(locale, 'Repeat weekdays', '重复星期')}>
            {WEEKDAYS.map((day, index) => <button type="button" key={day} aria-label={locale === 'zh' ? `周${day}` : DAY[index]} aria-pressed={draft.weekdays.includes(index)} disabled={observe || busyId !== null} onClick={() => setDraft(current => ({ ...current, weekdays: current.weekdays.includes(index) ? current.weekdays.filter(value => value !== index) : [...current.weekdays, index].sort() }))}>{locale === 'zh' ? day : DAY[index]}</button>)}</div>}
          <footer><label className={styles.enable}><input type="checkbox" checked={draft.enabled} disabled={observe || busyId !== null} onChange={event => setDraft(current => ({ ...current, enabled: event.target.checked }))} />{text(locale, 'Enable plan', '启用计划')}</label>
            <button type="submit" className={styles.primaryButton} disabled={observe || busyId !== null || !selectedRoute || revisionChanged || !draft.start_date || !draft.end_date || (draft.frequency === 'weekly' && !draft.weekdays.length)}>{busyId === 'save' ? text(locale, 'Saving…', '保存中…') : text(locale, 'Save plan', '保存计划')}</button></footer>
          {!choices.length && !routeUnavailable && <p className={styles.note}>{text(locale, 'Save an inspection route first', '先保存巡检路线')}</p>}
          {saved && <p className={styles.saved} role="status">{saved}</p>}
        </form>
      </div>
      <section className={styles.agenda} aria-label={text(locale, 'Plans for selected day', '所选日期的计划')}>
        <header><strong>{text(locale, 'Plans for this day', '当天安排')}</strong><span>{daySchedules.length}</span></header>
        {daySchedules.length === 0 && <p className={styles.empty}>{loading && !snapshot ? text(locale, 'Loading plans…', '正在读取计划…') : text(locale, 'No plans for this day', '当天暂无计划')}</p>}
        <div className={styles.planList}>{daySchedules.map(schedule => <article key={schedule.id} className={styles.plan} data-enabled={schedule.enabled}>
          <div className={styles.planTime}><Clock3 size={15} /><strong>{schedule.local_time}</strong></div>
          <div className={styles.planBody}><strong>{schedule.name}</strong><span>{schedule.route_id} · v{schedule.revision} · {text(locale, schedule.frequency === 'once' ? 'Once' : schedule.frequency === 'daily' ? 'Daily' : 'Weekly', schedule.frequency === 'once' ? '单次' : schedule.frequency === 'daily' ? '每天' : '每周')}</span>
            <small>{schedule.enabled ? text(locale, 'Enabled', '已启用') : text(locale, 'Disabled', '未启用')}</small></div>
          <div className={styles.planActions}>
            <button type="button" className={styles.textButton} disabled={observe || busyId !== null} onClick={() => void mutate(schedule.id, async () => {
              const updated = await saveInspectionSchedule({ ...inspectionScheduleInput(schedule), enabled: !schedule.enabled }, schedule.id)
              if (editingId === schedule.id) setDraft(current => ({ ...current, enabled: updated.enabled }))
            })}>{text(locale, schedule.enabled ? 'Disable' : 'Enable', schedule.enabled ? '停用' : '启用')}</button>
            <button type="button" className={styles.iconButton} onClick={() => edit(schedule)} disabled={busyId !== null} aria-label={text(locale, `Edit ${schedule.name}`, `编辑 ${schedule.name}`)}><Pencil size={15} /></button>
            <button type="button" className={styles.iconButton} disabled={observe || busyId !== null} onClick={() => void mutate(schedule.id, async () => {
              await deleteInspectionSchedule(schedule.id)
              if (editingId === schedule.id) { setEditingId(null); setDraft(newDraft(selectedDate)) }
            })} aria-label={text(locale, `Delete ${schedule.name}`, `删除 ${schedule.name}`)}><Trash2 size={15} /></button>
          </div>
        </article>)}</div>
        <details className={styles.history} open={dayOccurrences.length > 0 || undefined}>
        <summary className={styles.historyHeading}><strong>{text(locale, 'Execution records', '执行记录')}</strong><span>{dayOccurrences.length}</span></summary>
        {snapshot && !snapshot.scheduler_running && <p className={styles.note}>{text(locale, 'Scheduler offline', '定时服务未运行')}</p>}
        {dayOccurrences.length === 0 && <p className={styles.empty}>{text(locale, 'No execution records', '暂无执行记录')}</p>}
        {dayOccurrences.map(occurrence => <article className={styles.occurrence} key={occurrence.id}>
          <time>{occurrenceTime(occurrence.scheduled_for, locale)}</time><div><strong>{occurrenceLabel(occurrence.state)}</strong>
            <span>{schedules.find(schedule => schedule.id === occurrence.schedule_id)?.name ?? text(locale, 'Deleted plan', '已删除计划')}</span>
            {occurrence.reason && <small>{reasonLabel(occurrence.reason, locale)}</small>}</div>
          {occurrence.task_id && <button type="button" className={styles.textButton} onClick={() => onOpenTask(occurrence.task_id!)}>{text(locale, 'View task', '查看任务')}</button>}
        </article>)}
        </details>
      </section>
    </div>
  </section>
}
