import { dashboardFetch } from './observationMode.ts'

export type InspectionScheduleFrequency = 'once' | 'daily' | 'weekly'

export interface InspectionScheduleInput {
  name: string
  route_id: string
  map_id: string
  revision: number
  start_date: string
  end_date: string
  local_time: string
  frequency: InspectionScheduleFrequency
  weekdays: number[]
  timezone: 'Asia/Shanghai'
  enabled: boolean
}

export interface InspectionSchedule extends InspectionScheduleInput {
  id: string
  next_run_at: string | null
  created_at: number
  updated_at: number
}

export interface InspectionScheduleOccurrence {
  id: string
  schedule_id: string
  scheduled_for: string
  state: 'dispatching' | 'submitted' | 'rejected' | 'missed' | 'unknown'
  reason: string
  task_id: string | null
  request_id: string
}

export interface InspectionScheduleSnapshot {
  timezone: 'Asia/Shanghai'
  scheduler_running: boolean
  schedules: InspectionSchedule[]
  occurrences: InspectionScheduleOccurrence[]
}

export function inspectionDateTimestamp(value: string): number | null {
  if (!/^\d{4}-\d{2}-\d{2}$/.test(value)) return null
  const timestamp = Date.parse(`${value}T00:00:00Z`)
  return Number.isFinite(timestamp) && new Date(timestamp).toISOString().slice(0, 10) === value ? timestamp : null
}

export function inspectionToday(now = new Date()): string {
  const parts = new Intl.DateTimeFormat('en', { timeZone: 'Asia/Shanghai', year: 'numeric', month: '2-digit', day: '2-digit' }).formatToParts(now)
  const part = (type: string) => parts.find(item => item.type === type)!.value
  return `${part('year')}-${part('month')}-${part('day')}`
}

export function shiftInspectionDate(date: string, days: number): string {
  const timestamp = inspectionDateTimestamp(date)
  if (timestamp === null) throw new Error('日期无效。')
  return new Date(timestamp + days * 86_400_000).toISOString().slice(0, 10)
}

export function inspectionWeekday(date: string): number {
  const timestamp = inspectionDateTimestamp(date)
  if (timestamp === null) throw new Error('日期无效。')
  return (new Date(timestamp).getUTCDay() + 6) % 7
}

export function inspectionMonthCells(month: string): Array<{ date: string; day: number; currentMonth: boolean }> {
  const first = `${month}-01`
  const timestamp = inspectionDateTimestamp(first)
  if (timestamp === null) throw new Error('月份无效。')
  const year = Number(month.slice(0, 4))
  const monthNumber = Number(month.slice(5))
  const days = new Date(Date.UTC(year, monthNumber, 0)).getUTCDate()
  const offset = inspectionWeekday(first)
  return Array.from({ length: Math.ceil((days + offset) / 7) * 7 }, (_, index) => {
    const date = shiftInspectionDate(first, index - offset)
    return { date, day: Number(date.slice(-2)), currentMonth: date.startsWith(month) }
  })
}

export function inspectionScheduleMatchesDate(schedule: InspectionScheduleInput, date: string): boolean {
  if (date < schedule.start_date || date > schedule.end_date) return false
  if (schedule.frequency === 'once') return date === schedule.start_date
  return schedule.frequency === 'daily' || schedule.weekdays.includes(inspectionWeekday(date))
}

export function inspectionScheduleInput(schedule: InspectionSchedule): InspectionScheduleInput {
  const { name, route_id, map_id, revision, start_date, end_date, local_time, frequency, weekdays, timezone, enabled } = schedule
  return { name, route_id, map_id, revision, start_date, end_date, local_time, frequency, weekdays, timezone, enabled }
}

function validSchedule(value: unknown): value is InspectionSchedule {
  if (!value || typeof value !== 'object') return false
  const item = value as InspectionSchedule
  return ['id', 'name', 'route_id', 'map_id', 'start_date', 'end_date', 'local_time'].every(key => typeof (item as unknown as Record<string, unknown>)[key] === 'string')
    && inspectionDateTimestamp(item.start_date) !== null && inspectionDateTimestamp(item.end_date) !== null
    && item.start_date <= item.end_date && /^([01]\d|2[0-3]):[0-5]\d$/.test(item.local_time)
    && ['once', 'daily', 'weekly'].includes(item.frequency) && item.timezone === 'Asia/Shanghai'
    && Number.isInteger(item.revision) && item.revision > 0 && typeof item.enabled === 'boolean'
    && Array.isArray(item.weekdays) && item.weekdays.every(day => Number.isInteger(day) && day >= 0 && day <= 6)
    && (item.next_run_at === null || (typeof item.next_run_at === 'string' && Number.isFinite(Date.parse(item.next_run_at))))
    && Number.isFinite(item.created_at) && Number.isFinite(item.updated_at)
}

async function request(path: string, method = 'GET', body?: object): Promise<Record<string, unknown>> {
  const response = await dashboardFetch(path, {
    method, headers: body ? { 'Content-Type': 'application/json' } : undefined,
    body: body ? JSON.stringify(body) : undefined, signal: AbortSignal.timeout(15_000),
  })
  const data = await response.json().catch(() => null)
  if (!response.ok) {
    const detail = data?.detail
    const message = typeof detail === 'string' ? detail : detail?.message ?? data?.message
    const reason = detail?.reason ?? data?.reason ?? data?.error
    const reasons: Record<string, string> = {
      schedule_has_no_future_occurrence: '这个时间范围已没有可执行的时段，请调整日期、时间或重复星期。',
      schedule_invalid_date_range: '开始和结束日期不匹配，请重新选择日期范围。',
      schedule_not_found: '该计划已不存在，请刷新后重新选择。',
      inspection_route_revision_mismatch: '路线版本已更新，请重新绑定已保存路线。',
    }
    throw new Error(reasons[reason] || message || (response.status === 404 || response.status === 503 ? '巡检计划服务暂不可用。' : `巡检计划请求失败（${response.status}）。`))
  }
  if (!data || typeof data !== 'object') throw new Error('巡检计划响应无效。')
  return data
}

const schedulesPath = '/api/v1/inspection/schedules'

export async function fetchInspectionSchedules(start: string, end: string): Promise<InspectionScheduleSnapshot> {
  const data = await request(`${schedulesPath}?${new URLSearchParams({ start, end })}`)
  if (data.timezone !== 'Asia/Shanghai' || typeof data.scheduler_running !== 'boolean'
    || !Array.isArray(data.schedules) || !data.schedules.every(validSchedule)
    || !Array.isArray(data.occurrences) || data.occurrences.some(item => !item || typeof item.id !== 'string'
      || typeof item.schedule_id !== 'string' || !Number.isFinite(Date.parse(item.scheduled_for))
      || !['dispatching', 'submitted', 'rejected', 'missed', 'unknown'].includes(item.state)
      || typeof item.reason !== 'string' || typeof item.request_id !== 'string'
      || (item.task_id != null && typeof item.task_id !== 'string'))) throw new Error('巡检计划数据格式无效。')
  return data as unknown as InspectionScheduleSnapshot
}

export async function saveInspectionSchedule(input: InspectionScheduleInput, id?: string): Promise<InspectionSchedule> {
  const data = await request(id ? `${schedulesPath}/${encodeURIComponent(id)}` : schedulesPath, id ? 'PUT' : 'POST', input)
  if (!validSchedule(data)) throw new Error('巡检计划保存结果无效，请刷新核对。')
  return data
}

export async function deleteInspectionSchedule(id: string): Promise<void> {
  await request(`${schedulesPath}/${encodeURIComponent(id)}`, 'DELETE')
}
