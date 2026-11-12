import assert from 'node:assert/strict'
import { createRequire } from 'node:module'
import { readFileSync } from 'node:fs'
import { runInNewContext } from 'node:vm'
import test from 'node:test'
import ts from 'typescript'
import * as schedules from '../src/services/inspectionSchedules.ts'
import type { InspectionSchedule, InspectionScheduleSnapshot } from '../src/services/inspectionSchedules.ts'

const schedule: InspectionSchedule = {
  id: 'morning-plan', name: '晨间巡检', route_id: 'route-1', map_id: 'room', revision: 3,
  start_date: '2026-11-10', end_date: '2026-11-20', local_time: '09:00', frequency: 'daily', weekdays: [],
  timezone: 'Asia/Shanghai', enabled: false, next_run_at: null, created_at: 1, updated_at: 1,
}

test('calendar dates preserve leap days month boundaries and Beijing midnight', () => {
  assert.equal(schedules.inspectionDateTimestamp('2025-02-29'), null)
  assert.equal(schedules.inspectionDateTimestamp('2024-02-29') !== null, true)
  assert.equal(schedules.shiftInspectionDate('2026-12-31', 1), '2027-01-01')
  assert.equal(schedules.shiftInspectionDate('2024-03-01', -1), '2024-02-29')
  assert.equal(schedules.inspectionToday(new Date('2026-11-09T16:30:00Z')), '2026-11-10')
  const november = schedules.inspectionMonthCells('2026-11')
  assert.equal(november[0].date, '2026-10-26')
  assert.equal(november.length, 42)
  assert.equal(november.filter(day => day.currentMonth).length, 30)
  const february = schedules.inspectionMonthCells('2024-02')
  assert.equal(february.filter(day => day.currentMonth).length, 29)
})

test('date ranges include both endpoints and weekly rules use Monday zero', () => {
  assert.equal(schedules.inspectionScheduleMatchesDate(schedule, '2026-11-10'), true)
  assert.equal(schedules.inspectionScheduleMatchesDate(schedule, '2026-11-20'), true)
  assert.equal(schedules.inspectionScheduleMatchesDate(schedule, '2026-11-21'), false)
  assert.equal(schedules.inspectionScheduleMatchesDate({ ...schedule, frequency: 'once' }, '2026-11-11'), false)
  const weekly = { ...schedule, frequency: 'weekly' as const, weekdays: [1] }
  assert.equal(schedules.inspectionWeekday('2026-11-10'), 1)
  assert.equal(schedules.inspectionScheduleMatchesDate(weekly, '2026-11-10'), true)
  assert.equal(schedules.inspectionScheduleMatchesDate(weekly, '2026-11-11'), false)
})

function compile(path: string) {
  return ts.transpileModule(readFileSync(new URL(path, import.meta.url), 'utf8'), {
    compilerOptions: { module: ts.ModuleKind.CommonJS, target: ts.ScriptTarget.ES2022, jsx: ts.JsxEmit.ReactJSX },
  }).outputText
}

test('schedule HTTP contracts keep saved route revision and reject malformed snapshots', async () => {
  let body: unknown = schedule
  let status = 200
  const requests: Array<{ path: string; init: RequestInit }> = []
  const exports: Record<string, unknown> = {}
  runInNewContext(compile('../src/services/inspectionSchedules.ts'), {
    exports, require: () => ({ dashboardFetch: async (path: string, init: RequestInit) => {
      requests.push({ path, init })
      return new Response(JSON.stringify(body), { status })
    } }), AbortSignal, Error, URLSearchParams, Date, Intl,
  })
  const save = exports.saveInspectionSchedule as typeof schedules.saveInspectionSchedule
  const read = exports.fetchInspectionSchedules as typeof schedules.fetchInspectionSchedules
  const remove = exports.deleteInspectionSchedule as typeof schedules.deleteInspectionSchedule
  await save(schedules.inspectionScheduleInput(schedule))
  assert.equal(requests[0].init.method, 'POST')
  assert.equal(JSON.parse(requests[0].init.body as string).revision, 3)
  await save(schedules.inspectionScheduleInput(schedule), 'plan/1')
  assert.equal(requests[1].path, '/api/v1/inspection/schedules/plan%2F1')
  assert.equal(requests[1].init.method, 'PUT')
  body = { timezone: 'Asia/Shanghai', scheduler_running: false, schedules: [schedule], occurrences: [] }
  assert.equal((await read('2026-11-01', '2026-11-30')).scheduler_running, false)
  assert.match(requests.at(-1)!.path, /start=2026-11-01&end=2026-11-30/)
  body = { timezone: 'Asia/Shanghai', scheduler_running: true, schedules: [{ ...schedule, start_date: '2026-02-30' }], occurrences: [] }
  await assert.rejects(read('2026-11-01', '2026-11-30'), /数据格式无效/)
  body = { deleted: true }
  await remove('plan/1')
  assert.equal(requests.at(-1)!.init.method, 'DELETE')
  status = 503
  body = { error: 'scheduler_unavailable' }
  await assert.rejects(read('2026-11-01', '2026-11-30'), /巡检计划服务暂不可用/)
  status = 422
  body = { detail: { reason: 'schedule_has_no_future_occurrence' } }
  await assert.rejects(read('2026-11-01', '2026-11-30'), /请调整日期、时间或重复星期/)
})

function renderCalendar(snapshot: InspectionScheduleSnapshot | null, routes: unknown[] = [], callbacks: { open?: (id: string) => void } = {}) {
  const require = createRequire(import.meta.url)
  const react = require('react')
  const exports: Record<string, unknown> = {}
  let slot = 0
  const reactMock = { ...react,
    useState(initial: unknown) {
      const index = slot++
      const value = index === 0 ? '2026-11-10' : index === 6 ? snapshot : typeof initial === 'function' ? initial() : initial
      return [value, () => {}]
    }, useMemo: (factory: () => unknown) => factory(), useCallback: (callback: unknown) => callback,
    useRef: (value: unknown) => ({ current: value }), useEffect() {},
  }
  runInNewContext(compile('../src/components/InspectionCalendar.tsx'), {
    exports, require: (name: string) => {
      if (name === 'react') return reactMock
      if (name.endsWith('/services/inspectionSchedules')) return schedules
      if (name.endsWith('/services/observationMode')) return { isObservationMode: () => false }
      if (name.endsWith('/i18n')) return { text: (_locale: string, _en: string, zh: string) => zh }
      if (name.endsWith('.css')) return { default: {} }
      return require(name)
    }, Date, Intl, Error,
  })
  const component = exports.InspectionCalendar as (props: object) => unknown
  const tree = component({ routes, locale: 'zh', onOpenTask: callbacks.open ?? (() => {}) })
  return { tree, html: require('react-dom/server').renderToStaticMarkup(tree) as string }
}

test('calendar keeps offline and disabled plans visible and prevents saving without a route', () => {
  const { html } = renderCalendar({ timezone: 'Asia/Shanghai', scheduler_running: false, schedules: [schedule], occurrences: [] })
  assert.match(html, /定时服务未运行/)
  assert.match(html, /未启用/)
  assert.match(html, /先保存巡检路线/)
  assert.match(html, /type="submit"[^>]*disabled/)
  assert.doesNotMatch(html, /已就绪|运行成功/)
})

test('known scheduling refusal is shown as a direct operational reason', () => {
  const { html } = renderCalendar({ timezone: 'Asia/Shanghai', scheduler_running: true, schedules: [schedule], occurrences: [{
    id: 'rejected-1', schedule_id: schedule.id, scheduled_for: '2026-11-10T01:00:00Z', state: 'rejected',
    reason: 'inspection_schedule_task_active', task_id: '', request_id: 'request-rejected',
  }] })
  assert.match(html, /已有任务正在执行/)
  assert.doesNotMatch(html, /inspection_schedule_task_active/)
})

test('submitted occurrence opens its actual task without claiming completion', () => {
  const opened: string[] = []
  const { tree, html } = renderCalendar({ timezone: 'Asia/Shanghai', scheduler_running: true, schedules: [schedule], occurrences: [{
    id: 'occ-1', schedule_id: schedule.id, scheduled_for: '2026-11-10T01:00:00Z', state: 'submitted',
    reason: '', task_id: 'native-task-1', request_id: 'request-1',
  }] }, [], { open: id => opened.push(id) })
  assert.match(html, /已提交/)
  assert.match(html, /查看任务/)
  assert.doesNotMatch(html, /已完成/)
  type Element = { type?: string; props?: { children?: unknown; onClick?: () => void } }
  const buttons: Element[] = []
  const visit = (node: unknown) => {
    if (Array.isArray(node)) { node.forEach(visit); return }
    if (!node || typeof node !== 'object') return
    const element = node as Element
    if (element.type === 'button' && element.props?.children === '查看任务') buttons.push(element)
    const children = element.props?.children
    for (const child of Array.isArray(children) ? children : [children]) visit(child)
  }
  visit(tree)
  assert.equal(buttons.length, 1)
  buttons[0].props!.onClick!()
  assert.deepEqual(opened, ['native-task-1'])
})

test('saving with the default saved route keeps that binding available for the next edit', async () => {
  const require = createRequire(import.meta.url)
  const react = require('react')
  const exports: Record<string, unknown> = {}
  const slots: unknown[] = []
  let cursor = 0
  const service = { ...schedules,
    saveInspectionSchedule: async (input: schedules.InspectionScheduleInput) => ({ ...schedule, ...input }),
    fetchInspectionSchedules: async () => ({ timezone: 'Asia/Shanghai', scheduler_running: false, schedules: [schedule], occurrences: [] }),
  }
  runInNewContext(compile('../src/components/InspectionCalendar.tsx'), {
    exports, require: (name: string) => {
      if (name === 'react') return { ...react,
        useState(initial: unknown) {
          const slot = cursor++
          if (!(slot in slots)) slots[slot] = typeof initial === 'function' ? initial() : initial
          return [slots[slot], (next: unknown) => { slots[slot] = typeof next === 'function' ? next(slots[slot]) : next }]
        }, useMemo: (factory: () => unknown) => factory(), useCallback: (callback: unknown) => callback,
        useRef: (value: unknown) => ({ current: value }), useEffect() {},
      }
      if (name.endsWith('/services/inspectionSchedules')) return service
      if (name.endsWith('/services/observationMode')) return { isObservationMode: () => false }
      if (name.endsWith('/i18n')) return { text: (_locale: string, _en: string, zh: string) => zh }
      if (name.endsWith('.css')) return { default: {} }
      return require(name)
    }, Date, Intl, Error,
  })
  const render = exports.InspectionCalendar as (props: object) => unknown
  type Element = { type?: string; props?: { children?: unknown; type?: string; value?: unknown; disabled?: boolean; onSubmit?: (event: object) => void; 'aria-label'?: string } }
  const elements = (node: unknown, type: string): Element[] => {
    if (Array.isArray(node)) return node.flatMap(child => elements(child, type))
    if (!node || typeof node !== 'object') return []
    const element = node as Element
    return [...(element.type === type ? [element] : []), ...elements(element.props?.children, type)]
  }
  const props = { locale: 'zh', routes: [{ id: schedule.route_id, map_id: schedule.map_id, revision: 3, points: [] }], onOpenTask() {} }
  const first = render(props)
  elements(first, 'form')[0].props!.onSubmit!({ preventDefault() {} })
  for (let index = 0; index < 20; index++) await Promise.resolve()
  cursor = 0
  const second = render(props)
  const route = elements(second, 'select').find(element => element.props?.['aria-label'] === '计划使用的路线')
  assert.equal(route?.props?.value, `${schedule.map_id}/${schedule.route_id}`)
  assert.equal(elements(second, 'button').find(element => element.props?.type === 'submit')?.props?.disabled, false)
})
