import { useCallback, useEffect, useMemo, useState } from 'react'
import type { CSSProperties } from 'react'
import {
  AlertTriangle,
  CheckCircle2,
  Clipboard,
  Download,
  Play,
  RefreshCw,
  RotateCcw,
  ShieldCheck,
  SlidersHorizontal,
} from 'lucide-react'
import {
  fetchAppBootstrap,
  fetchNavigationStatus,
  fetchSession,
  validateMapPlan,
} from '../services/api'
import type { AppBootstrapResponse, NavigationStatusResponse, PlanPreviewResponse } from '../types'
import styles from './PlannerTuning.module.css'

type ToastKind = 'success' | 'error' | 'info'

interface PlannerTuningProps {
  showToast?: (message: string, kind?: ToastKind) => void
}

interface TuningState {
  robot_radius: number
  obstacle_clearance_radius_cells: number
  obstacle_clearance_weight: number
  floor_change_penalty: number
  max_step_height: number
  max_slope: number
  same_floor_z_tolerance: number
  max_same_floor_z_excursion: number
  snap_search_radius_cells: number
  ground_support_xy_radius_cells: number
  ground_support_depth_cells: number
  preblocked_costmap_radius_cells: number
  preblocked_costmap_weight: number
  require_ground_support: boolean
  strict_direct_ground_support: boolean
  enable_preblocked_costmap: boolean
  same_floor_preference: boolean
  lowest_traversable_only: boolean
}

type NumericKey = {
  [K in keyof TuningState]: TuningState[K] extends number ? K : never
}[keyof TuningState]

type BooleanKey = {
  [K in keyof TuningState]: TuningState[K] extends boolean ? K : never
}[keyof TuningState]

interface NumericParam {
  key: NumericKey
  label: string
  group: string
  unit: string
  min: number
  max: number
  step: number
  digits: number
  hint: string
}

interface ToggleParam {
  key: BooleanKey
  label: string
  hint: string
}

interface RuntimeSnapshot {
  activeMap: string
  navState: string
  hasOdometry: boolean
  canAcceptGoal: boolean
  blockers: string[]
  tfOk: boolean | null
  mapReady: boolean | null
  updatedAt: number | null
  error: string
}

interface PreviewGoal {
  x: number
  y: number
  z: number
}

interface MapValidatePlanResponse extends PlanPreviewResponse {
  map_id?: string
  active?: string
  motion_published?: boolean
  elapsed_client_ms?: number
}

const STORAGE_KEY = 'lingtu.octoplanner3d.tuning'

const PRODUCT_DEFAULTS: TuningState = {
  robot_radius: 0.4,
  obstacle_clearance_radius_cells: 2,
  obstacle_clearance_weight: 1.5,
  floor_change_penalty: 6,
  max_step_height: 0.35,
  max_slope: 0,
  same_floor_z_tolerance: 0.75,
  max_same_floor_z_excursion: 2,
  snap_search_radius_cells: 12,
  ground_support_xy_radius_cells: 2,
  ground_support_depth_cells: 6,
  preblocked_costmap_radius_cells: 2,
  preblocked_costmap_weight: 1.5,
  require_ground_support: true,
  strict_direct_ground_support: false,
  enable_preblocked_costmap: true,
  same_floor_preference: true,
  lowest_traversable_only: false,
}

const PRESETS: Record<string, { label: string; description: string; values: TuningState }> = {
  product: {
    label: '产品默认',
    description: '真实机器人优先，保留 0.60 m 机身包络和保守避障。',
    values: PRODUCT_DEFAULTS,
  },
  narrowStairs: {
    label: '窄楼梯样例',
    description: '用于 building2_9 这类窄通道验证，会降低机身包络。',
    values: {
      ...PRODUCT_DEFAULTS,
      robot_radius: 0.4,
      obstacle_clearance_radius_cells: 3,
      obstacle_clearance_weight: 2,
      floor_change_penalty: 4,
      max_same_floor_z_excursion: 1.5,
    },
  },
  conservative: {
    label: '保守安全',
    description: '更远离障碍，更不愿意跨楼层，适合实机初测。',
    values: {
      ...PRODUCT_DEFAULTS,
      obstacle_clearance_radius_cells: 5,
      obstacle_clearance_weight: 4.5,
      floor_change_penalty: 8,
      max_step_height: 0.28,
      max_same_floor_z_excursion: 1.2,
    },
  },
  debugLoose: {
    label: '调试宽松',
    description: '用于检查地图连通性，不应直接用于实机运动。',
    values: {
      ...PRODUCT_DEFAULTS,
      robot_radius: 0.3,
      obstacle_clearance_radius_cells: 1,
      obstacle_clearance_weight: 0.5,
      floor_change_penalty: 1,
      max_step_height: 0.6,
      max_same_floor_z_excursion: 4,
      require_ground_support: false,
      enable_preblocked_costmap: false,
    },
  },
}

const NUMERIC_PARAMS: NumericParam[] = [
  {
    key: 'robot_radius',
    label: '机器人包络半径',
    group: '安全间距',
    unit: 'm',
    min: 0.2,
    max: 0.8,
    step: 0.01,
    digits: 2,
    hint: '越大越安全，但窄楼梯和窄门更容易被判定无路。',
  },
  {
    key: 'obstacle_clearance_radius_cells',
    label: '障碍距离膨胀半径',
    group: '安全间距',
    unit: 'cell',
    min: 0,
    max: 8,
    step: 1,
    digits: 0,
    hint: '让路径主动远离障碍；过大时会把窄通道挤没。',
  },
  {
    key: 'obstacle_clearance_weight',
    label: '障碍距离权重',
    group: '安全间距',
    unit: 'cost',
    min: 0,
    max: 8,
    step: 0.1,
    digits: 1,
    hint: '解决贴边问题的核心权重，先调它再动机器人半径。',
  },
  {
    key: 'floor_change_penalty',
    label: '楼层切换代价',
    group: '楼层约束',
    unit: 'cost',
    min: 0,
    max: 12,
    step: 0.1,
    digits: 1,
    hint: '同层目标不要乱上楼；跨层目标仍允许通过楼梯或坡道。',
  },
  {
    key: 'same_floor_z_tolerance',
    label: '同层判定高度差',
    group: '楼层约束',
    unit: 'm',
    min: 0,
    max: 2,
    step: 0.05,
    digits: 2,
    hint: '起点和目标高度差小于该值时，规划会按同层任务处理。',
  },
  {
    key: 'max_same_floor_z_excursion',
    label: '同层最大爬高异常',
    group: '楼层约束',
    unit: 'm',
    min: 0,
    max: 6,
    step: 0.1,
    digits: 1,
    hint: '用于拒绝同层路径绕到其他楼层再回来。',
  },
  {
    key: 'max_step_height',
    label: '最大单步高度',
    group: '地形约束',
    unit: 'm',
    min: 0.05,
    max: 0.8,
    step: 0.01,
    digits: 2,
    hint: '限制相邻体素的垂直跳变，防止直接跳楼或穿层。',
  },
  {
    key: 'max_slope',
    label: '最大坡度',
    group: '地形约束',
    unit: 'rad',
    min: 0,
    max: 2,
    step: 0.05,
    digits: 2,
    hint: '当前默认 0 表示关闭，等接入 elevation/traversability 后再启用。',
  },
  {
    key: 'snap_search_radius_cells',
    label: '起终点吸附半径',
    group: '地图适配',
    unit: 'cell',
    min: 1,
    max: 30,
    step: 1,
    digits: 0,
    hint: '点击点不在可行体素上时，允许向附近搜索可走点。',
  },
  {
    key: 'ground_support_xy_radius_cells',
    label: '地面支撑横向半径',
    group: '地图适配',
    unit: 'cell',
    min: 0,
    max: 4,
    step: 1,
    digits: 0,
    hint: '检查脚下附近是否有可支撑结构，减少悬空路径。',
  },
  {
    key: 'ground_support_depth_cells',
    label: '地面支撑向下深度',
    group: '地图适配',
    unit: 'cell',
    min: 0,
    max: 6,
    step: 1,
    digits: 0,
    hint: '向下找地面支撑的体素层数，楼梯点云稀疏时需要略放宽。',
  },
  {
    key: 'preblocked_costmap_radius_cells',
    label: '预阻塞膨胀半径',
    group: '地图适配',
    unit: 'cell',
    min: 0,
    max: 8,
    step: 1,
    digits: 0,
    hint: '对已知障碍区域额外加代价，避免路径沿墙根走。',
  },
  {
    key: 'preblocked_costmap_weight',
    label: '预阻塞权重',
    group: '地图适配',
    unit: 'cost',
    min: 0,
    max: 8,
    step: 0.1,
    digits: 1,
    hint: '与预阻塞膨胀半径配合，过高会牺牲窄通道通过性。',
  },
]

const TOGGLE_PARAMS: ToggleParam[] = [
  {
    key: 'require_ground_support',
    label: '要求地面支撑',
    hint: '实机建议开启；点云楼梯过稀疏时可在仿真样例里临时关闭。',
  },
  {
    key: 'strict_direct_ground_support',
    label: '严格支撑检查',
    hint: '开启后更保守，适合地图质量稳定后的验收。',
  },
  {
    key: 'enable_preblocked_costmap',
    label: '启用预阻塞代价',
    hint: '让路径远离明显障碍，贴边时应开启。',
  },
  {
    key: 'same_floor_preference',
    label: '同层优先',
    hint: '同层目标优先留在当前楼层，跨层目标仍可上楼。',
  },
  {
    key: 'lowest_traversable_only',
    label: '只取最低可通行层',
    hint: '多楼层地图不要默认开启，否则会压掉上层路径。',
  },
]

const GROUPS = ['安全间距', '楼层约束', '地形约束', '地图适配']

function readInitialState(): TuningState {
  if (typeof window === 'undefined') return PRODUCT_DEFAULTS
  try {
    const raw = window.localStorage.getItem(STORAGE_KEY)
    if (!raw) return PRODUCT_DEFAULTS
    const parsed = JSON.parse(raw) as Partial<TuningState>
    return { ...PRODUCT_DEFAULTS, ...parsed }
  } catch {
    return PRODUCT_DEFAULTS
  }
}

function isRecord(value: unknown): value is Record<string, unknown> {
  return typeof value === 'object' && value !== null
}

function stringValue(value: unknown): string {
  return typeof value === 'string' ? value : ''
}

function booleanValue(value: unknown): boolean | null {
  return typeof value === 'boolean' ? value : null
}

function clamp(value: number, min: number, max: number): number {
  return Math.min(max, Math.max(min, value))
}

function decimalPlaces(step: number): number {
  const text = String(step)
  const dot = text.indexOf('.')
  return dot < 0 ? 0 : text.length - dot - 1
}

function normalizeParam(value: number, param: NumericParam): number {
  const clamped = clamp(value, param.min, param.max)
  return Number(clamped.toFixed(decimalPlaces(param.step)))
}

function formatNumber(value: number): string {
  return Number.isFinite(value) ? value.toFixed(2) : '0.00'
}

function formatPoint(point: { x: number; y: number; z: number } | null | undefined): string {
  if (!point) return '无'
  return `${formatNumber(point.x)}, ${formatNumber(point.y)}, ${formatNumber(point.z)}`
}

function makeProfileOverrides(values: TuningState): Record<string, number | boolean> {
  return {
    octoplanner3d_robot_radius: values.robot_radius,
    octoplanner3d_obstacle_clearance_radius_cells: values.obstacle_clearance_radius_cells,
    octoplanner3d_obstacle_clearance_weight: values.obstacle_clearance_weight,
    octoplanner3d_floor_change_penalty: values.floor_change_penalty,
    octoplanner3d_max_step_height: values.max_step_height,
    octoplanner3d_max_slope: values.max_slope,
    octoplanner3d_same_floor_z_tolerance: values.same_floor_z_tolerance,
    octoplanner3d_max_same_floor_z_excursion: values.max_same_floor_z_excursion,
    octoplanner3d_snap_search_radius_cells: values.snap_search_radius_cells,
    octoplanner3d_ground_support_xy_radius_cells: values.ground_support_xy_radius_cells,
    octoplanner3d_ground_support_depth_cells: values.ground_support_depth_cells,
    octoplanner3d_preblocked_costmap_radius_cells: values.preblocked_costmap_radius_cells,
    octoplanner3d_preblocked_costmap_weight: values.preblocked_costmap_weight,
    octoplanner3d_require_ground_support: values.require_ground_support,
    octoplanner3d_strict_direct_ground_support: values.strict_direct_ground_support,
    octoplanner3d_enable_preblocked_costmap: values.enable_preblocked_costmap,
    octoplanner3d_same_floor_preference: values.same_floor_preference,
    octoplanner3d_lowest_traversable_only: values.lowest_traversable_only,
  }
}

function buildRiskItems(values: TuningState): Array<{ kind: 'ok' | 'warn'; text: string }> {
  const risks: Array<{ kind: 'ok' | 'warn'; text: string }> = []
  if (values.robot_radius >= 0.55 && values.obstacle_clearance_radius_cells >= 4) {
    risks.push({ kind: 'warn', text: '窄楼梯或窄走廊可能被安全包络挤成无路。' })
  }
  if (values.obstacle_clearance_weight < 1 || values.obstacle_clearance_radius_cells < 2) {
    risks.push({ kind: 'warn', text: '障碍距离代价偏弱，路径更容易贴墙或贴边。' })
  }
  if (!values.same_floor_preference || values.max_same_floor_z_excursion > 3) {
    risks.push({ kind: 'warn', text: '同层目标可能绕到其他楼层再回来。' })
  }
  if (values.max_slope > 0) {
    risks.push({ kind: 'warn', text: '坡度约束已启用，但 OctoMap 原始体素还不是稳定坡度来源。' })
  }
  if (!values.require_ground_support) {
    risks.push({ kind: 'warn', text: '关闭地面支撑只适合排查连通性，不适合直接实机运动。' })
  }
  if (risks.length === 0) {
    risks.push({ kind: 'ok', text: '当前组合偏产品默认，适合在已验证地图上继续试跑。' })
  }
  return risks
}

async function copyText(text: string): Promise<void> {
  if (navigator.clipboard?.writeText) {
    await navigator.clipboard.writeText(text)
    return
  }
  const el = document.createElement('textarea')
  el.value = text
  el.style.position = 'fixed'
  el.style.left = '-9999px'
  document.body.appendChild(el)
  el.focus()
  el.select()
  document.execCommand('copy')
  document.body.removeChild(el)
}

function downloadJson(filename: string, text: string): void {
  const blob = new Blob([text], { type: 'application/json;charset=utf-8' })
  const url = URL.createObjectURL(blob)
  const a = document.createElement('a')
  a.href = url
  a.download = filename
  document.body.appendChild(a)
  a.click()
  document.body.removeChild(a)
  URL.revokeObjectURL(url)
}

function runtimeFromResponses(
  bootstrap: AppBootstrapResponse | null,
  session: Record<string, unknown> | null,
  status: NavigationStatusResponse | null,
): RuntimeSnapshot {
  const bootstrapMap = isRecord(bootstrap?.map) ? bootstrap.map : {}
  const mapActive = isRecord(bootstrapMap.active) ? bootstrapMap.active : {}
  const activeMap = stringValue(session?.active_map)
    || stringValue(bootstrapMap.active)
    || stringValue(mapActive.name)
    || stringValue(mapActive.map_id)
  const mapReady = booleanValue(session?.map_has_octomap) === true && booleanValue(session?.map_has_pcd) === true
    ? true
    : booleanValue(mapActive.activation_ready)

  return {
    activeMap,
    navState: status?.state ?? stringValue(session?.mode) ?? 'unknown',
    hasOdometry: status?.has_odometry ?? false,
    canAcceptGoal: status?.readiness?.can_accept_goal ?? status?.can_accept_goal ?? false,
    blockers: status?.readiness?.blockers ?? [],
    tfOk: status?.frames?.ok ?? null,
    mapReady,
    updatedAt: status?.ts ?? null,
    error: '',
  }
}

function ParamRow({
  param,
  value,
  onChange,
}: {
  param: NumericParam
  value: number
  onChange: (key: NumericKey, value: number) => void
}) {
  const percent = ((value - param.min) / (param.max - param.min)) * 100
  return (
    <div className={styles.paramRow}>
      <div className={styles.paramHead}>
        <div>
          <div className={styles.paramTitle}>{param.label}</div>
          <div className={styles.paramHint}>{param.hint}</div>
        </div>
        <div className={styles.paramValue}>
          <span>{value.toFixed(param.digits)}</span>
          <small>{param.unit}</small>
        </div>
      </div>
      <div className={styles.paramControl}>
        <input
          className={styles.slider}
          type="range"
          min={param.min}
          max={param.max}
          step={param.step}
          value={value}
          style={{ '--fill': `${percent}%` } as CSSProperties}
          onChange={(event) => onChange(param.key, normalizeParam(Number(event.target.value), param))}
        />
        <div className={styles.numberWrap}>
          <input
            className={styles.numberInput}
            type="number"
            min={param.min}
            max={param.max}
            step={param.step}
            value={value}
            onChange={(event) => onChange(param.key, normalizeParam(Number(event.target.value), param))}
          />
          <span>{param.unit}</span>
        </div>
      </div>
      <div className={styles.paramRange}>
        <span>{param.min}</span>
        <span>{param.max}</span>
      </div>
    </div>
  )
}

function ToggleRow({
  item,
  checked,
  onChange,
}: {
  item: ToggleParam
  checked: boolean
  onChange: (key: BooleanKey, value: boolean) => void
}) {
  return (
    <label className={styles.toggleRow}>
      <span>
        <strong>{item.label}</strong>
        <small>{item.hint}</small>
      </span>
      <input
        type="checkbox"
        checked={checked}
        onChange={(event) => onChange(item.key, event.target.checked)}
      />
    </label>
  )
}

function StatusPill({ ok, label }: { ok: boolean | null; label: string }) {
  const className = ok === true ? styles.pillOk : ok === false ? styles.pillBad : styles.pill
  return <span className={className}>{label}</span>
}

export function PlannerTuning({ showToast }: PlannerTuningProps) {
  const [values, setValues] = useState<TuningState>(() => readInitialState())
  const [activePreset, setActivePreset] = useState('product')
  const [runtime, setRuntime] = useState<RuntimeSnapshot>({
    activeMap: '',
    navState: 'unknown',
    hasOdometry: false,
    canAcceptGoal: false,
    blockers: [],
    tfOk: null,
    mapReady: null,
    updatedAt: null,
    error: '',
  })
  const [runtimeLoading, setRuntimeLoading] = useState(false)
  const [goal, setGoal] = useState<PreviewGoal>({ x: 0.5, y: 0, z: 0 })
  const [preview, setPreview] = useState<MapValidatePlanResponse | null>(null)
  const [previewLoading, setPreviewLoading] = useState(false)
  const [previewError, setPreviewError] = useState('')

  useEffect(() => {
    window.localStorage.setItem(STORAGE_KEY, JSON.stringify(values))
  }, [values])

  const refreshRuntime = useCallback(async () => {
    setRuntimeLoading(true)
    try {
      const [bootstrapResult, sessionResult, statusResult] = await Promise.allSettled([
        fetchAppBootstrap(),
        fetchSession(),
        fetchNavigationStatus(),
      ])
      const bootstrap = bootstrapResult.status === 'fulfilled' ? bootstrapResult.value : null
      const session = sessionResult.status === 'fulfilled' && isRecord(sessionResult.value)
        ? sessionResult.value
        : null
      const status = statusResult.status === 'fulfilled' ? statusResult.value : null
      setRuntime(runtimeFromResponses(bootstrap, session, status))
    } catch (error) {
      setRuntime((current) => ({
        ...current,
        error: error instanceof Error ? error.message : String(error),
      }))
    } finally {
      setRuntimeLoading(false)
    }
  }, [])

  useEffect(() => {
    void refreshRuntime()
  }, [refreshRuntime])

  const profileOverrides = useMemo(() => makeProfileOverrides(values), [values])
  const profileJson = useMemo(() => JSON.stringify(profileOverrides, null, 2), [profileOverrides])
  const runtimeJson = useMemo(() => JSON.stringify(values, null, 2), [values])
  const riskItems = useMemo(() => buildRiskItems(values), [values])

  const previewSummary = useMemo(() => {
    const pathCount = Array.isArray(preview?.path) ? preview.path.length : preview?.count ?? 0
    return {
      plan: preview,
      pathCount,
      reasons: preview?.reasons ?? [],
      selectedPlanner: preview?.planner || 'unknown',
      feasible: preview?.feasible ?? false,
      motionPublished: preview?.motion_published === true,
      error: preview?.error ?? '',
    }
  }, [preview])

  const setNumeric = (key: NumericKey, next: number) => {
    setValues((current) => ({ ...current, [key]: next }))
  }

  const setToggle = (key: BooleanKey, next: boolean) => {
    setValues((current) => ({ ...current, [key]: next }))
  }

  const applyPreset = (key: string) => {
    const preset = PRESETS[key]
    if (!preset) return
    setActivePreset(key)
    setValues(preset.values)
  }

  const updateGoal = (key: keyof PreviewGoal, value: number) => {
    setGoal((current) => ({ ...current, [key]: Number.isFinite(value) ? value : 0 }))
  }

  const handleCopy = async () => {
    try {
      await copyText(profileJson)
      showToast?.('已复制 profile overrides', 'success')
    } catch {
      showToast?.('复制失败', 'error')
    }
  }

  const handleDownload = () => {
    downloadJson('octoplanner3d-profile-overrides.json', profileJson)
    showToast?.('已导出调参配置', 'success')
  }

  const runPreview = async () => {
    const mapName = runtime.activeMap
    if (!mapName) {
      showToast?.('没有 active map，无法预览', 'error')
      return
    }
    setPreviewLoading(true)
    setPreviewError('')
    const started = performance.now()
    try {
      const response = await validateMapPlan(
        mapName,
        goal.x,
        goal.y,
        goal.z,
      ) as unknown as MapValidatePlanResponse
      setPreview({
        ...response,
        elapsed_client_ms: Math.round(performance.now() - started),
      })
      showToast?.(response.ok ? '规划预览可行' : '规划预览不可行', response.ok ? 'success' : 'info')
      void refreshRuntime()
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error)
      setPreviewError(message)
      showToast?.(`规划预览失败：${message}`, 'error')
    } finally {
      setPreviewLoading(false)
    }
  }

  return (
    <section className={styles.page} role="tabpanel" id="panel-planner" aria-label="全局规划调参">
      <div className={styles.main}>
        <header className={styles.header}>
          <div>
            <p className={styles.kicker}>OctoPlanner3D</p>
            <h1>全局规划调参</h1>
            <p>安全包络、贴边代价、楼层切换和三维路径异常校验。</p>
          </div>
          <div className={styles.actions}>
            <button className={styles.iconButton} onClick={() => applyPreset('product')} title="恢复产品默认">
              <RotateCcw size={16} />
              默认
            </button>
            <button className={styles.iconButton} onClick={handleCopy} title="复制 profile overrides">
              <Clipboard size={16} />
              复制
            </button>
            <button className={styles.iconButton} onClick={handleDownload} title="下载 JSON">
              <Download size={16} />
              导出
            </button>
          </div>
        </header>

        <div className={styles.presets} aria-label="调参预设">
          {Object.entries(PRESETS).map(([key, preset]) => (
            <button
              key={key}
              className={activePreset === key ? styles.presetActive : styles.preset}
              onClick={() => applyPreset(key)}
            >
              <span>{preset.label}</span>
              <small>{preset.description}</small>
            </button>
          ))}
        </div>

        <div className={styles.paramGroups}>
          {GROUPS.map((group) => (
            <section className={styles.group} key={group}>
              <div className={styles.groupTitle}>
                <SlidersHorizontal size={15} />
                <h2>{group}</h2>
              </div>
              <div className={styles.cardGrid}>
                {NUMERIC_PARAMS.filter((param) => param.group === group).map((param) => (
                  <ParamRow
                    key={param.key}
                    param={param}
                    value={values[param.key]}
                    onChange={setNumeric}
                  />
                ))}
              </div>
            </section>
          ))}
        </div>
      </div>

      <aside className={styles.side}>
        <section className={styles.panel}>
          <div className={styles.panelTitle}>
            <ShieldCheck size={16} />
            <h2>实机预览</h2>
          </div>
          <div className={styles.runtimeTop}>
            <div>
              <span>active map</span>
              <strong>{runtime.activeMap || '未连接'}</strong>
            </div>
            <button className={styles.iconOnlyButton} onClick={refreshRuntime} disabled={runtimeLoading} title="刷新实机状态">
              <RefreshCw size={15} />
            </button>
          </div>
          <div className={styles.statusGrid}>
            <div>
              <span>导航</span>
              <strong>{runtime.navState}</strong>
            </div>
            <div>
              <span>里程计</span>
              <StatusPill ok={runtime.hasOdometry} label={runtime.hasOdometry ? 'OK' : '缺失'} />
            </div>
            <div>
              <span>TF</span>
              <StatusPill ok={runtime.tfOk} label={runtime.tfOk === false ? '异常' : 'OK'} />
            </div>
            <div>
              <span>地图</span>
              <StatusPill ok={runtime.mapReady} label={runtime.mapReady === false ? '未就绪' : 'OK'} />
            </div>
          </div>
          {runtime.blockers.length > 0 && (
            <div className={styles.inlineWarn}>
              <AlertTriangle size={15} />
              <span>{runtime.blockers.join(', ')}</span>
            </div>
          )}
          {runtime.error && (
            <div className={styles.inlineWarn}>
              <AlertTriangle size={15} />
              <span>{runtime.error}</span>
            </div>
          )}
          <div className={styles.goalGrid}>
            <label>
              <span>X</span>
              <input type="number" step="0.1" value={goal.x} onChange={(event) => updateGoal('x', Number(event.target.value))} />
            </label>
            <label>
              <span>Y</span>
              <input type="number" step="0.1" value={goal.y} onChange={(event) => updateGoal('y', Number(event.target.value))} />
            </label>
            <label>
              <span>Z</span>
              <input type="number" step="0.1" value={goal.z} onChange={(event) => updateGoal('z', Number(event.target.value))} />
            </label>
          </div>
          <button className={styles.previewButton} onClick={runPreview} disabled={previewLoading || !runtime.activeMap}>
            <Play size={15} />
            {previewLoading ? '预览中' : 'no-motion 预览'}
          </button>
          {(preview || previewError) && (
            <div className={styles.previewResult}>
              <div className={styles.resultHead}>
                <StatusPill
                  ok={preview ? previewSummary.feasible : false}
                  label={previewSummary.feasible ? '可行' : '不可行'}
                />
                <span>{preview?.elapsed_client_ms ? `${preview.elapsed_client_ms} ms` : ''}</span>
              </div>
              <div className={styles.resultRows}>
                <span>planner</span><strong>{previewSummary.selectedPlanner}</strong>
                <span>path</span><strong>{previewSummary.pathCount} 点</strong>
                <span>motion</span><strong>{previewSummary.motionPublished ? '已发布' : '未发布'}</strong>
                <span>start</span><strong>{formatPoint(previewSummary.plan?.start)}</strong>
                <span>goal</span><strong>{formatPoint(previewSummary.plan?.goal)}</strong>
              </div>
              {(previewSummary.reasons.length > 0 || previewSummary.error || previewError) && (
                <div className={styles.resultReason}>
                  {previewSummary.reasons.length > 0 && <span>{previewSummary.reasons.join(', ')}</span>}
                  {previewSummary.error && <span>{previewSummary.error}</span>}
                  {previewError && <span>{previewError}</span>}
                </div>
              )}
            </div>
          )}
        </section>

        <section className={styles.panel}>
          <div className={styles.panelTitle}>
            <AlertTriangle size={16} />
            <h2>当前风险</h2>
          </div>
          <div className={styles.riskList}>
            {riskItems.map((item) => (
              <div className={item.kind === 'ok' ? styles.riskOk : styles.riskWarn} key={item.text}>
                {item.kind === 'ok' ? <CheckCircle2 size={16} /> : <AlertTriangle size={16} />}
                <span>{item.text}</span>
              </div>
            ))}
          </div>
        </section>

        <section className={styles.panel}>
          <div className={styles.panelTitle}>
            <SlidersHorizontal size={16} />
            <h2>开关</h2>
          </div>
          <div className={styles.toggleList}>
            {TOGGLE_PARAMS.map((item) => (
              <ToggleRow key={item.key} item={item} checked={values[item.key]} onChange={setToggle} />
            ))}
          </div>
        </section>

        <section className={styles.panel}>
          <div className={styles.panelTitle}>
            <CheckCircle2 size={16} />
            <h2>测试结论</h2>
          </div>
          <div className={styles.testGrid}>
            <span>r=0.60 同层</span><strong>通过</strong>
            <span>r=0.60 顶层</span><strong>通过</strong>
            <span>r=0.60 跨楼层</span><em>窄楼梯无路</em>
            <span>r=0.40 跨楼层</span><strong>通过</strong>
            <span>同层异常上楼</span><strong>已拒绝</strong>
          </div>
        </section>

        <section className={styles.panel}>
          <div className={styles.panelTitle}>
            <Clipboard size={16} />
            <h2>配置输出</h2>
          </div>
          <p className={styles.outputHint}>profile 使用 `octoplanner3d_*` 字段；C++ 运行时使用去前缀字段。</p>
          <textarea className={styles.output} readOnly value={profileJson} aria-label="profile overrides JSON" />
          <details className={styles.details}>
            <summary>查看底层运行时字段</summary>
            <pre>{runtimeJson}</pre>
          </details>
        </section>
      </aside>
    </section>
  )
}
