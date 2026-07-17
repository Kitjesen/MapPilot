import { useRef, useEffect, useCallback, useState } from 'react'
import type { SSEState, PathPoint, PlanPreviewResponse } from '../types'
import * as api from '../services/api'
import styles from './PathView.module.css'

interface PathViewProps {
  sseState: SSEState
  showToast: (message: string, kind?: 'success' | 'error' | 'info') => void
}

// Canvas drawing constants
const BG_COLOR      = '#0a0e17'
const GRID_COLOR    = 'rgba(255,255,255,0.03)'
const PATH_COLOR    = '#00ff88'
const ROBOT_COLOR   = '#00ff88'
const GOAL_COLOR    = '#ef4444'
const TRAIL_COLOR   = 'rgba(0,255,136,0.20)'
const ARROW_INTERVAL = 8  // draw direction arrow every N path segments
const TRAIL_MAX      = 200
const GOAL_SPEED_OPTIONS = [0.25, 0.4, 0.6]
const GOAL_RADIUS_OPTIONS = [0.25, 0.45, 0.8]

interface ViewTransform {
  scale: number       // pixels per metre
  originX: number     // canvas px for world (0,0)
  originY: number     // canvas px for world (0,0)
}

function worldToCanvas(wx: number, wy: number, t: ViewTransform): [number, number] {
  return [t.originX + wx * t.scale, t.originY - wy * t.scale]
}

function canvasToWorld(cx: number, cy: number, t: ViewTransform): [number, number] {
  return [(cx - t.originX) / t.scale, -(cy - t.originY) / t.scale]
}

function formatPlanSafetySummary(preview: PlanPreviewResponse | null | undefined): string {
  if (!preview) return ''
  const safety = preview.path_safety ?? {}
  const planner = preview.selected_planner ?? preview.planner
  const safetyOk = safety.ok === true ? 'ok' : safety.ok === false ? 'blocked' : ''
  return [
    planner ? `planner=${planner}` : null,
    preview.plan_safety_policy ? `policy=${preview.plan_safety_policy}` : null,
    safetyOk ? `safety=${safetyOk}` : null,
    preview.fallback_reason ? `fallback=${preview.fallback_reason}` : null,
    preview.rejected_plans?.length ? `rejected=${preview.rejected_plans.length}` : null,
  ].filter((v): v is string => Boolean(v)).join(' | ')
}

function uniqueStrings(values: string[]): string[] {
  const seen = new Set<string>()
  const out: string[] = []
  for (const raw of values) {
    const value = raw.trim()
    if (!value || seen.has(value)) continue
    seen.add(value)
    out.push(value)
  }
  return out
}

function blockerLabel(value: string): string {
  const labels: Record<string, string> = {
    navigation_session_inactive: 'not in a navigation product session',
    no_active_command_source: 'no active command source',
    odometry_missing: 'odometry missing',
    map_not_ready: 'map not ready',
    localizer_not_ready: 'localizer not ready',
  }
  return labels[value] ?? value
}

function formatPlanPreviewFailure(
  preview: PlanPreviewResponse | null | undefined,
  reasons: string[] = [],
  error?: string | null,
): string {
  const safety = formatPlanSafetySummary(preview)
  const reason = reasons.slice(0, 3).join(' / ') || error || preview?.error || 'plan preview rejected'
  return safety ? `${reason} (${safety})` : reason
}

function computeTransform(
  points: PathPoint[],
  robotX: number,
  robotY: number,
  width: number,
  height: number,
): ViewTransform {
  // Collect all world points to auto-fit
  const xs: number[] = [robotX]
  const ys: number[] = [robotY]
  for (const p of points) {
    xs.push(p.x)
    ys.push(p.y)
  }

  const minX = Math.min(...xs)
  const maxX = Math.max(...xs)
  const minY = Math.min(...ys)
  const maxY = Math.max(...ys)

  const spanX = maxX - minX || 10
  const spanY = maxY - minY || 10

  const padding = 0.15  // 15% padding on each side
  const scaleX = (width  * (1 - 2 * padding)) / spanX
  const scaleY = (height * (1 - 2 * padding)) / spanY
  const scale  = Math.min(scaleX, scaleY)

  const midWorldX = (minX + maxX) / 2
  const midWorldY = (minY + maxY) / 2
  const originX = width  / 2 - midWorldX * scale
  const originY = height / 2 + midWorldY * scale

  return { scale, originX, originY }
}

function drawGrid(ctx: CanvasRenderingContext2D, t: ViewTransform, w: number, h: number) {
  ctx.strokeStyle = GRID_COLOR
  ctx.lineWidth = 1

  // 1-metre grid spacing; ensure at least 4 lines visible
  const step = 1  // metres per grid line
  const [wMinX] = canvasToWorld(0, 0, t)
  const [wMaxX] = canvasToWorld(w, 0, t)
  const [, wMinY] = canvasToWorld(0, h, t)
  const [, wMaxY] = canvasToWorld(0, 0, t)

  ctx.beginPath()
  for (let x = Math.floor(wMinX); x <= Math.ceil(wMaxX); x += step) {
    const [cx] = worldToCanvas(x, 0, t)
    ctx.moveTo(cx, 0)
    ctx.lineTo(cx, h)
  }
  for (let y = Math.floor(wMinY); y <= Math.ceil(wMaxY); y += step) {
    const [, cy] = worldToCanvas(0, y, t)
    ctx.moveTo(0, cy)
    ctx.lineTo(w, cy)
  }
  ctx.stroke()
}

function drawScaleBar(ctx: CanvasRenderingContext2D, t: ViewTransform, h: number) {
  // Draw a 1-metre scale bar at bottom-left
  const barMetres = 1
  const barPx = barMetres * t.scale
  const x0 = 16, y0 = h - 16
  ctx.strokeStyle = 'rgba(255,255,255,0.4)'
  ctx.lineWidth = 1.5
  ctx.beginPath()
  ctx.moveTo(x0, y0)
  ctx.lineTo(x0 + barPx, y0)
  ctx.moveTo(x0, y0 - 4)
  ctx.lineTo(x0, y0 + 4)
  ctx.moveTo(x0 + barPx, y0 - 4)
  ctx.lineTo(x0 + barPx, y0 + 4)
  ctx.stroke()
  ctx.fillStyle = 'rgba(255,255,255,0.5)'
  ctx.font = '10px "JetBrains Mono", monospace'
  ctx.fillText('1 m', x0 + barPx + 6, y0 + 4)
}

function drawPath(
  ctx: CanvasRenderingContext2D,
  path: PathPoint[],
  t: ViewTransform,
) {
  if (path.length < 2) return

  // Path line
  ctx.strokeStyle = PATH_COLOR
  ctx.lineWidth = 2
  ctx.setLineDash([])
  ctx.beginPath()
  const [x0, y0] = worldToCanvas(path[0].x, path[0].y, t)
  ctx.moveTo(x0, y0)
  for (let i = 1; i < path.length; i++) {
    const [px, py] = worldToCanvas(path[i].x, path[i].y, t)
    ctx.lineTo(px, py)
  }
  ctx.stroke()

  // Waypoint circles
  ctx.fillStyle = PATH_COLOR
  for (const p of path) {
    const [cx, cy] = worldToCanvas(p.x, p.y, t)
    ctx.beginPath()
    ctx.arc(cx, cy, 4, 0, Math.PI * 2)
    ctx.fill()
  }

  // Direction arrows every ARROW_INTERVAL segments
  ctx.fillStyle = PATH_COLOR
  for (let i = ARROW_INTERVAL; i < path.length; i += ARROW_INTERVAL) {
    const prev = path[i - 1]
    const curr = path[i]
    const [ax, ay] = worldToCanvas(curr.x, curr.y, t)
    const angle = Math.atan2(-(curr.y - prev.y), curr.x - prev.x)
    ctx.save()
    ctx.translate(ax, ay)
    ctx.rotate(-angle)
    ctx.beginPath()
    ctx.moveTo(6, 0)
    ctx.lineTo(-4, -4)
    ctx.lineTo(-4, 4)
    ctx.closePath()
    ctx.fill()
    ctx.restore()
  }
}

function drawGoal(ctx: CanvasRenderingContext2D, path: PathPoint[], t: ViewTransform) {
  if (path.length === 0) return
  const last = path[path.length - 1]
  const [cx, cy] = worldToCanvas(last.x, last.y, t)
  ctx.strokeStyle = GOAL_COLOR
  ctx.lineWidth = 2
  ctx.fillStyle = GOAL_COLOR
  ctx.beginPath()
  ctx.arc(cx, cy, 6, 0, Math.PI * 2)
  ctx.fill()
  // Crosshair rings
  ctx.beginPath()
  ctx.arc(cx, cy, 12, 0, Math.PI * 2)
  ctx.stroke()
}

function drawPendingGoal(
  ctx: CanvasRenderingContext2D,
  goal: { x: number; y: number } | null,
  t: ViewTransform,
) {
  if (!goal) return
  const [cx, cy] = worldToCanvas(goal.x, goal.y, t)
  ctx.save()
  ctx.strokeStyle = GOAL_COLOR
  ctx.fillStyle = 'rgba(239, 68, 68, 0.18)'
  ctx.lineWidth = 2
  ctx.setLineDash([5, 4])
  ctx.beginPath()
  ctx.arc(cx, cy, 14, 0, Math.PI * 2)
  ctx.stroke()
  ctx.setLineDash([])
  ctx.beginPath()
  ctx.arc(cx, cy, 4, 0, Math.PI * 2)
  ctx.fill()
  ctx.restore()
}

function drawRobot(
  ctx: CanvasRenderingContext2D,
  rx: number,
  ry: number,
  yaw: number,
  t: ViewTransform,
) {
  const [cx, cy] = worldToCanvas(rx, ry, t)
  // Body circle
  ctx.fillStyle = ROBOT_COLOR
  ctx.shadowColor = 'rgba(0,255,136,0.4)'
  ctx.shadowBlur = 8
  ctx.beginPath()
  ctx.arc(cx, cy, 6, 0, Math.PI * 2)
  ctx.fill()
  ctx.shadowBlur = 0

  // Heading triangle
  ctx.save()
  ctx.translate(cx, cy)
  ctx.rotate(-yaw)
  ctx.fillStyle = ROBOT_COLOR
  ctx.beginPath()
  ctx.moveTo(12, 0)
  ctx.lineTo(6, -5)
  ctx.lineTo(6, 5)
  ctx.closePath()
  ctx.fill()
  ctx.restore()
}

function drawTrail(
  ctx: CanvasRenderingContext2D,
  trail: Array<[number, number]>,
  t: ViewTransform,
) {
  ctx.fillStyle = TRAIL_COLOR
  for (const [wx, wy] of trail) {
    const [cx, cy] = worldToCanvas(wx, wy, t)
    ctx.beginPath()
    ctx.arc(cx, cy, 2, 0, Math.PI * 2)
    ctx.fill()
  }
}

export function PathView({ sseState, showToast }: PathViewProps) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  const wrapperRef = useRef<HTMLDivElement>(null)
  const trailRef = useRef<Array<[number, number]>>([])
  const transformRef = useRef<ViewTransform>({ scale: 40, originX: 0, originY: 0 })
  const [hoverCoords, setHoverCoords] = useState<[number, number] | null>(null)
  const [trailLength, setTrailLength] = useState(0)
  const [pendingGoal, setPendingGoal] = useState<{ x: number; y: number } | null>(null)
  const [pendingGoalPreview, setPendingGoalPreview] = useState<PlanPreviewResponse | null>(null)
  const [goalMaxSpeed, setGoalMaxSpeed] = useState(0.4)
  const [goalAcceptanceRadius, setGoalAcceptanceRadius] = useState(0.45)

  const rawPath = sseState.globalPath?.points ?? []
  const path = rawPath.filter(
    (p): p is PathPoint =>
      p != null && typeof p.x === 'number' && typeof p.y === 'number'
  )
  const odom   = sseState.odometry
  const robotX = typeof odom?.x === 'number' ? odom.x : 0
  const robotY = typeof odom?.y === 'number' ? odom.y : 0
  const yaw    = typeof odom?.yaw === 'number' ? odom.yaw : 0
  const pathLength = path.length
  const navigationStatus = sseState.navigationStatus
  const activeCmdSource = navigationStatus?.control?.active_cmd_source ?? 'none'
  const activeCmdBlocksGoal = activeCmdSource !== '' && activeCmdSource !== 'none'
  const odomValid = odom != null && Number.isFinite(robotX) && Number.isFinite(robotY)
  const canAcceptGoal =
    navigationStatus?.readiness?.can_accept_goal ??
    navigationStatus?.can_accept_goal ??
    false
  const goalBlockers = uniqueStrings([
    ...(navigationStatus?.readiness?.blockers ?? []),
    ...(navigationStatus?.feedback?.blockers ?? []),
    activeCmdBlocksGoal
      ? `active command source: ${navigationStatus?.control?.active_source?.label ?? activeCmdSource}`
      : null,
    !odomValid ? 'no valid odometry' : null,
  ].filter((v): v is string => Boolean(v)))
  const goalDisabledReason =
    canAcceptGoal && !activeCmdBlocksGoal
      ? ''
      : goalBlockers.map(blockerLabel).slice(0, 3).join(' / ') || 'navigation is not ready'
  const canSendGoal = goalDisabledReason === ''
  const pendingGoalPlanSummary = formatPlanSafetySummary(pendingGoalPreview)

  // Accumulate position trail
  useEffect(() => {
    if (odom == null) return
    trailRef.current.push([odom.x, odom.y])
    if (trailRef.current.length > TRAIL_MAX) {
      trailRef.current.shift()
    }
    setTrailLength(trailRef.current.length)
  }, [odom])

  // Main render loop
  const render = useCallback(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    const w = canvas.width
    const h = canvas.height
    if (w === 0 || h === 0) return

    // Compute auto-fit transform
    const t = computeTransform(path, robotX, robotY, w, h)
    transformRef.current = t

    // Clear
    ctx.clearRect(0, 0, w, h)
    ctx.fillStyle = BG_COLOR
    ctx.fillRect(0, 0, w, h)

    drawGrid(ctx, t, w, h)
    drawTrail(ctx, trailRef.current, t)
    drawPath(ctx, path, t)
    if (path.length > 0) drawGoal(ctx, path, t)
    drawPendingGoal(ctx, pendingGoal, t)
    drawRobot(ctx, robotX, robotY, yaw, t)
    drawScaleBar(ctx, t, h)
  }, [path, pendingGoal, robotX, robotY, yaw])

  // Resize canvas to match wrapper and re-render
  useEffect(() => {
    const wrapper = wrapperRef.current
    const canvas  = canvasRef.current
    if (!wrapper || !canvas) return

    const observer = new ResizeObserver(entries => {
      for (const entry of entries) {
        const { width, height } = entry.contentRect
        canvas.width  = Math.round(width)
        canvas.height = Math.round(height)
        render()
      }
    })
    observer.observe(wrapper)
    return () => observer.disconnect()
  }, [render])

  // Re-render on state changes
  useEffect(() => {
    render()
  }, [render])

  // Click on canvas -> construct and preview a goal before sending it.
  const handleCanvasClick = useCallback(async (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (!canSendGoal) {
      showToast(`Cannot send goal: ${goalDisabledReason}`, 'error')
      return
    }
    const canvas = canvasRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const cx = (e.clientX - rect.left) * (canvas.width  / rect.width)
    const cy = (e.clientY - rect.top)  * (canvas.height / rect.height)
    const [wx, wy] = canvasToWorld(cx, cy, transformRef.current)
    try {
      const candidate = await api.constructGoalCandidate({
        x: wx,
        y: wy,
        source: 'map_click',
        target_type: 'map_point',
        label: 'path_view_click',
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed,
      })
      if (!candidate.ok || candidate.preview?.feasible === false) {
        const reason = formatPlanPreviewFailure(
          candidate.preview,
          candidate.reasons,
          candidate.error,
        )
        showToast(`目标不可达: ${reason}`, 'error')
        return
      }
      const target = candidate.target
      const goalX = target?.x ?? wx
      const goalY = target?.y ?? wy
      setPendingGoal({ x: goalX, y: goalY })
      setPendingGoalPreview(candidate.preview ?? null)
    } catch (e: unknown) {
      showToast(api.formatCommandError(e, 'Goal preview failed'), 'error')
    }
  }, [canSendGoal, goalAcceptanceRadius, goalDisabledReason, goalMaxSpeed, showToast])

  const handleConfirmGoal = useCallback(async () => {
    if (!pendingGoal) return
    if (!canSendGoal) {
      showToast(`Cannot send goal: ${goalDisabledReason}`, 'error')
      return
    }
    try {
      const res = await api.navigateClick(pendingGoal.x, pendingGoal.y, {
        source: 'map_click',
        target_type: 'map_point',
        label: 'path_view_click',
        acceptance_radius_m: goalAcceptanceRadius,
        max_speed_mps: goalMaxSpeed,
      })
      setPendingGoal(null)
      setPendingGoalPreview(null)
      showToast(
        api.formatCommandAck(res, `Goal (${pendingGoal.x.toFixed(2)}, ${pendingGoal.y.toFixed(2)})`),
        'success',
      )
    } catch (e: unknown) {
      showToast(api.formatCommandError(e, 'Send goal failed'), 'error')
    }
  }, [canSendGoal, goalAcceptanceRadius, goalDisabledReason, goalMaxSpeed, pendingGoal, showToast])

  const handleMouseMove = useCallback((e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current
    if (!canvas) return
    const rect = canvas.getBoundingClientRect()
    const cx = (e.clientX - rect.left) * (canvas.width  / rect.width)
    const cy = (e.clientY - rect.top)  * (canvas.height / rect.height)
    setHoverCoords(canvasToWorld(cx, cy, transformRef.current))
  }, [])

  const handleMouseLeave = useCallback(() => setHoverCoords(null), [])

  const handleClearTrail = useCallback(() => {
    trailRef.current = []
    setTrailLength(0)
    render()
  }, [render])

  return (
    <div className={styles.container}>
      <div className={styles.toolbar}>
        <span className={styles.toolbarTitle}>轨迹视图</span>
        <span className={pathLength > 0 ? styles.badge : `${styles.badge} ${styles.badgeEmpty}`}>
          {pathLength > 0 ? `路径 ${pathLength} 点` : '无路径'}
        </span>
        <label className={styles.goalControl}>
          <span>Speed</span>
          <select
            className={styles.goalSelect}
            value={goalMaxSpeed}
            onChange={(e) => setGoalMaxSpeed(Number(e.target.value))}
          >
            {GOAL_SPEED_OPTIONS.map((speed) => (
              <option key={speed} value={speed}>{speed.toFixed(2)} m/s</option>
            ))}
          </select>
        </label>
        <label className={styles.goalControl}>
          <span>Radius</span>
          <select
            className={styles.goalSelect}
            value={goalAcceptanceRadius}
            onChange={(e) => setGoalAcceptanceRadius(Number(e.target.value))}
          >
            {GOAL_RADIUS_OPTIONS.map((radius) => (
              <option key={radius} value={radius}>{radius.toFixed(2)} m</option>
            ))}
          </select>
        </label>
        <button className={styles.btnGhost} onClick={handleClearTrail}>清除轨迹</button>
      </div>

      <div className={styles.canvasWrapper} ref={wrapperRef}>
        <canvas
          ref={canvasRef}
          className={styles.canvas}
          onClick={handleCanvasClick}
          onMouseMove={handleMouseMove}
          onMouseLeave={handleMouseLeave}
        />
        {hoverCoords && (
          <div className={styles.coords}>
            {hoverCoords[0].toFixed(2)}, {hoverCoords[1].toFixed(2)} m
          </div>
        )}
        {pendingGoal && (
          <div className={styles.goalPanel}>
            <span className={styles.goalTitle}>目标预览</span>
            <span className={styles.goalCoords}>
              {pendingGoal.x.toFixed(2)}, {pendingGoal.y.toFixed(2)} m
            </span>
            {pendingGoalPlanSummary && (
              <span className={styles.goalPlanSummary} title={pendingGoalPlanSummary}>
                {pendingGoalPlanSummary}
              </span>
            )}
            {goalDisabledReason && (
              <span className={styles.goalReason} title={goalDisabledReason}>{goalDisabledReason}</span>
            )}
            <button
              className={styles.goalConfirm}
              onClick={handleConfirmGoal}
              disabled={!canSendGoal}
            >
              发送
            </button>
            <button
              className={styles.goalCancel}
              onClick={() => {
                setPendingGoal(null)
                setPendingGoalPreview(null)
              }}
            >
              取消
            </button>
          </div>
        )}
        <div className={styles.hint}>点击地图预览导航目标</div>
      </div>

      <div className={styles.statsRow}>
        <div className={styles.stat}>
          <span className={styles.statLabel}>机器人位置</span>
          <span className={styles.statValue}>
            {robotX.toFixed(2)}, {robotY.toFixed(2)} m
          </span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>航向</span>
          <span className={styles.statValue}>
            {((yaw * 180) / Math.PI).toFixed(1)}°
          </span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>路径点数</span>
          <span className={styles.statValue}>{pathLength}</span>
        </div>
        <div className={styles.stat}>
          <span className={styles.statLabel}>历史轨迹</span>
          <span className={styles.statValue}>{trailLength}</span>
        </div>
      </div>
    </div>
  )
}
