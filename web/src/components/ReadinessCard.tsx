import { useEffect, useState } from 'react'
import type { ReadinessResponse, RoutecheckLatestResponse, RoutecheckSummary, SSEState } from '../types'
import * as api from '../services/api'
import styles from './ReadinessCard.module.css'

interface ReadinessCardProps {
  sseState: SSEState
}

function fmtNum(v: unknown, digits = 2): string {
  return typeof v === 'number' && Number.isFinite(v) ? v.toFixed(digits) : '--'
}

function labelList(values: string[], fallback: string): string[] {
  const seen = new Set<string>()
  const clean = values
    .map(v => String(v).trim())
    .filter((value) => {
      if (!value || seen.has(value)) return false
      seen.add(value)
      return true
    })
  return clean.length > 0 ? clean.slice(0, 5) : [fallback]
}

function stringList(value: unknown): string[] {
  return Array.isArray(value)
    ? value.map(item => String(item).trim()).filter(Boolean)
    : []
}

function calibrationWarnings(readiness: ReadinessResponse | null): string[] {
  const calibration = readiness?.runtime?.calibration
  if (calibration == null || typeof calibration !== 'object') return []
  return stringList((calibration as Record<string, unknown>).warnings)
}

function asRecord(value: unknown): Record<string, unknown> {
  return value != null && typeof value === 'object' && !Array.isArray(value)
    ? value as Record<string, unknown>
    : {}
}

function stringValue(value: unknown): string | null {
  return typeof value === 'string' && value.trim() ? value.trim() : null
}

function plannerSafetyLines(nav: SSEState['navigationStatus']): string[] {
  const diagnostics = nav?.diagnostics
  const report = asRecord(diagnostics?.last_plan_report)
  const safety = asRecord(report.selected_path_safety)
  const rejectedCount = Array.isArray(report.rejected_plans)
    ? report.rejected_plans.length
    : 0
  const planner = stringValue(report.selected_planner)
  const policy = diagnostics?.plan_safety_policy ?? stringValue(report.policy)
  const safetyState = safety.ok === true
    ? 'ok'
    : safety.ok === false
      ? 'blocked'
      : null
  const blockedSamples = typeof safety.blocked_sample_count === 'number'
    ? safety.blocked_sample_count
    : null
  const fallback = stringValue(report.fallback_reason)

  const lines = [
    planner ? `Planner: ${planner}` : null,
    policy ? `Policy: ${policy}` : null,
    safetyState ? `Path safety: ${safetyState}` : null,
    blockedSamples != null ? `Blocked samples: ${blockedSamples}` : null,
    fallback ? `Fallback: ${fallback}` : null,
    rejectedCount > 0 ? `Rejected plans: ${rejectedCount}` : null,
  ].filter((v): v is string => Boolean(v))
  return lines.length > 0 ? lines : ['No planner safety report yet']
}

function routecheckLines(routecheck: RoutecheckLatestResponse | null): string[] {
  if (!routecheck) return ['Routecheck not loaded yet']
  if (!routecheck.ok || !routecheck.latest) {
    return [routecheck.reason ?? 'No routecheck summary found']
  }

  const latest: RoutecheckSummary = routecheck.latest
  const phases = latest.phases ?? {}
  const phaseLines = Object.entries(phases).flatMap(([name, phase]) => {
    const planner = stringValue(phase.selected_planner) ?? stringValue(phase.planner)
    const outcome = stringValue(phase.outcome)
      ?? (phase.ok === true || phase.feasible === true
        ? 'pass'
        : phase.ok === false || phase.feasible === false
          ? 'blocked'
          : null)
    const fallback = stringValue(phase.fallback_reason)
    const error = stringValue(phase.error)
    return [
      `${name}: ${[outcome, planner].filter(Boolean).join(' / ') || 'recorded'}`,
      fallback ? `${name} fallback: ${fallback}` : null,
      error ? `${name} error: ${error}` : null,
    ].filter((line): line is string => Boolean(line))
  })

  const outcome = latest.outcome ? `Outcome: ${latest.outcome}` : null
  const count = `Runs found: ${routecheck.count}`
  const artifact = routecheck.artifact_dir ? `Artifact: ${routecheck.artifact_dir}` : null
  return [outcome, count, ...phaseLines, artifact].filter((line): line is string => Boolean(line)).slice(0, 6)
}

export function ReadinessCard({ sseState }: ReadinessCardProps) {
  const [clientReadiness, setClientReadiness] = useState<ReadinessResponse | null>(null)
  const [routecheck, setRoutecheck] = useState<RoutecheckLatestResponse | null>(null)
  const nav = sseState.navigationStatus
  const readiness = nav?.readiness
  const localization = nav?.localization
  const control = nav?.control
  const motion = nav?.motion
  const progress = nav?.progress
  const target = nav?.target
  const plannerLines = plannerSafetyLines(nav)

  const goalReady = readiness?.can_accept_goal ?? nav?.can_accept_goal ?? false
  const canExecute = readiness?.can_execute_autonomy ?? false
  const clientReasons = clientReadiness?.reasons ?? []
  const clientAdvisories = [
    ...(clientReadiness?.advisories ?? []),
    ...calibrationWarnings(clientReadiness),
  ]
  const blockers = labelList([...(readiness?.blockers ?? []), ...clientReasons], 'No readiness blockers')
  const advisories = labelList([...(readiness?.advisories ?? []), ...clientAdvisories], 'No advisories')
  const routecheckItems = routecheckLines(routecheck)
  const routecheckBlocked = routecheck?.ok === false || routecheck?.latest?.outcome === 'fail'
  const owner = control?.active_source?.label ?? control?.active_cmd_source ?? 'none'
  const speedMode = motion?.speed_policy?.mode ?? 'unknown'
  const distanceToGoal = fmtNum(target?.distance_to_goal_m)
  const progressText = progress
    ? `${progress.wp_index}/${progress.wp_total} (${Math.round(progress.fraction * 100)}%)`
    : '--'

  useEffect(() => {
    let cancelled = false
    const load = async () => {
      try {
        const next = await api.fetchReadiness()
        if (!cancelled) setClientReadiness(next)
      } catch {
        if (!cancelled) setClientReadiness(null)
      }
    }
    void load()
    const timer = window.setInterval(load, 5000)
    return () => {
      cancelled = true
      window.clearInterval(timer)
    }
  }, [])

  useEffect(() => {
    let cancelled = false
    const load = async () => {
      try {
        const next = await api.fetchRoutecheckLatest()
        if (!cancelled) setRoutecheck(next)
      } catch {
        if (!cancelled) setRoutecheck(null)
      }
    }
    void load()
    const timer = window.setInterval(load, 10000)
    return () => {
      cancelled = true
      window.clearInterval(timer)
    }
  }, [])

  return (
    <div className={styles.card}>
      <div className={styles.header}>
        <div>
          <div className={styles.eyebrow}>Navigation Readiness</div>
          <div className={styles.title}>{nav ? nav.state : 'NO DATA'}</div>
        </div>
        <span className={goalReady ? styles.readyPill : styles.blockedPill}>
          {nav ? (goalReady ? 'GOAL READY' : 'GOAL BLOCKED') : 'UNKNOWN'}
        </span>
      </div>

      <div className={styles.grid}>
        <Metric label="Autonomy" value={canExecute ? 'READY' : 'HELD'} tone={canExecute ? 'ok' : 'warn'} />
        <Metric label="Owner" value={owner} tone={owner === 'none' ? 'dim' : 'warn'} />
        <Metric label="Localization" value={localization?.state ?? 'unknown'} tone={localization?.ready ? 'ok' : 'warn'} />
        <Metric label="Pose" value={localization?.pose_freshness ?? 'unknown'} tone={localization?.pose_fresh ? 'ok' : 'warn'} />
        <Metric label="Progress" value={progressText} />
        <Metric label="Goal Dist." value={`${distanceToGoal} m`} />
        <Metric label="Speed Mode" value={speedMode} tone={speedMode === 'normal' ? 'dim' : 'warn'} />
        <Metric label="Speed Scale" value={fmtNum(motion?.speed_scale ?? nav?.speed_scale, 2)} />
      </div>

      <section className={styles.section}>
        <div className={styles.sectionTitle}>Blockers</div>
        <ul className={styles.list}>
          {blockers.map((item, index) => (
            <li key={`${item}-${index}`} className={goalReady ? styles.mutedItem : styles.alertItem}>{item}</li>
          ))}
        </ul>
      </section>

      <section className={styles.section}>
        <div className={styles.sectionTitle}>Advisories</div>
        <ul className={styles.list}>
          {advisories.map((item, index) => (
            <li key={`${item}-${index}`} className={styles.mutedItem}>{item}</li>
          ))}
        </ul>
      </section>

      <section className={styles.section}>
        <div className={styles.sectionTitle}>Planner Safety</div>
        <ul className={styles.list}>
          {plannerLines.map((item, index) => (
            <li
              key={`${item}-${index}`}
              className={
                item.includes('blocked') || item.includes('Fallback')
                  ? styles.alertItem
                  : styles.mutedItem
              }
            >
              {item}
            </li>
          ))}
        </ul>
      </section>

      <section className={styles.section}>
        <div className={styles.sectionTitle}>Routecheck</div>
        <ul className={styles.list}>
          {routecheckItems.map((item, index) => (
            <li
              key={`${item}-${index}`}
              className={
                routecheckBlocked || item.includes('blocked') || item.includes('fallback') || item.includes('error')
                  ? styles.alertItem
                  : styles.mutedItem
              }
              title={item}
            >
              {item}
            </li>
          ))}
        </ul>
      </section>
    </div>
  )
}

function Metric({
  label,
  value,
  tone = 'dim',
}: {
  label: string
  value: string
  tone?: 'ok' | 'warn' | 'dim'
}) {
  const valueClass =
    tone === 'ok' ? styles.metricOk :
    tone === 'warn' ? styles.metricWarn :
    styles.metricValue
  return (
    <div className={styles.metric}>
      <span className={styles.metricLabel}>{label}</span>
      <span className={valueClass}>{value}</span>
    </div>
  )
}
