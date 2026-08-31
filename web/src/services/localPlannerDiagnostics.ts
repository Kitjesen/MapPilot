export const LOCAL_PLANNER_DIAGNOSTICS_POLL_MS = 400

interface LocalPlannerDiagnosticSnapshot {
  nav_endpoint?: {
    local_map?: {
      traversability?: {
        complete?: boolean
        truncated?: boolean
        risk_cells_total?: number
        risk_cells_returned?: number
        unreported_cells?: string
      }
    }
  } | null
}

export function shouldPollLocalPlannerDiagnostics(
  debugNavigation: boolean,
  localPlannerLayerEnabled: boolean,
): boolean {
  return debugNavigation || localPlannerLayerEnabled
}

export function localPlannerSampleWarning(
  snapshot: LocalPlannerDiagnosticSnapshot | null | undefined,
): string | null {
  const diagnostic = snapshot?.nav_endpoint?.local_map?.traversability
  if (!diagnostic) return null
  const incomplete = diagnostic.complete === false
    || diagnostic.truncated === true
    || diagnostic.unreported_cells === 'not_serialized'
  if (!incomplete) return null
  const returned = diagnostic.risk_cells_returned
  const total = diagnostic.risk_cells_total
  const count = Number.isInteger(returned) && Number.isInteger(total) && (total ?? 0) >= 0
    ? `（${returned}/${total}）`
    : ''
  return `仅显示采样风险点，不代表完整风险栅格${count}`
}
