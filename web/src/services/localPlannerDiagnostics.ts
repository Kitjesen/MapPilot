export const LOCAL_PLANNER_DIAGNOSTICS_POLL_MS = 400

export function shouldPollLocalPlannerDiagnostics(
  debugNavigation: boolean,
  localPlannerLayerEnabled: boolean,
): boolean {
  return debugNavigation || localPlannerLayerEnabled
}
