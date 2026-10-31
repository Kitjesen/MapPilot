import type {
  InspectionFailurePolicy,
  InspectionRoute,
  InspectionRoutePoint,
  InspectionRouteRequest,
  InspectionRouteResponse,
  InspectionTaskCommandResponse,
  InspectionTaskStatusResponse,
} from '../types'

export interface RouteDraftPoint extends Partial<InspectionRoutePoint> {
  x: number
  y: number
  z: number
}

export interface RouteDraftIdentity {
  mapId: string
  contentEpoch: number
}

export interface RouteDraft {
  identity: RouteDraftIdentity | null
  points: RouteDraftPoint[]
  source?: InspectionRoute
}

export interface RouteStartIntent {
  signature: string
  routeId: string
  requestId: string
}

export type OpenRouteTaskSelection =
  | { kind: 'none'; task: null }
  | { kind: 'one'; task: InspectionTaskStatusResponse }
  | { kind: 'ambiguous'; task: null }

export const EMPTY_ROUTE_DRAFT: RouteDraft = { identity: null, points: [] }

export function sameRouteDraftIdentity(
  left: RouteDraftIdentity | null,
  right: RouteDraftIdentity | null,
): boolean {
  return left?.mapId === right?.mapId && left?.contentEpoch === right?.contentEpoch
}

export function resolveRouteDraftIdentity(
  mapId: string | null,
  mapListEpoch: number | null | undefined,
  loadedCloudEpoch: number | null | undefined,
): RouteDraftIdentity | null {
  if (!mapId) return null
  const contentEpoch = loadedCloudEpoch ?? mapListEpoch
  if (!Number.isSafeInteger(contentEpoch) || Number(contentEpoch) < 0) return null
  return { mapId, contentEpoch: Number(contentEpoch) }
}

export function routeDraftFromSaved(route: InspectionRoute): RouteDraft {
  return {
    identity: resolveRouteDraftIdentity(route.map_id ?? null, route.map_content_epoch, undefined),
    points: route.points.map(point => ({ ...point })),
    source: route,
  }
}

export function routeDraftSignature(draft: RouteDraft, tolerance: number, name = ''): string {
  return JSON.stringify({
    identity: draft.identity,
    points: draft.points,
    tolerance,
    name,
    sourceId: draft.source?.id,
    loopCount: draft.source?.loop_count,
    failurePolicy: draft.source?.failure_policy,
    maxRetries: draft.source?.max_retries,
  })
}

export function ensureRouteStartIntent(
  current: RouteStartIntent | null,
  signature: string,
  createRequestId: () => string,
  createRouteId: () => string,
): RouteStartIntent {
  return current?.signature === signature ? current : {
    signature,
    routeId: createRouteId(),
    requestId: createRequestId(),
  }
}

export function selectUniqueOpenRouteTask(
  tasks: InspectionTaskStatusResponse[],
): OpenRouteTaskSelection {
  const openTasks = tasks.filter(task => !task.terminal)
  if (openTasks.length === 0) return { kind: 'none', task: null }
  if (openTasks.length === 1) return { kind: 'one', task: openTasks[0] }
  return { kind: 'ambiguous', task: null }
}

export function appendRouteDraftPoint(
  draft: RouteDraft,
  identity: RouteDraftIdentity,
  point: RouteDraftPoint,
): RouteDraft {
  if (![point.x, point.y, point.z].every(Number.isFinite)) return draft
  if (!sameRouteDraftIdentity(draft.identity, identity)) return { identity, points: [point] }
  return { ...draft, identity, points: [...draft.points, point] }
}

export function removeRouteDraftPoint(draft: RouteDraft, index: number): RouteDraft {
  if (index < 0 || index >= draft.points.length) return draft
  return { ...draft, points: draft.points.filter((_, pointIndex) => pointIndex !== index) }
}

export function moveRouteDraftPoint(
  draft: RouteDraft,
  index: number,
  direction: -1 | 1,
): RouteDraft {
  const target = index + direction
  if (index < 0 || index >= draft.points.length || target < 0 || target >= draft.points.length) {
    return draft
  }
  const points = [...draft.points]
  const [point] = points.splice(index, 1)
  points.splice(target, 0, point)
  return { ...draft, points }
}

export function buildInspectionRouteRequest(
  draft: RouteDraft,
  options: { id: string; name: string; tolerance: number },
): InspectionRouteRequest {
  if (!draft.identity || draft.points.length < 2) {
    throw new Error('A route draft requires a map identity and at least two points')
  }
  const usedIds = new Set(draft.points.map(point => point.id))
  let nextPointNumber = 1
  return {
    id: draft.source?.id ?? options.id,
    name: options.name,
    map_id: draft.identity.mapId,
    map_content_epoch: draft.identity.contentEpoch,
    revision: (draft.source?.revision ?? 0) + 1,
    loop_count: draft.source?.loop_count ?? 1,
    failure_policy: (draft.source?.failure_policy ?? 'stop') as InspectionFailurePolicy,
    max_retries: draft.source?.max_retries ?? 0,
    points: draft.points.map(point => {
      let id = point.id
      if (id === undefined) {
        while (usedIds.has(`point-${nextPointNumber}`)) nextPointNumber += 1
        id = `point-${nextPointNumber++}`
      }
      return {
        id,
        x: point.x,
        y: point.y,
        z: point.z,
        yaw: point.yaw ?? null,
        tolerance: point.tolerance ?? options.tolerance,
        dwell: point.dwell ?? 0,
        action: point.action ?? '',
        enabled: point.enabled ?? true,
      }
    }),
  }
}

export async function saveAndStartRouteDraft(
  request: InspectionRouteRequest,
  requestId: string,
  operations: {
    save: (value: InspectionRouteRequest) => Promise<InspectionRouteResponse>
    start: (
      routeId: string,
      options: { map_id: string; revision: number; request_id: string },
    ) => Promise<InspectionTaskCommandResponse>
    onSaved?: (route: { id: string; revision: number }) => void
  },
): Promise<InspectionTaskCommandResponse> {
  const saved = await operations.save(request)
  const route = {
    id: saved.route.id,
    revision: Math.max(1, Number(saved.route.revision ?? 1)),
  }
  operations.onSaved?.(route)
  return operations.start(route.id, {
    map_id: request.map_id,
    revision: route.revision,
    request_id: requestId,
  })
}
