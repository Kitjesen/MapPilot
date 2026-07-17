import * as THREE from 'three'

import type {
  NativeLocalPlannerCandidate,
  NavigationDdsSnapshotResponse,
} from '../../../types'

export const LOCAL_PLANNER_LAYER_COLORS = {
  obstacle: 0xff4057,
  traversability: 0xffa629,
  feasible: 0xd8dee9,
  blocked: 0xff4057,
  rejected: 0x6b7280,
  selected: 0x35f26d,
} as const

const LOCAL_PLANNER_ENDPOINT_MAX_AGE_S = 2.5

function endpointSnapshotFresh(snapshot: NavigationDdsSnapshotResponse): boolean {
  const endpointStamp = Number(snapshot.nav_endpoint?.stamp_s)
  const responseStamp = Number(snapshot.ts)
  return Number.isFinite(endpointStamp)
    && endpointStamp > 0
    && Number.isFinite(responseStamp)
    && Math.abs(responseStamp - endpointStamp) <= LOCAL_PLANNER_ENDPOINT_MAX_AGE_S
}

function finitePoint(value: unknown): [number, number, number] | null {
  if (!Array.isArray(value) || value.length < 2) return null
  const x = Number(value[0])
  const y = Number(value[1])
  const z = value.length > 2 ? Number(value[2]) : 0
  if (![x, y, z].every(Number.isFinite)) return null
  return [x, y, z]
}

function scenePoint(value: unknown, heightOffset = 0): THREE.Vector3 | null {
  const point = finitePoint(value)
  return point ? new THREE.Vector3(point[0], point[2] + heightOffset, -point[1]) : null
}

function candidateColor(candidate: NativeLocalPlannerCandidate): number {
  const state = candidate.dominant_state ?? candidate.state ?? ''
  if (state.includes('collision') || state.includes('blocked')) {
    return LOCAL_PLANNER_LAYER_COLORS.blocked
  }
  if (state.includes('terrain')) return LOCAL_PLANNER_LAYER_COLORS.traversability
  if (state.includes('reject') || state.includes('rotation')) {
    return LOCAL_PLANNER_LAYER_COLORS.rejected
  }
  return LOCAL_PLANNER_LAYER_COLORS.feasible
}

function createPathLine(
  candidate: NativeLocalPlannerCandidate,
  color: number,
): THREE.Line | null {
  const points = (candidate.path ?? [])
    .map(point => scenePoint(point, 0.08))
    .filter((point): point is THREE.Vector3 => point != null)
  if (points.length < 2) return null
  const geometry = new THREE.BufferGeometry().setFromPoints(points)
  const material = new THREE.LineBasicMaterial({
    color,
    transparent: true,
    opacity: color === LOCAL_PLANNER_LAYER_COLORS.feasible ? 0.58 : 0.82,
    depthTest: false,
  })
  const line = new THREE.Line(geometry, material)
  line.renderOrder = 40
  return line
}

function createSelectedPath(candidate: NativeLocalPlannerCandidate): THREE.Mesh | null {
  const points = (candidate.path ?? [])
    .map(point => scenePoint(point, 0.12))
    .filter((point): point is THREE.Vector3 => point != null)
  if (points.length < 2) return null
  const curve = new THREE.CatmullRomCurve3(points, false, 'catmullrom', 0.5)
  const geometry = new THREE.TubeGeometry(
    curve,
    Math.max(24, points.length * 4),
    0.035,
    5,
    false,
  )
  const material = new THREE.MeshBasicMaterial({
    color: LOCAL_PLANNER_LAYER_COLORS.selected,
    depthTest: false,
  })
  const mesh = new THREE.Mesh(geometry, material)
  mesh.name = 'selected-path'
  mesh.renderOrder = 44
  return mesh
}

function addObstaclePoints(
  group: THREE.Group,
  snapshot: NavigationDdsSnapshotResponse,
): void {
  const localMap = snapshot.nav_endpoint?.local_map
  if (
    !localMap?.enabled
    || localMap.frame_id !== 'map'
    || localMap.obstacle_points_fresh === false
  ) return

  const points = (localMap.obstacle_points ?? [])
    .map(point => scenePoint(point))
    .filter((point): point is THREE.Vector3 => point != null)
  if (points.length === 0) return

  const geometry = new THREE.BufferGeometry().setFromPoints(points)
  const material = new THREE.PointsMaterial({
    color: LOCAL_PLANNER_LAYER_COLORS.obstacle,
    size: 0.1,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.92,
    depthTest: false,
  })
  const cloud = new THREE.Points(geometry, material)
  cloud.name = 'planner-obstacles'
  cloud.renderOrder = 42
  group.add(cloud)
}

function addTraversabilityRisk(
  group: THREE.Group,
  snapshot: NavigationDdsSnapshotResponse,
): void {
  const localMap = snapshot.nav_endpoint?.local_map
  const traversability = localMap?.traversability
  if (
    !localMap?.enabled
    || localMap.frame_id !== 'map'
    || traversability?.fresh === false
  ) return

  const resolution = Number(traversability?.resolution_m)
  const origin = traversability?.origin_xy
  if (
    !Number.isFinite(resolution)
    || resolution <= 0
    || !Array.isArray(origin)
    || origin.length < 2
  ) return
  const originX = Number(origin[0])
  const originY = Number(origin[1])
  if (!Number.isFinite(originX) || !Number.isFinite(originY)) return

  const cells = (traversability?.risk_cells ?? []).filter(cell => (
    Array.isArray(cell)
    && cell.length >= 3
    && cell.slice(0, 3).every(value => Number.isFinite(Number(value)))
  ))
  if (cells.length === 0) return

  const geometry = new THREE.BoxGeometry(resolution * 0.88, 0.035, resolution * 0.88)
  const material = new THREE.MeshBasicMaterial({
    color: LOCAL_PLANNER_LAYER_COLORS.traversability,
    transparent: true,
    opacity: 0.52,
    depthTest: false,
  })
  const mesh = new THREE.InstancedMesh(geometry, material, cells.length)
  const transform = new THREE.Matrix4()
  cells.forEach((cell, index) => {
    const row = Number(cell[0])
    const col = Number(cell[1])
    const cost = Math.abs(Number(cell[2]))
    const scale = Math.max(0.35, Math.min(1, cost))
    transform.compose(
      new THREE.Vector3(
        originX + (col + 0.5) * resolution,
        0.04,
        -(originY + (row + 0.5) * resolution),
      ),
      new THREE.Quaternion(),
      new THREE.Vector3(scale, 1, scale),
    )
    mesh.setMatrixAt(index, transform)
  })
  mesh.instanceMatrix.needsUpdate = true
  mesh.name = 'traversability-risk'
  mesh.renderOrder = 38
  group.add(mesh)
}

function addCandidatePaths(
  group: THREE.Group,
  snapshot: NavigationDdsSnapshotResponse,
): void {
  const diagnostics = snapshot.nav_endpoint?.local_candidates
  if (!diagnostics?.valid || diagnostics.frame_id !== 'map') return

  const candidates = new THREE.Group()
  candidates.name = 'candidate-paths'
  let selected: THREE.Mesh | null = null
  for (const candidate of diagnostics.candidates ?? []) {
    if (candidate.selected) {
      selected = createSelectedPath(candidate)
      continue
    }
    const path = createPathLine(candidate, candidateColor(candidate))
    if (path) candidates.add(path)
  }
  if (candidates.children.length > 0) group.add(candidates)
  if (selected) group.add(selected)
}

export function createLocalPlannerDiagnosticLayer(
  snapshot: NavigationDdsSnapshotResponse | null | undefined,
): THREE.Group | null {
  if (!snapshot?.nav_endpoint || !endpointSnapshotFresh(snapshot)) return null
  const group = new THREE.Group()
  group.name = 'native-local-planner-diagnostics'
  addTraversabilityRisk(group, snapshot)
  addObstaclePoints(group, snapshot)
  addCandidatePaths(group, snapshot)
  return group.children.length > 0 ? group : null
}

export function disposeLocalPlannerDiagnosticLayer(
  scene: THREE.Scene,
  group: THREE.Group | null | undefined,
): void {
  if (!group) return
  scene.remove(group)
  group.traverse(object => {
    const renderable = object as THREE.Mesh | THREE.Line | THREE.Points
    renderable.geometry?.dispose()
    const material = renderable.material
    if (Array.isArray(material)) material.forEach(item => item.dispose())
    else material?.dispose()
  })
}
