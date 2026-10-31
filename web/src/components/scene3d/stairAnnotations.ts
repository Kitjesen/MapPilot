import * as THREE from 'three'
import type { Bounds, StairAnnotation, StairDraft } from '../../types/stairs.ts'
import { lingtuToThree } from '../../services/coordinateFrame.ts'
import { cloudPointIsVisible } from './layers/layerUtils.ts'

export interface ScreenRectangle {
  x0: number
  y0: number
  x1: number
  y1: number
}

/** A screen rectangle selects observed map points, not a floor or a traversable volume. */
export function selectSavedMapBounds(
  savedMapFlat: readonly number[],
  savedPoints: THREE.Points,
  camera: THREE.Camera,
  viewport: { width: number; height: number },
  rectangle: ScreenRectangle,
): Bounds | null {
  if (!savedPoints.visible || viewport.width <= 0 || viewport.height <= 0) return null
  const left = Math.min(rectangle.x0, rectangle.x1)
  const right = Math.max(rectangle.x0, rectangle.x1)
  const top = Math.min(rectangle.y0, rectangle.y1)
  const bottom = Math.max(rectangle.y0, rectangle.y1)
  const min = { x: Infinity, y: Infinity, z: Infinity }
  const max = { x: -Infinity, y: -Infinity, z: -Infinity }
  const world = new THREE.Vector3()
  const projected = new THREE.Vector3()
  camera.updateWorldMatrix(true, false)
  for (let index = 0; index + 2 < savedMapFlat.length; index += 3) {
    const x = savedMapFlat[index]
    const y = savedMapFlat[index + 1]
    const z = savedMapFlat[index + 2]
    if (![x, y, z].every(Number.isFinite)) continue
    world.set(...lingtuToThree([x, y, z]))
    if (!cloudPointIsVisible(savedPoints, world)) continue
    projected.copy(world).project(camera)
    if (projected.z < -1 || projected.z > 1) continue
    const screenX = (projected.x + 1) * viewport.width / 2
    const screenY = (1 - projected.y) * viewport.height / 2
    if (screenX < left || screenX > right || screenY < top || screenY > bottom) continue
    min.x = Math.min(min.x, x); max.x = Math.max(max.x, x)
    min.y = Math.min(min.y, y); max.y = Math.max(max.y, y)
    min.z = Math.min(min.z, z); max.z = Math.max(max.z, z)
  }
  if (!Number.isFinite(min.x)) return null
  // Give planar observations a visible box without inventing an entrance or ground height.
  for (const axis of ['x', 'y', 'z'] as const) {
    if (max[axis] - min[axis] < 0.05) {
      const center = (min[axis] + max[axis]) / 2
      min[axis] = center - 0.025
      max[axis] = center + 0.025
    }
  }
  return { min, max }
}

function addAnnotation(group: THREE.Group, annotation: StairDraft, draft: boolean): void {
  const color = draft ? 0x78b8df : 0x8ca5b6
  const opacity = draft ? 0.95 : 0.5
  if (annotation.bounds) {
    const { min, max } = annotation.bounds
    const size = new THREE.Vector3(max.x - min.x, max.z - min.z, max.y - min.y)
    const geometry = new THREE.BoxGeometry(size.x, size.y, size.z)
    const edges = new THREE.EdgesGeometry(geometry)
    geometry.dispose()
    const box = new THREE.LineSegments(edges, new THREE.LineBasicMaterial({
      color, transparent: true, opacity, depthTest: false,
    }))
    box.position.set(...lingtuToThree([(min.x + max.x) / 2, (min.y + max.y) / 2, (min.z + max.z) / 2]))
    box.name = 'stair-bounds'
    box.renderOrder = 42
    group.add(box)
  }
  for (const entrance of ['lower', 'upper'] as const) {
    const point = annotation[entrance]
    if (!point) continue
    const marker = new THREE.Mesh(new THREE.SphereGeometry(draft ? 0.09 : 0.065, 12, 8),
      new THREE.MeshBasicMaterial({ color: entrance === 'lower' ? color : 0xcfe8f7,
        transparent: true, opacity, depthTest: false }))
    marker.position.set(...lingtuToThree([point.x, point.y, point.z]))
    marker.name = `stair-${entrance}`
    marker.renderOrder = 44
    group.add(marker)
  }
  if (annotation.lower && annotation.upper) {
    const lower = new THREE.Vector3(...lingtuToThree([annotation.lower.x, annotation.lower.y, annotation.lower.z]))
    const upper = new THREE.Vector3(...lingtuToThree([annotation.upper.x, annotation.upper.y, annotation.upper.z]))
    const direction = upper.clone().sub(lower)
    const length = direction.length()
    if (length > 0) {
      const arrow = new THREE.ArrowHelper(direction.normalize(), lower, length, color,
        Math.min(0.2, length * 0.2), Math.min(0.1, length * 0.1))
      arrow.name = 'stair-direction'
      for (const child of [arrow.line, arrow.cone]) {
        const material = child.material as THREE.Material
        material.transparent = true
        material.opacity = opacity
        material.depthTest = false
        child.renderOrder = 43
      }
      group.add(arrow)
    }
  }
}

export function createStairAnnotations(annotations: readonly StairAnnotation[], draft?: StairDraft | null): THREE.Group {
  const group = new THREE.Group()
  group.name = 'stair-annotations'
  for (const annotation of annotations) {
    const entry = new THREE.Group()
    entry.name = `stair-${annotation.id}`
    addAnnotation(entry, annotation, false)
    group.add(entry)
  }
  if (draft) {
    const entry = new THREE.Group()
    entry.name = 'stair-draft'
    addAnnotation(entry, draft, true)
    group.add(entry)
  }
  return group
}
