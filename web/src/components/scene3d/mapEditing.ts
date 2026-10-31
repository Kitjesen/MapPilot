import * as THREE from 'three'
import type { Bounds } from '../../types/stairs.ts'
import type { MapEditView } from '../../types/mapEdit.ts'
import { lingtuToThree, threeToLingtu } from '../../services/coordinateFrame.ts'
import type { ScreenRectangle } from './stairAnnotations.ts'

/** A top-view drag defines XY on one common plane, never a screen point mask. */
export function mapEditDragBounds(camera: THREE.Camera, viewport: { width: number; height: number }, rectangle: ScreenRectangle, source: Bounds): Bounds | null {
  if (viewport.width <= 0 || viewport.height <= 0 || Math.abs(rectangle.x1 - rectangle.x0) < 5 || Math.abs(rectangle.y1 - rectangle.y0) < 5) return null
  const plane = new THREE.Plane(new THREE.Vector3(0, 1, 0), -source.min.z)
  const ray = new THREE.Raycaster()
  const corners: number[][] = []
  camera.updateMatrixWorld()
  for (const [x, y] of [[rectangle.x0, rectangle.y0], [rectangle.x1, rectangle.y0], [rectangle.x1, rectangle.y1], [rectangle.x0, rectangle.y1]]) {
    ray.setFromCamera(new THREE.Vector2(x / viewport.width * 2 - 1, 1 - y / viewport.height * 2), camera)
    const hit = ray.ray.intersectPlane(plane, new THREE.Vector3())
    if (!hit) return null
    corners.push(threeToLingtu([hit.x, hit.y, hit.z]))
  }
  return {
    min: { x: Math.min(...corners.map(p => p[0])), y: Math.min(...corners.map(p => p[1])), z: source.min.z },
    max: { x: Math.max(...corners.map(p => p[0])), y: Math.max(...corners.map(p => p[1])), z: source.max.z },
  }
}

export function frameMapEditView(camera: THREE.PerspectiveCamera, target: THREE.Vector3, bounds: Bounds, view: MapEditView): void {
  const box = new THREE.Box3(new THREE.Vector3(...lingtuToThree([bounds.min.x, bounds.max.y, bounds.min.z])),
    new THREE.Vector3(...lingtuToThree([bounds.max.x, bounds.min.y, bounds.max.z])))
  const center = box.getCenter(new THREE.Vector3())
  const size = box.getSize(new THREE.Vector3())
  const direction = view === 'top' ? new THREE.Vector3(0, 1, 0.00001)
    : view === 'front' ? new THREE.Vector3(1, 0.00001, 0)
      : view === 'side' ? new THREE.Vector3(0, 0.00001, 1) : new THREE.Vector3(-1, 0.9, 1)
  const span = view === 'top' ? Math.max(size.z, size.x / camera.aspect)
    : view === 'front' ? Math.max(size.y, size.z / camera.aspect)
      : view === 'side' ? Math.max(size.y, size.x / camera.aspect)
        : size.length() / Math.min(1, camera.aspect)
  const distance = Math.max(1, span) / (2 * Math.tan(THREE.MathUtils.degToRad(camera.fov / 2))) * 1.25 + size.length() / 2
  camera.up.set(0, 1, 0)
  camera.position.copy(center).add(direction.normalize().multiplyScalar(distance))
  target.copy(center)
  camera.lookAt(center)
  camera.updateMatrixWorld()
}

/** Enter selection without losing the user's chosen map location or zoom. */
export function alignMapEditTopView(camera: THREE.PerspectiveCamera, target: THREE.Vector3, distance = camera.position.distanceTo(target)): void {
  camera.up.set(0, 1, 0)
  camera.position.copy(target).add(new THREE.Vector3(0, 1, 0.00001).normalize().multiplyScalar(distance))
  camera.lookAt(target)
  camera.updateMatrixWorld()
}

export function createMapEditBox(bounds: Bounds): THREE.Box3Helper {
  const box = new THREE.Box3(new THREE.Vector3(...lingtuToThree([bounds.min.x, bounds.max.y, bounds.min.z])),
    new THREE.Vector3(...lingtuToThree([bounds.max.x, bounds.min.y, bounds.max.z])))
  const helper = new THREE.Box3Helper(box, 0xe6b96d)
  helper.name = 'map-edit-selection'
  const material = helper.material as THREE.LineBasicMaterial
  material.depthTest = false
  material.transparent = true
  material.opacity = 0.9
  helper.renderOrder = 55
  return helper
}

export function mapEditSelectionAnchor(camera: THREE.Camera, viewport: { width: number; height: number }, bounds: Bounds): { x: number; y: number } | null {
  const point = new THREE.Vector3(...lingtuToThree([
    (bounds.min.x + bounds.max.x) / 2, (bounds.min.y + bounds.max.y) / 2, bounds.min.z,
  ])).project(camera)
  if (point.z < -1 || point.z > 1 || ![point.x, point.y, point.z].every(Number.isFinite)) return null
  return { x: Math.round((point.x + 1) * viewport.width / 2), y: Math.round((1 - point.y) * viewport.height / 2) }
}
