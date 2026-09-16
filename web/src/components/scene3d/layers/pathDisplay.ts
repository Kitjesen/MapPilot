import * as THREE from 'three'
import { lingtuToThree } from '../../../services/coordinateFrame.ts'

export const PATH_COLORS = { global: 0x6d8fae, local: 0x079dc5, trail: 0x98a5b2 } as const

/** Render the supplied polyline without fitting another spline through it. */
export function createPathDisplay(points: Array<{ x: number; y: number; z?: number | null }>,
  kind: 'global' | 'local'): THREE.Group | null {
  if (!points.every(p => [p.x, p.y, p.z ?? 0].every(Number.isFinite))) return null
  const vertices = points
    .map(p => new THREE.Vector3(...lingtuToThree([p.x, p.y, (p.z ?? 0) + (kind === 'local' ? 0.025 : 0.02)])))
  const segments = vertices.slice(1).map((end, index) => ({ start: vertices[index], end }))
    .filter(segment => segment.start.distanceToSquared(segment.end) > 1e-10)
  if (segments.length === 0) return null
  const group = new THREE.Group()
  const radius = kind === 'local' ? 0.021 : 0.012
  const material = new THREE.MeshBasicMaterial({ color: PATH_COLORS[kind], transparent: true,
    opacity: kind === 'local' ? 1 : 0.76 })
  const cylinders = new THREE.InstancedMesh(new THREE.CylinderGeometry(radius, radius, 1, 6), material, segments.length)
  const transform = new THREE.Object3D()
  const up = new THREE.Vector3(0, 1, 0)
  for (let index = 0; index < segments.length; index++) {
    const { start, end } = segments[index]
    transform.position.copy(start).add(end).multiplyScalar(0.5)
    transform.quaternion.setFromUnitVectors(up, end.clone().sub(start).normalize())
    transform.scale.set(1, start.distanceTo(end), 1)
    transform.updateMatrix()
    cylinders.setMatrixAt(index, transform.matrix)
  }
  cylinders.instanceMatrix.needsUpdate = true
  cylinders.computeBoundingSphere()
  group.add(cylinders)
  const joins = new THREE.InstancedMesh(new THREE.SphereGeometry(radius, 6, 4), material, vertices.length)
  const matrix = new THREE.Matrix4()
  vertices.forEach((point, index) => joins.setMatrixAt(index, matrix.makeTranslation(point.x, point.y, point.z)))
  joins.instanceMatrix.needsUpdate = true
  joins.computeBoundingSphere()
  group.add(joins)
  return group
}
