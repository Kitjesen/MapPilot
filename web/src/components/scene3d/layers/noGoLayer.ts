import * as THREE from 'three'
import type { NoGoRegion } from '../../../services/noGo.ts'

export function createNoGoLayer(regions: readonly NoGoRegion[]): THREE.Group {
  const group = new THREE.Group()
  group.name = 'no-go-regions'
  for (const region of regions) {
    const color = region.kind === 'glass' ? 0x439ac4 : 0xf04452
    const size = region.max.map((value, axis) => value - region.min[axis])
    const center = region.min.map((value, axis) => (value + region.max[axis]) / 2)
    const mesh = new THREE.Mesh(new THREE.BoxGeometry(size[0], size[2], size[1]),
      new THREE.MeshBasicMaterial({ color, opacity: .18, transparent: true,
        depthWrite: false, side: THREE.DoubleSide }))
    mesh.name = region.label || region.id
    mesh.position.set(center[0], center[2], -center[1])
    const outline = new THREE.LineSegments(new THREE.EdgesGeometry(mesh.geometry),
      new THREE.LineBasicMaterial({ color, transparent: true, opacity: .85 }))
    mesh.add(outline)
    group.add(mesh)
  }
  return group
}
