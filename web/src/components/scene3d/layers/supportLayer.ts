import * as THREE from 'three'
import type { SurfaceMap } from '../../../services/surfaceMap.ts'

export function createSupportLayer(surface: SurfaceMap): THREE.Group {
  const group = new THREE.Group()
  group.name = 'support-surfaces'
  for (const [name, xyz, color] of [
    ['observed', surface.observed_xyz, 0x27bdc7], ['inferred', surface.inferred_xyz, 0xff9b32],
  ] as const) {
    const geometry = new THREE.PlaneGeometry(surface.resolution, surface.resolution)
    geometry.rotateX(-Math.PI / 2)
    const material = new THREE.MeshBasicMaterial({ color, side: THREE.DoubleSide, transparent: true,
      opacity: name === 'inferred' ? .95 : .6, depthWrite: false, polygonOffset: true, polygonOffsetFactor: -1 })
    const mesh = new THREE.InstancedMesh(geometry, material, xyz.length / 3)
    const matrix = new THREE.Matrix4()
    for (let i = 0; i < xyz.length; i += 3) {
      matrix.makeTranslation(xyz[i], xyz[i + 2], -xyz[i + 1])
      mesh.setMatrixAt(i / 3, matrix)
    }
    mesh.name = name
    group.add(mesh)
  }
  return group
}
