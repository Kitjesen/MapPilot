import * as THREE from 'three'
import { PLANNING_CELL_COLORS, type ReadyPlanningMap } from '../../../services/planningMap.ts'
import { createFlatGridGroup, type GroupedMesh } from './layerUtils.ts'

export function createPlanningMapLayer(map: ReadyPlanningMap): GroupedMesh | null {
  const pixels = new Uint8Array(map.cols * map.rows * 4)
  map.cells.forEach((cell, index) => pixels.set(PLANNING_CELL_COLORS[cell], index * 4))
  const texture = new THREE.DataTexture(pixels, map.cols, map.rows)
  texture.flipY = false
  texture.colorSpace = THREE.SRGBColorSpace
  texture.minFilter = THREE.NearestFilter
  texture.magFilter = THREE.NearestFilter
  texture.generateMipmaps = false
  texture.needsUpdate = true
  const width = map.cols * map.resolution
  const height = map.rows * map.resolution
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, height),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false, toneMapped: false, side: THREE.DoubleSide })) as GroupedMesh
  mesh.name = 'native-planning-eligibility'
  mesh.renderOrder = 2
  createFlatGridGroup(mesh, [map.origin[0], map.origin[1]], width, height, 0, map.origin[2] + .012)
  return mesh
}
