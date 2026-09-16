import * as THREE from 'three'
import { PLANNING_CELL_COLORS, type ReadyPlanningMap } from '../../../services/planningMap.ts'
import { createFlatGridGroup, type GroupedMesh } from './layerUtils.ts'

export function createPlanningMapLayer(map: ReadyPlanningMap): GroupedMesh | null {
  const canvas = document.createElement('canvas')
  canvas.width = map.cols
  canvas.height = map.rows
  const context = canvas.getContext('2d')
  if (!context) return null
  const image = context.createImageData(map.cols, map.rows)
  map.cells.forEach((cell, index) => image.data.set(PLANNING_CELL_COLORS[cell], index * 4))
  context.putImageData(image, 0, 0)
  const texture = new THREE.CanvasTexture(canvas)
  texture.flipY = false
  texture.colorSpace = THREE.SRGBColorSpace
  texture.minFilter = THREE.NearestFilter
  texture.magFilter = THREE.NearestFilter
  const width = map.cols * map.resolution
  const height = map.rows * map.resolution
  const mesh = new THREE.Mesh(new THREE.PlaneGeometry(width, height),
    new THREE.MeshBasicMaterial({ map: texture, transparent: true, depthWrite: false, toneMapped: false, side: THREE.DoubleSide })) as GroupedMesh
  mesh.name = 'native-planning-eligibility'
  mesh.renderOrder = 2
  createFlatGridGroup(mesh, [map.origin[0], map.origin[1]], width, height, 0, map.origin[2] + .012)
  return mesh
}
