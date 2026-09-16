import * as THREE from 'three'

import {
  MAPPING_OBSERVATION_COLORS,
  type MappingObservationCell,
  type MappingObservationState,
} from '../../../services/mappingObservation.ts'
import { createFlatGridGroup, type GroupedMesh } from './layerUtils.ts'

export function createMappingObservationLayer(
  state: MappingObservationState,
  displayZ?: number,
): GroupedMesh | null {
  if (state.status !== 'ready') return null
  const { layer, cells } = state
  const width = layer.cols
  const height = layer.rows
  const pixels = new Uint8Array(width * height * 4)
  for (let row = 0; row < layer.rows; row++) {
    for (let col = 0; col < layer.cols; col++) {
      const cell = cells[row * layer.cols + col] as MappingObservationCell
      const color = MAPPING_OBSERVATION_COLORS[cell]
      pixels.set(color, (row * width + col) * 4)
    }
  }
  const texture = new THREE.DataTexture(pixels, width, height)
  texture.flipY = false
  texture.colorSpace = THREE.SRGBColorSpace
  texture.minFilter = THREE.NearestFilter
  texture.magFilter = THREE.NearestFilter
  texture.generateMipmaps = false
  texture.needsUpdate = true
  const sizeX = layer.cols * layer.resolution
  const sizeY = layer.rows * layer.resolution
  const mesh = new THREE.Mesh(
    new THREE.PlaneGeometry(sizeX, sizeY),
    new THREE.MeshBasicMaterial({
      map: texture,
      transparent: true,
      depthWrite: false,
      toneMapped: false,
      side: THREE.DoubleSide,
    }),
  ) as GroupedMesh
  mesh.name = 'mapping-observation'
  mesh.renderOrder = 3
  createFlatGridGroup(
    mesh, [layer.origin[0], layer.origin[1]], sizeX, sizeY, layer.yaw,
    displayZ ?? layer.origin[2] + 0.016,
  )
  return mesh
}
