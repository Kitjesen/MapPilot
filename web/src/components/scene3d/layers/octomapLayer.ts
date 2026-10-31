import * as THREE from 'three'
import type { Octomap } from '../../../services/octomap.ts'
import { writeCloudHeightColor } from '../../../services/cloudHeightColor.ts'

export const OCTOMAP_VIEW_RADIUS_M = 10

export function createOctomapLayer(map: Octomap, centerX: number, centerY: number, colorMode: 'height' | 'neutral', scale = 1): THREE.InstancedMesh | null {
  const { cells, bounds } = map
  const selected: number[] = []
  for (let i = 0; i < cells.length; i += 4) {
    if (Math.abs(cells[i] - centerX) <= OCTOMAP_VIEW_RADIUS_M
      && Math.abs(cells[i + 1] - centerY) <= OCTOMAP_VIEW_RADIUS_M) selected.push(i)
  }
  if (!selected.length) return null
  // Aggregate occupied leaves into aligned OctoMap parents for display only.
  const displayed = new Map<string, number[]>()
  const minimumSize = map.resolution * scale
  for (const i of selected) {
    const size = Math.max(minimumSize, cells[i + 3])
    const center = [0, 1, 2].map(axis => (Math.floor(cells[i + axis] / size) + .5) * size)
    displayed.set(`${size}:${center.join(',')}`, [...center, size])
  }
  const material = new THREE.MeshBasicMaterial({ color: 0xffffff, toneMapped: false })
  material.onBeforeCompile = shader => {
    shader.vertexShader = `varying vec3 voxelPosition;\n${shader.vertexShader}`.replace(
      '#include <begin_vertex>', '#include <begin_vertex>\nvoxelPosition = position;',
    )
    shader.fragmentShader = `varying vec3 voxelPosition;\n${shader.fragmentShader}`.replace(
      '#include <color_fragment>', '#include <color_fragment>\nvec3 edge = step(vec3(0.475), abs(voxelPosition));\nif (edge.x + edge.y + edge.z > 1.5) diffuseColor.rgb *= 0.55;',
    )
  }
  const mesh = new THREE.InstancedMesh(new THREE.BoxGeometry(1, 1, 1), material, displayed.size)
  mesh.name = 'navigation-octomap'
  mesh.userData.nativeCells = cells
  mesh.userData.nativeIndices = selected
  const transform = new THREE.Matrix4(), color = new THREE.Color(), rgb = new Float32Array(3)
  Array.from(displayed.values()).forEach(([x, y, z, size], index) => {
    transform.makeScale(size, size, size).setPosition(x, z, -y)
    mesh.setMatrixAt(index, transform)
    if (colorMode === 'height') {
      writeCloudHeightColor(z, rgb, 0, bounds.min[2], bounds.max[2])
      color.setRGB(rgb[0], rgb[1], rgb[2])
    } else color.setHex(0xc3c6c9)
    mesh.setColorAt(index, color)
  })
  mesh.computeBoundingBox()
  mesh.computeBoundingSphere()
  return mesh
}

/** A display parent is larger than its observations; select only the native surface. */
export function pickOctomapSurface(raycaster: THREE.Raycaster, mesh: THREE.InstancedMesh): THREE.Vector3 | null {
  if (!mesh.visible) return null
  const cells = mesh.userData.nativeCells as Float32Array
  const indices = mesh.userData.nativeIndices as number[]
  const ray = raycaster.ray.clone().applyMatrix4(mesh.matrixWorld.clone().invert())
  const box = new THREE.Box3(), hit = new THREE.Vector3()
  const material = mesh.material as THREE.MeshBasicMaterial
  let closest: THREE.Vector3 | null = null, distance = raycaster.far
  for (const i of indices) {
    const half = cells[i + 3] / 2, x = cells[i], y = cells[i + 2], z = -cells[i + 1]
    box.min.set(x - half, y - half, z - half)
    box.max.set(x + half, y + half, z + half)
    if (!ray.intersectBox(box, hit)) continue
    hit.applyMatrix4(mesh.matrixWorld)
    const d = raycaster.ray.origin.distanceTo(hit)
    if (d < raycaster.near || d >= distance || material.clippingPlanes?.some(plane => plane.distanceToPoint(hit) < 0)) continue
    closest = hit.clone()
    distance = d
  }
  return closest
}

export function clipOctomapLayer(mesh: THREE.InstancedMesh, minZ?: number | null, maxZ?: number | null): void {
  const material = mesh.material as THREE.MeshBasicMaterial
  material.clippingPlanes = [
    ...(minZ != null ? [new THREE.Plane(new THREE.Vector3(0, 1, 0), -minZ)] : []),
    ...(maxZ != null ? [new THREE.Plane(new THREE.Vector3(0, -1, 0), maxZ)] : []),
  ]
  material.needsUpdate = true
}
