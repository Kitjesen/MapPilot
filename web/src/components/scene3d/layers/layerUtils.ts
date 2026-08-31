import * as THREE from 'three'
import { lingtuXYToThree, lingtuYawToThree } from '../../../services/coordinateFrame.ts'

export type GroupedMesh = THREE.Mesh & { _group?: THREE.Group }

export function disposeGroupedMesh(scene: THREE.Scene, mesh: GroupedMesh | null) {
  if (!mesh) return
  const parent = mesh._group ?? mesh
  scene.remove(parent)
  mesh.geometry.dispose()
  const material = mesh.material
  if (Array.isArray(material)) {
    material.forEach(m => m.dispose())
  } else {
    ;(material as THREE.MeshBasicMaterial).map?.dispose()
    material.dispose()
  }
}

export function createFlatGridGroup(
  mesh: GroupedMesh,
  origin: [number, number],
  sizeX: number,
  sizeY: number,
  yaw: number,
  zOffset: number,
) {
  const hx = sizeX / 2
  const hy = sizeY / 2
  const cosY = Math.cos(yaw)
  const sinY = Math.sin(yaw)
  const cx = origin[0] + cosY * hx - sinY * hy
  const cy = origin[1] + sinY * hx + cosY * hy

  mesh.rotation.x = -Math.PI / 2
  const group = new THREE.Group()
  group.rotation.y = lingtuYawToThree(yaw)
  const [tx, , tz] = lingtuXYToThree(cx, cy, zOffset)
  group.position.set(tx, zOffset, tz)
  group.add(mesh)
  mesh._group = group
  return group
}

export function createHeightGridGroup(
  mesh: GroupedMesh,
  origin: [number, number] | [number, number, number],
  sizeX: number,
  sizeY: number,
  yaw: number,
) {
  const hx = sizeX / 2
  const hy = sizeY / 2
  const cosY = Math.cos(yaw)
  const sinY = Math.sin(yaw)
  const cx = origin[0] + cosY * hx - sinY * hy
  const cy = origin[1] + sinY * hx + cosY * hy

  const group = new THREE.Group()
  group.rotation.y = lingtuYawToThree(yaw)
  const [tx, , tz] = lingtuXYToThree(cx, cy)
  group.position.set(tx, 0, tz)
  group.add(mesh)
  mesh._group = group
  return group
}
