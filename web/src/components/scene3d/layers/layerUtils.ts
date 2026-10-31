import * as THREE from 'three'
import { lingtuXYToThree, lingtuYawToThree } from '../../../services/coordinateFrame.ts'

export type GroupedMesh = THREE.Mesh & { _group?: THREE.Group }

export interface CloudHeightRange {
  minZ: number | null
  maxZ: number | null
}

export type CloudHeightStyle = 'clip' | 'fade'

interface CloudFadeMaterial {
  range: THREE.Vector4
  onBeforeCompile: THREE.PointsMaterial['onBeforeCompile']
  transparent: boolean
  depthWrite: boolean
  enabled: boolean
}

const cloudFadeMaterials = new WeakMap<THREE.PointsMaterial, CloudFadeMaterial>()
const CLOUD_FADE_DISTANCE_M = 0.6

function updateCloudHeightFade(material: THREE.PointsMaterial, enabled: boolean, minZ: number | null, maxZ: number | null): void {
  let fade = cloudFadeMaterials.get(material)
  if (enabled && !fade) {
    const previousProgramKey = material.customProgramCacheKey()
    fade = {
      range: new THREE.Vector4(),
      onBeforeCompile: material.onBeforeCompile,
      transparent: material.transparent,
      depthWrite: material.depthWrite,
      enabled: false,
    }
    const state = fade
    material.onBeforeCompile = (shader, renderer) => {
      state.onBeforeCompile.call(material, shader, renderer)
      shader.uniforms.cloudHeightRange = { value: state.range }
      shader.vertexShader = `varying float vCloudWorldZ;\n${shader.vertexShader}`.replace(
        '#include <project_vertex>',
        '#include <project_vertex>\nvCloudWorldZ = (modelMatrix * vec4(transformed, 1.0)).y;',
      )
      shader.fragmentShader = `varying float vCloudWorldZ;\nuniform vec4 cloudHeightRange;\n${shader.fragmentShader}`.replace(
        '#include <alphatest_fragment>',
        `float belowRange = max(cloudHeightRange.x - vCloudWorldZ, 0.0) * cloudHeightRange.z;
float aboveRange = max(vCloudWorldZ - cloudHeightRange.y, 0.0) * cloudHeightRange.w;
float outsideRange = max(belowRange, aboveRange);
if (outsideRange >= ${CLOUD_FADE_DISTANCE_M}) discard;
diffuseColor.a *= 1.0 - smoothstep(0.0, ${CLOUD_FADE_DISTANCE_M}, outsideRange);
#include <alphatest_fragment>`,
      )
    }
    material.customProgramCacheKey = () => `${previousProgramKey}:cloud-height-fade`
    material.needsUpdate = true
    cloudFadeMaterials.set(material, fade)
  }
  if (!fade) return
  // Keep one uniform object across cached clip/full/fade programs.
  fade.range.set(minZ ?? 0, maxZ ?? 0, Number(enabled && minZ != null), Number(enabled && maxZ != null))
  if (fade.enabled !== enabled) {
    material.transparent = enabled ? true : fade.transparent
    material.depthWrite = enabled ? false : fade.depthWrite
    fade.enabled = enabled
    material.needsUpdate = true
  }
}

export function interpolateCloudHeightRange(from: CloudHeightRange, to: CloudHeightRange, progress: number): CloudHeightRange {
  const t = Math.min(1, Math.max(0, progress))
  return {
    minZ: from.minZ == null || to.minZ == null ? to.minZ : from.minZ + (to.minZ - from.minZ) * t,
    maxZ: from.maxZ == null || to.maxZ == null ? to.maxZ : from.maxZ + (to.maxZ - from.maxZ) * t,
  }
}

/** Display-only world Z range; LingTu Z maps to Three world Y. */
export function updateCloudHeightRange(points: THREE.Points | null, minZ?: number | null, maxZ?: number | null, style: CloudHeightStyle = 'clip'): void {
  if (!points) return
  const material = points.material as THREE.PointsMaterial
  const hasMin = minZ != null && Number.isFinite(minZ)
  const hasMax = maxZ != null && Number.isFinite(maxZ)
  updateCloudHeightFade(material, style === 'fade' && (hasMin || hasMax), hasMin ? minZ : null, hasMax ? maxZ : null)
  const count = style === 'clip' ? Number(hasMin) + Number(hasMax) : 0
  if ((material.clippingPlanes?.length ?? 0) !== count) {
    material.clippingPlanes = count ? Array.from({ length: count }, () => new THREE.Plane()) : null
    material.needsUpdate = true
  }
  const planes = material.clippingPlanes
  if (!planes) return
  let index = 0
  if (hasMin) {
    planes[index].normal.set(0, 1, 0)
    planes[index++].constant = -minZ
  }
  if (hasMax) {
    planes[index].normal.set(0, -1, 0)
    planes[index].constant = maxZ
  }
}

export function cloudPointIsVisible(points: THREE.Points, worldPoint: THREE.Vector3): boolean {
  const material = points.material as THREE.PointsMaterial
  if (!points.visible || material.clippingPlanes?.some(plane => plane.distanceToPoint(worldPoint) < 0)) return false
  const fade = cloudFadeMaterials.get(material)
  if (!fade?.enabled) return true
  const outsideRange = Math.max(
    Math.max(fade.range.x - worldPoint.y, 0) * fade.range.z,
    Math.max(worldPoint.y - fade.range.y, 0) * fade.range.w,
  )
  return outsideRange < CLOUD_FADE_DISTANCE_M
}

/** Fit only rendered vertices, excluding clipped points and unused live-buffer capacity. */
export function visibleCloudBounds(points: THREE.Points): THREE.Box3 {
  const bounds = new THREE.Box3()
  if (!points.visible) return bounds
  points.updateWorldMatrix(true, false)
  const positions = points.geometry.getAttribute('position')
  const { start, count } = points.geometry.drawRange
  const end = Math.min(positions.count, start + count)
  const vertex = new THREE.Vector3()
  for (let index = start; index < end; index++) {
    vertex.fromBufferAttribute(positions, index).applyMatrix4(points.matrixWorld)
    if (cloudPointIsVisible(points, vertex)) bounds.expandByPoint(vertex)
  }
  return bounds
}

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
